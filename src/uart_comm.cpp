/**
 * uart_comm.cpp - 串口通信驱动 (RK3588 UART9)
 * 协议: [0xAA][0x55][LEN][CMD][payload...][CRC8]
 */
#include "uart_comm.h"
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <pthread.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <atomic>
#include <time.h>
#include <stdarg.h>
#include <sys/time.h>

/* ---------- 协议常量 ---------- */
#define FRAME_HEAD0       0xAA
#define FRAME_HEAD1       0x55

#define CMD_HEARTBEAT     0x01  /* 心跳/握手 */
#define CMD_TARGET_POSE   0x10  /* RK3588 -> 下位机: 目标位姿 */
#define CMD_CURRENT_POSE  0x20  /* 下位机 -> RK3588: 当前位姿 */

/* ---------- 全局状态 ---------- */
static int g_uart_fd = -1;
static pthread_t g_recv_thread;
static std::atomic<int> g_recv_running{0};
static uart_pose_callback_t g_pose_cb = NULL;
/* All producers share one physical UART writer. */
static pthread_mutex_t g_uart_tx_mutex = PTHREAD_MUTEX_INITIALIZER;

/* 运动完成状态: 1=完成/空闲, 0=运动中 */
volatile int g_uart_move_complete = 0;

/* 下位机初始化成功: 1=已收到 "init success", 0=未收到 */
volatile int g_uart_init_success = 0;

/* 下位机归位完成: 1=已收到归位后的首次 "move_success", 0=未收到 */
volatile int g_uart_homing_done = 0;

/* 头部静止状态: 1=头部当前静止(wx/wz<3), 0=头部在动 */
volatile int g_head_stationary = 0;

/* 机械臂到位状态: 1=头部静止持续400ms且距离上次发令>200ms */
volatile int g_arm_stable = 0;

/* 握手延时期间禁止 UART 发送: 1=禁止, 0=允许 */
volatile int g_uart_block_tx = 0;

/* ---------- 诊断日志: 带运行时长, 同时写文件和终端 ---------- */
static FILE* g_diag_fp = NULL;
static pthread_mutex_t g_diag_mutex = PTHREAD_MUTEX_INITIALIZER;
static struct timespec g_prog_start;

/* ---------- [UART-RX] 原始数据记录到 /tmp/cmd ---------- */
static FILE* g_cmd_fp = NULL;
static pthread_mutex_t g_cmd_mutex = PTHREAD_MUTEX_INITIALIZER;

static void cmd_log_raw(const char* line) {
    if (!g_cmd_fp) {
        g_cmd_fp = fopen("/tmp/cmd", "w");
        if (g_cmd_fp) setbuf(g_cmd_fp, NULL);
    }
    if (!g_cmd_fp) return;

    pthread_mutex_lock(&g_cmd_mutex);
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm tm_info;
    localtime_r(&tv.tv_sec, &tm_info);
    fprintf(g_cmd_fp, "[%02d:%02d:%02d.%03d] %s\n",
            tm_info.tm_hour, tm_info.tm_min, tm_info.tm_sec,
            (int)(tv.tv_usec / 1000), line);
    fflush(g_cmd_fp);
    pthread_mutex_unlock(&g_cmd_mutex);
}

static void print_timestamp(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm tm_info;
    localtime_r(&tv.tv_sec, &tm_info);
    printf("[%02d:%02d:%02d.%03d] ", tm_info.tm_hour, tm_info.tm_min, tm_info.tm_sec,
           (int)(tv.tv_usec / 1000));
}

static void diag_init(void) {
    if (g_diag_fp) return;
    g_diag_fp = fopen("/tmp/uart_complete_diag.log", "w");
    if (g_diag_fp) setbuf(g_diag_fp, NULL);
    clock_gettime(CLOCK_MONOTONIC, &g_prog_start);
}

static double get_runtime_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec - g_prog_start.tv_sec) * 1000.0 +
           (ts.tv_nsec - g_prog_start.tv_nsec) / 1e6;
}

void diag_log(const char* fmt, ...) {
    if (!g_diag_fp) diag_init();
    pthread_mutex_lock(&g_diag_mutex);

    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm tm_info;
    localtime_r(&tv.tv_sec, &tm_info);
    char ts[32];
    snprintf(ts, sizeof(ts), "[%02d:%02d:%02d.%03d] ",
             tm_info.tm_hour, tm_info.tm_min, tm_info.tm_sec,
             (int)(tv.tv_usec / 1000));

    fprintf(g_diag_fp, "%s", ts);
    va_list args;
    va_start(args, fmt);
    vfprintf(g_diag_fp, fmt, args);
    va_end(args);
    fprintf(g_diag_fp, "\n");
    fflush(g_diag_fp);

    printf("%s", ts);
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    printf("\n");

    pthread_mutex_unlock(&g_diag_mutex);
}

/* ---------- 工具函数 ---------- */
static uint8_t crc8(const uint8_t* data, size_t len)
{
    uint8_t c = 0;
    for (size_t i = 0; i < len; ++i) c += data[i];
    return c;
}

static int set_baudrate(struct termios* tty, int baudrate)
{
    speed_t bd;
    switch (baudrate) {
        case 9600:    bd = B9600;    break;
        case 19200:   bd = B19200;   break;
        case 38400:   bd = B38400;   break;
        case 57600:   bd = B57600;   break;
        case 115200:  bd = B115200;  break;
        case 230400:  bd = B230400;  break;
        case 460800:  bd = B460800;  break;
        case 921600:  bd = B921600;  break;
        case 1500000: bd = B1500000; break;
        default:
            fprintf(stderr, "[UART] unsupported baudrate %d\n", baudrate);
            return -1;
    }
    cfsetospeed(tty, bd);
    cfsetispeed(tty, bd);
    return 0;
}

/* ---------- 接口实现 ---------- */
int uart_init(const char* device, int baudrate)
{
    g_uart_fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (g_uart_fd < 0) {
        fprintf(stderr, "[UART] open %s failed: %s\n", device, strerror(errno));
        return -1;
    }

    /* 防止是控制终端 */
    if (isatty(g_uart_fd)) {
        /* 正常 */
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(g_uart_fd, &tty) != 0) {
        fprintf(stderr, "[UART] tcgetattr failed: %s\n", strerror(errno));
        close(g_uart_fd);
        g_uart_fd = -1;
        return -1;
    }

    if (set_baudrate(&tty, baudrate) != 0) {
        close(g_uart_fd);
        g_uart_fd = -1;
        return -1;
    }

    /* 8N1, 无硬件流控, 无软件流控, RAW 模式 */
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR | IGNCR);
    tty.c_oflag &= ~OPOST;

    /* 非阻塞读取超时: VMIN=0, VTIME=1 -> 100ms */
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(g_uart_fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "[UART] tcsetattr failed: %s\n", strerror(errno));
        close(g_uart_fd);
        g_uart_fd = -1;
        return -1;
    }

    tcflush(g_uart_fd, TCIOFLUSH);
    /* printf("[UART] %s opened @ %d baud (8N1, no flow ctrl)\n", device, baudrate); */
    return 0;
}

void uart_cleanup(void)
{
    uart_stop_receiver();
    if (g_diag_fp) {
        fclose(g_diag_fp);
        g_diag_fp = NULL;
    }
    if (g_cmd_fp) {
        fclose(g_cmd_fp);
        g_cmd_fp = NULL;
    }
    if (g_uart_fd >= 0) {
        tcflush(g_uart_fd, TCIOFLUSH);
        close(g_uart_fd);
        g_uart_fd = -1;
        /* printf("[UART] closed\n"); */
    }
}

int uart_send_raw(const uint8_t* data, size_t len)
{
    if (!data || len == 0) return -1;
    pthread_mutex_lock(&g_uart_tx_mutex);
    if (g_uart_fd < 0 || g_uart_block_tx) {
        pthread_mutex_unlock(&g_uart_tx_mutex);
        return -1;
    }

    /* printf("[UART-TX] (%zu bytes):", len);
    for (size_t i = 0; i < len && i < 32; ++i) {
        printf(" %02X", data[i]);
    }
    if (len > 32) printf(" ...");
    printf("\n"); */
    ssize_t w = write(g_uart_fd, data, len);
    if ((size_t)w != len) {
        fprintf(stderr, "[UART] write failed: %zd/%zu (%s)\n", w, len, strerror(errno));
        pthread_mutex_unlock(&g_uart_tx_mutex);
        return -1;
    }
    tcdrain(g_uart_fd);  /* 等待发送完成 */
    pthread_mutex_unlock(&g_uart_tx_mutex);
    return 0;
}

static int uart_send_power_pattern(uint8_t first)
{
    uint8_t frame[10];
    uint8_t second = first == 0xFF ? 0xAA : 0xFF;
    for (int i = 0; i < 10; ++i) frame[i] = (i % 2 == 0) ? first : second;
    pthread_mutex_lock(&g_uart_tx_mutex);
    if (g_uart_fd < 0) {
        pthread_mutex_unlock(&g_uart_tx_mutex);
        return -1;
    }
    ssize_t written = write(g_uart_fd, frame, sizeof(frame));
    if (written == (ssize_t)sizeof(frame)) tcdrain(g_uart_fd);
    pthread_mutex_unlock(&g_uart_tx_mutex);
    return written == (ssize_t)sizeof(frame) ? 0 : -1;
}

int uart_send_power_on(void) { return uart_send_power_pattern(0xFF); }
int uart_send_power_off(void) { return uart_send_power_pattern(0xAA); }

int uart_recv_raw(uint8_t* buf, size_t max_len, int timeout_ms)
{
    if (g_uart_fd < 0) return -1;
    struct pollfd pfd = { g_uart_fd, POLLIN, 0 };
    int ret = poll(&pfd, 1, timeout_ms);
    if (ret < 0) {
        if (errno != EINTR) fprintf(stderr, "[UART] poll error: %s\n", strerror(errno));
        return -1;
    }
    if (ret == 0) return 0;  /* 超时 */
    ssize_t r = read(g_uart_fd, buf, max_len);
    if (r < 0) {
        if (errno != EAGAIN && errno != EINTR)
            fprintf(stderr, "[UART] read error: %s\n", strerror(errno));
        return -1;
    }
    return (int)r;
}

/* 打包发送一帧 */
static int send_frame(uint8_t cmd, const uint8_t* payload, uint8_t len)
{
    uint8_t buf[256];
    if (len > 250) {
        fprintf(stderr, "[UART] payload too long: %d\n", len);
        return -1;
    }
    buf[0] = FRAME_HEAD0;
    buf[1] = FRAME_HEAD1;
    buf[2] = len;
    buf[3] = cmd;
    if (len > 0) memcpy(&buf[4], payload, len);
    buf[4 + len] = crc8(&buf[2], len + 2);  /* CRC = LEN + CMD + payload */
    return uart_send_raw(buf, 5 + len);
}

int uart_send_heartbeat(void)
{
    /* printf("[UART] sending heartbeat...\n"); */
    return send_frame(CMD_HEARTBEAT, NULL, 0);
}

int uart_send_target_pose(const Pose6D* pose)
{
    if (!pose) return -1;
    uint8_t p[28];
    memcpy(&p[0],  &pose->x,  4);
    memcpy(&p[4],  &pose->y,  4);
    memcpy(&p[8],  &pose->z,  4);
    memcpy(&p[12], &pose->qx, 4);
    memcpy(&p[16], &pose->qy, 4);
    memcpy(&p[20], &pose->qz, 4);
    memcpy(&p[24], &pose->qw, 4);
    return send_frame(CMD_TARGET_POSE, p, 28);
}

int uart_send_arm_target(float x, float y, float z, float k1, float k2, uint8_t flag)
{
    // 握手延时期间禁止发送
    if (g_uart_block_tx) {
        return -1;
    }

    diag_log("[UART-TX] x=%.1f y=%.1f z=%.1f k1=%.1f k2=%.1f flag=0x%02X", x, y, z, k1, k2, flag);
    int16_t data[5];
    data[0] = (int16_t)x;
    data[1] = (int16_t)y;
    data[2] = (int16_t)z;
    data[3] = (int16_t)k1;
    data[4] = (int16_t)k2;

    uint8_t frame[11];
    memcpy(frame, data, 10);
    frame[10] = flag;

    int ret = uart_send_raw(frame, sizeof(frame));
    if (ret == 0) uart_set_move_pending();
    return ret;
}

void uart_set_move_pending(void)
{
    g_uart_move_complete = 0;
}

/* ---------- 接收线程 ---------- */
static void* recv_thread_func(void* arg)
{
    (void)arg;
    uint8_t rx_buf[64];

    /* printf("[UART] receiver thread started (binary mode)\n"); */

    static char rx_text[512];
    static int  rx_text_len = 0;

    while (g_recv_running.load()) {
        int n = uart_recv_raw(rx_buf, sizeof(rx_buf), 100);
        if (n <= 0) continue;

        /* --- Step 1 诊断：打印原始 RX 字节流 --- */
        char rx_line[256];
        int pos = 0;
        /* print_timestamp(); */
        pos += snprintf(rx_line + pos, sizeof(rx_line) - pos, "[UART-RX] n=%d | ", n);
        for (int i = 0; i < n && i < 64; ++i) {
            if (rx_buf[i] >= 32 && rx_buf[i] <= 126)
                pos += snprintf(rx_line + pos, sizeof(rx_line) - pos, "%c", rx_buf[i]);
            else
                pos += snprintf(rx_line + pos, sizeof(rx_line) - pos, "\\x%02X", rx_buf[i]);
        }
        cmd_log_raw(rx_line);

        /* print_timestamp();
        printf("[UART] RX (%d bytes):", n);
        for (int i = 0; i < n && i < 16; ++i) {
            printf(" %02X", rx_buf[i]);
        }
        if (n > 16) printf(" ...");
        printf("\n"); */

        /* 简单文本缓冲: 检测 "move complete" */
        int copy = n;
        if (rx_text_len + copy > (int)sizeof(rx_text) - 1)
            copy = (int)sizeof(rx_text) - 1 - rx_text_len;
        for (int i = 0; i < copy; ++i)
            rx_text[rx_text_len++] = (char)rx_buf[i];
        rx_text[rx_text_len] = '\0';

        if (strstr(rx_text, "init success") != NULL) {
            g_uart_init_success = 1;
            rx_text_len = 0;
            rx_text[0] = '\0';
        } else if (strstr(rx_text, "move_success") != NULL) {
            if (!g_uart_homing_done) {
                g_uart_homing_done = 1;
            }
            g_uart_move_complete = 1;
            rx_text_len = 0;
            rx_text[0] = '\0';
        }
        /* 防垃圾堆积 */
        if (rx_text_len > 400) {
            rx_text_len = 0;
            rx_text[0] = '\0';
        }
    }

    printf("[UART] receiver thread stopped\n");
    return NULL;
}

int uart_start_receiver(uart_pose_callback_t cb)
{
    if (g_recv_running.load()) return 0;
    if (g_uart_fd < 0) {
        fprintf(stderr, "[UART] cannot start receiver: not initialized\n");
        return -1;
    }
    g_pose_cb = cb;
    g_recv_running.store(1);
    if (pthread_create(&g_recv_thread, NULL, recv_thread_func, NULL) != 0) {
        g_recv_running.store(0);
        fprintf(stderr, "[UART] pthread_create failed\n");
        return -1;
    }
    return 0;
}

void uart_stop_receiver(void)
{
    if (!g_recv_running.load()) return;
    g_recv_running.store(0);
    pthread_join(g_recv_thread, NULL);
}
