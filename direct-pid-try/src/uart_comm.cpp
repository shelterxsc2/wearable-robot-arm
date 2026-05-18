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
    printf("[UART] %s opened @ %d baud (8N1, no flow ctrl)\n", device, baudrate);
    return 0;
}

void uart_cleanup(void)
{
    uart_stop_receiver();
    if (g_uart_fd >= 0) {
        tcflush(g_uart_fd, TCIOFLUSH);
        close(g_uart_fd);
        g_uart_fd = -1;
        printf("[UART] closed\n");
    }
}

int uart_send_raw(const uint8_t* data, size_t len)
{
    if (g_uart_fd < 0) return -1;
    printf("[UART] TX (%zu bytes):", len);
    for (size_t i = 0; i < len && i < 32; ++i) {
        printf(" %02X", data[i]);
    }
    if (len > 32) printf(" ...");
    printf("\n");
    ssize_t w = write(g_uart_fd, data, len);
    if ((size_t)w != len) {
        fprintf(stderr, "[UART] write failed: %zd/%zu (%s)\n", w, len, strerror(errno));
        return -1;
    }
    tcdrain(g_uart_fd);  /* 等待发送完成 */
    return 0;
}

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
    printf("[UART] sending heartbeat...\n");
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

int uart_send_arm_target(float x, float y, float z, float k1, float k2)
{
    printf("[UART] TX ARM_TARGET: x=%.1f y=%.1f z=%.1f k1=%.1f k2=%.1f\n", x, y, z, k1, k2);
    int16_t data[5];
    data[0] = (int16_t)x;
    data[1] = (int16_t)y;
    data[2] = (int16_t)z;
    data[3] = (int16_t)k1;
    data[4] = (int16_t)k2;
    return uart_send_raw((const uint8_t*)data, sizeof(data));
}

/* ---------- 接收线程 ---------- */
static void* recv_thread_func(void* arg)
{
    (void)arg;
    uint8_t rx_buf[64];

    printf("[UART] receiver thread started (binary mode)\n");

    while (g_recv_running.load()) {
        int n = uart_recv_raw(rx_buf, sizeof(rx_buf), 100);
        if (n <= 0) continue;

        printf("[UART] RX (%d bytes):", n);
        for (int i = 0; i < n && i < 16; ++i) {
            printf(" %02X", rx_buf[i]);
        }
        if (n > 16) printf(" ...");
        printf("\n");
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
