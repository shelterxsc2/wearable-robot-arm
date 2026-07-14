/**
 * ELF2 (RK3588) YOLOv8-Pose 完整版 (DHCP修复版 + RTMP/RTSP自适应 + 端侧控制)
 * 流程: WiFi -> 探测云服务器 -> RTMP/RTSP -> RGA -> NPU -> 推流 + 控制
 */
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <gst/gst.h>
#include <pthread.h>
#include <ctime>
#include <cerrno>
#include <sys/prctl.h>

#include "wifi.h"
#include "rga_npu.h"
#include "stream_manager.h"
#include "ctrl_server.h"
#include "ws_client.h"
#include "cloud_command.h"
#include "voice_control.h"
#include "arm_power_control.h"
#include "cloud_report.h"
#include "bluetooth_spp.h"
#include "uart_comm.h"
#include "nrf24_linux.h"
#include "imu2_i2c.h"
#include "gst_rtmp.h"
#include <sys/wait.h>

static pid_t g_rule_engine_pid = -1;
static pid_t g_hand_pipeline_pid = -1;
static pid_t g_voice_kws_pid = -1;
static const char *RULE_SOCKET_PATH = "/tmp/rule_engine.sock";
static const char *HAND_SOCKET_PATH = "/tmp/hand_pipeline.sock";
static const char *VOICE_SOCKET_PATH = "/tmp/voice_kws.sock";

/* ========== 云服务器配置（来自历史备份） ========== */
#define DEVICE_ID           "device-003"
#define CLOUD_IP            "47.93.162.124"
#define RTMP_URL            "rtmp://47.93.162.124:1935/live/device-003"
#define WS_URL              "ws://47.93.162.124/ws?deviceId=device-003"

#define VIDEO_DEVICE        "/dev/video21"  // USB Camera3 (Realtek)
#define WIFI_IFNAME         "wlan0"
#define WIFI_SSID           "iQOO 12"
#define WIFI_PASSWD         "070103xsc"
#define CPU_FREQ_TARGET     1608000
#define CTRL_SERVER_PORT    8080

static volatile sig_atomic_t g_running = 1;
static pthread_t g_nrf24_tid = 0;
static int g_nrf24_enabled = 0;
static pthread_t g_imu2_tid = 0;
static int g_imu2_enabled = 0;
static volatile sig_atomic_t g_should_quit = 0;
static volatile sig_atomic_t g_ws_ready = 0;
static pthread_t g_handshake_tid = 0;
static int g_handshake_started = 0;

struct RuntimeState {
    bool rule_started = false;
    bool hand_started = false;
    bool voice_started = false;
    bool voice_control_started = false;
    bool arm_power_initialized = false;
    bool npu_initialized = false;
    bool rga_initialized = false;
    bool bluetooth_initialized = false;
    bool bluetooth_started = false;
    bool uart_initialized = false;
    bool ctrl_started = false;
    bool ws_started = false;
    pthread_t ws_tid = 0;
};

static pid_t start_sidecar(const char *name, char *const argv[])
{
    pid_t pid = fork();
    if (pid < 0) {
        fprintf(stderr, "[Main] Failed to fork %s: %s\n", name, strerror(errno));
        return -1;
    }
    if (pid == 0) {
        // Do not leave an NPU-owning orphan if the parent is killed abruptly.
        prctl(PR_SET_PDEATHSIG, SIGTERM);
        if (getppid() == 1) _exit(1);
        execvp(argv[0], argv);
        fprintf(stderr, "[Main] Failed to exec %s: %s\n", name, strerror(errno));
        _exit(127);
    }
    printf("[Main] %s process started PID=%d\n", name, pid);
    return pid;
}

static bool child_is_alive(pid_t pid)
{
    if (pid <= 0) return false;
    int status = 0;
    pid_t ret = waitpid(pid, &status, WNOHANG);
    return ret == 0;
}

static bool wait_for_sidecars(int timeout_ms)
{
    int waited_ms = 0;
    while (!g_should_quit && waited_ms < timeout_ms) {
        bool rule_ready = access(RULE_SOCKET_PATH, F_OK) == 0;
        bool hand_ready = access(HAND_SOCKET_PATH, F_OK) == 0;
        if (rule_ready && hand_ready) return true;
        if (!child_is_alive(g_rule_engine_pid) ||
            !child_is_alive(g_hand_pipeline_pid)) {
            fprintf(stderr, "[Main] A sidecar exited during startup\n");
            return false;
        }
        usleep(100000);
        waited_ms += 100;
    }
    fprintf(stderr, "[Main] Sidecar readiness timeout after %d ms\n", timeout_ms);
    return false;
}

static void stop_child(pid_t *pid, const char *name, const char *socket_path)
{
    if (!pid || *pid <= 0) {
        if (socket_path) unlink(socket_path);
        return;
    }

    int status = 0;
    pid_t ret = waitpid(*pid, &status, WNOHANG);
    if (ret == 0) {
        kill(*pid, SIGTERM);
        for (int i = 0; i < 30; ++i) {
            ret = waitpid(*pid, &status, WNOHANG);
            if (ret == *pid || (ret < 0 && errno == ECHILD)) break;
            usleep(100000);
        }
        if (ret == 0) {
            fprintf(stderr, "[Main] %s did not stop after 3s; sending SIGKILL\n", name);
            kill(*pid, SIGKILL);
            while (waitpid(*pid, &status, 0) < 0 && errno == EINTR) {}
        }
    }
    printf("[Main] %s stopped\n", name);
    *pid = -1;
    if (socket_path) unlink(socket_path);
}

/* 上位机-下位机握手状态: 0=wait_init, 1=a_init, 2=send_ff, 3=wait_homing, 4=normal */
volatile int g_host_state = 0;

// 设置CPU性能模式 (RK3588 big.LITTLE: policy0 + policy4)
static void set_cpu_governor(const char* policy, int freq) {
    char path[128];
    snprintf(path, sizeof(path), "/sys/devices/system/cpu/cpufreq/%s/scaling_governor", policy);
    int fd = open(path, O_WRONLY);
    if (fd >= 0) { write(fd, "userspace", 9); close(fd); usleep(10000); }
    snprintf(path, sizeof(path), "/sys/devices/system/cpu/cpufreq/%s/scaling_setspeed", policy);
    fd = open(path, O_WRONLY);
    if (fd >= 0) {
        char buf[32];
        snprintf(buf, sizeof(buf), "%d", freq);
        write(fd, buf, strlen(buf));
        close(fd);
    }
}

static void set_cpu_performance(void) {
    printf("[CPU] Setting performance mode...\n");
    set_cpu_governor("policy0", CPU_FREQ_TARGET);  // A55 簇
    set_cpu_governor("policy4", CPU_FREQ_TARGET);  // A76 簇
    system("echo performance | tee $(find /sys/ -name *governor 2>/dev/null) > /dev/null 2>&1");

    FILE *fp = fopen("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq", "r");
    if (fp) {
        int freq = 0;
        fscanf(fp, "%d", &freq);
        fclose(fp);
        printf("[CPU] Current frequency: %d MHz\n", freq / 1000);
    }
}

// DHCP失败后的静态IP配置
static void setup_static_ip_fallback(void) {
    printf("⚠️ DHCP failed, setting static IP...\n");

    printf("📝 Trying 192.168.1.x subnet...\n");
    system("ifconfig wlan0 192.168.1.100 netmask 255.255.255.0 up");
    system("route add default gw 192.168.1.1 2>/dev/null || true");
    system("echo 'nameserver 114.114.114.114' > /etc/resolv.conf");

    sleep(1);
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, "wlan0", IFNAMSIZ - 1);

    if (ioctl(sock, SIOCGIFADDR, &ifr) >= 0) {
        struct sockaddr_in *addr = (struct sockaddr_in *)&ifr.ifr_addr;
        char *ip = inet_ntoa(addr->sin_addr);
        if (strcmp(ip, "0.0.0.0") != 0 && strlen(ip) > 7) {
            printf("✅ Static IP configured: %s\n", ip);
            close(sock);
            return;
        }
    }
    close(sock);

    printf("📝 Trying 192.168.0.x subnet...\n");
    system("ifconfig wlan0 192.168.0.100 netmask 255.255.255.0 up");
    system("route add default gw 192.168.0.1 2>/dev/null || true");

    printf("✅ Static IP fallback complete (192.168.0.100)\n");
}

static void sigint_handler(int sig) {
    (void)sig;
    g_should_quit = 1;
    g_running = 0;
}

static void monitor_sidecar(pid_t *pid, const char *name)
{
    if (!pid || *pid <= 0 || g_should_quit) return;
    int status = 0;
    pid_t ret = waitpid(*pid, &status, WNOHANG);
    if (ret == *pid) {
        fprintf(stderr, "[Main] %s exited unexpectedly status=%d; stopping system\n",
                name, status);
        *pid = -1;
        g_should_quit = 1;
        g_running = 0;
    }
}

static void monitor_optional_sidecar(pid_t *pid, const char *name)
{
    if (!pid || *pid <= 0 || g_should_quit) return;
    int status = 0;
    pid_t ret = waitpid(*pid, &status, WNOHANG);
    if (ret == *pid) {
        fprintf(stderr, "[Main] Optional %s exited status=%d; continuing without it\n",
                name, status);
        *pid = -1;
    }
}

/* 在主循环线程中安全地检查并退出 */
static gboolean check_quit_timer(gpointer user_data) {
    (void)user_data;
    monitor_sidecar(&g_rule_engine_pid, "RuleEngine");
    monitor_sidecar(&g_hand_pipeline_pid, "HandPipeline");
    monitor_optional_sidecar(&g_voice_kws_pid, "VoiceKWS");
    return g_should_quit ? G_SOURCE_REMOVE : G_SOURCE_CONTINUE;
}

/* ========== WebSocket 云端交互线程（来自历史备份） ========== */
static void *ws_worker_thread(void *arg) {
    (void)arg;
    srand((unsigned)time(NULL));

    int sock = -1;
    double reconnect_delay = 2.0;
    time_t start_time = time(NULL);
    int registered = 0;
    char cloud_message[4096];

    while (g_running) {
        if (sock < 0) {
            sock = ws_connect_url(WS_URL);
            if (sock < 0) {
                double wait = reconnect_delay;
                while (wait > 0.0 && g_running) {
                    double step = wait > 0.5 ? 0.5 : wait;
                    usleep((useconds_t)(step * 1000000.0));
                    wait -= step;
                }
                reconnect_delay = reconnect_delay * 1.5;
                if (reconnect_delay > 30.0) reconnect_delay = 30.0;
                continue;
            }
            printf("[WS] Connected to cloud: %s\n", WS_URL);
            reconnect_delay = 2.0;
            registered = 0;

            /* 设置非阻塞，用于接收服务器控制消息 */
            int flags = fcntl(sock, F_GETFL, 0);
            fcntl(sock, F_SETFL, flags | O_NONBLOCK);

            /* 第一步：发送注册帧（纯 frame_ts，无 data，必须在 500ms 内） */
            if (ws_send_text(sock, "{\"type\":\"frame_ts\"}") < 0) {
                printf("[WS] Registration send failed\n");
                ws_close(sock);
                sock = -1;
                continue;
            }
            printf("[WS] Registration frame_ts sent\n");

            /* 给服务器 200ms 处理注册 */
            usleep(200000);
            registered = 1;
            g_ws_ready = 1;
        }

        /* 每 100ms 循环一次 */
        usleep(100000);
        if (!g_running) break;

        /* 非阻塞接收服务器消息（控制指令），并检测连接是否存活 */
        int n = ws_recv_text(sock, cloud_message, sizeof(cloud_message));
        if (n < 0) {
            printf("[WS] Server closed connection\n");
            ws_close(sock);
            sock = -1;
            registered = 0;
            continue;
        } else if (n > 0) {
            int handled = control_handle_cloud_json(cloud_message);
            printf("[WS] Cloud command %s: %s\n",
                   handled > 0 ? "handled" : (handled < 0 ? "invalid" : "ignored"),
                   cloud_message);
        }

        /* 发送心跳（带 data） */
        if (registered) {
            char report[1024];
            while (cloud_report_take(report, sizeof(report)) > 0) {
                if (ws_send_text(sock, report) < 0) {
                    printf("[WS] Control report send failed, reconnecting...\n");
                    ws_close(sock); sock = -1; registered = 0;
                    break;
                }
            }
            if (!registered) continue;
            guint64 fc = get_rtmp_frame_count();

            time_t now = time(NULL);
            double elapsed = difftime(now, start_time);

            char msg[512];
            snprintf(msg, sizeof(msg),
                "{\"type\":\"frame_ts\",\"data\":{\"timestamp\":%ld,\"elapsed\":%.3f,\"frame_count\":%llu,\"device\":\"%s\"}}",
                (long)now, elapsed, (unsigned long long)fc, DEVICE_ID);

            if (ws_send_text(sock, msg) < 0) {
                printf("[WS] Heartbeat send failed, reconnecting...\n");
                ws_close(sock);
                sock = -1;
                registered = 0;
            }
        }
    }

    ws_close(sock);
    return NULL;
}

/* 握手线程信号：置1后 RX 线程收集 5 帧 IMU 算平均，再构造 R_init */
volatile int g_wait_a_init = 0;

/* ========== 上位机-下位机握手线程（阻塞式）==========
 * 1. 阻塞等待 "init success"
 * 2. 发送 FF 验证帧
 * 3. 阻塞等待 move_complete（下位机归位完成）
 * 4. 等 5 帧 IMU 算平均做 A-init（g_wait_a_init = 1）
 * 5. 进入 NORMAL，开始 UART-Tx
 */
static void* handshake_thread(void* arg) {
    (void)arg;

    // 1. 阻塞等待 init success
    printf("[Handshake] Waiting for init success...\n");
    while (g_running && !g_uart_init_success) {
        usleep(10000);  // 10ms
    }
    if (!g_running) return NULL;
    printf("[Handshake] Init success received.\n");

    // 2. 发 FF 验证帧
    uint8_t ff_frame[10] = {0xFF, 0xAA, 0xFF, 0xAA, 0xFF, 0xAA, 0xFF, 0xAA, 0xFF, 0xAA};
    uart_send_raw(ff_frame, 10);
    printf("[Handshake] FF verification frame sent.\n");

    // 3. 等待下位机归位完成（7s 延时，期间禁止UART发送）
    printf("[Handshake] Waiting 7s for homing...\n");
    extern volatile int g_uart_block_tx;
    g_uart_block_tx = 1;
    for (int i = 0; i < 70 && g_running; ++i) usleep(100000);
    g_uart_block_tx = 0;
    if (!g_running) return NULL;
    printf("[Handshake] 7s homing wait done.\n");

    // 4. A-init：等 RX 线程收集 5 帧 IMU 算平均
    g_wait_a_init = 1;
    printf("[Handshake] Waiting for 5 IMU frames avg for A-init...\n");
    while (g_running && !g_r_init_set) {
        usleep(10000);
    }
    if (!g_running) return NULL;
    printf("[Handshake] A-init (R_init) captured.\n");

    // 5. 进入 NORMAL，开始允许 UART-Tx
    g_host_state = 4;
    printf("[Handshake] Entering NORMAL.\n");

    return NULL;
}

static void shutdown_runtime(RuntimeState *state, bool send_exit_frame)
{
    if (!state) return;
    printf("\n[Main] Cleaning up...\n");
    g_should_quit = 1;
    g_running = 0;

    /* Release camera/encoder before tearing down NPU/RGA dependencies. */
    stream_manager_stop();
    printf("[Main] Stream manager stopped\n");

    if (state->ws_started) {
        pthread_join(state->ws_tid, NULL);
        state->ws_started = false;
        printf("[Main] WebSocket reporter stopped\n");
    }
    if (state->arm_power_initialized) {
        arm_power_shutdown();
        state->arm_power_initialized = false;
    }

    if (state->ctrl_started) {
        ctrl_server_stop();
        state->ctrl_started = false;
    }
    if (state->bluetooth_initialized) {
        bluetooth_spp_cleanup();
        state->bluetooth_started = false;
        state->bluetooth_initialized = false;
    }
    if (state->voice_control_started) {
        voice_control_stop();
        state->voice_control_started = false;
        printf("[Main] Voice control worker stopped\n");
    }
    if (state->npu_initialized) {
        cleanup_npu();
        state->npu_initialized = false;
    }
    if (g_nrf24_enabled) {
        nrf24_rx_thread_stop();
        pthread_join(g_nrf24_tid, NULL);
        g_nrf24_tid = 0;
        g_nrf24_enabled = 0;
        nrf24_linux_deinit();
        printf("[Main] NRF24 stopped\n");
    }
    if (g_imu2_enabled) {
        imu2_i2c_thread_stop();
        g_imu2_tid = 0;
        g_imu2_enabled = 0;
        imu2_i2c_deinit();
        printf("[Main] IMU2 I2C stopped\n");
    }

    if (state->uart_initialized) {
        (void)send_exit_frame;
        uart_cleanup();
        state->uart_initialized = false;
    }
    if (state->rga_initialized) {
        cleanup_rga();
        state->rga_initialized = false;
    }

    stop_child(&g_hand_pipeline_pid, "HandPipeline", HAND_SOCKET_PATH);
    stop_child(&g_rule_engine_pid, "RuleEngine", RULE_SOCKET_PATH);
    stop_child(&g_voice_kws_pid, "VoiceKWS", VOICE_SOCKET_PATH);
    state->hand_started = false;
    state->rule_started = false;
    state->voice_started = false;
    printf("[Main] Shutdown complete\n");
}

int main(int argc, char *argv[]) {
    // 取消 stdout 缓冲，确保日志实时落盘
    setbuf(stdout, NULL);
    setbuf(stderr, NULL);

    // Install handlers before any initialization that may block or spawn.
    struct sigaction sa{};
    sa.sa_handler = sigint_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    RuntimeState state;
    int exit_code = 0;

    /* 临时标定模式不跨进程保留，避免上次 mode=3 导致重启后行为异常。 */
    remove("/tmp/calib_mode.txt");

    printf("============================================================\n");
    printf("ELF2 (RK3588) YOLOv8-Pose AI推流系统 (RTMP/RTSP自适应版)\n");
    printf("============================================================\n\n");

    // 1. CPU调频
    set_cpu_performance();

    // 2. 初始化WiFi
    printf("[Main] Initializing WiFi...\n");
    if (wifi_enable_interface(WIFI_IFNAME) != 0) {
        fprintf(stderr, "[Main] Failed to enable WiFi, continuing...\n");
    } else if (wifi_connect_wpa(WIFI_IFNAME, WIFI_SSID, WIFI_PASSWD) != 0) {
        fprintf(stderr, "[Main] Failed to connect WiFi, continuing...\n");
    }

    if (wifi_dhcp(WIFI_IFNAME) != 0) {
        setup_static_ip_fallback();
    }

    printf("[Main] Cleaning up duplicate IPs if any...\n");
    system("ip addr flush dev wlan0 label wlan0:0 2>/dev/null || true");

    char ip_str[64] = "";
    wifi_print_ip(WIFI_IFNAME, ip_str, sizeof(ip_str));

    if (strlen(ip_str) == 0) {
        printf("⚠️ No IP obtained, forcing static IP...\n");
        setup_static_ip_fallback();
        wifi_print_ip(WIFI_IFNAME, ip_str, sizeof(ip_str));
    }

    if (g_should_quit) {
        shutdown_runtime(&state, false);
        return 130;
    }

    // 3. Start both NPU sidecars in parallel and wait for actual sockets,
    // instead of relying on fixed sleeps.
    unlink(RULE_SOCKET_PATH);
    unlink(HAND_SOCKET_PATH);
    printf("\n[Main] Starting sidecar services...\n");
    char *rule_argv[] = {
        const_cast<char *>("python3"),
        const_cast<char *>("scripts/rule_engine_server.py"), nullptr
    };
    char *hand_argv[] = {
        const_cast<char *>("python3"),
        const_cast<char *>("scripts/hand_pipeline_server.py"),
        const_cast<char *>("--mode"), const_cast<char *>("roi_gesture"),
        const_cast<char *>("--quiet"), nullptr
    };
    g_rule_engine_pid = start_sidecar("RuleEngine", rule_argv);
    state.rule_started = g_rule_engine_pid > 0;
    g_hand_pipeline_pid = start_sidecar("HandPipeline", hand_argv);
    state.hand_started = g_hand_pipeline_pid > 0;
    if (!state.rule_started || !state.hand_started || !wait_for_sidecars(20000)) {
        fprintf(stderr, "[Main] Sidecar startup failed; aborting initialization\n");
        shutdown_runtime(&state, false);
        return g_should_quit ? 130 : 1;
    }
    printf("[Main] All sidecars ready\n");

    // Voice KWS is deliberately optional: microphone/model failures must not
    // take down vision or mechanical control. The worker reconnects whenever
    // the sidecar socket becomes available.
    unlink(VOICE_SOCKET_PATH);
    char *voice_argv[] = {
        const_cast<char *>("taskset"), const_cast<char *>("-c"),
        const_cast<char *>("2"), const_cast<char *>("nice"),
        const_cast<char *>("-n"), const_cast<char *>("10"),
        const_cast<char *>("python3"),
        const_cast<char *>("scripts/voice_kws_server.py"),
        const_cast<char *>("--config"),
        const_cast<char *>("config/voice_kws.json"), nullptr
    };
    bool voice_disabled = getenv("VOICE_KWS_DISABLE") != nullptr;
    if (!voice_disabled) {
        g_voice_kws_pid = start_sidecar("VoiceKWS", voice_argv);
        state.voice_started = g_voice_kws_pid > 0;
        if (voice_control_start(VOICE_SOCKET_PATH) == 0) {
            state.voice_control_started = true;
            printf("[Main] Voice control worker started (optional sidecar)\n");
        } else {
            fprintf(stderr, "[Main] Voice control worker failed to start; continuing\n");
        }
    } else {
        printf("[Main] VoiceKWS disabled by VOICE_KWS_DISABLE (benchmark baseline)\n");
    }

    // 4. 初始化NPU
    printf("\n[Main] Initializing NPU...\n");
    if (init_npu() != 0) {
        fprintf(stderr, "[Main] NPU initialization failed\n");
        shutdown_runtime(&state, false);
        return g_should_quit ? 130 : 1;
    }
    state.npu_initialized = true;
    if (g_should_quit) {
        shutdown_runtime(&state, false);
        return 130;
    }

    // 4. 初始化RGA
    printf("\n[Main] Initializing RGA...\n");
    if (init_rga() != 0) {
        fprintf(stderr, "[Main] RGA initialization failed\n");
        shutdown_runtime(&state, false);
        return g_should_quit ? 130 : 1;
    }
    state.rga_initialized = true;
    if (g_should_quit) {
        shutdown_runtime(&state, false);
        return 130;
    }

    // 5. 蓝牙 BLE 遥控器
    printf("\n[Main] Initializing Bluetooth remote...\n");
    if (bluetooth_spp_init() == 0) {
        state.bluetooth_initialized = true;
        if (bluetooth_spp_start() == 0) {
            state.bluetooth_started = true;
            printf("[Main] Bluetooth remote client started\n");
        } else {
            fprintf(stderr, "[Main] Bluetooth remote start failed, continuing without it\n");
        }
    } else {
        fprintf(stderr, "[Main] Bluetooth remote init failed, continuing without it\n");
    }
    if (g_should_quit) {
        shutdown_runtime(&state, false);
        return 130;
    }

    // 6. 初始化 UART
    printf("\n[Main] Initializing UART...\n");
    if (uart_init("/dev/ttyS9", 115200) != 0) {
        fprintf(stderr, "[Main] UART init failed, continuing without motor control\n");
    } else {
        state.uart_initialized = true;
        if (uart_start_receiver(NULL) == 0) {
            printf("[Main] UART ready @ /dev/ttyS9 115200\n");
            arm_power_init();
            state.arm_power_initialized = true;
            printf("[Main] Arm remains retracted until an explicit power-on command\n");
        } else {
            fprintf(stderr, "[Main] UART receiver start failed\n");
        }
    }

    // 7. 启动端侧控制服务器
    printf("\n[Main] Starting control server on port %d...\n", CTRL_SERVER_PORT);
    if (ctrl_server_start(CTRL_SERVER_PORT) != 0) {
        fprintf(stderr, "[Main] Control server start failed, continuing without it\n");
    } else state.ctrl_started = true;

    // 8. 启动NRF24接收线程
    printf("\n[Main] Mode: FACE (default)\n");
    if (nrf24_linux_init() == 0) {
        if (nrf24_rx_thread_start(&g_nrf24_tid) == 0) {
            g_nrf24_enabled = 1;
            printf("[Main] NRF24 RX thread started\n");
        } else {
            nrf24_linux_deinit();
            printf("[Main] NRF24 RX thread start failed, continuing without it\n");
        }
    } else {
        printf("[Main] NRF24 init failed, continuing without it\n");
    }

    // 8.5 启动板载IMU2 I2C采样线程 (I2C4, 100 Hz)
    if (imu2_i2c_init() == 0) {
        if (imu2_i2c_thread_start(&g_imu2_tid) == 0) {
            g_imu2_enabled = 1;
            printf("[Main] IMU2 I2C thread started (100 Hz)\n");
        } else {
            imu2_i2c_deinit();
            printf("[Main] IMU2 I2C thread start failed, continuing without it\n");
        }
    } else {
        printf("[Main] IMU2 I2C init failed, continuing without it\n");
    }

    // 9. 探测云服务器并选择推流模式
    gst_init(&argc, &argv);

    StreamType stream_type = STREAM_TYPE_RTSP;

    printf("\n[Main] Probing RTMP server (%s)...\n", RTMP_URL);
    if (!g_should_quit && probe_rtmp_server(RTMP_URL, 3000)) {
        stream_type = STREAM_TYPE_RTMP;
        printf("[Main] RTMP server is UP. Will push RTMP + WebSocket.\n");
    } else {
        stream_type = STREAM_TYPE_RTSP;
        printf("[Main] RTMP server is DOWN. Will fallback to local RTSP.\n");
    }

    /* Keep the cloud control session alive so REST can later switch local -> cloud. */
    if (pthread_create(&state.ws_tid, NULL, ws_worker_thread, NULL) == 0) {
        state.ws_started = true;
        printf("[Main] WebSocket reporter started -> %s\n", WS_URL);
    } else {
        fprintf(stderr, "[Main] WebSocket thread start failed\n");
    }

    /* RTMP 模式：等待 WS 注册成功后再启动推流，避免服务器丢弃未注册设备的 RTMP 数据 */
    if (stream_type == STREAM_TYPE_RTMP && state.ws_started) {
        printf("[Main] Waiting for WS registration before starting RTMP stream...\n");
        int wait_ms = 0;
        while (!g_ws_ready && wait_ms < 10000 && !g_should_quit) {
            usleep(100000);
            wait_ms += 100;
        }
        if (g_ws_ready) {
            printf("[Main] WS registration confirmed. Starting RTMP stream now.\n");
        } else {
            printf("[Main] WS registration timeout (10s). Starting RTMP stream anyway.\n");
        }
    }

    // NRF24 IMU 控制在常驻 main 循环运行，不受推流重启影响。
    printf("[Main] NRF24 control timer started (50ms)\n");

    printf("[Main] Starting %s stream...\n",
           (stream_type == STREAM_TYPE_RTMP) ? "RTMP" : "RTSP");
    printf("[Main] Press Ctrl+C to stop\n\n");

    int stream_ret = stream_manager_init(VIDEO_DEVICE, RTMP_URL);
    if (stream_ret == 0 && !g_should_quit)
        stream_ret = stream_manager_start(stream_type);
    if (stream_ret != 0) {
        fprintf(stderr, "[Main] Stream failed\n");
        exit_code = 1;
    } else {
        while (!g_should_quit) {
            check_quit_timer(NULL);
            arm_power_tick();
            if (arm_power_is_on() && !arm_power_shutdown_in_progress())
                nrf24_control_update();
            usleep(50000);
        }
    }
    shutdown_runtime(&state, state.uart_initialized);
    if (g_should_quit && exit_code == 0) return 0;
    return exit_code;
}
