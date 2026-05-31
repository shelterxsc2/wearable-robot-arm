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

#include "wifi.h"
#include "rga_npu.h"
#include "stream_manager.h"
#include "ctrl_server.h"
#include "ws_client.h"
#include "bluetooth_spp.h"
#include "uart_comm.h"
#include "nrf24_linux.h"
#include "gst_rtmp.h"

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

static gboolean g_running = TRUE;
static pthread_t g_nrf24_tid = 0;
static int g_nrf24_enabled = 0;
static GMainLoop *g_loop = NULL;
static volatile sig_atomic_t g_should_quit = 0;
static volatile sig_atomic_t g_ws_ready = 0;

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
    g_running = FALSE;
}

/* 在主循环线程中安全地检查并退出 */
static gboolean check_quit_timer(gpointer user_data) {
    (void)user_data;
    if (g_should_quit && g_loop) {
        printf("\n[Main] Stopping...\n");
        GMainLoop *loop = g_loop;
        g_loop = NULL;
        g_main_loop_quit(loop);
        return G_SOURCE_REMOVE;
    }
    return G_SOURCE_CONTINUE;
}

/* ========== WebSocket 云端交互线程（来自历史备份） ========== */
static void *ws_worker_thread(void *arg) {
    (void)arg;
    srand((unsigned)time(NULL));

    int sock = -1;
    double reconnect_delay = 2.0;
    time_t start_time = time(NULL);
    int registered = 0;
    char discard[2048];

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
        ssize_t n = recv(sock, discard, sizeof(discard), 0);
        if (n == 0) {
            printf("[WS] Server closed connection\n");
            ws_close(sock);
            sock = -1;
            registered = 0;
            continue;
        } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            printf("[WS] Recv error: %s\n", strerror(errno));
            ws_close(sock);
            sock = -1;
            registered = 0;
            continue;
        }

        /* 发送心跳（带 data） */
        if (registered) {
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

int main(int argc, char *argv[]) {
    // 取消 stdout 缓冲，确保日志实时落盘
    setbuf(stdout, NULL);

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

    // 3. 初始化NPU
    printf("\n[Main] Initializing NPU...\n");
    if (init_npu() != 0) {
        fprintf(stderr, "[Main] NPU initialization failed\n");
        return 1;
    }

    // 4. 初始化RGA
    printf("\n[Main] Initializing RGA...\n");
    if (init_rga() != 0) {
        fprintf(stderr, "[Main] RGA initialization failed\n");
        cleanup_npu();
        return 1;
    }

    // 5. 蓝牙 SPP（临时禁用）
    // bluetooth_spp_init / bluetooth_spp_start 已注释

    // 6. 初始化 UART
    printf("\n[Main] Initializing UART...\n");
    if (uart_init("/dev/ttyS9", 115200) != 0) {
        fprintf(stderr, "[Main] UART init failed, continuing without motor control\n");
    } else {
        uart_start_receiver(NULL);
        printf("[Main] UART ready @ /dev/ttyS9 115200\n");
    }

    // 7. 启动端侧控制服务器
    printf("\n[Main] Starting control server on port %d...\n", CTRL_SERVER_PORT);
    if (ctrl_server_start(CTRL_SERVER_PORT) != 0) {
        fprintf(stderr, "[Main] Control server start failed, continuing without it\n");
    }

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

    // 9. 探测云服务器并选择推流模式
    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);

    gst_init(&argc, &argv);

    /* 在主循环线程中注册安全退出检查器（信号处理函数中不能直接调用 g_main_loop_quit） */
    g_timeout_add(100, check_quit_timer, NULL);

    StreamType stream_type = STREAM_TYPE_RTSP;
    pthread_t ws_tid = 0;
    int ws_started = 0;

    printf("\n[Main] Probing RTMP server (%s)...\n", RTMP_URL);
    if (probe_rtmp_server(RTMP_URL, 3000)) {
        stream_type = STREAM_TYPE_RTMP;
        printf("[Main] RTMP server is UP. Will push RTMP + WebSocket.\n");
        if (pthread_create(&ws_tid, NULL, ws_worker_thread, NULL) == 0) {
            ws_started = 1;
            printf("[Main] WebSocket reporter started -> %s\n", WS_URL);
        } else {
            fprintf(stderr, "[Main] WebSocket thread start failed\n");
        }
    } else {
        stream_type = STREAM_TYPE_RTSP;
        printf("[Main] RTMP server is DOWN. Will fallback to local RTSP (no WebSocket).\n");
    }

    /* RTMP 模式：等待 WS 注册成功后再启动推流，避免服务器丢弃未注册设备的 RTMP 数据 */
    if (stream_type == STREAM_TYPE_RTMP && ws_started) {
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

    // NRF24 IMU 控制：用 GLib 定时器每 50ms 独立运行，不依赖视频帧
    g_timeout_add(50, [](gpointer) -> gboolean {
        nrf24_control_update();
        return G_SOURCE_CONTINUE;
    }, NULL);
    printf("[Main] NRF24 control timer started (50ms)\n");

    printf("[Main] Starting %s stream...\n",
           (stream_type == STREAM_TYPE_RTMP) ? "RTMP" : "RTSP");
    printf("[Main] Press Ctrl+C to stop\n\n");

    int stream_ret = start_stream(VIDEO_DEVICE, RTMP_URL, stream_type, &g_loop);
    /* start_stream 返回时内部已 unref loop，防止 sigint_handler 对已释放指针调用 quit */
    g_loop = NULL;

    if (stream_ret != 0) {
        fprintf(stderr, "[Main] Stream failed\n");
        g_running = FALSE;
        if (ws_started) pthread_join(ws_tid, NULL);
        bluetooth_spp_stop();
        bluetooth_spp_cleanup();
        cleanup_npu();
        cleanup_rga();
        ctrl_server_stop();
        return 1;
    }

    printf("\n[Main] Cleaning up...\n");
    g_running = FALSE;
    if (ws_started) pthread_join(ws_tid, NULL);
    if (g_nrf24_enabled) {
        nrf24_rx_thread_stop();
        pthread_join(g_nrf24_tid, NULL);
        nrf24_linux_deinit();
        printf("[Main] NRF24 stopped\n");
    }
    uart_cleanup();
    bluetooth_spp_stop();
    bluetooth_spp_cleanup();
    ctrl_server_stop();
    cleanup_npu();
    cleanup_rga();
    printf("[Main] Shutdown complete\n");
    return 0;
}
