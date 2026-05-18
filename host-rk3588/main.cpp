/**
 * ELF2 (RK3588) YOLOv8-Pose 完整版 (修复DHCP问题)
 * 流程: WiFi -> GStreamer -> RGA -> NPU -> RGA -> RTSP
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

#include "wifi.h"
#include "rga_npu.h"
#include "gst_rtsp.h"
#include "bluetooth_spp.h"

#define VIDEO_DEVICE        "/dev/video21"  // USB Camera3 (Realtek)
#define WIFI_IFNAME         "wlan0"
#define WIFI_SSID           "tkh1288"
#define WIFI_PASSWD         "supercell5000"
#define CPU_FREQ_TARGET     1608000

static gboolean g_running = TRUE;
static GMainLoop *g_loop = NULL;

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
    
    // 尝试常见网段
    // 方案1: 192.168.1.x
    printf("📝 Trying 192.168.1.x subnet...\n");
    system("ifconfig wlan0 192.168.1.100 netmask 255.255.255.0 up");
    system("route add default gw 192.168.1.1 2>/dev/null || true");
    system("echo 'nameserver 114.114.114.114' > /etc/resolv.conf");
    
    // 检查是否成功
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
    
    // 方案2: 如果上面的失败，尝试 192.168.0.x
    printf("📝 Trying 192.168.0.x subnet...\n");
    system("ifconfig wlan0 192.168.0.100 netmask 255.255.255.0 up");
    system("route add default gw 192.168.0.1 2>/dev/null || true");
    
    printf("✅ Static IP fallback complete (192.168.0.100)\n");
}

static void sigint_handler(int sig) {
    (void)sig;
    printf("\n[Main] Stopping...\n");
    g_running = FALSE;
    if (g_loop) {
        g_main_loop_quit(g_loop);
    }
}

int main(int argc, char *argv[]) {
    printf("============================================================\n");
    printf("ELF2 (RK3588) YOLOv8-Pose AI推流系统 (DHCP修复版)\n");
    printf("============================================================\n\n");
    
    // 1. CPU调频
    set_cpu_performance();
    
    // 1.5 尝试屏蔽 MIPI 摄像头的内核日志（不影响 USB 摄像头）
    system("dmesg -n 3 2>/dev/null || true");
    system("rmmod rkcif-mipi-lvds 2>/dev/null || true");
    system("rmmod rkcif 2>/dev/null || true");
    
    // 2. 初始化WiFi
    printf("[Main] Initializing WiFi...\n");
    if (wifi_enable_interface(WIFI_IFNAME) != 0) {
        fprintf(stderr, "[Main] Failed to enable WiFi, continuing...\n");
    } else if (wifi_connect_wpa(WIFI_IFNAME, WIFI_SSID, WIFI_PASSWD) != 0) {
        fprintf(stderr, "[Main] Failed to connect WiFi, continuing...\n");
    }
    
    // DHCP获取IP
    if (wifi_dhcp(WIFI_IFNAME) != 0) {
        setup_static_ip_fallback();
    }
    
    // 清理可能残留的旧IP（避免双IP问题）
    printf("[Main] Cleaning up duplicate IPs if any...\n");
    system("ip addr flush dev wlan0 label wlan0:0 2>/dev/null || true");
    
    char ip_str[64] = "";
    wifi_print_ip(WIFI_IFNAME, ip_str, sizeof(ip_str));
    
    // 如果仍然没有IP，强制使用静态IP
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
    // printf("\n[Main] Initializing Bluetooth SPP...\n");
    // if (bluetooth_spp_init() != 0) {
    //     fprintf(stderr, "[Main] Bluetooth init failed, continuing without BT control\n");
    // } else {
    //     bluetooth_spp_start();
    // }
    
    // 6. 启动RTSP推流
    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);
    
    printf("\n[Main] Mode: FACE (default)\n");
    printf("[Main] Starting RTSP stream...\n");
    printf("[Main] Press Ctrl+C to stop\n\n");

    gst_init(&argc, &argv);
    
    if (start_rtsp_server(VIDEO_DEVICE, &g_loop) != 0) {
        fprintf(stderr, "[Main] RTSP stream failed\n");
        bluetooth_spp_stop();
        bluetooth_spp_cleanup();
        cleanup_npu();
        cleanup_rga();
        return 1;
    }
    
    printf("\n[Main] Cleaning up...\n");
    g_running = FALSE;
    bluetooth_spp_stop();
    bluetooth_spp_cleanup();
    cleanup_npu();
    cleanup_rga();
    printf("[Main] Shutdown complete\n");
    return 0;
}
