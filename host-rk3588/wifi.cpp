/**
 * wifi_fix.cpp - 修复版 WiFi 模块，解决 DHCP 问题
 */
#include "wifi.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cstddef>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <wpa_ctrl.h>
#include <fcntl.h>

#ifndef WPA_CTRL_PATH
#define WPA_CTRL_PATH "/var/run/wpa_supplicant"
#endif

// 检查网络接口是否存在
static int check_interface(const char *ifname) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    
    int ret = ioctl(sock, SIOCGIFFLAGS, &ifr);
    close(sock);
    
    if (ret < 0) {
        fprintf(stderr, "❌ Interface %s does not exist\n", ifname);
        return -1;
    }
    return 0;
}

// 等待接口就绪
static int wait_for_interface(const char *ifname, int timeout_sec) {
    for (int i = 0; i < timeout_sec; i++) {
        if (check_interface(ifname) == 0) {
            return 0;
        }
        sleep(1);
    }
    return -1;
}

int wifi_enable_interface(const char *ifname) {
    // 先检查接口是否存在
    if (wait_for_interface(ifname, 5) < 0) {
        fprintf(stderr, "❌ WiFi interface %s not found\n", ifname);
        return -1;
    }
    
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    
    ioctl(sock, SIOCGIFFLAGS, &ifr);
    ifr.ifr_flags |= IFF_UP | IFF_RUNNING;
    ioctl(sock, SIOCSIFFLAGS, &ifr);
    close(sock);
    
    // 等待接口启动
    sleep(1);
    
    printf("✅ WiFi interface %s enabled\n", ifname);
    return 0;
}

int wifi_connect_wpa(const char *ifname, const char *ssid, const char *pwd) {
    char path[128];
    snprintf(path, sizeof(path), "%s/%s", WPA_CTRL_PATH, ifname);
    
    // 等待 wpa_supplicant 就绪
    int retries = 0;
    struct wpa_ctrl *ctrl = NULL;
    while (retries < 10 && !ctrl) {
        ctrl = wpa_ctrl_open(path);
        if (!ctrl) {
            // 尝试启动 wpa_supplicant
            if (retries == 0) {
                char cmd[256];
                snprintf(cmd, sizeof(cmd),
                         "wpa_supplicant -B -i %s -c /etc/wpa_supplicant/wpa_supplicant.conf > /dev/null 2>&1",
                         ifname);
                system(cmd);
            }
            sleep(1);
            retries++;
        }
    }
    
    if (!ctrl) {
        fprintf(stderr, "❌ Cannot open wpa_supplicant control interface\n");
        return -1;
    }

    char cmd[256], reply[1024];
    size_t rlen;

    // 添加网络
    rlen = sizeof(reply);
    snprintf(cmd, sizeof(cmd), "ADD_NETWORK");
    if (wpa_ctrl_request(ctrl, cmd, strlen(cmd), reply, &rlen, NULL) < 0) {
        fprintf(stderr, "❌ ADD_NETWORK failed\n");
        wpa_ctrl_close(ctrl);
        return -1;
    }
    int id = atoi(reply);

    // 设置 SSID
    rlen = sizeof(reply);
    snprintf(cmd, sizeof(cmd), "SET_NETWORK %d ssid \"%s\"", id, ssid);
    wpa_ctrl_request(ctrl, cmd, strlen(cmd), reply, &rlen, NULL);

    // 设置密码
    rlen = sizeof(reply);
    snprintf(cmd, sizeof(cmd), "SET_NETWORK %d psk \"%s\"", id, pwd);
    wpa_ctrl_request(ctrl, cmd, strlen(cmd), reply, &rlen, NULL);

    // 启用网络
    rlen = sizeof(reply);
    snprintf(cmd, sizeof(cmd), "ENABLE_NETWORK %d", id);
    wpa_ctrl_request(ctrl, cmd, strlen(cmd), reply, &rlen, NULL);
    
    // 保存配置
    rlen = sizeof(reply);
    snprintf(cmd, sizeof(cmd), "SAVE_CONFIG");
    wpa_ctrl_request(ctrl, cmd, strlen(cmd), reply, &rlen, NULL);

    printf("🔗 Connecting to WiFi: %s ...\n", ssid);
    
    // 等待连接完成，最多30秒
    for (int i = 0; i < 30; i++) {
        memset(reply, 0, sizeof(reply));
        rlen = sizeof(reply);
        snprintf(cmd, sizeof(cmd), "STATUS");
        if (wpa_ctrl_request(ctrl, cmd, strlen(cmd), reply, &rlen, NULL) >= 0) {
            if (strstr(reply, "wpa_state=COMPLETED")) {
                printf("✅ WiFi connected: %s\n", ssid);
                wpa_ctrl_close(ctrl);
                // 等待接口稳定
                sleep(2);
                return 0;
            }
        }
        sleep(1);
    }
    
    fprintf(stderr, "❌ WiFi connection timeout\n");
    wpa_ctrl_close(ctrl);
    return -1;
}

// 检查是否已有 IP
static int check_ip(const char *ifname) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    
    int has_ip = 0;
    if (ioctl(sock, SIOCGIFADDR, &ifr) >= 0) {
        struct sockaddr_in *addr = (struct sockaddr_in *)&ifr.ifr_addr;
        if (addr->sin_addr.s_addr != 0) {
            has_ip = 1;
        }
    }
    close(sock);
    return has_ip;
}

int wifi_dhcp(const char *ifname) {
    // 先检查是否已有 IP
    if (check_ip(ifname)) {
        printf("✅ IP already assigned\n");
        return 0;
    }
    
    // 杀死旧的 udhcpc 进程
    system("killall udhcpc 2>/dev/null");
    sleep(1);
    
    printf("🔄 Requesting DHCP lease...\n");
    
    // 使用更长的超时时间和更多重试
    // -t 20: 最多20次尝试
    // -T 3: 每次超时3秒
    // -n: 退出如果租约获取失败
    // -q: 退出成功后
    pid_t pid = fork();
    if (pid == 0) {
        // 子进程执行 udhcpc
        execl("/sbin/udhcpc", "udhcpc", "-i", ifname, 
              "-t", "20",      // 20次重试
              "-T", "3",       // 3秒超时
              "-n",            // 失败退出
              "-q",            // 静默
              "-x", "hostname:rv1126b", // 主机名
              NULL);
        exit(1);
    }
    
    int status;
    waitpid(pid, &status, 0);
    
    if (WIFEXITED(status) && WEXITSTATUS(status) == 0) {
        printf("✅ DHCP acquired IP\n");
        return 0;
    }
    
    // 如果 udhcpc 失败，尝试 dhclient
    printf("⚠️ udhcpc failed, trying dhclient...\n");
    pid = fork();
    if (pid == 0) {
        execl("/sbin/dhclient", "dhclient", "-v", ifname, NULL);
        exit(1);
    }
    waitpid(pid, &status, 0);
    
    if (WIFEXITED(status) && WEXITSTATUS(status) == 0) {
        printf("✅ DHCP (dhclient) acquired IP\n");
        return 0;
    }
    
    fprintf(stderr, "❌ DHCP failed\n");
    return -1;
}

void wifi_print_ip(const char *ifname, char *ip_str, size_t ip_len) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    
    if (ioctl(sock, SIOCGIFADDR, &ifr) >= 0) {
        struct sockaddr_in *addr = (struct sockaddr_in *)&ifr.ifr_addr;
        snprintf(ip_str, ip_len, "%s", inet_ntoa(addr->sin_addr));
        printf("\n===== WiFi Network Info =====\n");
        printf("IP address: %s\n", ip_str);
        
        // 显示子网掩码
        if (ioctl(sock, SIOCGIFNETMASK, &ifr) >= 0) {
            struct sockaddr_in *mask = (struct sockaddr_in *)&ifr.ifr_netmask;
            printf("Netmask: %s\n", inet_ntoa(mask->sin_addr));
        }
    } else {
        printf("⚠️ Could not get IP address\n");
        ip_str[0] = '\0';
    }
    close(sock);
}

// 备用：使用静态 IP
int wifi_static_ip(const char *ifname, const char *ip, const char *netmask, const char *gateway) {
    char cmd[256];
    
    printf("📝 Setting static IP: %s\n", ip);
    
    // 设置 IP
    snprintf(cmd, sizeof(cmd), "ifconfig %s %s netmask %s up", 
             ifname, ip, netmask);
    system(cmd);
    
    // 设置网关
    if (gateway) {
        snprintf(cmd, sizeof(cmd), "route add default gw %s 2>/dev/null || true", gateway);
        system(cmd);
    }
    
    // 设置 DNS
    system("echo 'nameserver 114.114.114.114' > /etc/resolv.conf");
    system("echo 'nameserver 8.8.8.8' >> /etc/resolv.conf");
    
    printf("✅ Static IP configured: %s\n", ip);
    return 0;
}
