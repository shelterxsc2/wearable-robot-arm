/**
 * ctrl_server.cpp - 端侧 HTTP 控制服务器
 * 提供 REST API 用于模式切换、标定控制、状态查询等
 * 不依赖外部 HTTP 库，基于标准 socket 实现
 */
#include "ctrl_server.h"
#include "rga_npu.h"
#include "uart_comm.h"
#include "nrf24_linux.h"
#include "stream_manager.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <fcntl.h>
#include <errno.h>

#define RESP_OK     "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n"
#define RESP_BAD    "HTTP/1.1 400 Bad Request\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n"
#define RESP_NOTF   "HTTP/1.1 404 Not Found\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n"

static int g_srv_fd = -1;
static pthread_t g_srv_tid = 0;
static volatile int g_srv_running = 0;

static void send_json(int client, const char *header, const char *json)
{
    char buf[2048];
    snprintf(buf, sizeof(buf), "%s%s", header, json);
    send(client, buf, strlen(buf), 0);
}

/* 从 URL 中提取查询参数值 */
static int get_query_param(const char *url, const char *key, char *out, size_t out_len)
{
    const char *q = strchr(url, '?');
    if (!q) return -1;
    q++;

    size_t klen = strlen(key);
    char search[klen + 2];
    snprintf(search, sizeof(search), "%s=", key);

    const char *p = strstr(q, search);
    if (!p) return -1;
    p += klen + 1;

    const char *end = strchr(p, '&');
    if (!end) end = p + strlen(p);

    size_t len = (size_t)(end - p);
    if (len >= out_len) len = out_len - 1;
    memcpy(out, p, len);
    out[len] = '\0';
    return 0;
}

/* 处理单个 HTTP 请求 */
static void handle_client(int client)
{
    char req[2048] = {0};
    int n = (int)recv(client, req, sizeof(req) - 1, 0);
    if (n <= 0) {
        close(client);
        return;
    }
    req[n] = '\0';

    /* 解析 method 和 path */
    char method[16] = "";
    char path[256] = "";
    sscanf(req, "%15s %255s", method, path);

    printf("[Ctrl] %s %s\n", method, path);

    if (strcmp(method, "GET") == 0 && strcmp(path, "/status") == 0) {
        /* 返回系统状态 */
        PoseMode mode = get_pose_mode();
        StreamType st = get_current_stream_type();
        char json[512];
        snprintf(json, sizeof(json),
                 "{\"pose_mode\":\"%s\",\"stream_type\":\"%s\","
                 "\"uart_move_complete\":%d,\"head_stationary\":%d,\"arm_stable\":%d}"
                 "\n",
                 pose_mode_name(mode),
                 (st == STREAM_TYPE_RTMP) ? "rtmp" : "rtsp",
                 g_uart_move_complete,
                 g_head_stationary,
                 g_arm_stable);
        send_json(client, RESP_OK, json);
    }
    else if (strcmp(method, "POST") == 0 && strncmp(path, "/mode", 5) == 0) {
        char type_val[32] = "";
        if (get_query_param(path, "type", type_val, sizeof(type_val)) == 0) {
            if (strcmp(type_val, "face") == 0) {
                set_pose_mode(MODE_FACE);
                send_json(client, RESP_OK, "{\"ok\":true,\"mode\":\"face\"}\n");
            } else if (strcmp(type_val, "body") == 0) {
                set_pose_mode(MODE_BODY);
                send_json(client, RESP_OK, "{\"ok\":true,\"mode\":\"body\"}\n");
            } else {
                send_json(client, RESP_BAD, "{\"ok\":false,\"error\":\"invalid type\"}\n");
            }
        } else {
            send_json(client, RESP_BAD, "{\"ok\":false,\"error\":\"missing type\"}\n");
        }
    }
    else if (strcmp(method, "POST") == 0 && strncmp(path, "/calib", 6) == 0) {
        char mode_val[8] = "";
        if (get_query_param(path, "mode", mode_val, sizeof(mode_val)) == 0) {
            int mode = atoi(mode_val);
            if (mode < 0 || mode > 3) {
                send_json(client, RESP_BAD,
                          "{\"ok\":false,\"error\":\"mode must be 0..3\"}\n");
                close(client);
                return;
            }
            FILE *fp = fopen("/tmp/calib_mode.txt", "w");
            if (fp) {
                fprintf(fp, "%d\n", mode);
                fclose(fp);
            }
            char json[128];
            snprintf(json, sizeof(json), "{\"ok\":true,\"calib_mode\":%d}\n", mode);
            send_json(client, RESP_OK, json);
        } else {
            /* 没有 mode 参数时，返回当前标定状态 */
            FILE *fp = fopen("/tmp/calib_mode.txt", "r");
            int mode = 0;
            if (fp) {
                fscanf(fp, "%d", &mode);
                fclose(fp);
            }
            char json[128];
            snprintf(json, sizeof(json), "{\"ok\":true,\"calib_mode\":%d}\n", mode);
            send_json(client, RESP_OK, json);
        }
    }
    else if (strcmp(method, "POST") == 0 && strncmp(path, "/servo", 6) == 0) {
        char k1_str[16] = "";
        char k2_str[16] = "";
        float k1 = -1, k2 = -1;
        if (get_query_param(path, "k1", k1_str, sizeof(k1_str)) == 0) {
            k1 = (float)atof(k1_str);
        }
        if (get_query_param(path, "k2", k2_str, sizeof(k2_str)) == 0) {
            k2 = (float)atof(k2_str);
        }
        if (k1 >= 0 && k2 >= 0) {
            /* 写入 servo 标定文件供标定模式读取 */
            FILE *fp = fopen("/tmp/servo_calib.txt", "w");
            if (fp) {
                fprintf(fp, "%.1f\n", k1);
                fclose(fp);
            }
            /* 直接发送一次测试指令 */
            uart_send_arm_target(0.0f, 67.0f, 40.0f, k1, k2, 0x01);
            char json[256];
            snprintf(json, sizeof(json), "{\"ok\":true,\"k1\":%.1f,\"k2\":%.1f}\n", k1, k2);
            send_json(client, RESP_OK, json);
        } else {
            send_json(client, RESP_BAD, "{\"ok\":false,\"error\":\"missing k1 or k2\"}\n");
        }
    }
    else if (strcmp(method, "POST") == 0 && strncmp(path, "/cmd", 4) == 0) {
        char action[64] = "";
        if (get_query_param(path, "action", action, sizeof(action)) == 0) {
            if (strcmp(action, "rebaseline") == 0) {
                /* 重新标定 baseline：删除标定文件，下次 nrf24_control_update 会重新设置 baseline */
                remove("/tmp/calib_mode.txt");
                remove("/tmp/servo_calib.txt");
                send_json(client, RESP_OK, "{\"ok\":true,\"action\":\"rebaseline\"}\n");
            } else if (strcmp(action, "nrf24_reset") == 0) {
                /* 通过全局状态触发 NRF24 重新初始化（实际效果有限，仅做演示） */
                send_json(client, RESP_OK, "{\"ok\":true,\"action\":\"nrf24_reset\"}\n");
            } else {
                send_json(client, RESP_BAD, "{\"ok\":false,\"error\":\"unknown action\"}\n");
            }
        } else {
            send_json(client, RESP_BAD, "{\"ok\":false,\"error\":\"missing action\"}\n");
        }
    }
    else {
        send_json(client, RESP_NOTF, "{\"ok\":false,\"error\":\"not found\"}\n");
    }

    close(client);
}

static void* server_thread_func(void *arg)
{
    int port = *(int*)arg;
    free(arg);

    int srv = socket(AF_INET, SOCK_STREAM, 0);
    if (srv < 0) {
        perror("[Ctrl] socket");
        return NULL;
    }

    int reuse = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(srv, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("[Ctrl] bind");
        close(srv);
        return NULL;
    }

    if (listen(srv, 5) < 0) {
        perror("[Ctrl] listen");
        close(srv);
        return NULL;
    }

    g_srv_fd = srv;
    printf("[Ctrl] HTTP control server listening on port %d\n", port);

    while (g_srv_running) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int client = accept(srv, (struct sockaddr *)&client_addr, &client_len);
        if (client < 0) {
            if (errno == EINTR || errno == EAGAIN) continue;
            break;
        }
        handle_client(client);
    }

    close(srv);
    g_srv_fd = -1;
    printf("[Ctrl] HTTP control server stopped\n");
    return NULL;
}

int ctrl_server_start(int port)
{
    if (g_srv_running) return 0;

    int *pport = (int *)malloc(sizeof(int));
    if (!pport) return -1;
    *pport = port;

    g_srv_running = 1;
    if (pthread_create(&g_srv_tid, NULL, server_thread_func, pport) != 0) {
        g_srv_running = 0;
        free(pport);
        return -1;
    }
    return 0;
}

void ctrl_server_stop(void)
{
    if (!g_srv_running) return;
    g_srv_running = 0;
    if (g_srv_fd >= 0) {
        shutdown(g_srv_fd, SHUT_RDWR);
    }
    pthread_join(g_srv_tid, NULL);
}
