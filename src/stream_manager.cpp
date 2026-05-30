/**
 * stream_manager.cpp - 推流管理器
 * 初始化时探测 RTMP 服务器，决定推 RTMP 还是 RTSP
 */
#include "stream_manager.h"
#include "gst_rtsp.h"
#include "gst_rtmp.h"
#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <fcntl.h>
#include <errno.h>
#include <poll.h>

static StreamType g_current_type = STREAM_TYPE_RTSP;

/* 从 RTMP URL 解析 host 和 port */
static int parse_rtmp_url(const char *url, char *host, size_t host_len, int *port)
{
    /* 跳过 rtmp:// */
    const char *p = url;
    if (strncmp(p, "rtmp://", 7) == 0) {
        p += 7;
    } else if (strncmp(p, "rtmps://", 8) == 0) {
        p += 8;
    } else {
        return -1;
    }

    const char *slash = strchr(p, '/');
    const char *colon = strchr(p, ':');

    if (colon && (!slash || colon < slash)) {
        int hlen = (int)(colon - p);
        if (hlen >= (int)host_len) hlen = (int)host_len - 1;
        memcpy(host, p, hlen);
        host[hlen] = '\0';
        *port = atoi(colon + 1);
    } else {
        int hlen = slash ? (int)(slash - p) : (int)strlen(p);
        if (hlen >= (int)host_len) hlen = (int)host_len - 1;
        memcpy(host, p, hlen);
        host[hlen] = '\0';
        *port = 1935; /* RTMP 默认端口 */
    }
    return 0;
}

int probe_rtmp_server(const char *rtmp_url, int timeout_ms)
{
    char host[256] = "";
    int port = 1935;

    if (parse_rtmp_url(rtmp_url, host, sizeof(host), &port) != 0) {
        fprintf(stderr, "[Probe] Invalid RTMP URL: %s\n", rtmp_url);
        return 0;
    }

    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("[Probe] socket");
        return 0;
    }

    /* 非阻塞连接 */
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);

    struct hostent *he = gethostbyname(host);
    if (he && he->h_addr_list[0]) {
        memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
    } else if (inet_pton(AF_INET, host, &addr.sin_addr) <= 0) {
        fprintf(stderr, "[Probe] Cannot resolve host: %s\n", host);
        close(sock);
        return 0;
    }

    int rc = connect(sock, (struct sockaddr *)&addr, sizeof(addr));
    if (rc < 0 && errno == EINPROGRESS) {
        struct pollfd pfd = { sock, POLLOUT, 0 };
        rc = poll(&pfd, 1, timeout_ms);
        if (rc > 0) {
            int so_error;
            socklen_t len = sizeof(so_error);
            getsockopt(sock, SOL_SOCKET, SO_ERROR, &so_error, &len);
            rc = (so_error == 0) ? 0 : -1;
        } else {
            rc = -1;
        }
    } else if (rc < 0) {
        rc = -1;
    }

    close(sock);

    if (rc == 0) {
        printf("[Probe] RTMP server %s:%d is reachable.\n", host, port);
        return 1;
    } else {
        printf("[Probe] RTMP server %s:%d is NOT reachable. Fallback to RTSP.\n", host, port);
        return 0;
    }
}

int start_stream(const char *device, const char *rtmp_url, StreamType type, GMainLoop **loop_ptr)
{
    g_current_type = type;
    if (type == STREAM_TYPE_RTMP) {
        return start_rtmp_stream(device, rtmp_url, loop_ptr);
    } else {
        return start_rtsp_server(device, loop_ptr);
    }
}

StreamType get_current_stream_type(void)
{
    return g_current_type;
}
