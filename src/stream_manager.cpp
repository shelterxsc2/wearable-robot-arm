/**
 * stream_manager.cpp - 推流管理器
 * 初始化时探测 RTMP 服务器，决定推 RTMP 还是 RTSP
 */
#include "stream_manager.h"
#include "gst_rtsp.h"
#include "gst_rtmp.h"
#include "gst_unified.h"
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
#include <pthread.h>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <string>

static std::atomic<StreamType> g_current_type{STREAM_TYPE_RTSP};

namespace {
std::mutex manager_mutex;
std::condition_variable manager_cv;
std::string manager_device;
std::string manager_rtmp_url;
pthread_t manager_thread{};
bool manager_thread_active = false;
bool manager_starting = false;
bool manager_running = false;
bool manager_stopping = false;
int manager_last_result = -1;
GMainLoop *manager_loop = NULL;

void *stream_worker(void *)
{
    StreamType type;
    std::string device;
    std::string url;
    {
        std::lock_guard<std::mutex> lock(manager_mutex);
        type = g_current_type.load();
        device = manager_device;
        url = manager_rtmp_url;
    }

    GMainLoop *loop = NULL;
    int ret = start_unified_stream(device.c_str(), url.c_str(), type, &loop);
    {
        std::lock_guard<std::mutex> lock(manager_mutex);
        manager_loop = NULL;
        manager_running = false;
        manager_starting = false;
        manager_last_result = ret;
    }
    manager_cv.notify_all();
    return NULL;
}

int start_locked(StreamType type, std::unique_lock<std::mutex> &lock)
{
    g_current_type = type;
    manager_loop = NULL;
    manager_last_result = -1;
    manager_starting = true;
    manager_running = false;
    manager_stopping = false;
    if (pthread_create(&manager_thread, NULL, stream_worker, NULL) != 0) {
        manager_starting = false;
        return -1;
    }
    manager_thread_active = true;

    /* start_stream publishes its loop only after the pipeline/server exists. */
    manager_cv.wait(lock, [] { return manager_loop != NULL || !manager_starting; });
    bool ready = manager_loop != NULL;
    if (ready) {
        manager_running = true;
        manager_starting = false;
        return 0;
    }
    return -1;
}

void stop_locked(std::unique_lock<std::mutex> &lock)
{
    if (!manager_thread_active) return;
    manager_stopping = true;
    GMainLoop *loop = manager_loop;
    if (loop) g_main_loop_quit(loop);
    pthread_t tid = manager_thread;
    lock.unlock();
    pthread_join(tid, NULL);
    lock.lock();
    manager_thread_active = false;
    manager_loop = NULL;
    manager_running = false;
    manager_starting = false;
    manager_stopping = false;
}
}

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
    return g_current_type.load();
}

int stream_manager_init(const char *device, const char *rtmp_url)
{
    if (!device || !*device || !rtmp_url || !*rtmp_url) return -1;
    std::lock_guard<std::mutex> lock(manager_mutex);
    if (manager_thread_active) return -1;
    manager_device = device;
    manager_rtmp_url = rtmp_url;
    return 0;
}

int stream_manager_start(StreamType type)
{
    std::unique_lock<std::mutex> lock(manager_mutex);
    if (manager_thread_active || manager_device.empty()) return -1;
    int ret = start_locked(type, lock);
    if (ret != 0 && manager_thread_active) stop_locked(lock);
    return ret;
}

int stream_manager_switch(const char *mode, int *changed)
{
    if (changed) *changed = 0;
    if (!mode) return -1;

    StreamType target;
    if (strcmp(mode, "local") == 0) target = STREAM_TYPE_RTSP;
    else if (strcmp(mode, "cloud") == 0) {
        std::string url;
        { std::lock_guard<std::mutex> lock(manager_mutex); url = manager_rtmp_url; }
        if (!probe_rtmp_server(url.c_str(), 3000)) return -2;
        target = STREAM_TYPE_RTMP;
    } else if (strcmp(mode, "auto") == 0) {
        std::string url;
        { std::lock_guard<std::mutex> lock(manager_mutex); url = manager_rtmp_url; }
        target = probe_rtmp_server(url.c_str(), 3000) ? STREAM_TYPE_RTMP : STREAM_TYPE_RTSP;
    } else return -1;

    std::unique_lock<std::mutex> lock(manager_mutex);
    if (!manager_thread_active || !manager_running) return -3;
    StreamType old = g_current_type.load();
    if (old == target) return 0;
    lock.unlock();
    int ret = unified_stream_switch(target);
    lock.lock();
    if (ret != 0) return -2; /* old branch is retained until target is ready */
    g_current_type = target;
    if (changed) *changed = 1;
    return 0;
}

void stream_manager_stop(void)
{
    std::unique_lock<std::mutex> lock(manager_mutex);
    stop_locked(lock);
}

int stream_manager_is_running(void)
{
    std::lock_guard<std::mutex> lock(manager_mutex);
    return manager_running ? 1 : 0;
}

const char *stream_manager_state_name(void)
{
    std::lock_guard<std::mutex> lock(manager_mutex);
    if (manager_stopping) return "stopping";
    if (manager_starting) return "starting";
    if (!manager_running) return "unavailable";
    return g_current_type.load() == STREAM_TYPE_RTMP ? "streaming_cloud" : "streaming_local";
}

void stream_manager_notify_loop_ready(GMainLoop *loop)
{
    std::lock_guard<std::mutex> lock(manager_mutex);
    manager_loop = loop;
    manager_cv.notify_all();
}
