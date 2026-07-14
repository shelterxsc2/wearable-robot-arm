#ifndef STREAM_MANAGER_H
#define STREAM_MANAGER_H

#include <gst/gst.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    STREAM_TYPE_RTSP = 0,
    STREAM_TYPE_RTMP = 1
} StreamType;

/**
 * @brief 探测 RTMP 服务器是否可达
 * @param rtmp_url  RTMP 地址，如 "rtmp://host:1935/live/key"
 * @param timeout_ms TCP 连接超时，单位毫秒
 * @return 1 可达，0 不可达
 */
int probe_rtmp_server(const char *rtmp_url, int timeout_ms);

/**
 * @brief 启动推流（RTSP 或 RTMP）
 * @param device    V4L2 设备路径
 * @param rtmp_url  RTMP 推流地址（RTMP 模式时使用）
 * @param type      STREAM_TYPE_RTSP 或 STREAM_TYPE_RTMP
 * @param loop_ptr  输出 GMainLoop 指针
 * @return 0 成功，-1 失败
 */
int start_stream(const char *device, const char *rtmp_url, StreamType type, GMainLoop **loop_ptr);

/* Long-lived manager. The worker owns the active GMainLoop and serializes
 * stop/release/start so camera and encoder resources are never double-owned. */
int stream_manager_init(const char *device, const char *rtmp_url);
int stream_manager_start(StreamType type);
int stream_manager_switch(const char *mode, int *changed);
void stream_manager_stop(void);
int stream_manager_is_running(void);
const char *stream_manager_state_name(void);
void stream_manager_notify_loop_ready(GMainLoop *loop);

/**
 * @brief 获取当前推流类型
 */
StreamType get_current_stream_type(void);

#ifdef __cplusplus
}
#endif

#endif // STREAM_MANAGER_H
