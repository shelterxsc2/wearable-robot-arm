#ifndef GST_RTMP_H
#define GST_RTMP_H

#include <gst/gst.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 启动 RTMP 推流（基于 MJPG 解码路径）
 * @param device    V4L2 设备路径，如 "/dev/video21"
 * @param rtmp_url  RTMP 推流地址，如 "rtmp://host:1935/live/key"
 * @param loop_ptr  输出 GMainLoop 指针
 * @return 0 成功，-1 失败
 */
int start_rtmp_stream(const char *device, const char *rtmp_url, GMainLoop **loop_ptr);
guint64 get_rtmp_frame_count(void);

#ifdef __cplusplus
}
#endif

#endif // GST_RTMP_H
