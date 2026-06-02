#ifndef GST_RTMP_H
#define GST_RTMP_H

#include <gst/gst.h>

#ifdef __cplusplus
extern "C" {
#endif

int start_rtmp_stream(const char *device, const char *rtmp_url, GMainLoop **loop_ptr);

/* 获取已处理的帧计数（供 WebSocket 上报使用） */
guint64 get_frame_count(void);

#ifdef __cplusplus
}
#endif

#endif // GST_RTMP_H
