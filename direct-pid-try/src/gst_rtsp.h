#ifndef GST_RTSP_H
#define GST_RTSP_H

#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>

#ifdef __cplusplus
extern "C" {
#endif

int start_rtsp_server(const char *device, GMainLoop **loop_ptr);

#ifdef __cplusplus
}
#endif

#endif // GST_RTSP_H
