#ifndef GST_UNIFIED_H
#define GST_UNIFIED_H

#include <gst/gst.h>
#include "stream_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

int start_unified_stream(const char *device, const char *rtmp_url,
                         StreamType initial, GMainLoop **loop_ptr);
int unified_stream_switch(StreamType target);

#ifdef __cplusplus
}
#endif
#endif
