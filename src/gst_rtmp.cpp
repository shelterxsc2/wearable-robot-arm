/**
 * gst_rtmp.cpp - RTMP 云端推流 (ELF2 RK3588)
 * 基于 identity handoff 架构（和备份版本一致）
 * Pipeline: v4l2src(YUY2) → identity → RGA(YUYV→NV12) → process_frame → appsrc → mpph264enc → flvmux → rtmpsink
 */
#include "gst_rtmp.h"
#include "rga_npu.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <signal.h>
#include <time.h>
#include <gst/app/gstappsrc.h>

static GMainLoop *g_loop = NULL;
static volatile guint64 g_frame_count = 0;

guint64 get_rtmp_frame_count(void) {
    return g_frame_count;
}

static gboolean bus_message_cb(GstBus *bus, GstMessage *message, gpointer user_data) {
    (void)bus; (void)user_data;
    switch (GST_MESSAGE_TYPE(message)) {
        case GST_MESSAGE_ERROR: {
            GError *err = NULL;
            gchar *dbg = NULL;
            gst_message_parse_error(message, &err, &dbg);
            g_printerr("[GStreamer] ERROR: %s\n", err->message);
            g_error_free(err);
            g_free(dbg);
            if (g_loop) g_main_loop_quit(g_loop);
            break;
        }
        case GST_MESSAGE_WARNING: {
            GError *err = NULL;
            gchar *dbg = NULL;
            gst_message_parse_warning(message, &err, &dbg);
            g_printerr("[GStreamer] WARNING: %s\n", err->message);
            g_error_free(err);
            g_free(dbg);
            break;
        }
        case GST_MESSAGE_EOS:
            g_print("[GStreamer] End of stream\n");
            break;
        default:
            break;
    }
    return TRUE;
}

/* ---------- identity handoff: 抓帧 → RGA YUYV→NV12 → AI 绘制 → 打时间戳 → 推给 appsrc ---------- */
static inline uint64_t get_us() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000;
}

static void identity_handoff(GstElement *identity, GstBuffer *buffer, gpointer user_data) {
    (void)identity;
    GstElement *appsrc = (GstElement *)user_data;

    g_frame_count++;

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t start_us = ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000;
    set_frame_start_time_us(start_us);

    uint64_t t0 = get_us();

    /* 固定 1920x1080，输入为 YUY2，需 RGA 硬件转 NV12 */
    const int width = 1920;
    const int height = 1080;
    gsize yuyv_size = (gsize)width * height * 2;
    gsize nv12_size = (gsize)width * height * 3 / 2;

    GstBuffer *out_buffer = gst_buffer_new_allocate(NULL, nv12_size, NULL);
    if (!out_buffer) {
        fprintf(stderr, "[GStreamer] Failed to allocate NV12 output buffer\n");
        return;
    }

    GstMapInfo in_map, out_map;
    if (gst_buffer_map(buffer, &in_map, GST_MAP_READ) &&
        gst_buffer_map(out_buffer, &out_map, GST_MAP_WRITE)) {

        if (in_map.size < yuyv_size) {
            fprintf(stderr, "[GStreamer] YUY2 buffer too small: %zu < %zu\n", in_map.size, yuyv_size);
            memset(out_map.data, 0, nv12_size);
        } else {
            if (convert_yuyv_to_nv12((uint8_t*)in_map.data, (uint8_t*)out_map.data, width, height) != 0) {
                fprintf(stderr, "[GStreamer] RGA YUYV->NV12 failed, fallback to zero\n");
                memset(out_map.data, 0, nv12_size);
            }
        }
        uint64_t t1 = get_us();
        report_gst_getframe_us(t1 - t0);

        process_frame((uint8_t*)out_map.data, width, height);

        gst_buffer_unmap(buffer, &in_map);
        gst_buffer_unmap(out_buffer, &out_map);
    }

    /* 用系统时钟 running time 打时间戳 */
    static GstClockTime base_time = GST_CLOCK_TIME_NONE;
    GstClock *clock = gst_system_clock_obtain();
    GstClockTime now = gst_clock_get_time(clock);
    if (base_time == GST_CLOCK_TIME_NONE) base_time = now;
    GST_BUFFER_PTS(out_buffer) = now - base_time;
    GST_BUFFER_DURATION(out_buffer) = GST_SECOND / 30;
    gst_object_unref(clock);

    uint64_t t2 = get_us();
    GstFlowReturn push_ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc), out_buffer);
    uint64_t t3 = get_us();
    report_gst_encode_us(t3 - t2);
    if (push_ret != GST_FLOW_OK) {
        gst_buffer_unref(out_buffer);
    }
}

int start_rtmp_stream(const char *device, const char *rtmp_url, GMainLoop **loop_ptr) {
    gst_init(NULL, NULL);

    gchar *pipeline_str = g_strdup_printf(
        "v4l2src device=%s io-mode=auto "
        "! video/x-raw,format=YUY2,width=1920,height=1080,framerate=30/1 "
        "! identity name=myid ! fakesink sync=false "
        /* 视频编码 + 推流分支 */
        "appsrc name=mysrc caps=video/x-raw,format=NV12,width=1920,height=1080,framerate=30/1 "
        "! queue max-size-buffers=1 leaky=downstream "
        "! mpph264enc bps=4000000 bps-max=8000000 rc-mode=vbr gop=15 profile=high "
        "! h264parse config-interval=1 "
        "! flvmux streamable=true "
        "! queue leaky=downstream max-size-buffers=5 max-size-time=0 "
        "! rtmpsink sync=false location=%s",
        device, rtmp_url);

    g_print("[RTMP] Starting stream to: %s\n", rtmp_url);
    g_print("[RTMP] Pipeline: YUY2 → RGA → NV12 → AI → H264 → FLV → RTMP\n");

    GError *error = NULL;
    GstElement *pipeline = gst_parse_launch(pipeline_str, &error);
    g_free(pipeline_str);

    if (!pipeline) {
        g_printerr("[RTMP] Failed to create pipeline: %s\n", error ? error->message : "unknown");
        if (error) g_error_free(error);
        return -1;
    }
    if (error) {
        g_printerr("[RTMP] Parse warning: %s\n", error->message);
        g_error_free(error);
    }

    GstElement *identity = gst_bin_get_by_name(GST_BIN(pipeline), "myid");
    GstElement *appsrc  = gst_bin_get_by_name(GST_BIN(pipeline), "mysrc");

    if (!identity || !appsrc) {
        g_printerr("[RTMP] Cannot get identity or appsrc from pipeline\n");
        if (identity) g_object_unref(identity);
        if (appsrc)  g_object_unref(appsrc);
        gst_object_unref(pipeline);
        return -1;
    }

    g_object_set(identity, "signal-handoffs", TRUE, NULL);
    g_signal_connect(identity, "handoff", G_CALLBACK(identity_handoff), appsrc);
    g_object_unref(identity);

    g_object_set(appsrc,
                 "is-live", TRUE,
                 "do-timestamp", TRUE,
                 "stream-type", 0,
                 "format", GST_FORMAT_TIME,
                 NULL);
    g_object_unref(appsrc);

    GstBus *bus = gst_element_get_bus(pipeline);
    if (bus) {
        gst_bus_add_watch(bus, bus_message_cb, NULL);
        g_object_unref(bus);
    }

    GstStateChangeReturn ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        g_printerr("[RTMP] Failed to start pipeline (RTMP server unreachable?)\n");
        gst_object_unref(pipeline);
        return -1;
    }

    g_print("\n[RTMP] ==============================================\n");
    g_print("[RTMP] Stream pushing to: %s\n", rtmp_url);
    g_print("[RTMP] Features: YOLOv8-Pose20 FP + RGA hardware draw\n");
    g_print("[RTMP] Press Ctrl+C to stop\n");
    g_print("[RTMP] ==============================================\n\n");

    GMainLoop *loop = g_main_loop_new(NULL, FALSE);
    *loop_ptr = loop;
    g_loop = loop;
    g_main_loop_run(loop);

    g_print("[RTMP] Stopping...\n");
    gst_element_send_event(pipeline, gst_event_new_flush_start());
    gst_element_send_event(pipeline, gst_event_new_flush_stop(TRUE));

    g_thread_new("rtmp-stop", [](gpointer data) -> gpointer {
        GstElement *p = GST_ELEMENT(data);
        gst_element_set_state(p, GST_STATE_NULL);
        gst_object_unref(p);
        return NULL;
    }, pipeline);

    g_loop = NULL;
    g_main_loop_unref(loop);
    return 0;
}
