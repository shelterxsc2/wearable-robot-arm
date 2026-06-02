/**
 * gst_rtmp.cpp - RTMP 云端推流 (ELF2 RK3588)
 * 基于 real 工程的 identity handoff 低延迟 pipeline
 * 适配 twice 工程的 USB 摄像头 YUY2 输入 + RGA 硬件转 NV12
 * 
 * 【备份版本】1920x1080 YUY2 采集，1080p30 RTMP 推流
 * 备份时间：2026-05-15
 * 用途：当摄像头支持 YUYV 1080p30（USB3.0 模式）时可恢复此版本
 */
#include "gst_rtmp.h"
#include "rga_npu.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <signal.h>
#include <time.h>
#include <gst/app/gstappsrc.h>

#define ALSA_AUDIO_DEVICE   "hw:3,0"

static GMainLoop *g_loop = NULL;
static volatile guint64 g_frame_count = 0;
static time_t g_last_print_time = 0;

guint64 get_frame_count(void) {
    return g_frame_count;
}

static gboolean bus_message_cb(GstBus *bus, GstMessage *message, gpointer user_data) {
    (void)bus; (void)user_data;
    switch (GST_MESSAGE_TYPE(message)) {
        case GST_MESSAGE_ERROR: {
            GError *err = NULL;
            gchar *dbg = NULL;
            GstObject *src = GST_MESSAGE_SRC(message);
            const gchar *name = src ? GST_OBJECT_NAME(src) : "unknown";
            gst_message_parse_error(message, &err, &dbg);
            g_printerr("[GStreamer] ERROR from %s: %s | %s\n", name, err->message, dbg ? dbg : "");
            g_error_free(err);
            g_free(dbg);
            if (name && (strstr(name, "rtmp") || strstr(name, "flv") || strstr(name, "queue"))) {
                g_printerr("[GStreamer] Network/sink error ignored, camera keeps running\n");
            } else {
                if (g_loop) g_main_loop_quit(g_loop);
            }
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
            g_print("[GStreamer] End of stream (sink disconnected?)\n");
            break;
        default:
            break;
    }
    return TRUE;
}

/* ---------- identity handoff: 抓帧 → YUY2→NV12 → AI 绘制 → 打时间戳 → 推给 appsrc ---------- */
static void identity_handoff(GstElement *identity, GstBuffer *buffer, gpointer user_data) {
    (void)identity;
    GstElement *appsrc = (GstElement *)user_data;

    g_frame_count++;
    time_t now_wall = time(NULL);
    if (now_wall - g_last_print_time >= 5) {
        g_print("[RTMP] Processed %llu frames, current FPS ~%.1f\n",
                (unsigned long long)g_frame_count, g_frame_count / 5.0);
        g_frame_count = 0;
        g_last_print_time = now_wall;
    }

    /* 记录帧接收时间，用于延迟测量 */
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t start_us = ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000;
    set_frame_start_time_us(start_us);

    /* 分配 NV12 buffer（USB 摄像头输入为 YUY2，需 RGA 硬件转换） */
    gsize nv12_size = 1920 * 1080 * 3 / 2;
    GstBuffer *out_buffer = gst_buffer_new_allocate(NULL, nv12_size, NULL);
    if (!out_buffer) {
        fprintf(stderr, "[GStreamer] Failed to allocate NV12 output buffer\n");
        return;
    }

    GstMapInfo in_map, out_map;
    if (gst_buffer_map(buffer, &in_map, GST_MAP_READ) &&
        gst_buffer_map(out_buffer, &out_map, GST_MAP_WRITE)) {

        if (convert_yuyv_to_nv12((uint8_t*)in_map.data, (uint8_t*)out_map.data, 1920, 1080) != 0) {
            fprintf(stderr, "[GStreamer] RGA YUYV->NV12 failed, fallback to zero\n");
            memset(out_map.data, 0, nv12_size);
        }
        process_frame((uint8_t*)out_map.data, 1920, 1080);

        gst_buffer_unmap(buffer, &in_map);
        gst_buffer_unmap(out_buffer, &out_map);
    }

    /* 用系统时钟 running time 打时间戳，比固定 33ms 更精确 */
    static GstClockTime base_time = GST_CLOCK_TIME_NONE;
    GstClock *clock = gst_system_clock_obtain();
    GstClockTime now = gst_clock_get_time(clock);
    if (base_time == GST_CLOCK_TIME_NONE) base_time = now;
    GST_BUFFER_PTS(out_buffer) = now - base_time;
    GST_BUFFER_DURATION(out_buffer) = GST_SECOND / 5;
    gst_object_unref(clock);

    /* 推送到 appsrc */
    GstFlowReturn push_ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc), out_buffer);
    if (push_ret != GST_FLOW_OK) {
        gst_buffer_unref(out_buffer);
    }
}

int start_rtmp_stream(const char *device, const char *rtmp_url, GMainLoop **loop_ptr) {
    gst_init(NULL, NULL);

    gchar *pipeline_str = g_strdup_printf(
        "v4l2src device=%s io-mode=mmap "
        "! video/x-raw,format=YUY2,width=1920,height=1080,framerate=5/1 "
        "! tee name=t "
        /* fakesink 分支：最小缓冲，只用来防止 v4l2src 阻塞 */
        "t. ! queue max-size-buffers=1 leaky=downstream ! fakesink "
        /* identity 抓帧分支：同样最小缓冲 */
        "t. ! queue max-size-buffers=1 leaky=downstream ! identity name=myid ! fakesink "
        /* 视频编码 + 推流分支 */
        "appsrc name=mysrc caps=video/x-raw,format=NV12,width=1920,height=1080,framerate=5/1 "
        "! queue max-size-buffers=1 leaky=downstream "
        /* GOP=15：0.5s 一个关键帧，首屏延迟更低；profile=high */
        "! mpph264enc bps=4000000 bps-max=8000000 rc-mode=vbr gop=15 profile=high "
        /* config-interval=1：每秒插入一次 SPS/PPS，防止中途花屏 */
        "! h264parse config-interval=1 "
        "! flvmux name=mux streamable=true "
        /* 音频分支：USB 无线麦克风 → AAC */
        "alsasrc device=" ALSA_AUDIO_DEVICE " "
        "! audioconvert ! audioresample ! audio/x-raw,rate=44100,channels=1 "
        "! queue max-size-buffers=10 leaky=downstream "
        "! voaacenc "
        "! aacparse "
        "! queue max-size-buffers=10 leaky=downstream "
        "! mux. "
        /* 网络缓冲也减到最小 */
        "mux. ! queue leaky=downstream max-size-buffers=5 max-size-time=0 "
        /* sync=false：rtmpsink 不等待 PTS，收到就发，降低延迟 */
        "! rtmpsink sync=false location=%s",
        device, rtmp_url);

    g_print("[RTMP] Starting stream to: %s\n", rtmp_url);
    g_print("[RTMP] Pipeline: 1080p30 YUY2->NV12(RGA) + AI overlay + H264 + AAC(audio) -> FLV -> RTMP (low-latency)\n");

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
                 "format", GST_FORMAT_TIME,
                 "block", FALSE,
                 "max-buffers", 1,
                 "leaky-type", GST_APP_LEAKY_TYPE_DOWNSTREAM,
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
    g_print("[RTMP] Features: YOLOv8-Pose20 + RGA hardware draw + USB mic audio (low-latency)\n");
    g_print("[RTMP] Press Ctrl+C to stop\n");
    g_print("[RTMP] ==============================================\n\n");

    *loop_ptr = g_main_loop_new(NULL, FALSE);
    g_loop = *loop_ptr;
    g_main_loop_run(*loop_ptr);

    g_print("[RTMP] Stopping...\n");
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    g_loop = NULL;

    g_main_loop_unref(*loop_ptr);
    return 0;
}
