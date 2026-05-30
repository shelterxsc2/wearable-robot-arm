/**
 * gst_rtsp.cpp - YOLOv8-Pose推流优化版 (ELF2 RK3588)
 * 启用process_frame进行AI推理和绘制
 */
#include "gst_rtsp.h"
#include "rga_npu.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <signal.h>
#include <time.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>

static GMainLoop *loop = NULL;

// 帧计数（用于统计）
static volatile guint64 g_frames_in = 0;
static volatile guint64 g_frames_out = 0;

void set_frame_counters(volatile guint64 *in, volatile guint64 *out) {
    // 不使用，改在process_frame内部统计
}

/* ---------- Pipeline 各环节耗时统计 ---------- */
#define STAT_WINDOW 30

static inline uint64_t get_us() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000;
}

static struct {
    uint64_t pull_us;      // gst_app_sink_pull_sample
    uint64_t setup_us;     // caps解析 + buffer分配 + map
    uint64_t memcpy_us;    // 数据复制/修复
    uint64_t ai_us;        // process_frame
    uint64_t push_us;      // gst_app_src_push_buffer
    uint64_t total_us;     // 进入new_sample_cb 到 push完成
    uint64_t interval_us;  // 帧间隔
} g_stat[STAT_WINDOW];

static int g_stat_idx = 0;
static int g_stat_count = 0;
static uint64_t g_last_enter_us = 0;

static void print_pipe_stats() {
    double sum_pull = 0, sum_setup = 0, sum_memcpy = 0;
    double sum_ai = 0, sum_push = 0, sum_total = 0, sum_interval = 0;
    for (int i = 0; i < STAT_WINDOW; i++) {
        sum_pull     += g_stat[i].pull_us;
        sum_setup    += g_stat[i].setup_us;
        sum_memcpy   += g_stat[i].memcpy_us;
        sum_ai       += g_stat[i].ai_us;
        sum_push     += g_stat[i].push_us;
        sum_total    += g_stat[i].total_us;
        sum_interval += g_stat[i].interval_us;
    }
    double n = STAT_WINDOW;
    printf("\n[PIPE-STAT] ======== Last %d frames avg ========\n", STAT_WINDOW);
    printf("[PIPE-STAT] pull_sample : %7.2f ms (v4l2src->mppjpegdec->appsink)\n", sum_pull    / n / 1000.0);
    printf("[PIPE-STAT] setup       : %7.2f ms (caps+alloc+map)\n",           sum_setup   / n / 1000.0);
    printf("[PIPE-STAT] memcpy      : %7.2f ms (plane copy)\n",               sum_memcpy  / n / 1000.0);
    printf("[PIPE-STAT] AI process  : %7.2f ms (RGA+NPU+draw)\n",             sum_ai      / n / 1000.0);
    printf("[PIPE-STAT] push_buffer : %7.2f ms (appsrc->queue)\n",            sum_push    / n / 1000.0);
    printf("[PIPE-STAT] total       : %7.2f ms (enter->push done)\n",         sum_total   / n / 1000.0);
    printf("[PIPE-STAT] interval    : %7.2f ms => %.1f fps\n",                sum_interval/ n / 1000.0,
                                                                       1000000.0 / (sum_interval / n));
    printf("[PIPE-STAT] ===================================\n\n");
}

static gboolean bus_message_cb(GstBus *bus, GstMessage *message, gpointer user_data) {
    switch (GST_MESSAGE_TYPE(message)) {
        case GST_MESSAGE_ERROR: {
            GError *err = NULL;
            gchar *dbg = NULL;
            gst_message_parse_error(message, &err, &dbg);
            g_printerr("[GStreamer] ERROR: %s\n", err->message);
            g_error_free(err);
            g_free(dbg);
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

static guint64 g_frame_count = 0;
static time_t g_last_print_time = 0;

static GstFlowReturn new_sample_cb(GstElement *sink, gpointer data) {
    GstElement *appsrc = (GstElement *)data;
    GstSample *sample = NULL;
    GstBuffer *buffer = NULL;
    GstMapInfo info = {0};
    GstCaps *caps = NULL;
    GstStructure *structure = NULL;
    int width = 0, height = 0;
    const char *format = NULL;

    uint64_t t_enter = get_us();
    uint64_t interval_us = (g_last_enter_us > 0) ? (t_enter - g_last_enter_us) : 0;
    g_last_enter_us = t_enter;

    static int sample_count = 0;
    sample_count++;
    if (sample_count <= 5 || sample_count % 30 == 0) {
        printf("[GST-DEBUG] new_sample_cb called, count=%d\n", sample_count);
    }

    uint64_t t0 = get_us();
    sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
    uint64_t t1 = get_us();
    if (!sample) {
        printf("[GST-DEBUG] pull_sample returned NULL\n");
        return GST_FLOW_OK;
    }

    /* Record frame receive time for latency measurement */
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t start_us = ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000;
    set_frame_start_time_us(start_us);

    buffer = gst_sample_get_buffer(sample);
    if (!buffer) {
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }

    // 获取caps信息
    caps = gst_sample_get_caps(sample);
    if (!caps) {
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }
    
    structure = gst_caps_get_structure(caps, 0);
    if (!gst_structure_get_int(structure, "width", &width) ||
        !gst_structure_get_int(structure, "height", &height)) {
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }
    
    format = gst_structure_get_string(structure, "format");
    if (!format) {
        fprintf(stderr, "[GStreamer] No format in caps\n");
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }
    if (strcmp(format, "NV12") != 0) {
        fprintf(stderr, "[GStreamer] Warning: Expected NV12, got %s, attempting anyway\n", format);
    }

    g_frames_in++;

    gsize nv12_size = (gsize)width * height * 3 / 2;

    // mppjpegdec 输出可能是只读 dmabuf，分配可写 buffer 做绘制
    GstBuffer *out_buffer = gst_buffer_new_allocate(NULL, nv12_size, NULL);
    if (!out_buffer) {
        fprintf(stderr, "[GStreamer] Failed to allocate output buffer\n");
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }

    // 映射输入 buffer（只读）
    GstMapInfo in_info = {0};
    if (!gst_buffer_map(buffer, &in_info, GST_MAP_READ)) {
        gst_buffer_unref(out_buffer);
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }

    // 映射输出 buffer（可写）
    GstMapInfo out_info = {0};
    if (!gst_buffer_map(out_buffer, &out_info, GST_MAP_WRITE)) {
        gst_buffer_unmap(buffer, &in_info);
        gst_buffer_unref(out_buffer);
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }

    uint64_t t2 = get_us();

    // MPP 解码器输出高度对齐到 16 的倍数（如 1080 -> 1088）
    // 实际布局: Y(1920 x align_h) + UV(1920 x align_h/2)
    // 需要紧凑布局: Y(1920 x height) + UV(1920 x height/2)
    if (in_info.size > nv12_size) {
        int align_h = (int)(in_info.size / (width * 3 / 2));
        // Y 平面：前 height 行直接复制（连续）
        memcpy(out_info.data, in_info.data, width * height);
        // UV 平面：从 Y padding 之后复制（连续）
        memcpy(out_info.data + width * height,
               in_info.data + width * align_h,
               width * height / 2);
    } else {
        // 无 padding，整块复制
        memcpy(out_info.data, in_info.data, nv12_size);
    }

    uint64_t t3 = get_us();

    gst_buffer_unmap(buffer, &in_info);

    // 在可写 buffer 上做 AI 绘制
    process_frame((uint8_t*)out_info.data, width, height);
    g_frames_out++;

    uint64_t t4 = get_us();

    gst_buffer_unmap(out_buffer, &out_info);

    // 复制时间戳
    gst_buffer_copy_into(out_buffer, buffer, GST_BUFFER_COPY_TIMESTAMPS, 0, -1);

    gst_sample_unref(sample);

    // 推送新 buffer（所有权转移给 appsrc）
    uint64_t t5 = get_us();
    GstFlowReturn push_ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc), out_buffer);
    uint64_t t6 = get_us();
    if (push_ret != GST_FLOW_OK) {
        g_frames_out--;
        gst_buffer_unref(out_buffer);
    }

    // 记录统计
    int idx = g_stat_idx;
    g_stat[idx].pull_us     = t1 - t0;
    g_stat[idx].setup_us    = t2 - t1;
    g_stat[idx].memcpy_us   = t3 - t2;
    g_stat[idx].ai_us       = t4 - t3;
    g_stat[idx].push_us     = t6 - t5;
    g_stat[idx].total_us    = t6 - t_enter;
    g_stat[idx].interval_us = interval_us;
    g_stat_idx = (g_stat_idx + 1) % STAT_WINDOW;

    // 隐藏 PIPE-STAT 输出：设为 1 恢复打印
    #define ENABLE_PIPE_STAT 0
    #if ENABLE_PIPE_STAT
    if (++g_stat_count % STAT_WINDOW == 0) {
        print_pipe_stats();
    }
    #endif

    return GST_FLOW_OK;
}

static void client_connected_cb(GstRTSPServer *server, GstRTSPClient *client, gpointer user_data) {
    // g_print("[RTSP] New client connected\n");
    (void)server; (void)client; (void)user_data;
}

static void media_configure_cb(GstRTSPMediaFactory *factory, GstRTSPMedia *media, gpointer user_data) {
    GstElement *element = gst_rtsp_media_get_element(media);
    if (!element) {
        g_printerr("[RTSP] Cannot get media element\n");
        return;
    }

    GstElement *appsink = gst_bin_get_by_name(GST_BIN(element), "mysink");
    GstElement *appsrc = gst_bin_get_by_name(GST_BIN(element), "mysrc");

    if (appsink && appsrc) {
        g_object_set(appsrc,
                     "max-bytes", (guint64)(1920 * 1080 * 2 * 2),
                     "is-live", TRUE,
                     "do-timestamp", TRUE,
                     "stream-type", 0,   /* GST_APP_STREAM_TYPE_STREAM */
                     "format", GST_FORMAT_TIME,
                     NULL);

        g_signal_connect(appsink, "new-sample", G_CALLBACK(new_sample_cb), appsrc);
        g_object_unref(appsink);
        g_object_unref(appsrc);
        g_print("[RTSP] Media configured, AI processing enabled\n");
    } else {
        g_printerr("[RTSP] Cannot get appsink or appsrc\n");
    }

    GstBus *bus = gst_element_get_bus(element);
    if (bus) {
        gst_bus_add_watch(bus, bus_message_cb, NULL);
        g_object_unref(bus);
    }
    gst_object_unref(element);
}

int start_rtsp_server(const char *device, GMainLoop **loop_ptr) {
    gst_init(NULL, NULL);
    
    // USB Camera3 pipeline: YUYV -> appsink, AI draws on NV12, appsrc feeds encoder
    gchar *media_launch = g_strdup_printf(
        "( "
        "v4l2src device=%s min-buffers=2 io-mode=auto "
        "! image/jpeg,width=1920,height=1080,framerate=30/1 "
        "! mppjpegdec "
        "! video/x-raw,format=NV12,width=1920,height=1080,framerate=30/1 "
        "! tee name=t "
        "t. ! queue max-size-buffers=1 leaky=downstream ! fakesink "
        "t. ! queue max-size-buffers=1 leaky=downstream "
        "! appsink name=mysink emit-signals=true sync=false max-buffers=1 drop=true "
        "appsrc name=mysrc caps=video/x-raw,format=NV12,width=1920,height=1080,framerate=30/1 "
        "! queue max-size-buffers=1 leaky=downstream "
        "! mpph264enc bps=4000000 bps-max=8000000 rc-mode=vbr gop=30 profile=high "
        "! h264parse "
        "! rtph264pay name=pay0 pt=96 config-interval=1 "
        ")",
        device);

    g_print("[RTSP] Starting server...\n");
    g_print("[RTSP] Pipeline configured for 1080p30 + AI overlay\n");

    GstRTSPServer *server = gst_rtsp_server_new();
    if (!server) {
        g_printerr("[RTSP] Cannot create server\n");
        g_free(media_launch);
        return -1;
    }

    GstRTSPMountPoints *mounts = gst_rtsp_server_get_mount_points(server);
    GstRTSPMediaFactory *factory = gst_rtsp_media_factory_new();
    
    gst_rtsp_media_factory_set_launch(factory, media_launch);
    g_free(media_launch);
    
    gst_rtsp_media_factory_set_shared(factory, TRUE);
    gst_rtsp_media_factory_set_latency(factory, 100);
    gst_rtsp_media_factory_set_transport_mode(factory, GST_RTSP_TRANSPORT_MODE_PLAY);
    
    g_signal_connect(factory, "media-configure", G_CALLBACK(media_configure_cb), NULL);
    gst_rtsp_mount_points_add_factory(mounts, "/stream", factory);
    g_object_unref(mounts);
    
    // 监听客户端连接事件
    g_signal_connect(server, "client-connected", G_CALLBACK(client_connected_cb), NULL);

    if (gst_rtsp_server_attach(server, NULL) == 0) {
        g_printerr("[RTSP] Cannot attach server\n");
        g_object_unref(server);
        return -1;
    }

    guint port = gst_rtsp_server_get_bound_port(server);
    g_print("\n[RTSP] ==============================================\n");
    g_print("[RTSP] Server running on port %u\n", port);
    g_print("[RTSP] Stream URL: rtsp://<IP>:%u/stream\n", port);
    g_print("[RTSP] Features: YOLOv8-Pose20 FP + RGA hardware draw\n");
    g_print("[RTSP] ==============================================\n\n");

    GMainLoop *loop = g_main_loop_new(NULL, FALSE);
    *loop_ptr = loop;
    g_main_loop_run(loop);

    g_main_loop_unref(loop);
    g_object_unref(server);
    return 0;
}
