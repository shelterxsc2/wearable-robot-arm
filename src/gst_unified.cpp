#include "gst_unified.h"
#include "rga_npu.h"
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>
#include <time.h>

namespace {
std::mutex state_mutex;
GstElement *pipeline = NULL, *encoded_tee = NULL;
GstElement *rtmp_bin = NULL;
GstPad *rtmp_tee_pad = NULL;
GstRTSPServer *rtsp_server = NULL;
guint rtsp_attach_id = 0;
std::string rtmp_location;
StreamType active_type = STREAM_TYPE_RTSP;
std::mutex clients_mutex;
std::vector<GstElement *> rtsp_appsrcs;

struct PadBlock { std::mutex mutex; std::condition_variable cv; bool hit = false; };
GstPadProbeReturn block_pad(GstPad *, GstPadProbeInfo *, gpointer data) {
    PadBlock *wait = static_cast<PadBlock *>(data);
    { std::lock_guard<std::mutex> lock(wait->mutex); wait->hit = true; }
    wait->cv.notify_one();
    return GST_PAD_PROBE_OK;
}

uint64_t mono_us() {
    timespec ts{}; clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000;
}

void raw_handoff(GstElement *, GstBuffer *buffer, gpointer data) {
    GstElement *appsrc = GST_ELEMENT(data);
    const int w = 1920, h = 1080;
    const gsize size = (gsize)w * h * 3 / 2;
    GstBuffer *out = gst_buffer_new_allocate(NULL, size, NULL);
    if (!out) return;
    GstMapInfo in{}, dst{};
    uint64_t t0 = mono_us();
    set_frame_start_time_us(t0);
    if (!gst_buffer_map(buffer, &in, GST_MAP_READ) ||
        !gst_buffer_map(out, &dst, GST_MAP_WRITE)) {
        if (in.data) gst_buffer_unmap(buffer, &in);
        if (dst.data) gst_buffer_unmap(out, &dst);
        gst_buffer_unref(out); return;
    }
    if (in.size >= size) {
        if (in.size > size) {
            int ah = (int)(in.size / (w * 3 / 2));
            memcpy(dst.data, in.data, w * h);
            memcpy(dst.data + w * h, in.data + w * ah, w * h / 2);
        } else memcpy(dst.data, in.data, size);
    } else memset(dst.data, 0, size);
    report_gst_getframe_us(mono_us() - t0);
    process_frame(dst.data, w, h);
    gst_buffer_unmap(buffer, &in);
    gst_buffer_unmap(out, &dst);
    gst_buffer_copy_into(out, buffer, GST_BUFFER_COPY_TIMESTAMPS, 0, -1);
    uint64_t t1 = mono_us();
    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc), out);
    report_gst_encode_us(mono_us() - t1);
    if (ret != GST_FLOW_OK) gst_buffer_unref(out);
}

void media_unprepared(GstRTSPMedia *, gpointer data) {
    GstElement *src = GST_ELEMENT(data);
    std::lock_guard<std::mutex> lock(clients_mutex);
    auto it = std::find(rtsp_appsrcs.begin(), rtsp_appsrcs.end(), src);
    if (it != rtsp_appsrcs.end()) rtsp_appsrcs.erase(it);
    gst_object_unref(src);
}

void media_configure(GstRTSPMediaFactory *, GstRTSPMedia *media, gpointer) {
    GstElement *root = gst_rtsp_media_get_element(media);
    GstElement *src = root ? gst_bin_get_by_name_recurse_up(GST_BIN(root), "net_src") : NULL;
    if (src) {
        g_object_set(src, "is-live", TRUE, "format", GST_FORMAT_TIME,
                     "do-timestamp", TRUE, NULL);
        { std::lock_guard<std::mutex> lock(clients_mutex); rtsp_appsrcs.push_back(src); }
        g_signal_connect(media, "unprepared", G_CALLBACK(media_unprepared), src);
    }
    if (root) gst_object_unref(root);
}

GstFlowReturn encoded_sample(GstElement *sink, gpointer) {
    GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
    if (!sample) return GST_FLOW_OK;
    GstBuffer *buf = gst_sample_get_buffer(sample);
    std::vector<GstElement *> clients;
    {
        std::lock_guard<std::mutex> lock(clients_mutex);
        for (auto *src : rtsp_appsrcs) clients.push_back(GST_ELEMENT(gst_object_ref(src)));
    }
    for (auto *src : clients) {
        GstBuffer *copy = gst_buffer_copy(buf);
        GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(src), copy);
        if (ret != GST_FLOW_OK) gst_buffer_unref(copy);
        gst_object_unref(src);
    }
    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

GstRTSPFilterResult remove_client(GstRTSPServer *, GstRTSPClient *, gpointer) {
    return GST_RTSP_FILTER_REMOVE;
}

int enable_rtsp() {
    if (rtsp_server) return 0;
    rtsp_server = gst_rtsp_server_new();
    GstRTSPMountPoints *mounts = gst_rtsp_server_get_mount_points(rtsp_server);
    GstRTSPMediaFactory *factory = gst_rtsp_media_factory_new();
    gst_rtsp_media_factory_set_launch(factory,
        "( appsrc name=net_src caps=video/x-h264,stream-format=byte-stream,alignment=au "
        "! queue max-size-buffers=15 leaky=downstream ! h264parse "
        "! rtph264pay name=pay0 pt=96 config-interval=1 )");
    gst_rtsp_media_factory_set_shared(factory, TRUE);
    g_signal_connect(factory, "media-configure", G_CALLBACK(media_configure), NULL);
    gst_rtsp_mount_points_add_factory(mounts, "/stream", factory);
    gst_object_unref(mounts);
    rtsp_attach_id = gst_rtsp_server_attach(rtsp_server, NULL);
    if (!rtsp_attach_id) { gst_object_unref(rtsp_server); rtsp_server = NULL; return -1; }
    g_print("[Unified] RTSP output ready: rtsp://<IP>:8554/stream\n");
    return 0;
}

void disable_rtsp() {
    if (!rtsp_server) return;
    gst_rtsp_server_client_filter(rtsp_server, remove_client, NULL);
    if (rtsp_attach_id) g_source_remove(rtsp_attach_id);
    rtsp_attach_id = 0;
    gst_object_unref(rtsp_server); rtsp_server = NULL;
    /* media_unprepared owns removal/unref of each appsrc reference. */
}

int enable_rtmp() {
    if (rtmp_bin) return 0;
    gchar *desc = g_strdup_printf(
        "queue max-size-buffers=15 leaky=downstream ! h264parse config-interval=1 "
        "! flvmux streamable=true ! queue leaky=downstream max-size-buffers=5 "
        "! rtmpsink sync=false location=%s", rtmp_location.c_str());
    GError *err = NULL;
    GstElement *bin = gst_parse_bin_from_description(desc, TRUE, &err);
    g_free(desc);
    if (!bin) { if (err) g_error_free(err); return -1; }
    if (!gst_bin_add(GST_BIN(pipeline), bin)) { gst_object_unref(bin); return -1; }
    GstPad *sink = gst_element_get_static_pad(bin, "sink");
    GstPad *tee_pad = gst_element_request_pad_simple(encoded_tee, "src_%u");
    if (!sink || !tee_pad || gst_pad_link(tee_pad, sink) != GST_PAD_LINK_OK) {
        if (sink) gst_object_unref(sink);
        if (tee_pad) { gst_element_release_request_pad(encoded_tee, tee_pad); gst_object_unref(tee_pad); }
        gst_bin_remove(GST_BIN(pipeline), bin); return -1;
    }
    gst_object_unref(sink);
    if (!gst_element_sync_state_with_parent(bin)) {
        GstPad *failed_sink = gst_element_get_static_pad(bin, "sink");
        if (failed_sink) { gst_pad_unlink(tee_pad, failed_sink); gst_object_unref(failed_sink); }
        gst_element_release_request_pad(encoded_tee, tee_pad); gst_object_unref(tee_pad);
        gst_bin_remove(GST_BIN(pipeline), bin); return -1;
    }
    GstState parent_state = GST_STATE(pipeline);
    if (parent_state == GST_STATE_PLAYING) {
        GstStateChangeReturn ready = gst_element_get_state(bin, NULL, NULL, 5 * GST_SECOND);
        if (ready == GST_STATE_CHANGE_FAILURE || ready == GST_STATE_CHANGE_ASYNC) {
            gst_element_set_state(bin, GST_STATE_NULL);
            GstPad *failed_sink = gst_element_get_static_pad(bin, "sink");
            if (failed_sink) { gst_pad_unlink(tee_pad, failed_sink); gst_object_unref(failed_sink); }
            gst_element_release_request_pad(encoded_tee, tee_pad);
            gst_object_unref(tee_pad);
            gst_bin_remove(GST_BIN(pipeline), bin);
            return -1;
        }
    }
    rtmp_bin = bin; rtmp_tee_pad = tee_pad;
    g_print("[Unified] RTMP output enabled -> %s\n", rtmp_location.c_str());
    return 0;
}

void disable_rtmp() {
    if (!rtmp_bin) return;
    PadBlock wait;
    gulong probe = gst_pad_add_probe(rtmp_tee_pad, GST_PAD_PROBE_TYPE_BLOCK_DOWNSTREAM,
                                     block_pad, &wait, NULL);
    {
        std::unique_lock<std::mutex> lock(wait.mutex);
        wait.cv.wait_for(lock, std::chrono::milliseconds(500), [&] { return wait.hit; });
    }
    gst_element_set_state(rtmp_bin, GST_STATE_NULL);
    gst_element_get_state(rtmp_bin, NULL, NULL, 3 * GST_SECOND);
    GstPad *sink = gst_element_get_static_pad(rtmp_bin, "sink");
    if (sink) { gst_pad_unlink(rtmp_tee_pad, sink); gst_object_unref(sink); }
    if (probe) gst_pad_remove_probe(rtmp_tee_pad, probe);
    gst_element_release_request_pad(encoded_tee, rtmp_tee_pad);
    gst_object_unref(rtmp_tee_pad); rtmp_tee_pad = NULL;
    gst_bin_remove(GST_BIN(pipeline), rtmp_bin); rtmp_bin = NULL;
}

gboolean bus_cb(GstBus *, GstMessage *msg, gpointer data) {
    if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
        GError *e=NULL; gchar *d=NULL; gst_message_parse_error(msg,&e,&d);
        g_printerr("[Unified] ERROR: %s\n", e->message); g_error_free(e); g_free(d);
        g_main_loop_quit((GMainLoop *)data);
    }
    return TRUE;
}
}

int unified_stream_switch(StreamType target) {
    std::lock_guard<std::mutex> lock(state_mutex);
    if (!pipeline) return -1;
    if (target == active_type && (target == STREAM_TYPE_RTMP ? rtmp_bin != NULL : rtsp_server != NULL)) return 0;
    if (target == STREAM_TYPE_RTMP) {
        if (enable_rtmp() != 0) return -1;
        disable_rtsp();
    } else {
        if (enable_rtsp() != 0) return -1;
        disable_rtmp();
    }
    active_type = target;
    return 0;
}

int start_unified_stream(const char *device, const char *url, StreamType initial,
                         GMainLoop **loop_ptr) {
    gchar *desc = g_strdup_printf(
        "v4l2src device=%s io-mode=auto ! image/jpeg,width=1920,height=1080,framerate=30/1 "
        "! mppjpegdec ! video/x-raw,format=NV12,width=1920,height=1080 "
        "! videoflip video-direction=180 ! identity name=raw_id ! fakesink sync=false "
        "appsrc name=processed_src caps=video/x-raw,format=NV12,width=1920,height=1080,framerate=30/1 "
        "is-live=true format=time do-timestamp=true ! queue max-size-buffers=1 leaky=downstream "
        "! mpph264enc bps=4000000 bps-max=8000000 rc-mode=vbr gop=15 "
        "header-mode=each-idr profile=high "
        "! h264parse config-interval=1 "
        "! video/x-h264,stream-format=byte-stream,alignment=au ! tee name=encoded_tee "
        "encoded_tee. ! queue leaky=downstream max-size-buffers=2 ! fakesink sync=false "
        "encoded_tee. ! queue leaky=downstream max-size-buffers=2 "
        "! appsink name=encoded_sink emit-signals=true sync=false max-buffers=2 drop=true", device);
    GError *err=NULL;
    pipeline = gst_parse_launch(desc, &err); g_free(desc);
    if (!pipeline) { if(err)g_error_free(err); return -1; }
    rtmp_location = url; active_type = initial;
    GstElement *id=gst_bin_get_by_name(GST_BIN(pipeline),"raw_id");
    GstElement *src=gst_bin_get_by_name(GST_BIN(pipeline),"processed_src");
    encoded_tee=gst_bin_get_by_name(GST_BIN(pipeline),"encoded_tee");
    GstElement *sink=gst_bin_get_by_name(GST_BIN(pipeline),"encoded_sink");
    g_object_set(id,"signal-handoffs",TRUE,NULL);
    g_signal_connect(id,"handoff",G_CALLBACK(raw_handoff),src);
    g_signal_connect(sink,"new-sample",G_CALLBACK(encoded_sample),NULL);
    gst_object_unref(id); gst_object_unref(src); gst_object_unref(sink);
    if ((initial==STREAM_TYPE_RTMP ? enable_rtmp() : enable_rtsp()) != 0) return -1;
    GMainLoop *loop=g_main_loop_new(NULL,FALSE);
    GstBus *bus=gst_element_get_bus(pipeline); gst_bus_add_watch(bus,bus_cb,loop); gst_object_unref(bus);
    if (gst_element_set_state(pipeline,GST_STATE_PLAYING)==GST_STATE_CHANGE_FAILURE) return -1;
    *loop_ptr=loop; stream_manager_notify_loop_ready(loop); g_main_loop_run(loop);
    disable_rtmp(); disable_rtsp();
    gst_element_set_state(pipeline,GST_STATE_NULL); gst_object_unref(encoded_tee); encoded_tee=NULL;
    gst_object_unref(pipeline); pipeline=NULL; g_main_loop_unref(loop);
    return 0;
}
