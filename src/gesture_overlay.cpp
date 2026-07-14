#include "gesture_overlay.h"
#include "arm_power_control.h"
#include "control_router.h"
#include "gesture_control.h"

#include <cstddef>
#include <im2d.h>
#include <opencv2/opencv.hpp>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <errno.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {

constexpr int INPUT_W = 224;
constexpr int INPUT_H = 224;
constexpr int INFERENCE_INTERVAL = 3;
constexpr int MAX_ROI = 512;
constexpr float CONFIRM_SCORE = 0.70f;
constexpr int CONFIRM_COUNT = 3;
constexpr uint64_t RESULT_FRESH_US = 900000;
constexpr uint64_t STABLE_HOLD_US = 1500000;
constexpr const char *SOCKET_PATH = "/tmp/hand_pipeline.sock";

struct Result {
    bool connected = false;
    bool valid = false;
    float bbox[4] = {0};
    float palm_score = 0;
    char gesture[48] = "None";
    float gesture_score = 0;
    float landmarks[21 * 3] = {0};
    char stable[48] = "";
    uint64_t stable_until_us = 0;
    int confirm_count = 0;
    float inference_ms = 0;
    uint64_t timestamp_us = 0;
    GestureHandRoi roi{};
};

pthread_t g_thread{};
pthread_mutex_t g_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t g_cond = PTHREAD_COND_INITIALIZER;
std::atomic<int> g_running{0};
bool g_thread_started = false;
constexpr int PENDING_CAPACITY = 2;
int g_pending_head = 0;
int g_pending_tail = 0;
int g_pending_count = 0;
GestureHandRoi g_rois[2]{};
int g_roi_count = 0;
GestureHandRoi g_pending_rois[PENDING_CAPACITY]{};
unsigned int g_pending_generations[PENDING_CAPACITY]{};
uint8_t *g_small_nv12 = nullptr;
uint8_t *g_crop_nv12 = nullptr;
uint8_t *g_pending_rgb[PENDING_CAPACITY] = {nullptr, nullptr};
uint8_t *g_work_rgb = nullptr;
unsigned int g_frame_counter = 0;
Result g_results[2];
GestureControlState g_control_state{};

uint64_t now_us()
{
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

bool send_all(int fd, const void *data, size_t size)
{
    const uint8_t *p = static_cast<const uint8_t *>(data);
    while (size > 0) {
        ssize_t n = send(fd, p, size, MSG_NOSIGNAL);
        if (n <= 0) return false;
        p += n;
        size -= (size_t)n;
    }
    return true;
}

bool recv_all(int fd, void *data, size_t size)
{
    uint8_t *p = static_cast<uint8_t *>(data);
    while (size > 0 && g_running.load()) {
        ssize_t n = recv(fd, p, size, 0);
        if (n <= 0) return false;
        p += n;
        size -= (size_t)n;
    }
    return size == 0;
}

int connect_sidecar()
{
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    timeval timeout{0, 500000};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, SOCKET_PATH, sizeof(addr.sun_path) - 1);
    if (connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
        close(fd);
        return -1;
    }
    return fd;
}

bool infer(int fd, Result &out)
{
    uint32_t header[3] = {INPUT_W, INPUT_H, 5};
    if (!send_all(fd, header, sizeof(header)) ||
        !send_all(fd, g_work_rgb, (size_t)INPUT_W * INPUT_H * 3)) return false;

    uint32_t status = 0;
    if (!recv_all(fd, &status, sizeof(status))) return false;
    out.valid = status != 0;
    if (!out.valid) return true;

    float bbox_score[5]{};
    if (!recv_all(fd, bbox_score, sizeof(bbox_score))) return false;
    memcpy(out.bbox, bbox_score, sizeof(out.bbox));
    out.palm_score = bbox_score[4];

    uint32_t label_len = 0;
    if (!recv_all(fd, &label_len, sizeof(label_len))) return false;
    uint32_t wire_len = label_len;
    char label[256]{};
    if (wire_len >= sizeof(label)) return false;
    if (wire_len && !recv_all(fd, label, wire_len)) return false;
    label[wire_len] = '\0';
    strncpy(out.gesture, wire_len ? label : "None", sizeof(out.gesture) - 1);
    if (!recv_all(fd, &out.gesture_score, sizeof(float))) return false;

    uint32_t count = 0;
    if (!recv_all(fd, &count, sizeof(count)) || count > 21) return false;
    memset(out.landmarks, 0, sizeof(out.landmarks));
    return count == 0 || recv_all(fd, out.landmarks, (size_t)count * 3 * sizeof(float));
}

void update_confirmation(Result &next, const Result &previous)
{
    bool candidate = next.valid && strcmp(next.gesture, "None") != 0 &&
                     next.gesture_score >= CONFIRM_SCORE;
    if (candidate && strcmp(next.gesture, previous.gesture) == 0) {
        next.confirm_count = previous.confirm_count + 1;
    } else {
        next.confirm_count = candidate ? 1 : 0;
    }
    if (candidate && next.confirm_count >= CONFIRM_COUNT) {
        strncpy(next.stable, next.gesture, sizeof(next.stable) - 1);
        next.stable_until_us = next.timestamp_us + STABLE_HOLD_US;
    } else if (previous.stable[0] && previous.stable_until_us > next.timestamp_us) {
        strncpy(next.stable, previous.stable, sizeof(next.stable) - 1);
        next.stable_until_us = previous.stable_until_us;
    }
}

void apply_control_action(GestureControlAction action,
                          unsigned int expected_generation)
{
    int ret = 0;
    switch (action) {
        case GESTURE_ACTION_MODE_FACE:
            ret = control_request_mode_if_generation(
                MODE_FACE, CONTROL_SOURCE_GESTURE, expected_generation);
            break;
        case GESTURE_ACTION_MODE_INTRO:
            ret = control_request_mode_if_generation(
                MODE_INTRO, CONTROL_SOURCE_GESTURE, expected_generation);
            break;
        case GESTURE_ACTION_MODE_INTERVIEW:
            ret = control_request_mode_if_generation(
                MODE_INTERVIEW, CONTROL_SOURCE_GESTURE, expected_generation);
            break;
        case GESTURE_ACTION_PROFILE_NEAR:
            ret = control_request_profile_if_generation(
                0, CONTROL_SOURCE_GESTURE, expected_generation);
            break;
        case GESTURE_ACTION_PROFILE_FAR:
            ret = control_request_profile_if_generation(
                1, CONTROL_SOURCE_GESTURE, expected_generation);
            break;
        case GESTURE_ACTION_POWER_CONFIRM:
            if (arm_power_confirm_gesture() == 0) {
                ret = 0;
            } else {
                ret = control_request_mode_if_generation(
                    MODE_FACE, CONTROL_SOURCE_GESTURE, expected_generation);
            }
            break;
        case GESTURE_ACTION_NONE:
            return;
    }
    printf("[GestureCmd] %s result=%s\n", gesture_control_action_name(action),
           ret == 0 ? "applied" : (ret == 1 ? "stale" : "rejected"));
}

void *worker(void *)
{
    int fd = -1;
    uint64_t last_connect_try = 0;
    uint64_t last_connect_log = 0;
    unsigned int inference_count = 0;
    while (g_running.load()) {
        pthread_mutex_lock(&g_mutex);
        while (g_pending_count == 0 && g_running.load()) pthread_cond_wait(&g_cond, &g_mutex);
        if (!g_running.load()) { pthread_mutex_unlock(&g_mutex); break; }
        int slot = g_pending_head;
        memcpy(g_work_rgb, g_pending_rgb[slot], (size_t)INPUT_W * INPUT_H * 3);
        GestureHandRoi work_roi = g_pending_rois[slot];
        unsigned int work_generation = g_pending_generations[slot];
        g_pending_head = (g_pending_head + 1) % PENDING_CAPACITY;
        --g_pending_count;
        pthread_mutex_unlock(&g_mutex);

        if (fd < 0) {
            uint64_t now = now_us();
            if (now - last_connect_try < 1000000) continue;
            last_connect_try = now;
            fd = connect_sidecar();
            pthread_mutex_lock(&g_mutex);
            g_results[0].connected = fd >= 0;
            g_results[1].connected = fd >= 0;
            pthread_mutex_unlock(&g_mutex);
            if (fd < 0) {
                if (now - last_connect_log >= 5000000) {
                    printf("[Gesture] waiting for sidecar socket %s\n", SOCKET_PATH);
                    last_connect_log = now;
                }
                continue;
            }
            printf("[Gesture] connected to sidecar %s\n", SOCKET_PATH);
        }

        Result next;
        next.connected = true;
        next.roi = work_roi;
        uint64_t start = now_us();
        if (!infer(fd, next)) {
            close(fd);
            fd = -1;
            pthread_mutex_lock(&g_mutex);
            g_results[0].connected = false;
            g_results[1].connected = false;
            pthread_mutex_unlock(&g_mutex);
            continue;
        }
        next.timestamp_us = now_us();
        next.inference_ms = (float)(next.timestamp_us - start) / 1000.0f;
        GestureControlAction action = GESTURE_ACTION_NONE;
        pthread_mutex_lock(&g_mutex);
        int side = work_roi.side == 1 ? 1 : 0;
        update_confirmation(next, g_results[side]);
        g_results[side] = next;
        unsigned int current_generation = control_get_mode_generation();
        if (work_generation == current_generation) {
            action = gesture_control_update(
                &g_control_state, side, next.valid ? next.gesture : "None",
                next.valid ? next.gesture_score : 0.0f, get_pose_mode(),
                current_generation, next.timestamp_us);
        } else {
            // A mode change happened while this ROI was in flight. Never let a
            // stale result immediately undo the newer command.
            gesture_control_update(&g_control_state, side, "None", 0.0f,
                                   get_pose_mode(), current_generation,
                                   next.timestamp_us);
        }
        pthread_mutex_unlock(&g_mutex);
        apply_control_action(action, work_generation);
        ++inference_count;
        if (inference_count == 1 || inference_count % 20 == 0) {
            printf("[Gesture] #%u side=%s hand=%d label=%s score=%.2f stable=%s infer=%.1fms\n",
                   inference_count, work_roi.side ? "Right" : "Left", next.valid ? 1 : 0,
                   next.valid ? next.gesture : "NoHand", next.gesture_score,
                   next.stable[0] ? next.stable : "-", next.inference_ms);
        }
    }
    if (fd >= 0) close(fd);
    return nullptr;
}

}  // namespace

int gesture_overlay_init(void)
{
    if (g_thread_started) return 0;
    if (posix_memalign((void **)&g_small_nv12, 64, INPUT_W * INPUT_H * 3 / 2) != 0 ||
        posix_memalign((void **)&g_crop_nv12, 64, MAX_ROI * MAX_ROI * 3 / 2) != 0 ||
        posix_memalign((void **)&g_pending_rgb[0], 64, INPUT_W * INPUT_H * 3) != 0 ||
        posix_memalign((void **)&g_pending_rgb[1], 64, INPUT_W * INPUT_H * 3) != 0 ||
        posix_memalign((void **)&g_work_rgb, 64, INPUT_W * INPUT_H * 3) != 0) {
        gesture_overlay_shutdown();
        return -1;
    }
    g_running.store(1);
    gesture_control_reset(&g_control_state, control_get_mode_generation());
    if (pthread_create(&g_thread, nullptr, worker, nullptr) != 0) {
        g_running.store(0);
        gesture_overlay_shutdown();
        return -1;
    }
    g_thread_started = true;
    printf("[Gesture] wrist-ROI async control ready: 224x224, fixed interval=%d, "
           "display=%d@%.2f control=3@%.2f cooldown=1.5s\n",
           INFERENCE_INTERVAL, CONFIRM_COUNT, CONFIRM_SCORE, CONFIRM_SCORE);
    return 0;
}

void gesture_overlay_set_hand_rois(const GestureHandRoi *rois, int count)
{
    pthread_mutex_lock(&g_mutex);
    g_roi_count = std::max(0, std::min(2, count));
    for (int i = 0; i < g_roi_count; ++i) g_rois[i] = rois[i];
    pthread_mutex_unlock(&g_mutex);
}

void gesture_overlay_submit_latest(const uint8_t *nv12, int width, int height)
{
    if (!g_thread_started || !nv12 || width <= 0 || height <= 0) return;
    GestureHandRoi rois[2]{};
    int roi_count = 0;
    pthread_mutex_lock(&g_mutex);
    roi_count = g_roi_count;
    for (int i = 0; i < roi_count; ++i) rois[i] = g_rois[i];
    pthread_mutex_unlock(&g_mutex);
    if (roi_count == 0) return;
    if (++g_frame_counter % INFERENCE_INTERVAL != 0) return;
    unsigned int generation = control_get_mode_generation();

    // A sampled frame submits both hands.  At a 15 FPS visual rate this is
    // 5 sampled frames/s; two hand inferences remain asynchronous and bounded.
    pthread_mutex_lock(&g_mutex);
    g_pending_head = 0;
    g_pending_tail = 0;
    g_pending_count = 0;  // newest sampled frame wins
    for (int roi_index = 0; roi_index < roi_count; ++roi_index) {
        GestureHandRoi roi = rois[roi_index];
        // NV12 crops must be even and stay inside the source.  The hand model
        // also expects a square crop; reject bad geometry before calling RGA.
        bool valid_roi = roi.width >= 96 && roi.width <= MAX_ROI &&
                         roi.width == roi.height && roi.width % 16 == 0 &&
                         roi.x >= 0 && roi.y >= 0 &&
                         (roi.x & 1) == 0 && (roi.y & 1) == 0 &&
                         roi.x + roi.width <= width && roi.y + roi.height <= height;
        if (!valid_roi) {
            static unsigned int invalid_roi_count = 0;
            if (++invalid_roi_count == 1 || invalid_roi_count % 30 == 0) {
                printf("[Gesture] reject invalid ROI x=%d y=%d w=%d h=%d frame=%dx%d count=%u\n",
                       roi.x, roi.y, roi.width, roi.height, width, height,
                       invalid_roi_count);
            }
            continue;
        }
        rga_buffer_t src = wrapbuffer_virtualaddr((void *)nv12, width, height,
                                                   RK_FORMAT_YCbCr_420_SP);
        rga_buffer_t crop = wrapbuffer_virtualaddr(g_crop_nv12, roi.width, roi.height,
                                                    RK_FORMAT_YCbCr_420_SP);
        rga_buffer_t small = wrapbuffer_virtualaddr(g_small_nv12, INPUT_W, INPUT_H,
                                                     RK_FORMAT_YCbCr_420_SP);
        int slot = g_pending_tail;
        rga_buffer_t rgb = wrapbuffer_virtualaddr(g_pending_rgb[slot], INPUT_W, INPUT_H,
                                                   RK_FORMAT_RGB_888);
        im_rect crop_rect = {roi.x, roi.y, roi.width, roi.height};
        IM_STATUS crop_status = imcrop(src, crop, crop_rect);
        if (crop_status != IM_STATUS_SUCCESS) {
            static unsigned int crop_error_count = 0;
            if (++crop_error_count == 1 || crop_error_count % 30 == 0)
                printf("[Gesture] RGA ROI crop failed status=%d count=%u\n",
                       (int)crop_status, crop_error_count);
            continue;
        }
        IM_STATUS resize_status = imresize(crop, small, 0, 0, INTER_LINEAR);
        if (resize_status != IM_STATUS_SUCCESS) {
            static unsigned int resize_error_count = 0;
            if (++resize_error_count == 1 || resize_error_count % 30 == 0)
                printf("[Gesture] RGA ROI resize failed status=%d count=%u\n",
                       (int)resize_status, resize_error_count);
            continue;
        }
        IM_STATUS color_status = imcvtcolor(small, rgb, RK_FORMAT_YCbCr_420_SP,
                                            RK_FORMAT_RGB_888,
                                            IM_YUV_TO_RGB_BT601_LIMIT);
        if (color_status != IM_STATUS_SUCCESS) {
            static unsigned int color_error_count = 0;
            if (++color_error_count == 1 || color_error_count % 30 == 0)
                printf("[Gesture] RGA ROI color failed status=%d count=%u\n",
                       (int)color_status, color_error_count);
            continue;
        }
        g_pending_rois[slot] = roi;
        g_pending_generations[slot] = generation;
        g_pending_tail = (g_pending_tail + 1) % PENDING_CAPACITY;
        ++g_pending_count;
    }
    if (g_pending_count > 0) pthread_cond_signal(&g_cond);
    pthread_mutex_unlock(&g_mutex);
}

void gesture_overlay_draw(uint8_t *nv12, int width, int height)
{
    if (!nv12 || width <= 0 || height <= 0) return;
    Result results[2];
    GestureHandRoi rois[2]{};
    int roi_count = 0;
    pthread_mutex_lock(&g_mutex);
    results[0] = g_results[0];
    results[1] = g_results[1];
    roi_count = g_roi_count;
    for (int i = 0; i < roi_count; ++i) rois[i] = g_rois[i];
    pthread_mutex_unlock(&g_mutex);

    cv::Mat y(height, width, CV_8UC1, nv12);
    char lines[2][160];
    bool roi_present[2] = {false, false};
    for (int i = 0; i < roi_count; ++i) roi_present[rois[i].side ? 1 : 0] = true;
    int max_text_width = 0;
    for (int side = 0; side < 2; ++side) {
        Result& result = results[side];
        uint64_t age = result.timestamp_us ? now_us() - result.timestamp_us : UINT64_MAX;
        const char *side_name = side ? "Right Hand" : "Left Hand";
        if (!roi_present[side])
            snprintf(lines[side], sizeof(lines[side]), "%s: no wrist ROI", side_name);
        else if (!result.connected)
            snprintf(lines[side], sizeof(lines[side]), "%s: offline", side_name);
        else if (age > RESULT_FRESH_US)
            snprintf(lines[side], sizeof(lines[side]), "%s: scanning", side_name);
        else
            snprintf(lines[side], sizeof(lines[side]), "%s: %s %.2f  %.0fms%s%s",
                     side_name, result.valid ? result.gesture : "NoHand",
                     result.gesture_score, result.inference_ms,
                     result.stable[0] ? "  Stable: " : "", result.stable);
        int baseline = 0;
        cv::Size line_size = cv::getTextSize(lines[side], cv::FONT_HERSHEY_SIMPLEX,
                                             0.64, 2, &baseline);
        max_text_width = std::max(max_text_width, line_size.width);
    }
    int x = std::max(10, width - max_text_width - 18);
    int top = 12;
    cv::rectangle(y, cv::Point(x - 6, top - 4),
                  cv::Point(std::min(width - 1, x + max_text_width + 6), top + 58),
                  cv::Scalar(0), -1);
    cv::putText(y, lines[0], cv::Point(x, top + 18),
                cv::FONT_HERSHEY_SIMPLEX, 0.64, cv::Scalar(255), 2);
    cv::putText(y, lines[1], cv::Point(x, top + 46),
                cv::FONT_HERSHEY_SIMPLEX, 0.64, cv::Scalar(255), 2);

    for (int i = 0; i < roi_count; ++i) {
        const GestureHandRoi& roi = rois[i];
        cv::rectangle(y, cv::Point(roi.x, roi.y),
                      cv::Point(roi.x + roi.width, roi.y + roi.height),
                      cv::Scalar(180), 2);
        cv::putText(y, roi.side ? "R-hand ROI" : "L-hand ROI",
                    cv::Point(roi.x, std::max(20, roi.y - 5)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255), 2);
    }

    for (int side = 0; side < 2; ++side) {
        Result& result = results[side];
        uint64_t age = result.timestamp_us ? now_us() - result.timestamp_us : UINT64_MAX;
        if (!roi_present[side] || !result.valid || age > RESULT_FRESH_US) continue;
        float sx = (float)result.roi.width / INPUT_W;
        float sy = (float)result.roi.height / INPUT_H;
        for (int i = 0; i < 21; ++i) {
            int px = result.roi.x + (int)(result.landmarks[i * 3] * sx);
            int py = result.roi.y + (int)(result.landmarks[i * 3 + 1] * sy);
            if (px >= 0 && px < width && py >= 0 && py < height)
                cv::circle(y, cv::Point(px, py), 3, cv::Scalar(255), -1);
        }
    }
}

void gesture_overlay_shutdown(void)
{
    if (g_thread_started) {
        g_running.store(0);
        pthread_mutex_lock(&g_mutex);
        pthread_cond_broadcast(&g_cond);
        pthread_mutex_unlock(&g_mutex);
        pthread_join(g_thread, nullptr);
        g_thread_started = false;
    }
    free(g_small_nv12); g_small_nv12 = nullptr;
    free(g_crop_nv12); g_crop_nv12 = nullptr;
    free(g_pending_rgb[0]); g_pending_rgb[0] = nullptr;
    free(g_pending_rgb[1]); g_pending_rgb[1] = nullptr;
    free(g_work_rgb); g_work_rgb = nullptr;
}
