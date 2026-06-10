/**
 * rga_npu.cpp - 双模型切换版：FACE (6点脸部) + BODY (YOLOv8n-pose 17点)
 */
#include "rga_npu.h"
#include "uart_comm.h"
#include "nrf24_linux.h"
#include <rknn_api.h>
#include <cstddef>
#include <im2d.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <vector>
#include <algorithm>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <errno.h>
#include <opencv2/opencv.hpp>
#include <atomic>

// ========== 配置 ==========
// #define FACE_MODEL_PATH     "./models/face_best.rknn"
#define FACE_LM_MODEL_PATH  "./models/face_landmark_468_fp16.rknn"
#define BODY_MODEL_PATH     "./models/best.rknn"
#define MODEL_INPUT_SIZE    640
#define FACE_LM_INPUT_SIZE  192
#define MAX_KEYPOINTS       20
#define OBJ_THRESHOLD       0.25f
#define NMS_THRESHOLD       0.45f
#define MAX_DETECTIONS      10
#define KPT_CONF_THRESHOLD  0.30f
#define RULE_MODEL_PATH     "./models/mode_rule_engine_fp16.rknn"

// ========== 全局变量 ==========
static std::atomic<PoseMode> g_mode{MODE_FACE};

// Face landmark model (MediaPipe Face Mesh 468, 192x192)
static rknn_context face_lm_ctx = 0;
static rknn_tensor_attr face_lm_output_attr[2];
static rknn_tensor_mem *face_lm_output_mem[2] = {NULL, NULL};
static uint8_t *face_lm_input_buf = NULL;
static uint8_t *face_lm_tmp_nv12 = NULL;
static uint8_t *face_lm_crop_buf = NULL;
#define FACE_LM_MAX_ROI     512

// Body model
static rknn_context body_ctx = 0;
static rknn_tensor_attr body_output_attr;
static rknn_tensor_mem *body_output_mem = NULL;

static uint8_t *rga_input_buf = NULL;
static uint8_t *rga_output_buf = NULL;
static float *npu_input_fp32 = NULL;
static uint16_t *npu_input_fp16 = NULL;

// Rule engine socket client
static int g_rule_sock = -1;
#define RULE_SOCK_PATH "/tmp/rule_engine.sock"

// Rule engine state registers (cross-frame) — 对应 handcraft 模型的 7 个 INT64 输出
static int64_t g_rule_state = 0;       // output[0]
static int64_t g_rule_r_hold = 0;      // output[1] right_hold
static int64_t g_rule_l_hold = 0;      // output[2] left_hold
static int64_t g_rule_c_hold = 0;      // output[3] center_hold
static int64_t g_rule_n_hold = 0;      // output[4] neutral_hold
static int64_t g_rule_r_miss = 0;      // output[5] right_keep_miss
static int64_t g_rule_l_miss = 0;      // output[6] left_keep_miss

static int g_frame_count = 0;
static int npu_initialized = 0;
static uint64_t g_frame_start_us = 0;

static const int PNP_CALIB_TARGETS[] = {
    0, -15, -30, -45, -60, -75
};
static const int PNP_CALIB_TARGET_COUNT =
    (int)(sizeof(PNP_CALIB_TARGETS) / sizeof(PNP_CALIB_TARGETS[0]));
static const uint64_t PNP_CALIB_PREPARE_US = 5000000;
static const uint64_t PNP_CALIB_SAMPLE_US = 8000000;

enum {
    PNP_CALIB_INACTIVE = 0,
    PNP_CALIB_PREPARE = 1,
    PNP_CALIB_SAMPLE = 2,
    PNP_CALIB_COMPLETE = 3
};

static std::atomic<int> g_pnp_calib_target_idx{-1};
static std::atomic<int> g_pnp_calib_phase{PNP_CALIB_INACTIVE};
static std::atomic<uint64_t> g_pnp_calib_phase_start_us{0};
static std::atomic<float> g_arm_target_yaw_deg{0.0f};

struct PnpYawCalibrationPoint {
    float arm_yaw_deg;
    float compensation_deg;
};

/*
 * compensation = -measured PnP yaw while the face is aligned with the camera.
 * Positive arm-yaw points come from the first run; negative points come from
 * the trusted negative-side rerun. The 0-degree value comes from the first run.
 */
static const PnpYawCalibrationPoint PNP_YAW_CALIBRATION[] = {
    {-75.0f, -6.5947f},
    {-60.0f, -0.3278f},
    {-45.0f,  3.2507f},
    {-30.0f,  6.2222f},
    {-15.0f,  5.0208f},
    {  0.0f,  3.4033f},
    { 15.0f,  6.1024f},
    { 30.0f, 10.1275f},
    { 45.0f, 14.6082f},
    { 60.0f, 16.5775f},
    { 75.0f, 24.1004f}
};

static float interpolate_pnp_yaw_compensation(float arm_yaw_deg) {
    const int count =
        (int)(sizeof(PNP_YAW_CALIBRATION) / sizeof(PNP_YAW_CALIBRATION[0]));

    if (arm_yaw_deg <= PNP_YAW_CALIBRATION[0].arm_yaw_deg)
        return PNP_YAW_CALIBRATION[0].compensation_deg;
    if (arm_yaw_deg >= PNP_YAW_CALIBRATION[count - 1].arm_yaw_deg)
        return PNP_YAW_CALIBRATION[count - 1].compensation_deg;

    for (int i = 0; i < count - 1; ++i) {
        const PnpYawCalibrationPoint& a = PNP_YAW_CALIBRATION[i];
        const PnpYawCalibrationPoint& b = PNP_YAW_CALIBRATION[i + 1];
        if (arm_yaw_deg <= b.arm_yaw_deg) {
            float ratio = (arm_yaw_deg - a.arm_yaw_deg) /
                          (b.arm_yaw_deg - a.arm_yaw_deg);
            return a.compensation_deg +
                   ratio * (b.compensation_deg - a.compensation_deg);
        }
    }
    return 0.0f;
}

static int read_calib_mode(void) {
    FILE *fp = fopen("/tmp/calib_mode.txt", "r");
    if (!fp) return 0;

    int mode = 0;
    if (fscanf(fp, "%d", &mode) != 1) mode = 0;
    fclose(fp);
    return mode;
}

void set_frame_start_time_us(uint64_t us) {
    g_frame_start_us = us;
}

int convert_yuyv_to_nv12(uint8_t *src, uint8_t *dst, int width, int height) {
    rga_buffer_t rga_src = wrapbuffer_virtualaddr(src, width, height, RK_FORMAT_YUYV_422);
    rga_buffer_t rga_dst = wrapbuffer_virtualaddr(dst, width, height, RK_FORMAT_YCbCr_420_SP);
    IM_STATUS ret = imresize(rga_src, rga_dst, 1.0, 1.0, INTER_LINEAR);
    if (ret != IM_STATUS_SUCCESS) {
        printf("[RGA] YUYV->NV12 failed: %d\n", ret);
        return -1;
    }
    return 0;
}

// ========== 3D Face Pose ==========
static const int POSE_REQUIRED_IDS[6] = {1, 2, 0, 17, 18, 19};
static const int FACE_VALID_KPS[6] = {0, 1, 2, 17, 18, 19};
static const std::vector<cv::Point3f> FACE_3D_POINTS = {
    {-30.0f, -25.0f, -60.0f},   // 1: right eye
    {30.0f,  -25.0f, -60.0f},   // 2: left eye
    {0.0f,   -5.0f,  -90.0f},   // 0: nose
    {-25.0f, 20.0f,  -65.0f},   // 17: right mouth
    {25.0f,  20.0f,  -65.0f},   // 18: left mouth
    {0.0f,   50.0f,  -40.0f}    // 19: chin
};

// face_landmark_468 -> MediaPipe canonical face geometry 的 12 点 PnP
static const int FACE_LM_12_IDS[12] = {
    133,  // 0: right eye inner
    263,  // 1: left eye outer
    1,    // 2: nose tip
    61,   // 3: right mouth corner
    291,  // 4: left mouth corner
    152,  // 5: chin
    33,   // 6: right eye outer
    362,  // 7: left eye inner
    48,   // 8: right nose
    278,  // 9: left nose
    105,  // 10: right brow
    334   // 11: left brow
};

/*
 * Source: MediaPipe canonical_face_model.obj, vertex indices above.
 * Canonical coordinates are centimeters with +Y up and +Z toward the face
 * front. This project uses +Y down and face-front toward -Z, so vertices are
 * transformed as (x, -y, -z), scaled to millimeters, then translated to keep
 * landmark 1 at the previous nose anchor (0, -5, -90).
 */
static const std::vector<cv::Point3f> FACE_LM_12_3D = {
    {-18.564320f, -42.121100f, -52.823000f},  // 133
    { 44.458590f, -42.908560f, -46.978180f},  // 263
    {  0.000000f,  -5.000000f, -90.000000f},  // 1
    {-24.562060f,  27.157560f, -58.082800f},  // 61
    { 24.562060f,  27.157560f, -58.082800f},  // 291
    {  0.000000f,  77.765130f, -57.888880f},  // 152
    {-44.458590f, -42.908560f, -46.978180f},  // 33
    { 18.564320f, -42.121100f, -52.823000f},  // 362
    {-16.086350f,  -6.843490f, -73.385890f},  // 48
    { 16.086350f,  -6.843490f, -73.385890f},  // 278
    {-39.865620f, -67.363520f, -59.907110f},  // 105
    { 39.865620f, -67.363520f, -59.907110f}   // 334
};

// USB Camera3 2.7mm 130° wide-angle temporary estimate
// TODO: run cv::calibrateCamera with checkerboard for accurate values
// USB Camera3 (Realtek) calibrated 2026-04-26
// RMS error: 0.9759 px | Checkerboard 9x6 corners @ 25mm
static const cv::Mat CAMERA_MATRIX = (cv::Mat_<float>(3,3) <<
    689.58f, 0.0f,     982.87f,
    0.0f,    686.99f,  394.59f,
    0.0f,    0.0f,     1.0f);

static const cv::Mat DIST_COEFFS = (cv::Mat_<float>(1,5) <<
    -0.140459f, 0.270074f, 0.000120f, 0.003055f, -0.395587f);

/* ---------- 全链路各环节耗时统计 ---------- */
#define STAT_WINDOW 30

static inline uint64_t get_us() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000;
}

static struct {
    uint64_t get_frame_us;      // 图像获取 (GST pull_sample / identity handoff)
    uint64_t rga_preprocess_us; // body RGA resize + cvtcolor
    uint64_t npu_body_us;       // body: set + run + post_process
    uint64_t roi_crop_us;       // ROI: RGA crop+resize+cvtcolor + f32 convert
    uint64_t npu_face_us;       // face_lm: set + run + post_process + coord map
    uint64_t draw_us;           // 画点 / 画框 / draw_detections
    uint64_t encode_push_us;    // 编码推流 (GST push_buffer)
    uint64_t total_us;          // process_frame 总耗时
} g_stat[STAT_WINDOW];

static int g_stat_idx = 0;
static int g_stat_count = 0;
static uint64_t g_next_get_frame_us = 0;
static uint64_t g_next_encode_push_us = 0;

void report_gst_getframe_us(uint64_t us) {
    g_next_get_frame_us = us;
}

void report_gst_encode_us(uint64_t us) {
    g_next_encode_push_us = us;
}

static void print_pipeline_stats() {
    double sum_get = 0, sum_rga = 0, sum_npu_body = 0;
    double sum_roi = 0, sum_npu_face = 0, sum_draw = 0;
    double sum_encode = 0, sum_total = 0;
    for (int i = 0; i < STAT_WINDOW; i++) {
        sum_get      += g_stat[i].get_frame_us;
        sum_rga      += g_stat[i].rga_preprocess_us;
        sum_npu_body += g_stat[i].npu_body_us;
        sum_roi      += g_stat[i].roi_crop_us;
        sum_npu_face += g_stat[i].npu_face_us;
        sum_draw     += g_stat[i].draw_us;
        sum_encode   += g_stat[i].encode_push_us;
        sum_total    += g_stat[i].total_us;
    }
    double n = STAT_WINDOW;
    printf("\n[PIPE-STAT] ======== Last %d frames avg ========\n", STAT_WINDOW);
    printf("[PIPE-STAT] get_frame     : %7.2f ms  (GST capture)\n",    sum_get      / n / 1000.0);
    printf("[PIPE-STAT] rga_preprocess: %7.2f ms  (resize+cvtcolor)\n", sum_rga      / n / 1000.0);
    printf("[PIPE-STAT] npu_body      : %7.2f ms  (set+run+post)\n",    sum_npu_body / n / 1000.0);
    printf("[PIPE-STAT] roi_crop      : %7.2f ms  (crop+resize+cvt+f32)\n", sum_roi   / n / 1000.0);
    printf("[PIPE-STAT] npu_face_lm   : %7.2f ms  (set+run+post)\n",    sum_npu_face / n / 1000.0);
    printf("[PIPE-STAT] draw          : %7.2f ms  (dots/boxes)\n",      sum_draw     / n / 1000.0);
    printf("[PIPE-STAT] encode_push   : %7.2f ms  (GST push+encode)\n",  sum_encode   / n / 1000.0);
    printf("[PIPE-STAT] total_ai      : %7.2f ms\n",                      sum_total    / n / 1000.0);
    printf("[PIPE-STAT] ===================================\n\n");
}

// ========== 数据结构 ==========
typedef struct { float x, y, visibility; } Keypoint;
typedef struct {
    float x1, y1, x2, y2;
    float score;
    Keypoint kps[MAX_KEYPOINTS];
} PoseDetection;

// ========== One Euro Filter + Face Tracker ==========
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class OneEuroFilter {
    double freq_, mincutoff_, beta_, dcutoff_;
    double x_prev_, dx_prev_;
    bool initialized_;
public:
    OneEuroFilter(double freq = 15.0, double mincutoff = 0.5, double beta = 0.003, double dcutoff = 1.0)
        : freq_(freq), mincutoff_(mincutoff), beta_(beta), dcutoff_(dcutoff),
          x_prev_(0), dx_prev_(0), initialized_(false) {}

    void set_params(double freq, double mincutoff, double beta, double dcutoff) {
        freq_ = freq; mincutoff_ = mincutoff; beta_ = beta; dcutoff_ = dcutoff;
    }

    void reset(double x0) { x_prev_ = x0; dx_prev_ = 0; initialized_ = true; }
    double get_mincutoff() const { return mincutoff_; }
    double get_beta() const { return beta_; }

    double filter(double x) {
        if (!initialized_) { reset(x); return x; }
        double dx = (x - x_prev_) * freq_;
        double a_d = alpha(freq_, dcutoff_);
        double dx_hat = lowpass(dx_prev_, dx, a_d);
        double cutoff = mincutoff_ + beta_ * std::abs(dx_hat);
        double a = alpha(freq_, cutoff);
        double x_hat = lowpass(x_prev_, x, a);
        x_prev_ = x_hat;
        dx_prev_ = dx_hat;
        return x_hat;
    }
private:
    double alpha(double freq, double cutoff) {
        if (cutoff <= 0) cutoff = 1e-6;
        double tau = 1.0 / (2.0 * M_PI * cutoff);
        return 1.0 / (1.0 + tau * freq);
    }
    double lowpass(double prev, double curr, double alpha) {
        return alpha * curr + (1.0 - alpha) * prev;
    }
};

struct FaceTracker {
    static constexpr int WIN_SIZE = 10;
    struct Record { float cx, cy, area; bool valid; };
    Record history[WIN_SIZE];
    int hist_idx = 0;
    int valid_count = 0;
    float last_cx = 0, last_cy = 0;
    bool tracked = false;

    // 人脸尺寸跟踪（用于动态 ROI）
    float last_face_w = 0, last_face_h = 0;
    int face_size_life = 0;
    static constexpr int FACE_SIZE_MAX_LIFE = 5;
    float prev_roi_size = 0;

    int num_kps;
    PoseMode mode;

    OneEuroFilter weak_kf_x[MAX_KEYPOINTS];
    OneEuroFilter weak_kf_y[MAX_KEYPOINTS];
    OneEuroFilter strong_kf_x[MAX_KEYPOINTS];
    OneEuroFilter strong_kf_y[MAX_KEYPOINTS];
    bool weak_init[MAX_KEYPOINTS] = {false};
    bool strong_init[MAX_KEYPOINTS] = {false};

    int tele_frames = 0;
    int tele_tracked = 0;
    float tele_jitter_sum = 0;
    float tele_jitter_max = 0;
    float tele_speed_sum = 0;
    float tele_speed_max = 0;
    int tele_kp_count = 0;
    int tele_speed_cnt = 0;

    float prev_raw_x[MAX_KEYPOINTS] = {0};
    float prev_raw_y[MAX_KEYPOINTS] = {0};
    bool prev_raw_valid[MAX_KEYPOINTS] = {false};

    bool is_weak_kp(int k) const {
        if (mode == MODE_FACE) {
            return (k >= 0 && k <= 4) || (k >= 17 && k <= 19);
        } else {
            return (k >= 0 && k <= 4);
        }
    }

    FaceTracker(PoseMode m = MODE_FACE, int nkps = 20) : num_kps(nkps), mode(m) {
        for (int i = 0; i < WIN_SIZE; i++) history[i] = {0, 0, 0, false};
        for (int k = 0; k < MAX_KEYPOINTS; k++) {
            // weak: pose 关键点 (0,1,2,17,18,19)，beta 降半减少抖动
            weak_kf_x[k].set_params(15.0, 1.2, 0.015, 1.0);
            weak_kf_y[k].set_params(15.0, 1.2, 0.015, 1.0);
            // strong: 其他关键点
            strong_kf_x[k].set_params(15.0, 0.8, 0.008, 1.0);
            strong_kf_y[k].set_params(15.0, 0.8, 0.008, 1.0);
        }
    }

    void reset_mode(PoseMode m, int nkps) {
        mode = m;
        num_kps = nkps;
        tracked = false;
        valid_count = 0;
        hist_idx = 0;
        for (int i = 0; i < WIN_SIZE; i++) history[i] = {0, 0, 0, false};
        for (int k = 0; k < MAX_KEYPOINTS; k++) {
            weak_init[k] = false;
            strong_init[k] = false;
            prev_raw_valid[k] = false;
        }
        last_face_w = 0; last_face_h = 0;
        face_size_life = 0; prev_roi_size = 0;
    }

    int update(const std::vector<PoseDetection>& dets, int img_w, int img_h) {
        float short_edge = std::min(img_w, img_h);
        float max_drift = short_edge * 0.15f;

        const PoseDetection* best = nullptr;
        float best_score = 1e9f;
        int best_idx = -1;

        for (size_t i = 0; i < dets.size(); i++) {
            float cx = (dets[i].x1 + dets[i].x2) * 0.5f;
            float cy = (dets[i].y1 + dets[i].y2) * 0.5f;

            float score = 1e9f;
            if (tracked) {
                float d = std::hypot(cx - last_cx, cy - last_cy);
                if (d > max_drift) continue;
                score = d;
            } else {
                score = std::hypot(cx - img_w * 0.5f, cy - img_h * 0.5f);
            }

            if (score < best_score) {
                best_score = score;
                best = &dets[i];
                best_idx = (int)i;
            }
        }

        auto& slot = history[hist_idx];
        if (slot.valid) valid_count--;

        if (best) {
            slot.cx = (best->x1 + best->x2) * 0.5f;
            slot.cy = (best->y1 + best->y2) * 0.5f;
            slot.area = (best->x2 - best->x1) * (best->y2 - best->y1);
            slot.valid = true;
            valid_count++;
        } else {
            slot.valid = false;
        }
        hist_idx = (hist_idx + 1) % WIN_SIZE;

        if (valid_count >= 5 && best) {
            float sum_cx = 0, sum_cy = 0;
            int cnt = 0;
            for (int i = 0; i < WIN_SIZE; i++) {
                if (history[i].valid) {
                    sum_cx += history[i].cx;
                    sum_cy += history[i].cy;
                    cnt++;
                }
            }
            last_cx = sum_cx / cnt;
            last_cy = sum_cy / cnt;
            tracked = true;
            return best_idx;
        } else {
            tracked = false;
            return -1;
        }
    }

    void apply_filter(PoseDetection& det) {
        for (int k = 0; k < num_kps; k++) {
            bool visible = det.kps[k].visibility > 0.5f;
            if (!visible) {
                weak_init[k] = false;
                strong_init[k] = false;
                prev_raw_valid[k] = false;
                continue;
            }

            float raw_x = det.kps[k].x;
            float raw_y = det.kps[k].y;

            double fx, fy;
            bool use_weak = is_weak_kp(k);

            if (use_weak) {
                if (!weak_init[k]) {
                    weak_kf_x[k].reset(raw_x);
                    weak_kf_y[k].reset(raw_y);
                    weak_init[k] = true;
                }
                fx = weak_kf_x[k].filter(raw_x);
                fy = weak_kf_y[k].filter(raw_y);
            } else {
                if (!strong_init[k]) {
                    strong_kf_x[k].reset(raw_x);
                    strong_kf_y[k].reset(raw_y);
                    strong_init[k] = true;
                }
                fx = strong_kf_x[k].filter(raw_x);
                fy = strong_kf_y[k].filter(raw_y);
            }

            float jitter = std::hypot(raw_x - (float)fx, raw_y - (float)fy);
            tele_jitter_sum += jitter;
            if (jitter > tele_jitter_max) tele_jitter_max = jitter;

            if (prev_raw_valid[k]) {
                float speed = std::hypot(raw_x - prev_raw_x[k], raw_y - prev_raw_y[k]);
                tele_speed_sum += speed;
                if (speed > tele_speed_max) tele_speed_max = speed;
                tele_speed_cnt++;
            }
            tele_kp_count++;

            prev_raw_x[k] = raw_x;
            prev_raw_y[k] = raw_y;
            prev_raw_valid[k] = true;

            det.kps[k].x = (float)fx;
            det.kps[k].y = (float)fy;
        }
    }

    void update_face_size(float w, float h) {
        const float ALPHA = 0.5f;
        if (last_face_w > 0 && last_face_h > 0) {
            last_face_w = ALPHA * w + (1.0f - ALPHA) * last_face_w;
            last_face_h = ALPHA * h + (1.0f - ALPHA) * last_face_h;
        } else {
            last_face_w = w;
            last_face_h = h;
        }
        face_size_life = FACE_SIZE_MAX_LIFE;
    }

    bool has_face_size() const {
        return face_size_life > 0 && last_face_w > 32.0f && last_face_h > 32.0f;
    }

    float get_roi_size() const {
        if (!has_face_size()) return 0;
        // 468 点 bbox 不含头发顶部，padding 放大确保覆盖全头
        // 比例 + 固定余量：远距离不暴涨，近距离有保底
        return std::max(last_face_w, last_face_h) * 1.3f + 50.0f;
    }

    void print_telemetry() {
        if (tele_frames == 0 || tele_kp_count == 0) {
            printf("[Telemetry] No data accumulated\n");
            return;
        }
        float avg_jitter = tele_jitter_sum / tele_kp_count;
        float avg_speed = tele_speed_cnt > 0 ? tele_speed_sum / tele_speed_cnt : 0.0f;

        printf("[Telemetry] Mode=%s | Frames=%d Tracked=%d Rate=%.1f%%\n",
               pose_mode_name(mode), tele_frames, tele_tracked, 100.0f * tele_tracked / tele_frames);
        printf("[Telemetry] AvgJitter=%.2fpx MaxJitter=%.2fpx\n", avg_jitter, tele_jitter_max);
        printf("[Telemetry] AvgSpeed=%.2fpx/frame MaxSpeed=%.2fpx/frame\n", avg_speed, tele_speed_max);

        tele_frames = 0;
        tele_tracked = 0;
        tele_jitter_sum = 0;
        tele_jitter_max = 0;
        tele_speed_sum = 0;
        tele_speed_max = 0;
        tele_kp_count = 0;
        tele_speed_cnt = 0;
    }
};

// 1-Euro filters for pose rvec/tvec
// 当前实际 fps ~15，滤波器频率必须匹配
static OneEuroFilter flt_rx(15.0, 2.5, 0.08, 1.0);
static OneEuroFilter flt_ry(15.0, 2.5, 0.08, 1.0);
static OneEuroFilter flt_rz(15.0, 2.5, 0.08, 1.0);
static OneEuroFilter flt_tx(15.0, 2.5, 0.08, 1.0);
static OneEuroFilter flt_ty(15.0, 2.5, 0.08, 1.0);
static OneEuroFilter flt_tz(15.0, 2.5, 0.08, 1.0);
static bool pose_filter_init = false;
static cv::Mat prev_rvec, prev_tvec;
static bool have_prev_pose = false;

// ========== 工具函数 ==========
static void rgb_to_yuv(uint8_t r, uint8_t g, uint8_t b, uint8_t& y, uint8_t& u, uint8_t& v) {
    y = (uint8_t)std::min(255.0, std::max(0.0, 0.299*r + 0.587*g + 0.114*b));
    u = (uint8_t)std::min(255.0, std::max(0.0, -0.169*r - 0.331*g + 0.5*b + 128.0));
    v = (uint8_t)std::min(255.0, std::max(0.0, 0.5*r - 0.419*g - 0.081*b + 128.0));
}

static void set_pixel_nv12(uint8_t* nv12, int w, int h, int x, int y,
                           uint8_t y_val, uint8_t u_val, uint8_t v_val) {
    if (x < 0 || x >= w || y < 0 || y >= h) return;
    nv12[y * w + x] = y_val;
    int uv_off = w * h + (y / 2) * w + (x / 2) * 2;
    nv12[uv_off]     = u_val;
    nv12[uv_off + 1] = v_val;
}

static void draw_line_nv12(uint8_t* nv12, int w, int h, int x0, int y0, int x1, int y1,
                           uint8_t y_val, uint8_t u_val, uint8_t v_val) {
    int dx = std::abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    while (true) {
        set_pixel_nv12(nv12, w, h, x0, y0, y_val, u_val, v_val);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

static void draw_crosshair(uint8_t* y_plane, int w, int h, int cx, int cy, int len) {
    for (int x = cx - len; x <= cx + len; x++) {
        if (x >= 0 && x < w) y_plane[cy * w + x] = 255;
    }
    for (int y = cy - len; y <= cy + len; y++) {
        if (y >= 0 && y < h) y_plane[y * w + cx] = 255;
    }
}

static void draw_pose_indicator(uint8_t* nv12, int w, int h, double yaw, double pitch) {
    uint8_t* y_plane = nv12;
    int cx = w / 2;
    int cy = h / 2;
    draw_crosshair(y_plane, w, h, cx, cy, 20);
    int scale = 400;
    int tx = cx + (int)(yaw * scale);
    int ty = cy + (int)(pitch * scale);
    tx = std::max(5, std::min(tx, w - 6));
    ty = std::max(5, std::min(ty, h - 6));
    draw_line_nv12(nv12, w, h, cx, cy, tx, ty, 200, 128, 128);
    for (int dy = -3; dy <= 3; dy++) {
        for (int dx = -3; dx <= 3; dx++) {
            int px = tx + dx;
            int py = ty + dy;
            if (px >= 0 && px < w && py >= 0 && py < h) {
                y_plane[py * w + px] = 255;
            }
        }
    }
}

/* 角度归一化到 [-180, 180]，消除万向节环绕问题 */
static inline float normalize_angle_deg(float deg)
{
    while (deg > 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

/* ========== 8 状态运动状态机（修复版）========== */
typedef enum {
    STATE_STOP_TO_ACCEL = 0,
    STATE_ACCEL,
    STATE_ACCEL_TO_CONST,
    STATE_CONST_SPEED,
    STATE_CONST_TO_DECEL,
    STATE_DECEL_TO_STOP,
    STATE_STOP,
    STATE_DECEL_STOP_TO_ACCEL
} MotionState;

static const char* motion_state_name(MotionState s)
{
    switch (s) {
        case STATE_STOP_TO_ACCEL:       return "静止→加速";
        case STATE_ACCEL:               return "加速";
        case STATE_ACCEL_TO_CONST:      return "加速→匀速";
        case STATE_CONST_SPEED:         return "匀速";
        case STATE_CONST_TO_DECEL:      return "匀速→减速";
        case STATE_DECEL_TO_STOP:       return "减速→停止";
        case STATE_STOP:                return "停止";
        case STATE_DECEL_STOP_TO_ACCEL: return "减速→停止→加速";
    }
    return "未知";
}

/* ========== 基于 wz 历史（10ms 分辨率）的运动上下文 ==========
 * 50ms 控制周期内，分析 NRF24 最近 5 帧（50ms）的完整 wz 序列，
 * 而不是只读一个瞬时值。能检测峰值/谷值/过零/趋势。
 */
struct MotionContext {
    float wz_first;          // 窗口第一个值
    float wz_peak;           // 窗口峰值
    float wz_valley;         // 窗口谷值
    float wz_latest;         // 窗口最后一个值
    float wz_avg;            // 窗口平均（用于停止判断）
    bool  has_crossed_zero;  // 窗口内是否发生过零
    bool  initialized;

    MotionContext() : wz_first(0), wz_peak(0), wz_valley(0), wz_latest(0),
                      wz_avg(0), has_crossed_zero(false), initialized(false) {}

    /* 从 NRF24 wz 历史数组更新（取最近 max_samples 个） */
    void update_from_hist(const float hist[], int count, int max_samples = 5) {
        if (count <= 0) return;
        int start = (count > max_samples) ? (count - max_samples) : 0;
        int n = count - start;
        if (n <= 0) return;

        wz_first = hist[start];
        wz_peak = hist[start];
        wz_valley = hist[start];
        wz_latest = hist[count - 1];
        has_crossed_zero = false;

        float sum = 0.0f;
        for (int i = start; i < count; i++) {
            float v = hist[i];
            if (v > wz_peak) wz_peak = v;
            if (v < wz_valley) wz_valley = v;
            sum += v;
            if (i > start && hist[i - 1] * v < 0.0f) has_crossed_zero = true;
        }
        wz_avg = sum / n;
        if (!initialized) initialized = true;
    }

    /* 停止判断：窗口平均 */
    bool is_stop() const { return std::fabs(wz_avg) < 5.0f; }

    /* 加速趋势：速度绝对值在增大，且没有过零 */
    bool accel_trend() const {
        return !has_crossed_zero && std::fabs(wz_latest) > std::fabs(wz_first) + 10.0f;
    }

    /* 减速趋势：速度绝对值在减小，且没有过零 */
    bool decel_trend() const {
        return !has_crossed_zero && std::fabs(wz_latest) < std::fabs(wz_first) - 10.0f;
    }

    /* 稳态趋势：速度绝对值变化不大，且没有过零 */
    bool steady_trend() const {
        return !has_crossed_zero && std::fabs(std::fabs(wz_latest) - std::fabs(wz_first)) <= 10.0f;
    }
};

static MotionState next_motion_state(MotionState prev, const MotionContext& ctx)
{
    bool is_stop = ctx.is_stop();
    bool accel_trend = ctx.accel_trend();
    bool decel_trend = ctx.decel_trend();
    bool steady_trend = ctx.steady_trend();
    bool crossed = ctx.has_crossed_zero;

    switch (prev) {
        case STATE_STOP:
            if (!is_stop) return STATE_STOP_TO_ACCEL;
            return STATE_STOP;

        case STATE_STOP_TO_ACCEL:
            if (is_stop) return STATE_STOP;
            if (accel_trend) return STATE_ACCEL;
            if (steady_trend) return STATE_ACCEL_TO_CONST;
            return STATE_ACCEL;

        case STATE_ACCEL:
            if (is_stop) return STATE_STOP;
            if (crossed) return STATE_DECEL_STOP_TO_ACCEL;
            if (accel_trend) return STATE_ACCEL;
            if (decel_trend) return STATE_CONST_TO_DECEL;
            if (steady_trend) return STATE_ACCEL_TO_CONST;
            return STATE_ACCEL;

        case STATE_ACCEL_TO_CONST:
            if (is_stop) return STATE_STOP;
            if (steady_trend) return STATE_CONST_SPEED;
            if (decel_trend) return STATE_CONST_TO_DECEL;
            return STATE_CONST_SPEED;

        case STATE_CONST_SPEED:
            if (is_stop) return STATE_STOP;
            if (decel_trend) return STATE_CONST_TO_DECEL;
            if (accel_trend) return STATE_ACCEL_TO_CONST;
            return STATE_CONST_SPEED;

        case STATE_CONST_TO_DECEL:
            if (is_stop) return STATE_STOP;
            if (decel_trend) return STATE_DECEL_TO_STOP;
            return STATE_DECEL_TO_STOP;

        case STATE_DECEL_TO_STOP:
            if (is_stop) return STATE_STOP;
            if (!is_stop && accel_trend) return STATE_DECEL_STOP_TO_ACCEL;
            return STATE_DECEL_TO_STOP;

        case STATE_DECEL_STOP_TO_ACCEL:
            if (is_stop) return STATE_STOP;
            if (accel_trend) return STATE_ACCEL;
            if (decel_trend) return STATE_DECEL_TO_STOP;
            if (steady_trend) return STATE_ACCEL_TO_CONST;
            return STATE_ACCEL;
    }
    return STATE_STOP;
}

/* ========== 终点预测器 ==========
 * 核心思想：根据当前速度和运动状态，预测人脸最终停止位置，
 * 提前发令让机械臂直接运动到预测终点，实现"同时到达"。
 */
struct EndpointPredictor {
    float pred_delta_yaw;   // 相对于当前 yaw 的预测位移（度）
    MotionState pred_state; // 预测时的状态
    bool valid;

    EndpointPredictor() : pred_delta_yaw(0.0f), valid(false), pred_state(STATE_STOP) {}

    void reset() { pred_delta_yaw = 0.0f; valid = false; }

    /* 更新预测：状态调制系数 k + 自适应预测窗口 dt */
    void update(float wz, MotionState state) {
        float abs_wz = std::fabs(wz);

        /* 预测时间窗口 dt：速度越大，运动结束越早，窗口越短 */
        float dt;
        if (abs_wz < 20.0f)       dt = 0.50f;  // 低速：预测 500ms
        else if (abs_wz < 60.0f)  dt = 0.35f;  // 中速：预测 350ms
        else                      dt = 0.25f;  // 高速：预测 250ms

        /* 状态调制系数 k：
         * - 加速→匀速：速度平台确立，预测最确定，k 最大
         * - 匀速：不知何时减速，最不确定，k 最小
         * - 减速：可估算剩余位移，k 中等
         */
        float k;
        switch (state) {
            case STATE_ACCEL:            k = 0.35f; break;
            case STATE_ACCEL_TO_CONST:   k = 0.70f; break;  // ★ 主窗口，最确定
            case STATE_CONST_SPEED:      k = 0.30f; break;  // 不确定，保守
            case STATE_CONST_TO_DECEL:   k = 0.60f; break;  // 修正窗口
            case STATE_DECEL_TO_STOP:    k = 0.25f; break;  // 剩余少
            case STATE_STOP:             k = 0.00f; break;
            case STATE_STOP_TO_ACCEL:    k = 0.35f; break;
            case STATE_DECEL_STOP_TO_ACCEL: k = 0.45f; break;
            default:                     k = 0.40f; break;
        }

        pred_delta_yaw = wz * dt * k;
        pred_state = state;
        valid = (state != STATE_STOP);
    }

    float get_target_yaw(float current_yaw) const {
        return current_yaw + pred_delta_yaw;
    }
};

/* ========== A-inverse 姿态解耦：消除 IMU 初始安装角 ==========
 * 初始姿态 R_init 在上电时记录，之后每帧做 R_rel = R_current * R_init^T
 * 得到头部相对于上电姿态的真实旋转。
 * 欧拉角顺序：ZYX (Yaw-Pitch-Roll)
 */
static inline cv::Mat eulerZYXToMat(float roll_deg, float pitch_deg, float yaw_deg)
{
    float r = roll_deg * (float)M_PI / 180.0f;
    float p = pitch_deg * (float)M_PI / 180.0f;
    float y = yaw_deg * (float)M_PI / 180.0f;
    float cr = cosf(r), sr = sinf(r);
    float cp = cosf(p), sp = sinf(p);
    float cy = cosf(y), sy = sinf(y);
    return (cv::Mat_<float>(3,3) <<
        cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr,
        sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr,
        -sp,    cp*sr,             cp*cr);
}

static inline void matToEulerZYX(const cv::Mat& R, float& roll_deg, float& pitch_deg, float& yaw_deg)
{
    float sy = sqrtf(R.at<float>(0,0)*R.at<float>(0,0) + R.at<float>(1,0)*R.at<float>(1,0));
    bool singular = sy < 1e-6f;
    float rr, pp, yy;
    if (!singular) {
        yy = atan2f(R.at<float>(1,0), R.at<float>(0,0));
        pp = atan2f(-R.at<float>(2,0), sy);
        rr = atan2f(R.at<float>(2,1), R.at<float>(2,2));
    } else {
        yy = atan2f(-R.at<float>(1,2), R.at<float>(1,1));
        pp = atan2f(-R.at<float>(2,0), sy);
        rr = 0.0f;
    }
    roll_deg  = rr * 180.0f / (float)M_PI;
    pitch_deg = pp * 180.0f / (float)M_PI;
    yaw_deg   = yy * 180.0f / (float)M_PI;
}

/* A-inverse R_init 捕获标志 */
volatile int g_r_init_set = 0;

/* 独立的 NRF24 IMU 控制链路：俯仰控制（仿照偏航控制架构） */
void nrf24_control_update(void)
{
    static const float NRF_FACE_X_CM = 0.0f;
    static const float NRF_FACE_Y_CM = 10.0f;
    static const float NRF_FACE_Z_CM = 30.0f;
    static const float NRF_TRACK_DIST_CM = 57.0f;
    static const float NRF_SERVO1_DEG = 50.0f;
    static const float NRF_SERVO2_DEG = 145.0f;

    /* 偏航控制状态（从 imu-main-test 恢复） */
    static bool yaw_baseline_set = false;
    static float nrf_baseline_yaw_deg = 0.0f;
    static MotionState prev_state_yaw = STATE_STOP;
    static MotionState prev_print_state_yaw = STATE_STOP;
    static MotionContext ctx_yaw;
    static EndpointPredictor predictor_yaw;
    static uint64_t last_cmd_us_yaw = 0;
    static float last_cmd_target_yaw = 0.0f;
    static const float CMD_YAW_THRESHOLD_DEG = 5.0f;
    static bool yaw_stop_cmd_sent = false;
    static uint64_t yaw_move_complete_us = 0;
    static bool yaw_last_move_complete = false;

    /* 俯仰控制状态 */
    static bool pitch_baseline_set = false;
    static float nrf_baseline_roll_deg = 0.0f;
    static uint64_t last_update_us = 0;
    static MotionState prev_state = STATE_STOP;
    static MotionState prev_print_state = STATE_STOP;
    static MotionContext ctx;
    static EndpointPredictor predictor;

    /* A-inverse 初始姿态矩阵 */
    static cv::Mat R_init;
    static float rel_roll = 0.0f, rel_pitch = 0.0f, rel_yaw = 0.0f;

    /* PnP 视觉零飘修正：累积修正矩阵 */
    static const float KI_PNP = 0.05f;
    static cv::Mat R_bias_total;

    /* A-init 触发信号由握手线程控制 (g_wait_a_init) */

    /* 发令控制状态 */
    static uint64_t last_cmd_us = 0;
    static float last_cmd_target_pitch = 0.0f;
    static const float CMD_PITCH_THRESHOLD_DEG = 5.0f;

    uint64_t now_us = get_us();
    if (now_us - last_update_us < 50000)   /* 50ms 采样周期 */
        return;
    last_update_us = now_us;

    float current_roll_deg = 0.0f;
    float current_yaw_deg = 0.0f;
    float current_pitch_deg = 0.0f;
    float wx = 0.0f;
    float wy = 0.0f;
    float wz = 0.0f;
    bool imu_valid = false;
    float wy_hist[NRF24_WY_HIST_SIZE];
    float wz_hist[NRF24_WZ_HIST_SIZE];
    int wy_count = 0;
    int wz_count = 0;
    uint32_t nrf_rx_count = 0;
    uint32_t nrf_err_count = 0;

    pthread_mutex_lock(&g_nrf24_state.mutex);
    current_roll_deg = g_nrf24_state.gy_roll;
    current_yaw_deg = g_nrf24_state.gy_yaw;
    current_pitch_deg = g_nrf24_state.gy_pitch;
    wx = g_nrf24_state.gy_wx;
    wy = g_nrf24_state.gy_wy;
    wz = g_nrf24_state.gy_wz;
    imu_valid = g_nrf24_state.imu_valid;
    wy_count = g_nrf24_state.gy_wy_count;
    wz_count = g_nrf24_state.gy_wz_count;
    nrf_rx_count = g_nrf24_state.rx_count;
    nrf_err_count = g_nrf24_state.error_count;
    if (wy_count > 0) {
        for (int i = 0; i < wy_count; i++) {
            int pos = (g_nrf24_state.gy_wy_idx - wy_count + i + NRF24_WY_HIST_SIZE)
                      % NRF24_WY_HIST_SIZE;
            wy_hist[i] = g_nrf24_state.gy_wy_hist[pos];
        }
    }
    if (wz_count > 0) {
        for (int i = 0; i < wz_count; i++) {
            int pos = (g_nrf24_state.gy_wz_idx - wz_count + i + NRF24_WZ_HIST_SIZE)
                      % NRF24_WZ_HIST_SIZE;
            wz_hist[i] = g_nrf24_state.gy_wz_hist[pos];
        }
    }
    pthread_mutex_unlock(&g_nrf24_state.mutex);

    /* ========== A-inverse：消除 IMU 初始安装角 ==========
     * A-init 基准现在由 RX 线程收集 5 帧高频数据（10ms/帧）后算平均值得到。
     * 这里只需检测 R_init 是否已构造（RX 线程已设置 g_r_init_set）。
     */
    if (imu_valid) {
        if (g_r_init_set && R_init.empty()) {
            cv::Mat R_current = eulerZYXToMat(current_roll_deg, current_pitch_deg, current_yaw_deg);
            R_init = R_current.clone();
            printf("[A-INIT] R_init built from 5-frame avg: roll=%.2f pitch=%.2f yaw=%.2f\n",
                   current_roll_deg, current_pitch_deg, current_yaw_deg);
        }
        if (g_r_init_set) {
            cv::Mat R_current = eulerZYXToMat(current_roll_deg, current_pitch_deg, current_yaw_deg);
            if (!R_bias_total.empty()) {
                R_current = R_bias_total * R_current;
            }
            cv::Mat R_rel_mat = R_current * R_init.t();
            matToEulerZYX(R_rel_mat, rel_roll, rel_pitch, rel_yaw);
        }
    }

    /* printf("[IMU] roll=%+.2f pitch=%+.2f yaw=%+.2f | wx=%+.2f wz=%+.2f valid=%d\n",
           current_roll_deg, current_pitch_deg, current_yaw_deg, wx, wz, imu_valid); */

    /*
     * PnP yaw 多点标定：
     * 机械臂依次移动到固定 yaw、pitch=0 的位置并保持，用户转头正对相机。
     * 每点准备 5 秒，然后视觉线程采样 8 秒。标定期间暂停正常 IMU 跟随。
     */
    static bool pnp_calib_active = false;
    static int pnp_calib_idx = 0;
    static int pnp_calib_phase = PNP_CALIB_INACTIVE;
    static uint64_t pnp_calib_phase_start = 0;
    static bool pnp_calib_command_sent = false;

    if (read_calib_mode() == 3) {
        if (!pnp_calib_active) {
            pnp_calib_active = true;
            pnp_calib_idx = 0;
            pnp_calib_phase = PNP_CALIB_PREPARE;
            pnp_calib_phase_start = 0;
            pnp_calib_command_sent = false;
            printf("[PnP-Calib] Moving arm through yaw targets, pitch fixed at 0 deg\n");
        }

        if (pnp_calib_idx < PNP_CALIB_TARGET_COUNT) {
            int target_yaw = PNP_CALIB_TARGETS[pnp_calib_idx];

            if (!pnp_calib_command_sent) {
                float yaw_rad = -(float)target_yaw * (float)M_PI / 180.0f;
                const float l1 = 8.0f;
                const float l2 = 5.0f;
                const float l3 = 40.0f;
                const float l4 = 28.0f;
                float tx = l3 * std::sin(yaw_rad);
                float ty = l4 + l3 * std::cos(yaw_rad);
                float tz = l2 + l1;

                const float servo1 = 30.0f;
                float servo2 = 50.0f + 0.4f * target_yaw;
                if (servo2 > 270.0f) servo2 = 270.0f;
                if (servo2 < 0.0f) servo2 = 0.0f;

                if (uart_send_arm_target(tx, ty, tz, servo2, servo1, 0x01) == 0) {
                    g_arm_target_yaw_deg.store((float)target_yaw);
                    pnp_calib_command_sent = true;
                    pnp_calib_phase_start = now_us;
                    printf("[PnP-Calib] Arm target yaw=%+d pitch=0 -> "
                           "x=%.1f y=%.1f z=%.1f\n",
                           target_yaw, tx, ty, tz);
                }
            } else {
                uint64_t elapsed = now_us - pnp_calib_phase_start;
                if (pnp_calib_phase == PNP_CALIB_PREPARE &&
                    elapsed >= PNP_CALIB_PREPARE_US) {
                    pnp_calib_phase = PNP_CALIB_SAMPLE;
                    pnp_calib_phase_start = now_us;
                    printf("[PnP-Calib] Target %+d: sampling 8s, face the camera\n",
                           target_yaw);
                } else if (pnp_calib_phase == PNP_CALIB_SAMPLE &&
                           elapsed >= PNP_CALIB_SAMPLE_US) {
                    pnp_calib_idx++;
                    pnp_calib_command_sent = false;
                    pnp_calib_phase_start = 0;
                    if (pnp_calib_idx < PNP_CALIB_TARGET_COUNT) {
                        pnp_calib_phase = PNP_CALIB_PREPARE;
                    } else {
                        pnp_calib_phase = PNP_CALIB_COMPLETE;
                        printf("[PnP-Calib] All targets complete\n");
                    }
                }
            }
        }

        g_pnp_calib_target_idx.store(pnp_calib_idx);
        g_pnp_calib_phase.store(pnp_calib_phase);
        g_pnp_calib_phase_start_us.store(pnp_calib_phase_start);
        return;
    }

    if (pnp_calib_active) {
        pnp_calib_active = false;
        pnp_calib_idx = 0;
        pnp_calib_phase = PNP_CALIB_INACTIVE;
        pnp_calib_phase_start = 0;
        pnp_calib_command_sent = false;
        g_pnp_calib_target_idx.store(-1);
        g_pnp_calib_phase.store(PNP_CALIB_INACTIVE);
        g_pnp_calib_phase_start_us.store(0);
        printf("[PnP-Calib] Control stopped\n");
    }

    /* ========== 标定模式：锁定 gx/gy/gz，只调 J4（不依赖 imu_valid）========== */
    static bool calib_active = false;
    static float calib_locked_roll = 0.0f;
    static float calib_locked_yaw = 0.0f;
    static float calib_locked_tx = 0.0f;
    static float calib_locked_ty = 0.0f;
    static float calib_locked_tz = 0.0f;
    static uint64_t last_calib_cmd_us = 0;
    static float calib_servo1 = 50.0f;

    FILE* calib_fp = fopen("/tmp/calib_mode.txt", "r");
    if (calib_fp) {
        int mode = 0;
        if (fscanf(calib_fp, "%d", &mode) == 1 && (mode == 1 || mode == 2)) {
            fclose(calib_fp);

            /* 第一次进入标定模式：锁定当前 roll/yaw，计算固定位置 */
            if (!calib_active) {
                calib_active = true;
                calib_locked_roll = current_roll_deg;   /* IMU 无效时为 0，不影响 */
                calib_locked_yaw = current_yaw_deg;

                float d_roll = normalize_angle_deg(calib_locked_roll - calib_locked_roll); /* =0 */
                float cum_pitch = d_roll * (float)M_PI / 180.0f;
                calib_locked_tz = NRF_FACE_Z_CM + 15.0f * std::sin(cum_pitch);

                float d_yaw = normalize_angle_deg(calib_locked_yaw - calib_locked_yaw); /* =0 */
                float cum_yaw = -d_yaw * (float)M_PI / 180.0f;
                calib_locked_tx = NRF_FACE_X_CM + NRF_TRACK_DIST_CM * std::sin(cum_yaw);
                calib_locked_ty = NRF_FACE_Y_CM + NRF_TRACK_DIST_CM * std::cos(cum_yaw);

                if (mode == 1) {
                    /* printf("[CALIB] 标定模式启动！锁定 roll=%.2f yaw=%.2f | tx=%.1f ty=%.1f tz=%.1f\n",
                           calib_locked_roll, calib_locked_yaw, calib_locked_tx, calib_locked_ty, calib_locked_tz); */
                } else {
                    /* printf("[CALIB] 扫描测试模式启动！锁定 roll=%.2f yaw=%.2f | tx=%.1f ty=%.1f tz=%.1f\n",
                           calib_locked_roll, calib_locked_yaw, calib_locked_tx, calib_locked_ty, calib_locked_tz); */
                }
            }

            if (mode == 1) {
                /* 读取舵机标定文件 */
                FILE* servo_fp = fopen("/tmp/servo_calib.txt", "r");
                if (servo_fp) {
                    float val;
                    int n = fscanf(servo_fp, "%f", &val);
                    if (n == 1) {
                        calib_servo1 = val;
                    } else {
                        /* printf("[CALIB-DEBUG] fscanf failed, n=%d, errno=%d\n", n, errno); */
                    }
                    fclose(servo_fp);
                } else {
                    /* printf("[CALIB-DEBUG] fopen /tmp/servo_calib.txt failed, errno=%d\n", errno); */
                }
            } else {
                /* 扫描测试模式：k2(J4) 从 100 扫到 180，每 200ms +10° */
                static float scan_angle = 100.0f;
                static int scan_dir = 1;
                scan_angle += scan_dir * 10.0f;
                if (scan_angle >= 180.0f) { scan_angle = 180.0f; scan_dir = -1; }
                if (scan_angle <= 100.0f) { scan_angle = 100.0f; scan_dir = 1; }
                calib_servo1 = scan_angle;
            }

            /* 每 200ms 发一次固定位置 + 可调舵机 */
            if (now_us - last_calib_cmd_us > 200000) {
                /* 控制指令: k1=50, k2=145 */
                uart_send_arm_target(calib_locked_tx, calib_locked_ty, calib_locked_tz,
                                     NRF_SERVO1_DEG, NRF_SERVO2_DEG, 0x01);
                last_calib_cmd_us = now_us;
                /* printf("[CALIB] J4(k2)=%.1f | roll=%+.2f yaw=%+.2f (locked roll=%.2f yaw=%.2f)\n",
                       calib_servo1, current_roll_deg, current_yaw_deg,
                       calib_locked_roll, calib_locked_yaw); */
            }
            return;  /* 跳过正常控制逻辑 */
        }
        fclose(calib_fp);
    }
    /* 退出标定模式 */
    if (calib_active) {
        calib_active = false;
        /* printf("[CALIB] 标定模式退出，恢复正常控制\n"); */
    }

    /* 计算 50ms 内角度历史的 A-inverse 平均值 */
    float avg_rel_roll = 0.0f, avg_rel_pitch = 0.0f, avg_rel_yaw = 0.0f;
    int avg_n = 0;
    if (g_r_init_set) {
        pthread_mutex_lock(&g_nrf24_state.mutex);
        int angle_count = g_nrf24_state.gy_angle_count;
        for (int i = 0; i < angle_count; i++) {
            int pos = (g_nrf24_state.gy_angle_idx - angle_count + i + NRF24_ANGLE_HIST_SIZE)
                      % NRF24_ANGLE_HIST_SIZE;
            float h_r = g_nrf24_state.gy_roll_hist[pos];
            float h_p = g_nrf24_state.gy_pitch_hist[pos];
            float h_y = g_nrf24_state.gy_yaw_hist[pos];
            cv::Mat R_h = eulerZYXToMat(h_r, h_p, h_y);
            cv::Mat R_h_rel = R_h * R_init.t();
            float hr, hp, hy;
            matToEulerZYX(R_h_rel, hr, hp, hy);
            avg_rel_roll += hr; avg_rel_pitch += hp; avg_rel_yaw += hy;
            avg_n++;
        }
        pthread_mutex_unlock(&g_nrf24_state.mutex);
        if (avg_n > 0) {
            avg_rel_roll  /= avg_n;
            avg_rel_pitch /= avg_n;
            avg_rel_yaw   /= avg_n;
        }
    }
    /* printf("[IMU-AVG] rel_roll=%+.2f rel_pitch=%+.2f rel_yaw=%+.2f (n=%d) valid=%d init=%d | nrf_rx=%u err=%u\n",
           avg_rel_roll, avg_rel_pitch, avg_rel_yaw, avg_n, (int)imu_valid, (int)g_r_init_set,
           nrf_rx_count, nrf_err_count); */

    if (!imu_valid) {
        /* printf("[NRF-STATE] 无效数据\n"); */
        return;
    }

    /* A-init 完成前不发令，防止 predictor 初始噪声误触发 */
    if (!g_r_init_set) {
        return;
    }

    /* A-init 完成前不发坐标指令（由 !g_r_init_set 在上文拦截） */

    /* 基准标定（A-inverse 后初始姿态已归零，标量 baseline 不再需要） */
    if (!pitch_baseline_set) {
        pitch_baseline_set = true;
        last_cmd_target_pitch = 0.0f;
        predictor.reset();
    }

    /* 用 wy 历史（10ms 分辨率）更新运动上下文并运行状态机 */
    ctx.update_from_hist(wy_hist, wy_count, 5);   /* 取最近 5 个 ≈ 50ms */
    MotionState curr_state = next_motion_state(prev_state, ctx);
    prev_state = curr_state;

    /* 更新终点预测 */
    predictor.update(wy, curr_state);

    /* 只在状态变化时打印 */
    if (curr_state != prev_print_state) {
        /* printf("[NRF-STATE] wx=%+.2f pred=%+.1f° | %s\n",
               wx, predictor.pred_delta_yaw, motion_state_name(curr_state)); */
        prev_print_state = curr_state;
    }

    /* ========== Pitch: 预测终点驱动的发令策略 ========== */
    bool is_stop = ctx.is_stop();

    /* ========== Yaw: 从 imu-main-test 恢复的完整状态机 ========== */
    if (!yaw_baseline_set) {
        yaw_baseline_set = true;
        last_cmd_target_yaw = 0.0f;
        predictor_yaw.reset();
    }

    ctx_yaw.update_from_hist(wz_hist, wz_count, 5);
    MotionState curr_state_yaw = next_motion_state(prev_state_yaw, ctx_yaw);
    prev_state_yaw = curr_state_yaw;
    predictor_yaw.update(wz, curr_state_yaw);

    if (curr_state_yaw != prev_print_state_yaw) {
        /* printf("[NRF-STATE] wz=%+.2f pred=%+.1f° | %s\n",
               wz, predictor_yaw.pred_delta_yaw, motion_state_name(curr_state_yaw)); */
        prev_print_state_yaw = curr_state_yaw;
    }

    bool is_stop_yaw = ctx_yaw.is_stop();

    /* ========== PnP 视觉零飘修正（只在完全静止态执行）==========
     * 当 pitch/yaw 都静止且 PnP 已连续 3 帧有效时，构造旋转修正矩阵
     * 叠加到 R_bias_total，并重新计算 rel_pitch/rel_yaw。
     */
    float pitch_delta = 0.0f, yaw_delta = 0.0f;
    bool do_pnp_correct = false;
    if (g_r_init_set && is_stop && is_stop_yaw) {
        pthread_mutex_lock(&g_nrf24_state.mutex);
        if (g_nrf24_state.pnp_correction_ready) {
            pitch_delta = 0.0f;  // 先关闭 pitch 修正
            yaw_delta   = -KI_PNP * g_nrf24_state.pnp_yaw_correction;
            g_nrf24_state.pnp_correction_ready = false;
            do_pnp_correct = true;
        }
        pthread_mutex_unlock(&g_nrf24_state.mutex);
    }
    if (do_pnp_correct) {
        if (R_bias_total.empty()) {
            R_bias_total = cv::Mat::eye(3, 3, CV_32F);
        }
        cv::Mat R_delta = eulerZYXToMat(0.0f, pitch_delta, yaw_delta);
        R_bias_total = R_delta * R_bias_total;

        if (imu_valid) {
            cv::Mat R_current = eulerZYXToMat(current_roll_deg, current_pitch_deg, current_yaw_deg);
            cv::Mat R_corrected = R_bias_total * R_current;
            cv::Mat R_rel_mat = R_corrected * R_init.t();
            matToEulerZYX(R_rel_mat, rel_roll, rel_pitch, rel_yaw);
        }
    }

    float target_pitch_deg = is_stop ? rel_pitch
                                     : predictor.get_target_yaw(rel_pitch);
    float delta_pitch_deg = normalize_angle_deg(target_pitch_deg - last_cmd_target_pitch);
    bool pitch_moved_enough = std::fabs(delta_pitch_deg) > CMD_PITCH_THRESHOLD_DEG;

    uint64_t pitch_interval = 200000;
    bool pitch_first_window  = (curr_state == STATE_ACCEL_TO_CONST);
    bool pitch_second_window = (curr_state == STATE_CONST_TO_DECEL);
    if (pitch_first_window || pitch_second_window) pitch_interval = 150000;
    bool pitch_interval_ok = (now_us - last_cmd_us) >= pitch_interval;

    bool pitch_should_cmd = false;
    if (pitch_interval_ok && pitch_moved_enough) {
        if (pitch_first_window || pitch_second_window || is_stop) {
            pitch_should_cmd = true;
            if (is_stop) predictor.reset();
        } else {
            pitch_should_cmd = true;
        }
    }

    float use_yaw = -rel_yaw;  // 极性修正
    float target_yaw_deg = is_stop_yaw ? use_yaw
                                       : predictor_yaw.get_target_yaw(use_yaw);
    float delta_yaw_deg = normalize_angle_deg(target_yaw_deg - last_cmd_target_yaw);
    bool yaw_moved_enough = std::fabs(delta_yaw_deg) > CMD_YAW_THRESHOLD_DEG;

    uint64_t yaw_interval = 200000;
    bool yaw_first_window  = (curr_state_yaw == STATE_ACCEL_TO_CONST);
    bool yaw_second_window = (curr_state_yaw == STATE_CONST_TO_DECEL);
    if (yaw_first_window || yaw_second_window) yaw_interval = 150000;
    bool yaw_interval_ok = (now_us - last_cmd_us_yaw) >= yaw_interval;

    bool yaw_should_cmd = false;
    if (yaw_interval_ok && yaw_moved_enough) {
        if (yaw_first_window) {
            yaw_should_cmd = true;
        } else if (is_stop_yaw) {
            // 静止态也允许发令（供 PnP 零飘修正驱动机械臂微动）
            yaw_should_cmd = true;
        } else {
            yaw_should_cmd = true;
            yaw_stop_cmd_sent = false;      // 头部重新运动，允许下次静止再发一次
            yaw_last_move_complete = false; // 头部运动态重置稳定计时
        }
    }

    static float last_tx = NRF_FACE_X_CM;
    static float last_ty = NRF_FACE_Y_CM;
    static float last_tz = NRF_FACE_Z_CM;

    bool should_cmd = pitch_should_cmd || yaw_should_cmd;
    printf("[CMD] pitch=%+.2f->%+.2f(d=%+.2f%s) yaw=%+.2f->%+.2f(d=%+.2f%s) | p_cmd=%d y_cmd=%d | tx=%.1f ty=%.1f tz=%.1f\n",
           rel_pitch, target_pitch_deg, delta_pitch_deg, pitch_moved_enough ? "" : "_thr",
           rel_yaw, target_yaw_deg, delta_yaw_deg, yaw_moved_enough ? "" : "_thr",
           pitch_should_cmd, yaw_should_cmd, last_tx, last_ty, last_tz);
    if (should_cmd) {
        float delta_pitch_deg = normalize_angle_deg(target_pitch_deg - nrf_baseline_roll_deg);
        float cum_pitch_offset = delta_pitch_deg * (float)M_PI / 180.0f;
        if (cum_pitch_offset > (float)M_PI / 2.0f)
            cum_pitch_offset = (float)M_PI / 2.0f;
        if (cum_pitch_offset < -(float)M_PI / 2.0f)
            cum_pitch_offset = -(float)M_PI / 2.0f;

        float delta_yaw_deg = normalize_angle_deg(target_yaw_deg - nrf_baseline_yaw_deg);
        float cum_yaw_offset = -delta_yaw_deg * (float)M_PI / 180.0f;
        if (cum_yaw_offset > (float)M_PI / 2.0f)
            cum_yaw_offset = (float)M_PI / 2.0f;
        if (cum_yaw_offset < -(float)M_PI / 2.0f)
            cum_yaw_offset = -(float)M_PI / 2.0f;

        const float l1 = 8.0f;
        const float l2 = 5.0f;
        const float l3 = 40.0f;
        const float l4 = 28.0f;
        float k = 1.6f;
        last_tx = l3 * std::sin(cum_yaw_offset) * std::cos(cum_pitch_offset);
        last_ty = l4 - l1 * std::sin(cum_pitch_offset) + l3 * std::cos(cum_pitch_offset) * std::cos(cum_yaw_offset);
        last_tz = l2 + l1 * std::cos(cum_pitch_offset * k) + l3 * std::sin(cum_pitch_offset * k) * std::cos(cum_yaw_offset);

        // 位置死区：坐标变化 < 2cm 不发令，抑制 Y 轴附近 atan2 敏感导致的微抖
        static float prev_sent_tx = 0.0f;
        static float prev_sent_ty = 0.0f;
        static float prev_sent_tz = 0.0f;
        static bool prev_sent_initialized = false;
        const float POS_DEADZONE_CM = 1.0f;

        bool pos_changed_enough = false;
        if (!prev_sent_initialized) {
            pos_changed_enough = true;
        } else {
            pos_changed_enough =
                std::fabs(last_tx - prev_sent_tx) >= POS_DEADZONE_CM ||
                std::fabs(last_ty - prev_sent_ty) >= POS_DEADZONE_CM ||
                std::fabs(last_tz - prev_sent_tz) >= POS_DEADZONE_CM;
        }
        if (!pos_changed_enough) {
            should_cmd = false;
        } else {
            prev_sent_tx = last_tx;
            prev_sent_ty = last_ty;
            prev_sent_tz = last_tz;
            prev_sent_initialized = true;
        }

        /* ========== 舵机控制 ==========
         * prediction（flag=0x00）时：坐标(tx,ty,tz)用预测值，但舵机保持上一次
         * 实际停止态（flag=0x01）算出的角度，避免高频prediction导致舵机抖动。
         */
        static const float SERVO1_BASELINE = 30.0f;
        static const float K_PITCH_SERVO = -1.2f;

        float curr_servo1 = SERVO1_BASELINE + K_PITCH_SERVO * delta_pitch_deg;
        if (curr_servo1 > 90.0f)  curr_servo1 = 90.0f;
        if (curr_servo1 < -90.0f) curr_servo1 = -90.0f;

        static const float SERVO2_BASELINE = 50.0f;
        static const float K_YAW_SERVO = 0.4f;

        float curr_servo2 = SERVO2_BASELINE + K_YAW_SERVO * delta_yaw_deg;
        if (curr_servo2 > 270.0f) curr_servo2 = 270.0f;
        if (curr_servo2 < 0.0f)   curr_servo2 = 0.0f;

        bool is_prediction = false;
        if (pitch_should_cmd && !is_stop) is_prediction = true;
        if (yaw_should_cmd && !is_stop_yaw) is_prediction = true;
        uint8_t flag = is_prediction ? 0x00 : 0x01;

        static float actual_servo1 = SERVO1_BASELINE;
        static float actual_servo2 = SERVO2_BASELINE;
        if (!is_prediction) {
            actual_servo1 = curr_servo1;
            actual_servo2 = curr_servo2;
        }

        float send_servo1 = is_prediction ? actual_servo1 : curr_servo1;
        float send_servo2 = is_prediction ? actual_servo2 : curr_servo2;

        int send_ret = uart_send_arm_target(last_tx, last_ty, last_tz,
                                            send_servo2, send_servo1, flag);
        if (send_ret == 0) {
            g_arm_target_yaw_deg.store(target_yaw_deg);
        }

        if (pitch_should_cmd) {
            last_cmd_target_pitch = target_pitch_deg;
            last_cmd_us = now_us;
        }
        if (yaw_should_cmd) {
            last_cmd_target_yaw = target_yaw_deg;
            last_cmd_us_yaw = now_us;
        }
    }

    /* ---------- 头部静止到位检测（替代下位机 complete）---------- */
    static bool was_stationary = false;
    static uint64_t stationary_since_us = 0;
    static uint64_t last_any_cmd_us = 0;

    if (should_cmd) {
        last_any_cmd_us = now_us;
    }

    /* 静止判断：滞后带（hysteresis），消灭边界抖动
     * 进入静止：瞬时 wx/wz 都 < 2°/s
     * 保持/退出静止：瞬时 wx/wz 任一 > 5°/s 才退出
     */
    static bool is_stationary_now = false;
    if (!is_stationary_now) {
        is_stationary_now = (std::fabs(wx) < 2.0f && std::fabs(wz) < 2.0f);
    } else {
        if (std::fabs(wx) > 5.0f || std::fabs(wz) > 5.0f)
            is_stationary_now = false;
    }

    if (is_stationary_now) {
        if (!was_stationary) {
            stationary_since_us = now_us;
            was_stationary = true;
        }
        g_head_stationary = 1;
    } else {
        was_stationary = false;
        g_head_stationary = 0;
    }

    g_arm_stable = (was_stationary &&
                    (now_us - stationary_since_us > 800000)) ? 1 : 0;

    /* 只在 stable 状态变化时打印，减少刷屏 */
    static int prev_arm_stable = -1;
    if (g_arm_stable != prev_arm_stable) {
        /* printf("[NRF-STATE] stable=%d (complete=%d) tx=%.1f ty=%.1f tz=%.1f | roll=%+.2f yaw=%+.2f | wx=%+.2f wz=%+.2f\n",
               g_arm_stable, g_uart_move_complete, last_tx, last_ty, last_tz, rel_roll, rel_yaw, wx, wz); */
        prev_arm_stable = g_arm_stable;
    }
}

static void estimate_and_draw_pose(uint8_t* nv12, int img_w, int img_h,
                                   const std::vector<cv::Point2f>& image_points,
                                   const std::vector<cv::Point3f>& object_points) {
    /* 根据当前图像分辨率动态缩放内参（基准标定分辨率 1920x1080） */
    static const float CALIB_W = 1920.0f;
    static const float CALIB_H = 1080.0f;
    float scale_x = (float)img_w / CALIB_W;
    float scale_y = (float)img_h / CALIB_H;
    cv::Mat K = CAMERA_MATRIX.clone();
    K.at<float>(0,0) *= scale_x;  // fx
    K.at<float>(1,1) *= scale_y;  // fy
    K.at<float>(0,2) *= scale_x;  // cx
    K.at<float>(1,2) *= scale_y;  // cy

    // PnP 连续有效帧计数器：3 帧有效后通知 IMU 控制线程可做零飘修正
    static int pnp_valid_cnt = 0;
    auto pnp_fail = [&]() {
        pnp_valid_cnt = 0;
        pthread_mutex_lock(&g_nrf24_state.mutex);
        g_nrf24_state.pnp_correction_ready = false;
        pthread_mutex_unlock(&g_nrf24_state.mutex);
    };
    auto pnp_success = [&]() {
        if (++pnp_valid_cnt >= 1) {
            pthread_mutex_lock(&g_nrf24_state.mutex);
            g_nrf24_state.pnp_correction_ready = true;
            pthread_mutex_unlock(&g_nrf24_state.mutex);
        }
    };

    if (image_points.size() < 4 || image_points.size() != object_points.size()) {
        pnp_fail();
        return;
    }

    cv::Mat rvec, tvec;
    if (have_prev_pose) {
        rvec = prev_rvec.clone();
        tvec = prev_tvec.clone();
    }
    bool success = cv::solvePnP(object_points, image_points,
                                K, DIST_COEFFS,
                                rvec, tvec, have_prev_pose,
                                cv::SOLVEPNP_ITERATIVE);
    if (!success) {
        pnp_fail();
        return;
    }

    // 重投影误差检查：坏帧直接丢弃，防止 3D 框畸变
    std::vector<cv::Point2f> reproj_pts;
    cv::projectPoints(object_points, rvec, tvec, K, DIST_COEFFS, reproj_pts);
    double reproj_error = 0.0;
    int npts = (int)image_points.size();
    for (int i = 0; i < npts; i++) {
        double dx = image_points[i].x - reproj_pts[i].x;
        double dy = image_points[i].y - reproj_pts[i].y;
        reproj_error += std::sqrt(dx*dx + dy*dy);
    }
    reproj_error /= npts;

    // PnP 调试打印（每10帧）——已注释
    /* static int pnp_debug_cnt = 0;
    if (++pnp_debug_cnt % 10 == 0) {
        printf("[PnP-Debug] reproj_err=%.1fpx | rvec(%.3f,%.3f,%.3f) tvec(%.1f,%.1f,%.1f)\n",
               reproj_error,
               rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2),
               tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
    } */

    if (reproj_error > 25.0) {
        static int bad_cnt = 0;
        if (++bad_cnt % 30 == 0) {
            printf("[Pose] Bad frame skipped, reprojection error=%.1fpx\n", reproj_error);
        }
        pnp_fail();
        return;
    }

    // 硬丢弃：OpenCV 相机坐标系 Z 正方向远离相机，tvec_z < 0 表示人脸在相机后方，是镜像解
    if (tvec.at<double>(2) < 0) {
        static int mirror_cnt = 0;
        if (++mirror_cnt % 30 == 0) {
            printf("[Pose] Mirror solution detected (tz=%.1f), dropped\n", tvec.at<double>(2));
        }
        pnp_fail();
        return;
    }

    // rvec 连续性检查：与上一帧旋转角差 > 60° 则丢弃并重置滤波器（放在滤波之前）
    if (have_prev_pose) {
        cv::Mat R_curr, R_prev;
        cv::Rodrigues(rvec, R_curr);
        cv::Rodrigues(prev_rvec, R_prev);
        cv::Mat R_rel = R_curr * R_prev.t();
        double trace = R_rel.at<double>(0,0) + R_rel.at<double>(1,1) + R_rel.at<double>(2,2);
        double cos_half = std::min(1.0, std::max(-1.0, (trace - 1.0) / 2.0));
        double angle_diff = std::acos(cos_half);
        if (angle_diff > M_PI / 3) {
            static int jump_cnt = 0;
            if (++jump_cnt % 30 == 0) {
                printf("[Pose] Jump detected (%.0f deg), fallback to prev pose & reset filter\n", angle_diff * 180.0 / M_PI);
            }
            rvec = prev_rvec.clone();
            tvec = prev_tvec.clone();
            pose_filter_init = false;  // 重置滤波器，防止正确解和镜像解被稀释
        }
    }

    prev_rvec = rvec.clone();
    prev_tvec = tvec.clone();
    have_prev_pose = true;

    // 一键开关 OneEuroFilter：设为 0 禁用滤波，直接用过滤前的 rvec/tvec
    #define PNP_USE_FILTER 0
    #if PNP_USE_FILTER
    if (!pose_filter_init) {
        flt_rx.reset(rvec.at<double>(0));
        flt_ry.reset(rvec.at<double>(1));
        flt_rz.reset(rvec.at<double>(2));
        flt_tx.reset(tvec.at<double>(0));
        flt_ty.reset(tvec.at<double>(1));
        flt_tz.reset(tvec.at<double>(2));
        pose_filter_init = true;
    }
    cv::Mat rvec_f = (cv::Mat_<double>(3,1) <<
        flt_rx.filter(rvec.at<double>(0)),
        flt_ry.filter(rvec.at<double>(1)),
        flt_rz.filter(rvec.at<double>(2)));
    cv::Mat tvec_f = (cv::Mat_<double>(3,1) <<
        flt_tx.filter(tvec.at<double>(0)),
        flt_ty.filter(tvec.at<double>(1)),
        flt_tz.filter(tvec.at<double>(2)));
    #else
    cv::Mat rvec_f = rvec.clone();
    cv::Mat tvec_f = tvec.clone();
    #endif

    // ========== (1) 逆解算：相机在人脸坐标系中的位姿 ==========
    // solvePnP 给出的是 人脸→相机 的变换: P_cam = R * P_obj + tvec
    // 逆解算得到 相机在人脸坐标系中 的位姿:
    //   R_cam_in_obj = R^T
    //   t_cam_in_obj = -R^T * tvec
    cv::Mat R_obj2cam_raw;
    cv::Rodrigues(rvec, R_obj2cam_raw);
    cv::Mat R_cam_in_obj = R_obj2cam_raw.t();
    cv::Mat t_cam_in_obj = -R_cam_in_obj * tvec;

    // 从 R_cam_in_obj 提取欧拉角 (ZYX: Yaw-Pitch-Roll, 单位rad)
    double cam_yaw_obj=0, cam_pitch_obj=0, cam_roll_obj=0;
    {
        double sy = std::sqrt(R_cam_in_obj.at<double>(0,0)*R_cam_in_obj.at<double>(0,0)
                            + R_cam_in_obj.at<double>(1,0)*R_cam_in_obj.at<double>(1,0));
        if (sy > 1e-6) {
            cam_pitch_obj = std::atan2(R_cam_in_obj.at<double>(2,1), R_cam_in_obj.at<double>(2,2));
            cam_yaw_obj   = std::atan2(-R_cam_in_obj.at<double>(2,0), sy);
            cam_roll_obj  = std::atan2(R_cam_in_obj.at<double>(1,0), R_cam_in_obj.at<double>(0,0));
        } else {
            cam_pitch_obj = std::atan2(-R_cam_in_obj.at<double>(1,2), R_cam_in_obj.at<double>(1,1));
            cam_yaw_obj   = std::atan2(-R_cam_in_obj.at<double>(2,0), sy);
        }
    }

    // 从 R_cam_in_obj 提取四元数
    double qw_obj, qx_obj, qy_obj, qz_obj;
    {
        double trace = R_cam_in_obj.at<double>(0,0) + R_cam_in_obj.at<double>(1,1) + R_cam_in_obj.at<double>(2,2);
        if (trace > 0) {
            double s = 0.5 / std::sqrt(trace + 1.0);
            qw_obj = 0.25 / s;
            qx_obj = (R_cam_in_obj.at<double>(2,1) - R_cam_in_obj.at<double>(1,2)) * s;
            qy_obj = (R_cam_in_obj.at<double>(0,2) - R_cam_in_obj.at<double>(2,0)) * s;
            qz_obj = (R_cam_in_obj.at<double>(1,0) - R_cam_in_obj.at<double>(0,1)) * s;
        } else {
            qw_obj = 1.0; qx_obj = qy_obj = qz_obj = 0.0;
        }
    }

    // ========== (2) α坐标系精确变换 ==========
    // 人脸坐标系 -> α坐标系 旋转矩阵 (固定轴向关系):
    //   X_f(右) → X_a(右)
    //   Y_f(下) → -Z_a(下)
    //   Z_f(前/远离相机) → -Y_a(后, 因α Y向前=朝向相机)
    static const cv::Mat R_af = (cv::Mat_<double>(3,3) <<
        1.0,  0.0,  0.0,
        0.0,  0.0, -1.0,
        0.0, -1.0,  0.0);

    // O点(膈俞穴上方5cm)在头部坐标系中的固定位置 (单位mm)
    // 基于GB/T 10000-1988 P50男性数据估算: O点在面部原点下方150mm、后方180mm
    static const cv::Mat d_O_in_head = (cv::Mat_<double>(3,1) << 0.0, 150.0, 180.0);

    // 头部在α坐标系中的实时旋转 = 头部→相机→α
    cv::Mat R_F2A = R_af * R_obj2cam_raw;

    // 精确模型: 相机在α坐标系中的位置 = R_F2A × (相机在头部系中的位置 - O点在头部系中的位置)
    cv::Mat t_cam_in_alpha = R_F2A * (t_cam_in_obj - d_O_in_head);

    double cox = t_cam_in_obj.at<double>(0);
    double coy = t_cam_in_obj.at<double>(1);
    double coz = t_cam_in_obj.at<double>(2);
    double cax = t_cam_in_alpha.at<double>(0);
    double cay = t_cam_in_alpha.at<double>(1);
    double caz = t_cam_in_alpha.at<double>(2);
    // ========== 逆解算 + α坐标系变换 结束 ==========

    /*
     * Fixed camera/model mounting calibration.
     *
     * Do not add offsets directly to Rodrigues-vector components: rvec is an
     * axis-angle representation, not an Euler-angle vector. Compose the fixed
     * pitch and yaw corrections as rotations in the camera frame instead.
     *
     * The previous calibration was:
     *   rvec.x += -0.10 rad
     *   pnp_yaw_correction = head_yaw - 14 deg
     * These are now represented by one matrix:
     *   R_calib = Ry(-14 deg) * Rx(-0.10 rad) * R_face2cam_raw
     */
    static const double MOUNT_PITCH_RAD = -0.10;
    static const double MOUNT_YAW_RAD = -14.0 * M_PI / 180.0;
    static const cv::Mat R_mount = []() {
        const double cp = std::cos(MOUNT_PITCH_RAD);
        const double sp = std::sin(MOUNT_PITCH_RAD);
        const double cy = std::cos(MOUNT_YAW_RAD);
        const double sy = std::sin(MOUNT_YAW_RAD);

        cv::Mat R_pitch = (cv::Mat_<double>(3,3) <<
            1.0, 0.0, 0.0,
            0.0,  cp, -sp,
            0.0,  sp,  cp);
        cv::Mat R_yaw = (cv::Mat_<double>(3,3) <<
             cy, 0.0,  sy,
            0.0, 1.0, 0.0,
            -sy, 0.0,  cy);
        return R_yaw * R_pitch;
    }();

    cv::Mat R_face2cam_raw;
    cv::Rodrigues(rvec_f, R_face2cam_raw);
    cv::Mat R_calib = R_mount * R_face2cam_raw;
    cv::Mat rvec_calib;
    cv::Rodrigues(R_calib, rvec_calib);

    double sy = std::sqrt(R_calib.at<double>(0,0) * R_calib.at<double>(0,0)
                          + R_calib.at<double>(1,0) * R_calib.at<double>(1,0));
    bool singular = sy < 1e-6;
    double head_pitch = 0, head_yaw = 0, head_roll = 0;
    if (!singular) {
        head_pitch = std::atan2(R_calib.at<double>(2,1), R_calib.at<double>(2,2));
        head_yaw   = std::atan2(-R_calib.at<double>(2,0), sy);
        head_roll  = std::atan2(R_calib.at<double>(1,0), R_calib.at<double>(0,0));
    } else {
        head_pitch = std::atan2(-R_calib.at<double>(1,2), R_calib.at<double>(1,1));
        head_yaw   = std::atan2(-R_calib.at<double>(2,0), sy);
        head_roll  = 0;
    }

    cv::Mat R_face2cam = R_calib;
    cv::Mat R_cam2face = R_face2cam.t();
    cv::Mat cam_pos = -R_cam2face * tvec_f;

    double tx = tvec_f.at<double>(0);
    double ty = tvec_f.at<double>(1);
    double tz = tvec_f.at<double>(2);
    double pos_yaw_err = std::atan2(tx, tz);
    double pos_pitch_err = std::atan2(ty, tz);

    // PnP result ready -- log latency here (before drawing) ——已注释
    double hp_deg = head_pitch * 180.0 / M_PI;
    double hy_deg = head_yaw * 180.0 / M_PI;
    double hr_deg = head_roll * 180.0 / M_PI;
    float arm_yaw_deg = g_arm_target_yaw_deg.load();
    float yaw_compensation_deg =
        interpolate_pnp_yaw_compensation(arm_yaw_deg);
    double corrected_hy_deg = hy_deg + yaw_compensation_deg;

    /* PnP yaw 多点标定：控制线程移动机械臂，视觉线程只负责采样。 */
    {
        static bool active = false;
        static int sample_idx = -1;
        static int previous_phase = PNP_CALIB_INACTIVE;
        static double yaw_sum = 0.0;
        static int yaw_count = 0;

        int calib_mode = read_calib_mode();
        uint64_t now_us = get_us();
        cv::Mat y_mat(img_h, img_w, CV_8UC1, nv12);

        if (calib_mode == 3) {
            if (!active) {
                active = true;
                sample_idx = -1;
                previous_phase = PNP_CALIB_INACTIVE;
                yaw_sum = 0.0;
                yaw_count = 0;

                FILE *fp = fopen("/tmp/pnp_yaw_calib_negative.csv", "w");
                if (fp) {
                    fprintf(fp, "target_yaw_deg,pnp_yaw_avg_deg,sample_count\n");
                    fclose(fp);
                }
                printf("[PnP-Calib] Visual recorder started\n");
            }

            int target_idx = g_pnp_calib_target_idx.load();
            int phase = g_pnp_calib_phase.load();
            uint64_t phase_start = g_pnp_calib_phase_start_us.load();

            if (previous_phase == PNP_CALIB_SAMPLE &&
                (phase != PNP_CALIB_SAMPLE || target_idx != sample_idx) &&
                sample_idx >= 0 && sample_idx < PNP_CALIB_TARGET_COUNT) {
                double average = yaw_count > 0 ? yaw_sum / yaw_count : 0.0;
                FILE *fp = fopen("/tmp/pnp_yaw_calib_negative.csv", "a");
                if (fp) {
                    fprintf(fp, "%d,%.4f,%d\n",
                            PNP_CALIB_TARGETS[sample_idx], average, yaw_count);
                    fclose(fp);
                }
                printf("[PnP-Calib] Target %+d deg -> PnP avg %+.4f deg (%d samples)\n",
                       PNP_CALIB_TARGETS[sample_idx], average, yaw_count);
                yaw_sum = 0.0;
                yaw_count = 0;
            }

            if (target_idx >= 0 && target_idx < PNP_CALIB_TARGET_COUNT) {
                int target = PNP_CALIB_TARGETS[target_idx];
                uint64_t elapsed_us = phase_start > 0 ? now_us - phase_start : 0;
                char line[128];

                if (phase == PNP_CALIB_PREPARE) {
                    double remain = (PNP_CALIB_PREPARE_US > elapsed_us)
                        ? (PNP_CALIB_PREPARE_US - elapsed_us) / 1000000.0 : 0.0;
                    snprintf(line, sizeof(line),
                             "PnP CALIB: arm yaw %+d, face camera, prepare %.1fs",
                             target, remain);
                } else if (phase == PNP_CALIB_SAMPLE) {
                    if (sample_idx != target_idx) {
                        sample_idx = target_idx;
                        yaw_sum = 0.0;
                        yaw_count = 0;
                    }
                    yaw_sum += hy_deg;
                    yaw_count++;
                    double remain = (PNP_CALIB_SAMPLE_US > elapsed_us)
                        ? (PNP_CALIB_SAMPLE_US - elapsed_us) / 1000000.0 : 0.0;
                    double running_avg = yaw_count > 0 ? yaw_sum / yaw_count : 0.0;
                    snprintf(line, sizeof(line),
                             "PnP CALIB: arm yaw %+d, sample %.1fs, PnP avg %+.2f",
                             target, remain, running_avg);
                } else {
                    snprintf(line, sizeof(line), "PnP CALIB: moving arm...");
                }

                cv::putText(y_mat, line, cv::Point(10, 100),
                            cv::FONT_HERSHEY_SIMPLEX, 0.9,
                            cv::Scalar(255), 2);
            } else if (phase == PNP_CALIB_COMPLETE) {
                cv::putText(y_mat,
                            "PnP CALIB COMPLETE: /tmp/pnp_yaw_calib_negative.csv",
                            cv::Point(10, 100),
                            cv::FONT_HERSHEY_SIMPLEX, 0.9,
                            cv::Scalar(255), 2);
            }
            previous_phase = phase;
        } else if (active) {
            active = false;
            sample_idx = -1;
            previous_phase = PNP_CALIB_INACTIVE;
            yaw_sum = 0.0;
            yaw_count = 0;
            printf("[PnP-Calib] Stopped\n");
        }
    }

    // PnP → IMU 控制：固定安装校正后，再按机械臂目标 yaw 查表补偿
    pthread_mutex_lock(&g_nrf24_state.mutex);
    g_nrf24_state.pnp_yaw_correction   = (float)corrected_hy_deg;
    g_nrf24_state.pnp_pitch_correction = (float)hp_deg;
    g_nrf24_state.pnp_valid = true;
    pthread_mutex_unlock(&g_nrf24_state.mutex);

    pnp_success();

    double pye_deg = pos_yaw_err * 180.0 / M_PI;
    double ppe_deg = pos_pitch_err * 180.0 / M_PI;
    (void)hp_deg; (void)hy_deg; (void)hr_deg; (void)pye_deg; (void)ppe_deg;
    /* struct timespec ts_now;
    clock_gettime(CLOCK_MONOTONIC, &ts_now);
    uint64_t now_us = ts_now.tv_sec * 1000000ULL + ts_now.tv_nsec / 1000;
    uint64_t elapsed_us = now_us - g_frame_start_us;
    printf("[Latency] Frame->PnP: %.2f ms | Pose(P%.1f Y%.1f R%.1f) | PosErr(Yaw=%.1f Pitch=%.1f)\n",
           elapsed_us / 1000.0, hp_deg, hy_deg, hr_deg, pye_deg, ppe_deg); */

    // 在画面左上角醒目显示 Yaw/Pitch/Roll + rvec + 四元数（黑底白字）
    double rx = rvec_calib.at<double>(0);
    double ry = rvec_calib.at<double>(1);
    double rz = rvec_calib.at<double>(2);

    // 从旋转矩阵计算四元数
    double qw, qx, qy, qz;
    double trace = R_face2cam.at<double>(0,0) + R_face2cam.at<double>(1,1) + R_face2cam.at<double>(2,2);
    if (trace > 0) {
        double s = 0.5 / sqrt(trace + 1.0);
        qw = 0.25 / s;
        qx = (R_face2cam.at<double>(2,1) - R_face2cam.at<double>(1,2)) * s;
        qy = (R_face2cam.at<double>(0,2) - R_face2cam.at<double>(2,0)) * s;
        qz = (R_face2cam.at<double>(1,0) - R_face2cam.at<double>(0,1)) * s;
    } else {
        if (R_face2cam.at<double>(0,0) > R_face2cam.at<double>(1,1) && R_face2cam.at<double>(0,0) > R_face2cam.at<double>(2,2)) {
            double s = 2.0 * sqrt(1.0 + R_face2cam.at<double>(0,0) - R_face2cam.at<double>(1,1) - R_face2cam.at<double>(2,2));
            qw = (R_face2cam.at<double>(2,1) - R_face2cam.at<double>(1,2)) / s;
            qx = 0.25 * s;
            qy = (R_face2cam.at<double>(0,1) + R_face2cam.at<double>(1,0)) / s;
            qz = (R_face2cam.at<double>(0,2) + R_face2cam.at<double>(2,0)) / s;
        } else if (R_face2cam.at<double>(1,1) > R_face2cam.at<double>(2,2)) {
            double s = 2.0 * sqrt(1.0 + R_face2cam.at<double>(1,1) - R_face2cam.at<double>(0,0) - R_face2cam.at<double>(2,2));
            qw = (R_face2cam.at<double>(0,2) - R_face2cam.at<double>(2,0)) / s;
            qx = (R_face2cam.at<double>(0,1) + R_face2cam.at<double>(1,0)) / s;
            qy = 0.25 * s;
            qz = (R_face2cam.at<double>(1,2) + R_face2cam.at<double>(2,1)) / s;
        } else {
            double s = 2.0 * sqrt(1.0 + R_face2cam.at<double>(2,2) - R_face2cam.at<double>(0,0) - R_face2cam.at<double>(1,1));
            qw = (R_face2cam.at<double>(1,0) - R_face2cam.at<double>(0,1)) / s;
            qx = (R_face2cam.at<double>(0,2) + R_face2cam.at<double>(2,0)) / s;
            qy = (R_face2cam.at<double>(1,2) + R_face2cam.at<double>(2,1)) / s;
            qz = 0.25 * s;
        }
    }

    char line1[80], line2[80], line3[80], line4[80];
    snprintf(line1, sizeof(line1),
             "Yaw=%.1f Raw=%.1f Arm=%.1f Comp=%+.1f",
             corrected_hy_deg, hy_deg, arm_yaw_deg, yaw_compensation_deg);
    snprintf(line2, sizeof(line2), "rvec=(%.2f, %.2f, %.2f)", rx, ry, rz);
    snprintf(line3, sizeof(line3), "quat=(%.2f, %.2f, %.2f, %.2f)", qw, qx, qy, qz);
    // 重点标出 α坐标系下相机位置 (单位cm)
    snprintf(line4, sizeof(line4), "cam_alpha=(%.1f, %.1f, %.1f) cm", cax/10.0, cay/10.0, caz/10.0);

    cv::Mat y_mat(img_h, img_w, CV_8UC1, nv12);
    int baseline = 0;
    double font_scale_big = 1.5;
    double font_scale_small = 1.1;
    int thick_big = 3;
    int thick_small = 2;

    cv::Size sz1 = cv::getTextSize(line1, cv::FONT_HERSHEY_SIMPLEX, font_scale_big, thick_big, &baseline);
    cv::Size sz2 = cv::getTextSize(line2, cv::FONT_HERSHEY_SIMPLEX, font_scale_small, thick_small, &baseline);
    cv::Size sz3 = cv::getTextSize(line3, cv::FONT_HERSHEY_SIMPLEX, font_scale_small, thick_small, &baseline);
    cv::Size sz4 = cv::getTextSize(line4, cv::FONT_HERSHEY_SIMPLEX, font_scale_big, thick_big, &baseline);
    int max_w = std::max({sz1.width, sz2.width, sz3.width, sz4.width});
    int pad_x = 12;
    int pad_y = 6;
    int line_h1 = sz1.height + 8;
    int line_h2 = sz2.height + 6;
    int line_h3 = sz3.height + 6;
    int line_h4 = sz4.height + 8;  // 和line1一样大
    int total_h = line_h1 + line_h2 + line_h3 + line_h4 + pad_y;

    // 黑色背景
    cv::rectangle(y_mat, cv::Point(8, 8),
                  cv::Point(8 + max_w + pad_x * 2, 8 + total_h),
                  cv::Scalar(0), -1);
    // 四行白色文字（第4行和line1同字号，重点标出）
    int y_pos = 8 + pad_y + sz1.height;
    cv::putText(y_mat, line1, cv::Point(8 + pad_x, y_pos),
                cv::FONT_HERSHEY_SIMPLEX, font_scale_big, cv::Scalar(255), thick_big);
    y_pos += line_h2;
    cv::putText(y_mat, line2, cv::Point(8 + pad_x, y_pos),
                cv::FONT_HERSHEY_SIMPLEX, font_scale_small, cv::Scalar(255), thick_small);
    y_pos += line_h3;
    cv::putText(y_mat, line3, cv::Point(8 + pad_x, y_pos),
                cv::FONT_HERSHEY_SIMPLEX, font_scale_small, cv::Scalar(255), thick_small);
    y_pos += line_h4;
    cv::putText(y_mat, line4, cv::Point(8 + pad_x, y_pos),
                cv::FONT_HERSHEY_SIMPLEX, font_scale_big, cv::Scalar(255), thick_big);

    std::vector<cv::Point2f> proj_origin;
    cv::projectPoints(std::vector<cv::Point3f>{{0,0,0}}, rvec_calib, tvec_f, K, DIST_COEFFS, proj_origin);
    cv::Point2f origin = proj_origin[0];

    std::vector<cv::Point3f> axis_3d = {{40,0,0}, {0,40,0}, {0,0,40}};
    std::vector<cv::Point2f> proj_axis;
    cv::projectPoints(axis_3d, rvec_calib, tvec_f, K, DIST_COEFFS, proj_axis);

    uint8_t yr, ur, vr, yg, ug, vg, yb, ub, vb;
    rgb_to_yuv(255, 0, 0, yr, ur, vr);
    rgb_to_yuv(0, 255, 0, yg, ug, vg);
    rgb_to_yuv(0, 0, 255, yb, ub, vb);

    draw_line_nv12(nv12, img_w, img_h, (int)origin.x, (int)origin.y,
                   (int)proj_axis[0].x, (int)proj_axis[0].y, yr, ur, vr);
    draw_line_nv12(nv12, img_w, img_h, (int)origin.x, (int)origin.y,
                   (int)proj_axis[1].x, (int)proj_axis[1].y, yg, ug, vg);
    draw_line_nv12(nv12, img_w, img_h, (int)origin.x, (int)origin.y,
                   (int)proj_axis[2].x, (int)proj_axis[2].y, yb, ub, vb);

    // ===== 调试绘制：2D 关键点(白方块+编号) vs 重投影点(红叉) =====
    {
        uint8_t yw, uw, vw, yr, ur, vr;
        rgb_to_yuv(255, 255, 255, yw, uw, vw);  // 白色
        rgb_to_yuv(255, 0, 0, yr, ur, vr);       // 红色
        for (size_t i = 0; i < image_points.size(); i++) {
            int x2d = (int)image_points[i].x;
            int y2d = (int)image_points[i].y;
            int xrp = (int)reproj_pts[i].x;
            int yrp = (int)reproj_pts[i].y;
            // 2D 检测点：白方块 5x5
            for (int dy = -2; dy <= 2; dy++)
                for (int dx = -2; dx <= 2; dx++)
                    set_pixel_nv12(nv12, img_w, img_h, x2d+dx, y2d+dy, yw, uw, vw);
            // 重投影点：红叉 7x7
            for (int d = -3; d <= 3; d++) {
                set_pixel_nv12(nv12, img_w, img_h, xrp+d, yrp+d, yr, ur, vr);
                set_pixel_nv12(nv12, img_w, img_h, xrp+d, yrp-d, yr, ur, vr);
            }
            // 编号（Y平面白字）
            char klabel[16];
            snprintf(klabel, sizeof(klabel), "%zu", i);
            cv::putText(y_mat, klabel, cv::Point(x2d + 6, y2d - 6),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255), 1);
        }
    }
    // ===== 调试绘制结束 =====

    // 一键开关：只画坐标轴（调试用），设为 0 画完整立方体
    #define PNP_DRAW_CUBE 1
    #if PNP_DRAW_CUBE
    // 立方体：前面 Z=-100（靠近相机，对齐鼻尖 Z=-90），后面 Z=-20（远离相机）
    // 宽80mm×高105mm×深80mm，更贴合人脸轮廓
    std::vector<cv::Point3f> cube_3d = {
        {-40,-45,-20}, {40,-45,-20}, {40,60,-20}, {-40,60,-20},    // 0-3: 后面（远离相机）
        {-40,-45,-100}, {40,-45,-100}, {40,60,-100}, {-40,60,-100}  // 4-7: 前面（靠近相机）
    };
    std::vector<cv::Point2f> proj_cube;
    cv::projectPoints(cube_3d, rvec_calib, tvec_f, K, DIST_COEFFS, proj_cube);

    // 前表面（靠近相机）- 亮绿色
    uint8_t yf, uf, vf;
    rgb_to_yuv(0, 255, 0, yf, uf, vf);
    // 连接线 - 青色
    uint8_t ye, ue, ve;
    rgb_to_yuv(0, 255, 255, ye, ue, ve);
    // 后表面（远离相机）- 暗绿色
    uint8_t yd, ud, vd;
    rgb_to_yuv(0, 128, 0, yd, ud, vd);

    // 后表面边（暗色，可能被遮挡）
    const int back_edges[4][2] = {{0,1},{1,2},{2,3},{3,0}};
    for (int i = 0; i < 4; i++) {
        int s = back_edges[i][0], e = back_edges[i][1];
        draw_line_nv12(nv12, img_w, img_h,
            (int)proj_cube[s].x, (int)proj_cube[s].y,
            (int)proj_cube[e].x, (int)proj_cube[e].y,
            yd, ud, vd);
    }
    // 连接线（中亮色）
    const int conn_edges[4][2] = {{0,4},{1,5},{2,6},{3,7}};
    for (int i = 0; i < 4; i++) {
        int s = conn_edges[i][0], e = conn_edges[i][1];
        draw_line_nv12(nv12, img_w, img_h,
            (int)proj_cube[s].x, (int)proj_cube[s].y,
            (int)proj_cube[e].x, (int)proj_cube[e].y,
            ye, ue, ve);
    }
    // 前表面边（亮绿色，最显眼）
    const int front_edges[4][2] = {{4,5},{5,6},{6,7},{7,4}};
    for (int i = 0; i < 4; i++) {
        int s = front_edges[i][0], e = front_edges[i][1];
        draw_line_nv12(nv12, img_w, img_h,
            (int)proj_cube[s].x, (int)proj_cube[s].y,
            (int)proj_cube[e].x, (int)proj_cube[e].y,
            yf, uf, vf);
    }
    #endif

    // ===== 验证打印：cube 顶点的相机坐标Z和2D投影坐标 =====
    {
        double z_cam[8];
        for (int i = 0; i < 8; i++) {
            cv::Mat P_obj = (cv::Mat_<double>(3,1) << cube_3d[i].x, cube_3d[i].y, cube_3d[i].z);
            cv::Mat P_cam = R_face2cam * P_obj + tvec_f;
            z_cam[i] = P_cam.at<double>(2);
        }
        double z_back_avg  = (z_cam[0]+z_cam[1]+z_cam[2]+z_cam[3]) / 4.0;  // Z=-20
        double z_front_avg = (z_cam[4]+z_cam[5]+z_cam[6]+z_cam[7]) / 4.0;  // Z=-100

        // ========== (3) 绘制逆解算 + α坐标系结果 ==========
        char vline1[128], vline2[128], vline3[128], vline4[128];
        snprintf(vline1, sizeof(vline1), "Zback:%.0f Zfront:%.0f ZRAT:%.2f",
                 z_back_avg, z_front_avg, z_back_avg / z_front_avg);
        snprintf(vline2, sizeof(vline2), "P0:%.0f,%.0f P1:%.0f,%.0f P4:%.0f,%.0f P5:%.0f,%.0f",
                 proj_cube[0].x, proj_cube[0].y,
                 proj_cube[1].x, proj_cube[1].y,
                 proj_cube[4].x, proj_cube[4].y,
                 proj_cube[5].x, proj_cube[5].y);
        snprintf(vline3, sizeof(vline3),
                 "cam_obj=(%.1f,%.1f,%.1f)cm Y%.1f P%.1f R%.1f",
                 cox/10.0, coy/10.0, coz/10.0,
                 cam_yaw_obj * 180.0 / M_PI,
                 cam_pitch_obj * 180.0 / M_PI,
                 cam_roll_obj * 180.0 / M_PI);
        snprintf(vline4, sizeof(vline4),
                 "cam_alpha=(%.1f,%.1f,%.1f)cm",
                 cax/10.0, cay/10.0, caz/10.0);

        int vbaseline = 0;
        double vfont = 0.75;
        int vthick = 1;
        cv::Size vsz1 = cv::getTextSize(vline1, cv::FONT_HERSHEY_SIMPLEX, vfont, vthick, &vbaseline);
        cv::Size vsz2 = cv::getTextSize(vline2, cv::FONT_HERSHEY_SIMPLEX, vfont, vthick, &vbaseline);
        cv::Size vsz3 = cv::getTextSize(vline3, cv::FONT_HERSHEY_SIMPLEX, vfont, vthick, &vbaseline);
        cv::Size vsz4 = cv::getTextSize(vline4, cv::FONT_HERSHEY_SIMPLEX, vfont, vthick, &vbaseline);
        int vmax_w = std::max({vsz1.width, vsz2.width, vsz3.width, vsz4.width});
        int vpad = 8;
        int vgap = 4;
        int vtotal_h = vsz1.height + vsz2.height + vsz3.height + vsz4.height + vgap * 3 + 8;
        int vx = 8;
        int vy = img_h - vtotal_h - 8;

        cv::rectangle(y_mat, cv::Point(vx, vy),
                      cv::Point(vx + vmax_w + vpad*2, vy + vtotal_h),
                      cv::Scalar(0), -1);
        int vy_off = vy + vsz1.height + 4;
        cv::putText(y_mat, vline1, cv::Point(vx + vpad, vy_off),
                    cv::FONT_HERSHEY_SIMPLEX, vfont, cv::Scalar(255), vthick);
        vy_off += vsz2.height + vgap;
        cv::putText(y_mat, vline2, cv::Point(vx + vpad, vy_off),
                    cv::FONT_HERSHEY_SIMPLEX, vfont, cv::Scalar(255), vthick);
        vy_off += vsz3.height + vgap;
        cv::putText(y_mat, vline3, cv::Point(vx + vpad, vy_off),
                    cv::FONT_HERSHEY_SIMPLEX, vfont, cv::Scalar(255), vthick);
        vy_off += vsz4.height + vgap;
        cv::putText(y_mat, vline4, cv::Point(vx + vpad, vy_off),
                    cv::FONT_HERSHEY_SIMPLEX, vfont, cv::Scalar(255), vthick);
    }
    // ===== 验证打印结束 =====

    draw_pose_indicator(nv12, img_w, img_h, head_yaw, head_pitch);

    /* [DEPRECATED] 旧 PnP-based 控制链路已注释掉，改用 NRF24 IMU 控制链路
    // ========== 人脸跟随验证模式（舵机固定，只调机械臂位置） ==========
    // ... 原控制逻辑已注释 ...
    */
}

// ========== FP16/FP32转换 ==========
static float fp16_to_fp32(uint16_t h) {
    uint32_t sign = (h >> 15) & 0x1;
    uint32_t exp = (h >> 10) & 0x1F;
    uint32_t mant = h & 0x3FF;
    if (exp == 0) return sign ? -0.0f : 0.0f;
    if (exp == 31) return 0.0f / 0.0f;
    exp = exp - 15 + 127;
    uint32_t f = (sign << 31) | (exp << 23) | (mant << 13);
    float r; memcpy(&r, &f, 4); return r;
}

static uint16_t fp32_to_fp16(float f) {
    uint32_t x; memcpy(&x, &f, 4);
    uint32_t sign = (x >> 31) & 0x1;
    uint32_t exp = (x >> 23) & 0xFF;
    uint32_t mant = x & 0x7FFFFF;
    if (exp == 0) return sign << 15;
    if (exp == 255) {
        // NaN vs Inf: mant!=0 -> NaN, mant==0 -> Inf
        if (mant != 0) return (sign << 15) | 0x7E00;
        return (sign << 15) | 0x7C00;
    }
    int new_exp = (int)exp - 127 + 15;
    if (new_exp >= 31) return (sign << 15) | 0x7C00;
    if (new_exp <= 0) return sign << 15;
    uint32_t new_mant = mant >> 13;
    return (sign << 15) | ((uint32_t)new_exp << 10) | new_mant;
}

// ========== IoU + NMS ==========
static float compute_iou(const PoseDetection& a, const PoseDetection& b) {
    float x1 = std::max(a.x1, b.x1), y1 = std::max(a.y1, b.y1);
    float x2 = std::min(a.x2, b.x2), y2 = std::min(a.y2, b.y2);
    float inter = std::max(0.0f, x2-x1) * std::max(0.0f, y2-y1);
    float area_a = (a.x2-a.x1)*(a.y2-a.y1);
    float area_b = (b.x2-b.x1)*(b.y2-b.y1);
    return inter / (area_a + area_b - inter + 1e-6f);
}

static void nms(std::vector<PoseDetection>& dets, float thresh) {
    std::sort(dets.begin(), dets.end(), [](const auto& a, const auto& b){return a.score>b.score;});
    std::vector<bool> keep(dets.size(), true);
    for (size_t i=0; i<dets.size(); i++) if (keep[i])
        for (size_t j=i+1; j<dets.size(); j++) if (keep[j])
            if (compute_iou(dets[i], dets[j]) > thresh) keep[j] = false;
    std::vector<PoseDetection> res;
    for (size_t i=0; i<dets.size(); i++) if (keep[i]) res.push_back(dets[i]);
    dets.swap(res);
}

// ========== 后处理 ==========

// FACE模式后处理：[1,65,8400] 20个关键点，只保留6个有效点
static void post_process_face(uint16_t* fp16_data, int img_w, int img_h,
                              std::vector<PoseDetection>& detections) {
    detections.clear();
    float scale_x = (float)img_w / MODEL_INPUT_SIZE;
    float scale_y = (float)img_h / MODEL_INPUT_SIZE;
    const int NUM_ANCHORS = 8400;
    const int NUM_KP = 20;

    for (int i = 0; i < NUM_ANCHORS; i++) {
        float cx = fp16_to_fp32(fp16_data[0 * NUM_ANCHORS + i]);
        float cy = fp16_to_fp32(fp16_data[1 * NUM_ANCHORS + i]);
        float bw = fp16_to_fp32(fp16_data[2 * NUM_ANCHORS + i]);
        float bh = fp16_to_fp32(fp16_data[3 * NUM_ANCHORS + i]);
        float conf = fp16_to_fp32(fp16_data[4 * NUM_ANCHORS + i]);

        if (conf < OBJ_THRESHOLD) continue;

        PoseDetection det;
        det.score = conf;

        float x1_640 = cx - bw / 2.0f;
        float y1_640 = cy - bh / 2.0f;
        float x2_640 = cx + bw / 2.0f;
        float y2_640 = cy + bh / 2.0f;

        x1_640 = std::max(0.0f, std::min(x1_640, (float)MODEL_INPUT_SIZE));
        y1_640 = std::max(0.0f, std::min(y1_640, (float)MODEL_INPUT_SIZE));
        x2_640 = std::max(0.0f, std::min(x2_640, (float)MODEL_INPUT_SIZE));
        y2_640 = std::max(0.0f, std::min(y2_640, (float)MODEL_INPUT_SIZE));

        det.x1 = x1_640 * scale_x;
        det.y1 = y1_640 * scale_y;
        det.x2 = x2_640 * scale_x;
        det.y2 = y2_640 * scale_y;

        for (int k = 0; k < NUM_KP; k++) {
            float kx = fp16_to_fp32(fp16_data[(5 + k*3 + 0) * NUM_ANCHORS + i]);
            float ky = fp16_to_fp32(fp16_data[(5 + k*3 + 1) * NUM_ANCHORS + i]);
            float kv = fp16_to_fp32(fp16_data[(5 + k*3 + 2) * NUM_ANCHORS + i]);

            det.kps[k].x = std::max(0.0f, std::min(kx, (float)MODEL_INPUT_SIZE)) * scale_x;
            det.kps[k].y = std::max(0.0f, std::min(ky, (float)MODEL_INPUT_SIZE)) * scale_y;
            det.kps[k].visibility = (kv > KPT_CONF_THRESHOLD) ? kv : 0.0f;
        }

        // 屏蔽噪声点：只保留6个有效点
        for (int k = 0; k < NUM_KP; k++) {
            bool valid = false;
            for (int j = 0; j < 6; j++) {
                if (FACE_VALID_KPS[j] == k) { valid = true; break; }
            }
            if (!valid) det.kps[k].visibility = 0.0f;
        }

        detections.push_back(det);
    }

    nms(detections, NMS_THRESHOLD);
    if (detections.size() > MAX_DETECTIONS) detections.resize(MAX_DETECTIONS);
}

// BODY模式后处理：[1,56,8400] 17个COCO关键点
static void post_process_body(uint16_t* fp16_data, int img_w, int img_h,
                              std::vector<PoseDetection>& detections) {
    detections.clear();
    float scale_x = (float)img_w / MODEL_INPUT_SIZE;
    float scale_y = (float)img_h / MODEL_INPUT_SIZE;
    const int NUM_ANCHORS = 8400;
    const int NUM_KP = 17;

    for (int i = 0; i < NUM_ANCHORS; i++) {
        float cx = fp16_to_fp32(fp16_data[0 * NUM_ANCHORS + i]);
        float cy = fp16_to_fp32(fp16_data[1 * NUM_ANCHORS + i]);
        float bw = fp16_to_fp32(fp16_data[2 * NUM_ANCHORS + i]);
        float bh = fp16_to_fp32(fp16_data[3 * NUM_ANCHORS + i]);
        float conf = fp16_to_fp32(fp16_data[4 * NUM_ANCHORS + i]);

        if (conf < OBJ_THRESHOLD) continue;

        PoseDetection det;
        det.score = conf;

        float x1_640 = cx - bw / 2.0f;
        float y1_640 = cy - bh / 2.0f;
        float x2_640 = cx + bw / 2.0f;
        float y2_640 = cy + bh / 2.0f;

        x1_640 = std::max(0.0f, std::min(x1_640, (float)MODEL_INPUT_SIZE));
        y1_640 = std::max(0.0f, std::min(y1_640, (float)MODEL_INPUT_SIZE));
        x2_640 = std::max(0.0f, std::min(x2_640, (float)MODEL_INPUT_SIZE));
        y2_640 = std::max(0.0f, std::min(y2_640, (float)MODEL_INPUT_SIZE));

        det.x1 = x1_640 * scale_x;
        det.y1 = y1_640 * scale_y;
        det.x2 = x2_640 * scale_x;
        det.y2 = y2_640 * scale_y;

        for (int k = 0; k < NUM_KP; k++) {
            float kx = fp16_to_fp32(fp16_data[(5 + k*3 + 0) * NUM_ANCHORS + i]);
            float ky = fp16_to_fp32(fp16_data[(5 + k*3 + 1) * NUM_ANCHORS + i]);
            float kv = fp16_to_fp32(fp16_data[(5 + k*3 + 2) * NUM_ANCHORS + i]);

            det.kps[k].x = std::max(0.0f, std::min(kx, (float)MODEL_INPUT_SIZE)) * scale_x;
            det.kps[k].y = std::max(0.0f, std::min(ky, (float)MODEL_INPUT_SIZE)) * scale_y;
            det.kps[k].visibility = kv;
        }
        // 剩余清零
        for (int k = NUM_KP; k < MAX_KEYPOINTS; k++) {
            det.kps[k].visibility = 0.0f;
        }

        detections.push_back(det);
    }

    nms(detections, NMS_THRESHOLD);
    if (detections.size() > MAX_DETECTIONS) detections.resize(MAX_DETECTIONS);
}

// ========== 画检测框和关键点 ==========
static const uint8_t digit_font[10][5] = {
    {0b111, 0b101, 0b101, 0b101, 0b111},
    {0b010, 0b110, 0b010, 0b010, 0b111},
    {0b111, 0b001, 0b111, 0b100, 0b111},
    {0b111, 0b001, 0b111, 0b001, 0b111},
    {0b101, 0b101, 0b111, 0b001, 0b001},
    {0b111, 0b100, 0b111, 0b001, 0b111},
    {0b111, 0b100, 0b111, 0b101, 0b111},
    {0b111, 0b001, 0b001, 0b010, 0b010},
    {0b111, 0b101, 0b111, 0b101, 0b111},
    {0b111, 0b101, 0b111, 0b001, 0b111},
};

static void draw_digit(uint8_t* y_plane, int img_w, int img_h, int cx, int cy, int digit) {
    if (digit < 0 || digit > 9) return;
    for (int row = 0; row < 5; row++) {
        uint8_t bits = digit_font[digit][row];
        for (int col = 0; col < 3; col++) {
            if (bits & (1 << (2 - col))) {
                int px = cx + col;
                int py = cy + row;
                if (px >= 0 && px < img_w && py >= 0 && py < img_h) {
                    y_plane[py * img_w + px] = 255;
                }
            }
        }
    }
}

static void draw_number(uint8_t* y_plane, int img_w, int img_h, int cx, int cy, int num) {
    if (num < 0 || num > 99) return;
    if (num >= 10) {
        draw_digit(y_plane, img_w, img_h, cx, cy, num / 10);
        draw_digit(y_plane, img_w, img_h, cx + 4, cy, num % 10);
    } else {
        draw_digit(y_plane, img_w, img_h, cx, cy, num);
    }
}

static void draw_detections(uint8_t* nv12, int img_w, int img_h,
                            const std::vector<PoseDetection>& detections, int num_kps) {
    if (detections.empty()) return;

    uint8_t* y_plane = nv12;

    for (const auto& det : detections) {
        int x1 = (int)det.x1, y1 = (int)det.y1;
        int x2 = (int)det.x2, y2 = (int)det.y2;

        x1 = std::max(2, std::min(x1, img_w-2));
        y1 = std::max(2, std::min(y1, img_h-2));
        x2 = std::max(2, std::min(x2, img_w-2));
        y2 = std::max(2, std::min(y2, img_h-2));

        if (x2-x1 < 2 || y2-y1 < 2) continue;

        // 画框（白色边框）
        for (int x = x1; x <= x2 && x < img_w; x++) {
            if (y1 < img_h) y_plane[y1 * img_w + x] = 255;
            if (y2 < img_h) y_plane[y2 * img_w + x] = 255;
        }
        for (int y = y1; y <= y2 && y < img_h; y++) {
            if (x1 < img_w) y_plane[y * img_w + x1] = 255;
            if (x2 < img_w) y_plane[y * img_w + x2] = 255;
        }

        // 画关键点（白色小点）和编号
        for (int k = 0; k < num_kps; k++) {
            if (det.kps[k].visibility > KPT_CONF_THRESHOLD) {
                int kx = (int)det.kps[k].x;
                int ky = (int)det.kps[k].y;
                if (kx >= 2 && ky >= 2 && kx < img_w-2 && ky < img_h-2) {
                    for (int dy = -1; dy <= 1; dy++) {
                        for (int dx = -1; dx <= 1; dx++) {
                            y_plane[(ky+dy) * img_w + (kx+dx)] = 255;
                        }
                    }
                    draw_number(y_plane, img_w, img_h, kx + 3, ky + 3, k);
                }
            }
        }
    }
}

// ========== NPU初始化 ==========
static int init_single_model(const char* path, rknn_context* ctx,
                              rknn_tensor_attr* out_attr,
                              rknn_tensor_mem** out_mem,
                              const char* name) {
    printf("[NPU] Loading %s model: %s\n", name, path);
    if (access(path, F_OK) != 0) {
        printf("[ERROR] %s model not found: %s\n", name, path);
        return -1;
    }

    int ret = rknn_init(ctx, (void*)path, 0, 0, NULL);
    if (ret < 0) {
        printf("[ERROR] %s rknn_init failed: %d\n", name, ret);
        return -1;
    }

    // 三核加速
    ret = rknn_set_core_mask(*ctx, RKNN_NPU_CORE_0_1_2);
    if (ret < 0) {
        printf("[WARN] %s rknn_set_core_mask failed: %d\n", name, ret);
    } else {
        printf("[NPU] %s enabled 3-core acceleration\n", name);
    }

    rknn_input_output_num io_num;
    rknn_query(*ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    printf("[NPU] %s In=%d Out=%d\n", name, io_num.n_input, io_num.n_output);

    rknn_tensor_attr in_attr;
    memset(&in_attr, 0, sizeof(in_attr));
    in_attr.index = 0;
    rknn_query(*ctx, RKNN_QUERY_INPUT_ATTR, &in_attr, sizeof(in_attr));
    printf("[NPU] %s Input: [%d,%d,%d,%d] type=%d size=%d\n", name,
           in_attr.dims[0], in_attr.dims[1], in_attr.dims[2], in_attr.dims[3],
           in_attr.type, in_attr.size);

    memset(out_attr, 0, sizeof(*out_attr));
    out_attr->index = 0;
    rknn_query(*ctx, RKNN_QUERY_OUTPUT_ATTR, out_attr, sizeof(*out_attr));
    printf("[NPU] %s Output[0]: [%d,%d,%d,%d] type=%d elems=%d size=%d\n", name,
           out_attr->dims[0], out_attr->dims[1], out_attr->dims[2], out_attr->dims[3],
           out_attr->type, out_attr->n_elems, out_attr->size);

    *out_mem = rknn_create_mem(*ctx, out_attr->size);
    if (!*out_mem) {
        printf("[ERROR] %s failed to alloc output memory (size=%d)\n", name, out_attr->size);
        return -1;
    }
    return 0;
}

static int init_rule_engine(void) {
    printf("[NPU] Connecting to RuleEngine service: %s\n", RULE_SOCK_PATH);
    g_rule_sock = socket(AF_UNIX, SOCK_STREAM, 0);
    if (g_rule_sock < 0) {
        printf("[WARN] RuleEngine socket create failed: %s\n", strerror(errno));
        return -1;
    }
    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, RULE_SOCK_PATH, sizeof(addr.sun_path)-1);
    if (connect(g_rule_sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        printf("[WARN] RuleEngine connect to %s failed: %s\n", RULE_SOCK_PATH, strerror(errno));
        close(g_rule_sock);
        g_rule_sock = -1;
        return -1;
    }
    printf("[NPU] RuleEngine connected to %s\n", RULE_SOCK_PATH);
    return 0;
}

int init_npu() {
    printf("\n========== NPU Init ==========\n");

    if (init_single_model(FACE_LM_MODEL_PATH, &face_lm_ctx, &face_lm_output_attr[0], &face_lm_output_mem[0], "FaceLM") != 0)
        return -1;
    // face_lm 有 2 个输出，需分别查询和分配
    memset(&face_lm_output_attr[1], 0, sizeof(face_lm_output_attr[1]));
    face_lm_output_attr[1].index = 1;
    rknn_query(face_lm_ctx, RKNN_QUERY_OUTPUT_ATTR, &face_lm_output_attr[1], sizeof(face_lm_output_attr[1]));
    printf("[NPU] FaceLM Output[1]: [%d,%d,%d,%d] type=%d elems=%d size=%d\n",
           face_lm_output_attr[1].dims[0], face_lm_output_attr[1].dims[1],
           face_lm_output_attr[1].dims[2], face_lm_output_attr[1].dims[3],
           face_lm_output_attr[1].type, face_lm_output_attr[1].n_elems, face_lm_output_attr[1].size);
    face_lm_output_mem[1] = rknn_create_mem(face_lm_ctx, face_lm_output_attr[1].size);
    if (!face_lm_output_mem[1]) {
        printf("[ERROR] FaceLM output[1] alloc failed (size=%d)\n", face_lm_output_attr[1].size);
        return -1;
    }
    if (init_single_model(BODY_MODEL_PATH, &body_ctx, &body_output_attr, &body_output_mem, "Body") != 0)
        return -1;

    // 加载 RuleEngine（可选，失败不阻塞）
    init_rule_engine();

    // 分配公共缓冲区
    if (posix_memalign((void**)&rga_input_buf, 64, MODEL_INPUT_SIZE*MODEL_INPUT_SIZE*3/2) != 0 ||
        posix_memalign((void**)&rga_output_buf, 64, MODEL_INPUT_SIZE*MODEL_INPUT_SIZE*3) != 0 ||
        posix_memalign((void**)&npu_input_fp32, 64, MODEL_INPUT_SIZE*MODEL_INPUT_SIZE*3*sizeof(float)) != 0 ||
        posix_memalign((void**)&npu_input_fp16, 64, MODEL_INPUT_SIZE*MODEL_INPUT_SIZE*3*sizeof(uint16_t)) != 0 ||
        posix_memalign((void**)&face_lm_input_buf, 64, FACE_LM_INPUT_SIZE*FACE_LM_INPUT_SIZE*3) != 0 ||
        posix_memalign((void**)&face_lm_tmp_nv12, 64, FACE_LM_INPUT_SIZE*FACE_LM_INPUT_SIZE*3/2) != 0 ||
        posix_memalign((void**)&face_lm_crop_buf, 64, FACE_LM_MAX_ROI*FACE_LM_MAX_ROI*3/2) != 0) {
        printf("[ERROR] Buffer alloc failed\n");
        return -1;
    }

    printf("[NPU] Init OK (FaceLM + Body loaded)\n\n");
    npu_initialized = 1;
    return 0;
}

int init_rga() { return 0; }

// ========== 帧处理 ==========
static void process_frame_face(uint8_t *nv12, int width, int height) {
    uint64_t t0 = get_us();

    // ===== 第一步：best.rknn 人体检测 =====
    rga_buffer_t src = wrapbuffer_virtualaddr(nv12, width, height, RK_FORMAT_YCbCr_420_SP);
    rga_buffer_t tmp = wrapbuffer_virtualaddr(rga_input_buf, MODEL_INPUT_SIZE, MODEL_INPUT_SIZE, RK_FORMAT_YCbCr_420_SP);
    rga_buffer_t dst = wrapbuffer_virtualaddr(rga_output_buf, MODEL_INPUT_SIZE, MODEL_INPUT_SIZE, RK_FORMAT_RGB_888);

    if (imresize(src, tmp, 0, 0, INTER_LINEAR) != IM_STATUS_SUCCESS) return;
    if (imcvtcolor(tmp, dst, RK_FORMAT_YCbCr_420_SP, RK_FORMAT_RGB_888, IM_YUV_TO_RGB_BT601_LIMIT) != IM_STATUS_SUCCESS) return;
    uint64_t t1 = get_us();

    // NPU推理：best.rknn
    rknn_input in;
    memset(&in, 0, sizeof(in));
    in.index = 0;
    in.type = RKNN_TENSOR_UINT8;
    in.fmt = RKNN_TENSOR_NHWC;
    in.buf = rga_output_buf;
    in.size = MODEL_INPUT_SIZE * MODEL_INPUT_SIZE * 3;
    rknn_inputs_set(body_ctx, 1, &in);
    rknn_set_io_mem(body_ctx, body_output_mem, &body_output_attr);
    uint64_t t2 = get_us();

    if (rknn_run(body_ctx, NULL) < 0) return;
    uint64_t t3 = get_us();

    std::vector<PoseDetection> body_dets;
    if (body_output_attr.type == RKNN_TENSOR_FLOAT16) {
        post_process_body((uint16_t*)body_output_mem->virt_addr, width, height, body_dets);
    }
    uint64_t t4 = get_us();

    // ========== Intended Person Tracker (from backup) ==========
    // 多人场景下锁定目标人物，只给 intended person 的 ROI 跑 Face LM
    static FaceTracker tracker(MODE_FACE, 17);
    if (tracker.mode != MODE_FACE) tracker.reset_mode(MODE_FACE, 17);
    tracker.tele_frames++;

    int matched_idx = tracker.update(body_dets, width, height);
    std::vector<PoseDetection> display_dets;

    if (matched_idx >= 0) {
        tracker.tele_tracked++;
        tracker.apply_filter(body_dets[matched_idx]);
        display_dets.push_back(body_dets[matched_idx]);
    }

    if (!display_dets.empty()) {
        draw_detections(nv12, width, height, display_dets, 17);
    }

    if (display_dets.empty()) {
        // 无人/丢失则返回，但仍记录统计
        int idx = g_stat_idx;
        g_stat[idx].get_frame_us      = g_next_get_frame_us;  g_next_get_frame_us = 0;
        g_stat[idx].rga_preprocess_us = t1 - t0;
        g_stat[idx].npu_body_us       = t4 - t1;
        g_stat[idx].roi_crop_us       = 0;
        g_stat[idx].npu_face_us       = 0;
        g_stat[idx].draw_us           = 0;
        g_stat[idx].encode_push_us    = g_next_encode_push_us; g_next_encode_push_us = 0;
        g_stat[idx].total_us          = t4 - t0;
        g_stat_idx = (g_stat_idx + 1) % STAT_WINDOW;
        if (++g_stat_count % STAT_WINDOW == 0) print_pipeline_stats();
        return;
    }

    // 取跟踪到的目标人物（intended person）
    const PoseDetection* best_det = &display_dets[0];

    // 面部点坐标（Face LM 映射回原始图像），未做 Face LM 时默认 0
    int img_rm_x = 0, img_rm_y = 0, img_lm_x = 0, img_lm_y = 0, img_chin_x = 0, img_chin_y = 0;

    // ===== 第二步：估算 face ROI + 关键点质量检查 =====
    float roi_cx = 0, roi_cy = 0, roi_size_f = 0;
    int roi_x = 0, roi_y = 0, roi_w = 0, roi_h = 0;
    float v_nose = best_det->kps[0].visibility;
    float v_l_eye  = best_det->kps[1].visibility;
    float v_r_eye  = best_det->kps[2].visibility;
    float v_l_shoulder = best_det->kps[5].visibility;
    float v_r_shoulder = best_det->kps[6].visibility;

    // 严格的人脸完整性检查
    bool face_valid = true;
    const char* skip_reason = nullptr;
    if (v_nose <= KPT_CONF_THRESHOLD) {
        face_valid = false; skip_reason = "nose low conf";
    } else if (v_l_eye <= KPT_CONF_THRESHOLD && v_r_eye <= KPT_CONF_THRESHOLD) {
        face_valid = false; skip_reason = "both eyes low conf";
    } else if (best_det->kps[0].y > height * 0.55f) {
        face_valid = false; skip_reason = "nose too low";
    } else {
        float shoulder_y = (best_det->kps[5].y + best_det->kps[6].y) * 0.5f;
        if (shoulder_y - best_det->kps[0].y < height * 0.06f) {
            face_valid = false; skip_reason = "face too short";
        }
    }
    bool skip_face_lm = !face_valid;

    if (skip_face_lm) {
        printf("[Face] Skip Face LM: %s (nose=%.2f eye_l=%.2f eye_r=%.2f sh_l=%.2f sh_r=%.2f)\n",
               skip_reason, v_nose, v_l_eye, v_r_eye, v_l_shoulder, v_r_shoulder);
        if (tracker.face_size_life > 0) tracker.face_size_life--;
    } else {
        if (v_nose > 0.5f) {
            roi_cx = best_det->kps[0].x;
            // 上一版 shoulder-based 作为最大范围
            float shoulder_w = fabs(best_det->kps[5].x - best_det->kps[6].x);
            float shoulder_roi = shoulder_w * 1.6f;
            // 这一版 face-based 作为最小范围（远距离精准）
            float face_roi = tracker.get_roi_size();

            float target_roi;
            if (face_roi > 0) {
                // face-based 已做指数平滑，直接生效
                target_roi = face_roi;
                if (target_roi > shoulder_roi) target_roi = shoulder_roi;
                roi_cy = best_det->kps[0].y + tracker.last_face_h * 0.05f;
                roi_size_f = target_roi;
            } else {
                // fallback: shoulder-based，每帧最多增长 16px（防止突变）
                target_roi = shoulder_roi;
                roi_cy = best_det->kps[0].y + 30.0f;
                if (tracker.prev_roi_size > 0) {
                    float diff = target_roi - tracker.prev_roi_size;
                    if (diff > 16.0f) diff = 16.0f;
                    roi_size_f = tracker.prev_roi_size + diff;
                } else {
                    roi_size_f = target_roi;
                }
            }
            tracker.prev_roi_size = roi_size_f;

            // printf("[FaceROI] used=%.1f target=%.1f (face=%.1f shoulder=%.1f)\n",
            //        roi_size_f, target_roi, face_roi, shoulder_roi);
            if (roi_size_f < 150.0f) roi_size_f = 150.0f;
            if (roi_size_f > 640.0f) roi_size_f = 640.0f;
        } else {
            roi_cx = (best_det->x1 + best_det->x2) * 0.5f;
            roi_cy = best_det->y1 + (best_det->y2 - best_det->y1) * 0.35f;
            roi_size_f = (best_det->y2 - best_det->y1) * 0.55f;
            tracker.prev_roi_size = roi_size_f;
            if (roi_size_f < 150.0f) roi_size_f = 150.0f;
            if (roi_size_f > 640.0f) roi_size_f = 640.0f;
        }

        roi_x = (int)(roi_cx - roi_size_f * 0.5f);
        roi_y = (int)(roi_cy - roi_size_f * 0.5f);
        roi_w = (int)roi_size_f;
        roi_h = (int)roi_size_f;

        if (roi_x < 0) roi_x = 0;
        if (roi_y < 0) roi_y = 0;
        // 越界时同步缩小宽高，保持正方形（避免人脸拉伸）
        if (roi_x + roi_w > width) {
            roi_w = width - roi_x;
            roi_h = roi_w;
        }
        if (roi_y + roi_h > height) {
            roi_h = height - roi_y;
            roi_w = roi_h;
        }
        // RGA 要求 wstride 是 16 的倍数，NV12 要求偶数偏移
        roi_w = (roi_w / 16) * 16;
        roi_h = roi_w;
        if (roi_x % 2 != 0) roi_x++;
        if (roi_y % 2 != 0) roi_y++;
        if (roi_w < 32 || roi_h < 32) {
            skip_face_lm = true;
        }
    }

    uint8_t* y_plane = nv12;
    if (!skip_face_lm) {
        // 画 ROI 框
        auto draw_roi_rect = [&](int rx, int ry, int rw, int rh) {
            int x1 = rx, y1 = ry, x2 = rx + rw - 1, y2 = ry + rh - 1;
            if (x1 < 0) x1 = 0; if (y1 < 0) y1 = 0;
            if (x2 >= width) x2 = width - 1;
            if (y2 >= height) y2 = height - 1;
            for (int t = 0; t < 3; t++) {
                if (y1 + t < height) for (int x = x1; x <= x2; x++) y_plane[(y1 + t) * width + x] = 180;
                if (y2 - t >= 0)     for (int x = x1; x <= x2; x++) y_plane[(y2 - t) * width + x] = 180;
                if (x1 + t < width)  for (int y = y1; y <= y2; y++) y_plane[y * width + (x1 + t)] = 180;
                if (x2 - t >= 0)     for (int y = y1; y <= y2; y++) y_plane[y * width + (x2 - t)] = 180;
            }
        };
        draw_roi_rect(roi_x, roi_y, roi_w, roi_h);
    }

    uint64_t t5 = t4, t6 = t4, t7 = t4;
    if (!skip_face_lm) {
        // ===== RGA 裁剪 ROI -> 192x192 RGB =====
        rga_buffer_t src_full = wrapbuffer_virtualaddr(nv12, width, height, RK_FORMAT_YCbCr_420_SP);
        rga_buffer_t crop_buf = wrapbuffer_virtualaddr(face_lm_crop_buf, roi_w, roi_h, RK_FORMAT_YCbCr_420_SP);
        im_rect crop_rect = {roi_x, roi_y, roi_w, roi_h};
        if (imcrop(src_full, crop_buf, crop_rect) != IM_STATUS_SUCCESS) {
            skip_face_lm = true;
            goto rule_engine_phase;
        }

        rga_buffer_t tmp_lm = wrapbuffer_virtualaddr(face_lm_tmp_nv12, 192, 192, RK_FORMAT_YCbCr_420_SP);
        if (imresize(crop_buf, tmp_lm, 0, 0, INTER_LINEAR) != IM_STATUS_SUCCESS) {
            skip_face_lm = true;
            goto rule_engine_phase;
        }

        rga_buffer_t dst_lm = wrapbuffer_virtualaddr(face_lm_input_buf, 192, 192, RK_FORMAT_RGB_888);
        if (imcvtcolor(tmp_lm, dst_lm, RK_FORMAT_YCbCr_420_SP, RK_FORMAT_RGB_888, IM_YUV_TO_RGB_BT601_LIMIT) != IM_STATUS_SUCCESS) {
            skip_face_lm = true;
            goto rule_engine_phase;
        }
        t5 = get_us();

        // ===== 第四步：RGA 输出直接给 NPU =====
        // ===== 第五步：跑 face_landmark_468_fp16.rknn =====
        rknn_input in_lm;
        memset(&in_lm, 0, sizeof(in_lm));
        in_lm.index = 0;
        in_lm.type = RKNN_TENSOR_UINT8;
        in_lm.fmt = RKNN_TENSOR_NHWC;
        in_lm.buf = face_lm_input_buf;
        in_lm.size = 192 * 192 * 3;
        rknn_inputs_set(face_lm_ctx, 1, &in_lm);
        rknn_set_io_mem(face_lm_ctx, face_lm_output_mem[0], &face_lm_output_attr[0]);
        rknn_set_io_mem(face_lm_ctx, face_lm_output_mem[1], &face_lm_output_attr[1]);

        if (rknn_run(face_lm_ctx, NULL) < 0) {
            printf("[NPU] face_lm rknn_run failed\n");
            skip_face_lm = true;
            goto rule_engine_phase;
        }
        t6 = get_us();

        // ===== 第六步：后处理 468 点 =====
        float lm_out[1404];
        if (face_lm_output_attr[0].type == RKNN_TENSOR_FLOAT16) {
            uint16_t* fp16_ptr = (uint16_t*)face_lm_output_mem[0]->virt_addr;
            for (int i = 0; i < 1404; i++) lm_out[i] = fp16_to_fp32(fp16_ptr[i]);
        } else {
            memcpy(lm_out, face_lm_output_mem[0]->virt_addr, sizeof(lm_out));
        }

        // 计算 468 点在 ROI 内的 bbox，映射回原始图像尺寸
        float lm_min_x = 192, lm_min_y = 192, lm_max_x = 0, lm_max_y = 0;
        for (int i = 0; i < 468; i++) {
            float x = lm_out[i * 3 + 0];
            float y = lm_out[i * 3 + 1];
            if (x < lm_min_x) lm_min_x = x;
            if (y < lm_min_y) lm_min_y = y;
            if (x > lm_max_x) lm_max_x = x;
            if (y > lm_max_y) lm_max_y = y;
        }
        // Sanity check: 468 点分布是否合理
        float face_w_ratio = (lm_max_x - lm_min_x) / 192.0f;
        float face_h_ratio = (lm_max_y - lm_min_y) / 192.0f;
        if (face_w_ratio < 0.22f || face_h_ratio < 0.22f ||
            face_w_ratio > 0.92f || face_h_ratio > 0.92f) {
            // printf("[FaceLM] Reject: bad bbox ratio (w=%.2f h=%.2f)\n", face_w_ratio, face_h_ratio);
            skip_face_lm = true;
            if (tracker.face_size_life > 0) tracker.face_size_life--;
            goto rule_engine_phase;
        }

        float lm_scale_x = (float)roi_w / 192.0f;
        float lm_scale_y = (float)roi_h / 192.0f;
        float face_w_img = (lm_max_x - lm_min_x) * lm_scale_x;
        float face_h_img = (lm_max_y - lm_min_y) * lm_scale_y;
        tracker.update_face_size(face_w_img, face_h_img);

        // 调试：打印前几个 landmark 坐标和 ROI 信息
        // printf("[FaceLM] ROI=%dx%d@%d,%d  lm[0]=(%.1f,%.1f) lm[61]=(%.1f,%.1f) lm[152]=(%.1f,%.1f) lm[291]=(%.1f,%.1f)  face_bbox=%.1fx%.1f\n",
        //        roi_w, roi_h, roi_x, roi_y,
        //        lm_out[0*3+0], lm_out[0*3+1],
        //        lm_out[61*3+0], lm_out[61*3+1],
        //        lm_out[152*3+0], lm_out[152*3+1],
        //        lm_out[291*3+0], lm_out[291*3+1],
        //        face_w_img, face_h_img);

        // MediaPipe Face Mesh 468 关键索引
        const int LM_RIGHT_MOUTH = 61;
        const int LM_LEFT_MOUTH  = 291;
        const int LM_CHIN        = 152;

        float rm_x   = lm_out[LM_RIGHT_MOUTH * 3 + 0];
        float rm_y   = lm_out[LM_RIGHT_MOUTH * 3 + 1];
        float lm_x   = lm_out[LM_LEFT_MOUTH  * 3 + 0];
        float lm_y   = lm_out[LM_LEFT_MOUTH  * 3 + 1];
        float chin_x = lm_out[LM_CHIN * 3 + 0];
        float chin_y = lm_out[LM_CHIN * 3 + 1];

        // 映射回原始图像坐标（模型输出是 [0,192] 归一化坐标）
        img_rm_x   = (int)(rm_x   * lm_scale_x + roi_x);
        img_rm_y   = (int)(rm_y   * lm_scale_y + roi_y);
        img_lm_x   = (int)(lm_x   * lm_scale_x + roi_x);
        img_lm_y   = (int)(lm_y   * lm_scale_y + roi_y);
        img_chin_x = (int)(chin_x * lm_scale_x + roi_x);
        img_chin_y = (int)(chin_y * lm_scale_y + roi_y);

        // ===== 第七步：画点 =====
        auto draw_white_dot = [&](int x, int y, int label) {
            if (x >= 3 && y >= 3 && x < width - 3 && y < height - 3) {
                for (int dy = -2; dy <= 2; dy++) {
                    for (int dx = -2; dx <= 2; dx++) {
                        y_plane[(y + dy) * width + (x + dx)] = 255;
                    }
                }
                draw_number(y_plane, width, height, x + 4, y + 4, label);
            }
        };

        draw_white_dot(img_rm_x, img_rm_y, LM_RIGHT_MOUTH);
        draw_white_dot(img_lm_x, img_lm_y, LM_LEFT_MOUTH);
        draw_white_dot(img_chin_x, img_chin_y, LM_CHIN);

        // face_landmark_468 -> 12 点 PnP
        {
            std::vector<cv::Point2f> face_lm_2d;
            for (int i = 0; i < 12; i++) {
                int idx = FACE_LM_12_IDS[i];
                float x = lm_out[idx * 3 + 0] * lm_scale_x + roi_x;
                float y = lm_out[idx * 3 + 1] * lm_scale_y + roi_y;
                face_lm_2d.emplace_back(x, y);
            }
            estimate_and_draw_pose(nv12, width, height, face_lm_2d, FACE_LM_12_3D);
        }
        t7 = get_us();
    }

rule_engine_phase:

    // ===== RuleEngine: 20 关键点手势状态推理 (Python Socket 服务) =====
    if (g_rule_sock >= 0) {
        float kpts_20[20][2];
        float valid_mask[20];
        // COCO → Observer 视角：交换 left/right 成对点
        static const int LR_PAIRS[8][2] = {
            {1,2}, {3,4}, {5,6}, {7,8}, {9,10}, {11,12}, {13,14}, {15,16}
        };
        for (int i = 0; i < 17; i++) {
            int coco_idx = i;
            for (int p = 0; p < 8; p++) {
                if (LR_PAIRS[p][0] == i) { coco_idx = LR_PAIRS[p][1]; break; }
                if (LR_PAIRS[p][1] == i) { coco_idx = LR_PAIRS[p][0]; break; }
            }
            if (display_dets[0].kps[coco_idx].visibility > KPT_CONF_THRESHOLD) {
                kpts_20[i][0] = display_dets[0].kps[coco_idx].x;
                kpts_20[i][1] = display_dets[0].kps[coco_idx].y;
                valid_mask[i] = 1.0f;
            } else {
                kpts_20[i][0] = 0.0f;
                kpts_20[i][1] = 0.0f;
                valid_mask[i] = 0.0f;
            }
        }
        if (!skip_face_lm) {
            kpts_20[17][0] = (float)img_lm_x; kpts_20[17][1] = (float)img_lm_y;
            kpts_20[18][0] = (float)img_rm_x; kpts_20[18][1] = (float)img_rm_y;
            kpts_20[19][0] = (float)img_chin_x; kpts_20[19][1] = (float)img_chin_y;
            valid_mask[17] = valid_mask[18] = valid_mask[19] = 1.0f;
        } else {
            kpts_20[17][0] = kpts_20[17][1] = 0.0f;
            kpts_20[18][0] = kpts_20[18][1] = 0.0f;
            kpts_20[19][0] = kpts_20[19][1] = 0.0f;
            valid_mask[17] = valid_mask[18] = valid_mask[19] = 0.0f;
        }

        // flatten kpts: interleaved [x0,y0, x1,y1, ..., x19,y19] for reshape(1,20,2)
        float kpts_flat[40];
        for (int i = 0; i < 20; i++) {
            kpts_flat[i * 2]     = kpts_20[i][0];
            kpts_flat[i * 2 + 1] = kpts_20[i][1];
        }

        // bbox [x1, y1, x2, y2] for rule_engine_v2.rknn
        float bbox[4] = {
            display_dets[0].x1,
            display_dets[0].y1,
            display_dets[0].x2,
            display_dets[0].y2
        };

        int64_t state_fb[7] = {
            g_rule_state, g_rule_r_hold, g_rule_l_hold,
            g_rule_c_hold, g_rule_n_hold, g_rule_r_miss, g_rule_l_miss
        };

        // v2 protocol: 44f(kpts+bbox) + 20f(valid_mask) + 7q(state_fb) = 312 bytes
        for (int retry = 0; retry < 2; retry++) {
            if (g_rule_sock < 0 && init_rule_engine() != 0) break;

            bool send_ok = true;
            ssize_t n = send(g_rule_sock, kpts_flat, sizeof(kpts_flat), MSG_MORE | MSG_NOSIGNAL);
            if (n != sizeof(kpts_flat)) send_ok = false;
            n = send(g_rule_sock, bbox, sizeof(bbox), MSG_MORE | MSG_NOSIGNAL);
            if (n != sizeof(bbox)) send_ok = false;
            n = send(g_rule_sock, valid_mask, sizeof(valid_mask), MSG_MORE | MSG_NOSIGNAL);
            if (n != sizeof(valid_mask)) send_ok = false;
            n = send(g_rule_sock, state_fb, sizeof(state_fb), MSG_NOSIGNAL);
            if (n != sizeof(state_fb)) send_ok = false;

            if (send_ok) {
                int64_t result[7];
                ssize_t total = 0;
                while (total < (ssize_t)sizeof(result)) {
                    n = recv(g_rule_sock, ((char*)result) + total, sizeof(result) - total, 0);
                    if (n <= 0) break;
                    total += n;
                }
                if (total == sizeof(result)) {
                    g_rule_state  = result[0];
                    g_rule_r_hold = result[1];
                    g_rule_l_hold = result[2];
                    g_rule_c_hold = result[3];
                    g_rule_n_hold = result[4];
                    g_rule_r_miss = result[5];
                    g_rule_l_miss = result[6];

                    // 画面左下角打印 state
                    const char* state_names[] = {"Idle", "Mode1", "Mode2"};
                    int s = (int)g_rule_state;
                    if (s < 0 || s > 2) s = 0;
                    cv::Mat y_mat(height, width, CV_8UC1, nv12);
                    cv::putText(y_mat, cv::format("State: %s", state_names[s]),
                                cv::Point(10, height - 20),
                                cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(255), 2);
                    break;  // success
                } else {
                    printf("[RuleEngine] recv failed (%zd/%zu), reconnecting...\n", total, sizeof(result));
                }
            } else {
                printf("[RuleEngine] send failed, reconnecting...\n");
            }

            // close and retry
            close(g_rule_sock); g_rule_sock = -1;
        }
    }

    // 记录统计
    int idx = g_stat_idx;
    g_stat[idx].get_frame_us      = g_next_get_frame_us;  g_next_get_frame_us = 0;
    g_stat[idx].rga_preprocess_us = t1 - t0;
    g_stat[idx].npu_body_us       = t4 - t1;
    g_stat[idx].roi_crop_us       = t5 - t4;
    g_stat[idx].npu_face_us       = t6 - t5;
    g_stat[idx].draw_us           = t7 - t6;
    g_stat[idx].encode_push_us    = g_next_encode_push_us; g_next_encode_push_us = 0;
    g_stat[idx].total_us          = t7 - t0;
    g_stat_idx = (g_stat_idx + 1) % STAT_WINDOW;
    if (++g_stat_count % STAT_WINDOW == 0) print_pipeline_stats();
}

static void process_frame_body(uint8_t *nv12, int width, int height) {
    uint64_t t0 = get_us();

    // RGA预处理
    rga_buffer_t src = wrapbuffer_virtualaddr(nv12, width, height, RK_FORMAT_YCbCr_420_SP);
    rga_buffer_t tmp = wrapbuffer_virtualaddr(rga_input_buf, MODEL_INPUT_SIZE, MODEL_INPUT_SIZE, RK_FORMAT_YCbCr_420_SP);
    rga_buffer_t dst = wrapbuffer_virtualaddr(rga_output_buf, MODEL_INPUT_SIZE, MODEL_INPUT_SIZE, RK_FORMAT_RGB_888);

    if (imresize(src, tmp, 0, 0, INTER_LINEAR) != IM_STATUS_SUCCESS) return;
    if (imcvtcolor(tmp, dst, RK_FORMAT_YCbCr_420_SP, RK_FORMAT_RGB_888, IM_YUV_TO_RGB_BT601_LIMIT) != IM_STATUS_SUCCESS) return;
    uint64_t t1 = get_us();

    // NPU推理
    rknn_input in;
    memset(&in, 0, sizeof(in));
    in.index = 0;
    in.type = RKNN_TENSOR_UINT8;
    in.fmt = RKNN_TENSOR_NHWC;
    in.buf = rga_output_buf;
    in.size = MODEL_INPUT_SIZE * MODEL_INPUT_SIZE * 3;
    rknn_inputs_set(body_ctx, 1, &in);

    rknn_set_io_mem(body_ctx, body_output_mem, &body_output_attr);
    uint64_t t2 = get_us();

    if (rknn_run(body_ctx, NULL) < 0) {
        printf("[NPU] body rknn_run failed\n");
        return;
    }
    uint64_t t3 = get_us();

    std::vector<PoseDetection> detections;
    if (body_output_attr.type == RKNN_TENSOR_FLOAT16) {
        post_process_body((uint16_t*)body_output_mem->virt_addr, width, height, detections);
    }
    uint64_t t4 = get_us();

    static FaceTracker tracker(MODE_BODY, 17);
    if (tracker.mode != MODE_BODY) tracker.reset_mode(MODE_BODY, 17);
    tracker.tele_frames++;

    int matched_idx = tracker.update(detections, width, height);
    std::vector<PoseDetection> display_dets;

    if (matched_idx >= 0) {
        tracker.tele_tracked++;
        tracker.apply_filter(detections[matched_idx]);
        display_dets.push_back(detections[matched_idx]);
    }
    uint64_t t5 = get_us();

    if (!display_dets.empty()) {
        draw_detections(nv12, width, height, display_dets, 17);
        // BODY模式不画3D人脸姿态
    }
    uint64_t t6 = get_us();

    // 记录统计
    int idx = g_stat_idx;
    g_stat[idx].get_frame_us      = g_next_get_frame_us;  g_next_get_frame_us = 0;
    g_stat[idx].rga_preprocess_us = t1 - t0;
    g_stat[idx].npu_body_us       = t4 - t1;
    g_stat[idx].roi_crop_us       = 0;
    g_stat[idx].npu_face_us       = 0;
    g_stat[idx].draw_us           = t6 - t4;
    g_stat[idx].encode_push_us    = g_next_encode_push_us; g_next_encode_push_us = 0;
    g_stat[idx].total_us          = t6 - t0;
    g_stat_idx = (g_stat_idx + 1) % STAT_WINDOW;
    if (++g_stat_count % STAT_WINDOW == 0) print_pipeline_stats();
}

void process_frame(uint8_t *nv12, int width, int height) {
    g_frame_count++;
    if (!npu_initialized) return;

    PoseMode mode = get_pose_mode();
    if (mode == MODE_FACE) {
        process_frame_face(nv12, width, height);
    } else {
        process_frame_body(nv12, width, height);
    }
}

// ========== 清理 ==========
void cleanup_npu() {
    if (face_lm_output_mem[1]) rknn_destroy_mem(face_lm_ctx, face_lm_output_mem[1]);
    if (face_lm_output_mem[0]) rknn_destroy_mem(face_lm_ctx, face_lm_output_mem[0]);
    if (body_output_mem) rknn_destroy_mem(body_ctx, body_output_mem);
    if (face_lm_ctx) rknn_destroy(face_lm_ctx);
    if (body_ctx) rknn_destroy(body_ctx);

    // RuleEngine cleanup
    if (g_rule_sock >= 0) {
        close(g_rule_sock);
        g_rule_sock = -1;
    }

    pose_filter_init = false;
    have_prev_pose = false;
    prev_rvec.release();
    prev_tvec.release();

    free(rga_input_buf);
    free(rga_output_buf);
    free(npu_input_fp32);
    free(npu_input_fp16);
    free(face_lm_input_buf);
    free(face_lm_tmp_nv12);
    free(face_lm_crop_buf);
    printf("[NPU] Cleanup\n");
}

void cleanup_rga() {}

// ========== Mode控制 ==========
void set_pose_mode(PoseMode mode) {
    g_mode.store(mode);
    printf("[Mode] Switched to %s\n", pose_mode_name(mode));
}

PoseMode get_pose_mode(void) {
    return g_mode.load();
}

const char* pose_mode_name(PoseMode mode) {
    return (mode == MODE_FACE) ? "FACE" : "BODY";
}
