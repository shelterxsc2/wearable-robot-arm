/**
 * rga_npu.cpp - 双模型切换版：FACE (6点脸部) + BODY (YOLOv8n-pose 17点)
 */
#include "rga_npu.h"
#include "uart_comm.h"
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
#include <opencv2/opencv.hpp>
#include <atomic>

// ========== 配置 ==========
#define FACE_MODEL_PATH     "./models/face_best.rknn"
#define BODY_MODEL_PATH     "./models/best.rknn"
#define MODEL_INPUT_SIZE    640
#define MAX_KEYPOINTS       20
#define OBJ_THRESHOLD       0.25f
#define NMS_THRESHOLD       0.45f
#define MAX_DETECTIONS      10

// ========== 全局变量 ==========
static std::atomic<PoseMode> g_mode{MODE_FACE};

// Face model
static rknn_context face_ctx = 0;
static rknn_tensor_attr face_output_attr;
static rknn_tensor_mem *face_output_mem = NULL;

// Body model
static rknn_context body_ctx = 0;
static rknn_tensor_attr body_output_attr;
static rknn_tensor_mem *body_output_mem = NULL;

static uint8_t *rga_input_buf = NULL;
static uint8_t *rga_output_buf = NULL;
static float *npu_input_fp32 = NULL;
static uint16_t *npu_input_fp16 = NULL;

static int g_frame_count = 0;
static int npu_initialized = 0;
static uint64_t g_frame_start_us = 0;

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

/* ---------- AI 内部各环节耗时统计 ---------- */
#define AI_STAT_WINDOW 30

static inline uint64_t get_us() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000;
}

static struct {
    uint64_t rga_us;        // RGA resize + cvtcolor
    uint64_t npu_set_us;    // rknn_inputs_set + rknn_set_io_mem
    uint64_t npu_run_us;    // rknn_run
    uint64_t post_us;       // post_process + nms
    uint64_t track_us;      // tracker.update + apply_filter
    uint64_t draw_us;       // estimate_and_draw_pose / draw_detections
    uint64_t total_us;      // process_frame_face/body 总耗时
} g_ai_stat[AI_STAT_WINDOW];

static int g_ai_stat_idx = 0;
static int g_ai_stat_count = 0;

static void print_ai_stats() {
    double sum_rga = 0, sum_npu_set = 0, sum_npu_run = 0;
    double sum_post = 0, sum_track = 0, sum_draw = 0, sum_total = 0;
    for (int i = 0; i < AI_STAT_WINDOW; i++) {
        sum_rga     += g_ai_stat[i].rga_us;
        sum_npu_set += g_ai_stat[i].npu_set_us;
        sum_npu_run += g_ai_stat[i].npu_run_us;
        sum_post    += g_ai_stat[i].post_us;
        sum_track   += g_ai_stat[i].track_us;
        sum_draw    += g_ai_stat[i].draw_us;
        sum_total   += g_ai_stat[i].total_us;
    }
    double n = AI_STAT_WINDOW;
    printf("\n[AI-STAT] ======== Last %d frames avg ========\n", AI_STAT_WINDOW);
    printf("[AI-STAT] RGA resize+cvt : %7.2f ms\n", sum_rga     / n / 1000.0);
    printf("[AI-STAT] NPU set_io     : %7.2f ms\n", sum_npu_set / n / 1000.0);
    printf("[AI-STAT] NPU run        : %7.2f ms\n", sum_npu_run / n / 1000.0);
    printf("[AI-STAT] post_process   : %7.2f ms\n", sum_post    / n / 1000.0);
    printf("[AI-STAT] track+filter   : %7.2f ms\n", sum_track   / n / 1000.0);
    printf("[AI-STAT] draw           : %7.2f ms\n", sum_draw    / n / 1000.0);
    printf("[AI-STAT] total AI       : %7.2f ms\n", sum_total   / n / 1000.0);
    printf("[AI-STAT] ===================================\n\n");
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

static void estimate_and_draw_pose(uint8_t* nv12, int img_w, int img_h, const PoseDetection& det) {
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

    std::vector<cv::Point2f> image_points;
    for (int i = 0; i < 6; i++) {
        int kidx = POSE_REQUIRED_IDS[i];
        if (det.kps[kidx].visibility <= 0.5f) return;
        image_points.emplace_back(det.kps[kidx].x, det.kps[kidx].y);
    }

    cv::Mat rvec, tvec;
    if (have_prev_pose) {
        rvec = prev_rvec.clone();
        tvec = prev_tvec.clone();
    }
    bool success = cv::solvePnP(FACE_3D_POINTS, image_points,
                                K, DIST_COEFFS,
                                rvec, tvec, have_prev_pose,
                                cv::SOLVEPNP_ITERATIVE);
    if (!success) return;

    // 重投影误差检查：坏帧直接丢弃，防止 3D 框畸变
    std::vector<cv::Point2f> reproj_pts;
    cv::projectPoints(FACE_3D_POINTS, rvec, tvec, K, DIST_COEFFS, reproj_pts);
    double reproj_error = 0.0;
    for (int i = 0; i < 6; i++) {
        double dx = image_points[i].x - reproj_pts[i].x;
        double dy = image_points[i].y - reproj_pts[i].y;
        reproj_error += std::sqrt(dx*dx + dy*dy);
    }
    reproj_error /= 6.0;

    // PnP 调试打印（每10帧）
    static int pnp_debug_cnt = 0;
    if (++pnp_debug_cnt % 10 == 0) {
        printf("[PnP-Debug] reproj_err=%.1fpx | rvec(%.3f,%.3f,%.3f) tvec(%.1f,%.1f,%.1f)\n",
               reproj_error,
               rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2),
               tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
    }

    if (reproj_error > 25.0) {
        static int bad_cnt = 0;
        if (++bad_cnt % 10 == 0) {
            printf("[Pose] Bad frame skipped, reprojection error=%.1fpx\n", reproj_error);
        }
        return;
    }

    // 硬丢弃：OpenCV 相机坐标系 Z 正方向远离相机，tvec_z < 0 表示人脸在相机后方，是镜像解
    if (tvec.at<double>(2) < 0) {
        static int mirror_cnt = 0;
        if (++mirror_cnt % 10 == 0) {
            printf("[Pose] Mirror solution detected (tz=%.1f), dropped\n", tvec.at<double>(2));
        }
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
            if (++jump_cnt % 10 == 0) {
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

    static const double PITCH_OFFSET = -0.10;
    static const double YAW_OFFSET   =  0.03;
    rvec_f.at<double>(0) += PITCH_OFFSET;
    rvec_f.at<double>(1) += YAW_OFFSET;

    cv::Mat R_calib;
    cv::Rodrigues(rvec_f, R_calib);
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

    cv::Mat R_face2cam;
    cv::Rodrigues(rvec_f, R_face2cam);
    cv::Mat R_cam2face = R_face2cam.t();
    cv::Mat cam_pos = -R_cam2face * tvec_f;

    double tx = tvec_f.at<double>(0);
    double ty = tvec_f.at<double>(1);
    double tz = tvec_f.at<double>(2);
    double pos_yaw_err = std::atan2(tx, tz);
    double pos_pitch_err = std::atan2(ty, tz);

    // PnP result ready -- log latency here (before drawing)
    struct timespec ts_now;
    clock_gettime(CLOCK_MONOTONIC, &ts_now);
    uint64_t now_us = ts_now.tv_sec * 1000000ULL + ts_now.tv_nsec / 1000;
    uint64_t elapsed_us = now_us - g_frame_start_us;
    double hp_deg = head_pitch * 180.0 / M_PI;
    double hy_deg = head_yaw * 180.0 / M_PI;
    double hr_deg = head_roll * 180.0 / M_PI;
    double pye_deg = pos_yaw_err * 180.0 / M_PI;
    double ppe_deg = pos_pitch_err * 180.0 / M_PI;

    printf("[Latency] Frame->PnP: %.2f ms | Pose(P%.1f Y%.1f R%.1f) | PosErr(Yaw=%.1f Pitch=%.1f)\n",
           elapsed_us / 1000.0, hp_deg, hy_deg, hr_deg, pye_deg, ppe_deg);

    // 在画面左上角醒目显示 Yaw/Pitch/Roll + rvec + 四元数（黑底白字）
    double rx = rvec_f.at<double>(0);
    double ry = rvec_f.at<double>(1);
    double rz = rvec_f.at<double>(2);

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
    snprintf(line1, sizeof(line1), "Yaw=%.1f  Pitch=%.1f  Roll=%.1f", hy_deg, hp_deg, hr_deg);
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
    cv::projectPoints(std::vector<cv::Point3f>{{0,0,0}}, rvec_f, tvec_f, K, DIST_COEFFS, proj_origin);
    cv::Point2f origin = proj_origin[0];

    std::vector<cv::Point3f> axis_3d = {{40,0,0}, {0,40,0}, {0,0,40}};
    std::vector<cv::Point2f> proj_axis;
    cv::projectPoints(axis_3d, rvec_f, tvec_f, K, DIST_COEFFS, proj_axis);

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
        for (int i = 0; i < 6; i++) {
            int kidx = POSE_REQUIRED_IDS[i];
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
            snprintf(klabel, sizeof(klabel), "%d", kidx);
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
    cv::projectPoints(cube_3d, rvec_f, tvec_f, K, DIST_COEFFS, proj_cube);

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

    // ========== 人脸跟随验证模式（舵机固定，只调机械臂位置） ==========
    // 坐标系：原点为云台电机（基座系）
    //   +X: 穿戴者右侧
    //   +Y: 人脸前方（远离底座，钓鱼竿伸出的方向）
    //   +Z: 竖直向上
    //
    // 核心逻辑：人脸可以绕竖直轴转动（偏头），相机跟随人脸朝向在水平面内摆动。
    // 舵机固定（就当没有舵机），只控制机械臂三个关节改变相机位置。
    // 人脸朝 +Y 时相机在 (0, 67, 40)；人脸右转朝 +X 时相机右摆到 (57, 10, 40) 附近。

    // 人脸固定位置 (cm)
    static const float FACE_X_CM = 0.0f;
    static const float FACE_Y_CM = 10.0f;
    static const float FACE_Z_CM = 35.0f;
    // 相机始终保持在人脸正前方的水平距离 (cm)
    static const float TRACK_DIST_CM = 57.0f;   // 使得 yaw=0 时相机 y=67
    // 相机比人脸固定高出的高度 (cm)
    static const float CAM_HEIGHT_OFFSET_CM = 5.0f;  // z = 35 + 5 = 40
    // 舵机角度固定（上电初始值，验证过程中不变）
    static const float SERVO1_DEG = 90.0f;
    static const float SERVO2_DEG = 157.0f;

    // 从 PnP 获取人脸偏航角/俯仰角（rad）
    //   head_yaw ≈ 0  : 人脸正对前方 (+Y)
    //   head_yaw > 0  : 人脸右转 (+X方向)，相机应跟随右摆
    //   head_yaw < 0  : 人脸左转 (-X方向)，相机应跟随左摆
    //   head_pitch ≈ 0: 人脸正视前方（不抬头不低头）
    //   head_pitch > 0: 人脸抬头，相机应升高（或降低，实测后反向即可）
    //   head_pitch < 0: 人脸低头，相机应降低（或升高）
    // 如果实测发现方向相反，给 head_yaw / head_pitch 加负号即可
    float yaw_rad = (float)head_yaw;
    float pitch_rad = -(float)head_pitch;  // 极性取反，实测后确认方向

    // ---------- 偏航方向：带死区的积分跟随 ----------
    static const float DEAD_ZONE_RAD = 5.0f * (float)M_PI / 180.0f;
    static const float CHASE_K = 0.08f;
    static float cum_yaw_offset = 0.0f;

    if (yaw_rad > DEAD_ZONE_RAD) {
        cum_yaw_offset += CHASE_K * (yaw_rad - DEAD_ZONE_RAD);
    } else if (yaw_rad < -DEAD_ZONE_RAD) {
        cum_yaw_offset += CHASE_K * (yaw_rad + DEAD_ZONE_RAD);
    }
    if (cum_yaw_offset > (float)M_PI / 2.0f) cum_yaw_offset = (float)M_PI / 2.0f;
    if (cum_yaw_offset < -(float)M_PI / 2.0f) cum_yaw_offset = -(float)M_PI / 2.0f;

    // ---------- 俯仰方向：PI 控制 + 基准偏移 + 死区衰减 ----------
    // P 项提供快速响应，I 项消除稳态误差，死区衰减防止卡死
    static const float PITCH_BIAS_DEG = 11.5f;
    float pitch_calib = pitch_rad - PITCH_BIAS_DEG * (float)M_PI / 180.0f;

    static const float DEAD_ZONE_PITCH_RAD = 3.0f * (float)M_PI / 180.0f;
    static const float KP_PITCH = 2.0f;      // 比例系数，越大响应越快
    static const float KI_PITCH = 0.06f;     // 积分系数
    static const float MAX_I_PITCH = (float)M_PI / 9.0f;  // 积分限幅 ±20°
    static float cum_pitch_offset = 0.0f;

    float err = pitch_calib;
    float p_term = 0.0f;

    if (err > DEAD_ZONE_PITCH_RAD) {
        float eff_err = err - DEAD_ZONE_PITCH_RAD;
        p_term = KP_PITCH * eff_err;
        cum_pitch_offset += KI_PITCH * eff_err;
    } else if (err < -DEAD_ZONE_PITCH_RAD) {
        float eff_err = err + DEAD_ZONE_PITCH_RAD;
        p_term = KP_PITCH * eff_err;
        cum_pitch_offset += KI_PITCH * eff_err;
    } else {
        // 死区内：积分衰减，P 项为 0
        cum_pitch_offset *= 0.95f;
    }
    // 积分限幅
    if (cum_pitch_offset > MAX_I_PITCH) cum_pitch_offset = MAX_I_PITCH;
    if (cum_pitch_offset < -MAX_I_PITCH) cum_pitch_offset = -MAX_I_PITCH;

    // P + I 输出
    float pitch_output = p_term + cum_pitch_offset;
    // 总输出限幅 ±60°
    if (pitch_output > (float)M_PI / 3.0f) pitch_output = (float)M_PI / 3.0f;
    if (pitch_output < -(float)M_PI / 3.0f) pitch_output = -(float)M_PI / 3.0f;

    // 相机目标位置：水平面由偏航决定，高度由俯仰 PI 输出决定
    float target_x = FACE_X_CM + TRACK_DIST_CM * std::sin(cum_yaw_offset);
    float target_y = FACE_Y_CM + TRACK_DIST_CM * std::cos(cum_yaw_offset);
    static const float NEUTRAL_Z_CM = FACE_Z_CM + CAM_HEIGHT_OFFSET_CM;  // 40 cm
    static const float PITCH_RANGE_CM = 15.0f;
    float target_z = NEUTRAL_Z_CM + PITCH_RANGE_CM * std::sin(pitch_output);

    // 每 3 帧发送一次目标位姿（30fps 下约 10Hz）
    static int uart_send_cnt = 0;
    if (++uart_send_cnt % 3 == 0) {
        uart_send_arm_target(target_x, target_y, target_z,
                             SERVO1_DEG, SERVO2_DEG);
    }

    // ---------- 验证打印 ----------
    printf("\n========== [VERIFY] 人脸跟随验证 ==========\n");
    printf("[VERIFY] 人脸位置: (%.1f, %.1f, %.1f) cm\n",
           FACE_X_CM, FACE_Y_CM, FACE_Z_CM);
    printf("[VERIFY] PnP  yaw=%.2f pitch_raw=%.2f pitch_calib=%.2f (deg)\n",
           yaw_rad * 180.0f / (float)M_PI,
           pitch_rad * 180.0f / (float)M_PI,
           pitch_calib * 180.0f / (float)M_PI);
    printf("[VERIFY] 偏航累积: yaw_off=%.2f deg | 俯仰 PI: P=%.2f I=%.2f out=%.2f (deg)\n",
           cum_yaw_offset * 180.0f / (float)M_PI,
           p_term * 180.0f / (float)M_PI,
           cum_pitch_offset * 180.0f / (float)M_PI,
           pitch_output * 180.0f / (float)M_PI);
    printf("[VERIFY] 相机目标: (%.1f, %.1f, %.1f) cm | 舵机: %.1f, %.1f\n",
           target_x, target_y, target_z, SERVO1_DEG, SERVO2_DEG);
    printf("[VERIFY] PnP tvec=(%.1f, %.1f, %.1f) mm | pos_err(yaw=%.2f, pitch=%.2f deg)\n",
           tx, ty, tz, pye_deg, ppe_deg);
    printf("[VERIFY] => P 项快速响应，I 项消除稳态；若震荡改小 KP_PITCH\n");
    printf("========== [VERIFY] END ==========\n\n");
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
    if (exp == 255) return (sign << 15) | 0x7C00;
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
            det.kps[k].visibility = kv;
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
            if (det.kps[k].visibility > 0.5f) {
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
    in_attr.index = 0;
    rknn_query(*ctx, RKNN_QUERY_INPUT_ATTR, &in_attr, sizeof(in_attr));
    printf("[NPU] %s Input: [%d,%d,%d,%d] type=%d\n", name,
           in_attr.dims[0], in_attr.dims[1], in_attr.dims[2], in_attr.dims[3], in_attr.type);

    out_attr->index = 0;
    rknn_query(*ctx, RKNN_QUERY_OUTPUT_ATTR, out_attr, sizeof(*out_attr));
    printf("[NPU] %s Output: [%d,%d,%d,%d] type=%d elems=%d\n", name,
           out_attr->dims[0], out_attr->dims[1], out_attr->dims[2], out_attr->dims[3],
           out_attr->type, out_attr->n_elems);

    int es = (out_attr->type == RKNN_TENSOR_FLOAT16) ? 2 : 4;
    *out_mem = rknn_create_mem(*ctx, out_attr->n_elems * es);
    if (!*out_mem) {
        printf("[ERROR] %s failed to alloc output memory\n", name);
        return -1;
    }
    return 0;
}

int init_npu() {
    printf("\n========== NPU Init ==========\n");

    if (init_single_model(FACE_MODEL_PATH, &face_ctx, &face_output_attr, &face_output_mem, "Face") != 0)
        return -1;
    if (init_single_model(BODY_MODEL_PATH, &body_ctx, &body_output_attr, &body_output_mem, "Body") != 0)
        return -1;

    // 分配公共缓冲区
    if (posix_memalign((void**)&rga_input_buf, 64, MODEL_INPUT_SIZE*MODEL_INPUT_SIZE*3/2) != 0 ||
        posix_memalign((void**)&rga_output_buf, 64, MODEL_INPUT_SIZE*MODEL_INPUT_SIZE*3) != 0 ||
        posix_memalign((void**)&npu_input_fp32, 64, MODEL_INPUT_SIZE*MODEL_INPUT_SIZE*3*sizeof(float)) != 0 ||
        posix_memalign((void**)&npu_input_fp16, 64, MODEL_INPUT_SIZE*MODEL_INPUT_SIZE*3*sizeof(uint16_t)) != 0) {
        printf("[ERROR] Buffer alloc failed\n");
        return -1;
    }

    printf("[NPU] Init OK (Face + Body loaded)\n\n");
    npu_initialized = 1;
    return 0;
}

int init_rga() { return 0; }

// ========== 帧处理 ==========
static void process_frame_face(uint8_t *nv12, int width, int height) {
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
    rknn_inputs_set(face_ctx, 1, &in);

    rknn_set_io_mem(face_ctx, face_output_mem, &face_output_attr);
    uint64_t t2 = get_us();

    if (rknn_run(face_ctx, NULL) < 0) {
        printf("[NPU] face rknn_run failed\n");
        return;
    }
    uint64_t t3 = get_us();

    std::vector<PoseDetection> detections;
    if (face_output_attr.type == RKNN_TENSOR_FLOAT16) {
        post_process_face((uint16_t*)face_output_mem->virt_addr, width, height, detections);
    }
    uint64_t t4 = get_us();

    static FaceTracker tracker(MODE_FACE, 20);
    if (tracker.mode != MODE_FACE) tracker.reset_mode(MODE_FACE, 20);
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
        // 画6个面部关键点（白点+编号），不画检测框
        const auto& det = display_dets[0];
        uint8_t* y_plane = nv12;
        const int face_kids[6] = {0, 1, 2, 17, 18, 19};
        for (int i = 0; i < 6; i++) {
            int k = face_kids[i];
            if (det.kps[k].visibility > 0.5f) {
                int kx = (int)det.kps[k].x;
                int ky = (int)det.kps[k].y;
                if (kx >= 2 && ky >= 2 && kx < width - 2 && ky < height - 2) {
                    for (int dy = -1; dy <= 1; dy++) {
                        for (int dx = -1; dx <= 1; dx++) {
                            y_plane[(ky + dy) * width + (kx + dx)] = 255;
                        }
                    }
                    draw_number(y_plane, width, height, kx + 3, ky + 3, k);
                }
            }
        }
        estimate_and_draw_pose(nv12, width, height, display_dets[0]);
    }
    uint64_t t6 = get_us();

    // 记录统计
    int idx = g_ai_stat_idx;
    g_ai_stat[idx].rga_us     = t1 - t0;
    g_ai_stat[idx].npu_set_us = t2 - t1;
    g_ai_stat[idx].npu_run_us = t3 - t2;
    g_ai_stat[idx].post_us    = t4 - t3;
    g_ai_stat[idx].track_us   = t5 - t4;
    g_ai_stat[idx].draw_us    = t6 - t5;
    g_ai_stat[idx].total_us   = t6 - t0;
    g_ai_stat_idx = (g_ai_stat_idx + 1) % AI_STAT_WINDOW;
    // 隐藏 AI-STAT 输出：设为 1 恢复打印
    #define ENABLE_AI_STAT 0
    #if ENABLE_AI_STAT
    if (++g_ai_stat_count % AI_STAT_WINDOW == 0) {
        print_ai_stats();
    }
    #endif
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
    int idx = g_ai_stat_idx;
    g_ai_stat[idx].rga_us     = t1 - t0;
    g_ai_stat[idx].npu_set_us = t2 - t1;
    g_ai_stat[idx].npu_run_us = t3 - t2;
    g_ai_stat[idx].post_us    = t4 - t3;
    g_ai_stat[idx].track_us   = t5 - t4;
    g_ai_stat[idx].draw_us    = t6 - t5;
    g_ai_stat[idx].total_us   = t6 - t0;
    g_ai_stat_idx = (g_ai_stat_idx + 1) % AI_STAT_WINDOW;
    #if ENABLE_AI_STAT
    if (++g_ai_stat_count % AI_STAT_WINDOW == 0) {
        print_ai_stats();
    }
    #endif
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
    if (face_output_mem) rknn_destroy_mem(face_ctx, face_output_mem);
    if (body_output_mem) rknn_destroy_mem(body_ctx, body_output_mem);
    if (face_ctx) rknn_destroy(face_ctx);
    if (body_ctx) rknn_destroy(body_ctx);

    pose_filter_init = false;
    have_prev_pose = false;
    prev_rvec.release();
    prev_tvec.release();

    free(rga_input_buf);
    free(rga_output_buf);
    free(npu_input_fp32);
    free(npu_input_fp16);
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

