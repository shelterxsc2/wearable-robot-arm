// Head direction vector test tool.
//
// Build:
//   g++ -std=c++17 -O2 src/head_vector_test.cpp src/nrf24_linux.c src/imu2_i2c.c \
//       -o build/head_vector_test -lpthread
//
// Run:
//   sudo ./build/head_vector_test
//   sudo ./build/head_vector_test --center-samples 120 --period-ms 250

#include "nrf24_linux.h"
#include "imu2_i2c.h"

#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <pthread.h>
#include <unistd.h>

volatile int g_wait_a_init = 0;
volatile int g_r_init_set = 0;

static volatile sig_atomic_t g_stop = 0;

static void on_signal(int)
{
    g_stop = 1;
}

struct Vec3 {
    float x, y, z;
};

struct Mat3 {
    float m[3][3];
};

struct ImuSample {
    float roll, pitch, yaw;
    float qw, qx, qy, qz;
    float wx, wy, wz;
    bool quat_valid;
    bool valid;
    uint32_t count;
};

static Mat3 mat_mul(const Mat3& a, const Mat3& b)
{
    Mat3 r{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 3; ++k) {
                s += a.m[i][k] * b.m[k][j];
            }
            r.m[i][j] = s;
        }
    }
    return r;
}

static Mat3 mat_t(const Mat3& a)
{
    Mat3 r{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            r.m[i][j] = a.m[j][i];
        }
    }
    return r;
}

static Vec3 mat_vec(const Mat3& a, const Vec3& v)
{
    return {
        a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z,
        a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z,
        a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z
    };
}

static float vec_dot(const Vec3& a, const Vec3& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static float vec_norm(const Vec3& v)
{
    return sqrtf(vec_dot(v, v));
}

static Vec3 vec_scale(const Vec3& v, float s)
{
    return {v.x * s, v.y * s, v.z * s};
}

static Vec3 vec_add(const Vec3& a, const Vec3& b)
{
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

static Vec3 vec_normalize_or(const Vec3& v, const Vec3& fallback)
{
    float n = vec_norm(v);
    if (n < 1e-6f) return fallback;
    return vec_scale(v, 1.0f / n);
}

static Mat3 euler_zyx_to_mat(float roll_deg, float pitch_deg, float yaw_deg)
{
    const float d2r = (float)M_PI / 180.0f;
    float r = roll_deg * d2r;
    float p = pitch_deg * d2r;
    float y = yaw_deg * d2r;
    float cr = cosf(r), sr = sinf(r);
    float cp = cosf(p), sp = sinf(p);
    float cy = cosf(y), sy = sinf(y);

    Mat3 R{};
    R.m[0][0] = cy * cp;
    R.m[0][1] = cy * sp * sr - sy * cr;
    R.m[0][2] = cy * sp * cr + sy * sr;
    R.m[1][0] = sy * cp;
    R.m[1][1] = sy * sp * sr + cy * cr;
    R.m[1][2] = sy * sp * cr - cy * sr;
    R.m[2][0] = -sp;
    R.m[2][1] = cp * sr;
    R.m[2][2] = cp * cr;
    return R;
}

static Mat3 quat_to_mat(float qw, float qx, float qy, float qz)
{
    float n = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
    if (n < 1e-6f) {
        Mat3 I{};
        I.m[0][0] = 1.0f;
        I.m[1][1] = 1.0f;
        I.m[2][2] = 1.0f;
        return I;
    }
    qw /= n; qx /= n; qy /= n; qz /= n;

    Mat3 R{};
    R.m[0][0] = 1.0f - 2.0f * (qy * qy + qz * qz);
    R.m[0][1] = 2.0f * (qx * qy - qz * qw);
    R.m[0][2] = 2.0f * (qx * qz + qy * qw);
    R.m[1][0] = 2.0f * (qx * qy + qz * qw);
    R.m[1][1] = 1.0f - 2.0f * (qx * qx + qz * qz);
    R.m[1][2] = 2.0f * (qy * qz - qx * qw);
    R.m[2][0] = 2.0f * (qx * qz - qy * qw);
    R.m[2][1] = 2.0f * (qy * qz + qx * qw);
    R.m[2][2] = 1.0f - 2.0f * (qx * qx + qy * qy);
    return R;
}

static void mat_to_euler_zyx(const Mat3& R, float& roll_deg, float& pitch_deg, float& yaw_deg)
{
    float sy = sqrtf(R.m[0][0] * R.m[0][0] + R.m[1][0] * R.m[1][0]);
    float rr, pp, yy;
    if (sy >= 1e-6f) {
        yy = atan2f(R.m[1][0], R.m[0][0]);
        pp = atan2f(-R.m[2][0], sy);
        rr = atan2f(R.m[2][1], R.m[2][2]);
    } else {
        yy = atan2f(-R.m[1][2], R.m[1][1]);
        pp = atan2f(-R.m[2][0], sy);
        rr = 0.0f;
    }
    roll_deg = rr * 180.0f / (float)M_PI;
    pitch_deg = pp * 180.0f / (float)M_PI;
    yaw_deg = yy * 180.0f / (float)M_PI;
}

static Vec3 mat_to_rotvec_deg(const Mat3& R)
{
    float trace = R.m[0][0] + R.m[1][1] + R.m[2][2];
    float c = (trace - 1.0f) * 0.5f;
    if (c > 1.0f) c = 1.0f;
    if (c < -1.0f) c = -1.0f;
    float angle = acosf(c);
    if (angle < 1e-5f) {
        return {
            (R.m[2][1] - R.m[1][2]) * 0.5f * 180.0f / (float)M_PI,
            (R.m[0][2] - R.m[2][0]) * 0.5f * 180.0f / (float)M_PI,
            (R.m[1][0] - R.m[0][1]) * 0.5f * 180.0f / (float)M_PI
        };
    }

    float s = 2.0f * sinf(angle);
    if (fabsf(s) < 1e-6f) {
        return {0.0f, 0.0f, 0.0f};
    }
    Vec3 axis{
        (R.m[2][1] - R.m[1][2]) / s,
        (R.m[0][2] - R.m[2][0]) / s,
        (R.m[1][0] - R.m[0][1]) / s
    };
    return vec_scale(axis, angle * 180.0f / (float)M_PI);
}

static ImuSample read_head(void)
{
    ImuSample s{};
    pthread_mutex_lock(&g_nrf24_state.mutex);
    s.roll = g_nrf24_state.gy_roll;
    s.pitch = g_nrf24_state.gy_pitch;
    s.yaw = g_nrf24_state.gy_yaw;
    s.qw = g_nrf24_state.gy_qw;
    s.qx = g_nrf24_state.gy_qx;
    s.qy = g_nrf24_state.gy_qy;
    s.qz = g_nrf24_state.gy_qz;
    s.wx = g_nrf24_state.gy_wx;
    s.wy = g_nrf24_state.gy_wy;
    s.wz = g_nrf24_state.gy_wz;
    s.quat_valid = g_nrf24_state.quat_valid;
    s.valid = g_nrf24_state.imu_valid;
    s.count = g_nrf24_state.rx_count;
    pthread_mutex_unlock(&g_nrf24_state.mutex);
    return s;
}

static ImuSample read_waist(void)
{
    ImuSample s{};
    pthread_mutex_lock(&g_imu2_state.mutex);
    s.roll = g_imu2_state.roll;
    s.pitch = g_imu2_state.pitch;
    s.yaw = g_imu2_state.yaw;
    s.wx = g_imu2_state.gx;
    s.wy = g_imu2_state.gy;
    s.wz = g_imu2_state.gz;
    s.valid = g_imu2_state.valid && g_imu2_state.sample_count > 0;
    s.count = g_imu2_state.sample_count;
    pthread_mutex_unlock(&g_imu2_state.mutex);
    return s;
}

static const char* horizontal_word(float yaw_deg)
{
    if (yaw_deg > 3.0f) return "偏右";
    if (yaw_deg < -3.0f) return "偏左";
    return "正中";
}

static const char* vertical_word(float pitch_deg)
{
    if (pitch_deg > 3.0f) return "偏上";
    if (pitch_deg < -3.0f) return "偏下";
    return "水平";
}

static const char* combined_word(float yaw_deg, float pitch_deg)
{
    const bool left = yaw_deg < -3.0f;
    const bool right = yaw_deg > 3.0f;
    const bool up = pitch_deg > 3.0f;
    const bool down = pitch_deg < -3.0f;

    if (left && up) return "朝左上";
    if (right && up) return "朝右上";
    if (left && down) return "朝左下";
    if (right && down) return "朝右下";
    if (left) return "朝左";
    if (right) return "朝右";
    if (up) return "朝上";
    if (down) return "朝下";
    return "朝中";
}

static bool collect_center(int samples, Mat3& head_init, Mat3& waist_init)
{
    double hr = 0, hp = 0, hy = 0;
    double hqw = 0, hqx = 0, hqy = 0, hqz = 0;
    bool have_q_ref = false;
    float ref_qw = 1.0f, ref_qx = 0.0f, ref_qy = 0.0f, ref_qz = 0.0f;
    double wr = 0, wp = 0, wy = 0;
    int got = 0;
    uint32_t last_h = 0;
    uint32_t last_w = 0;

    while (!g_stop && got < samples) {
        ImuSample h = read_head();
        ImuSample w = read_waist();
        if (h.valid && h.quat_valid && w.valid && h.count != last_h && w.count != last_w) {
            last_h = h.count;
            last_w = w.count;
            hr += h.roll; hp += h.pitch; hy += h.yaw;
            if (!have_q_ref) {
                ref_qw = h.qw; ref_qx = h.qx; ref_qy = h.qy; ref_qz = h.qz;
                have_q_ref = true;
            }
            float qw = h.qw, qx = h.qx, qy = h.qy, qz = h.qz;
            float dot = qw * ref_qw + qx * ref_qx + qy * ref_qy + qz * ref_qz;
            if (dot < 0.0f) {
                qw = -qw; qx = -qx; qy = -qy; qz = -qz;
            }
            hqw += qw; hqx += qx; hqy += qy; hqz += qz;
            wr += w.roll; wp += w.pitch; wy += w.yaw;
            got++;
        }
        usleep(2000);
    }

    if (got <= 0) return false;

    float aqw = (float)(hqw / got);
    float aqx = (float)(hqx / got);
    float aqy = (float)(hqy / got);
    float aqz = (float)(hqz / got);
    head_init = quat_to_mat(aqw, aqx, aqy, aqz);
    waist_init = euler_zyx_to_mat((float)(wr / got), (float)(wp / got), (float)(wy / got));
    printf("[CENTER] samples=%d head_rpy=%.2f/%.2f/%.2f head_q=%.4f/%.4f/%.4f/%.4f "
           "waist_rpy=%.2f/%.2f/%.2f\n",
           got, hr / got, hp / got, hy / got, aqw, aqx, aqy, aqz,
           wr / got, wp / got, wy / got);
    return true;
}

static void compute_vector_yaw_pitch(const ImuSample& h,
                                     const ImuSample& w,
                                     const Mat3& head_init,
                                     const Mat3& waist_init,
                                     float& yaw_deg,
                                     float& pitch_deg,
                                     Mat3* rel_out)
{
    const Vec3 forward_axis{0.0f, 1.0f, 0.0f};
    Mat3 Rh = h.quat_valid ? quat_to_mat(h.qw, h.qx, h.qy, h.qz)
                            : euler_zyx_to_mat(h.roll, h.pitch, h.yaw);
    Mat3 Rw = euler_zyx_to_mat(w.roll, w.pitch, w.yaw);
    Mat3 R_head_delta = mat_mul(Rh, mat_t(head_init));
    Mat3 R_waist_delta = mat_mul(Rw, mat_t(waist_init));
    Mat3 R_rel = mat_mul(mat_t(R_waist_delta), R_head_delta);

    Vec3 f = mat_vec(R_rel, forward_axis);
    yaw_deg = atan2f(f.x, f.y) * 180.0f / (float)M_PI;
    pitch_deg = atan2f(f.z, sqrtf(f.x * f.x + f.y * f.y)) *
                180.0f / (float)M_PI;
    if (rel_out) {
        *rel_out = R_rel;
    }
}

static int calibrate_pitch_sign(const Mat3& head_init,
                                const Mat3& waist_init)
{
    printf("[PITCH-SIGN] Raise head for 2.0s...\n");
    double sum = 0.0;
    int count = 0;
    int reject_yaw = 0;

    for (int i = 0; i < 40 && !g_stop; ++i) {
        ImuSample h = read_head();
        ImuSample w = read_waist();
        if (h.valid && w.valid) {
            float yaw = 0.0f;
            float pitch = 0.0f;
            compute_vector_yaw_pitch(h, w, head_init, waist_init, yaw, pitch, nullptr);
            if (fabsf(yaw) <= 45.0f && fabsf(pitch) >= 2.0f) {
                sum += pitch;
                count++;
            } else if (fabsf(yaw) > 45.0f) {
                reject_yaw++;
            }
        }
        usleep(50000);
    }

    float avg = count > 0 ? (float)(sum / count) : 0.0f;
    int sign = avg >= 0.0f ? 1 : -1;
    if (fabsf(avg) < 2.0f) {
        sign = 1;
        printf("[PITCH-SIGN] Too little pitch motion, fallback sign=+1\n");
    }
    printf("[PITCH-SIGN] done: avg_raw_pitch=%+.2f samples=%d rejected_yaw=%d sign=%+d "
           "(raise-head will be positive)\n",
           avg, count, reject_yaw, sign);
    return sign;
}

static float calibrate_yaw_pitch_coupling(const Mat3& head_init,
                                          const Mat3& waist_init,
                                          int duration_sec)
{
    printf("[YAW-PITCH] Keep head level, turn left/right for %d.0s...\n",
           duration_sec);
    double xy = 0.0;
    double xx = 0.0;
    int count = 0;

    int loops = duration_sec * 20;  /* 50ms/sample */
    if (loops < 20) loops = 20;
    for (int i = 0; i < loops && !g_stop; ++i) {
        ImuSample h = read_head();
        ImuSample w = read_waist();
        if (h.valid && w.valid) {
            float yaw = 0.0f;
            float pitch = 0.0f;
            compute_vector_yaw_pitch(h, w, head_init, waist_init, yaw, pitch, nullptr);
            if (fabsf(yaw) >= 5.0f && fabsf(yaw) <= 70.0f && fabsf(pitch) <= 45.0f) {
                xy += (double)yaw * (double)pitch;
                xx += (double)yaw * (double)yaw;
                count++;
            }
        }
        usleep(50000);
    }

    float k = (xx > 1e-6) ? (float)(xy / xx) : 0.0f;
    if (fabsf(k) > 1.0f) {
        printf("[YAW-PITCH] coupling too large %.3f, clamped to 0\n", k);
        k = 0.0f;
    }
    printf("[YAW-PITCH] done: pitch_raw ~= %.3f * yaw_raw, samples=%d\n", k, count);
    return k;
}

static Vec3 calibrate_motion_axis(const Mat3& head_init,
                                  const Mat3& waist_init,
                                  const char* tag,
                                  const char* prompt,
                                  int pitch_sign,
                                  bool prefer_yaw_axis,
                                  int duration_sec)
{
    printf("[%s] %s for %d.0s...\n", tag, prompt, duration_sec);
    Vec3 sum{0.0f, 0.0f, 0.0f};
    int count = 0;

    int loops = duration_sec * 20;  /* 50ms/sample */
    if (loops < 20) loops = 20;
    for (int i = 0; i < loops && !g_stop; ++i) {
        ImuSample h = read_head();
        ImuSample w = read_waist();
        if (h.valid && w.valid) {
            float yaw = 0.0f;
            float pitch = 0.0f;
            Mat3 R_rel{};
            compute_vector_yaw_pitch(h, w, head_init, waist_init, yaw, pitch, &R_rel);
            Vec3 rv = mat_to_rotvec_deg(R_rel);
            float mag = vec_norm(rv);
            if (mag >= 5.0f && mag <= 80.0f) {
                float orient = prefer_yaw_axis ? yaw : (pitch * (float)pitch_sign);
                if (fabsf(orient) >= 3.0f) {
                    if (orient < 0.0f) rv = vec_scale(rv, -1.0f);
                    sum = vec_add(sum, vec_normalize_or(rv, {0.0f, 0.0f, 0.0f}));
                    count++;
                }
            }
        }
        usleep(50000);
    }

    Vec3 fallback = prefer_yaw_axis ? Vec3{0.0f, 0.0f, 1.0f} : Vec3{1.0f, 0.0f, 0.0f};
    Vec3 axis = vec_normalize_or(sum, fallback);
    printf("[%s] done: axis=(%+.3f,%+.3f,%+.3f), samples=%d\n",
           tag, axis.x, axis.y, axis.z, count);
    return axis;
}

int main(int argc, char** argv)
{
    int center_samples = 120;
    int period_ms = 250;
    int axis_calib_sec = 8;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--center-samples") == 0 && i + 1 < argc) {
            center_samples = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--period-ms") == 0 && i + 1 < argc) {
            period_ms = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--axis-calib-sec") == 0 && i + 1 < argc) {
            axis_calib_sec = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printf("Usage: %s [--center-samples N] [--period-ms MS] [--axis-calib-sec SEC]\n", argv[0]);
            return 0;
        } else {
            fprintf(stderr, "Unknown arg: %s\n", argv[i]);
            return 2;
        }
    }
    if (center_samples < 10) center_samples = 10;
    if (period_ms < 50) period_ms = 50;
    if (axis_calib_sec < 2) axis_calib_sec = 2;

    setbuf(stdout, NULL);
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    printf("[HEAD-VEC] Starting NRF24 head IMU + waist IMU direction test\n");
    printf("[HEAD-VEC] Keep head/body facing center during center capture.\n");

    if (nrf24_linux_init() != 0) {
        fprintf(stderr, "[HEAD-VEC] NRF24 init failed\n");
        return 1;
    }
    pthread_t nrf_tid = 0;
    if (nrf24_rx_thread_start(&nrf_tid) != 0) {
        fprintf(stderr, "[HEAD-VEC] NRF24 RX thread failed\n");
        nrf24_linux_deinit();
        return 1;
    }

    if (imu2_i2c_init() != 0) {
        nrf24_rx_thread_stop();
        nrf24_linux_deinit();
        return 1;
    }
    pthread_t imu2_tid = 0;
    if (imu2_i2c_thread_start(&imu2_tid) != 0) {
        imu2_i2c_deinit();
        nrf24_rx_thread_stop();
        nrf24_linux_deinit();
        return 1;
    }

    printf("[HEAD-VEC] Capturing center from %d samples...\n", center_samples);
    Mat3 head_init{}, waist_init{};
    if (!collect_center(center_samples, head_init, waist_init)) {
        fprintf(stderr, "[HEAD-VEC] Failed to capture center\n");
        imu2_i2c_thread_stop();
        imu2_i2c_deinit();
        nrf24_rx_thread_stop();
        nrf24_linux_deinit();
        return 1;
    }

    const int pitch_sign = -1;
    const Vec3 yaw_axis{+0.007f, +0.017f, -1.000f};
    const Vec3 pitch_axis{-0.256f, -0.967f, +0.016f};
    printf("[AXIS-FIXED] pitch_sign=%+d yaw_axis=(%+.3f,%+.3f,%+.3f) "
           "pitch_axis=(%+.3f,%+.3f,%+.3f)\n",
           pitch_sign,
           yaw_axis.x, yaw_axis.y, yaw_axis.z,
           pitch_axis.x, pitch_axis.y, pitch_axis.z);
    printf("[HEAD-VEC] Logging every %d ms. Ctrl-C to stop.\n", period_ms);
    printf("[HEAD-VEC] Format: fixed-axis direction | raw vector yaw/pitch | debug rel_euler | raw head/waist\n");

    while (!g_stop) {
        ImuSample h = read_head();
        ImuSample w = read_waist();
        if (!h.valid || !w.valid) {
            printf("[HEAD-VEC] waiting valid IMUs head=%d waist=%d\n", h.valid ? 1 : 0, w.valid ? 1 : 0);
            usleep(period_ms * 1000);
            continue;
        }

        Mat3 R_rel{};
        float yaw_deg = 0.0f;
        float pitch_deg = 0.0f;
        compute_vector_yaw_pitch(h, w, head_init, waist_init,
                                 yaw_deg, pitch_deg, &R_rel);
        float raw_yaw = yaw_deg;
        float raw_pitch = pitch_deg * (float)pitch_sign;
        Vec3 rv = mat_to_rotvec_deg(R_rel);
        float axis_yaw = vec_dot(rv, yaw_axis);
        float axis_pitch = vec_dot(rv, pitch_axis);
        yaw_deg = axis_yaw;
        pitch_deg = axis_pitch;

        float rr = 0.0f, rp = 0.0f, ry = 0.0f;
        mat_to_euler_zyx(R_rel, rr, rp, ry);

        float qnorm = sqrtf(h.qw * h.qw + h.qx * h.qx + h.qy * h.qy + h.qz * h.qz);
        printf("%s：%s %.1f度，%s %.1f度 | axis_yaw=%+.2f axis_pitch=%+.2f "
               "| raw_yaw=%+.2f raw_pitch=%+.2f "
               "| head_mode=%s qnorm=%.3f | rel_rpy=%+.2f/%+.2f/%+.2f | head_rpy=%+.1f/%+.1f/%+.1f "
               "| waist_rpy=%+.1f/%+.1f/%+.1f\n",
               combined_word(yaw_deg, pitch_deg),
               horizontal_word(yaw_deg), fabsf(yaw_deg),
               vertical_word(pitch_deg), fabsf(pitch_deg),
               yaw_deg, pitch_deg,
               raw_yaw, raw_pitch,
               h.quat_valid ? "quat" : "euler", qnorm,
               rr, rp, ry,
               h.roll, h.pitch, h.yaw,
               w.roll, w.pitch, w.yaw);

        usleep(period_ms * 1000);
    }

    printf("\n[HEAD-VEC] Stopping...\n");
    imu2_i2c_thread_stop();
    imu2_i2c_deinit();
    nrf24_rx_thread_stop();
    nrf24_linux_deinit();
    return 0;
}
