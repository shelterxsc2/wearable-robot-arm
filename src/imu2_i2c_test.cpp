// IMU2 polarity test tool.
// Build:
//   g++ -std=c++17 -O2 src/imu2_i2c_test.cpp src/imu2_i2c.c -o build/imu2_i2c_test -lpthread
// Run:
//   sudo ./build/imu2_i2c_test
//   sudo ./build/imu2_i2c_test --out /tmp/imu2_left_right.txt

#include "imu2_i2c.h"

#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <pthread.h>
#include <unistd.h>

static volatile sig_atomic_t g_stop = 0;

static void on_signal(int)
{
    g_stop = 1;
}

static float normalize_angle_deg(float a)
{
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

static const char* sign_label(float v, float deadband)
{
    if (v > deadband) return "+";
    if (v < -deadband) return "-";
    return "0";
}

static int read_imu2(float* roll, float* pitch, float* yaw,
                     float* gx, float* gy, float* gz,
                     uint32_t* sample_count)
{
    int valid = 0;
    pthread_mutex_lock(&g_imu2_state.mutex);
    valid = g_imu2_state.valid ? 1 : 0;
    *roll = g_imu2_state.roll;
    *pitch = g_imu2_state.pitch;
    *yaw = g_imu2_state.yaw;
    *gx = g_imu2_state.gx;
    *gy = g_imu2_state.gy;
    *gz = g_imu2_state.gz;
    *sample_count = g_imu2_state.sample_count;
    pthread_mutex_unlock(&g_imu2_state.mutex);
    return valid;
}

int main(int argc, char** argv)
{
    int baseline_samples = 100;
    int print_period_ms = 100;
    const char* out_path = "/tmp/imu2_i2c_test.txt";

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--baseline") == 0 && i + 1 < argc) {
            baseline_samples = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--period-ms") == 0 && i + 1 < argc) {
            print_period_ms = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--out") == 0 && i + 1 < argc) {
            out_path = argv[++i];
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printf("Usage: %s [--baseline N] [--period-ms MS] [--out PATH]\n", argv[0]);
            return 0;
        } else {
            fprintf(stderr, "Unknown arg: %s\n", argv[i]);
            return 2;
        }
    }

    if (baseline_samples <= 0) baseline_samples = 100;
    if (print_period_ms < 20) print_period_ms = 20;

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    if (imu2_i2c_init() != 0) {
        return 1;
    }

    pthread_t tid = 0;
    if (imu2_i2c_thread_start(&tid) != 0) {
        imu2_i2c_deinit();
        return 1;
    }

    printf("\n[IMU2-TEST] Keep IMU2 fixed/still for baseline (%d samples, about %.1fs).\n",
           baseline_samples, baseline_samples * 0.01f);

    float sum_r = 0.0f, sum_p = 0.0f, sum_y = 0.0f;
    int got = 0;
    uint32_t last_sample = 0;

    while (!g_stop && got < baseline_samples) {
        float r, p, y, gx, gy, gz;
        uint32_t sc;
        if (read_imu2(&r, &p, &y, &gx, &gy, &gz, &sc) && sc != last_sample) {
            last_sample = sc;
            sum_r += r;
            sum_p += p;
            sum_y += y;
            got++;
        }
        usleep(2000);
    }

    if (g_stop || got == 0) {
        imu2_i2c_thread_stop();
        imu2_i2c_deinit();
        return 1;
    }

    const float base_r = sum_r / got;
    const float base_p = sum_p / got;
    const float base_y = sum_y / got;

    FILE* log_fp = fopen(out_path, "w");
    if (!log_fp) {
        fprintf(stderr, "[IMU2-TEST] Failed to open output %s\n", out_path);
        imu2_i2c_thread_stop();
        imu2_i2c_deinit();
        return 1;
    }
    setbuf(log_fp, NULL);

    time_t wall_now = time(NULL);
    fprintf(log_fp, "# IMU2 polarity test log\n");
    fprintf(log_fp, "# start_unix=%ld\n", (long)wall_now);
    fprintf(log_fp, "# baseline_samples=%d print_period_ms=%d\n",
            baseline_samples, print_period_ms);
    fprintf(log_fp, "# baseline_roll=%.6f baseline_pitch=%.6f baseline_yaw=%.6f\n",
            base_r, base_p, base_y);
    fprintf(log_fp, "# Describe the motion sequence separately when sharing this log.\n");
    fprintf(log_fp, "t_ms,sample,roll,pitch,yaw,droll,dpitch,dyaw,gx,gy,gz,yaw_sign,gz_sign\n");

    printf("[IMU2-TEST] Baseline: roll=%+.2f pitch=%+.2f yaw=%+.2f (n=%d)\n",
           base_r, base_p, base_y, got);
    printf("[IMU2-TEST] Logging CSV to: %s\n", out_path);
    printf("[IMU2-TEST] Now rotate the arm/body IMU left/right.\n");
    printf("[IMU2-TEST] Watch dyaw and gz signs. Ctrl+C to stop.\n\n");
    printf(" sample | roll pitch yaw | droll dpitch dyaw | gx gy gz | yaw_sign gz_sign\n");
    printf("--------+----------------+--------------------+----------+-----------------\n");

    uint64_t row = 0;
    while (!g_stop) {
        float r, p, y, gx, gy, gz;
        uint32_t sc;
        if (read_imu2(&r, &p, &y, &gx, &gy, &gz, &sc)) {
            const float dr = normalize_angle_deg(r - base_r);
            const float dp = normalize_angle_deg(p - base_p);
            const float dy = normalize_angle_deg(y - base_y);
            const uint64_t t_ms = row * (uint64_t)print_period_ms;
            const char* yaw_s = sign_label(dy, 1.0f);
            const char* gz_s = sign_label(gz, 3.0f);
            printf("%7u | %+5.1f %+5.1f %+5.1f | %+6.1f %+6.1f %+6.1f | %+6.1f %+6.1f %+6.1f | %s %s\n",
                   sc, r, p, y, dr, dp, dy, gx, gy, gz,
                   yaw_s, gz_s);
            fprintf(log_fp, "%llu,%u,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%s,%s\n",
                    (unsigned long long)t_ms, sc,
                    r, p, y, dr, dp, dy, gx, gy, gz, yaw_s, gz_s);
            row++;
        } else {
            printf("[IMU2-TEST] invalid sample\n");
            fprintf(log_fp, ",,,,,,,,,,,invalid,invalid\n");
        }
        usleep((useconds_t)print_period_ms * 1000);
    }

    printf("\n[IMU2-TEST] Stopping...\n");
    printf("[IMU2-TEST] Log saved: %s\n", out_path);
    fclose(log_fp);
    imu2_i2c_thread_stop();
    imu2_i2c_deinit();
    return 0;
}
