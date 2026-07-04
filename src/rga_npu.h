#ifndef RGA_NPU_H
#define RGA_NPU_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MODE_FACE = 0,
    MODE_BODY = 1,
    MODE_INTRO = 2,
    MODE_INTERVIEW = 3
} PoseMode;

int init_npu();
int init_rga();
void process_frame(uint8_t *nv12, int width, int height);
void cleanup_npu();
void cleanup_rga();

void set_pose_mode(PoseMode mode);
PoseMode get_pose_mode(void);
const char* pose_mode_name(PoseMode mode);

void set_arm_profile(int profile);
int get_arm_profile(void);
const char* arm_profile_name(int profile);
int toggle_head_pitch_sign(void);
int get_head_pitch_sign(void);

/* USB YUYV -> NV12 (via RGA hardware) */
int convert_yuyv_to_nv12(uint8_t *src, uint8_t *dst, int width, int height);

/* Frame receive timestamp for latency measurement */
void set_frame_start_time_us(uint64_t us);

/* GST pipeline timing reports */
void report_gst_getframe_us(uint64_t us);
void report_gst_encode_us(uint64_t us);

/* NRF24 IMU 独立控制链路（不依赖视觉帧） */
void nrf24_control_update(void);

/* A-inverse R_init 捕获标志: 1=已捕获 */
extern volatile int g_r_init_set;

/* 请求把当前头部相对姿态作为控制中心 */
extern volatile int g_head_center_request;

/* 握手线程信号: 置1后 nrf24_control_update() 在下一帧 IMU 时捕获 R_init */
extern volatile int g_wait_a_init;

/* 上位机握手状态: 0=wait_init, 1=a_init, 2=send_ff, 3=wait_homing, 4=normal */
extern volatile int g_host_state;

#ifdef __cplusplus
}
#endif

#endif // RGA_NPU_H
