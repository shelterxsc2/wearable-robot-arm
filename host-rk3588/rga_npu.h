#ifndef RGA_NPU_H
#define RGA_NPU_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { MODE_FACE = 0, MODE_BODY = 1 } PoseMode;

int init_npu();
int init_rga();
void process_frame(uint8_t *nv12, int width, int height);
void cleanup_npu();
void cleanup_rga();

void set_pose_mode(PoseMode mode);
PoseMode get_pose_mode(void);
const char* pose_mode_name(PoseMode mode);

/* USB YUYV -> NV12 (via RGA hardware) */
int convert_yuyv_to_nv12(uint8_t *src, uint8_t *dst, int width, int height);

/* Frame receive timestamp for latency measurement */
void set_frame_start_time_us(uint64_t us);

#ifdef __cplusplus
}
#endif

#endif // RGA_NPU_H
