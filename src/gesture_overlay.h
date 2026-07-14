#ifndef GESTURE_OVERLAY_H
#define GESTURE_OVERLAY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Wrist ROI submitted to the async hand pipeline and gesture controller. */
typedef struct {
    int x;
    int y;
    int width;
    int height;
    int side; /* 0=left wrist, 1=right wrist */
} GestureHandRoi;

int gesture_overlay_init(void);
void gesture_overlay_set_hand_rois(const GestureHandRoi *rois, int count);
void gesture_overlay_submit_latest(const uint8_t *nv12, int width, int height);
void gesture_overlay_draw(uint8_t *nv12, int width, int height);
void gesture_overlay_shutdown(void);

#ifdef __cplusplus
}
#endif

#endif
