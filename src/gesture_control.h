#ifndef GESTURE_CONTROL_H
#define GESTURE_CONTROL_H

#include "rga_npu.h"

#include <stdint.h>

enum GestureControlAction {
    GESTURE_ACTION_NONE = 0,
    GESTURE_ACTION_MODE_FACE,
    GESTURE_ACTION_MODE_INTRO,
    GESTURE_ACTION_MODE_INTERVIEW,
    GESTURE_ACTION_PROFILE_NEAR,
    GESTURE_ACTION_PROFILE_FAR,
    GESTURE_ACTION_POWER_CONFIRM,
};

struct GestureControlSideState {
    GestureControlAction candidate = GESTURE_ACTION_NONE;
    int count = 0;
};

struct GestureControlState {
    GestureControlSideState sides[2];
    uint64_t last_trigger_us = 0;
    unsigned int mode_generation = 0;
};

void gesture_control_reset(GestureControlState *state,
                           unsigned int mode_generation = 0);

GestureControlAction gesture_control_update(GestureControlState *state,
                                            int side,
                                            const char *label,
                                            float score,
                                            PoseMode mode,
                                            unsigned int mode_generation,
                                            uint64_t timestamp_us);

const char *gesture_control_action_name(GestureControlAction action);

#endif
