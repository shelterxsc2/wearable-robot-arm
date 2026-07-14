#include "gesture_control.h"

#include <cstring>

namespace {

constexpr float CONTROL_SCORE = 0.70f;
constexpr int CONTROL_CONFIRM_COUNT = 3;
constexpr uint64_t CONTROL_COOLDOWN_US = 1500000ULL;

GestureControlAction map_action(int side, const char *label, PoseMode mode)
{
    if (!label) return GESTURE_ACTION_NONE;

    // Scenario modes keep only the explicit escape gesture alive. BODY and
    // FIRST_PERSON do not accept gesture commands.
    if (mode == MODE_INTRO || mode == MODE_INTERVIEW) {
        return strcmp(label, "ILoveYou") == 0
                   ? GESTURE_ACTION_POWER_CONFIRM
                   : GESTURE_ACTION_NONE;
    }
    if (mode != MODE_FACE) return GESTURE_ACTION_NONE;

    if (strcmp(label, "ILoveYou") == 0) return GESTURE_ACTION_POWER_CONFIRM;
    if (strcmp(label, "Open_Palm") == 0 || strcmp(label, "Open") == 0) return GESTURE_ACTION_MODE_INTRO;
    if (strcmp(label, "Closed_Fist") == 0 || strcmp(label, "Close") == 0) return GESTURE_ACTION_MODE_INTERVIEW;
    if (strcmp(label, "Pointing_Up") == 0 || strcmp(label, "Pointer-UP") == 0) {
        return side == 0 ? GESTURE_ACTION_PROFILE_NEAR
                         : GESTURE_ACTION_PROFILE_FAR;
    }
    return GESTURE_ACTION_NONE;
}

}  // namespace

void gesture_control_reset(GestureControlState *state,
                           unsigned int mode_generation)
{
    if (!state) return;
    *state = GestureControlState{};
    state->mode_generation = mode_generation;
}

GestureControlAction gesture_control_update(GestureControlState *state,
                                            int side,
                                            const char *label,
                                            float score,
                                            PoseMode mode,
                                            unsigned int mode_generation,
                                            uint64_t timestamp_us)
{
    if (!state || side < 0 || side > 1) return GESTURE_ACTION_NONE;
    if (state->mode_generation != mode_generation) {
        state->sides[0] = GestureControlSideState{};
        state->sides[1] = GestureControlSideState{};
        state->mode_generation = mode_generation;
    }

    GestureControlAction action = score >= CONTROL_SCORE
                                      ? map_action(side, label, mode)
                                      : GESTURE_ACTION_NONE;
    GestureControlSideState& hand = state->sides[side];
    if (action == GESTURE_ACTION_NONE) {
        hand = GestureControlSideState{};
        return GESTURE_ACTION_NONE;
    }

    if (hand.candidate == action) {
        ++hand.count;
    } else {
        hand.candidate = action;
        hand.count = 1;
    }
    if (hand.count < CONTROL_CONFIRM_COUNT) return GESTURE_ACTION_NONE;
    if (state->last_trigger_us != 0 &&
        timestamp_us - state->last_trigger_us < CONTROL_COOLDOWN_US) {
        return GESTURE_ACTION_NONE;
    }

    state->last_trigger_us = timestamp_us;
    state->sides[0] = GestureControlSideState{};
    state->sides[1] = GestureControlSideState{};
    return action;
}

const char *gesture_control_action_name(GestureControlAction action)
{
    switch (action) {
        case GESTURE_ACTION_MODE_FACE: return "ILoveYou -> FACE";
        case GESTURE_ACTION_MODE_INTRO: return "Open_Palm -> INTRO";
        case GESTURE_ACTION_MODE_INTERVIEW: return "Closed_Fist -> INTERVIEW";
        case GESTURE_ACTION_PROFILE_NEAR: return "Left Pointing_Up -> near profile";
        case GESTURE_ACTION_PROFILE_FAR: return "Right Pointing_Up -> far profile";
        case GESTURE_ACTION_POWER_CONFIRM: return "ILoveYou (OK semantic) -> confirm retract or FACE";
        case GESTURE_ACTION_NONE: break;
    }
    return "none";
}
