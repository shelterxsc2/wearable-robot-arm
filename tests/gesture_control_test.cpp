#include "gesture_control.h"

#include <cassert>

static GestureControlAction repeat(GestureControlState& state, int side,
                                   const char *label, PoseMode mode,
                                   unsigned int generation, uint64_t start_us,
                                   int count)
{
    GestureControlAction action = GESTURE_ACTION_NONE;
    for (int i = 0; i < count; ++i) {
        action = gesture_control_update(&state, side, label, 0.90f, mode,
                                        generation, start_us + i * 100000ULL);
    }
    return action;
}

int main()
{
    GestureControlState state{};

    assert(repeat(state, 0, "Open_Palm", MODE_FACE, 1, 1000000, 2) ==
           GESTURE_ACTION_NONE);
    assert(repeat(state, 0, "Open_Palm", MODE_FACE, 1, 1200000, 1) ==
           GESTURE_ACTION_MODE_INTRO);

    // Current canned model maps trial0's OK semantic to ILoveYou.
    assert(repeat(state, 0, "Closed_Fist", MODE_INTRO, 2, 2000000, 6) ==
           GESTURE_ACTION_NONE);
    assert(repeat(state, 1, "ILoveYou", MODE_INTRO, 2, 3000000, 3) ==
           GESTURE_ACTION_POWER_CONFIRM);

    // Pointing_Up is side-dependent in FACE.
    assert(repeat(state, 0, "Pointing_Up", MODE_FACE, 3, 5000000, 3) ==
           GESTURE_ACTION_PROFILE_NEAR);
    assert(repeat(state, 1, "Pointing_Up", MODE_FACE, 3, 7000000, 3) ==
           GESTURE_ACTION_PROFILE_FAR);

    // Low confidence and a generation change both reset confirmation.
    gesture_control_reset(&state, 4);
    assert(repeat(state, 0, "Open_Palm", MODE_FACE, 4, 9000000, 2) ==
           GESTURE_ACTION_NONE);
    assert(gesture_control_update(&state, 0, "Open_Palm", 0.69f, MODE_FACE,
                                  4, 9400000) == GESTURE_ACTION_NONE);
    assert(repeat(state, 0, "Open_Palm", MODE_FACE, 4, 9500000, 2) ==
           GESTURE_ACTION_NONE);
    assert(gesture_control_update(&state, 0, "Open_Palm", 0.90f, MODE_FACE,
                                  5, 9900000) == GESTURE_ACTION_NONE);

    // BODY and FIRST_PERSON never accept gesture actions.
    assert(repeat(state, 0, "Open_Palm", MODE_BODY, 6, 11000000, 6) ==
           GESTURE_ACTION_NONE);
    assert(repeat(state, 0, "ILoveYou", MODE_FIRST_PERSON, 7, 12000000, 6) ==
           GESTURE_ACTION_NONE);
    return 0;
}
