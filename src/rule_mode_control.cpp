#include "rule_mode_control.h"

void rule_mode_control_reset(RuleModeControlState *state)
{
    if (state) *state = RuleModeControlState{};
}

RuleModeAction rule_mode_control_update(RuleModeControlState *state,
                                        int current_state)
{
    if (!state || current_state < 0 || current_state > 2)
        return RULE_MODE_ACTION_NONE;

    if (!state->initialized) {
        state->initialized = true;
        state->previous_state = current_state;
        return RULE_MODE_ACTION_NONE;
    }

    int previous_state = state->previous_state;
    state->previous_state = current_state;
    if (previous_state == 0 && current_state == 1)
        return RULE_MODE_ACTION_INTRO;
    if (previous_state == 0 && current_state == 2)
        return RULE_MODE_ACTION_INTERVIEW;
    return RULE_MODE_ACTION_NONE;
}

const char *rule_mode_action_name(RuleModeAction action)
{
    switch (action) {
        case RULE_MODE_ACTION_INTRO: return "mode0 -> mode1: INTRO";
        case RULE_MODE_ACTION_INTERVIEW: return "mode0 -> mode2: INTERVIEW";
        case RULE_MODE_ACTION_NONE: break;
    }
    return "none";
}
