#ifndef RULE_MODE_CONTROL_H
#define RULE_MODE_CONTROL_H

enum RuleModeAction {
    RULE_MODE_ACTION_NONE = 0,
    RULE_MODE_ACTION_INTRO,
    RULE_MODE_ACTION_INTERVIEW,
};

struct RuleModeControlState {
    bool initialized = false;
    int previous_state = 0;
};

void rule_mode_control_reset(RuleModeControlState *state);
RuleModeAction rule_mode_control_update(RuleModeControlState *state,
                                        int current_state);
const char *rule_mode_action_name(RuleModeAction action);

#endif
