#include "rule_mode_control.h"

#include <cassert>

int main()
{
    RuleModeControlState state{};

    // The first model result establishes the edge detector baseline.
    assert(rule_mode_control_update(&state, 0) == RULE_MODE_ACTION_NONE);
    assert(rule_mode_control_update(&state, 1) == RULE_MODE_ACTION_INTRO);
    assert(rule_mode_control_update(&state, 1) == RULE_MODE_ACTION_NONE);

    // Returning to model state 0 never exits a scenario automatically.
    assert(rule_mode_control_update(&state, 0) == RULE_MODE_ACTION_NONE);
    assert(rule_mode_control_update(&state, 2) == RULE_MODE_ACTION_INTERVIEW);
    assert(rule_mode_control_update(&state, 0) == RULE_MODE_ACTION_NONE);

    // Direct 1 -> 2 is not an entry edge; the model must pass through 0.
    rule_mode_control_reset(&state);
    assert(rule_mode_control_update(&state, 1) == RULE_MODE_ACTION_NONE);
    assert(rule_mode_control_update(&state, 2) == RULE_MODE_ACTION_NONE);
    assert(rule_mode_control_update(&state, 0) == RULE_MODE_ACTION_NONE);
    assert(rule_mode_control_update(&state, 2) == RULE_MODE_ACTION_INTERVIEW);

    // Invalid model output is ignored without corrupting the previous state.
    assert(rule_mode_control_update(&state, 9) == RULE_MODE_ACTION_NONE);
    assert(rule_mode_control_update(&state, 0) == RULE_MODE_ACTION_NONE);
    assert(rule_mode_control_update(&state, 1) == RULE_MODE_ACTION_INTRO);
    return 0;
}
