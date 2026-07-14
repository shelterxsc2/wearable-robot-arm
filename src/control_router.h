#ifndef CONTROL_ROUTER_H
#define CONTROL_ROUTER_H

#include "rga_npu.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CONTROL_SOURCE_REST = 0,
    CONTROL_SOURCE_CLOUD,
    CONTROL_SOURCE_BLUETOOTH,
    CONTROL_SOURCE_GESTURE,
    CONTROL_SOURCE_VOICE,
    CONTROL_SOURCE_RULE_ENGINE,
    CONTROL_SOURCE_LOCAL
} ControlSource;

const char *control_source_name(ControlSource source);
int control_mode_from_string(const char *name, PoseMode *mode);
int control_request_mode(PoseMode mode, ControlSource source);
int control_request_profile(int profile, ControlSource source);
int control_request_mode_if_generation(PoseMode mode, ControlSource source,
                                       unsigned int expected_generation);
int control_request_profile_if_generation(int profile, ControlSource source,
                                          unsigned int expected_generation);
int control_request_first_person_target(float x, float y, float z,
                                        ControlSource source);
void control_set_annotation_enabled(int enabled, ControlSource source);
int control_get_annotation_enabled(void);
unsigned int control_get_mode_generation(void);

#ifdef __cplusplus
}
#endif

#endif
