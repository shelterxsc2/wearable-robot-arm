#include "first_person_control.h"

#include <algorithm>

FirstPersonServoTarget first_person_map_head(float yaw_deg,
                                             float signed_pitch_deg)
{
    FirstPersonServoTarget out;
    out.j4_deg = std::max(-90.0f, std::min(90.0f, 10.0f - signed_pitch_deg));
    out.j5_deg = std::max(0.0f, std::min(270.0f, 180.0f + yaw_deg));
    return out;
}
