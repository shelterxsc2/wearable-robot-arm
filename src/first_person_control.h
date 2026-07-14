#ifndef FIRST_PERSON_CONTROL_H
#define FIRST_PERSON_CONTROL_H

struct FirstPersonServoTarget {
    float j4_deg;
    float j5_deg;
};

FirstPersonServoTarget first_person_map_head(float yaw_deg,
                                             float signed_pitch_deg);

#endif
