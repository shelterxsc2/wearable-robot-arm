#include "first_person_control.h"

#include <cassert>
#include <cmath>

static void near(float actual, float expected)
{
    assert(std::fabs(actual - expected) < 0.001f);
}

int main()
{
    FirstPersonServoTarget centered = first_person_map_head(0.0f, 0.0f);
    near(centered.j4_deg, 10.0f);
    near(centered.j5_deg, 180.0f);

    FirstPersonServoTarget moved = first_person_map_head(5.0f, 6.0f);
    near(moved.j4_deg, 4.0f);
    near(moved.j5_deg, 185.0f);

    FirstPersonServoTarget clamped = first_person_map_head(500.0f, 500.0f);
    near(clamped.j4_deg, -90.0f);
    near(clamped.j5_deg, 270.0f);
    return 0;
}
