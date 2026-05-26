#include "Control_Algorithm.h"
#include "DMJ4310_Motor_Driver.h"
#include "LK4005_Motor_Driver.h"

// 弧度归一化到0~2pi
float Normalize_Angle(float Angle)
{
    float Res = fmodf(Angle, 2.0f * PI);
    if (Res < 0)
    {
        Res += 2.0f * PI;
    }
    return Res;
}

/*
//达妙电机由于其特殊的CAN报文控制用不到这个函数
*/
void Motor_MIT_Control(Motor_MIT_Control_Handle_t *Motor_MIT_Control_Handle)
{
    Motor_MIT_Control_Handle->Output = Motor_MIT_Control_Handle->MIT_Kp * (Motor_MIT_Control_Handle->Motor_Position_Target - Motor_MIT_Control_Handle->Motor_Position_Actual) + Motor_MIT_Control_Handle->MIT_Kd * (Motor_MIT_Control_Handle->Motor_Velocity_Target - Motor_MIT_Control_Handle->Motor_Velocity_Actual) + Motor_MIT_Control_Handle->Motor_Torque_Feedforward;

    if (fabsf(Motor_MIT_Control_Handle->Motor_Velocity_Actual) >= 0.001f)
    {
        Motor_MIT_Control_Handle->Output += (Motor_MIT_Control_Handle->Motor_Velocity_Actual) / fabsf(Motor_MIT_Control_Handle->Motor_Velocity_Actual) * Motor_MIT_Control_Handle->Motor_Torque_Friction;
    }
}

static float Calc_V1(float a_max, float j)
{
    return (a_max * a_max) / (2.0f * j);
}

static float Calc_Decel_Dist(float v, float a_max, float j)
{
    return (v * v) / (2.0f * a_max) + (v * a_max) / (2.0f * j);
}

void Speed_Plan_Update(Speed_Plan_Handle_t *Speed_Plan_Handle, float position_actual, float position_target)
{
    uint32_t Current_Time = HAL_GetTick();
    float dt = (Current_Time - Speed_Plan_Handle->Time_Stamp) * 0.001f;
    float v1 = Calc_V1(Speed_Plan_Handle->a_limit, Speed_Plan_Handle->j_limit);

    switch (Speed_Plan_Handle->Speed_Plan_State)
    {
    case idle:
    {
        Speed_Plan_Handle->a = 0;
        Speed_Plan_Handle->v = 0;
        Speed_Plan_Handle->s = 0;
        break;
    }
    case init:
    {
        float old_direction = Speed_Plan_Handle->direction_flag;
        Speed_Plan_Handle->error_s = position_target - position_actual;
        Speed_Plan_Handle->position_initial = position_actual;

        if (Speed_Plan_Handle->error_s >= 0)
        {
            Speed_Plan_Handle->direction_flag = 1.0f;
        }
        else
        {
            Speed_Plan_Handle->direction_flag = -1.0f;
        }

        /* Direction reversal: attenuate inherited velocity to reduce Kd shock */
        if (old_direction * Speed_Plan_Handle->direction_flag < 0.0f)
        {
            Speed_Plan_Handle->v *= 0.3f;
        }

        /* Adaptive v_limit: pre-compute peak speed for this displacement */
        {
            float S = fabsf(Speed_Plan_Handle->error_s);
            float v1 = Calc_V1(Speed_Plan_Handle->a_max, Speed_Plan_Handle->j);
            float b = (Speed_Plan_Handle->a_max * Speed_Plan_Handle->a_max) / Speed_Plan_Handle->j;
            float discriminant = b * b + 4.0f * S * Speed_Plan_Handle->a_max;
            float v_peak = (-b + sqrtf(discriminant)) / 2.0f;

            if (v_peak < 2.0f * v1)
            {
                /* Triangle S-curve: cannot even reach a_max */
                v_peak = powf(0.5f * Speed_Plan_Handle->j * S * S, 1.0f / 3.0f);
            }

            /* Short-distance attenuation: further reduce v_limit for small moves */
            float scale = 1.0f;
            if (S < 0.03f)
            {
                scale = 0.15f;   /* tiny step: 30% */
            }
            else if (S < 0.06f)
            {
                scale = 0.2f;   /* small step: 40% */
            }
            else if (S < 0.15f)
            {
                scale = 0.3f;   /* medium step: 60% */
            }
            v_peak *= scale;

            Speed_Plan_Handle->v_limit = fminf(v_peak, Speed_Plan_Handle->v_max);
            if (Speed_Plan_Handle->v_limit < 0.0f)
            {
                Speed_Plan_Handle->v_limit = 0.0f;
            }

            /* Dynamic j_limit for smoother short-distance motion */
            if (S < 0.03f)
            {
                Speed_Plan_Handle->j_limit = 6.0f;
            }
            else if (S < 0.06f)
            {
                Speed_Plan_Handle->j_limit = 8.0f;
            }
            else if (S < 0.15f)
            {
                Speed_Plan_Handle->j_limit = 10.0f;
            }
            else
            {
                Speed_Plan_Handle->j_limit = Speed_Plan_Handle->j;
            }

            /* Compute effective max accel: ensure v_limit >= 2*v1' */
            float v1_limit = Calc_V1(Speed_Plan_Handle->a_max, Speed_Plan_Handle->j_limit);
            if (Speed_Plan_Handle->v_limit < 2.0f * v1_limit)
            {
                Speed_Plan_Handle->a_limit = sqrtf(Speed_Plan_Handle->j_limit * Speed_Plan_Handle->v_limit);
            }
            else
            {
                Speed_Plan_Handle->a_limit = Speed_Plan_Handle->a_max;
            }
        }

        Speed_Plan_Handle->a = 0;
        Speed_Plan_Handle->s = 0;

        Speed_Plan_Handle->Speed_Plan_State = phase1;
        break;
    }
    case phase1:
    {
        Speed_Plan_Handle->a += Speed_Plan_Handle->j_limit * dt;
        Speed_Plan_Handle->v += Speed_Plan_Handle->a * dt;
        Speed_Plan_Handle->s += Speed_Plan_Handle->v * dt;

        if (Speed_Plan_Handle->a >= Speed_Plan_Handle->a_limit)
        {
            Speed_Plan_Handle->a = Speed_Plan_Handle->a_limit;
            Speed_Plan_Handle->Speed_Plan_State = phase2;
        }
        break;
    }
    case phase2:
    {
        Speed_Plan_Handle->v += Speed_Plan_Handle->a * dt;
        Speed_Plan_Handle->s += Speed_Plan_Handle->v * dt;

        if (Speed_Plan_Handle->v >= Speed_Plan_Handle->v_limit - v1)
        {
            Speed_Plan_Handle->Speed_Plan_State = phase3;
        }
        break;
    }
    case phase3:
    {
        Speed_Plan_Handle->a -= Speed_Plan_Handle->j_limit * dt;
        Speed_Plan_Handle->v += Speed_Plan_Handle->a * dt;
        Speed_Plan_Handle->s += Speed_Plan_Handle->v * dt;

        if (Speed_Plan_Handle->a <= 0)
        {
            Speed_Plan_Handle->a = 0;
            if (Speed_Plan_Handle->v > Speed_Plan_Handle->v_limit)
                Speed_Plan_Handle->v = Speed_Plan_Handle->v_limit;
            Speed_Plan_Handle->Speed_Plan_State = phase3_end;
        }
        break;
    }
    case phase3_end:
    {
        float decel_dist = Calc_Decel_Dist(Speed_Plan_Handle->v, Speed_Plan_Handle->a_limit, Speed_Plan_Handle->j_limit);

        if (Speed_Plan_Handle->s >= fabsf(Speed_Plan_Handle->error_s) - decel_dist)
        {
            Speed_Plan_Handle->Speed_Plan_State = phase5;
        }
        else
        {
            Speed_Plan_Handle->Speed_Plan_State = phase4;
        }
        break;
    }
    case phase4:
    {
        Speed_Plan_Handle->s += Speed_Plan_Handle->v * dt;

        float decel_dist = Calc_Decel_Dist(Speed_Plan_Handle->v, Speed_Plan_Handle->a_limit, Speed_Plan_Handle->j_limit);

        if (Speed_Plan_Handle->s >= fabsf(Speed_Plan_Handle->error_s) - decel_dist)
        {
            Speed_Plan_Handle->Speed_Plan_State = phase5;
        }
        break;
    }
    case phase5:
    {
        Speed_Plan_Handle->a -= Speed_Plan_Handle->j * dt;
        Speed_Plan_Handle->v += Speed_Plan_Handle->a * dt;
        Speed_Plan_Handle->s += Speed_Plan_Handle->v * dt;

        if (Speed_Plan_Handle->a <= -Speed_Plan_Handle->a_limit)
        {
            Speed_Plan_Handle->a = -Speed_Plan_Handle->a_limit;
            Speed_Plan_Handle->Speed_Plan_State = phase6;
        }
        break;
    }
    case phase6:
    {
        Speed_Plan_Handle->v += Speed_Plan_Handle->a * dt;
        Speed_Plan_Handle->s += Speed_Plan_Handle->v * dt;

        if (Speed_Plan_Handle->v <= v1)
        {
            Speed_Plan_Handle->Speed_Plan_State = phase7;
        }
        break;
    }
    case phase7:
    {
        Speed_Plan_Handle->a += Speed_Plan_Handle->j_limit * dt;
        Speed_Plan_Handle->v += Speed_Plan_Handle->a * dt;
        Speed_Plan_Handle->s += Speed_Plan_Handle->v * dt;

        if (Speed_Plan_Handle->a >= 0 || Speed_Plan_Handle->v <= 0)
        {
            Speed_Plan_Handle->a = 0;
            Speed_Plan_Handle->v = 0;
            Speed_Plan_Handle->Speed_Plan_State = idle;
        }
        break;
    }
    }
    if (fabsf(Speed_Plan_Handle->s - fabsf(Speed_Plan_Handle->error_s)) <= 0.003f || Speed_Plan_Handle->v < 0 || Speed_Plan_Handle->s - fabsf(Speed_Plan_Handle->error_s) >= 0)
    {
        Speed_Plan_Handle->v = 0;
        Speed_Plan_Handle->a = 0;
        Speed_Plan_Handle->s = 0;
        Speed_Plan_Handle->Speed_Plan_State = idle;
    }
    Speed_Plan_Handle->Time_Stamp = HAL_GetTick();
}

float Upperarm_Gravity_Compensation(float Upperarm_Motor_Angle, float Forearm_Motor_Angle)
{
    return ((Robotic_Arm_Mass_L1 * Robotic_Arm_Length_L1 * 0.5f + Robotic_Arm_Mass_Forearm_Motor * Robotic_Arm_Length_L1) * g * cosf(Robotic_Arm_Angle_Offset + Upperarm_Motor_Angle) + (Robotic_Arm_Mass_L2 * (Robotic_Arm_Length_L2 * 0.5f) + Robotic_Arm_Mass_End * Robotic_Arm_Length_L2) * g * cosf(Robotic_Arm_Angle_Offset + Upperarm_Motor_Angle + Forearm_Motor_Angle));
}

float Forearm_Gravity_Compensation(float Upperarm_Motor_Angle, float Forearm_Motor_Angle)
{
    return (Robotic_Arm_Mass_L2 * Robotic_Arm_Length_L2 * 0.5f + Robotic_Arm_Mass_End * Robotic_Arm_Length_L2) * g * cosf(Robotic_Arm_Angle_Offset + Upperarm_Motor_Angle + Forearm_Motor_Angle);
}

/*
单位统一为国际单位制
*/
void Coordinate_Inverse_Settlement(float X, float Y, float Z, float Servo_Angle,float *Gimbal_Angle, float *Joint_Upper_Angle, float *Joint_Fore_Angle)
{
    if (fabsf((Robotic_Arm_Length_L1 * Robotic_Arm_Length_L1 + Robotic_Arm_Length_L2 * Robotic_Arm_Length_L2 - X * X - Y * Y - (Z - Robotic_Arm_Length_Connect) * (Z - Robotic_Arm_Length_Connect)) / (2.0f * Robotic_Arm_Length_L1 * Robotic_Arm_Length_L2)) < 1.0f && fabsf(Robotic_Arm_Length_L2 * sinf(acosf((Robotic_Arm_Length_L1 * Robotic_Arm_Length_L1 + Robotic_Arm_Length_L2 * Robotic_Arm_Length_L2 - X * X - Y * Y - (Z - Robotic_Arm_Length_Connect) * (Z - Robotic_Arm_Length_Connect)) / (2.0f * Robotic_Arm_Length_L1 * Robotic_Arm_Length_L2))) / sqrtf(X * X + Y * Y + (Z - Robotic_Arm_Length_Connect) * (Z - Robotic_Arm_Length_Connect))) < 1.0f)
    {
        *Gimbal_Angle = (PI / 2.0f) + atan2f(Y, X) + floorf(LK4005_Motor_Handle[0].Motor_Position_PID_Control_Handle.Motor_Position_Actual / (2.0f * PI)) * 2.0f * PI;
        float L2_Length_Virtual = sqrtf(Robotic_Arm_Length_L2 * Robotic_Arm_Length_L2 + Robotic_Arm_Length_End * Robotic_Arm_Length_End - 2.0f * Robotic_Arm_Length_End * Robotic_Arm_Length_L2 * cosf(Servo_Angle));
        float L2_Angle_Virtual = fabsf(asinf(Robotic_Arm_Length_End * sinf(Servo_Angle) / L2_Length_Virtual));

        float temp = acosf((Robotic_Arm_Length_L1 * Robotic_Arm_Length_L1 + L2_Length_Virtual * L2_Length_Virtual - X * X - Y * Y - (Z - Robotic_Arm_Length_Connect) * (Z - Robotic_Arm_Length_Connect)) / (2.0f * Robotic_Arm_Length_L1 * L2_Length_Virtual));
        *Joint_Upper_Angle = -Normalize_Angle(asinf(L2_Length_Virtual * sinf(temp) / sqrtf(X * X + Y * Y + (Z - Robotic_Arm_Length_Connect) * (Z - Robotic_Arm_Length_Connect))) + atan2f(Z - Robotic_Arm_Length_Connect, sqrtf(X * X + Y * Y + (Z - Robotic_Arm_Length_Connect) * (Z - Robotic_Arm_Length_Connect))) + Angle_Joint_Upper_Offset);
        *Joint_Fore_Angle = Normalize_Angle(temp + Angle_Joint_Fore_Offset - L2_Angle_Virtual);
    }
}
