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

void Speed_Plan_Update(Speed_Plan_Handle_t *Speed_Plan_Handle, float position_actual, float position_target)
{
    uint32_t Current_Time = HAL_GetTick();
    float dt = (Current_Time - Speed_Plan_Handle->Time_Stamp) * 0.001f;

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
        Speed_Plan_Handle->error_s = position_target - position_actual;
        Speed_Plan_Handle->position_initial = position_actual;

        if (Speed_Plan_Handle->error_s >= 0.0f)
        {
            Speed_Plan_Handle->direction_flag = 1.0f;
        }
        else
        {
            Speed_Plan_Handle->direction_flag = -1.0f;
        }
        Speed_Plan_Handle->Speed_Plan_State = phase1;
        break;
    }
    case phase1:
    {
        Speed_Plan_Handle->a += Speed_Plan_Handle->j * dt;
        Speed_Plan_Handle->v += Speed_Plan_Handle->a * dt;
        Speed_Plan_Handle->s += Speed_Plan_Handle->v * dt;

        if (Speed_Plan_Handle->a >= Speed_Plan_Handle->a_max)
        {
            Speed_Plan_Handle->a = Speed_Plan_Handle->a_max;
            Speed_Plan_Handle->s1 = Speed_Plan_Handle->s;
            Speed_Plan_Handle->v1 = Speed_Plan_Handle->v;
            Speed_Plan_Handle->Speed_Plan_State = phase2;
        }
        break;
    }
    case phase2:
    {
        Speed_Plan_Handle->v += Speed_Plan_Handle->a * dt;
        Speed_Plan_Handle->s += Speed_Plan_Handle->v * dt;

        if (Speed_Plan_Handle->v >= Speed_Plan_Handle->v_max - Speed_Plan_Handle->v1)
        {
            Speed_Plan_Handle->s2 = Speed_Plan_Handle->s;
            Speed_Plan_Handle->Speed_Plan_State = phase3;
        }
        break;
    }
    case phase3:
    {
        Speed_Plan_Handle->a -= Speed_Plan_Handle->j * dt;
        Speed_Plan_Handle->v += Speed_Plan_Handle->a * dt;
        Speed_Plan_Handle->s += Speed_Plan_Handle->v * dt;

        if (Speed_Plan_Handle->a <= 0)
        {
            Speed_Plan_Handle->a = 0;
            Speed_Plan_Handle->v = Speed_Plan_Handle->v_max;
            Speed_Plan_Handle->s_accel_used = Speed_Plan_Handle->s;
            if (fabsf(Speed_Plan_Handle->error_s) >= powf(Speed_Plan_Handle->v_max, 2.0f) / Speed_Plan_Handle->a_max + Speed_Plan_Handle->a_max * Speed_Plan_Handle->v_max / Speed_Plan_Handle->j)
            {
                Speed_Plan_Handle->Speed_Plan_State = phase4;
            }
            else
            {
                Speed_Plan_Handle->Speed_Plan_State = phase5;
            }
        }
        break;
    }
    case phase4:
    {
        Speed_Plan_Handle->s += Speed_Plan_Handle->v * dt;
        if (Speed_Plan_Handle->s >= (fabsf(Speed_Plan_Handle->error_s) - Speed_Plan_Handle->s_accel_used))
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

        if (Speed_Plan_Handle->s >= (fabsf(Speed_Plan_Handle->error_s) - Speed_Plan_Handle->s2))
        {
            Speed_Plan_Handle->Speed_Plan_State = phase6;
        }
        break;
    }
    case phase6:
    {
        Speed_Plan_Handle->v += Speed_Plan_Handle->a * dt;
        Speed_Plan_Handle->s += Speed_Plan_Handle->v * dt;

        if (Speed_Plan_Handle->s >= (fabsf(Speed_Plan_Handle->error_s) - Speed_Plan_Handle->s1))
        {
            Speed_Plan_Handle->Speed_Plan_State = phase7;
        }
        break;
    }
    case phase7:
    {
        Speed_Plan_Handle->a += Speed_Plan_Handle->j * dt;
        Speed_Plan_Handle->v += Speed_Plan_Handle->a * dt;
        Speed_Plan_Handle->s += Speed_Plan_Handle->v * dt;

        if (Speed_Plan_Handle->v <= 0)
        {
            Speed_Plan_Handle->v = 0;
            Speed_Plan_Handle->Speed_Plan_State = idle;
        }
        break;
    }
    }
    if (fabsf(Speed_Plan_Handle->s - fabsf(Speed_Plan_Handle->error_s)) <= 0.003f || Speed_Plan_Handle->v < 0.0f || Speed_Plan_Handle->s - fabsf(Speed_Plan_Handle->error_s) >= 0.0f)
    {
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
        float L2_Length_Virtual = sqrtf(Robotic_Arm_Length_L2 * Robotic_Arm_Length_L2 + Robotic_Arm_Length_End * Robotic_Arm_Length_End - 2.0f * Robotic_Arm_Length_End * Robotic_Arm_Length_L2 * cosf(PI / 2.0f + Servo_Angle));
        float L2_Angle_Virtual = asinf(Robotic_Arm_Length_End * sinf(PI / 2.0f + Servo_Angle) / L2_Length_Virtual);

        float temp = acosf((Robotic_Arm_Length_L1 * Robotic_Arm_Length_L1 + L2_Length_Virtual * L2_Length_Virtual - X * X - Y * Y - (Z - Robotic_Arm_Length_Connect) * (Z - Robotic_Arm_Length_Connect)) / (2.0f * Robotic_Arm_Length_L1 * L2_Length_Virtual));
        *Joint_Upper_Angle = -Normalize_Angle(asinf(L2_Length_Virtual * sinf(temp) / sqrtf(X * X + Y * Y + (Z - Robotic_Arm_Length_Connect) * (Z - Robotic_Arm_Length_Connect))) + atan2f(Z - Robotic_Arm_Length_Connect, sqrtf(X * X + Y * Y + (Z - Robotic_Arm_Length_Connect) * (Z - Robotic_Arm_Length_Connect))) + Angle_Joint_Upper_Offset);
        *Joint_Fore_Angle = Normalize_Angle(temp + Angle_Joint_Fore_Offset - L2_Angle_Virtual);
    }
}

void Face_Coordinate_Settlement(FaceBased_Data_t FaceBased_Data, float *Gimbal_Angle_Target, float *Joint_Upper_Angle_Target, float *Joint_Fore_Angle_Target,float *Servo_Pitch_Angle_Target,float *Servo_Yaw_Angle_Target)
{
    /*
    具体加减符号根据坐标系确定,还未定下来
    */
    *Servo_Pitch_Angle_Target += FaceBased_Data.Pitch_Target - FaceBased_Data.Pitch_Actual;
    *Servo_Yaw_Angle_Target += FaceBased_Data.Yaw_Target - FaceBased_Data.Yaw_Actual;

}
