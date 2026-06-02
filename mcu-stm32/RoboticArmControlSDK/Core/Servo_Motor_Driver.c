#include "Servo_Motor_Driver.h"

Servo_Motor_Handle_t Servo_Motor_Handle[Servo_Motor_Number] = {0};

void Servo_Motor_Control_Init(void)
{
    /* ---------- FT90M：与摄像头直接相连 ---------- */
    Servo_Motor_Handle[0].Motor_PWM_Timer   = &htim1;
    Servo_Motor_Handle[0].Motor_PWM_Channel = TIM_CHANNEL_3;
    Servo_Motor_Handle[0].Motor_Type        = Servo;
    Servo_Motor_Handle[0].Motor_Position    = 0.9f;
    Servo_Motor_Handle[0].PWM_Pulse_Min     = FT90M_PWM_PULSE_MIN_US;
    Servo_Motor_Handle[0].PWM_Pulse_Max     = FT90M_PWM_PULSE_MAX_US;
    Servo_Motor_Handle[0].Angle_Offset_Rad  = FT90M_ANGLE_OFFSET_RAD;
    Servo_Motor_Handle[0].Angle_Min_Rad     = FT90M_ANGLE_MIN_RAD;
    Servo_Motor_Handle[0].Angle_Max_Rad     = FT90M_ANGLE_MAX_RAD;

    /* ---------- 国华 A009：与小臂相连 ---------- */
    Servo_Motor_Handle[1].Motor_PWM_Timer   = &htim2;
    Servo_Motor_Handle[1].Motor_PWM_Channel = TIM_CHANNEL_3;
    Servo_Motor_Handle[1].Motor_Type        = Servo;
    /* 初始接口角度 0 rad（φ_servo = 0），经偏置后实际 0.6 rad，末端在小臂延长线上 */
    Servo_Motor_Handle[1].Motor_Position    = 0.0f;
    Servo_Motor_Handle[1].PWM_Pulse_Min     = A009_PWM_PULSE_MIN_US;
    Servo_Motor_Handle[1].PWM_Pulse_Max     = A009_PWM_PULSE_MAX_US;
    Servo_Motor_Handle[1].Angle_Offset_Rad  = A009_ANGLE_OFFSET_RAD;
    Servo_Motor_Handle[1].Angle_Min_Rad     = A009_ANGLE_MIN_RAD;
    Servo_Motor_Handle[1].Angle_Max_Rad     = A009_ANGLE_MAX_RAD;

    HAL_TIM_PWM_Start(Servo_Motor_Handle[0].Motor_PWM_Timer, Servo_Motor_Handle[0].Motor_PWM_Channel);
    HAL_TIM_PWM_Start(Servo_Motor_Handle[1].Motor_PWM_Timer, Servo_Motor_Handle[1].Motor_PWM_Channel);

    /* 初始化完成后立即输出一次初始角度，避免停留在 CubeMX 默认 CCR */
    Servo_Motor_Set_Angle(&Servo_Motor_Handle[0]);
    Servo_Motor_Set_Angle(&Servo_Motor_Handle[1]);
}

void Servo_Motor_Set_Angle(Servo_Motor_Handle_t *handle)
{
    if (handle == NULL)
    {
        return;
    }

    float angle_interface = handle->Motor_Position;
    /* 叠加机械安装偏置，得到实际舵机目标角度 */
    float angle_actual = angle_interface + handle->Angle_Offset_Rad;

    /* 限幅到该舵机的实际机械角度范围，防止转超 */
    if (angle_actual < handle->Angle_Min_Rad)
    {
        angle_actual = handle->Angle_Min_Rad;
    }
    if (angle_actual > handle->Angle_Max_Rad)
    {
        angle_actual = handle->Angle_Max_Rad;
    }

    /* 通用线性映射：实际角度 -> 脉宽(us) -> CCR */
    float angle_ratio = (angle_actual - handle->Angle_Min_Rad) / (handle->Angle_Max_Rad - handle->Angle_Min_Rad);
    uint16_t pulse_us = handle->PWM_Pulse_Min +
                        (uint16_t)(angle_ratio * (float)(handle->PWM_Pulse_Max - handle->PWM_Pulse_Min));

    __HAL_TIM_SET_COMPARE(handle->Motor_PWM_Timer, handle->Motor_PWM_Channel, pulse_us);
}
