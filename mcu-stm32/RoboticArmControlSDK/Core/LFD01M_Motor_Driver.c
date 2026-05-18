#include "LFD01M_Motor_Driver.h"

LFD01M_Motor_Handle_t LFD01M_Motor_Handle[LFD01M_Motor_Number] = {0};

void LFD01M_Motor_Control_Init(void)
{
    LFD01M_Motor_Handle[0].Motor_PWM_Timer = &htim1;
    LFD01M_Motor_Handle[0].Motor_PWM_Channel = TIM_CHANNEL_3;
    LFD01M_Motor_Handle[0].Motor_Type = Servo;//上方的舵机与摄像头直接相连
    LFD01M_Motor_Handle[0].Motor_Position = 1.57f;
    LFD01M_Motor_Handle[1].Motor_PWM_Timer = &htim2;
    LFD01M_Motor_Handle[1].Motor_PWM_Channel = TIM_CHANNEL_3;
    LFD01M_Motor_Handle[1].Motor_Type = Servo;//下方的舵机
    LFD01M_Motor_Handle[1].Motor_Position = 1.57f;

    HAL_TIM_PWM_Start(LFD01M_Motor_Handle[0].Motor_PWM_Timer, LFD01M_Motor_Handle[0].Motor_PWM_Channel);
    HAL_TIM_PWM_Start(LFD01M_Motor_Handle[1].Motor_PWM_Timer, LFD01M_Motor_Handle[1].Motor_PWM_Channel);
}

void LFD01M_Motor_Set_Angle(LFD01M_Motor_Handle_t LFD01M_Motor_Handle)
{
    if (LFD01M_Motor_Handle.Motor_Position >= 0.0f && LFD01M_Motor_Handle.Motor_Position <= PI)
    {
        uint16_t Target_CCR_Temp = (uint16_t)(LFD01M_Motor_PWM_CCR_Min + (LFD01M_Motor_Handle.Motor_Position / PI) * (LFD01M_Motor_PWM_CCR_Max - LFD01M_Motor_PWM_CCR_Min));
        __HAL_TIM_SET_COMPARE(LFD01M_Motor_Handle.Motor_PWM_Timer, LFD01M_Motor_Handle.Motor_PWM_Channel, Target_CCR_Temp);
    }
}
