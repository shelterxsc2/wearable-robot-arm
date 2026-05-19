#include "Robotic_Arm_Control_API.h"

static uint8_t Gimbal_Start_Complete = 0;
static uint8_t Joint_Upper_Start_Complete = 0;
static uint8_t Joint_Fore_Start_Complete = 0;
static uint8_t Gimbal_Flag = 0;

void LFD01M_Motor_Handle_Update(void)
{
    uint8_t i = 0;
    if (Gimbal_Start_Complete)
    {
        for (i = 0; i < LFD01M_Motor_Number; i++)
        {
            if (LFD01M_Motor_Handle[i].Motor_Type == Servo)
            {
                LFD01M_Motor_Set_Angle(LFD01M_Motor_Handle[i]);
            }
            HAL_Delay(LFD01M_Motor_Control_Cycle);
        }
    }
}

void DMJ4310_Motor_Handle_Update(void)
{
    uint8_t i = 0;
    float Motor_Angle_Temp = 0;
    for (i = 0; i < LK4005_Motor_Number; i++)
    {
        if (LK4005_Motor_Handle[i].Motor_Type == Joint_Fore)
        {
            Motor_Angle_Temp = LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[0].Motor_Position_Actual;
            break;
        }
    }
    for (i = 0; i < DMJ4310_Motor_Number; i++)
    {
        if (DMJ4310_Motor_Handle[i].Wait_Count >= 20)
        {
            if (DMJ4310_Motor_Handle[i].Motor_Type == Joint_Upper)
            {
                Speed_Plan_Update(&DMJ4310_Motor_Handle[i].Motor_Speed_Plan_Handle, DMJ4310_Motor_Handle[i].Motor_MIT_Control_Handle.Motor_Position_Actual, DMJ4310_Motor_Handle[i].Motor_Position_Target);
                if (DMJ4310_Motor_Handle[i].Motor_Speed_Plan_Handle.Speed_Plan_State != idle)
                {
                    DMJ4310_Motor_Handle[i].Motor_MIT_Control_Handle.Motor_Position_Target = DMJ4310_Motor_Handle[i].Motor_Speed_Plan_Handle.position_initial + DMJ4310_Motor_Handle[i].Motor_Speed_Plan_Handle.direction_flag * DMJ4310_Motor_Handle[i].Motor_Speed_Plan_Handle.s;

                    DMJ4310_Motor_Handle[i].Motor_MIT_Control_Handle.Motor_Velocity_Target = DMJ4310_Motor_Handle[i].Motor_Speed_Plan_Handle.direction_flag * DMJ4310_Motor_Handle[i].Motor_Speed_Plan_Handle.v;
                }
                else
                {
                    DMJ4310_Motor_Handle[i].Motor_MIT_Control_Handle.Motor_Position_Target = DMJ4310_Motor_Handle[i].Motor_Position_Target;
                    DMJ4310_Motor_Handle[i].Motor_MIT_Control_Handle.Motor_Velocity_Target = 0.0f;
                }

                DMJ4310_Motor_Handle[i].Motor_MIT_Control_Handle.Motor_Torque_Feedforward = -Upperarm_Gravity_Compensation(Normalize_Angle((3.0f * PI / 2.0f) - DMJ4310_Motor_Handle[i].Motor_MIT_Control_Handle.Motor_Position_Actual), Normalize_Angle((3.0f * PI / 2.0f) - Angle_Joint_Fore_Offset + Motor_Angle_Temp));

                DMJ4310_Motor_MIT_Control(DMJ4310_Motor_Handle[i]);

                if (fabsf(DMJ4310_Motor_Handle[i].Motor_MIT_Control_Handle.Motor_Position_Actual - DMJ4310_Motor_Handle[i].Motor_Position_Target) <= 0.1f && Joint_Upper_Start_Complete == 0)
                {
                    Joint_Upper_Start_Complete = 1;
                }
            }
            HAL_Delay(DMJ4310_Motor_Control_Cycle);
        }
        else
        {
            uint8_t FDCAN_Send_Temp_[DMJ4310_Motor_FDCAN_Length] = {0};
            FDCAN_Send_Temp_[0] = 0x7F;
            FDCAN_Send_Temp_[1] = 0xFF;
            FDCAN_Send_Temp_[2] = 0x7F;
            FDCAN_Send_Temp_[3] = 0xF0;
            FDCAN_Send_Temp_[4] = 0x00;
            FDCAN_Send_Temp_[5] = 0x00;
            FDCAN_Send_Temp_[6] = 0x07;
            FDCAN_Send_Temp_[7] = 0xFF;

            FDCAN_Send_Standard(DMJ4310_Motor_Handle[i].Motor_FDCAN_Handle, DMJ4310_Motor_Handle[i].Motor_ID, FDCAN_Send_Temp_, DMJ4310_Motor_FDCAN_Length);
        }
    }
}

void LK4005_Motor_Handle_Update(void)
{
    uint8_t i = 0;
    float Motor_Angle_Temp = 0;
    for (i = 0; i < DMJ4310_Motor_Number; i++)
    {
        if (DMJ4310_Motor_Handle[i].Motor_Type == Joint_Upper)
        {
            Motor_Angle_Temp = DMJ4310_Motor_Handle[i].Motor_MIT_Control_Handle.Motor_Position_Actual;
            break;
        }
    }
    for (i = 0; i < LK4005_Motor_Number; i++)
    {
        LK4005_Motor_Read_Position(LK4005_Motor_Handle[i]);
        HAL_Delay(2); // 微小延迟完成FDCAN报文发送
        LK4005_Motor_Read_Velocity(LK4005_Motor_Handle[i]);
        HAL_Delay(2); // 微小延迟完成FDCAN报文发送
        if (LK4005_Motor_Handle[i].Wait_Count >= 20)
        {
            if (LK4005_Motor_Handle[i].Motor_Type == Joint_Fore)
            {
                Speed_Plan_Update(&LK4005_Motor_Handle[i].Motor_Speed_Plan_Handle, LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[0].Motor_Position_Actual, LK4005_Motor_Handle[i].Motor_Position_Target);

                if (LK4005_Motor_Handle[i].Motor_Speed_Plan_Handle.Speed_Plan_State != idle)
                {
                    LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[0].Motor_Position_Target = LK4005_Motor_Handle[i].Motor_Speed_Plan_Handle.position_initial + LK4005_Motor_Handle[i].Motor_Speed_Plan_Handle.direction_flag * LK4005_Motor_Handle[i].Motor_Speed_Plan_Handle.s;
                    LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[0].Motor_Velocity_Target = LK4005_Motor_Handle[i].Motor_Speed_Plan_Handle.direction_flag * LK4005_Motor_Handle[i].Motor_Speed_Plan_Handle.v;
                    Motor_MIT_Control(&LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[0]);

                    LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[0].Motor_Torque_Feedforward = Forearm_Gravity_Compensation(Normalize_Angle((3.0f * PI / 2.0f) - Motor_Angle_Temp), Normalize_Angle((3.0f * PI / 2.0f) - Angle_Joint_Fore_Offset + LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[0].Motor_Position_Actual));

                    LK4005_Motor_Torque_Control(LK4005_Motor_Handle[i], LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[0]);
                }
                else
                {
                    LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[1].Motor_Position_Target = LK4005_Motor_Handle[i].Motor_Position_Target;
                    LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[1].Motor_Velocity_Target = 0.0f;
                    Motor_MIT_Control(&LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[1]);

                    LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[1].Motor_Torque_Feedforward = Forearm_Gravity_Compensation(Normalize_Angle((3.0f * PI / 2.0f) - Motor_Angle_Temp), Normalize_Angle((3.0f * PI / 2.0f) - Angle_Joint_Fore_Offset + LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[1].Motor_Position_Actual));

                    LK4005_Motor_Torque_Control(LK4005_Motor_Handle[i], LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[1]);
                }

                if (fabsf(LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[1].Motor_Position_Actual - LK4005_Motor_Handle[i].Motor_Position_Target) <= 0.1f && Joint_Fore_Start_Complete == 0 && Joint_Upper_Start_Complete == 2)
                {
                    Joint_Fore_Start_Complete = 1;
                }
            }

            if (LK4005_Motor_Handle[i].Motor_Type == Gimbal)
            {

                Speed_Plan_Update(&LK4005_Motor_Handle[i].Motor_Speed_Plan_Handle, LK4005_Motor_Handle[i].Motor_MIT_Control_Handle[0].Motor_Position_Actual, LK4005_Motor_Handle[i].Motor_Position_Target);

                if (LK4005_Motor_Handle[i].Motor_Speed_Plan_Handle.Speed_Plan_State != idle)
                {
                    LK4005_Motor_Handle[i].Motor_Position_PID_Control_Handle.Motor_Position_Target = LK4005_Motor_Handle[i].Motor_Speed_Plan_Handle.position_initial + LK4005_Motor_Handle[i].Motor_Speed_Plan_Handle.direction_flag * LK4005_Motor_Handle[i].Motor_Speed_Plan_Handle.s;

                }
                else
                {
                    LK4005_Motor_Handle[i].Motor_Position_PID_Control_Handle.Motor_Position_Target = LK4005_Motor_Handle[i].Motor_Position_Target;
                }
                LK4005_Motor_Position_Control(LK4005_Motor_Handle[i]);
                if (fabsf(LK4005_Motor_Handle[i].Motor_Position_PID_Control_Handle.Motor_Position_Actual - LK4005_Motor_Handle[i].Motor_Position_Target) <= 0.1f && Gimbal_Start_Complete == 0 && Joint_Fore_Start_Complete == 2)
                {
                    Gimbal_Start_Complete = 1;
                }
            }

            HAL_Delay(LK4005_Motor_Control_Cycle);
        }
    }
}

void Robotic_Arm_Control_Init(void)
{
    Motor_Control_Init();
    HAL_Delay(1500);
    uint8_t i = 0;
    for (i = 0; i < DMJ4310_Motor_Number; i++)
    {
        DMJ4310_Motor_Enable(DMJ4310_Motor_Handle[i]);
    }
    Communication_Usart_Init();
}

void Robotic_Arm_Control(void)
{
    if (Joint_Upper_Start_Complete == 0)
    {
        if(LK4005_Motor_Handle[1].Wait_Count == 19)
        {
            LK4005_Motor_Handle[1].Motor_Position_Target = LK4005_Motor_Handle[1].Motor_MIT_Control_Handle[1].Motor_Position_Actual;
        }
    }
    else if (Joint_Upper_Start_Complete == 1)
    {
        LK4005_Motor_Handle[1].Motor_Position_Target = 1.67f;
        LK4005_Motor_Handle[1].Motor_Speed_Plan_Handle.Speed_Plan_State = init;
        Joint_Upper_Start_Complete = 2;
    }
    if(Joint_Fore_Start_Complete == 0)
    {
        if (LK4005_Motor_Handle[0].Wait_Count >= 15 && LK4005_Motor_Handle[0].Wait_Count <= 19 && Gimbal_Flag == 0)
        {
            LK4005_Motor_Handle[0].Motor_Position_Target = LK4005_Motor_Handle[0].Motor_Position_PID_Control_Handle.Motor_Position_Actual;
            Gimbal_Flag = 1;
        }
    }
    else if (Joint_Fore_Start_Complete == 1)
    {
        LK4005_Motor_Handle[0].Motor_Position_Target = 3.14f + floorf(LK4005_Motor_Handle[0].Motor_Position_PID_Control_Handle.Motor_Position_Actual / (2.0f * PI)) * 2.0f * PI;
        LK4005_Motor_Handle[0].Motor_Speed_Plan_Handle.Speed_Plan_State = init;
        Joint_Fore_Start_Complete = 2;
    }
    LFD01M_Motor_Handle_Update();
    DMJ4310_Motor_Handle_Update();
    LK4005_Motor_Handle_Update();
}
