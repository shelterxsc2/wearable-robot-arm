#include "LK4005_Motor_Driver.h"
#include "Control_Algorithm.h"

LK4005_Motor_Handle_t LK4005_Motor_Handle[LK4005_Motor_Number] = {0};
int32_t Position_Temp = 0;

void LK4005_Motor_Control_Init(void)
{
    /* [0] 云台 (Gimbal) */
    LK4005_Motor_Handle[0].Motor_FDCAN_Handle = &hfdcan1;
    LK4005_Motor_Handle[0].Motor_ID = 0x145;
    LK4005_Motor_Handle[0].Motor_Type = Gimbal;
    LK4005_Motor_Handle[0].Motor_Position_PID_Control_Handle.Motor_Position_Target = 0.0f;
    LK4005_Motor_Handle[0].Motor_Speed_Plan_Handle.j = 10.5f;
    LK4005_Motor_Handle[0].Motor_Speed_Plan_Handle.a_max = 3.5f;
    LK4005_Motor_Handle[0].Motor_Speed_Plan_Handle.v_max = 1.85f;

    /* [1] 大臂 (Joint_Upper) — 原DMJ4310位置，新LK电机 */
    LK4005_Motor_Handle[1].Motor_FDCAN_Handle = &hfdcan1;
    LK4005_Motor_Handle[1].Motor_ID = 0x143;
    LK4005_Motor_Handle[1].Motor_Type = Joint_Upper;
    LK4005_Motor_Handle[1].Motor_MIT_Control_Handle[0].MIT_Kp = 22.0f;
    LK4005_Motor_Handle[1].Motor_MIT_Control_Handle[1].MIT_Kp = 22.0f;
    LK4005_Motor_Handle[1].Motor_MIT_Control_Handle[0].MIT_Kd = 0.862f;
    LK4005_Motor_Handle[1].Motor_MIT_Control_Handle[1].MIT_Kd = 0.862f;
    LK4005_Motor_Handle[1].Motor_MIT_Control_Handle[1].Motor_Torque_Friction = 0.1f;
    LK4005_Motor_Handle[1].Motor_MIT_Control_Handle[0].Output = 0.0f;
    LK4005_Motor_Handle[1].Motor_MIT_Control_Handle[1].Output = 0.0f;
    LK4005_Motor_Handle[1].Motor_Speed_Plan_Handle.j = 32.5f;
    LK4005_Motor_Handle[1].Motor_Speed_Plan_Handle.a_max = 5.3f;
    LK4005_Motor_Handle[1].Motor_Speed_Plan_Handle.v_max = 3.0f;

    /* [2] 小臂 (Joint_Fore) */
    LK4005_Motor_Handle[2].Motor_FDCAN_Handle = &hfdcan1;
    LK4005_Motor_Handle[2].Motor_ID = 0x142;
    LK4005_Motor_Handle[2].Motor_Type = Joint_Fore;
    LK4005_Motor_Handle[2].Motor_MIT_Control_Handle[0].MIT_Kp = 19.0f;
    LK4005_Motor_Handle[2].Motor_MIT_Control_Handle[1].MIT_Kp = 19.0f;
    LK4005_Motor_Handle[2].Motor_MIT_Control_Handle[0].MIT_Kd = 0.872f;
    LK4005_Motor_Handle[2].Motor_MIT_Control_Handle[1].MIT_Kd = 0.872f;
    LK4005_Motor_Handle[2].Motor_MIT_Control_Handle[1].Motor_Torque_Friction = 0.0f;
    LK4005_Motor_Handle[2].Motor_MIT_Control_Handle[0].Output = 0.0f;
    LK4005_Motor_Handle[2].Motor_MIT_Control_Handle[1].Output = 0.0f;
    LK4005_Motor_Handle[2].Motor_Speed_Plan_Handle.j = 30.5f;
    LK4005_Motor_Handle[2].Motor_Speed_Plan_Handle.a_max = 4.9f;
    LK4005_Motor_Handle[2].Motor_Speed_Plan_Handle.v_max = 2.5f;

    /* FDCAN1 滤波器0: 覆盖所有三个电机ID范围 (0x140-0x14F) */
    FDCAN_FilterTypeDef sfilter0 = {0};
    sfilter0.IdType = FDCAN_STANDARD_ID;
    sfilter0.FilterIndex = 0;
    sfilter0.FilterType = FDCAN_FILTER_RANGE;
    sfilter0.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    sfilter0.FilterID1 = 0x140;
    sfilter0.FilterID2 = 0x14F;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &sfilter0);

    /* 只激活FDCAN1的FIFO1中断 */
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
}

void LK4005_Motor_Torque_Control(LK4005_Motor_Handle_t LK4005_Motor_Handle, Motor_MIT_Control_Handle_t Motor_MIT_Control_Handle)
{
    if (Motor_MIT_Control_Handle.Output <= -33.0f)
    {
        Motor_MIT_Control_Handle.Output = -33.0f;
    }
    else if (Motor_MIT_Control_Handle.Output >= 33.0f)
    {
        Motor_MIT_Control_Handle.Output = 33.0f;
    }
    uint8_t FDCAN_Send_Temp[LK4005_Motor_FDCAN_Length] = {0};
    int16_t Torque_Temp = (int16_t)(Motor_MIT_Control_Handle.Output * Torque_Conversion_Ratio);
    FDCAN_Send_Temp[0] = 0xA1;
    FDCAN_Send_Temp[4] = (uint8_t)(Torque_Temp & 0xFF);
    FDCAN_Send_Temp[5] = (uint8_t)((Torque_Temp >> 8) & 0xFF);
    FDCAN_Send_Standard(LK4005_Motor_Handle.Motor_FDCAN_Handle, LK4005_Motor_Handle.Motor_ID, FDCAN_Send_Temp, LK4005_Motor_FDCAN_Length);
}

void LK4005_Motor_Position_Control(LK4005_Motor_Handle_t LK4005_Motor_Handle)
{
    uint8_t FDCAN_Send_Temp[LK4005_Motor_FDCAN_Length] = {0};
    Position_Temp = (int32_t)(LK4005_Motor_Handle.Motor_Position_PID_Control_Handle.Motor_Position_Target * 180.0f / PI * 100.0f * Reduction_Ratio);
    FDCAN_Send_Temp[0] = 0xA3;
    FDCAN_Send_Temp[4] = (uint8_t)(Position_Temp & 0xFF);
    FDCAN_Send_Temp[5] = (uint8_t)((Position_Temp >> 8 * 1) & 0xFF);
    FDCAN_Send_Temp[6] = (uint8_t)((Position_Temp >> 8 * 2) & 0xFF);
    FDCAN_Send_Temp[7] = (uint8_t)((Position_Temp >> 8 * 3) & 0xFF);
    FDCAN_Send_Standard(LK4005_Motor_Handle.Motor_FDCAN_Handle, LK4005_Motor_Handle.Motor_ID, FDCAN_Send_Temp, LK4005_Motor_FDCAN_Length);
}

void LK4005_Motor_Read_Position(LK4005_Motor_Handle_t LK4005_Motor_Handle)
{
    uint8_t FDCAN_Send_Temp[LK4005_Motor_FDCAN_Length] = {0};
    FDCAN_Send_Temp[0] = 0x92;
    FDCAN_Send_Standard(LK4005_Motor_Handle.Motor_FDCAN_Handle, LK4005_Motor_Handle.Motor_ID, FDCAN_Send_Temp, LK4005_Motor_FDCAN_Length);
}

void LK4005_Motor_Read_Velocity(LK4005_Motor_Handle_t LK4005_Motor_Handle)
{
    uint8_t FDCAN_Send_Temp[LK4005_Motor_FDCAN_Length] = {0};
    FDCAN_Send_Temp[0] = 0x9C;
    FDCAN_Send_Standard(LK4005_Motor_Handle.Motor_FDCAN_Handle, LK4005_Motor_Handle.Motor_ID, FDCAN_Send_Temp, LK4005_Motor_FDCAN_Length);
}

void LK4005_Motor_Response_Data_Explain(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxHeaderTypeDef FDCAN_Rx_Head_Temp, uint8_t *FDCAN_Rx_Data_Temp, LK4005_Motor_Handle_t *LK4005_Motor_Handle)
{
    if (hfdcan == LK4005_Motor_Handle->Motor_FDCAN_Handle)
    {
        if (FDCAN_Rx_Head_Temp.Identifier == LK4005_Motor_Handle->Motor_ID && FDCAN_Rx_Head_Temp.DataLength == LK4005_Motor_FDCAN_Length)
        {
            if (FDCAN_Rx_Data_Temp[0] == 0x92)
            {
                /* 0x92 读取多圈角度：DATA[1]~DATA[7] 为 motorAngle(int64_t) 的 7 个字节，单位 0.01°/LSB */
                int64_t motorAngle_raw =
                    ((int64_t)(int8_t)FDCAN_Rx_Data_Temp[7] << 48) |
                    ((int64_t)FDCAN_Rx_Data_Temp[6] << 40) |
                    ((int64_t)FDCAN_Rx_Data_Temp[5] << 32) |
                    ((int64_t)FDCAN_Rx_Data_Temp[4] << 24) |
                    ((int64_t)FDCAN_Rx_Data_Temp[3] << 16) |
                    ((int64_t)FDCAN_Rx_Data_Temp[2] << 8) |
                    ((int64_t)FDCAN_Rx_Data_Temp[1]);

                /* 0.01°/LSB -> 度 -> 弧度 -> 除以减速比得到输出轴角度 */
                float motor_angle_rad = (float)motorAngle_raw * 0.01f * PI / 180.0f;
                LK4005_Motor_Handle->Motor_MIT_Control_Handle[0].Motor_Position_Actual = motor_angle_rad / Reduction_Ratio;

                if (LK4005_Motor_Handle->Motor_Type == Joint_Upper)
                {
                    LK4005_Motor_Handle->Motor_MIT_Control_Handle[0].Motor_Position_Actual += Angle_Joint_Upper_Offset;
                }
                else if (LK4005_Motor_Handle->Motor_Type == Joint_Fore)
                {
                    LK4005_Motor_Handle->Motor_MIT_Control_Handle[0].Motor_Position_Actual += Angle_Joint_Fore_Offset;
                }

                LK4005_Motor_Handle->Motor_Position_PID_Control_Handle.Motor_Position_Actual = LK4005_Motor_Handle->Motor_MIT_Control_Handle[0].Motor_Position_Actual;

                LK4005_Motor_Handle->Motor_MIT_Control_Handle[1].Motor_Position_Actual = LK4005_Motor_Handle->Motor_MIT_Control_Handle[0].Motor_Position_Actual;
            }

            else if (FDCAN_Rx_Data_Temp[0] == 0x9C)
            {
                LK4005_Motor_Handle->Motor_MIT_Control_Handle[0].Motor_Velocity_Actual = (float)((int16_t)(FDCAN_Rx_Data_Temp[5] << 8 | FDCAN_Rx_Data_Temp[4])) * PI / (180.0f * Reduction_Ratio);

                LK4005_Motor_Handle->Motor_MIT_Control_Handle[1].Motor_Velocity_Actual = LK4005_Motor_Handle->Motor_MIT_Control_Handle[0].Motor_Velocity_Actual;
            }
            LK4005_Motor_Handle->Wait_Count++;
        }
    }
}
