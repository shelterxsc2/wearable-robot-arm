#include "LK4005_Motor_Driver.h"
#include "Control_Algorithm.h"

LK4005_Motor_Handle_t LK4005_Motor_Handle[LK4005_Motor_Number] = {0};
int32_t Position_Temp = 0;

void LK4005_Motor_Control_Init(void)
{
    LK4005_Motor_Handle[0].Motor_FDCAN_Handle = &hfdcan1;
    LK4005_Motor_Handle[0].Motor_ID = 0x14C;
    LK4005_Motor_Handle[0].Motor_Type = Gimbal;
    LK4005_Motor_Handle[0].Motor_Speed_Plan_Handle.j = 8.0f;
    LK4005_Motor_Handle[0].Motor_Speed_Plan_Handle.a_max = 3.0f;
    LK4005_Motor_Handle[0].Motor_Speed_Plan_Handle.v_max = 0.75f;

    LK4005_Motor_Handle[1].Motor_FDCAN_Handle = &hfdcan2;
    LK4005_Motor_Handle[1].Motor_ID = 0x149;
    LK4005_Motor_Handle[1].Motor_Type = Joint_Fore;
    LK4005_Motor_Handle[1].Motor_MIT_Control_Handle[0].MIT_Kp = 100.5f;
    LK4005_Motor_Handle[1].Motor_MIT_Control_Handle[1].MIT_Kp = 100.5f;
    LK4005_Motor_Handle[1].Motor_MIT_Control_Handle[0].MIT_Kd = 2.05f;
    LK4005_Motor_Handle[1].Motor_MIT_Control_Handle[1].MIT_Kd = 1.45f;
    LK4005_Motor_Handle[1].Motor_MIT_Control_Handle[0].Motor_Torque_Friction = 0.02f;
    LK4005_Motor_Handle[1].Motor_MIT_Control_Handle[1].Motor_Torque_Friction = 0.02f;
    LK4005_Motor_Handle[1].Motor_MIT_Control_Handle[0].Output = 0.0f;
    LK4005_Motor_Handle[1].Motor_MIT_Control_Handle[1].Output = 0.0f;
    LK4005_Motor_Handle[1].Motor_Speed_Plan_Handle.j = 4.5f;
    LK4005_Motor_Handle[1].Motor_Speed_Plan_Handle.a_max = 1.0f;
    LK4005_Motor_Handle[1].Motor_Speed_Plan_Handle.v_max = 0.45f;

    FDCAN_FilterTypeDef sfilter = {0};
    sfilter.IdType = FDCAN_STANDARD_ID;
    sfilter.FilterIndex = 1;
    sfilter.FilterType = FDCAN_FILTER_RANGE;
    sfilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    sfilter.FilterID1 = 0x140;
    sfilter.FilterID2 = 0x14F;
    HAL_FDCAN_ConfigFilter(LK4005_Motor_Handle[0].Motor_FDCAN_Handle, &sfilter);
    HAL_FDCAN_ConfigFilter(LK4005_Motor_Handle[1].Motor_FDCAN_Handle, &sfilter);

    HAL_FDCAN_ActivateNotification(LK4005_Motor_Handle[0].Motor_FDCAN_Handle, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(LK4005_Motor_Handle[1].Motor_FDCAN_Handle, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
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
    Position_Temp = (int32_t)((LK4005_Motor_Handle.Motor_Position_PID_Control_Handle.Motor_Position_Target + Angle_Gimbal_Offset) * 180.0f / PI * 100.0f *Reduction_Ratio);
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
                    ((int64_t)FDCAN_Rx_Data_Temp[2] << 8)  |
                    ((int64_t)FDCAN_Rx_Data_Temp[1]);

                /* 0.01°/LSB -> 度 -> 弧度 -> 除以减速比得到输出轴角度 */
                float motor_angle_rad = (float)motorAngle_raw * 0.01f * PI / 180.0f;
                LK4005_Motor_Handle->Motor_MIT_Control_Handle[0].Motor_Position_Actual = motor_angle_rad / Reduction_Ratio;

                if (LK4005_Motor_Handle->Motor_Type == Gimbal)
                {
                    LK4005_Motor_Handle->Motor_MIT_Control_Handle[0].Motor_Position_Actual = LK4005_Motor_Handle->Motor_MIT_Control_Handle[0].Motor_Position_Actual - Angle_Gimbal_Offset;
                }
                else if (LK4005_Motor_Handle->Motor_Type == Joint_Fore)
                {
                    LK4005_Motor_Handle->Motor_MIT_Control_Handle[0].Motor_Position_Actual = Normalize_Angle(LK4005_Motor_Handle->Motor_MIT_Control_Handle[0].Motor_Position_Actual);
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
