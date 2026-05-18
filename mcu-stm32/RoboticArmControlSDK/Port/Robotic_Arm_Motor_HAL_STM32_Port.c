#include "Robotic_Arm_Motor_HAL_STM32_Port.h"
#include "DMJ4310_Motor_Driver.h"
#include "LFD01M_Motor_Driver.h"
#include "LK4005_Motor_Driver.h"
#include "Robotic_Arm_Control_API.h"

void Motor_Control_Init(void)
{
    LFD01M_Motor_Control_Init();
    DMJ4310_Motor_Control_Init();
    LK4005_Motor_Control_Init();
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_Start(&hfdcan2);
}

void FDCAN_Send_Standard(FDCAN_HandleTypeDef *FDCAN_Handle, uint16_t std_id, uint8_t *data, uint8_t length)
{
    FDCAN_TxHeaderTypeDef TxHeader = {0};
    TxHeader.Identifier = std_id;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = (length == 8) ? FDCAN_DLC_BYTES_8 : (length == 7) ? FDCAN_DLC_BYTES_7
                                                          : (length == 6)   ? FDCAN_DLC_BYTES_6
                                                          : (length == 5)   ? FDCAN_DLC_BYTES_5
                                                          : (length == 4)   ? FDCAN_DLC_BYTES_4
                                                          : (length == 3)   ? FDCAN_DLC_BYTES_3
                                                          : (length == 2)   ? FDCAN_DLC_BYTES_2
                                                          : (length == 1)   ? FDCAN_DLC_BYTES_1
                                                                            : FDCAN_DLC_BYTES_0;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_Handle, &TxHeader, data);
}

uint16_t Float_To_Uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if (x < x_min)
        x = x_min;
    else if (x > x_max)
        x = x_max;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float Uint_To_Float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef FDCAN_Rx_Head_Temp;
    uint8_t FDCAN_Rx_Data_Temp[DMJ4310_Motor_FDCAN_Length] = {0};
    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN_Rx_Head_Temp, FDCAN_Rx_Data_Temp);
    uint8_t i = 0;
    for (i = 0; i < DMJ4310_Motor_Number; i++)
    {
        DMJ4310_Motor_Response_Data_Explain(hfdcan, FDCAN_Rx_Head_Temp, FDCAN_Rx_Data_Temp ,&DMJ4310_Motor_Handle[i]);
    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    FDCAN_RxHeaderTypeDef FDCAN_Rx_Head_Temp;
    uint8_t FDCAN_Rx_Data_Temp[LK4005_Motor_FDCAN_Length] = {0};
    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &FDCAN_Rx_Head_Temp, FDCAN_Rx_Data_Temp);
    uint8_t i = 0;
    for (i = 0; i < LK4005_Motor_Number; i++)
    {
        LK4005_Motor_Response_Data_Explain(hfdcan, FDCAN_Rx_Head_Temp, FDCAN_Rx_Data_Temp, &LK4005_Motor_Handle[i]);
    }
}
