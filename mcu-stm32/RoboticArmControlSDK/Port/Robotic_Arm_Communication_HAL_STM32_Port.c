#include "Robotic_Arm_Communication_HAL_STM32_Port.h"
#include "string.h"
#include "stdio.h"
#include "LFD01M_Motor_Driver.h"
#include "LK4005_Motor_Driver.h"
#include "DMJ4310_Motor_Driver.h"
#include "Control_Algorithm.h"

uint8_t Usart_Used0_Rx_Buff[Usart_Used0_Rx_Buff_Length] = {0};

void Communication_Usart_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(Communication_Usart_Handle_Used0, Usart_Used0_Rx_Buff, Usart_Used0_Rx_Buff_Length);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == Communication_Usart_Instance_Used0)
    {
        float X_Temp = ((float)(int16_t)((Usart_Used0_Rx_Buff[1] << 8) | Usart_Used0_Rx_Buff[0])) / 100.0f;
        float Y_Temp = ((float)(int16_t)((Usart_Used0_Rx_Buff[3] << 8) | Usart_Used0_Rx_Buff[2])) / 100.0f;
        float Z_Temp = ((float)(int16_t)((Usart_Used0_Rx_Buff[5] << 8) | Usart_Used0_Rx_Buff[4])) / 100.0f;
        float Servo1_Temp = (float)(int16_t)((Usart_Used0_Rx_Buff[7] << 8) | Usart_Used0_Rx_Buff[6]) / 180.0f * PI;//与摄像头相连的舵机
        float Servo2_Temp = (float)(int16_t)((Usart_Used0_Rx_Buff[9] << 8) | Usart_Used0_Rx_Buff[8]) / 180.0f * PI;//与小臂相连的舵机

        X_Temp -= cosf(PI / 2.0f - Servo2_Temp) * Robotic_Arm_Length_End * cosf(atan2f(Y_Temp, X_Temp));
        Y_Temp -= cosf(PI / 2.0f - Servo2_Temp) * Robotic_Arm_Length_End * sinf(atan2f(Y_Temp, X_Temp));

        if (PI - Servo1_Temp >= 0.0f && PI - Servo1_Temp <= PI &&Servo2_Temp - (PI / 2.0f) + Angle_Servo_Offset >= 0.0f && Servo2_Temp - (PI / 2.0f) + Angle_Servo_Offset <= PI && DMJ4310_Motor_Handle[0].Motor_Speed_Plan_Handle.Speed_Plan_State == idle && LK4005_Motor_Handle[1].Motor_Speed_Plan_Handle.Speed_Plan_State == idle && LK4005_Motor_Handle[0].Motor_Speed_Plan_Handle.Speed_Plan_State == idle)
        {
            Coordinate_Inverse_Settlement(X_Temp, Y_Temp, Z_Temp, Servo2_Temp, &LK4005_Motor_Handle[0].Motor_Position_Target, &DMJ4310_Motor_Handle[0].Motor_Position_Target, &LK4005_Motor_Handle[1].Motor_Position_Target);

            DMJ4310_Motor_Handle[0].Motor_Speed_Plan_Handle.Speed_Plan_State = init;
            LK4005_Motor_Handle[1].Motor_Speed_Plan_Handle.Speed_Plan_State = init;
            LK4005_Motor_Handle[0].Motor_Speed_Plan_Handle.Speed_Plan_State = init;

            LFD01M_Motor_Handle[0].Motor_Position = PI - Servo1_Temp;
            LFD01M_Motor_Handle[1].Motor_Position = Servo2_Temp - (PI / 2.0f) + Angle_Servo_Offset;
        }
        /*Usart_Used0_Rx_Buff[Size] = '\0';
        if (strncmp((char *)Usart_Used0_Rx_Buff, "LFD", 3) == 0)
        {
            char tittle[8] = {0};
            int subtittle = 0;
            float temp1 = 0;

            sscanf((char *)Usart_Used0_Rx_Buff, "%s %d %f", tittle, &subtittle, &temp1);

            if (subtittle == 0)
            {
                LFD01M_Motor_Handle[0].Motor_Position = temp1;
            }
            else if (subtittle == 1)
            {
                LFD01M_Motor_Handle[1].Motor_Position = temp1;
            }
        }*/
        HAL_UARTEx_ReceiveToIdle_DMA(Communication_Usart_Handle_Used0, Usart_Used0_Rx_Buff, Usart_Used0_Rx_Buff_Length);
    }
}

void Communication_Test(void)
{
    uint8_t package[16] = {0};

    memcpy(&package[0], &LK4005_Motor_Handle[0].Motor_Position_Target, 4);
    memcpy(&package[4], &LK4005_Motor_Handle[0].Motor_Position_PID_Control_Handle.Motor_Position_Actual, 4);
    package[12] = 0x00;
    package[13] = 0x00;
    package[14] = 0x80;
    package[15] = 0x7f;
    HAL_UART_Transmit_DMA(&huart1, package, 16);
    HAL_Delay(3);
}
