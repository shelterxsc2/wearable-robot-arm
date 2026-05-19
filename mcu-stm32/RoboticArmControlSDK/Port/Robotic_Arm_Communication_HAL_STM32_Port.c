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
        /* ---------- 滤波与死区参数 ---------- */
        #define POS_DEADZONE_M      0.015f   // 位置死区 1cm (单位: m)
        #define SERVO1_DEADZONE_RAD 0.05f   // 舵机1死区 ≈ 2.9° (单位: rad)
        #define ALPHA               0.65f    // 一阶低通系数

        static float filt_X = 0.0f, filt_Y = 0.0f, filt_Z = 0.0f, filt_Servo1 = 0.0f;
        static float last_X = 0.0f, last_Y = 0.0f, last_Z = 0.0f, last_Servo1 = 0.0f;
        static uint8_t first_rx = 1;
        /* ---------------------------------- */

        float X_Temp = ((float)(int16_t)((Usart_Used0_Rx_Buff[1] << 8) | Usart_Used0_Rx_Buff[0])) / 100.0f;
        float Y_Temp = ((float)(int16_t)((Usart_Used0_Rx_Buff[3] << 8) | Usart_Used0_Rx_Buff[2])) / 100.0f;
        float Z_Temp = ((float)(int16_t)((Usart_Used0_Rx_Buff[5] << 8) | Usart_Used0_Rx_Buff[4])) / 100.0f;
        float Servo1_Temp = (float)(int16_t)((Usart_Used0_Rx_Buff[7] << 8) | Usart_Used0_Rx_Buff[6]) / 180.0f * PI;//与摄像头相连的舵机
        float Servo2_Temp = (float)(int16_t)((Usart_Used0_Rx_Buff[9] << 8) | Usart_Used0_Rx_Buff[8]) / 180.0f * PI;//与小臂相连的舵机

        /* ---------- 一阶低通滤波 ---------- */
        if (first_rx)
        {
            filt_X = X_Temp;
            filt_Y = Y_Temp;
            filt_Z = Z_Temp;
            filt_Servo1 = Servo1_Temp;
        }
        else
        {
            filt_X += ALPHA * (X_Temp - filt_X);
            filt_Y += ALPHA * (Y_Temp - filt_Y);
            filt_Z += ALPHA * (Z_Temp - filt_Z);
            filt_Servo1 += ALPHA * (Servo1_Temp - filt_Servo1);
        }
        /* ---------------------------------- */

        /* ---------- 死区判断 ---------- */
        if (!first_rx)
        {
            if (fabsf(filt_X - last_X) < POS_DEADZONE_M &&
                fabsf(filt_Y - last_Y) < POS_DEADZONE_M &&
                fabsf(filt_Z - last_Z) < POS_DEADZONE_M &&
                fabsf(filt_Servo1 - last_Servo1) < SERVO1_DEADZONE_RAD)
            {
                HAL_UARTEx_ReceiveToIdle_DMA(Communication_Usart_Handle_Used0,
                                             Usart_Used0_Rx_Buff,
                                             Usart_Used0_Rx_Buff_Length);
                return;
            }
        }
        /* ------------------------------ */

        if (PI - filt_Servo1 >= 0.0f && PI - filt_Servo1 <= PI && Servo2_Temp - (PI / 2.0f) + Angle_Servo_Offset >= 0.0f && Servo2_Temp - (PI / 2.0f) + Angle_Servo_Offset <= PI)
        {
            Coordinate_Inverse_Settlement(filt_X, filt_Y, filt_Z, (3.0f * PI / 2.0f) - Servo2_Temp, &LK4005_Motor_Handle[0].Motor_Position_Target, &DMJ4310_Motor_Handle[0].Motor_Position_Target, &LK4005_Motor_Handle[1].Motor_Position_Target);

            DMJ4310_Motor_Handle[0].Motor_Speed_Plan_Handle.Speed_Plan_State = init;
            LK4005_Motor_Handle[1].Motor_Speed_Plan_Handle.Speed_Plan_State = init;
            LK4005_Motor_Handle[0].Motor_Speed_Plan_Handle.Speed_Plan_State = init;

            LFD01M_Motor_Handle[0].Motor_Position = PI - filt_Servo1;
            LFD01M_Motor_Handle[1].Motor_Position = Servo2_Temp - (PI / 2.0f) + Angle_Servo_Offset;

            last_X = filt_X;
            last_Y = filt_Y;
            last_Z = filt_Z;
            last_Servo1 = filt_Servo1;
            first_rx = 0;
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
