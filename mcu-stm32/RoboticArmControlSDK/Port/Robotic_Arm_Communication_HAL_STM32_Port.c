#include "Robotic_Arm_Communication_HAL_STM32_Port.h"
#include "string.h"
#include "stdio.h"
#include "Servo_Motor_Driver.h"
#include "LK4005_Motor_Driver.h"
#include "Control_Algorithm.h"
#include "Robotic_Arm_Control_API.h"

uint8_t Usart_Used0_Rx_Buff[Usart_Used0_Rx_Buff_Length] = {0};

void Communication_Usart_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(Communication_Usart_Handle_Used0, Usart_Used0_Rx_Buff, Usart_Used0_Rx_Buff_Length);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == Communication_Usart_Instance_Used0)
    {
        if (Test_Mode_Active)
        {
            HAL_UARTEx_ReceiveToIdle_DMA(Communication_Usart_Handle_Used0,
                                         Usart_Used0_Rx_Buff,
                                         Usart_Used0_Rx_Buff_Length);
            return;
        }
        
        #define POS_DEADZONE_M      0.015f   // 位置死区 1cm (单位: m)
        #define SERVO1_DEADZONE_RAD 0.05f   // 舵机1死区 ≈ 2.9° (单位: rad)
        #define ALPHA               0.65f    // 一阶低通系数

        static float filt_X = 0.0f, filt_Y = 0.0f, filt_Z = 0.0f, filt_Servo1 = 0.0f;
        static float last_X = 0.0f, last_Y = 0.0f, last_Z = 0.0f, last_Servo1 = 0.0f;
        static uint8_t first_rx = 1;

        float X_Temp = ((float)(int16_t)((Usart_Used0_Rx_Buff[1] << 8) | Usart_Used0_Rx_Buff[0])) / 100.0f;
        float Y_Temp = ((float)(int16_t)((Usart_Used0_Rx_Buff[3] << 8) | Usart_Used0_Rx_Buff[2])) / 100.0f;
        float Z_Temp = ((float)(int16_t)((Usart_Used0_Rx_Buff[5] << 8) | Usart_Used0_Rx_Buff[4])) / 100.0f;
        float Servo1_Temp = (float)(int16_t)((Usart_Used0_Rx_Buff[7] << 8) | Usart_Used0_Rx_Buff[6]) / 180.0f * PI;//与摄像头相连的舵机(FT90M)
        float Servo2_Temp = (float)(int16_t)((Usart_Used0_Rx_Buff[9] << 8) | Usart_Used0_Rx_Buff[8]) / 180.0f * PI;//与小臂相连的舵机(A009)
        /* 不使用滤波和死区，直接赋值 */
        filt_X = X_Temp;
        filt_Y = Y_Temp;
        filt_Z = Z_Temp;
        filt_Servo1 = Servo1_Temp;

        // A009: 上位机0°对应共线(φ_servo=0)，角度值直接等于phi_servo
        float phi_servo = Servo2_Temp;
        // FT90M接口范围检查; A009接口范围: φ_servo ∈ [-0.6, π-0.6] 由驱动层限幅保护
        if (filt_Servo1 >= 0.0f && filt_Servo1 <= FT90M_ANGLE_MAX_RAD - FT90M_ANGLE_OFFSET_RAD)
        {
            float gimbal_joint, upper_joint, fore_joint;
            Coordinate_Inverse_Settlement(filt_X, filt_Y, filt_Z, phi_servo,
                                          &gimbal_joint, &upper_joint, &fore_joint);

            LK4005_Motor_Handle[0].Motor_Position_Target = gimbal_joint;            //云台
            LK4005_Motor_Handle[1].Motor_Position_Target = -4.0f * upper_joint;     //大臂：电机轴 = -4×关节角
            LK4005_Motor_Handle[2].Motor_Position_Target =  2.0f * fore_joint;      //小臂：电机轴 =  2×关节角

            LK4005_Motor_Handle[1].Motor_Speed_Plan_Handle.Speed_Plan_State = init;
            LK4005_Motor_Handle[2].Motor_Speed_Plan_Handle.Speed_Plan_State = init;
            LK4005_Motor_Handle[0].Motor_Speed_Plan_Handle.Speed_Plan_State = init;

            if (Servo_Control_Active)
            {
                // FT90M: 直接映射，不再取反
                Servo_Motor_Handle[0].Motor_Position = filt_Servo1;
                // A009: 接口角度直接等于 φ_servo（上位机0°=共线）
                Servo_Motor_Handle[1].Motor_Position = phi_servo;
            }

            last_X = filt_X;
            last_Y = filt_Y;
            last_Z = filt_Z;
            last_Servo1 = filt_Servo1;
            first_rx = 0;
        }
        /*
        Usart_Used0_Rx_Buff[Size] = '\0';
        if (strncmp((char *)Usart_Used0_Rx_Buff, "Motor", 5) == 0)
        {
            char tittle[8] = {0};
            int subtittle = 0;
            float temp1 = 0.0f;

            sscanf((char *)Usart_Used0_Rx_Buff, "%s %d %f", tittle, &subtittle, &temp1);

            if (subtittle == 0)
            {
                LK4005_Motor_Handle[1].Motor_Position_Target = temp1;
                LK4005_Motor_Handle[1].Motor_Speed_Plan_Handle.Speed_Plan_State = init;
            }
            else if (subtittle == 1)
            {
                LK4005_Motor_Handle[2].Motor_Position_Target = temp1;
                LK4005_Motor_Handle[2].Motor_Speed_Plan_Handle.Speed_Plan_State = init;
            }
        }
        else if (strncmp((char *)Usart_Used0_Rx_Buff, "Servo", 5) == 0)
        {
            char tittle[8] = {0};
            int subtittle = 0;
            float temp1 = 0;

            sscanf((char *)Usart_Used0_Rx_Buff, "%s %d %f", tittle, &subtittle, &temp1);

            if (subtittle == 0)
            {
                if (Servo_Control_Active)
                    Servo_Motor_Handle[0].Motor_Position = temp1;
            }
            else if (subtittle == 1)
            {
                if (Servo_Control_Active)
                    Servo_Motor_Handle[1].Motor_Position = temp1;
            }
        }
        */
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
