#include "Robotic_Arm_Communication_HAL_STM32_Port.h"
#include "string.h"
#include "stdio.h"
#include "math.h"
#include "Servo_Motor_Driver.h"
#include "LK4005_Motor_Driver.h"
#include "Control_Algorithm.h"
#include "Robotic_Arm_Control_API.h"

uint8_t Usart_Used0_Rx_Buff[Usart_Used0_Rx_Buff_Length] = {0};
uint8_t Feedback_Pending = 0;

extern uint8_t Retract_Sequence_Trigger;

void Communication_Usart_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(Communication_Usart_Handle_Used0, Usart_Used0_Rx_Buff, Usart_Used0_Rx_Buff_Length);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == Communication_Usart_Instance_Used0)
    {
        /* 处理 10 字节特殊指令 或 11 字节坐标指令 */
        if (Size != 10 && Size != 11)
        {
            HAL_UARTEx_ReceiveToIdle_DMA(Communication_Usart_Handle_Used0,
                                         Usart_Used0_Rx_Buff,
                                         Usart_Used0_Rx_Buff_Length);
            return;
        }
        
        /* Size == 10: 检查特殊指令 (初始化 / 收回) */
        if (Size == 10)
        {
            /* 检查是否是初始化指令: FF AA FF AA FF AA FF AA FF AA */
            uint8_t is_init_cmd = 1;
            for (int i = 0; i < 10; i++)
            {
                if (Usart_Used0_Rx_Buff[i] != ((i % 2 == 0) ? 0xFF : 0xAA))
                {
                    is_init_cmd = 0;
                    break;
                }
            }
            
            if (is_init_cmd)
            {
                /* 清除之前可能未完成的普通指令反馈 */
                Feedback_Pending = 0;
                
                /* 大臂和小臂同时启动，云台先锁定当前位置 */
                LK4005_Motor_Handle[1].Motor_Position_Target = -7.8f;
                LK4005_Motor_Handle[1].Motor_Speed_Plan_Handle.Speed_Plan_State = init;
                
                LK4005_Motor_Handle[2].Motor_Position_Target = 3.5f;
                LK4005_Motor_Handle[2].Motor_Speed_Plan_Handle.Speed_Plan_State = init;
                
                LK4005_Motor_Handle[0].Motor_Position_Target = LK4005_Motor_Handle[0].Motor_Position_PID_Control_Handle.Motor_Position_Actual;
                LK4005_Motor_Handle[0].Motor_Speed_Plan_Handle.Speed_Plan_State = init;
                
                /* 触发顺序启动状态机 */
                Init_Sequence_Trigger = 1;
                
                /* 收到 FF AA 后，停止上电初始化阶段的 init success 持续发送 */
                extern uint8_t PowerOn_Init_Sending;
                PowerOn_Init_Sending = 0;
                
                HAL_UARTEx_ReceiveToIdle_DMA(Communication_Usart_Handle_Used0,
                                             Usart_Used0_Rx_Buff,
                                             Usart_Used0_Rx_Buff_Length);
                return;
            }
            
            /* 检查是否是收回指令: AA FF AA FF AA FF AA FF AA FF */
            uint8_t is_retract_cmd = 1;
            for (int i = 0; i < 10; i++)
            {
                if (Usart_Used0_Rx_Buff[i] != ((i % 2 == 0) ? 0xAA : 0xFF))
                {
                    is_retract_cmd = 0;
                    break;
                }
            }
            
            if (is_retract_cmd)
            {
                /* 清除之前可能未完成的普通指令反馈 */
                Feedback_Pending = 0;
                
                /* 触发收回状态机：云台 → 小臂 → 大臂 */
                Retract_Sequence_Trigger = 1;
                
                HAL_UARTEx_ReceiveToIdle_DMA(Communication_Usart_Handle_Used0,
                                             Usart_Used0_Rx_Buff,
                                             Usart_Used0_Rx_Buff_Length);
                return;
            }
            
            /* 10 字节且不是特殊指令，丢弃 */
            HAL_UARTEx_ReceiveToIdle_DMA(Communication_Usart_Handle_Used0,
                                         Usart_Used0_Rx_Buff,
                                         Usart_Used0_Rx_Buff_Length);
            return;
        }
        
        /* ========== 普通坐标指令 (11字节) ========== */
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
        filt_Y = Y_Temp - 0.1f;
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
            LK4005_Motor_Handle[1].Motor_Position_Target = -4.0f * (upper_joint + 0.17f);     //大臂：电机轴 = -4×关节角
            LK4005_Motor_Handle[2].Motor_Position_Target =  2.0f * (fore_joint + 0.17f);      //小臂：电机轴 =  2×关节角

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
            
            Feedback_Pending = 2; /* 标记待发送 move_success */
        }
        
        HAL_UARTEx_ReceiveToIdle_DMA(Communication_Usart_Handle_Used0, Usart_Used0_Rx_Buff, Usart_Used0_Rx_Buff_Length);
    }
}

void Communication_Send_Init_Success(void)
{
    static uint8_t msg[] = "init success\r\n";
    HAL_UART_StateTypeDef state = HAL_UART_GetState(Communication_Usart_Handle_Used0);
    if ((state != HAL_UART_STATE_BUSY_TX) && (state != HAL_UART_STATE_BUSY_TX_RX))
    {
        HAL_UART_Transmit_DMA(Communication_Usart_Handle_Used0, msg, sizeof(msg) - 1);
    }
}

void Communication_Send_Move_Success(void)
{
    static char tx_buf[128];
    
    float upper_motor_angle = LK4005_Motor_Handle[1].Motor_MIT_Control_Handle[0].Motor_Position_Actual;
    float fore_motor_angle  = LK4005_Motor_Handle[2].Motor_MIT_Control_Handle[0].Motor_Position_Actual;
    float theta1 = -upper_motor_angle / 4.0f;
    float theta2 =  fore_motor_angle / 2.0f;
    float gimbal = LK4005_Motor_Handle[0].Motor_Position_PID_Control_Handle.Motor_Position_Actual;
    
    float vx = -sinf(theta1 + theta2) * sinf(gimbal);
    float vy = -sinf(theta1 + theta2) * cosf(gimbal);
    float vz =  cosf(theta1 + theta2);
    
    int len = snprintf(tx_buf, sizeof(tx_buf), "move_success %.4f %.4f %.4f %.4f\r\n", vx, vy, vz, gimbal);
    if (len > 0 && len < (int)sizeof(tx_buf))
    {
        HAL_UART_Transmit_DMA(Communication_Usart_Handle_Used0, (uint8_t *)tx_buf, len);
    }
}
