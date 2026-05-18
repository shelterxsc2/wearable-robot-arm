#ifndef __DMJ4310_MOTOR_DRIVER_H
#define __DMJ4310_MOTOR_DRIVER_H

#include "main.h"
#include "Robotic_Arm_Config.h"
#include "Robotic_Arm_Motor_HAL_STM32_Port.h"
#include "fdcan.h"

#define DMJ4310_Motor_FDCAN_Length 8
#define DMJ4310_Motor_Position_Max 12.5f
#define DMJ4310_Motor_Position_Min -12.5f
#define DMJ4310_Motor_Position_Length 16 //单位为位
#define DMJ4310_Motor_Velocity_Max 30.0f
#define DMJ4310_Motor_Velocity_Min -30.0f
#define DMJ4310_Motor_Velocity_Length 12 // 单位为位
#define DMJ4310_Motor_MIT_Kp_Max 500.0f
#define DMJ4310_Motor_MIT_Kp_Min 0.0f
#define DMJ4310_Motor_MIT_Kp_Length 12 // 单位为位
#define DMJ4310_Motor_MIT_Kd_Max 5.0f
#define DMJ4310_Motor_MIT_Kd_Min 0.0f
#define DMJ4310_Motor_MIT_Kd_Length 12 // 单位为位
#define DMJ4310_Motor_Torque_Max 10.0f
#define DMJ4310_Motor_Torque_Min -10.0f
#define DMJ4310_Motor_Torque_Length 12 // 单位为位
#define DMJ4310_Motor_FDCAN_Response_ID 2
#define DMJ4310_Motor_Control_Cycle 1
#define Angle_Joint_Upper_Offset 0.07f

extern DMJ4310_Motor_Handle_t DMJ4310_Motor_Handle[];

void DMJ4310_Motor_Control_Init(void);
void DMJ4310_Motor_MIT_Control(DMJ4310_Motor_Handle_t DMJ4310_Motor_Handle);
void DMJ4310_Motor_Enable(DMJ4310_Motor_Handle_t DMJ4310_Motor_Handle);
void DMJ4310_Motor_Response_Data_Explain(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxHeaderTypeDef FDCAN_Rx_Head_Temp, uint8_t *FDCAN_Rx_Data_Temp, DMJ4310_Motor_Handle_t *DMJ4310_Motor_Handle);

#endif
