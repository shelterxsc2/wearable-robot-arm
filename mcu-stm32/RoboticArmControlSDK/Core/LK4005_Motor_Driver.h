#ifndef __LK4005_MOTOR_DRIVER_H
#define __LK4005_MOTOR_DRIVER_H

#include "main.h"
#include "Robotic_Arm_Config.h"
#include "Robotic_Arm_Motor_HAL_STM32_Port.h"
#include "fdcan.h"

#define LK4005_Motor_FDCAN_Length 8
#define Reduction_Ratio 10.0f
#define Position_Conversion_Ratio 0.01f
#define Torque_Conversion_Ratio 62.0606f
#define Angle_Gimbal_Offset 0.75f //云台电机测得的角度比实际角度大的值,单位为rad
#define Angle_Joint_Fore_Offset -0.18f // 关节电机测得的角度比实际角度大的值,单位为rad
#define LK4005_Motor_Control_Cycle 1 //控制周期(不计算等待FDCAN的4ms),单位为ms

extern LK4005_Motor_Handle_t LK4005_Motor_Handle[];

void LK4005_Motor_Control_Init(void);
void LK4005_Motor_Torque_Control(LK4005_Motor_Handle_t LK4005_Motor_Handle, Motor_MIT_Control_Handle_t Motor_MIT_Control_Handle);
void LK4005_Motor_Position_Control(LK4005_Motor_Handle_t LK4005_Motor_Handle);
void LK4005_Motor_Read_Position(LK4005_Motor_Handle_t LK4005_Motor_Handle);
void LK4005_Motor_Read_Velocity(LK4005_Motor_Handle_t LK4005_Motor_Handle);
void LK4005_Motor_Response_Data_Explain(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxHeaderTypeDef FDCAN_Rx_Head_Temp, uint8_t *FDCAN_Rx_Data_Temp, LK4005_Motor_Handle_t *LK4005_Motor_Handle);

#endif
