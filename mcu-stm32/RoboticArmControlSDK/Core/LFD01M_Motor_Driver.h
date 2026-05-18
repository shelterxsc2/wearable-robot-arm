#ifndef __LFD01M_MOTOR_DRIVER_H
#define __LFD01M_MOTOR_DRIVER_H

#include "Robotic_Arm_Config.h"
#include "tim.h"

#define LFD01M_Motor_PWM_CCR_Max 2500
#define LFD01M_Motor_PWM_CCR_Min 500
#define LFD01M_Motor_Control_Cycle 1
#define Angle_Servo_Offset 0.4f

extern LFD01M_Motor_Handle_t LFD01M_Motor_Handle[];

void LFD01M_Motor_Control_Init(void);
void LFD01M_Motor_Set_Angle(LFD01M_Motor_Handle_t LFD01M_Motor_Handle);

#endif
