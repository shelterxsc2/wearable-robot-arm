#ifndef __ROBOTIC_ARM_CONTROL_API_H
#define __ROBOTIC_ARM_CONTROL_API_H

#include "LK4005_Motor_Driver.h"
#include "Servo_Motor_Driver.h"
#include "Control_Algorithm.h"
#include "Robotic_Arm_Communication_HAL_STM32_Port.h"

void Robotic_Arm_Control_Init(void);
void Robotic_Arm_Control(void);
void Servo_Motor_Handle_Update(void);
void LK4005_Motor_Handle_Update(void);

extern uint8_t Servo_Control_Active;
extern uint8_t Init_Sequence_Trigger;

#endif
