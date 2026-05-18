#ifndef __ROBOTIC_ARM_CONTROL_API_H
#define __ROBOTIC_ARM_CONTROL_API_H

#include "DMJ4310_Motor_Driver.h"
#include "LK4005_Motor_Driver.h"
#include "LFD01M_Motor_Driver.h"
#include "Control_Algorithm.h"
#include "Robotic_Arm_Communication_HAL_STM32_Port.h"

void Robotic_Arm_Control_Init(void);
void Robotic_Arm_Control(void);
void Robotic_Arm_Communication(void);

void LFD01M_Motor_Handle_Update(void);
void DMJ4310_Motor_Handle_Update(void);
void LK4005_Motor_Handle_Update(void);

#endif
