#ifndef __ROBOTIC_ARM_HAL_STM32_PORT_H
#define __ROBOTIC_ARM_HAL_STM32_PORT_H

#include "Robotic_Arm_Config.h"
#include "main.h"

void Motor_Control_Init(void);
void FDCAN_Send_Standard(FDCAN_HandleTypeDef *FDCAN_Handle, uint16_t std_id, uint8_t *data, uint8_t length);
uint16_t Float_To_Uint(float x, float x_min, float x_max, int bits);
float Uint_To_Float(int x_int, float x_min, float x_max, int bits);

#endif
