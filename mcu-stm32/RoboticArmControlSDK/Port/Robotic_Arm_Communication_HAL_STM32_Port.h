#ifndef __ROBOTIC_ARM_COMMUNICATION_HAL_STM32_PORT_H
#define __ROBOTIC_ARM_COMMUNICATION_HAL_STM32_PORT_H

#include "main.h"
#include "usart.h"

extern uint8_t Usart_Used0_Rx_Buff[];

#define Communication_Usart_Handle_Used0 &huart1
#define Communication_Usart_Instance_Used0 USART1
#define Usart_Used0_Rx_Buff_Length 10
#define Usart_Used0_Tx_Buff_Length 64

void Communication_Usart_Init(void);
void Communication_Usart(void);

#endif
