#include "BMI088_IMU_Driver.h"

void Accel_CS_Low(void)
{
    HAL_GPIO_WritePin(SPI2_CS0_GPIO_Port, SPI2_CS0_Pin, GPIO_PIN_RESET);
}

void Accel_CS_High(void)
{
    HAL_GPIO_WritePin(SPI2_CS0_GPIO_Port, SPI2_CS0_Pin, GPIO_PIN_SET);
}

void Gyro_CS_Low(void)
{
    HAL_GPIO_WritePin(SPI2_CS1_GPIO_Port, SPI2_CS1_Pin, GPIO_PIN_RESET);
}

void Gyro_CS_High(void)
{
    HAL_GPIO_WritePin(SPI2_CS1_GPIO_Port, SPI2_CS1_Pin, GPIO_PIN_SET);
}
