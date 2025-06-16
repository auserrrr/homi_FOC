#ifndef __TEST_H
#define __TEST_H


#include "stm32g4xx_hal.h"
#include "arm_math.h"
#include "stdio.h"
#include "string.h"
#include "svpwm.h"
#include "main.h"

#define angle120  2.0943951024f
#define angle240  4.1887902048f


uint8_t Sector(float *Uxyz);

void SIN_GENER(float sig,float Vref,float *Uab);
void Three_phase_gener(float angle,float *Iabc);
void SIN_GENER_Serial_Debug(UART_HandleTypeDef *huart,float *Uab);

void PWM_test(TIM_HandleTypeDef *htim,float *Duty);
void SVPWM_test(float *Uab,float *MIDD,float Vdc, float Z);
void SVPWM_test_Serial_Debug(UART_HandleTypeDef *huart,float *Uxyz_test);

void W25Q32_Reset(SPI_HandleTypeDef *hspi);
uint16_t W25Q32_Read_ID(SPI_HandleTypeDef *hspi);

void ANGEL_Serial_Debug(UART_HandleTypeDef *huart,float angle);
void Current_Serial_Debug(UART_HandleTypeDef *huart,float *Current_Value);

void LEDA_set(GPIO_PinState PinState);
void LEDB_set(GPIO_PinState PinState);

#endif
