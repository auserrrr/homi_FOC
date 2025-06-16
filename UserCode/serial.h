#ifndef __SERIAL_H
#define __SERIAL_H


#include "stm32g4xx_hal.h"
#include "stdio.h"
#include "string.h"

#include "controller.h"
#include "Signalprocess.h"

void Data_print(UART_HandleTypeDef *huart,SpeedPITypeDef hpi_speed,float *prob,AnalogTypeDef Analog_InitStructure,int8_t *sig);

#endif
