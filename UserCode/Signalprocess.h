#ifndef __SIGNALPROCESS_H
#define __SIGNALPROCESS_H


#include "stm32g4xx_hal.h"
#include "main.h"
#include "arm_math.h"
#include "test.h"
#include "string.h"

#define angle60   1.04719755f    //60度角弧度值
#define k         0.66666666f   //2/3放缩系数,Iabc to Idq时使用

/**
 * @brief 模拟量运算结果结构体
 * @note 用于寄存ADC电压原始数据采集,电流归算值,d-q坐标系电压电流值,机械角度,电角度,电机旋转速度等过程模拟量,
 *       对于FOC所有关于模拟量运算的函数均需要调用此结构体
 */
typedef struct
{

   uint16_t ADC_get[5];        /**<ADC采集原始数据，(0~4095) */
   float Current_Value[3];     /**<电机三相电流归算值，单位：mA */
   float Idq[2];               /**<电流环PI控制器输入，d-q坐标系电流值 */
   float Udq[2];               /**<电流环PI控制器输出，d-q坐标系电压值 */ 

   
   float elec_angle;           /**<电角度 */
   float mech_angle;           /**<机械角度，单位：rad */
   float speed;                /**<电机旋转速度 */
   uint16_t AS5048A_get;       /**<编码器原始数据，16bit */
	
}AnalogTypeDef;

void AS5048A_Init(SPI_HandleTypeDef *hspi,uint16_t *AS5048A_get);
float absolute_angle(uint16_t AS5048A_get);
void Analog_Init(AnalogTypeDef *Analog_InitStructure);
float speed_get(float angle, float dia, float threshold,float alpha);

void ADC_Init(ADC_HandleTypeDef *hadc,uint16_t *ADC_get);
void Current_transform(AnalogTypeDef *Analog_InitStructure,float prop1,float prop2);
void Iabc_to_Idq(AnalogTypeDef *Analog_InitStructure);

#endif
