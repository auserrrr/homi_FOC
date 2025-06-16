#ifndef __SVPWM_H
#define __SVPWM_H


#include "stm32g4xx_hal.h"
#include "arm_math.h"
#include "stdio.h"
#include "string.h"

#define rad60     1.04719755f //60度角弧度值   
#define MODU      0           //过调制系数
#define POLEPAIR  11.0f       //极对数

#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))

/**
 * @brief SVPWM结构体
 * @note 包含所有关于SVPWM发波器的参数
 */
typedef struct
{

    uint16_t TIMLOAD;     /**<定时器计数值 Counter Period*/
    uint16_t dTIMLOAD;    /**<定时器步长 = TIMLOAD / Duty_max(100) */
	
	float U_alpha;        /**<α坐标轴电压值 */
	float U_beta;         /**<β坐标轴电压值 */
	
    float Vdc;            /**<母线电压，实际作为一个运算参数，在运算过程中会被约分 */
    float Z;              /**<占空比归算系数:100/Vdc/2*0.95，此系数由简化SVPWM过程产生*/
  
    float Uxyz[3];        /**<三相电压U、V、W寄存数组，幅值为Vdc */
	
}SVPWMTypeDef;

void PWM_Enable(TIM_HandleTypeDef *htim);
void SVPWM_Init(SVPWMTypeDef *SVPWM_InitStructure);
void SVPWM_GENER(float *Udq,float angle,SVPWMTypeDef *SVPWM_InitStructure,TIM_HandleTypeDef *htim,float *prob);

#endif
