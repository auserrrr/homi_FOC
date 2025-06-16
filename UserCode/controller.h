#ifndef __CONTROLLER_H
#define __CONTROLLER_H


#include "stm32g4xx_hal.h"
#include "arm_math.h"
#include "main.h"
#include "svpwm.h"

/**
 * @brief 电流环PI控制器结构体
 * @note 包含d、q轴所有关于PI控制器的变量
 * 
 */
typedef struct
{
	
	float Iq_target;     /**<q轴参考电流，单位：mA */
	float Id_target;     /**<d轴参考电流(0,永磁电机)，单位：mA */
	
	float Kp_d;          /**<d轴比例系数*/
	float Ki_d;          /**<d轴积分系数 */
    float err_d;         /**<d轴本次误差 */
	float err_sum_d;     /**<d轴误差积分(用于位置式PI控制器) */
	float err_last_d;    /**<d轴上次误差(用于增量式PI控制器) */

	float Kp_q;          /**<q轴比例系数 */
	float Ki_q;          /**<q轴积分系数 */
	float err_q;         /**<q轴本次误差 */
	float err_sum_q;     /**<q轴误差积分(用于位置式PI控制器) */
	float err_last_q;    /**<q轴上次误差(用于增量式PI控制器) */

}CurrentPITypeDef;

/**
 * @brief 速度环PI(位置式)控制器结构体
 * @note 包含关于速度环PI(位置式)控制器所有变量
 */
typedef struct 
{
	float speed_target;  /**<速度参考值，无单位(未经归算) */

	float speed_err;     /**<速度误差 */
	float speed_err_sum; /**<速度误差积分 */

	float Kp;            /**<比例系数 */
	float Ki;            /**<积分系数 */

}SpeedPITypeDef;

/**
 * @brief 角度环PI(位置式)控制器结构体
 * @note 包含关于角度环PI(位置式)控制器所有变量
 * 
 */
typedef struct 
{
	float angel_target;  /**<角度参考值，单位：rad */

	float angel_err;     /**<角度误差 */
	float angel_err_sum; /**<角度误差积分 */

	float Kp;            /**<比例系数 */
	float Ki;            /**<积分系数 */

}AngelPITypeDef;

/**
 * @brief VBUS控制引脚状态
 * 
 */
typedef enum
{
  VBUS_ON = 0U,         /**<VBUS使能 */
  VBUS_OFF              /**<VBUS关闭 */
} CTRLA_PinState;
/**
 * @brief 泄放回路控制引脚
 * 
 */
typedef enum
{
  LOOP_OFF = 0U,       /**<回路关闭 */
  LOOP_ON              /**<回路开启，制动状态 */
} CTRLB_PinState;


void Current_PI_Init(CurrentPITypeDef *hpi_current);
void Speed_PI_Init(SpeedPITypeDef *hpi_speed);
void Angel_PI_Init(AngelPITypeDef *hpi_angel);

void PI_Current_inc(CurrentPITypeDef *hpi_current,float *Idq_sample,float *Udq_out,float alpha);
void PI_Current_pos(CurrentPITypeDef *hpi_current,float *Idq_sample,float *Udq_out);
float PI_Speed_pos(SpeedPITypeDef *hpi_speed,float speed_read);
float PI_Angel_pos(AngelPITypeDef *hpi_angel,float angel_read);

void CTRLA_VBUS(CTRLA_PinState PinState);
void CTRLB_Draloop(CTRLB_PinState PinState);

float pow_loop(float m, unsigned int n);
void Current_RX_Analysis(uint8_t *rx_Data,CurrentPITypeDef *hpi_current);
void Speed_RX_Analysis(uint8_t *rx_Data,SpeedPITypeDef *hpi_speed);
void sum_limit(float *err_sum,float max);

#endif
