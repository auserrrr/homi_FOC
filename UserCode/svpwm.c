/**
 * @file svpwm.c
 * @author 坏厚米 (you@domain.com)
 * @brief SVPWM算法相关函数，包括结构体初始化，扇区判定，波形产生等功能
 * @version 0.1
 * @date 2025-05-21
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "svpwm.h"
/**
 * @brief 扇区判定函数
 * 
 * @param [in] Uxyz  
 * @return 扇区判定值
 */
uint8_t Sector(float *Uxyz)
{
    uint8_t a = 0,b = 0,c = 0;

    if (Uxyz[0] <= 0) c = 1;
    if (Uxyz[1] <= 0) b = 1;
    if (Uxyz[2]  > 0) a = 1;

    return 4*c + 2*b + a;
}
/**
 * @brief PWM使能，使能高级定时器PWM主从模式的三个通道
 * 
 * @param htim 定时器句柄
 */
void PWM_Enable(TIM_HandleTypeDef *htim)
{
	HAL_TIM_Base_Start(htim);
	HAL_TIM_PWM_Start(htim,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(htim,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(htim,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(htim,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(htim,TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(htim,TIM_CHANNEL_3);
}

/**
 * @brief SVPWM结构体初始化
 * 
 * @param SVPWM_InitStructure 结构体
 */
void SVPWM_Init(SVPWMTypeDef *SVPWM_InitStructure)
{
	SVPWM_InitStructure->TIMLOAD = 1000;
	SVPWM_InitStructure->dTIMLOAD = 10;
    SVPWM_InitStructure->Vdc = 29.8;
    SVPWM_InitStructure->Z = 1.593960;
}
/**
 * @brief SVPWM波形产生
 * 
 * @param [in] Udq d-q轴电压值 
 * @param [in] angle 电机当前角度
 * @param SVPWM_InitStructure SVPWM结构体
 * 
 * @note 代码执行时间为5.9us，若加入打印三个浮点数的部分(阻塞方式)执行时间上升为2.4ms
 *       使用中断发送方式执行时间为140us
 */
void SVPWM_GENER(float *Udq,float angle,SVPWMTypeDef *SVPWM_InitStructure,TIM_HandleTypeDef *htim,float *prob)
{
    float MIDD0,MIDD1,MIDD2;

    float sin_Value = arm_sin_f32(angle);
    float cos_Value = arm_cos_f32(angle);

    //Udq限幅:Udq小于母线电压Vdc*0.577
    Udq[0] = min(Udq[0], 13.848f);
    Udq[0] = max(Udq[0],-13.848f);
    Udq[1] = min(Udq[1], 13.848f);
    Udq[1] = max(Udq[1],-13.848f);

	//反Park变换，d-q轴(旋转坐标系，电压)——>α-β轴(正交坐标系，电压)
	arm_inv_park_f32(Udq[0],Udq[1],&(SVPWM_InitStructure->U_alpha), &(SVPWM_InitStructure->U_beta),sin_Value,cos_Value);
	
	//α-β轴(正交坐标系，电压)-->三相电压值
	SVPWM_InitStructure->Uxyz[0] =  0.86602540f * SVPWM_InitStructure->U_alpha + 0.50f * SVPWM_InitStructure->U_beta;
    SVPWM_InitStructure->Uxyz[1] = -0.86602540f * SVPWM_InitStructure->U_alpha + 0.50f * SVPWM_InitStructure->U_beta;
    SVPWM_InitStructure->Uxyz[2] = SVPWM_InitStructure->U_beta;
   	
    switch (Sector(SVPWM_InitStructure->Uxyz)) 
    {
        case 1:

            MIDD0 = (SVPWM_InitStructure->Vdc + SVPWM_InitStructure->Uxyz[0] - SVPWM_InitStructure->Uxyz[1])*SVPWM_InitStructure->Z;
            MIDD1 = (SVPWM_InitStructure->Vdc + SVPWM_InitStructure->Uxyz[0] + SVPWM_InitStructure->Uxyz[1])*SVPWM_InitStructure->Z;
            MIDD2 = (SVPWM_InitStructure->Vdc - SVPWM_InitStructure->Uxyz[0] - SVPWM_InitStructure->Uxyz[1])*SVPWM_InitStructure->Z;
            break;                                                                                                              
																																
        case 2:                                                                                                                 
																																
            MIDD0 = (SVPWM_InitStructure->Vdc + SVPWM_InitStructure->Uxyz[0] - SVPWM_InitStructure->Uxyz[2])*SVPWM_InitStructure->Z;
            MIDD1 = (SVPWM_InitStructure->Vdc - SVPWM_InitStructure->Uxyz[0] + SVPWM_InitStructure->Uxyz[2])*SVPWM_InitStructure->Z;
            MIDD2 = (SVPWM_InitStructure->Vdc - SVPWM_InitStructure->Uxyz[0] - SVPWM_InitStructure->Uxyz[2])*SVPWM_InitStructure->Z;
            break;                                                                                                              
																																
        case 3:                                                                                                                 
																																
            MIDD0 = (SVPWM_InitStructure->Vdc - SVPWM_InitStructure->Uxyz[1] + SVPWM_InitStructure->Uxyz[2])*SVPWM_InitStructure->Z;
            MIDD1 = (SVPWM_InitStructure->Vdc + SVPWM_InitStructure->Uxyz[1] + SVPWM_InitStructure->Uxyz[2])*SVPWM_InitStructure->Z;
            MIDD2 = (SVPWM_InitStructure->Vdc + SVPWM_InitStructure->Uxyz[1] - SVPWM_InitStructure->Uxyz[2])*SVPWM_InitStructure->Z;
            break;                                                                                                              
																																
        case 4:                                                                                                                 
																																
            MIDD0 = (SVPWM_InitStructure->Vdc - SVPWM_InitStructure->Uxyz[1] + SVPWM_InitStructure->Uxyz[2])*SVPWM_InitStructure->Z;
            MIDD1 = (SVPWM_InitStructure->Vdc + SVPWM_InitStructure->Uxyz[1] + SVPWM_InitStructure->Uxyz[2])*SVPWM_InitStructure->Z;
            MIDD2 = (SVPWM_InitStructure->Vdc + SVPWM_InitStructure->Uxyz[1] - SVPWM_InitStructure->Uxyz[2])*SVPWM_InitStructure->Z;
            break;                                                                                
										                                                          
        case 5:                                                                                   
										                                                          
            MIDD0 = (SVPWM_InitStructure->Vdc + SVPWM_InitStructure->Uxyz[0] - SVPWM_InitStructure->Uxyz[2])*SVPWM_InitStructure->Z;
            MIDD1 = (SVPWM_InitStructure->Vdc - SVPWM_InitStructure->Uxyz[0] + SVPWM_InitStructure->Uxyz[2])*SVPWM_InitStructure->Z;
            MIDD2 = (SVPWM_InitStructure->Vdc - SVPWM_InitStructure->Uxyz[0] - SVPWM_InitStructure->Uxyz[2])*SVPWM_InitStructure->Z;
            break;                                                                                
										                                                          
        case 6:                                                                                   
										                                                          
            MIDD0 = (SVPWM_InitStructure->Vdc + SVPWM_InitStructure->Uxyz[0] - SVPWM_InitStructure->Uxyz[1])*SVPWM_InitStructure->Z;
            MIDD1 = (SVPWM_InitStructure->Vdc + SVPWM_InitStructure->Uxyz[0] + SVPWM_InitStructure->Uxyz[1])*SVPWM_InitStructure->Z;
            MIDD2 = (SVPWM_InitStructure->Vdc - SVPWM_InitStructure->Uxyz[0] - SVPWM_InitStructure->Uxyz[1])*SVPWM_InitStructure->Z;
            break;                      

        default:
            break;
            
}

            prob[0] =  MIDD0;
            prob[1] =  MIDD1;
            prob[2] =  MIDD2;
//过调制处理：
            // if (MIDD0 + MIDD1 > MODU) 
            // {
            //     float MIDDsum = MIDD0 + MIDD1;
				 
            //     MIDD0 = MIDD0 / MIDDsum;
            //     MIDD1 = MIDD1 / MIDDsum;
            //     MIDD2 = 0;
            // }

        uint16_t d0 = SVPWM_InitStructure->TIMLOAD - SVPWM_InitStructure->dTIMLOAD * MIDD0;
        uint16_t d1 = SVPWM_InitStructure->TIMLOAD - SVPWM_InitStructure->dTIMLOAD * MIDD1;
        uint16_t d2 = SVPWM_InitStructure->TIMLOAD - SVPWM_InitStructure->dTIMLOAD * MIDD2;

		__HAL_TIM_SetCompare(htim,TIM_CHANNEL_1,d0);
		__HAL_TIM_SetCompare(htim,TIM_CHANNEL_2,d1);
		__HAL_TIM_SetCompare(htim,TIM_CHANNEL_3,d2);
}


