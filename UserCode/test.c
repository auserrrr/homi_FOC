/**
 * @file test.c
 * @author 王灏 (you@domain.com)
 * @brief 测试函数，开环SVPWM测试，PWM模式调试，SPI通信等功能。
 * @version 0.1
 * @date 2025-05-20
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "test.h"


/**
 * @brief 正弦波发生器，将正弦波转换为α-β坐标系的电压值并存储在Uab数组中
 * 
 * @param d_sig 步进角
 * @param Vref 正弦波幅值
 * @param [out] Uab Uab[0]:Uα，Uab[1]:Uβ，Uab[2]:步进角累加值(大于2π时需清零，防止溢出)
 */
void SIN_GENER(float sig,float Vref,float *Uab)
{
	
	Uab[0] = Vref * arm_sin_f32(sig);
	Uab[1] = Vref * arm_cos_f32(sig);
	
}
/**
 * @brief 三相交错波形发生器
 * 
 * @param angle 
 * @param Iabc 
 */
void Three_phase_gener(float angle,float *Iabc)
{
	Iabc[0] = arm_cos_f32(angle);
	Iabc[1] = arm_cos_f32(angle + angle240);
	Iabc[2] = arm_cos_f32(angle + angle120);
}

/**
 * @brief 正弦波发生器串口调试工具，打印Uα，Uβ的值
 * 
 * @param huart 串口句柄
 * @param [in] Uab Uab[0]:Uα，Uab[1]:Uβ，Uab[2]:步进角累加值(0~2π)
 * @note 该函数需要在串口初始化完成后调用
 */
void SIN_GENER_Serial_Debug(UART_HandleTypeDef *huart,float *Uab)
{
	char DataSerial[60] = " ";
	sprintf(DataSerial,"%.2f,%.2f,%.2f\r\n",Uab[0],Uab[1],Uab[2]);
	HAL_UART_Transmit_IT(huart,(uint8_t*)DataSerial,strlen(DataSerial));
}

/**
 * @brief PWM输出函数
 * 
 * @param htim 定时器句柄
 * @param [in] Duty 待写入CCR寄存器的计数值数组，长度为3
 * @note 该函数需要在TIM_PWM_Init函数之后调用
 */
void PWM_test(TIM_HandleTypeDef *htim,float *Duty)
{
	uint16_t CCR_Value[3] = {0,0,0};
	for (uint8_t i = 0; i <3; i++)
	{
		CCR_Value[i] = 1000 - 10 * Duty[i];
	}
	
	__HAL_TIM_SetCompare(htim,TIM_CHANNEL_1,CCR_Value[0]);
	__HAL_TIM_SetCompare(htim,TIM_CHANNEL_2,CCR_Value[1]);
	__HAL_TIM_SetCompare(htim,TIM_CHANNEL_3,CCR_Value[2]);
}
/** 
 * @brief SVPWM生成测试，输入变量为Uα，Uβ；输出变量为未处理(未转换成占空比或计数值)的三相交错SVPWM波形
 * @param [in] Uab:输入变量，Uα，Uβ
 * @param [out] MIDD:输出变量，未处理的三相交错SVPWM波形
 * @param Vdc:直流母线电压
 * @param Z:增益系数	
 */
void SVPWM_test(float *Uab,float *MIDD,float Vdc, float Z)
{
	float Uxyz[3] = {0};

	Uxyz[0] =  0.8660254f * Uab[0] + 0.5f * Uab[1];
    Uxyz[1] = -0.8660254f * Uab[0] + 0.5f * Uab[1];
    Uxyz[2] = Uab[1];
	
	switch (Sector(Uxyz)) 
    {
        case 1:

            MIDD[0] = (Vdc + Uxyz[0] - Uxyz[1])*Z;
            MIDD[1] = (Vdc + Uxyz[0] + Uxyz[1])*Z;
            MIDD[2] = (Vdc - Uxyz[0] - Uxyz[1])*Z;
            break;                                                                                         
																																
        case 2:                                                                                                                 
																																
            MIDD[0] = (Vdc + Uxyz[0] - Uxyz[2])*Z;
            MIDD[1] = (Vdc - Uxyz[0] + Uxyz[2])*Z;
            MIDD[2] = (Vdc - Uxyz[0] - Uxyz[2])*Z;
            break;                                                                                                              
																																
        case 3:                                                                                                                 
																																
            MIDD[0] = (Vdc - Uxyz[1] + Uxyz[2])*Z;
            MIDD[1] = (Vdc + Uxyz[1] + Uxyz[2])*Z;
            MIDD[2] = (Vdc + Uxyz[1] - Uxyz[2])*Z;
            break;                                                                                                              
																																
        case 4:                                                                                                                 
																																
            MIDD[0] = (Vdc - Uxyz[1] + Uxyz[2])*Z;
            MIDD[1] = (Vdc + Uxyz[1] + Uxyz[2])*Z;
            MIDD[2] = (Vdc + Uxyz[1] - Uxyz[2])*Z;
            break;                                                                                
										                                                          
        case 5:                                                                                   
										                                                          
            MIDD[0] = (Vdc + Uxyz[0] - Uxyz[2])*Z;
            MIDD[1] = (Vdc - Uxyz[0] + Uxyz[2])*Z;
            MIDD[2] = (Vdc - Uxyz[0] - Uxyz[2])*Z;
            break;                                                                                
										                                                          
        case 6:                                                                                   
										                                                          
            MIDD[0] = (Vdc + Uxyz[0] - Uxyz[1])*Z;
            MIDD[1] = (Vdc + Uxyz[0] + Uxyz[1])*Z;
            MIDD[2] = (Vdc - Uxyz[0] - Uxyz[1])*Z;
            break;                      

        default:
            break;
            
}
	
}
/**
 * @brief SVPWM串口调试工具，打印SVPWM生成的三相交错波形
 * 
 * @param [in] huart 串口句柄
 * @param [in] Uxyz_test 待打印的数据，三个元素
 * @note 该函数需要在串口初始化完成后调用
 */
void SVPWM_test_Serial_Debug(UART_HandleTypeDef *huart,float *Uxyz_test)
{
	char DataSerial[60] = " ";
	sprintf(DataSerial,"%.2f,%.2f,%.2f\r\n",Uxyz_test[0],Uxyz_test[1],Uxyz_test[2]);
	HAL_UART_Transmit_IT(huart,(uint8_t*)DataSerial,strlen(DataSerial));
}
/**************************************************************************************************/
/**
 * @brief 25Q32 SPI Flash芯片的使能函数(CS引脚控制)
 * 
 * @note 该函数需要在SPI初始化完成后调用
 */
void W25Q32_Enable()
{
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
}
/**
 * @brief 25Q32 SPI Flash芯片的禁用函数(CS引脚控制)
 * 
 * @note 该函数需要在SPI初始化完成后调用
 */
void W25Q32_Disable()
{
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);
}

/**
 * @brief 25Q32 SPI Flash芯片复位函数
 * 
 * @param hspi SPI句柄
 */
void W25Q32_Reset(SPI_HandleTypeDef *hspi)
{
	uint8_t cmd[2] = {0x66,0x99};
	
	W25Q32_Enable();
	/* Send the reset command */
	HAL_SPI_Transmit(hspi,cmd,2,1000);	
	W25Q32_Disable();

}

/**
 * @brief 25Q32 SPI Flash芯片读取ID
 * 
 */
uint16_t W25Q32_Read_ID(SPI_HandleTypeDef *hspi)
{
	uint8_t cmd[4];
	uint8_t receive[2];
	
	uint16_t ID = 0;
	
	cmd[0] = 0x90;                         //读命令
	cmd[1] = (uint8_t)(0x00);   
	cmd[2] = (uint8_t)(0x00);
	cmd[3] = (uint8_t)(0x00);
	
	W25Q32_Enable();
	HAL_SPI_Transmit(hspi,cmd,4,1000);
	
	HAL_SPI_Receive(hspi,receive,2,1000);
	
	//高位先行(寄存器高八位数据被发送到receive[0])，位操作调整数据顺序(CUBEMX设置Data Size为8bit)
	ID |= receive[0]<<8;
	ID |= receive[1];
	
	W25Q32_Disable();
	return ID;
}
/**************************************************************************************/
/**
 * 
 */
void ANGEL_Serial_Debug(UART_HandleTypeDef *huart,float angle)
{
	char DataSerial[12] = " ";
	sprintf(DataSerial,"angel:%.2f\r\n",angle);
	HAL_UART_Transmit_IT(huart,(uint8_t*)DataSerial,strlen(DataSerial));
}

void Current_Serial_Debug(UART_HandleTypeDef *huart,float *Current_Value)
{
	char DataSerial[30] = " ";
	sprintf(DataSerial,"%.2f,%.2f,%.2f\r\n",Current_Value[0],Current_Value[1],Current_Value[2]);
	HAL_UART_Transmit_IT(huart,(uint8_t*)DataSerial,strlen(DataSerial));
}

/**
 * @brief LED控制
 * 
 * @param PinState 
 */
void LEDA_set(GPIO_PinState PinState)
{
	HAL_GPIO_WritePin(GPIOC,Test_A_Pin,PinState);
}

void LEDB_set(GPIO_PinState PinState)
{
	HAL_GPIO_WritePin(GPIOC,Test_B_Pin,PinState);
}

