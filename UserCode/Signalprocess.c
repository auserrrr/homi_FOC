/**
 * @file Signalprocess.c
 * @author 坏厚米 (you@domain.com)
 * @brief 模拟信号处理，电流、电压、角度、速度等模拟数据的采样，滤波，坐标系转换等
 * @version 0.1
 * @date 2025-05-20
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "Signalprocess.h"
/**
 * @brief Clark变换函数
 * 
 * @param [in] analog_Value 电流采样值(模拟量)
 * @param [out] Iab 转换值Iab[0]:Iα，Iab[1]:Iβ(α-β坐标系) 
 */
void Clark(float *analog_Value,float *Iab)
{
	 Iab[0] = analog_Value[0];
     Iab[1] = ((float32_t) 0.57735026919 * analog_Value[0] + (float32_t) 1.15470053838 * analog_Value[1]);
}
/**
 * @brief Park变换
 * 
 * @param [in] Iab α-β坐标系电流值，Iab[0]:Iα，Iab[1]:Iβ
 * @param [out] Idq d-q坐标系电流值，Idq[0]:Id，Idq[1]:Iq
 * @param [in] angle 当前电机转子角度 
 */
void Park(float *Iab,float *Idq,float angle)
{
	 float sinVal = arm_sin_f32(angle);
	 float cosVal = arm_cos_f32(angle);
	
	Idq[0] =  Iab[0] * cosVal + Iab[1] * sinVal;
    Idq[1] = -Iab[0] * sinVal + Iab[1] * cosVal;
}

/**
 * @brief 角度传感器初始化函数(基于SPI_DMA通道，连续模式)
 * 
 * @param hspi SPI句柄
 * @param AS5048A_get AS5048A寄存器读取值
 * @note 将SPI_DMA模式设置为连续模式，仅需调用一次此函数即可连续自动读取角度
 */
void AS5048A_Init(SPI_HandleTypeDef *hspi,uint16_t *AS5048A_get)
{
	static uint16_t tx = 0xffff;

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive_DMA(hspi, (uint8_t *)tx, (uint8_t *)AS5048A_get,1);
	

}


/**
 * @brief 读取AS5048A编码器值并转换为弧度角度（0~2π）
 * @param AS5048A_get 编码器原始数据指针
 * @return 当前角度（弧度）
 */
float absolute_angle(uint16_t AS5048A_get)
{
    uint16_t raw = (AS5048A_get) & 0x3FFF;
   
   //return 6.2831853f - (6.2831853f * raw / 0x3FFF);
   return 6.2831853f * (1.0f - (float)raw / 0x3FFF);
}

/**
 * @brief 速度检测，对两次角度差进行微分，得到速度
 * @param angle 当前角度（弧度，0~2π）
 * @param dia   比例系数
 * @param threshold 阈值角度，取值区间：(0~2π)采样周期内角度变化只要不超过此角度，
 * 算法都能正确判断方向和大小，如果电机转速过快或采样周期过低，计算结果会严重失真。
 * @param alpha 滤波系数
 * @return 估算速度
 * @note 
 */
float speed_get(float angle, float dia, float threshold,float alpha)
{
    static float last_angle = 0.0f;
	static float speed_filt = 0.0f;
	//static float speed_filt2 = 0.0f;

    float delta = angle - last_angle;

	/**
	 * 绕环处理机制：
     *如果差值大于π（180°），说明是正向跨越了零点（2π→0），实际应该减去一圈的长度（2π）
     *如果差值小于-π，说明是反向跨越了零点（0→2π），实际应该加上一圈的长度（2π）
	 * 对于非零点，此处理机制不会影响结果
	 */
    if (delta > threshold) 
	{
        delta -= 6.2831853f;
    } 
	else if (delta < -threshold) 
	{
        delta += 6.2831853f;
    }

    last_angle = angle;
	speed_filt = alpha * delta * dia + (1.0f - alpha) * speed_filt;
	//speed_filt2 = alpha * speed_filt + (1.0f - alpha) * speed_filt2;
	
    return speed_filt;
}

/**
 * @brief ADC初始化函数，使能ADC_DMA连续模式，此函数仅需调用一次即可自动读取ADC采集值
 * 
 * @param hadc ADC句柄
 * @param [out] ADC_get ADC读取数组，长度为5
 */
void ADC_Init(ADC_HandleTypeDef *hadc,uint16_t *ADC_get)
{
	HAL_ADCEx_Calibration_Start(hadc,ADC_SINGLE_ENDED);  //ADC校准
	HAL_ADC_Start_DMA(hadc,(uint32_t *)ADC_get,5);   //ADC_DMA使能
	
}

/**
 * @brief 模拟量运算结果结构体初始化
 * 
 * @param Analog_InitStructure 
 */
void Analog_Init(AnalogTypeDef *Analog_InitStructure)
{
	 memset(Analog_InitStructure->ADC_get,0,sizeof(Analog_InitStructure->ADC_get));
	 Analog_InitStructure->AS5048A_get = 0;
	  memset(Analog_InitStructure->Current_Value,0,sizeof(Analog_InitStructure->ADC_get));
	 Analog_InitStructure->elec_angle = 0;
	 memset(Analog_InitStructure->Idq,0,sizeof(Analog_InitStructure->Idq));
	 memset(Analog_InitStructure->Udq,0,sizeof(Analog_InitStructure->Udq));
}
/**
 * @brief ADC数据处理
 * 
 * @param prop1 ADC寄存器归算系数，将电流采样原始值归算至mA单位
 * @param prop2 低通滤波常数
 */
void Current_transform(AnalogTypeDef *Analog_InitStructure,float prop1,float prop2)
{
	
	static float filter_buff[3] = {0,0,0};
	
		filter_buff[0] = (Analog_InitStructure->ADC_get[0] - 1344.8) * prop1;
		filter_buff[1] = (Analog_InitStructure->ADC_get[1] - 1344.8) * prop1;
		filter_buff[2] = (Analog_InitStructure->ADC_get[2] - 1344.8) * prop1;

		Analog_InitStructure->Current_Value[0] = (filter_buff[0] * prop2 + Analog_InitStructure->Current_Value[0] * (1 - prop2));
		Analog_InitStructure->Current_Value[1] = (filter_buff[1] * prop2 + Analog_InitStructure->Current_Value[1] * (1 - prop2));
		Analog_InitStructure->Current_Value[2] = (filter_buff[2] * prop2 + Analog_InitStructure->Current_Value[2] * (1 - prop2));
	
}
/**
 * @brief Ia、Ib、Ic电流转换到d—q轴
 * 
 */
void Iabc_to_Idq(AnalogTypeDef *Analog_InitStructure)
{
	 float rad0 = Analog_InitStructure->elec_angle - angle120;
	 float rad1 = Analog_InitStructure->elec_angle + angle120;
    
	 Analog_InitStructure->Idq[1] = arm_sin_f32(Analog_InitStructure->elec_angle) * Analog_InitStructure->Current_Value[0] + arm_sin_f32(rad0) * Analog_InitStructure->Current_Value[1] + arm_sin_f32(rad1) * Analog_InitStructure->Current_Value[2];
	 Analog_InitStructure->Idq[0] = arm_cos_f32(Analog_InitStructure->elec_angle) * Analog_InitStructure->Current_Value[0] + arm_cos_f32(rad0) * Analog_InitStructure->Current_Value[1] + arm_cos_f32(rad1) * Analog_InitStructure->Current_Value[2];
	
	Analog_InitStructure->Idq[0] *= k;
	Analog_InitStructure->Idq[1] *= k;
}
