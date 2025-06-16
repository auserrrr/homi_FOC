#include "controller.h"
/**
 * @brief PI控制器,增量式
 * @param  Idq_sample d-q坐标系电流采样值
 * @param  Udq_out d-q坐标系电压输出值
 * @param  alapha 低通滤波系数 0.05~0.3
 * 
 */
void PI_Current_inc(CurrentPITypeDef *hpi_current,float *Idq_sample,float *Udq_out,float alpha)
{
	
	static float err_filt_d = 0;
	static float err_filt_q = 0;

	
	hpi_current->err_d = hpi_current->Id_target - Idq_sample[0];
	hpi_current->err_q = hpi_current->Iq_target - Idq_sample[1];

	err_filt_d = alpha * hpi_current->err_d + (1.0f - alpha) * err_filt_d;
	err_filt_q = alpha * hpi_current->err_q + (1.0f - alpha) * err_filt_q;


	Udq_out[0] += hpi_current->Kp_d * (err_filt_d - hpi_current->err_last_d) + hpi_current->Ki_d * err_filt_d;
    Udq_out[1] += hpi_current->Kp_q * (err_filt_q - hpi_current->err_last_q) + hpi_current->Ki_q * err_filt_q;
	
	hpi_current->err_last_d = err_filt_d;
	hpi_current->err_last_q = err_filt_q;

}

/**
 * @brief PI控制器,位置式
 * @param  Idq_sample d-q坐标系电流采样值
 * @param  Udq_out d-q坐标系电压输出值
 * 
 */
void PI_Current_pos(CurrentPITypeDef *hpi_current,float *Idq_sample,float *Udq_out)
{
	hpi_current->err_d = hpi_current->Id_target - Idq_sample[0];
	hpi_current->err_q = hpi_current->Iq_target - Idq_sample[1];

	hpi_current->err_sum_d += hpi_current->err_d ;
	hpi_current->err_sum_q += hpi_current->err_q ;
    
	sum_limit(&hpi_current->err_sum_d,1500);  //d轴积分限幅1500mA
	sum_limit(&hpi_current->err_sum_q,1500);  //q轴积分限幅1500mA

	Udq_out[0] = hpi_current->Kp_d * hpi_current->err_d + hpi_current->Ki_d * hpi_current->err_sum_d;
	Udq_out[1] = hpi_current->Kp_q * hpi_current->err_q + hpi_current->Ki_q * hpi_current->err_sum_q;
	//在SVPWM发生器环节对Udq进行限幅

}
/**
 * @brief 速度PI控制器,位置式
 * @param [in] speed_read 速度
 * @return q轴电流参考值
 * 
 */
float PI_Speed_pos(SpeedPITypeDef *hpi_speed,float speed_read)
{
	hpi_speed->speed_err  = hpi_speed->speed_target - speed_read;
	hpi_speed->speed_err_sum += hpi_speed->speed_err;

	//sum_limit(&hpi_speed->speed_err_sum,1500);  //速度积分限幅

	return hpi_speed->Kp * hpi_speed->speed_err + hpi_speed->Ki * hpi_speed->speed_err_sum;
}
/**
 * @brief 角度环PI控制器，位置式
 * @param [in] angel_read 当前角度
 * @return 速度参考值
 * 
 */
float PI_Angel_pos(AngelPITypeDef *hpi_angel,float angel_read)
{
	hpi_angel->angel_err = hpi_angel->angel_target - angel_read;
	hpi_angel->angel_err_sum += hpi_angel->angel_err;

	return hpi_angel->Kp * hpi_angel->angel_err + hpi_angel->Ki * hpi_angel->angel_err_sum;
}

/**
 * @brief 电流环(力矩环)PI控制器参数初始化
 * 
 * @param hpi_current PI控制器结构体
 * @note 纯电流环运行电机在空载情况下跑飞是正常现象
 */
void Current_PI_Init(CurrentPITypeDef *hpi_current)
{
	
	hpi_current->Id_target = 0;
	hpi_current->Iq_target = 0;
    //d轴PI控制器参数初始化
	hpi_current->Kp_d = 0.095110;
	hpi_current->Ki_d = 0.001920;
    hpi_current->err_d = 0;
	hpi_current->err_sum_d = 0;
	hpi_current->err_last_d = 0;
	//q轴PI控制器参数初始化
	hpi_current->Kp_q = -0.095110;
	hpi_current->Ki_q = -0.002120;
 	hpi_current->err_q = 0;
	hpi_current->err_sum_q = 0;
	hpi_current->err_last_q = 0;
}
/**
 * @brief 速度环PI控制器参数初始化
 * 
 * @param hpi_speed PI控制器结构体
 */
void Speed_PI_Init(SpeedPITypeDef *hpi_speed)
{
	hpi_speed->speed_target = 5;
	
	hpi_speed->Kp = 15.60;
	hpi_speed->Ki = 0.234;

	hpi_speed->speed_err = 0;
	hpi_speed->speed_err_sum = 0;
}
/**
 * @brief 角度环PI控制器参数初始化
 * 
 * @param hpi_angel PI控制器结构体
 */
void Angel_PI_Init(AngelPITypeDef *hpi_angel)
{
	hpi_angel->angel_target = 0;
	hpi_angel->Kp = 0.0001;
	hpi_angel->Ki = 0.000001;

	hpi_angel->angel_err = 0;
	hpi_angel->angel_err_sum = 0;
}

/**
 * @brief 电机母线电压控制函数
 * 
 * @param PinState 控制电平
 * 					 @arg VBUS_ON： 母线电压使能
  *                  @arg VBUS_OFF: 母线电压关闭
 */
void CTRLA_VBUS(CTRLA_PinState PinState)
{
	HAL_GPIO_WritePin(CTRLA_GPIO_Port,CTRLA_Pin,(GPIO_PinState)PinState);
}
/**
 * @brief 泄放回路控制函数
 * 
 * @param PinState 控制电平
 * 					 @arg LOOP_ON： 回路开启
  *                  @arg LOOP_OFF: 回路关闭
 */
void CTRLB_Draloop(CTRLB_PinState PinState)
{
	HAL_GPIO_WritePin(CTRLA_GPIO_Port,CTRLA_Pin,(GPIO_PinState)PinState);
}

/**
 * @brief 数学工具，幂运算
 * 
 * @param m 
 * @param n 
 * @return m^n
 */
float pow_loop(float m, unsigned int n)
{
    float result = 1.0f;
    for(unsigned int i = 0; i < n; i++)
    {
        result *= m;
    }
    return result;
}
/**
 * @brief 电流环调参串口指令解析函数
 * 
 * @param rx_Data  串口指令
 * @param PI_InitStructure 
 */
void Current_RX_Analysis(uint8_t *rx_Data,CurrentPITypeDef *hpi_current)
{
	float direction = rx_Data[2] - 48;    //方向
	float Mag = rx_Data[1] - 48;          //倍率
	float Para = rx_Data[3] -48;         //系数
	
	float var = 0.000001f * pow_loop(-1,direction) * pow_loop(10,Mag) * Para;

	switch (rx_Data[0] - 48)
	{
	case 0:
		hpi_current->Kp_d += var;
		break;
	
	case 1:
		hpi_current->Ki_d += var;
		break;

	case 2:
		hpi_current->Kp_q += var;
		break;

	case 3:
		hpi_current->Ki_q += var;
		break;

	case 4:
		hpi_current->Iq_target += var;
		break;
	default:

		break;
	}
}	

/**
 * @brief 速度环调参串口指令解析函数
 * 
 * @param rx_Data   串口指令
 * @param hpi_speed 
 */
void Speed_RX_Analysis(uint8_t *rx_Data,SpeedPITypeDef *hpi_speed)
{
	float direction = rx_Data[2] - 48;    //方向
	float Mag = rx_Data[1] - 48;          //倍率
	float Para = rx_Data[3] -48;         //系数
	
	float var = 0.000001f * pow_loop(-1,direction) * pow_loop(10,Mag) * Para;

	switch (rx_Data[0] - 48)
	{
	case 0:
		hpi_speed->Kp += var;
		break;
	
	case 1:
		hpi_speed->Ki += var;
		break;

	case 2:
		hpi_speed->speed_target += var;
		break;

	default:

		break;
	}

}
/**
 * @brief 积分限幅
 * 
 */
void sum_limit(float *err_sum,float max)
{
	if (*err_sum >= max)
	{
		*err_sum = max;
	}
	if (*err_sum <= -max)
	{
		*err_sum = -max;
	}
	
	
}
