# 厚米FOC驱动库
***
### 主要功能：
- 无刷电机电流、速度闭环驱动
- 基于VOFA+上位机(串口通信)波形显示与调参
- 可编程按键交互，可用于电机起动、急停、上位机显示内容选择等功能
### 硬件说明：https://oshwhub.com/al_user/esc_new
***
### FOC控制框图与代码架构
#### 电流环

![CURRENT](https://github.com/user-attachments/assets/e00fe871-8ecc-4540-a3bd-38b3f34c326b)
<p align="center">图1&emsp;&emsp;FOC电流环控制流程图</p>
其中"Iabc to Idq"环节是Clark变换与Pack变换的结合，输入参数为经过ADC采集的三相电机电流：Ia，Ib，Ic；输出参数为经过Clark变换(Ia,Ib,Ic投影到α-β轴)与Park变换(α-β轴投影到d-q轴)的结果Id与Iq。再经过PI控制器得到Ud与Uq的值，进入SVPWM发生器(内置反Park变换)生成对电机的控制电压。注意无论是Park变换还是反Park变换都需要读取当前电机的绝对位置才能进行正常运算。

代码路径：```UserCode/controller.c```


```
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
}
```
在对电流环控制之前，需要初始化电流采集、转换两个阶段。
- 电流采集通过初始化ADC DMA连续转换模式(仅需要在main函数里初始化一次)自动刷新；
- 电流转换通过函数```Current_transform()```与函数```Iabc_to_Idq()```完成。
***
电流采集频率约为100KHz <br>
角度采集频率为10KHz，通过单独的定时器控制频率 <br>
电流转换与PI控制器频率同频，通过定时中断控制为5KHz <br>
电流环控制主要代码如下(```main.c```)：
```
//定时中断回调函数      
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim == &htim2)    //5KHZ(200us) FOC力矩环(电流环)控制频率
   {
       Current_transform(&Analog_InitStructure,1.859,0.03);  
       Iabc_to_Idq(&Analog_InitStructure);
       PI_Current_pos(&hpi_current,Analog_InitStructure.Idq,Analog_InitStructure.Udq); 
       SVPWM_GENER(Analog_InitStructure.Udq,Analog_InitStructure.elec_angle,&SVPWM_InitStructure,&htim1,prob); 
   }
       ···
 }
```
#### 速度环
![FOCspeed](https://github.com/user-attachments/assets/48241337-687c-4733-8c7d-42d07b2750aa)
<p align="center">图2&emsp;&emsp;FOC速度环控制流程图</p>
速度环要求通过实时检测电机转速，并通过与目标转速比较得到q轴目标电流，作为电流环的输入。因为此项目使用的是永磁电机，d轴电流目标值恒为0即可。
速度环控制代码如下(```main.c```):
```
	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
	{
	      ···

  	if(htim == &htim4)  //1KHz  速度环运算频率
	  {
   	     //对角度进行微分计算速度，内置一阶低通滤波器(滤波阶数还是小一点比较好)
     	       Analog_InitStructure.speed = speed_get(Analog_InitStructure.mech_angle,1000.0,3.1415,0.05);
    	       hpi_current.Iq_target = PI_Speed_pos(&hpi_speed,Analog_InitStructure.speed);
	  }
	      ···
	  }
```
