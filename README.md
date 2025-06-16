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
&emsp;&emsp;其中"Iabc to Idq"环节是Clark变换与Pack变换的结合，输入参数为经过ADC采集的三相电机电流：Ia，Ib，Ic；输出参数为经过Clark变换(Ia,Ib,Ic投影到α-β轴)与Park变换(α-β轴投影到d-q轴)的结果Id与Iq。再经过PI控制器得到Ud与Uq的值，进入SVPWM发生器(内置反Park变换)生成对电机的控制电压。

&emsp;&emsp;整个流程通过直接输入Id与Iq的目标值直接控制电机电流，因此称之为电流环。

代码路径：UserCode/controller.c


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
&emsp;&emsp;在对电流环控制之前，需要初始化电流采集、转换两个阶段，电流采集通过初始化ADC DMA连续转换模式自动刷新；电流转换通过函数```Current_transform()```与函数```Iabc_to_Idq()```完成。电流采集频率约为100KHz，角度采集频率为10KHz，电流转换与PI控制器频率同频，通过定时中断控制为5KHz。代码如下：
```
//定时中断回调函数      
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim2)    //5KHZ(200us) FOC力矩环(电流环)控制频率
	{
   Current_transform(&Analog_InitStructure,1.859,0.03);  
   //prop1:电流采集归算系数，由采样电阻,电流互感器增益,ADC参考电源等参数计算,将电流单位归算为 mA
   //prop2:电流采集低通滤波系数,数值越小,电流波形越平滑
    //约7.9us
    Iabc_to_Idq(&Analog_InitStructure);
    PI_Current_pos(&hpi_current,Analog_InitStructure.Idq,Analog_InitStructure.Udq);    //(Id_ref,Iq_ref) + (Id,Iq) ---(PI)--->Ud,Uq
    //代码执行时间为5.9us
    SVPWM_GENER(Analog_InitStructure.Udq,Analog_InitStructure.elec_angle,&SVPWM_InitStructure,&htim1,prob); //inv_Park
  }
```
#### 速度环
![FOCspeed](https://github.com/user-attachments/assets/48241337-687c-4733-8c7d-42d07b2750aa)
<p align="center">图2&emsp;&emsp;FOC速度环控制流程图</p>
