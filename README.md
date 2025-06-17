# 厚米FOC驱动库
***
### 主要功能：
:star: 无刷电机电流、速度闭环驱动 <p>
:star: 基于VOFA+上位机(串口通信)波形显示与调参 <p>
:star: 可编程按键交互，可用于电机起动、急停、上位机显示内容选择等功能 <p>
### 关于硬件: :point_right: [厚米FOC](https://oshwhub.com/al_user/esc_new) :point_left:
***
### FOC控制框图与代码实现
#### 📖电流环控制框图

![CURRENT](https://github.com/user-attachments/assets/e00fe871-8ecc-4540-a3bd-38b3f34c326b)
<p align="center">图1&emsp;&emsp;FOC电流环控制流程图</p>

:boat: __对各环节的解释：__ <p>
:star:__PI：__    比例、积分控制器，代码中有位置式(_pos后缀)与增量式(_inc后缀)两种控制器，作者更倾向于使用位置式PI控制器 <p>
:star:__SVPWM：__ SVPWM发生器，内部集成反Park变换、反Clark变换、SVPWM运算器、计时器写入器四个模块，输入参数为Udq与电角度 <p>
:star:__ADC：__    模拟量数据采集与初步处理(滤波、归算等步骤) <p>
:star:__IabctoIdq：__ d-q轴坐标电流计算器，内部集成Clark变换与Park变换，输入参数为abc三相电流值与电角度,可直接输出IdIq计算值 <p>
:star:__Position&Speed：__ 角度与速度传感器 <p>

#### 📖代码运行逻辑
:star:a.初始化ADC_DMA连续模式，使得系统可以在无CPU干预的情况下高频(约100KHz)采样电机三相电流 <p>
:star:b.初始化10KHz定时中断，使能电机角度读取函数，并在回调函数里对数据进行初步处理 <p>
:star:c.初始化5KHz定时中断，作为电流环控制频率。在这个中断中进行：<p> ①电流信号归算```Current_transform()``` ②电流信号转换```Iabc_to_Idq()``` ③PI控制器运算```PI_Current_pos()``` ④SVPWM发波```SVPWM_GENER()``` <p>

:boat:__代码路径__:point_right: ```Core/main.c```
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
#### 📖速度环控制框图
![FOCspeed](https://github.com/user-attachments/assets/48241337-687c-4733-8c7d-42d07b2750aa)
<p align="center">图2&emsp;&emsp;FOC速度环控制流程图</p>

#### 📖代码运行逻辑
:star:a.保证电流环正常运行 <p>
:star:b.初始化1KHz定时中断，作为速度环控制频率在这个中断中进行：<p>①根据电角度读取值计算速度```speed_get()``` ② 速度PI控制器运算```PI_Speed_pos()```并将运算结果连接到电流PI控制器输入

:boat:__代码路径__:point_right: ```Core/main.c```

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
