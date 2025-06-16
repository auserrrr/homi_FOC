/**
 * @file serial.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-06-04
 * 
 * @copyright Copyright (c) 2025
 * 
 * @note 基于串口通讯的调试UI
 */
#include "serial.h"

/**
 * @brief 数据打印与切换
 * @param huart 串口句柄
 * @param hpi_speed 速度PI控制器句柄
 * @param prob 三相SVPWM占空比探针
 * @param Analog_InitStructure 模拟量初始化结构体
 * @param sig 信号标志位
 * 
 */
void Data_print(UART_HandleTypeDef *huart,SpeedPITypeDef hpi_speed,float *prob,AnalogTypeDef Analog_InitStructure,int8_t *sig)
{
    static char DataSerial[80] = " ";
    //static char Note[125] = "\r\nCommand Format:char[4] = \"abcd\"\r\na(0~4):0Kpd, 1Kid, 2Kpq,3Kiq, 4Iq_target\r\nbcd(0~9):Para += 0.000001*(10^b)*((-1)^c)*d\r\n";
    static char Note[120] = "\r\nCommand Format:char[4] = \"abcd\"\r\na(0~2):0Kp, 1Ki, 2Speed_target\r\nbcd(0~9):Para += 0.000001*(10^b)*((-1)^c)*d\r\n";


    switch (*sig)
        {
   
          case 0:    //errsum
          
         // sprintf(DataSerial,"%.2f,%.2f,%.2f,%.2f\r\n",hpi_current.err_sum_d,hpi_current.err_sum_q,hpi_current.err_d,hpi_current.err_q);
         sprintf(DataSerial,"%.2f,%.2f\r\n",hpi_speed.speed_err,hpi_speed.speed_err_sum);
         HAL_UART_Transmit_IT(huart,(uint8_t*)DataSerial,strlen(DataSerial));
          
          break;

          case 1:   //三相SVPWM占空比波形

          sprintf(DataSerial,"%.2f,%.2f,%.2f\r\n",prob[0],prob[1],prob[2]);
          HAL_UART_Transmit_IT(huart,(uint8_t*)DataSerial,strlen(DataSerial));

          break;
   
          case 2:  //三相电流采样值

          sprintf(DataSerial,"%.2f,%.2f,%.2f\r\n",Analog_InitStructure.Current_Value[0],Analog_InitStructure.Current_Value[1],Analog_InitStructure.Current_Value[2]);
          HAL_UART_Transmit_IT(huart,(uint8_t*)DataSerial,strlen(DataSerial));
   
          break;
    
          case 3: //PI控制器输入Idq

          sprintf(DataSerial,"%.2f,%.2f\r\n",Analog_InitStructure.Idq[0],Analog_InitStructure.Idq[1]);
          HAL_UART_Transmit_IT(huart,(uint8_t*)DataSerial,strlen(DataSerial));

          break;

          case 4://PI控制器输出Udq
        
          sprintf(DataSerial,"%.2f,%.2f\r\n",Analog_InitStructure.Udq[0],Analog_InitStructure.Udq[1]);
          HAL_UART_Transmit_IT(huart,(uint8_t*)DataSerial,strlen(DataSerial));

          break;

          case 5://角度与速度
          
          sprintf(DataSerial,"%.2f,%.2f,%.2f\r\n",Analog_InitStructure.speed,Analog_InitStructure.mech_angle,Analog_InitStructure.elec_angle);
          HAL_UART_Transmit_IT(huart,(uint8_t*)DataSerial,strlen(DataSerial));

          break;

          case 6://当前参数

          *sig = -1;
          sprintf(DataSerial,"\r\nKp:%.6f,Ki:%.6f,Speed_Target:%.6f\r\n",hpi_speed.Kp,hpi_speed.Ki,hpi_speed.speed_target);
          HAL_UART_Transmit_IT(huart,(uint8_t*)DataSerial,strlen(DataSerial));

          break;

          case 7://提示信息

          *sig = -1;
          HAL_UART_Transmit_IT(huart,(uint8_t*)Note,strlen(Note));

          break;

          default:
          break;
}
}

