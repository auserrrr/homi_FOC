/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @details
  * 
  *   -  工程名称：FOC_
  *   -  功能描述：基于STM32G431与HAL库的无刷电机有感FOC控制
  *   -  作者   ：王灏(qq:3265809330)
  *   -  日期   ：2025-06-04
  *   -  版本   ：v1.0
  *
  * @note 
  *   -  简介   ：为 "厚米FOC" 无刷电机控制板开发的FOC控制程序,板卡购买请联系作者QQ
  *   -  主要传感器或控制器：INA199、AS5048A、FD6288Q、TLV9062、TPS56302等
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "test.h"
#include "Signalprocess.h"
#include "controller.h"
#include "svpwm.h"
#include "serial.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

int8_t   d1 = -1;    
uint16_t d0 =  0;
AnalogTypeDef Analog_InitStructure;
SVPWMTypeDef SVPWM_InitStructure;
CurrentPITypeDef hpi_current;
SpeedPITypeDef hpi_speed;
AngelPITypeDef hpi_angel;
uint8_t rx_Data[4] = {0,0,0,0};
float prob[3] = {0,0,0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  if(htim == &htim3)  //10KHz  角度刷新频率
	{
    d0 ++;  
    AS5048A_Init(&hspi1,&Analog_InitStructure.AS5048A_get);
   
	}

  if(htim == &htim4)  //1KHz  速度环运算频率
	{
    //对角度进行微分计算速度，内置一阶低通滤波器(滤波阶数还是小一点比较好)
     Analog_InitStructure.speed = speed_get(Analog_InitStructure.mech_angle,1000.0,3.1415,0.05);
     hpi_current.Iq_target = PI_Speed_pos(&hpi_speed,Analog_InitStructure.speed);
	}

  if(htim == &htim15) //200Hz 角度环运算频率
  {
    //hpi_speed.speed_target = PI_Angel_pos(&hpi_angel,Analog_InitStructure.mech_angle);
  }
}

//ADC_DMA传递完成回调函数 
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc == &hadc1)
  {
    //刷新时间最大约10us  100KHz
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == &hspi1)
  {
    
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);    //CS引脚上拉,对数据处理期间停止读取角度
    //机械角度,3.762为编码器零点与电机零点之间的偏移角,估算值非精确值
    Analog_InitStructure.mech_angle = absolute_angle(Analog_InitStructure.AS5048A_get) + 3.762f;
    //电流闭环使用电角度,电角度 = 极对数 * 机械角度
    Analog_InitStructure.elec_angle = POLEPAIR * Analog_InitStructure.mech_angle;
   
 
  }
}

void HAL_UART_TxCpltCallback (UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
   
  
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   if (huart == &huart1)
   {
    
     Speed_RX_Analysis((uint8_t*)rx_Data,&hpi_speed);     //串口指令解析,速度环调参
     HAL_UART_Transmit_IT(&huart1,(uint8_t *)rx_Data,4);  //指令回显
     HAL_UART_Receive_IT(&huart1,(uint8_t *)rx_Data,4);   //手动使能串口接收中断

   }
}
//外部中断
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == Key1_Pin)  //Key1作用:开启电机总线电压;使能电机电流环,角度读取,速度环,角度环;通过标志位d1使能不同的打印任务
  {
    
    LEDA_set(GPIO_PIN_RESET);     //蓝灯OFF
    LEDB_set(GPIO_PIN_SET);       //黄灯ON
    CTRLA_VBUS(VBUS_ON);          //电机总线电压开启

   HAL_TIM_Base_Start_IT(&htim2);        //TIM2定时中断开启,5KHz,电流环
   HAL_TIM_Base_Start_IT(&htim3);        //TIM3定时中断开启,10KHz,角度刷新
   HAL_TIM_Base_Start_IT(&htim4);        //TIM4定时中断开启,1KHz,速度环
   HAL_TIM_Base_Start_IT(&htim15);       //TIM15定时中断开启200Hz,角度环
    d1 ++;
    d1 %= 6;
  }
  
  else if (GPIO_Pin == Key2_Pin)  //Key2作用:直接关闭电机总线电压;关闭电机电流环,速度环,角度环;通过标志位d1执行特定的打印任务
  {
    LEDA_set(GPIO_PIN_SET);              //蓝灯ON
    LEDB_set(GPIO_PIN_RESET);            //黄灯OFF
    CTRLA_VBUS(VBUS_OFF);                //电机总线电压关闭
    HAL_TIM_Base_Stop_IT(&htim2);        //TIM2定时中断关闭
    HAL_TIM_Base_Stop_IT(&htim4);        //TIM4定时中断关闭
    HAL_TIM_Base_Stop_IT(&htim15);       //TIM15定时中断关闭
    d1 = 6;

  }

  else if (GPIO_Pin == Key3_Pin)  //BOOT0(PB8)引脚复用按键,通过引脚d1执行特定的打印任务
  {
   d1 = 7;
  }
}
/**********************************************************************************/

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */
    
  AS5048A_Init(&hspi1,&Analog_InitStructure.AS5048A_get);    //编码器初始化
  Analog_Init(&Analog_InitStructure);                        //模拟量初始化

  SVPWM_Init(&SVPWM_InitStructure);     //SVPWM参数初始化
  Current_PI_Init(&hpi_current);        //电流环参数初始化
  Speed_PI_Init(&hpi_speed);            //速度环参数初始化
 // Angel_PI_Init(&hpi_angel);            //角度环参数初始化

  ADC_Init(&hadc1,Analog_InitStructure.ADC_get);     //ADC校准&DMA通道连续转换模式
  PWM_Enable(&htim1);                                //TIM1 PWM模式初始化
  HAL_UART_Receive_IT(&huart1,(uint8_t *)rx_Data,4); //串口接收中断使能,用于动态调参
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (d0 == 100)      //串口打印，周期为ms级，放到主循环
     {    
       d0 = 0;
       Data_print(&huart1,hpi_speed,prob,Analog_InitStructure,&d1);
      }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
