/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define a 0.7
#define middle angle 170
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int16_t moter_speed = 0;
uint8_t num = 0;
uint16_t ADC_value = 0;
uint16_t angle = 0;
//float target,actual,out;
//float kp =0.3,ki=0.15,kd=0.1;
//位置式PID
//float error0,error1,errorint;
//增量式PID
//float error0,error1,error2;
//双环串级PID
float interror1,interror0,interrorint,inttarget,intactual,intout;
float intkp=0.4,intki=0.05,intkd=0;
float exterror1,exterror0,exterrorint,exttarget,extactual,extout;
float extkp=0.2,extki=0,extkd=0;
int8_t duty;
int16_t location = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//电机控制
void motor_set_pwm(int8_t PWM)//范围0-99
{
	if(PWM >= 0)
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,PWM);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,-PWM);
	}
}

//串口回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		// 将ASCII数字字符转为数值：'0'→0, '1'→1, ..., '9'→9
		duty = num - '0';
		HAL_UART_Receive_IT(&huart1, &num,1);

	}
}

//printf重定向
int fputc(int ch, FILE *f) {
HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
return ch;
}

//定时器回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		if(htim ==&htim3){ 
//编码器代码
//			int16_t data = 0;
			moter_speed = __HAL_TIM_GET_COUNTER(&htim1);
//			data = (1-a)*moter_speed + a*data;
				__HAL_TIM_SET_COUNTER(&htim1,0); 
			
//位置式PID
//			target = duty*11;
//			actual = moter_speed;
//			error1 = error0;
//			error0 = target - actual;
//			errorint += error0;
//			out = kp*error0 + ki*errorint + kd*(error0-error1);
//			if(out > 99){	out = 99;}
//			if(out < -99){out = -99;}
////			if(0< out<3){out = 3;}
////			if(-3<out<0){out = -3;}
//			motor_set_pwm(out);
//增量式PID
//			target = duty*11.6;
//			actual = data;
//			error2 = error1;
//			error1 = error0;
//			error0 = target - actual;
//			out += kp*(error0-error1) + ki*error0 + kd*(error0-2*error1+error2);
//			if(out > 99){	out = 99;}
//			if(out < -99){out = -99;}
//			motor_set_pwm(out);		
//增量式定位置			
//			target = duty*11;
//			actual += data;
//			error2 = error1;
//			error1 = error0;
//			error0 = target - actual;
//			out += kp*(error0-error1) + ki*error0 + kd*(error0-2*error1+error2);
//			if(out > 99){	out = 99;}
//			if(out < -99){out = -99;}
//			motor_set_pwm(out);		

//位置式定位置,积分分离
//			target = duty*11;
//			actual += data;
//			error1 = error0;
//			error0 = target - actual;
//			if(fabs(error0)<50)
//			{errorint += error0;}
//			else
//			{errorint = 0;}
//				
//			out = kp*error0 + ki*errorint + kd*(error0-error1);
//			if(out > 99){	out = 99;}
//			if(out < -99){out = -99;}
//			motor_set_pwm(out);
//双环串级PID
//外环位置环
			exttarget = 0;
			extactual += moter_speed;
			exterror1 = exterror0;
			exterror0 = exttarget - extactual;
			exterrorint += exterror0;
			extout = extkp*exterror0+extki*exterrorint+extkd*(exterror0-exterror1);
			if(extout > 110){extout = 110;}
			if(extout < -110){extout = -110;}
//内环速度环
			inttarget = extout;
			intactual = moter_speed;
			interror1 = interror0;
			interror0 = inttarget - intactual;
			interrorint +=interror0;
			intout = intkp*interror0 + intki*interrorint + intkd*(interror0-interror1);
			if(intout > 90){intout = 90;}
			if(intout < -90){intout = -90;}
			motor_set_pwm(intout);
//串口发送数据
			printf("%d\n",angle);
//			printf("%f,%f,%f\n",target,actual,out);
		
//角度传感器
			angle = ((uint32_t)ADC_value * 360) / 4095;
// 			printf("%d\n",angle);
		}
			
}
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	//启动ADC自动校准程序
	HAL_ADCEx_Calibration_Start(&hadc1);
// 启动ADC
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&ADC_value,1);
//开启编码器
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
//开启PWM
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
//开启串口中断接收
HAL_UART_Receive_IT(&huart1, &num,1);
//开启定时器3的中断
HAL_TIM_Base_Start_IT(&htim3);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(duty == 0)
		{
			
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
#ifdef USE_FULL_ASSERT
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
