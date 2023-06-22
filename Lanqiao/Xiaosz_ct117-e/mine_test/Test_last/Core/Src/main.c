/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "lcd.h"
#include "string.h"
#include "i2c_hal.h"
#include "dht11_hal.h"
#include "ds18b20_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

int
	Key_presst[4]={0},
	Key_voltage=0,
	Dht11_re;
uint8_t
	Flag_led1state=0,
	temp,
	humidity,
	line[10][21],
	Rx_buffer,
	Key_state=0,
	Nixie_num[]={0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F,0x77,0x7C,0x39,0x5E,0x79,0x71};
float
	Voltage_pa4,
	Voltage_pa5,
	Duty_pa6;

#define SERH HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET)
#define SERL HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET)

#define SCKH HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET)
#define SCKL HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET)

#define RCKH HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET)
#define RCKL HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET)

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int Get_adc2(void)
{
	int value;
	value=HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Start_IT(&hadc2);
	return value;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	static int Tim3_ch1value1,Tim3_ch1value2,Tim3_ch1valueH,Tim3_ch1valueL,Tim3_ch1state=0;
	if(htim==&htim3)
	{
		if(Tim3_ch1state==0)
		{
			Tim3_ch1value1=HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_1);
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_FALLING);
			Tim3_ch1state=1;
		}
		else if(Tim3_ch1state==1)
		{
			Tim3_ch1value2=HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_1);
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_RISING);
			if(Tim3_ch1value2>Tim3_ch1value1)Tim3_ch1valueH=Tim3_ch1value2-Tim3_ch1value1;
			else Tim3_ch1valueH=Tim3_ch1value2-Tim3_ch1value1+65535;
			Tim3_ch1state=2;
		}
		else if(Tim3_ch1state==2)
		{
			Tim3_ch1value1=HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_1);
			if(Tim3_ch1value2<Tim3_ch1value1)Tim3_ch1valueL=Tim3_ch1value1-Tim3_ch1value2;
			else Tim3_ch1valueL=Tim3_ch1value1+65535-Tim3_ch1value2;
			Duty_pa6=Tim3_ch1valueH*1.0f/((Tim3_ch1valueH+Tim3_ch1valueL)*1.0f);
			Tim3_ch1state=0;
		}
	}
}

void Nixie_dis(uint8_t a,uint8_t b,uint8_t c)
{
	uint32_t value=(Nixie_num[c]<<16) | (Nixie_num[b]<<8) | Nixie_num[a];
	for(char i=0;i<24;i++)
	{
		if(value &0x800000)SERH;
		else SERL;
		SCKH;
		SCKL;
		value<<=1;
	}
	RCKH;
	RCKL;
}

void Led_control(uint8_t num, uint8_t state)
{
	HAL_GPIO_WritePin(GPIOC,0x80<<num,state?GPIO_PIN_SET:GPIO_PIN_RESET);
}

void Led_proc(void)
{
	Led_control(1,Flag_led1state);
	for(char i=2;i<9;i++)
	Led_control(i,1);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t adct=0,ledt=0;
	if(htim==&htim1)
	{
		if(++adct==5)Voltage_pa4=Get_adc2();
		if(adct==10)Voltage_pa5=Get_adc2(),adct=0;
		if(++ledt==10)ledt=0,Flag_led1state=!Flag_led1state;
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
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);
	I2CInit();
	LCD_Init();
	LCD_SetBackColor(Blue);
	LCD_SetTextColor(White);
	Nixie_dis(0,8,15);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		Nixie_dis(0,8,15);
		Dht11_re=Dht11_read();
		temp=Dht11_re>>24;
		humidity=(Dht11_re>>8) & 0xff;
		sprintf((char *)line[0],"h:%d w:%d %.2f",humidity,temp,Duty_pa6);
		LCD_DisplayStringLine(0,line[0]);
		sprintf((char *)line[1],"%d   %.2f",(int)Voltage_pa4,Voltage_pa5*1.0f/4096.0f*3.3f);
		LCD_DisplayStringLine(24,line[1]);
		Led_proc();
//		HAL_Delay(300);
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
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
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
