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

#include "dht11.h"
#include "ds18b20_hal.h"
#include "i2c_hal.h"
#include "lcd.h"
#include "string.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

uint8_t 
		line[10][21],
		Flag_interface=0,
		Open_times=0,
		Key_state=0,
		Key_statenz=0,
		Flag_trdo=0,
		Rx_buff,
		TXT[30],
		Flag_dht11read;
float 
		Voltage_pb15,
		Voltage_pa4,
		Voltage_pa5,
		Voltage_pb12,
		Temp_ds,
		Temp_dht,
		Humidity_dht,
		Duty_pa6,
		Duty_pa7;
int
		Frequ_pa15,
		Frequ_pb4,
		Frequ_pa1,
		Frequ_pa2,
		Key_presst[4]={0};
		
uint32_t 
		temp;
static int 
		Tim3_ch1value1st,Tim3_ch1value2nd,Tim3_ch1state=0,Tim3_ch1T,Tim3_ch1hT,
		Tim3_ch2value1st,Tim3_ch2value2nd,Tim3_ch2state=0,Tim3_ch2T,Tim3_ch2hT,
		Tim2_ch1value1st,Tim2_ch1value2nd,Tim2_ch1state=0,Tim2_ch1T,
		Tim2_ch2value1st,Tim2_ch2value2nd,Tim2_ch2state=0,Tim2_ch2T,
		Tim2_ch3value1st,Tim2_ch3value2nd,Tim2_ch3state=0,Tim2_ch3T;
#define SERH HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET)
#define SERL HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET)

#define SCKH HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET)
#define SCKL HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET)

#define RCKH HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET)
#define RCKL HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET)

uint8_t Nixie_num[]={0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F,0x77,0x7C,0x39,0x5E,0x79,0x71};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t Key_scan(void)
{
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)==0 && Key_presst[0]>=70){Key_presst[0]++;return 5;}//长按中
	else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)==0)Key_presst[0]++;//短按中
	else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)==1 && Key_presst[0]>=70){Key_presst[0]=0;return 9;}//长按后松开
	else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)==1 && Key_presst[0]>=5){Key_presst[0]=0;return 1;}//短按松开
	
	else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)==0 && Key_presst[1]>=70){Key_presst[1]++;return 6;}//长按中
	else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)==0)Key_presst[1]++;//短按中
	else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)==1 && Key_presst[1]>=70){Key_presst[1]=0;return 10;}//长按后松开
	else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)==1 && Key_presst[1]>=5){Key_presst[1]=0;return 2;}//短按松开
	
	else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2)==0 && Key_presst[2]>=70){Key_presst[2]++;return 7;}//长按中
	else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2)==0)Key_presst[2]++;//短按中
	else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2)==1 && Key_presst[2]>=70){Key_presst[2]=0;return 11;}//长按后松开
	else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2)==1 && Key_presst[2]>=5){Key_presst[2]=0;return 3;}//短按松开
	
	else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==0 && Key_presst[3]>=70){Key_presst[3]++;return 8;}//长按中
	else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==0)Key_presst[3]++;//短按中
	else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==1 && Key_presst[3]>=70){Key_presst[3]=0;return 12;}//长按后松开
	else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==1 && Key_presst[3]>=5){Key_presst[3]=0;return 4;}//短按松开
	else {Key_presst[0]=0,Key_presst[1]=0,Key_presst[2]=0,Key_presst[3]=0;}
	return 0;
}

void Lcd_dis(void)
{
	for(char i=0;i<10;i++)
	LCD_DisplayStringLine(i*24,line[i]);
}

void Data_interface(void)
{
	sprintf((char *)line[0],"Open_times:%d         ",Open_times);
	sprintf((char *)line[1],"keystate:%d           ",Key_state);
	sprintf((char *)line[2],"Key_statenz:%d        ",Key_statenz);
	sprintf((char *)line[3],"Voltage_pb12:%.2f     ",Voltage_pb12);
	sprintf((char *)line[4],"Voltage_pb15:%.2f     ",Voltage_pb15);
	
	sprintf((char *)line[5],"Voltage_pa4:%.2f  %d  ",Voltage_pa4,HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3));
	sprintf((char *)line[6],"Voltage_pa5:%.2f      ",Voltage_pa5);
	sprintf((char *)line[7],"Duty_pa6:%.2f         ",Duty_pa6);
	sprintf((char *)line[8],"Duty_pa7:%.2f         ",Duty_pa7);
	sprintf((char *)line[9],"%dHz %dHz %dHz        ",Frequ_pa1,Frequ_pa2,Frequ_pa15);
}

void Para_interface(void)
{
	sprintf((char *)line[0],"                    ");
	sprintf((char *)line[1],"                    ");
	sprintf((char *)line[2],"Duty_pa6:%.2f               ",Duty_pa6);
	sprintf((char *)line[3],"Duty_pa7:%.2f               ",Duty_pa7);
	sprintf((char *)line[4],"                    ");
	
	sprintf((char *)line[5],"                    ");
	sprintf((char *)line[6],"                    ");
	sprintf((char *)line[7],"                    ");
	sprintf((char *)line[8],"                    ");
	sprintf((char *)line[9],"                    ");
}

void Highlight_dis(uint8_t linenum)
{
	char i=0;
	for(;i<linenum;i++)
	LCD_DisplayStringLine(i*24,(uint8_t *)"                    ");
	LCD_SetBackColor(Yellow);
	LCD_DisplayStringLine(i*24,(uint8_t *)"                    ");
	LCD_SetBackColor(White);
	for(i++;i<10;i++)
	LCD_DisplayStringLine(i*24,(uint8_t *)"                    ");
}

void Write_rom(uint8_t add,uint8_t value)
{
	I2CStart();
	I2CSendByte(0xa0);
	I2CWaitAck();
	
	I2CSendByte(add);
	I2CWaitAck();
	
	I2CSendByte(value);
	I2CWaitAck();
	I2CStop();
}

uint8_t Read_rom(uint8_t add)
{
	uint8_t value;
	
	I2CStart();
	I2CSendByte(0xa0);
	I2CWaitAck();
	
	I2CSendByte(add);
	I2CWaitAck();
	
	I2CStart();
	I2CSendByte(0xa1);
	I2CWaitAck();
	
	value=I2CReceiveByte();
	I2CWaitAck();
	I2CStop();
	
	return value;
}

void Nixie_dis(uint8_t a,uint8_t b,uint8_t c)
{
	int value=(Nixie_num[c]<<16)+(Nixie_num[b]<<8)+(Nixie_num[a]);
	for(char i=0;i<24;i++)
	{
		if(value & 0x800000)
			SERH;
		else 
			SERL;
		SCKH;
		SCKL;
		value<<=1;
	}
	RCKH;
	RCKL;
}

uint32_t Get_adc1(void)
{
	float value;
	value=HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Start_IT(&hadc1);
	return value;
}

uint32_t Get_adc2(void)
{
	float value;
	value=HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Start_IT(&hadc2);
	return value;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	
	if(htim==&htim2 && htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1)
	{
		if(Tim2_ch1state==0)Tim2_ch1value1st=HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_1),Tim2_ch1state=1;
		else if(Tim2_ch1state==1)
		{
			Tim2_ch1value2nd=HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_1);
			if(Tim2_ch1value2nd>Tim2_ch1value1st)Tim2_ch1T=Tim2_ch1value2nd-Tim2_ch1value1st;
			else Tim2_ch1T=Tim2_ch1value2nd-Tim2_ch1value1st+65535;
			Frequ_pa15=1000000/Tim2_ch1T;
			Tim2_ch1state=0;
		}
	}
	if(htim==&htim2 && htim->Channel==HAL_TIM_ACTIVE_CHANNEL_2)
	{
		if(Tim2_ch2state==0)Tim2_ch2value1st=HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_2),Tim2_ch2state=1;
		else if(Tim2_ch2state==1)
		{
			Tim2_ch2value2nd=HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_2);
			if(Tim2_ch2value2nd>Tim2_ch2value1st)Tim2_ch2T=Tim2_ch2value2nd-Tim2_ch2value1st;
			else Tim2_ch2T=Tim2_ch2value2nd-Tim2_ch2value1st+65535;
			Frequ_pa1=1000000/Tim2_ch2T;
			Tim2_ch2state=0;
		}
	}
	if(htim==&htim2 && htim->Channel==HAL_TIM_ACTIVE_CHANNEL_3)
	{
		if(Tim2_ch3state==0)Tim2_ch3value1st=HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_3),Tim2_ch3state=1;
		else if(Tim2_ch3state==1)
		{
			Tim2_ch3value2nd=HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_3);
			if(Tim2_ch3value2nd>Tim2_ch3value1st)Tim2_ch3T=Tim2_ch3value2nd-Tim2_ch3value1st;
			else Tim2_ch3T=Tim2_ch3value2nd-Tim2_ch3value1st+65535;
			Frequ_pa2=1000000/Tim2_ch3T;
			Tim2_ch3state=0;
		}
	}
	if(htim==&htim3 && htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1)
	{
		if(Tim3_ch1state==0)
		{
			Tim3_ch1value1st=HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_1);//shang
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_FALLING);
			Tim3_ch1state=1;
		}
		else if(Tim3_ch1state==1)
		{
			Tim3_ch1value2nd=HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_1);//xia
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_RISING);
			if(Tim3_ch1value2nd>Tim3_ch1value1st)Tim3_ch1hT=Tim3_ch1value2nd-Tim3_ch1value1st;
			else Tim3_ch1hT=Tim3_ch1value2nd-Tim3_ch1value1st+65535;
			Tim3_ch1state=2;
		}
		else if(Tim3_ch1state==2)
		{
			Tim3_ch1value1st=HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_1);//shang
			if(Tim3_ch1value2nd<Tim3_ch1value1st)Tim3_ch1T=Tim3_ch1value1st-Tim3_ch1value2nd;
			else Tim3_ch1T=Tim3_ch1value1st+65535-Tim3_ch1value2nd;
			Duty_pa6=(float)Tim3_ch1hT/((float)Tim3_ch1T+(float)Tim3_ch1hT);
			Tim3_ch1state=0;
		}
	}
	if(htim==&htim3 && htim->Channel==HAL_TIM_ACTIVE_CHANNEL_2)
	{
		if(Tim3_ch2state==0)
		{
			Tim3_ch2value1st=HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_2);//shang
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_2,TIM_INPUTCHANNELPOLARITY_FALLING);
			Tim3_ch2state=1;
		}
		else if(Tim3_ch2state==1)
		{
			Tim3_ch2value2nd=HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_2);//xia
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_2,TIM_INPUTCHANNELPOLARITY_RISING);
			if(Tim3_ch2value2nd>Tim3_ch2value1st)Tim3_ch2hT=Tim3_ch2value2nd-Tim3_ch2value1st;
			else Tim3_ch2hT=Tim3_ch2value2nd-Tim3_ch2value1st+65535;
			Tim3_ch2state=2;
		}
		else if(Tim3_ch2state==2)
		{
			Tim3_ch2value1st=HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_2);//shang
			if(Tim3_ch2value2nd<Tim3_ch2value1st)Tim3_ch2T=Tim3_ch2value1st-Tim3_ch2value2nd;
			else Tim3_ch2T=Tim3_ch2value1st+65535-Tim3_ch2value2nd;
			Duty_pa7=(float)Tim3_ch2hT/((float)Tim3_ch2T+(float)Tim3_ch2hT);
			Tim3_ch2state=0;
		}
	}
}


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart==&huart1)
	{
		HAL_UART_Receive_IT(&huart1,&Rx_buff,1);
		HAL_UART_Transmit_IT(&huart1,&Rx_buff,1);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t adct=0;
	if(htim==&htim1)
	{
		Key_state=Key_scan();
		if(Key_state)
		Key_statenz=Key_state;
		if(++adct==5)Voltage_pa4=Get_adc2(),Voltage_pb12=Get_adc1();
		if(adct==10)Voltage_pb15=Get_adc2();
		if(adct==15)Voltage_pa5=Get_adc2(),adct=0;
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
//	ds18b20_init_x();
//	dht11_init();
	I2CInit();
	HAL_UART_Receive_IT(&huart1,&Rx_buff,1);
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2);
//	HAL_TIM_Base_Start_IT(&htim1);
	LCD_Init();
	LCD_SetBackColor(Blue);
	LCD_SetTextColor(White);
	Open_times=Read_rom(0);
	HAL_Delay(5);
	Write_rom(0,++Open_times);
	HAL_Delay(5);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		Data_interface();
		Lcd_dis();
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
