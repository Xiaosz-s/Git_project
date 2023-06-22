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
#include "i2c_hal.h"
#include "lcd.h"
#include "string.h"
#include "ds18b20_hal.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int
		Change_N,
		Key_presst[4]={0};
float
		Voltage_pa4,
		Voltage_pa5,
		Temp,
		Duty_pa7
		;
char	
		Temp_maxT,
		Compare_X,
		Temp_lastmaxX,
		Compare_lastT,
		Flag_interface=0,
		Key_state=0,
		line[10][21];
uint8_t
		Rx_buff,
		Change_Nl,
		Change_Nh,
		Flag_nixie=0,
		Flag_highlight=2;
	static float Ic_state=0,value1,value2,value3;	
#define SERH HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET)
#define SERL HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET)

#define SCKH HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET)
#define SCKL HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET)

#define RCKH HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET)
#define RCKL HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET)

const uint8_t Nixie_buff[] = {0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F,0x77,0x7c,0x39};

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

float Get_voltage(void)
{
	float value;
	value=HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Start_IT(&hadc2);
	return value*3.3f/4096.0f;
}

void Nixie(uint8_t a,uint8_t b,uint8_t c,uint8_t d)
{
	int value;
	if(d==1)value=(Nixie_buff[c]<<16) | (Nixie_buff[b]<<8) | Nixie_buff[a] | 0x80;
	else if(d==2)value=(Nixie_buff[c]<<16) | (Nixie_buff[b]<<8) | Nixie_buff[a] | 0x80<<8;
	else if(d==3)value=(Nixie_buff[c]<<16) | (Nixie_buff[b]<<8) | Nixie_buff[a] | 0x80<<16;
	else value=(Nixie_buff[c]<<16) | (Nixie_buff[b]<<8) | Nixie_buff[a];
	for(uint8_t i=0;i<24;i++)
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

void Highlight(uint8_t linenum)
{	
	char i=0;
	for(;i<linenum;i++)
	LCD_DisplayStringLine(i*24,(uint8_t *)line[i]);
	LCD_SetBackColor(Yellow);
	LCD_DisplayStringLine(i*24,(uint8_t *)line[i]);
	LCD_SetBackColor(Black);
	for(i=i+1;i<10;i++)
	LCD_DisplayStringLine(i*24,(uint8_t *)line[i]);
}

void Data_interface(void)
{
	sprintf(line[0],"       Main         ");
	sprintf(line[1],"                    ");
	sprintf(line[2],"   AO1:%.1f            ",Voltage_pa4);
	sprintf(line[3],"   AO2:%.1f            ",Voltage_pa5);
	sprintf(line[4],"   PWM2:%d%%           ",(int)(Duty_pa7*100.0f));
	
	sprintf(line[5],"   Temp:%.2f         ",Temp);
	sprintf(line[6],"   N:%d               ",Change_N);
	sprintf(line[7],"%.1f %.1f             ",value1,value2);
	sprintf(line[8],"%.1f %.2f               ",value3,Duty_pa7);
	sprintf(line[9],"                    ");
}

void Para_interface(void)
{
	sprintf(line[0],"       Para         ");
	sprintf(line[1],"                    ");
	sprintf(line[2],"   T:%d              ",Temp_maxT);
	sprintf(line[3],"   X:AO%d            ",Compare_X);
	sprintf(line[4],"                    ");
	
	sprintf(line[5],"                    ");
	sprintf(line[6],"                    ");
	sprintf(line[7],"                    ");
	sprintf(line[8],"                    ");
	sprintf(line[9],"                    ");
}

void Lcd_dis(void)
{
	for(char i=0;i<10;i++)
	LCD_DisplayStringLine(i*24,(uint8_t *)line[i]);
}

uint8_t Key_scan(void)
{
			 if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)==0)Key_presst[0]++;
	else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)==1 && Key_presst[0]>=5){Key_presst[0]=0;return 1;}
	else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)==0)Key_presst[1]++;
	else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)==1 && Key_presst[1]>=5){Key_presst[1]=0;return 2;}
	else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2)==0 && Key_presst[2]>=70){Key_presst[2]++;return 5;}
	else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2)==0)Key_presst[2]++;
	else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2)==1 && Key_presst[2]>=5){Key_presst[2]=0;return 3;}
	else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==0 && Key_presst[3]>=70){Key_presst[3]++;return 6;}
	else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==0)Key_presst[3]++;
	else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==1 && Key_presst[3]>=5){Key_presst[3]=0;return 4;}
	else {Key_presst[0]=0,Key_presst[1]=0,Key_presst[2]=0,Key_presst[3]=0;}
	return 0;
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

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	static int valuehigh,valuelow;
	if(htim==&htim3)
	{
		if(Ic_state==0)
		{
			value1=__HAL_TIM_GET_COMPARE(&htim3,TIM_CHANNEL_2);//xia
			Ic_state=1;
			if(value1>value3)valuehigh=value1-value3;
			else valuehigh=65535+value1-value3;
		}
		else if(Ic_state==1)
		{
			
			value2=__HAL_TIM_GET_COMPARE(&htim3,TIM_CHANNEL_2);//xia
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_2,TIM_TRIGGERPOLARITY_FALLING);
			Duty_pa7=valuehigh*1.0f/((valuehigh+valuelow)*1.0f);
			Ic_state=2;
		}
		else if(Ic_state==2)
		{
			
			value3=__HAL_TIM_GET_COMPARE(&htim3,TIM_CHANNEL_2);//shang
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_2,TIM_TRIGGERPOLARITY_RISING);
			if(value3>value2)valuelow=value3-value2;
			else valuelow=value3-value2+65535;
			Ic_state=0;
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t keyt=0,nixiet=0,uartt=0,adct=0;
	uint8_t TXT[30];
	if(htim==&htim1)
	{
		Key_state=Key_scan();
		if(Key_state==1)Flag_interface=!Flag_interface;
		else if(Key_state==2 && Flag_interface)Flag_highlight=++Flag_highlight>3?2:Flag_highlight;
		else if(Key_state==3 && Flag_interface && Flag_highlight==2)
		{
			Temp_maxT++;
			if(Temp_maxT>40)Temp_maxT=40;
		}
		else if(Key_state==3 && Flag_interface && Flag_highlight==3)Compare_X=++Compare_X>2?1:Compare_X;
		else if(Key_state==4 && Flag_interface && Flag_highlight==2)
		{
			Temp_maxT--;
			if(Temp_maxT<20)Temp_maxT=20;
		}
		else if(Key_state==4 && Flag_interface && Flag_highlight==3)Compare_X=++Compare_X>2?1:Compare_X;
		else if(++keyt>12 && Key_state==5 && Flag_interface && Flag_highlight==2)
		{
			Temp_maxT++;
			if(Temp_maxT>40)Temp_maxT=40;
			keyt=0;
		}
		else if(++keyt>12 && Key_state==6 && Flag_interface && Flag_highlight==2)
		{
			Temp_maxT--;
			if(Temp_maxT<20)Temp_maxT=20;
			keyt=0;
		}
		if(++nixiet>=200)Flag_nixie=!Flag_nixie,nixiet=0;
		
		if(++adct==5)Voltage_pa4=Get_voltage();
		if(adct>=10)Voltage_pa5=Get_voltage(),adct=0;
		
		if(++uartt>=100 && Compare_X==1 && Voltage_pa4>Duty_pa7*3.3f)
		{
			uartt=0;
			sprintf((char *)TXT,"$%.2f\r\n",Temp);
			HAL_UART_Transmit_IT(&huart1,TXT,strlen((char *)TXT));
		}
		else if(++uartt>=100 && Compare_X==2 && Voltage_pa5>Duty_pa7*3.3f)
		{
			uartt=0;
			sprintf((char *)TXT,"$%.2f\r\n",Temp);
			HAL_UART_Transmit_IT(&huart1,TXT,strlen((char *)TXT));
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static uint8_t Rx_state=0,TXT[30];
	if(huart==&huart1)
	{
		HAL_UART_Receive_IT(&huart1,&Rx_buff,1);
		if(Rx_buff=='S' && Rx_state==0)Rx_state=1;
		else if(Rx_buff=='T' && Rx_state==1)Rx_state=2;
		else if(Rx_buff=='\r' && Rx_state==2)Rx_state=3;
		else if(Rx_buff=='\n' && Rx_state==3)
		{
			Rx_state=0;
			sprintf((char *)TXT,"$%.2f\r\n",Temp);
			HAL_UART_Transmit_IT(&huart1,TXT,strlen((char *)TXT));
		}
		else if(Rx_buff=='P' && Rx_state==0)Rx_state=1;
		else if(Rx_buff=='A' && Rx_state==1)Rx_state=2;
		else if(Rx_buff=='R' && Rx_state==2)Rx_state=3;
		else if(Rx_buff=='A' && Rx_state==3)Rx_state=4;
		else if(Rx_buff=='\r' && Rx_state==4)Rx_state=5;
		else if(Rx_buff=='\n' && Rx_state==5)
		{
			Rx_state=0;
			sprintf((char *)TXT,"#%d,AO%d\r\n",Temp_maxT,Compare_X);
			HAL_UART_Transmit_IT(&huart1,TXT,strlen((char *)TXT));
		}
		else Rx_state=0;
//		sprintf((char *)TXT,"state:%d\r\n",Rx_state);
//		HAL_UART_Transmit_IT(&huart1,TXT,strlen((char *)TXT));
//		HAL_UART_Transmit_IT(&huart1,&Rx_buff,1);
		
	}
}



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	ds18b20_init_x();
	I2CInit();
	LCD_Init();
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_UART_Receive_IT(&huart1,&Rx_buff,1);
	Write_rom(0,99);
	HAL_Delay(5);
	Change_N=Read_rom(0);
	HAL_Delay(5);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		if(Flag_nixie)Nixie(12,Temp_maxT/10,Temp_maxT%10,6);
		else Nixie(10,0,Compare_X,6);
		Temp=Read_temp();
		if(Flag_interface)Para_interface(),Highlight(Flag_highlight);
		else Data_interface(),Lcd_dis();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
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
