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
#include "lcd.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t line[10][21],	//LCD存字符
				Rx_buff//串口所接收字符
				;
char Trigger_mode='A',  //触发模式
		 Flag_interface=0,//LCD页面显示标志位
		 Key_state,//按键状态
		 Flag_ledproc;//LED控制标志位 
		
float Angle_a,//角度
			Angle_b,//角度
			Voltage_pa4,//PA4所采集电压
			Angle_aa[5]={0},//角度a的近五次未排序角度
			Angle_bb[5]={0},//角度a的近五次未排序角度
			Angle_qa[5]={0},//角度a的近五次已排序角度
			Angle_qb[5]={0};//角度a的近五次已排序角度
int Key_presst[4]={0},//按键按下计数
		Angle_Pax=20,//角度变化量参数
		Angle_Pbx=20,//角度变化量参数
		Angle_ax,//角度变化量
		Angle_bx,//角度变化量
		Frequ_Pf=1000;//频率参数
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

float Voltage_gethadc2(void)
{
	float Value;
	Value=HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Start_IT(&hadc2);
	return Value*3.3f/4096;
}


uint16_t TIM3_IC1_Value1,TIM3_IC1_Value2;
uint16_t TIM3_IC1_High,TIM3_IC1_Low;
float TIM3_IC1_Duty;
uint8_t TIM3_IC1_Number;

uint16_t TIM3_IC2_Value1,TIM3_IC2_Value2;
uint16_t TIM3_IC2_High,TIM3_IC2_Low;
float TIM3_IC2_Duty;
uint8_t TIM3_IC2_Number;

uint32_t TIM2_IC2_Value1,TIM2_IC2_Value2;
uint32_t TIM2_IC2_Fre;
uint8_t TIM2_IC2_Number;
//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
//{
//	if(htim==&htim3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
//	{
//		Frequ_pa6=__HAL_TIM_GetCounter(&htim3);
//		__HAL_TIM_SetCounter(&htim3,0);
//		HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);
//		Frequ_pa6=1000000/Frequ_pa6;
//	}
//	
//	if(htim==&htim3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
//	{
//		Frequ_pa7=__HAL_TIM_GetCounter(&htim3);
//		__HAL_TIM_SetCounter(&htim3,0);
//		HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2);
//		Frequ_pa7=1000000/Frequ_pa7;
//	}
//}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM2)
    {
        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
        {
            if(TIM2_IC2_Number == 0)
            {
                TIM2_IC2_Value1 = __HAL_TIM_GET_COMPARE(htim,TIM_CHANNEL_2);
                TIM2_IC2_Number = 1;
            }
            else if(TIM2_IC2_Number == 1)
            {
                TIM2_IC2_Value2 = __HAL_TIM_GET_COMPARE(htim,TIM_CHANNEL_2);
                if(TIM2_IC2_Value1 > TIM2_IC2_Value2)
                    TIM2_IC2_Fre = 1000000 / ((0xFFFF - TIM2_IC2_Value1) + TIM2_IC2_Value2);
                else
                    TIM2_IC2_Fre = 1000000 / (TIM2_IC2_Value2 - TIM2_IC2_Value1);
                TIM2_IC2_Number = 0;
            }
        }
    }
    if(htim->Instance == TIM3)
    {
        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
        {
            if(TIM3_IC1_Number == 0)
            {
                TIM3_IC1_Value1 = __HAL_TIM_GET_COMPARE(htim,TIM_CHANNEL_1);
                __HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_FALLING);
                TIM3_IC1_Number = 1;
            }
            else if(TIM3_IC1_Number == 1)
            {
                TIM3_IC1_Value2 = __HAL_TIM_GET_COMPARE(htim,TIM_CHANNEL_1);
                __HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_RISING);
                if(TIM3_IC1_Value1 > TIM3_IC1_Value2)
                    TIM3_IC1_High = (0xFFFF - TIM3_IC1_Value1) + TIM3_IC1_Value2;
                else
                    TIM3_IC1_High = TIM3_IC1_Value2 - TIM3_IC1_Value1;
                TIM3_IC1_Value1 = TIM3_IC1_Value2;
                TIM3_IC1_Number = 2;
            }
            else if(TIM3_IC1_Number == 2)
            {
                TIM3_IC1_Value2 = __HAL_TIM_GET_COMPARE(htim,TIM_CHANNEL_1);
                if(TIM3_IC1_Value1 > TIM3_IC1_Value2)
                    TIM3_IC1_Low = (0xFFFF - TIM3_IC1_Value1) + TIM3_IC1_Value2;
                else
                    TIM3_IC1_Low = TIM3_IC1_Value2 - TIM3_IC1_Value1;
                TIM3_IC1_Duty = TIM3_IC1_High * 1.0 / (TIM3_IC1_High + TIM3_IC1_Low);
                TIM3_IC1_Number = 0;
            }
        }
        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
        {
            if(TIM3_IC2_Number == 0)
            {
                TIM3_IC2_Value1 = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2);
                __HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_2,TIM_INPUTCHANNELPOLARITY_FALLING);
                TIM3_IC2_Number = 1;
            }
            else if(TIM3_IC2_Number == 1)
            {
                TIM3_IC2_Value2 = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2);
                __HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_2,TIM_INPUTCHANNELPOLARITY_RISING);
							
                if(TIM3_IC2_Value1 > TIM3_IC2_Value2)
                    TIM3_IC2_High = (0xFFFF - TIM3_IC2_Value1) + TIM3_IC2_Value2;
                else
                    TIM3_IC2_High = TIM3_IC2_Value2 - TIM3_IC2_Value1;
                TIM3_IC2_Value1 = TIM3_IC2_Value2;
                TIM3_IC2_Number = 2;
            }
            else if(TIM3_IC2_Number == 2)
            {
                TIM3_IC2_Value2 = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2);
							
                if(TIM3_IC2_Value1 > TIM3_IC2_Value2)
                    TIM3_IC2_Low = (0xFFFF - TIM3_IC2_Value1) + TIM3_IC2_Value2;
                else
                    TIM3_IC2_Low = TIM3_IC2_Value2 - TIM3_IC2_Value1;
                TIM3_IC2_Duty = TIM3_IC2_High * 1.0 / (TIM3_IC2_High + TIM3_IC2_Low);
                TIM3_IC2_Number = 0;
            }
        }
    }
}



//void Lcd_print(void)
//{
//	sprintf((char *)line[0],"     Voltage:%.2f     ",Voltage_gethadc2());
//	sprintf((char *)line[1]," Frequ_pa6:%dHz      ",Frequ_pa6);
//	sprintf((char *)line[2]," Frequ_pa7:%dHz      ",Frequ_pa7);
//	sprintf((char *)line[3],"TIM3_CH1_DUTY:%.2f   ",TIM3_IC1_Duty);
//	sprintf((char *)line[4],"TIM3IC1Value1:%d ",TIM3_IC1_Value1);
//	
//	sprintf((char *)line[5],"TIM3_IC1_High:%d    ",TIM3_IC1_High);
//	sprintf((char *)line[6],"TIM3_IC1_Low:%d     ",TIM3_IC1_Low);
//	sprintf((char *)line[7],"TIM3_CH2_DUTY:%.2f   ",TIM3_IC2_Duty);
//	sprintf((char *)line[8],"TIM3IC1Value2:%d   ",TIM3_IC1_Value2);
//	sprintf((char *)line[9],"                    ");
//}

void Data_interface(void)
{
	sprintf((char *)line[0],"                    ");
	sprintf((char *)line[1],"        DATA        ");
	sprintf((char *)line[2],"   a:%.1f    %.2f    ",Angle_a,TIM3_IC1_Duty);
	sprintf((char *)line[3],"   b:%.1f    %.2f    ",Angle_b,TIM3_IC2_Duty);
	sprintf((char *)line[4],"   f:%dHz            ",TIM2_IC2_Fre);
	
	sprintf((char *)line[5],"                    ");
	sprintf((char *)line[6],"   ax:%d             ",Angle_ax);
	sprintf((char *)line[7],"   bx:%d             ",Angle_bx);
	sprintf((char *)line[8],"   mode:%c           ",Trigger_mode);
	sprintf((char *)line[9],"                    ");
}

void Parameter_interface(void)
{
	sprintf((char *)line[0],"                    ");
	sprintf((char *)line[1],"        PARA        ");
	sprintf((char *)line[2],"   Pax:%d            ",Angle_Pax);
	sprintf((char *)line[3],"   Pbx:%d            ",Angle_Pbx);
	sprintf((char *)line[4],"   Pf:%d             ",Frequ_Pf);
	
	sprintf((char *)line[5],"                    ");
	sprintf((char *)line[6],"                    ");
	sprintf((char *)line[7],"                    ");
	sprintf((char *)line[8],"                    ");
	sprintf((char *)line[9],"                    ");
}

void Lcd_dis(void)
{
	for(int i=0;i<10;i++)
	LCD_DisplayStringLine(i*24,line[i]);
//	LCD_DisplayStringLine(0,line[3]);
}

char Key_scan(void)
{
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)==0)Key_presst[0]++;
	else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)==1 && Key_presst[0]>=5){Key_presst[0]=0;return 1;}
	else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)==0)Key_presst[1]++;
	else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)==1 && Key_presst[1]>=5){Key_presst[1]=0;return 2;}
	else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2)==0)Key_presst[2]++;
	else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2)==1 && Key_presst[2]>=5){Key_presst[2]=0;return 3;}
	else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==0)Key_presst[3]++;
	else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==1 && Key_presst[3]>=5){Key_presst[3]=0;return 4;}
	else Key_presst[0]=0,Key_presst[1]=0,Key_presst[2]=0,Key_presst[3]=0;
	return 0;
}

void Led_control(uint8_t num,uint8_t state)
{
	if(state)HAL_GPIO_WritePin(GPIOC,(0x01<<7)<<num,GPIO_PIN_RESET);
	else HAL_GPIO_WritePin(GPIOC,(0x01<<7)<<num,GPIO_PIN_SET);
}

void Led_proc(void)
{
//	if(Flag_ledproc)
//	{
		
		if(Angle_ax>Angle_Pax)Led_control(1,1);
		else Led_control(1,0);
		if(Angle_bx>Angle_Pbx)Led_control(2,1);
		else Led_control(2,0);
		if(TIM2_IC2_Fre>Frequ_Pf)Led_control(3,1);
		else Led_control(3,0);
		if(Trigger_mode=='A')Led_control(4,1);
		else Led_control(4,0);
		if((90.0f+Angle_b-Angle_a)<10)Led_control(5,1);
		else Led_control(5,0);
		Led_control(6,0);
		Led_control(7,0);
		Led_control(8,0);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
//		Flag_ledproc=0;
//	}
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static float Change_num;
	if(htim==&htim1)
	{
		Key_state=Key_scan();
		if(Key_state==1)Flag_interface=!Flag_interface;
		else if(Key_state==2)
		{
			Angle_Pax=(Angle_Pax=Angle_Pax+10)>60?0:Angle_Pax;
			Angle_Pbx=(Angle_Pbx=Angle_Pbx+10)>60?0:Angle_Pbx;
		}
		else if(Key_state==3 && Flag_interface)Frequ_Pf=(Frequ_Pf=Frequ_Pf+1000)>10000?0:Frequ_Pf;
		else if(Key_state==3 && !Flag_interface)
		{
			if(Trigger_mode=='A')Trigger_mode='B';
			else if(Trigger_mode=='B')Trigger_mode='A';
		}
		else if(Key_state==4 && Trigger_mode=='A')
		{
			if(TIM3_IC1_Duty>=0.1f && TIM3_IC1_Duty<=0.9f)
				Angle_a=(180.0f/80.0f*(TIM3_IC1_Duty-0.1f))*100.0f;
			else if(TIM3_IC1_Duty>0.9f)Angle_a=180.0f;
			else if(TIM3_IC1_Duty<0.1f)Angle_a=0.0f;
			
			for(char i=4;i>0;i--)
			Angle_aa[i]=Angle_aa[i-1];
			Angle_aa[0]=Angle_a;
			
			Angle_ax=__fabs(Angle_aa[0]-Angle_aa[1]);
			
			for(char i=0;i<5;i++)
			Angle_qa[i]=Angle_aa[i];
			
			for(char i=0;i<4;i++)
			for(char j=0;j<4-i;j++)
			{
				if(Angle_qa[j]>Angle_qa[j+1])
				{
					Change_num=Angle_qa[j];
					Angle_qa[j]=Angle_qa[j+1];
					Angle_qa[j+1]=Change_num;
				}
			}
			
			if(TIM3_IC2_Duty>=0.1f && TIM3_IC2_Duty<=0.9f)
				Angle_b=(90.0f/80.0f*(TIM3_IC2_Duty-0.1f))*100.0f;
			else if(TIM3_IC2_Duty>0.9f)Angle_b=90.0f;
			else if(TIM3_IC2_Duty<0.1f)Angle_b=0.0f;
			
			for(char i=4;i>0;i--)
			Angle_bb[i]=Angle_bb[i-1];
			Angle_bb[0]=Angle_b;
			
			Angle_bx=__fabs(Angle_bb[0]-Angle_bb[1]);
			
			for(char i=0;i<5;i++)
			Angle_qb[i]=Angle_bb[i];
			
			for(char i=0;i<4;i++)
			for(char j=0;j<4-i;j++)
			{
				if(Angle_qb[j]>Angle_qb[j+1])
				{
					Change_num=Angle_qb[j];
					Angle_qb[j]=Angle_qb[j+1];
					Angle_qb[j+1]=Change_num;
				}
			}
		}
		
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static char Rx_state=0;
	char Tx_buff[100];
	if(huart==&huart1)
	{
		if(Rx_buff=='a' && Rx_state==0)Rx_state=1;
		else if(Rx_buff=='b' && Rx_state==0)Rx_state=2;
		else if(Rx_buff=='?' && Rx_state==1)
		{
			Rx_state=0;
			sprintf((char *)Tx_buff,"a:%.1f\r\n",Angle_a);
			HAL_UART_Transmit_IT(&huart1,(uint8_t *)Tx_buff,strlen(Tx_buff));
		}
		else if(Rx_buff=='?' && Rx_state==2)
		{
			Rx_state=0;
			sprintf((char *)Tx_buff,"b:%.1f\r\n",Angle_b);
			HAL_UART_Transmit_IT(&huart1,(uint8_t *)Tx_buff,strlen(Tx_buff));
		}
		else if(Rx_buff=='a' && Rx_state==1)Rx_state=3;
		else if(Rx_buff=='?' && Rx_state==3)
		{
			Rx_state=0;
			sprintf((char *)Tx_buff,"aa:%.1f-%.1f-%.1f-%.1f-%.1f\r\n",Angle_aa[0],Angle_aa[1],Angle_aa[2],Angle_aa[3],Angle_aa[4]);
			HAL_UART_Transmit_IT(&huart1,(uint8_t *)Tx_buff,strlen(Tx_buff));
		}
		else if(Rx_buff=='b' && Rx_state==2)Rx_state=4;
		else if(Rx_buff=='?' && Rx_state==4)
		{
			Rx_state=0;
			sprintf((char *)Tx_buff,"bb:%.1f-%.1f-%.1f-%.1f-%.1f\r\n",Angle_bb[0],Angle_bb[1],Angle_bb[2],Angle_bb[3],Angle_bb[4]);
			HAL_UART_Transmit_IT(&huart1,(uint8_t *)Tx_buff,strlen(Tx_buff));
		}
		else if(Rx_buff=='q')Rx_state=5;
		else if(Rx_buff=='a' && Rx_state==5)Rx_state=6;
		else if(Rx_buff=='b' && Rx_state==5)Rx_state=7;
		else if(Rx_buff=='?' && Rx_state==6)
		{
			Rx_state=0;
			sprintf((char *)Tx_buff,"qa:%.1f-%.1f-%.1f-%.1f-%.1f\r\n",Angle_qa[0],Angle_qa[1],Angle_qa[2],Angle_qa[3],Angle_qa[4]);
			HAL_UART_Transmit_IT(&huart1,(uint8_t *)Tx_buff,strlen(Tx_buff));
		}
		else if(Rx_buff=='?' && Rx_state==7)
		{
			Rx_state=0;
			sprintf((char *)Tx_buff,"qb:%.1f-%.1f-%.1f-%.1f-%.1f\r\n",Angle_qb[0],Angle_qb[1],Angle_qb[2],Angle_qb[3],Angle_qb[4]);
			HAL_UART_Transmit_IT(&huart1,(uint8_t *)Tx_buff,strlen(Tx_buff));
		}
		else
		{
			Rx_state=0;
			sprintf((char *)Tx_buff,"error\r\n");
			HAL_UART_Transmit_IT(&huart1,(uint8_t *)Tx_buff,strlen(Tx_buff));
		}
		HAL_UART_Receive_IT(&huart1,(uint8_t *)&Rx_buff,1);
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	LCD_Init();
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
//	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
//	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_4,500);
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_UART_Receive_IT(&huart1,(uint8_t *)&Rx_buff,1);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
	for(char i=1;i<9;i++)
	Led_control(i,0);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(Flag_interface==0)Data_interface();
		else if(Flag_interface==1)Parameter_interface();
		Lcd_dis();
		Led_proc();
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
