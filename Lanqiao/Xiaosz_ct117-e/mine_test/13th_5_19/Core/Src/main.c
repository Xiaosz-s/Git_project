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
#include "string.h"
#include "i2c_hal.h"
#include "lcd.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

int 
Key_presst[4]={0,0,0,0},// 按键按下时间
Frequ_pa1,//PA1频率
Frequ_pa7;//PA7频率

char 
line[10][21];//LCD字符串 10行 20列

uint8_t 
Key_state=0, //按键状态值
Frequ_x,Voltage_y, //频率X 电压Y参数
Flag_interface=0,//界面显示标志位
Flag_Voltagetest=0,//电压检测执行标志位
Flag_parametermode=0,//参数界面下 切换倍频 分频模式标志位 0-倍频 1-分频
Flag_recordingchannel=0,//记录界面下 切换不同测量通道标志位
Recording_channel[2]={4,5},//记录界面通道显示
Recording_n[2]={0,0}, //记录次数
Recording_firstmaxvoltage=1, //使得首次电压最小值为3.3
Flag_lcddisdirection=1, //1 正常显示  0 反向显示
Flag_uarttx=0,  //  串口输出标志位 1 PA1 2 PA4 3 PA5
Flag_ledchange=1, //led状态改变标志位
Flag_led4flash; //led4翻转标志位

float 
Voltage_pa4,//PA4电压值
Voltage_pa5,//PA5电压值
Sum_average[2]={0,0}, //记录电压总值以计算平均值
Average_h[2]={0,0},//平均值H
Voltage_mint[2]={0,0},//电压最小值
Voltage_maxa[2]={0,0};//电压最大值

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */



char Key_scan(void)
{
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)==0)Key_presst[0]++;
	else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)==1 && Key_presst[0]>5){Key_presst[0]=0;return 1;}
	else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)==0)Key_presst[1]++;
	else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)==1 && Key_presst[1]>5){Key_presst[1]=0;return 2;}
	else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2)==0)Key_presst[2]++;
	else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2)==1 && Key_presst[2]>5){Key_presst[2]=0;return 3;}
	else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==0)Key_presst[3]++;
	else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==1 && Key_presst[3]>100){Key_presst[3]=0;return 5;}
	else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==1 && Key_presst[3]>5){Key_presst[3]=0;return 4;}
	else {Key_presst[0]=0,Key_presst[1]=0,Key_presst[2]=0,Key_presst[3]=0;}
	return 0;
}

void Data_interface(void)
{
	sprintf(line[0],"                    ");
	sprintf(line[1],"        DATA        ");
	sprintf(line[2],"                    ");
	sprintf(line[3],"     PA4=%.2f       ",Voltage_pa4);
	sprintf(line[4],"     PA5=%.2f       ",Voltage_pa5);
	sprintf(line[5],"     PA1=%d          ",Frequ_pa1);
	sprintf(line[6],"                    ");
	sprintf(line[7],"                    ");
	sprintf(line[8],"                    ");
	sprintf(line[9],"                    ");
}

void Parameter_interface(void)
{
	sprintf(line[0],"                    ");
	sprintf(line[1],"        PARA        ");
	sprintf(line[2],"                    ");
	sprintf(line[3],"     X=%d            ",Frequ_x);
	sprintf(line[4],"     Y=%d            ",Voltage_y);
	sprintf(line[5],"                    ");
	sprintf(line[6],"                    ");
	sprintf(line[7],"                    ");
	sprintf(line[8],"                    ");
	sprintf(line[9],"                    ");
}

void Recording_interface(void)
{
	sprintf(line[0],"                    ");
	sprintf(line[1],"        REC-PA%d     ",Recording_channel[Flag_recordingchannel]);
	sprintf(line[2],"                    ");
	sprintf(line[3],"     N=%d            ",Recording_n[Flag_recordingchannel>0]);
	sprintf(line[4],"     A=%.2f            ",Voltage_maxa[Flag_recordingchannel>0]);
	sprintf(line[5],"     T=%.2f            ",Voltage_mint[Flag_recordingchannel>0]);
	sprintf(line[6],"     H=%.2f            ",Average_h[Flag_recordingchannel>0]);
	sprintf(line[7],"                    ");
	sprintf(line[8],"                    ");
	sprintf(line[9],"                    ");
}

float Get_adc(void)
{
	float adc;
	HAL_ADC_Start_IT(&hadc2);
	adc=HAL_ADC_GetValue(&hadc2);
	adc=adc*3.3f/4096;
	return adc;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	static int Frequ;
	if(htim==&htim2)
	{
		Frequ=__HAL_TIM_GetCounter(&htim2);
		__HAL_TIM_SetCounter(&htim2,0);
		HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_2);
		Frequ_pa1=Frequ/1000000;
	}
}

void Lcd_dis(void)
{
	for(char i=0;i<10;i++)
	if(Flag_lcddisdirection)
	LCD_DisplayStringLine(i*24,(uint8_t *)line[i]);
	else
	LCD_DisplayStringLine_reverse(i*24,(uint8_t *)line[9-i]);
}

uint8_t At24c02_read(uint8_t address)
{
	uint8_t value;
	I2CStart();
	I2CSendByte(0xa0);
	I2CWaitAck();
	I2CSendByte(address);
	I2CWaitAck();
	I2CStart();
	I2CSendByte(0xa1);
	I2CWaitAck();
	value=I2CReceiveByte();
	I2CWaitAck();
	I2CStop();
	return value;
}

void At24c02_write(uint8_t address, uint8_t value)
{
	I2CStart();
	I2CSendByte(0xa0);
	I2CWaitAck();
	I2CSendByte(address);
	I2CWaitAck();
	I2CSendByte(value);
	I2CWaitAck();
	I2CStop();
}

void Uart_proc(void)
{
	static char txt[30];
	if(Flag_uarttx==1)sprintf(txt,"PA1:%d\r\n",Frequ_x);
	else if(Flag_uarttx==2)sprintf(txt,"PA4:%.2f\r\n",Voltage_pa4);
	else if(Flag_uarttx==3)sprintf(txt,"PA5:%.2f\r\n",Voltage_pa5);
	if(Flag_uarttx)HAL_UART_Transmit_IT(&huart1,(uint8_t *)txt,strlen(txt)),Flag_uarttx=0;
}

void Led_control(int n,_Bool state)
{
	HAL_GPIO_WritePin(GPIOC,0x01<<7<<n,(state)?GPIO_PIN_RESET:GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
}

void Led_proc(void)
{
	if(Flag_ledchange)
	{
		Flag_ledchange=0;
		Led_control(1,Flag_parametermode);
		Led_control(2,!Flag_parametermode);
		Led_control(3,Flag_lcddisdirection);
		Led_control(4,Flag_led4flash);
		Led_control(5,0);
		Led_control(6,0);
		Led_control(7,0);
		Led_control(8,0);
	}
}



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static char Voltage_t=0,Led_flasht=0;
	if(htim==&htim1)
	{
		Key_state=Key_scan();
		if(Key_state==1)Flag_interface=++Flag_interface>2?0:Flag_interface;
		else if(Key_state==2)
		{
			Frequ_x=++Frequ_x>4?0:Frequ_x;
			At24c02_write(1,Frequ_x);
			for(int i=0;i<1000;i++)
			{
				
			}
		}
		else if(Key_state==3)
		{
			Voltage_y=++Voltage_y>4?0:Voltage_y;
			At24c02_write(0,Voltage_y);
			for(int i=0;i<1000;i++)
			{
				
			}
		}
		else if(Key_state==4 && Flag_interface==0)Flag_Voltagetest=1,Recording_n[0]++,Recording_n[1]++;
		else if(Key_state==4 && Flag_interface==1)Flag_parametermode=!Flag_parametermode,Flag_ledchange=1;
		else if(Key_state==4 && Flag_interface==2)Flag_recordingchannel=!Flag_recordingchannel;
		else if(Key_state==5 && Flag_interface==2)//置0
		{
			Recording_n[Flag_parametermode>0]=0;
			Voltage_maxa[Flag_parametermode>0]=0;
			Voltage_mint[Flag_parametermode>0]=0;
			Average_h[Flag_parametermode>0]=0;
		}
		
		if(Flag_parametermode) //倍频
		{
			Frequ_pa7=Frequ_pa1*Frequ_x;
			htim3.Instance->PSC=8000000/Frequ_pa7/200;
		}
		else //分频
		{
			Frequ_pa7=Frequ_pa1/Frequ_x;
			htim3.Instance->PSC=8000000/Frequ_pa7/200;
		}
		if(Flag_Voltagetest)  // 启动一次 电压采集
		{
			if(Recording_firstmaxvoltage)
			{
				Voltage_mint[0]=Voltage_mint[1]=3.3;
				Recording_firstmaxvoltage=0;
			}
			Voltage_t++;
			if(Voltage_t==5)
			{
				Voltage_pa4=Get_adc();
			}
			if(Voltage_t==10)
			{
				Voltage_t=0;
				Flag_Voltagetest=0;
				Voltage_pa5=Get_adc();
				
				Sum_average[0]+=Voltage_pa4;
				if(!Recording_n[0])Average_h[0]=0;
				else Average_h[0]=Sum_average[0]/Recording_n[0];
				
				Sum_average[1]+=Voltage_pa5;
				if(!Recording_n[1])Average_h[1]=0;
				else Average_h[1]=Sum_average[1]/Recording_n[1];
				
				if(Voltage_pa4>Voltage_maxa[0])Voltage_maxa[0]=Voltage_pa4;
				if(Voltage_pa4<Voltage_mint[0])Voltage_mint[0]=Voltage_pa4;
				
				if(Voltage_pa5>Voltage_maxa[1])Voltage_maxa[1]=Voltage_pa5;
				if(Voltage_pa5<Voltage_mint[1])Voltage_mint[1]=Voltage_pa5;
			}
		}
		Led_flasht++;
		if(Led_flasht==10)
		{
			if(Voltage_pa4>Voltage_pa5*Voltage_y)Flag_led4flash=!Flag_led4flash,Flag_ledchange=1;
			Led_flasht=0;
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static uint8_t res,Rx_state=0;
	if(huart==&huart1)
	{
		HAL_UART_Receive_IT(&huart1,&res,1);
		if(res=='#')Flag_lcddisdirection=!Flag_lcddisdirection,Flag_ledchange=1;
		else
		{
			if(Rx_state==0 && res=='P')Rx_state=1;
			else if(Rx_state==1 && res=='A')Rx_state=2;
			else if(Rx_state==2 && res=='1')Flag_uarttx=1,Rx_state=0;
			else if(Rx_state==2 && res=='4')Flag_uarttx=2,Rx_state=0;
			else if(Rx_state==2 && res=='5')Flag_uarttx=3,Rx_state=0;
			else Rx_state=0;
		}
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
	I2CInit();
	LCD_Init();
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	HAL_UART_Receive_IT(&huart1,(uint8_t *)"c",1);//这里第一次接收 会被吞掉。。。怎么解决呢
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_2);
	__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,100);
	Frequ_x=At24c02_read(1);
	Voltage_y=At24c02_read(0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(Flag_interface==0)Data_interface();
		else if(Flag_interface==1)Parameter_interface();
		else if(Flag_interface==2)Recording_interface();
		Uart_proc();
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 10;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
