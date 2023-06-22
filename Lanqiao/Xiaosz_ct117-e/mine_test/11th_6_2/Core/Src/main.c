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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
char
		Key_state=0,
		line[10][21],
		Flag_interface=0,
		Led_voltage=1,
		Led_frequ=2,
		Flag_follow,
		Led_voltagebuffer=0,
		Led_frequbuffer=1;
int
		Frequ_pa1,
		Frequ_pa2,
		Key_presst[4],
		adct=0;
float
		Voltage_pa4,
		Voltage_pa5;

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
	else{Key_presst[0]=0,Key_presst[1]=0,Key_presst[2]=0,Key_presst[3]=0;}
	return 0;
}

void Lcd_dis(void)
{
	for(char i=0;i<10;i++)
	LCD_DisplayStringLine(i*24,(uint8_t *)line[i]);
}

void Data_interface(void)
{
	sprintf(line[0],"                    ");
	sprintf(line[1],"    DATA            ");
	sprintf(line[2],"                    ");
	sprintf(line[3],"    V1:%.1fV         ",Voltage_pa4);
	sprintf(line[4],"    V2:%.1fV         ",Voltage_pa5);
	
	sprintf(line[5],"    F1:%dHz          ",Frequ_pa1);
	sprintf(line[6],"    F2:%dHz          ",Frequ_pa2);
	sprintf(line[7],"     %d              ",adct);
	sprintf(line[8],"                    ");
	sprintf(line[9],"                    ");
}

void Para_interface(void)
{
	sprintf(line[0],"                    ");
	sprintf(line[1],"    PARA            ");
	sprintf(line[2],"                    ");
	sprintf(line[3],"    VD:LD%d          ",Led_voltagebuffer);
	sprintf(line[4],"    FD:LD%d          ",Led_frequbuffer);
	
	sprintf(line[5],"                    ");
	sprintf(line[6],"                    ");
	sprintf(line[7],"                    ");
	sprintf(line[8],"                    ");
	sprintf(line[9],"                    ");
}

void Led_control(char num,char state)
{
	HAL_GPIO_WritePin(GPIOC,0x80<<num,state?GPIO_PIN_SET:GPIO_PIN_RESET);
}

void Led_proc(void)
{
	Led_control(Led_voltage,Voltage_pa4>Voltage_pa5);
	Led_control(Led_frequ,Frequ_pa1>Frequ_pa2);
	for(char i=1;i<9;i++)
	{
		if(i!=Led_voltage && i!=Led_frequ)
		Led_control(i,1);
	}
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
}

float Get_voltage(void)
{
	float value;
	value=HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Start_IT(&hadc2);
	return value/4096.0f*3.3f;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim2)
    {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
        {
            static uint32_t prev_capture_pa1 = 0;
            uint32_t curr_capture_pa1 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
            uint32_t pulse_width_pa1 = curr_capture_pa1 - prev_capture_pa1;
            prev_capture_pa1 = curr_capture_pa1;
						Frequ_pa1 = 1000000/pulse_width_pa1;
        }
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
        {
            static uint32_t prev_capture_pa2 = 0;
            uint32_t curr_capture_pa2 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_3);
            uint32_t pulse_width_pa2 = curr_capture_pa2 - prev_capture_pa2;
            prev_capture_pa2 = curr_capture_pa2;
						Frequ_pa2 = 1000000/pulse_width_pa2;
        }
    }
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim==&htim1)
	{
		Key_state=Key_scan();
		
		if(Key_state==1 && Flag_interface)
		{
			Flag_interface=!Flag_interface;
			Led_voltage=Led_voltagebuffer;
			Led_frequ=Led_frequbuffer;
		}
		else if(Key_state==1)Flag_interface=!Flag_interface;
		else if(Key_state==2 && Flag_interface)
		{
			Led_voltagebuffer=(++Led_voltagebuffer==Led_frequbuffer)?((++Led_voltagebuffer>8)?
			((Led_frequbuffer==1)?2:1):Led_voltagebuffer):((Led_voltagebuffer>8)?((Led_frequbuffer==1)?2:1):Led_voltagebuffer);
		}
		else if(Key_state==3 && Flag_interface)
		{
			Led_frequbuffer=(++Led_frequbuffer==Led_voltagebuffer)?((++Led_frequbuffer>8)?
			((Led_voltagebuffer==1)?2:1):Led_frequbuffer):((Led_frequbuffer>8)?((Led_voltagebuffer==1)?2:1):Led_frequbuffer);
		}
		else if(Key_state==4)
		{
			Flag_follow=!Flag_follow;
		}
		
		if(++adct==5)Voltage_pa4=Get_voltage();
		if(adct==10)Voltage_pa5=Get_voltage(),adct=0;
		
		if(Flag_follow)htim3.Instance->PSC=170000000/Frequ_pa1/10;
		else htim3.Instance->PSC=170000000/Frequ_pa2/10;
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	LCD_Init();
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,5);
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_3);
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
	for(char i=1;i<9;i++)
	Led_control(i,1);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		if(!Flag_interface)Data_interface();
		else Para_interface();
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
