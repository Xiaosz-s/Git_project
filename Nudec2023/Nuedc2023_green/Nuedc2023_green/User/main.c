#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "usart.h"
#include "oled.h"
#include "stdio.h"
#include "tim3.h"
#include "tim2.h"
#include "TIM1.h"

#define PI 3.14159


void myGPIO_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure; 	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE, ENABLE); //APB2外设GPIO时钟使能      		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; //选择端口                        
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //选择IO接口工作方式       
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //设置IO接口速度（2/10/50MHz）    
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_5; //选择端口                        
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //选择IO接口工作方式       
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //设置IO接口速度（2/10/50MHz）    
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_10; //选择端口                        
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //选择IO接口工作方式       
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //设置IO接口速度（2/10/50MHz）    
	GPIO_Init(GPIOB, &GPIO_InitStructure);
    
	GPIO_ResetBits(GPIOC,GPIO_Pin_4);
    GPIO_ResetBits(GPIOB,GPIO_Pin_10);
	GPIO_ResetBits(GPIOA,GPIO_Pin_5);
}


void Key_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE); //APB2外设GPIO时钟使能
	GPIO_InitTypeDef GPIO_InitStructure;
    
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
}



/*
 * 
 *
 * */





int main(void)
{ 
    myGPIO_Init();
    Key_Init();
    Pid_init();
	USART1_Init(115200);
    TIM2_PWM_Init(999,719);
	TIM3_PWM_Init(999,719);
    TIM1_Init(99,719);
    TIM_SetCompare4(TIM2,0);
    TIM_SetCompare4(TIM3,0);
    TIM2->CCR4=499;
    TIM3->CCR4=499;
    
	while(1)
	{
//        Stepmotor_control(-10000,-0);
	}		 
}



