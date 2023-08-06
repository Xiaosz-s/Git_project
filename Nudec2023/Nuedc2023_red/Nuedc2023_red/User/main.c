#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "usart.h"
#include "stdio.h"
#include "tim3.h"
#include "tim2.h"
#include "TIM1.h"
#include "xiaosz_oled.h"
#include "mykey.h"

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
	
	GPIO_ResetBits(GPIOC,GPIO_Pin_4);
	GPIO_ResetBits(GPIOA,GPIO_Pin_5);
}

void myPWMGPIO_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure; 	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE, ENABLE); //APB2??GPIO????      		
	
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_3; //????                        
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //??IO??????       
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //??IO????(2/10/50MHz)    
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_1; //????                        
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //??IO??????       
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //??IO????(2/10/50MHz)    
	GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_ResetBits(GPIOB,GPIO_Pin_1);
	GPIO_ResetBits(GPIOA,GPIO_Pin_3);
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
    char txt[30];
    myPWMGPIO_Init();
    myGPIO_Init();
    Key_Init();
    Pid_init();
    OLED_4Pin_Init();
	OLED_4Pin_Clear();
	USART1_Init(115200);
    
//    TIM2_PWM_Init(999,719);
//	TIM3_PWM_Init(999,719);
    TIM1_Init(99,719);
//    TIM_SetCompare4(TIM2,0);
//    TIM_SetCompare4(TIM3,0);
//    TIM2->CCR4=499;
//    TIM3->CCR4=499;
//    Stepmotor_control((int)0,(int)0);
//    Stepmotor_control(-10000,-10000);
//    Rect_usepoint[0][0]=87;
//    Rect_usepoint[0][1]=77;
//    Rect_usepoint[1][0]=114;
//    Rect_usepoint[1][1]=48;
//    Rect_usepoint[2][0]=73;
//    Rect_usepoint[2][1]=14;
//    Rect_usepoint[3][0]=46;
//    Rect_usepoint[3][1]=40;
//    Rect_usepoint[4][0]=87;
//    Rect_usepoint[4][1]=77;
    
	while(1)
	{
//        printf("6");
//        sprintf(txt," Hello world");
        sprintf(txt,"X%d%d Y%d%d Tm%d Km%d",StepmotorX_dir,StepmotorX_en,StepmotorY_dir,StepmotorY_en,Triggermode,Flag_adjust,Keymode);
		OLED_4Pin_ShowString(1,1,txt);
        sprintf(txt,"R%d F%d C%d O%d D%d",Flag_rectrecord,Flag_follow,Flag_steprecord,Flag_stepreset,Flag_draw);
		OLED_4Pin_ShowString(2,1,txt);
//        sprintf(txt,"Km:%d Tm%d ",Flag_adjust,Keymode,Triggermode);
//		OLED_4Pin_ShowString(3,1,txt);
        
        /*********************************TASK2*****************************************/
        
             if(Keymode==0 && Flag_adjust==1)sprintf(txt,"%dPoint%d:%d    ",Flag_adjust,Keymode,Point[0][1][0]);
        else if(Keymode==1 && Flag_adjust==1)sprintf(txt,"%dPoint%d:%d    ",Flag_adjust,Keymode,Point[0][1][1]);
        else if(Keymode==2 && Flag_adjust==1)sprintf(txt,"%dPoint%d:%d    ",Flag_adjust,Keymode,Point[0][2][0]);
        else if(Keymode==3 && Flag_adjust==1)sprintf(txt,"%dPoint%d:%d    ",Flag_adjust,Keymode,Point[0][2][1]);
        else if(Keymode==4 && Flag_adjust==1)sprintf(txt,"%dPoint%d:%d    ",Flag_adjust,Keymode,Point[0][3][0]);
        else if(Keymode==5 && Flag_adjust==1)sprintf(txt,"%dPoint%d:%d    ",Flag_adjust,Keymode,Point[0][3][1]);
        else if(Keymode==6 && Flag_adjust==1)sprintf(txt,"%dPoint%d:%d    ",Flag_adjust,Keymode,Point[0][4][0]);
        else if(Keymode==7 && Flag_adjust==1)sprintf(txt,"%dPoint%d:%d    ",Flag_adjust,Keymode,Point[0][4][1]);
        
        /*********************************TASK3*****************************************/
        
        else if(Keymode==0 && Flag_adjust==2)sprintf(txt,"%dPoint%d:%d    ",Flag_adjust,Keymode,Point[1][1][0]);
        else if(Keymode==1 && Flag_adjust==2)sprintf(txt,"%dPoint%d:%d    ",Flag_adjust,Keymode,Point[1][1][1]);
        else if(Keymode==2 && Flag_adjust==2)sprintf(txt,"%dPoint%d:%d    ",Flag_adjust,Keymode,Point[1][2][0]);
        else if(Keymode==3 && Flag_adjust==2)sprintf(txt,"%dPoint%d:%d    ",Flag_adjust,Keymode,Point[1][2][1]);
        else if(Keymode==4 && Flag_adjust==2)sprintf(txt,"%dPoint%d:%d    ",Flag_adjust,Keymode,Point[1][3][0]);
        else if(Keymode==5 && Flag_adjust==2)sprintf(txt,"%dPoint%d:%d    ",Flag_adjust,Keymode,Point[1][3][1]);
        else if(Keymode==6 && Flag_adjust==2)sprintf(txt,"%dPoint%d:%d    ",Flag_adjust,Keymode,Point[1][4][0]);
        else if(Keymode==7 && Flag_adjust==2)sprintf(txt,"%dPoint%d:%d    ",Flag_adjust,Keymode,Point[1][4][1]);
        
        /*********************************TASK4*****************************************/
        
        else if(Keymode==0 && Flag_adjust==3)sprintf(txt,"%dPoint%d:%.2f  ",Flag_adjust,Keymode,Rect_usepoint[0][0]);
        else if(Keymode==1 && Flag_adjust==3)sprintf(txt,"%dPoint%d:%.2f  ",Flag_adjust,Keymode,Rect_usepoint[0][1]);
        else if(Keymode==2 && Flag_adjust==3)sprintf(txt,"%dPoint%d:%.2f  ",Flag_adjust,Keymode,Rect_usepoint[1][0]);
        else if(Keymode==3 && Flag_adjust==3)sprintf(txt,"%dPoint%d:%.2f  ",Flag_adjust,Keymode,Rect_usepoint[1][1]);
        else if(Keymode==4 && Flag_adjust==3)sprintf(txt,"%dPoint%d:%.2f  ",Flag_adjust,Keymode,Rect_usepoint[2][0]);
        else if(Keymode==5 && Flag_adjust==3)sprintf(txt,"%dPoint%d:%.2f  ",Flag_adjust,Keymode,Rect_usepoint[2][1]);
        else if(Keymode==6 && Flag_adjust==3)sprintf(txt,"%dPoint%d:%.2f  ",Flag_adjust,Keymode,Rect_usepoint[3][0]);
        else if(Keymode==7 && Flag_adjust==3)sprintf(txt,"%dPoint%d:%.2f  ",Flag_adjust,Keymode,Rect_usepoint[3][1]);
        
		OLED_4Pin_ShowString(3,1,txt);

	}		 
}



