#include "TIM2.h"

//TIM4 PWM���ֳ�ʼ?
//PWM�����ʼ��
//arr:�Զ���װ��ֵ        psc:ʱ��Ԥ��Ƶ��



/******************************************************************************
* ģ������
* ��Ŀ���Ż����� ��  8·PWM���
* ��������       ��  2022/05/14
* ������         ��  ־��
* ģ�鹦��       ��  
* �޸�����       ��
* �ο��ĵ�       ��  ��ͨSTM32F4�⺯����
* ��Ŀ����ƽ̨   ��  STM32F103C8T6
* ����           ��  PWM��һ�㲽��
										 ʵ��Ҫ��TIM4������PWM���,��ʹ��TIM4��ͨ��1��2��3��4,�����ӳ�䵽PB6\PB7\PB8\PB9,����PWM�����ƶ��

											1��ʹ�ܶ�ʱ�������IOʱ�ӣ����ú���:RCC_APB1PeriphClockCmd();RCC_APB2PeriphClockCmd();
											2����ʼ��IOΪ���ù�����������ú���:GPIO_Init();�������ǰ�PB6\PB7\PB8\PB9������ʱ����PWM�������,Ҫ��ӳ�����ü�GPIO_Mode_AF_PP;
											   �������������������Ҫ����AFIOʱ�ӡ�
												 
											3����ʼ����ʱ�������ú���:TIM_TimeBaseInit();
											4����ʼ������Ƚϲ��������ú���:TIM_OCInitStructure();
											5��ʹ��Ԥװ�ؼĴ��������ú���:TIM_OC1PreloadConfig();
											6��ʹ�ܶ�ʱ�������ú���:TIM_Cmd();											
											7�����ö����ʼ�Ƕȣ����ú�����TIM_SetCompare1(TIM4,1945);
											
�߼���ʱ��1��	TIM_CtrlPWMOutputs(TIM1,ENABLE);//TIM1���ʹ��   �߼���ʱ�����Եü�����Ϊ��仰�Ҹ���һ�����ϣ����˽�֮��֮
											
											
     ���ö�ʱ����PWM���ʱ������	TIM_OCInitTypeDef�ṹ����������벶������TIM_ICInitTypeDef�ṹ��										
typedef struct
{
  uint16_t TIM_OCMode;        

  uint16_t TIM_OutputState;    ���״̬
  uint16_t TIM_OutputNState;   ����ͨ�������״̬
  uint16_t TIM_Pulse;          ռ�ձ�
  uint16_t TIM_OCPolarity;     �������
  uint16_t TIM_OCNPolarity;    ����ͨ�����������
  uint16_t TIM_OCIdleState;    ����״̬
  uint16_t TIM_OCNIdleState;   ����ͨ�������״̬
} TIM_OCInitTypeDef;

   
����֪ʶ������
TIM1_ETR     PA12            
TIM1_CH1     PA8            TIM2_CH1_ETR PA0            TIM3_CH1     PA6            TIM4_CH1     PB6 
TIM1_CH2     PA9            TIM2_CH2     PA1            TIM3_CH2     PA7            TIM4_CH2     PB7
TIM1_CH3     PA10           TIM2_CH3     PA2            TIM3_CH3     PB0            TIM4_CH3     PB8 
TIM1_CH4     PA11           TIM2_CH4     PA3            TIM3_CH4     PB1            TIM4_CH4     PB9  
*******************************************************************************/
void TIM2_PWM_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;   					 				//��ʼ��TIM1����ARR,PSC�������PWM������
	TIM_OCInitTypeDef  TIM_OCInitStructure;                         //PWMͨ������
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);     			  //ʹ�ܶ�ʱ��4
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);      				//AFIO���ù���ʱ��ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  ,ENABLE);   				//GPIOAʹ��
	
	
	//���ø�����Ϊ�����������
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;	       //TIM2_CH1/CH2/CH3/CH4
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;                                     //�����������
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);                                               //��ʼ��GPIO
	
	
	//��ʼ��TIM1,����TIM4��ARR��PSC
	TIM_TimeBaseStruct.TIM_Period = arr;                   					 //�����Զ���װ������ֵ   //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 
	TIM_TimeBaseStruct.TIM_Prescaler = psc;                					 //����Ԥ��Ƶֵ           //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  ����Ƶ
	TIM_TimeBaseStruct.TIM_ClockDivision = 0;               				 //����ʱ�ӷָ�TDTS = Tck_tim	
	TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;         //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStruct);                      //����ָ��������ʼ��TIMx  //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	


	
	//��ʼ��������Ƚϲ���
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;                  //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;      //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;          //�������:TIM����Ƚϼ��Ը�
	
	TIM_OC1Init(TIM2,&TIM_OCInitStructure);                            //����TIMX�Ĳ����趨��ʼ������ TIM2 ch1 ch2 ch3 ch4	
	TIM_OC2Init(TIM2,&TIM_OCInitStructure);
	TIM_OC3Init(TIM2,&TIM_OCInitStructure);
  TIM_OC4Init(TIM2,&TIM_OCInitStructure);
	
	TIM_OC1PreloadConfig(TIM2,TIM_OCPreload_Enable);                   //ʹ��Ԥװ�ؼĴ���
  TIM_OC2PreloadConfig(TIM2,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM2,TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM2,TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM2, ENABLE);                                //ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���	
	
	TIM_CtrlPWMOutputs(TIM2,ENABLE);                                   //TIM1���ʹ��   �߼���ʱ�����Եü�����Ϊ��仰�Ҹ���һ�����ϣ����˽�֮��֮
	
	TIM_Cmd(TIM2,ENABLE);                                              //ʹ��TIM2
	
	
	//�������ú���
	//���ã����ĸ�ͨ����ѡ��һ�������ñȽ�ֵ��ͨ���ڳ�ʼ���������Ѿ������˱Ƚ�ֵ���˺������ڳ���ʼ��֮����޸�
  //TIM_SetCompare1(TIM2,1850);
	//TIM_SetCompare2(TIM2,1850);
	//TIM_SetCompare3(TIM2,1850);
	//TIM_SetCompare4(TIM2,1850);
	
	

}




