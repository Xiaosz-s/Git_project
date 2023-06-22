#include "dht11_hal.h"


dht11Data dht11;

//
static void usDelay(uint32_t us)
{
	uint16_t i = 0;
	while(us--){
		i = 64;
		while(i--);
	}
}

//
void outDQ(uint8_t i)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	
	GPIO_InitStructure.Pin = HDQ;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	if(!i) 
		HAL_GPIO_WritePin(GPIOA, HDQ, GPIO_PIN_RESET);
	else 
		HAL_GPIO_WritePin(GPIOA, HDQ, GPIO_PIN_SET);
}

//
void inDQ(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.Pin = HDQ;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
}

//
void dht11Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
	outDQ(1);			
}

//
uint8_t recData(void)
{
	uint8_t i,temp=0,j=220;
	
	for(i=0; i<8; i++){
		
		while(!HAL_GPIO_ReadPin(GPIOA,HDQ));
		usDelay(40);
		if(HAL_GPIO_ReadPin(GPIOA,HDQ))
		{
			temp=(temp<<1)|1;
			while(HAL_GPIO_ReadPin(GPIOA,HDQ)&&(j--));	
		}	
		else
		{
			temp=(temp<<1)|0;
		}
	}
	return temp;
}

unsigned int Dht11_read(void)
{
	int time=0;
	long long value;
	HAL_GPIO_WritePin(GPIOA,HDQ,GPIO_PIN_RESET);
	usDelay(18000);
	HAL_GPIO_WritePin(GPIOA,HDQ,GPIO_PIN_SET);
	usDelay(20);
	inDQ();
	time=5000;
	while(!HAL_GPIO_ReadPin(GPIOA,HDQ) && time>0)time--;
	time=5000;
	while(HAL_GPIO_ReadPin(GPIOA,HDQ) && time>0)time--;
	for(char i=0;i<40;i++)
	{
		time=5000;
		while(!HAL_GPIO_ReadPin(GPIOA,HDQ) && time>0)time--;
		usDelay(28);
		if(HAL_GPIO_ReadPin(GPIOA,HDQ))value=(value<<1)+1;
		else value<<=1;
		time=5000;
		while(HAL_GPIO_ReadPin(GPIOA,HDQ) && time>0)time--;
	}
	outDQ(1);
	if(((value>>32)+(value>>24)+(value>>16)+(value>>8)-value) & 0xff)return 0;
	else return value>>8;
}
