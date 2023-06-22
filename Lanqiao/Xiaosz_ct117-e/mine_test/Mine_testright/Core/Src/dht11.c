#include "dht11.h"

#define delay_us(X)  delayd(X*64)

void delayd(unsigned int n)
{
  while (n--);
}

void dht11_init (void )
{
  GPIO_InitTypeDef GPIO_InitStructure;
	
	__HAL_RCC_GPIOA_CLK_ENABLE();
	
	GPIO_InitStructure.Pin = GPIO_PIN_7;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure); 
  
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);

}

void mode_input(void )
{
  GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.Pin = GPIO_PIN_7;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure); 
}
void mode_output(void )
{
  GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.Pin = GPIO_PIN_7;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure); 
}

unsigned int dht11_read(void)
{
  int i;
  long long val;
  int timeout;

	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
  delay_us(18000);  //pulldown  for 18ms
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
  delay_us( 20 );	//pullup for 30us

  mode_input();

  //等待DHT11拉高，80us
  timeout = 5000;
  while( (! HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7)) && (timeout > 0) ) timeout--;	 //wait HIGH

  //等待DHT11拉低，80us
  timeout = 5000;
  while( HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7) && (timeout > 0) ) timeout-- ;	 //wait LOW

#define CHECK_TIME 28

  for(i=0;i<40;i++)
  {
	timeout = 5000;
	while( (! HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7)) && (timeout > 0) ) timeout--;	 //wait HIGH

	delay_us(CHECK_TIME);
	if ( HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7) )
	{
	  val=(val<<1)+1;
	} else {
	  val<<=1;
	}

	timeout = 5000;
	while( HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7) && (timeout > 0) ) timeout-- ;	 //wait LOW
  }

  mode_output();
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);

  if (((val>>32)+(val>>24)+(val>>16)+(val>>8) -val ) & 0xff  ) return 0;
    else return val>>8; 

}
