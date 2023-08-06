#ifndef __MYKEY_H
#define __MYKEY_H

#include "stm32f10x.h"
#include "TIM1.h"
#include "TIM2.h"
#include "TIM3.h"


extern u8 Keymode,Triggermode,Flag_adjust;

void Key_set(void);



#endif


