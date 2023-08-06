#ifndef __XIAOSZ_OLED_H
#define __XIAOSZ_OLED_H			  	 
#include "sys.h"

//-----------------OLED端口定义---------------- 
#define OLED_RST_Clr() PCout(15)=0   //RST
#define OLED_RST_Set() PCout(15)=1   //RST

#define OLED_RS_Clr() PCout(0)=0    //DC
#define OLED_RS_Set() PCout(0)=1    //DC

#define OLED_SCLK_Clr()  PCout(13)=0  //SCL
#define OLED_SCLK_Set()  PCout(13)=1   //SCL

#define OLED_SDIN_Clr()  PCout(14)=0   //SDA
#define OLED_SDIN_Set()  PCout(14)=1   //SDA

#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据
//OLED控制用函数
void OLED_6Pin_WR_Byte(u8 dat,u8 cmd);	    
void OLED_6Pin_Display_On(void);
void OLED_6Pin_Display_Off(void);
void OLED_6Pin_Refresh_Gram(void);		   				   		    
void OLED_6Pin_Init(void);
void OLED_6Pin_Clear(void);
void OLED_6Pin_DrawPoint(u8 x,u8 y,u8 t);
void OLED_6Pin_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode);
void OLED_6Pin_ShowNumber(u8 x,u8 y,u32 num,u8 len,u8 size);
void OLED_6Pin_ShowString(u8 x,u8 y,const u8 *p);	 
u32 oled_pow(u8 m,u8 n);

void OLED_4Pin_Init(void);
void OLED_4Pin_Clear(void);
void OLED_4Pin_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_4Pin_ShowString(uint8_t Line, uint8_t Column, char *String);
void OLED_4Pin_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_4Pin_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED_4Pin_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_4Pin_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);


#endif  
	 
