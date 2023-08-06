#ifndef __TIM1_H
#define __TIM1_H

#include "stm32f10x.h"

typedef struct
{
  float Kp;
  float Ki;
  float Kd;
  float Err;
  float Err_last;
  float Err_l2st;  //?????
  float Integral_lim; //????
  float Differ_lim;
  float Err_Int;  //????
  float P_out;
  float I_out;
  float D_out;
  float Out;
  float Out_lim;
} PID;

void TIM1_Init(u16 arr,u16 psc);
void Fox_Pid_Position(float Target,float Feedback,PID* Pid);
//void Get_error(void);
int myabs(int a);
//int myabsf(int a);
void Pid_init(void);
void Scan_point(void);
void Stepmotor_control(int Speedx,int Speedy);
char keyscan(void);
void Key_set(void);
void Rect_record(void);
void Goback_origin(void);
void Step_drawrect(void);
void myStepmotor_control(void);

extern float Rect_usepoint[130][2];


extern PID Servo_x;
extern PID Servo_y;

#endif


