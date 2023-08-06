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
void Get_error(void);
int myabs(int a);
//int myabsf(int a);
void Pid_init(void);
void Scan_point(void);
void Stepmotor_control(int Speedx,int Speedy);
char keyscan(void);
void Rect_record(void);
void Goback_origin(void);
void Step_drawrect(void);
void myStepmotor_control(void);
void Count_step(void);
void Step_motor2D(void);
void myGPIO_set(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,u8 state);

extern u8   
        Imu_t,Uart_t,Pid_t,
        Pwm_state,
        StepmotorX_dir,StepmotorY_dir,
        StepmotorX_en,StepmotorY_en,
        Flag_steprecord,
        Flag_stepreset,Point_X,Point_Y,Point_Xlast,Point_Ylast,Error_X,Error_Y,
        Draw_state,Flag_getkp,Flag_draw,
        Rect_point[4][2],Rect_pointnum,Pointfollow_num,
        Flag_rectrecord,Flag_follow,Motor_choose,Countmove_t,Setmovexl_t,Setmovexr_t,Setmoveyu_t,Setmoveyd_t,
        Flag_IOPWM,Points_choose,Move_speed;

extern char    
        Key_state,
        mode,
        str[50],rx_buff[20];

extern float
        Real_aimx,Real_aimy,Aim_x,Aim_y,Reset_valueY,Reset_valueX,
        Center_x,Center_y,Rect_usepoint[130][2],Real_rectpoint[4][2],Kp_move;

extern PID Servo_x;
extern PID Servo_y;


extern int 
        longpresstime,shortpresstime,n,Rect_recordtimes,Sum_rectpoint[4][2],
        CurX_num,CurY_num,
        Point[2][7][2],
        keypresstime[4],keystate,StepXnum,StepYnum,uart_t;

#endif


