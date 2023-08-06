#include "TIM1.h"
#include "usart.h"
#include "mykey.h"


u8      
        Imu_t=0,Uart_t=0,Pid_t=0,control_t=0,
        Pwm_state=1,
        StepmotorX_dir=0,StepmotorY_dir=0,
        StepmotorX_en=0,StepmotorY_en=0,
        Flag_steprecord=0,
        Flag_stepreset=0,Point_X,Point_Y,Point_Xlast,Point_Ylast,Error_X,Error_Y,
        Draw_state=0,Flag_getkp=1,Flag_draw=0,
        Rect_point[4][2],Rect_pointnum=4,Pointfollow_num=0,
        Flag_rectrecord=0,Flag_follow=0,Motor_choose=0,Countmove_t,Setmovexl_t,Setmovexr_t,Setmoveyu_t,Setmoveyd_t,
        Flag_countx=1,Flag_county=1,Flag_IOPWM=1,Points_choose=0,
        Move_speed=0;

char    
        Key_state=0,
        mode=0,
        str[50],rx_buff[20];

float
        Real_aimx=720,Real_aimy=1130,Aim_x,Aim_y,Reset_valueY,Reset_valueX,
        Center_x,Center_y,Rect_usepoint[130][2],Real_rectpoint[4][2],Kp_move;

PID Servo_x;
PID Servo_y;


int 
        longpresstime=250,shortpresstime=10,n=0,Rect_recordtimes=0,Sum_rectpoint[4][2],
        CurX_num,CurY_num,
        Point[2][7][2]=
        {
        {{0,0},{950,880}, {911,-919}, {-964,-919},  {-900,900}, {950,920},    {0,0}},
        {{0,0},{-539,190},   {-539,880},  {520,900},   {535,220}, {-539,220},      {0,0}}
        },
        keypresstime[4],keystate,StepXnum=0,StepYnum=0,uart_t=0;
        

void TIM1_Init(u16 arr,u16 psc)    
{  
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);	
	 
	TIM_TimeBaseStructure.TIM_Period = arr;
	TIM_TimeBaseStructure.TIM_Prescaler =psc;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); 

	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);
	

	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);  
	TIM_Cmd(TIM1, ENABLE); 
}


void TIM1_UP_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) 
	{
        Key_state=keyscan();
        Key_set();
        if(Flag_rectrecord)
        {
            Rect_record();
        }
        
        if(Flag_stepreset)
        {
            if(StepYnum!=0)TIM2->CCR4=499;
            else TIM2->CCR4=0;
            if(StepXnum!=0)TIM3->CCR4=499;
            else TIM3->CCR4=0;
            Goback_origin();
        }
        if(Flag_draw)
        {
            Step_drawrect();
        }
        
        if(Flag_IOPWM && ++control_t>=Move_speed)
        {
            Step_motor2D();
            control_t=0;
        }
        
        if(++uart_t>100)
        {
            uart_t=0;
            printf("%d,%d,%d,%d,%d,%d,%d,%d,%d\n",Error_X,Error_Y,StepXnum,StepYnum,CurX_num,CurY_num,Key_state,StepmotorX_dir,StepmotorY_dir);
//            printf("%d\n",Countmove_t);
//            printf("%d,%d,%d,%.2f,%d,%d,%d\n",Error_X-Point_X,Error_Y-Point_Y,Pointfollow_num,Servo_x.Out,TIM2->CCR4,TIM3->CCR4,Flag_follow);
//            printf("%.2f,%.2f,%d,%d\n",Servo_x.Out,Servo_y.Out,TIM2->PSC,TIM3->PSC);
//            printf("%d,%d,%d,%d,%d,%d\n",Flag_follow,TIM2->CNT,StepYnum,StepXnum,Flag_countx,TIM2->CCR4);
//            printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%.2f\n",StepmotorX_dir,StepmotorY_dir,StepmotorX_en,StepmotorY_en,
//            Draw_state,Kp_move);
        }
        
       if(++Pid_t>=10 && Flag_follow)
       {
            Pid_t=0;
            Fox_Pid_Position((float)Error_X,(float)Point_X,&Servo_x);
            Fox_Pid_Position((float)Error_Y,(float)Point_Y,&Servo_y);
            Stepmotor_control((int)Servo_x.Out,(int)Servo_y.Out);
            Get_error();
        }
		 TIM_ClearITPendingBit(TIM1, TIM_IT_Update  ); 
	}
}

void Get_error(void)
{
    static int Check_t=0;
    Error_X=Rect_usepoint[Pointfollow_num][0];
    Error_Y=Rect_usepoint[Pointfollow_num][1];
    if(myabs(Error_X-Point_X)<2 && myabs(Error_Y-Point_Y)<2 && ++Check_t>10)
    {
        Check_t=0;
        Pointfollow_num++;
    }
    if(Pointfollow_num>Rect_pointnum)
    {
        Pointfollow_num=0;
        Flag_follow=0;
        Error_X=0;
        Error_Y=0;
        TIM2->CCR4=0;
        TIM3->CCR4=0;
    }
}



void Pid_init(void)
{
    Servo_x.Kp=100;
    Servo_x.Ki=0;
    Servo_x.Kd=0;
    Servo_x.Integral_lim=100;
    Servo_x.Out_lim=40000;
    Servo_x.Out=0;

    Servo_y.Kp=100;
    Servo_y.Ki=0;
    Servo_y.Kd=0;
    Servo_y.Integral_lim=100;
    Servo_y.Out_lim=40000;
    Servo_y.Out=0;
}

void Fox_Pid_Position(float Target,float Feedback,PID* Pid)
{

    Pid->Err=Target-Feedback;

    Pid->P_out=Pid->Kp*Pid->Err;
    Pid->D_out=Pid->Kd*(Pid->Err-Pid->Err_last);
    Pid->Err_Int+=Pid->Err;
    if(Pid->Err_Int > +Pid->Integral_lim) Pid->Err_Int=+Pid->Integral_lim;
    if(Pid->Err_Int < -Pid->Integral_lim)  Pid->Err_Int=-Pid->Integral_lim;

    Pid->I_out=Pid->Ki*Pid->Err_Int;
    Pid->Out=Pid->P_out+Pid->D_out+Pid->I_out;
    Pid->Err_last=Pid->Err;

    //Fox_float_lmit(Pid->Out,Pid->Integral_Min,Pid->Integral_Max);
    if(Pid->Out > +Pid->Out_lim) Pid->Out=+Pid->Out_lim;
    if(Pid->Out < -Pid->Out_lim) Pid->Out=-Pid->Out_lim;

}

int myabs(int a)
{
    return (a>0?a:-a);
}


float myabsf(float a)
{
    return (a>0?a:-a);
}

void Scan_point(void)
{
    float Point_diffx,Point_diffy;
    int Point_sign=0,Diff_num;
    Diff_num=Rect_pointnum/4;

    /*********************************?????*********************************/
    Point_diffx=(Real_rectpoint[1][0]-Real_rectpoint[0][0])/Diff_num;
    Point_diffy=(Real_rectpoint[1][1]-Real_rectpoint[0][1])/Diff_num;
    for(u8 i=0;i<Diff_num;i++,Point_sign++)
    {
        Rect_usepoint[Point_sign][0]=Real_rectpoint[0][0]+i*Point_diffx;
        Rect_usepoint[Point_sign][1]=Real_rectpoint[0][1]+i*Point_diffy;
    }
    /*********************************?????*********************************/
    Point_diffx=(Real_rectpoint[2][0]-Real_rectpoint[1][0])/Diff_num;
    Point_diffy=(Real_rectpoint[2][1]-Real_rectpoint[1][1])/Diff_num;
    for(u8 i=0;i<Diff_num;i++,Point_sign++)
    {
        Rect_usepoint[Point_sign][0]=Real_rectpoint[1][0]+i*Point_diffx;
        Rect_usepoint[Point_sign][1]=Real_rectpoint[1][1]+i*Point_diffy;
    }
    /*********************************?????*********************************/
    Point_diffx=(Real_rectpoint[3][0]-Real_rectpoint[2][0])/Diff_num;
    Point_diffy=(Real_rectpoint[3][1]-Real_rectpoint[2][1])/Diff_num;
    for(u8 i=0;i<Diff_num;i++,Point_sign++)
    {
        Rect_usepoint[Point_sign][0]=Real_rectpoint[2][0]+i*Point_diffx;
        Rect_usepoint[Point_sign][1]=Real_rectpoint[2][1]+i*Point_diffy;
    }
    /*********************************?????*********************************/
    Point_diffx=(Real_rectpoint[0][0]-Real_rectpoint[3][0])/Diff_num;
    Point_diffy=(Real_rectpoint[0][1]-Real_rectpoint[3][1])/Diff_num;
    for(u8 i=0;i<Diff_num;i++,Point_sign++)
    {
        Rect_usepoint[Point_sign][0]=Real_rectpoint[3][0]+i*Point_diffx;
        Rect_usepoint[Point_sign][1]=Real_rectpoint[3][1]+i*Point_diffy;
    }
    Rect_usepoint[Point_sign][0]=Real_rectpoint[0][0];
    Rect_usepoint[Point_sign][1]=Real_rectpoint[0][1];
    printf("\r\n%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n"
            ,Real_rectpoint[0][0],Real_rectpoint[0][1],Real_rectpoint[1][0],Real_rectpoint[1][1]
            ,Real_rectpoint[2][0],Real_rectpoint[2][1],Real_rectpoint[3][0],Real_rectpoint[3][1]);
    printf("\r\n\r\n");
    for(u8 i=0;i<20;i++)
    {
        printf("%.2f,%.2f,%d\r\n",Rect_usepoint[i][0],Rect_usepoint[i][1],Pointfollow_num);
    }
    printf("\r\n\r\n");
}


void Stepmotor_control(int Speedx,int Speedy)
{
    TIM3->CCR4=499;
    TIM2->CCR4=499;
    if(Speedx<0)
    {
        StepmotorX_dir=1;
        GPIO_ResetBits(GPIOC,GPIO_Pin_4);
    }
    else if(Speedx>0)
    {
        StepmotorX_dir=0;
        GPIO_SetBits(GPIOC,GPIO_Pin_4);
    }
    else
    {
        TIM3->CCR4=0;
    }
    
    if(Speedy>0)
    {
        StepmotorY_dir=1;
        GPIO_ResetBits(GPIOA,GPIO_Pin_5);
    }
    else if(Speedy<0)
    {
        StepmotorY_dir=0;
        GPIO_SetBits(GPIOA,GPIO_Pin_5);
    }
    else
    {
        TIM2->CCR4=0;
    }
    if(Speedy!=0)
    {
        TIM2->PSC=myabs(1000000/Speedy);
        TIM2->PSC=TIM2->PSC>10000?10000:TIM2->PSC;
    }
    if(Speedx!=0)
    {
        TIM3->PSC=myabs(1000000/Speedx);
        TIM3->PSC=TIM3->PSC>10000?10000:TIM3->PSC;
    }
    
}

char keyscan(void)
{
    if(!GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_12) && keypresstime[0]<=longpresstime)keypresstime[0]++;//
    else if(!GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_12) && keypresstime[0]>longpresstime){keypresstime[0]++;return 5;}//长按中
    else if(GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_12) && keypresstime[0]>longpresstime){keypresstime[0]=0;return 9;}//长按后松开
    else if(GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_12) && keypresstime[0]>shortpresstime){keypresstime[0]=0;return 1;}//短按后松开

    else if(!GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_13) && keypresstime[1]<=longpresstime)keypresstime[1]++;//??
    else if(!GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_13) && keypresstime[1]>longpresstime){keypresstime[1]++;return 6;}//???
    else if(GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_13) && keypresstime[1]>longpresstime){keypresstime[1]=0;return 10;}//?????
    else if(GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_13) && keypresstime[1]>shortpresstime){keypresstime[1]=0;return 2;}//?????

    else if(!GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_14) && keypresstime[2]<=longpresstime)keypresstime[2]++;//??
    else if(!GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_14) && keypresstime[2]>longpresstime){keypresstime[2]++;return 7;}//???
    else if(GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_14) && keypresstime[2]>longpresstime){keypresstime[2]=0;return 11;}//?????
    else if(GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_14) && keypresstime[2]>shortpresstime){keypresstime[2]=0;return 3;}//?????

    else if(!GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_15) && keypresstime[3]<=longpresstime)keypresstime[3]++;//??
    else if(!GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_15) && keypresstime[3]>longpresstime){keypresstime[3]++;return 8;}//???
    else if(GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_15) && keypresstime[3]>longpresstime){keypresstime[3]=0;return 12;}//?????
    else if(GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_15) && keypresstime[3]>shortpresstime){keypresstime[3]=0;return 4;}//?????
    else keypresstime[0]=0,keypresstime[1]=0,keypresstime[2]=0,keypresstime[3]=0;
    return 0;
}



void Rect_record(void)
{
    Rect_recordtimes++;
    for(u8 i=0;i<4;i++)
    {
        Sum_rectpoint[i][0]+=Rect_point[i][0];
        Real_rectpoint[i][0]=(float)Sum_rectpoint[i][0]/(float)Rect_recordtimes;
        Sum_rectpoint[i][1]+=Rect_point[i][1];
        Real_rectpoint[i][1]=(float)Sum_rectpoint[i][1]/(float)Rect_recordtimes;
    }
}


void Goback_origin(void)
{
    StepmotorY_en=1;
    StepmotorX_en=1;
    if(StepYnum>0)
    {
        StepmotorY_dir=1;
    }
    else
    {
        StepmotorY_dir=0;
    }

    if(StepXnum>0)
    {
        StepmotorX_dir=1;
    }
    else
    {
        StepmotorX_dir=0;
    }

//    if(abs(StepXnum)<1)StepmotorX_en=0;
//    if(abs(StepYnum)<10)StepmotorY_en=0;
    if(StepXnum==0)StepmotorX_en=0;
    if(StepYnum==0)StepmotorY_en=0;
    if(!StepmotorX_en && !StepmotorY_en)
    {
        Flag_stepreset=0;
    }
}


void Step_drawrect(void)
{
    if(Flag_getkp)
    {
        Kp_move=((float)(Point[Points_choose][Draw_state][0]-Point[Points_choose][Draw_state-1][0]))/
                ((float)(Point[Points_choose][Draw_state][1]-Point[Points_choose][Draw_state-1][1]));
        Flag_getkp=0;
    }
    StepmotorY_en=1;
    StepmotorX_en=1;

    if(myabsf(Kp_move)<=1?1:Point[Points_choose][Draw_state][1]-Point[Points_choose][Draw_state-1][1]>0?CurY_num<CurX_num/Kp_move:-Kp_move*CurX_num>CurY_num)
    {
        StepmotorY_en=1;
    }
    else
    {
        StepmotorY_en=0;
    }

    if(myabsf(Kp_move)>=1?1:Point[Points_choose][Draw_state][0]-Point[Points_choose][Draw_state-1][0]>0?CurX_num<Kp_move*CurY_num:CurX_num>Kp_move*CurY_num)
    {
        StepmotorX_en=1;
    }
    else
    {
        StepmotorX_en=0;
    }

    if(Point[Points_choose][Draw_state-1][1]>Point[Points_choose][Draw_state][1])
    {
        StepmotorY_dir=1;
    }
    else
    {
        StepmotorY_dir=0;
    }

    if(Point[Points_choose][Draw_state-1][0]>Point[Points_choose][Draw_state][0])
    {
        StepmotorX_dir=1;
    }
    else
    {
        StepmotorX_dir=0;
    }

    if(StepXnum==Point[Points_choose][Draw_state][0])StepmotorX_en=0;
    if(StepYnum==Point[Points_choose][Draw_state][1])StepmotorY_en=0;
    if(StepXnum==Point[Points_choose][Draw_state][0] && StepYnum==Point[Points_choose][Draw_state][1])
    {
        CurX_num=0;
        CurY_num=0;
        Flag_getkp=1;
        Draw_state++;
    }
    if(Draw_state==6)
    {
        Flag_getkp=1;
        Flag_draw=0;
        Draw_state=1;
    }
}

void myStepmotor_control(void)
{
    if(StepmotorX_dir)
    {
        GPIO_ResetBits(GPIOC,GPIO_Pin_4);
    }
    else
    {
        GPIO_SetBits(GPIOC,GPIO_Pin_4);
    }
    
    if(StepmotorY_dir)
    {
        GPIO_ResetBits(GPIOA,GPIO_Pin_5);
    }
    else
    {
        GPIO_SetBits(GPIOA,GPIO_Pin_5);
    }
    
    if(StepmotorY_en)
    {
        TIM2->CCR4=499;
    }
    else
    {
        TIM2->CCR4=0;
    }
    
    if(StepmotorX_en)
    {
        TIM3->CCR4=499;
    }
    else
    {
        TIM3->CCR4=0;
    }
    
}

void Count_step(void)
{
    if(TIM3->CNT>499 && TIM3->CCR4==499 && Flag_countx)
    {
        Flag_countx=0;
        if(StepmotorX_dir)
        {
            StepXnum--;
            CurX_num--;
        }
        else
        {
            StepXnum++; 
            CurX_num++;
        }
    }
    else if(TIM3->CNT<499)
    {
        Flag_countx=1;
    }
    
    if(TIM2->CNT>499 && TIM2->CCR4==499 && Flag_county)
    {
        Flag_county=0;
        if(StepmotorY_dir)
        {
            StepYnum++;
            CurY_num++;
        }
        else
        {
            StepYnum--; 
            CurY_num--;
        }
    }
    else if(TIM2->CNT<499)
    {
        Flag_county=1;
    }
}

void Step_motor2D(void)
{
    static u8 Pwm_stateX=0,Pwm_stateY=0;
    
    
    if(StepmotorX_en)
    {
        Pwm_stateX=!Pwm_stateX;
        if(StepmotorX_dir)
        {
            GPIO_ResetBits(GPIOC,GPIO_Pin_4);
            if(Flag_steprecord)
            {
                CurX_num--;
                StepXnum--;
            }
        }
        else
        {
            GPIO_SetBits(GPIOC,GPIO_Pin_4);
            if(Flag_steprecord)
            {
                CurX_num++;
                StepXnum++;
            }
        }
    }

    if(StepmotorY_en)
    {
        Pwm_stateY=!Pwm_stateY;
        if(StepmotorY_dir)
        {
            GPIO_ResetBits(GPIOA,GPIO_Pin_5);
            if(Flag_steprecord)
            {
                CurY_num--;
                StepYnum--;
            }
        }
        else
        {
            if(Flag_steprecord)
            {
                CurY_num++;
                StepYnum++;
            }
            GPIO_SetBits(GPIOA,GPIO_Pin_5);
        }
    }

    myGPIO_set(GPIOB,GPIO_Pin_1,Pwm_stateX);
    myGPIO_set(GPIOA,GPIO_Pin_3,Pwm_stateY);

}

void myGPIO_set(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,u8 state)
{
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GET_GPIO_PIN(GPIO_Pin));
  assert_param(IS_GPIO_BIT_ACTION(BitVal)); 
  
  if(state)
  {
    GPIOx->BSRR = GPIO_Pin;
  }
  else
  {
    GPIOx->BRR = GPIO_Pin;
  }
}






