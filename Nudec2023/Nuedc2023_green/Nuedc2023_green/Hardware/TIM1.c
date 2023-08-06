#include "TIM1.h"
#include "usart.h"



u8   Imu_t=0,Uart_t=0,Pid_t=0,
        Pwm_state=1,
        StepmotorX_dir=0,StepmotorY_dir=0,
        StepmotorX_en=0,StepmotorY_en=0,
        Flag_steprecord=0,
        Flag_stepreset=0,Pointg_X,Pointg_Y,Pointr_X,Pointr_Y,Point_Xlast,Point_Ylast,
        Draw_state=0,Flag_getkp=1,Flag_draw=0,
        Rect_point[4][2],Rect_pointnum=20,Pointfollow_num=0,
        Flag_rectrecord=0,Flag_follow=0,Motor_choose=0,AimX=75,AimY=60;

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
        CurX_num,CurY_num,Point[7][2]={{0,0},{-797,-483},{-828,776},{790,756},{784,-523},{-760,-483},{0,0}},
        keypresstime[4],keystate,StepXnum=0,StepYnum=0,uart_t=0,Error_X,Error_Y;

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
    static u8 Flag_countx=1,Flag_county=1;
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) 
	{
        Key_state=keyscan();
        Key_set();
        
        if(Flag_follow)
        {
            TIM2->CCR4=499;
            TIM3->CCR4=499;
        }
        else
        {
            TIM2->CCR4=0;
            TIM3->CCR4=0;
        }
        
        if(++uart_t>100)
        {
            uart_t=0;
//            printf("%d\n",Flag_follow);
            printf("%d,%d,%d,%d,%d,%d,%d\n",Pointr_X,Pointr_Y,Pointg_X,Pointg_Y,Error_X,Error_Y,Key_state);
//            printf("%.2f,%.2f,%d,%d\n",Servo_x.Out,Servo_y.Out,TIM2->PSC,TIM3->PSC);
//            printf("%d,%d,%d,%d,%d,%d\n",Flag_follow,TIM2->CNT,StepYnum,StepXnum,Flag_countx,TIM2->CCR4);
//            printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%.2f\n",StepmotorX_dir,StepmotorY_dir,StepmotorX_en,StepmotorY_en,StepXnum,StepYnum,CurX_num,CurY_num,
//            Draw_state,Kp_move);
            if(myabs(Error_X)<6 && myabs(Error_Y)<6)
            {
                GPIO_ResetBits(GPIOB,GPIO_Pin_10);
            }
            else
            {
                GPIO_SetBits(GPIOB,GPIO_Pin_10);
            }
        }
        
       if(++Pid_t>=10 && Flag_follow)
       {
            Pid_t=0;
            Error_X=Pointr_X-AimX;
            Error_Y=Pointr_Y-AimY;
            Fox_Pid_Position((float)0.0f,(float)-Error_X,&Servo_x);
            Fox_Pid_Position((float)0.0f,(float)-Error_Y,&Servo_y);
           Stepmotor_control((int)Servo_x.Out,(int)Servo_y.Out);
        }
		 TIM_ClearITPendingBit(TIM1, TIM_IT_Update  ); 
	}
}


void Pid_init(void)
{
    Servo_x.Kp=400;
    Servo_x.Ki=4;
    Servo_x.Kd=0;
    Servo_x.Integral_lim=100;
    Servo_x.Out_lim=9500;
    Servo_x.Out=0;

    Servo_y.Kp=400;
    Servo_y.Ki=4;
    Servo_y.Kd=0;
    Servo_y.Integral_lim=100;
    Servo_y.Out_lim=9500;
    Servo_y.Out=0;
}



//void Get_error(void)
//{
//    static int Check_t=0;
//    Error_X=Rect_usepoint[Pointfollow_num][0];
//    Error_Y=Rect_usepoint[Pointfollow_num][1];
//    if(myabs(Error_X-Point_X)<2 && myabs(Error_Y-Point_Y)<2 && ++Check_t>10)
//    {
//        Check_t=0;
//        Pointfollow_num++;
//    }
//    if(Pointfollow_num>Rect_pointnum)
//    {
//        Pointfollow_num=0;
//        Flag_follow=0;
//        Error_X=0;
//        Error_Y=0;
//        TIM2->CCR4=0;
//        TIM3->CCR4=0;
//    }
//}


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
    
    if(Speedx<0)
    {
        GPIO_ResetBits(GPIOC,GPIO_Pin_4);
    }
    else if(Speedx>0)
    {
        GPIO_SetBits(GPIOC,GPIO_Pin_4);
    }
    else
    {
        TIM3->CCR4=0;
    }
    
    if(Speedy>0)
    {
        GPIO_ResetBits(GPIOA,GPIO_Pin_5);
    }
    else if(Speedy<0)
    {
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

void Key_set(void)
{
    static u8 Flag_choose=0;
    if(Key_state==4)
    {
        Flag_follow=!Flag_follow;
    }
    else if(Key_state==1)
    {
        Flag_choose=!Flag_choose;
    }
    else if(Key_state==2)
    {
        if(Flag_choose)AimX++;
        else if(!Flag_choose)AimY++;
    }
    else if(Key_state==3)
    {
        if(Flag_choose)AimX--;
        else if(!Flag_choose)AimY--;
    }
    
    
//         if(Key_state==1)Motor_choose=!Motor_choose;
//    else if(Key_state==2 && Motor_choose==0)
//    {
//        StepmotorX_en =!StepmotorX_en;
//    }
//    else if(Key_state==2 && Motor_choose!=0)
//    {
//        StepmotorY_en =!StepmotorY_en;
//    }
//    else if(Key_state==3 && Motor_choose==0)
//    {
//        StepmotorX_dir=!StepmotorX_dir;
//    }
//    else if(Key_state==3 && Motor_choose!=0)
//    {
//        StepmotorY_dir=!StepmotorY_dir;
//    }
//    else if(Key_state==9)
//    {
//        if(Flag_steprecord)
//        {
//            Flag_steprecord=0;
//        }
//        else
//        {
//            StepXnum=0;
//            StepYnum=0;
//            Flag_steprecord=1;
//        }
//    }
//    else if(Key_state==10)
//    {
//        Flag_stepreset=1;
//    }
//    else if(Key_state==11)
//    {
//        Flag_draw=1;
//        Draw_state=1;
//    }
    
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
    if(StepYnum<0)
    {
        StepmotorY_dir=1;
    }
    else
    {
        StepmotorY_dir=0;
    }

    if(StepXnum<0)
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
        Kp_move=((float)(Point[Draw_state][0]-Point[Draw_state-1][0]))/
                ((float)(Point[Draw_state][1]-Point[Draw_state-1][1]));
        Flag_getkp=0;
    }
    StepmotorY_en=1;
    StepmotorX_en=1;

    if(myabsf(Kp_move)<=1?1:Point[Draw_state][1]-Point[Draw_state-1][1]>0?CurY_num<CurX_num/Kp_move:-Kp_move*CurX_num>CurY_num)
    {
        StepmotorY_en=1;
    }
    else
    {
        StepmotorY_en=0;
    }

    if(myabsf(Kp_move)>=1?1:Point[Draw_state][0]-Point[Draw_state-1][0]>0?CurX_num<Kp_move*CurY_num:CurX_num>Kp_move*CurY_num)
    {
        StepmotorX_en=1;
    }
    else
    {
        StepmotorX_en=0;
    }

    if(Point[Draw_state-1][1]<Point[Draw_state][1])
    {
        StepmotorY_dir=1;
    }
    else
    {
        StepmotorY_dir=0;
    }

    if(Point[Draw_state-1][0]<Point[Draw_state][0])
    {
        StepmotorX_dir=1;
    }
    else
    {
        StepmotorX_dir=0;
    }

    if(StepXnum==Point[Draw_state][0])StepmotorX_en=0;
    if(StepYnum==Point[Draw_state][1])StepmotorY_en=0;
    if(StepXnum==Point[Draw_state][0] && StepYnum==Point[Draw_state][1])
    {
        CurX_num=0;
        CurY_num=0;
        Flag_getkp=1;
        Draw_state++;
    }
    if(Draw_state==7)
    {
        Flag_getkp=1;
        Flag_draw=0;
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


