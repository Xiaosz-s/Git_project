#include "mykey.h"

u8 Keymode=0,Triggermode=0,Flag_adjust=0,Key_t=0;

void Key_set(void)
{
    if(Key_state==1)Keymode++;
    
    /*********************************DIR EN*****************************************/
    
    else if(Key_state==2 && Keymode==0 && Flag_adjust==0)StepmotorX_en=!StepmotorX_en;
    else if(Key_state==2 && Keymode==1 && Flag_adjust==0)StepmotorY_en=!StepmotorY_en;
    
    else if(Key_state==3 && Keymode==0 && Flag_adjust==0)StepmotorX_dir=!StepmotorX_dir;
    else if(Key_state==3 && Keymode==1 && Flag_adjust==0)StepmotorY_dir=!StepmotorY_dir;
    
    /*********************************TASK2*****************************************/
    
    else if(Key_state==2 && Keymode==0 && Flag_adjust==1)Point[0][1][0]+=10,Point[0][5][0]+=10;
    else if(Key_state==2 && Keymode==1 && Flag_adjust==1)Point[0][1][1]+=10,Point[0][5][1]+=10;
    else if(Key_state==2 && Keymode==2 && Flag_adjust==1)Point[0][2][0]+=10;
    else if(Key_state==2 && Keymode==3 && Flag_adjust==1)Point[0][2][1]+=10;
    else if(Key_state==2 && Keymode==4 && Flag_adjust==1)Point[0][3][0]+=10;
    else if(Key_state==2 && Keymode==5 && Flag_adjust==1)Point[0][3][1]+=10;
    else if(Key_state==2 && Keymode==6 && Flag_adjust==1)Point[0][4][0]+=10;
    else if(Key_state==2 && Keymode==7 && Flag_adjust==1)Point[0][4][1]+=10;
    
    else if(Key_state==3 && Keymode==0 && Flag_adjust==1)Point[0][1][0]-=10,Point[0][5][0]-=10;
    else if(Key_state==3 && Keymode==1 && Flag_adjust==1)Point[0][1][1]-=10,Point[0][5][1]-=10;
    else if(Key_state==3 && Keymode==2 && Flag_adjust==1)Point[0][2][0]-=10;
    else if(Key_state==3 && Keymode==3 && Flag_adjust==1)Point[0][2][1]-=10;
    else if(Key_state==3 && Keymode==4 && Flag_adjust==1)Point[0][3][0]-=10;
    else if(Key_state==3 && Keymode==5 && Flag_adjust==1)Point[0][3][1]-=10;
    else if(Key_state==3 && Keymode==6 && Flag_adjust==1)Point[0][4][0]-=10;
    else if(Key_state==3 && Keymode==7 && Flag_adjust==1)Point[0][4][1]-=10;
    
    /*********************************TASK3*****************************************/
    
    else if(Key_state==2 && Keymode==0 && Flag_adjust==2)Point[1][1][0]+=10,Point[1][5][0]+=10;
    else if(Key_state==2 && Keymode==1 && Flag_adjust==2)Point[1][1][1]+=10,Point[1][5][1]+=10;
    else if(Key_state==2 && Keymode==2 && Flag_adjust==2)Point[1][2][0]+=10;
    else if(Key_state==2 && Keymode==3 && Flag_adjust==2)Point[1][2][1]+=10;
    else if(Key_state==2 && Keymode==4 && Flag_adjust==2)Point[1][3][0]+=10;
    else if(Key_state==2 && Keymode==5 && Flag_adjust==2)Point[1][3][1]+=10;
    else if(Key_state==2 && Keymode==6 && Flag_adjust==2)Point[1][4][0]+=10;
    else if(Key_state==2 && Keymode==7 && Flag_adjust==2)Point[1][4][1]+=10;
    
    else if(Key_state==3 && Keymode==0 && Flag_adjust==2)Point[1][1][0]-=10,Point[1][5][0]-=10;
    else if(Key_state==3 && Keymode==1 && Flag_adjust==2)Point[1][1][1]-=10,Point[1][5][1]-=10;
    else if(Key_state==3 && Keymode==2 && Flag_adjust==2)Point[1][2][0]-=10;
    else if(Key_state==3 && Keymode==3 && Flag_adjust==2)Point[1][2][1]-=10;
    else if(Key_state==3 && Keymode==4 && Flag_adjust==2)Point[1][3][0]-=10;
    else if(Key_state==3 && Keymode==5 && Flag_adjust==2)Point[1][3][1]-=10;
    else if(Key_state==3 && Keymode==6 && Flag_adjust==2)Point[1][4][0]-=10;
    else if(Key_state==3 && Keymode==7 && Flag_adjust==2)Point[1][4][1]-=10;
    
    /*********************************TASK4*****************************************/
    
    else if(Key_state==2 && Keymode==0 && Flag_adjust==3)Rect_usepoint[0][0]+=1;
    else if(Key_state==2 && Keymode==1 && Flag_adjust==3)Rect_usepoint[0][1]+=1;
    else if(Key_state==2 && Keymode==2 && Flag_adjust==3)Rect_usepoint[1][0]+=1;
    else if(Key_state==2 && Keymode==3 && Flag_adjust==3)Rect_usepoint[1][1]+=1;
    else if(Key_state==2 && Keymode==4 && Flag_adjust==3)Rect_usepoint[2][0]+=1;
    else if(Key_state==2 && Keymode==5 && Flag_adjust==3)Rect_usepoint[2][1]+=1;
    else if(Key_state==2 && Keymode==6 && Flag_adjust==3)Rect_usepoint[3][0]+=1;
    else if(Key_state==2 && Keymode==7 && Flag_adjust==3)Rect_usepoint[3][1]+=1;
    
    else if(Key_state==3 && Keymode==0 && Flag_adjust==3)Rect_usepoint[0][0]-=1;
    else if(Key_state==3 && Keymode==1 && Flag_adjust==3)Rect_usepoint[0][1]-=1;
    else if(Key_state==3 && Keymode==2 && Flag_adjust==3)Rect_usepoint[1][0]-=1;
    else if(Key_state==3 && Keymode==3 && Flag_adjust==3)Rect_usepoint[1][1]-=1;
    else if(Key_state==3 && Keymode==4 && Flag_adjust==3)Rect_usepoint[2][0]-=1;
    else if(Key_state==3 && Keymode==5 && Flag_adjust==3)Rect_usepoint[2][1]-=1;
    else if(Key_state==3 && Keymode==6 && Flag_adjust==3)Rect_usepoint[3][0]-=1;
    else if(Key_state==3 && Keymode==7 && Flag_adjust==3)Rect_usepoint[3][1]-=1;
    
    /*********************************TASK CHOOSE*****************************************/
    
    else if(Key_state==4)Triggermode++;
    
    /*********************************TASK2 quick*****************************************/
    
    else if(Key_state==6 && Keymode==0 && Flag_adjust==1 && ++Key_t>=25)Point[0][1][0]+=1,Point[0][5][0]+=1,Key_t=0;
    else if(Key_state==6 && Keymode==1 && Flag_adjust==1 && ++Key_t>=25)Point[0][1][1]+=1,Point[0][5][1]+=1,Key_t=0;
    else if(Key_state==6 && Keymode==2 && Flag_adjust==1 && ++Key_t>=25)Point[0][2][0]+=1,Key_t=0;
    else if(Key_state==6 && Keymode==3 && Flag_adjust==1 && ++Key_t>=25)Point[0][2][1]+=1,Key_t=0;
    else if(Key_state==6 && Keymode==4 && Flag_adjust==1 && ++Key_t>=25)Point[0][3][0]+=1,Key_t=0;
    else if(Key_state==6 && Keymode==5 && Flag_adjust==1 && ++Key_t>=25)Point[0][3][1]+=1,Key_t=0;
    else if(Key_state==6 && Keymode==6 && Flag_adjust==1 && ++Key_t>=25)Point[0][4][0]+=1,Key_t=0;
    else if(Key_state==6 && Keymode==7 && Flag_adjust==1 && ++Key_t>=25)Point[0][4][1]+=1,Key_t=0;
    
    else if(Key_state==7 && Keymode==0 && Flag_adjust==1 && ++Key_t>=25)Point[0][1][0]-=1,Point[0][5][0]-=1,Key_t=0;
    else if(Key_state==7 && Keymode==1 && Flag_adjust==1 && ++Key_t>=25)Point[0][1][1]-=1,Point[0][5][1]-=1,Key_t=0;
    else if(Key_state==7 && Keymode==2 && Flag_adjust==1 && ++Key_t>=25)Point[0][2][0]-=1,Key_t=0;
    else if(Key_state==7 && Keymode==3 && Flag_adjust==1 && ++Key_t>=25)Point[0][2][1]-=1,Key_t=0;
    else if(Key_state==7 && Keymode==4 && Flag_adjust==1 && ++Key_t>=25)Point[0][3][0]-=1,Key_t=0;
    else if(Key_state==7 && Keymode==5 && Flag_adjust==1 && ++Key_t>=25)Point[0][3][1]-=1,Key_t=0;
    else if(Key_state==7 && Keymode==6 && Flag_adjust==1 && ++Key_t>=25)Point[0][4][0]-=1,Key_t=0;
    else if(Key_state==7 && Keymode==7 && Flag_adjust==1 && ++Key_t>=25)Point[0][4][1]-=1,Key_t=0;
    
    /*********************************TASK3 quick*****************************************/
    
    else if(Key_state==6 && Keymode==0 && Flag_adjust==2 && ++Key_t>=25)Point[1][1][0]+=1,Point[1][5][0]+=1,Key_t=0;
    else if(Key_state==6 && Keymode==1 && Flag_adjust==2 && ++Key_t>=25)Point[1][1][1]+=1,Point[1][5][1]+=1,Key_t=0;
    else if(Key_state==6 && Keymode==2 && Flag_adjust==2 && ++Key_t>=25)Point[1][2][0]+=1,Key_t=0;
    else if(Key_state==6 && Keymode==3 && Flag_adjust==2 && ++Key_t>=25)Point[1][2][1]+=1,Key_t=0;
    else if(Key_state==6 && Keymode==4 && Flag_adjust==2 && ++Key_t>=25)Point[1][3][0]+=1,Key_t=0;
    else if(Key_state==6 && Keymode==5 && Flag_adjust==2 && ++Key_t>=25)Point[1][3][1]+=1,Key_t=0;
    else if(Key_state==6 && Keymode==6 && Flag_adjust==2 && ++Key_t>=25)Point[1][4][0]+=1,Key_t=0;
    else if(Key_state==6 && Keymode==7 && Flag_adjust==2 && ++Key_t>=25)Point[1][4][1]+=1,Key_t=0;
    
    else if(Key_state==7 && Keymode==0 && Flag_adjust==2 && ++Key_t>=25)Point[1][1][0]-=1,Point[1][5][0]-=1,Key_t=0;
    else if(Key_state==7 && Keymode==1 && Flag_adjust==2 && ++Key_t>=25)Point[1][1][1]-=1,Point[1][5][1]-=1,Key_t=0;
    else if(Key_state==7 && Keymode==2 && Flag_adjust==2 && ++Key_t>=25)Point[1][2][0]-=1,Key_t=0;
    else if(Key_state==7 && Keymode==3 && Flag_adjust==2 && ++Key_t>=25)Point[1][2][1]-=1,Key_t=0;
    else if(Key_state==7 && Keymode==4 && Flag_adjust==2 && ++Key_t>=25)Point[1][3][0]-=1,Key_t=0;
    else if(Key_state==7 && Keymode==5 && Flag_adjust==2 && ++Key_t>=25)Point[1][3][1]-=1,Key_t=0;
    else if(Key_state==7 && Keymode==6 && Flag_adjust==2 && ++Key_t>=25)Point[1][4][0]-=1,Key_t=0;
    else if(Key_state==7 && Keymode==7 && Flag_adjust==2 && ++Key_t>=25)Point[1][4][1]-=1,Key_t=0;
    
    /*********************************ADJUST*****************************************/
    
    else if(Key_state==9)Flag_adjust++;
    
    /*********************************TASK TRIGGER*****************************************/
    
    else if(Key_state==12 && Triggermode==0)
    {
        Flag_rectrecord=!Flag_rectrecord;           //Record RECT
        if(!Flag_rectrecord)
        {
            Flag_IOPWM=0;
            TIM2_PWM_Init(999,719);
            TIM3_PWM_Init(999,719);
            Scan_point();
        }
    }
    else if(Key_state==12 && Triggermode==1)
    {
        Flag_follow=!Flag_follow;                   //PID RECT
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
    }
    else if(Key_state==12 && Triggermode==2)
    {
        if(Flag_steprecord)                         //Record center
        {
            Flag_steprecord=0;
        }
        else
        {
            StepXnum=0;
            StepYnum=0;
            Flag_steprecord=1;
        }
    }
    else if(Key_state==12 && Triggermode==3)
    {
        Flag_stepreset=1;                           //Reset / go center
    }
    else if(Key_state==12 && Triggermode==4)
    {
        Flag_draw=1;                                //Draw RECT
        Draw_state=1;
        Points_choose=0;
        Move_speed=0;
    }
    else if(Key_state==12 && Triggermode==5)
    {
        Flag_draw=!Flag_draw;                                //Draw RECT
        if(!Flag_draw)
        {
            StepmotorX_en=0;
            StepmotorY_en=0;
        }
        Points_choose=1;
//        Draw_state=1;
        Move_speed=2;
    }
    
    if(!Flag_adjust)Keymode=Keymode>1?0:Keymode;
    else if(Flag_adjust)Keymode=Keymode>7?0:Keymode;
    Triggermode=Triggermode>5?0:Triggermode;
    Flag_adjust=Flag_adjust>3?0:Flag_adjust;
}


