#include "motor.h"

int32 Length_L,Length_R;

int Measure_Speed_L=0,Measure_Speed_R=0;
int16 Speed_Right_L=0,Speed_Right_R=0;

int Motor_Speed_Max=0 ; //电机最大速度
int Motor_Speed_Min=0 ;  //电机最小速度
int Speed_Nor_Low=43,Speed_Nor_High=45;
int Speed_Up=10,Speed_Down=1;
float Distance=0;





void Motor_init()
{
	
	
}
void Speed_Change(int sta)//0减速,1常速,2加速,3全速
{
	if(Ramp_Flag)
		sta=0;
	switch(sta)
	{
//		case -2:Motor_Speed_Max = Speed_Nor_High - (int)(1.0f * Speed_Down); Motor_Speed_Min = Speed_Nor_Low - (int)(1.0f * Speed_Down);break;
//		
//		case -1:Motor_Speed_Max = Speed_Nor_High - (int)(0.3f * Speed_Down); Motor_Speed_Min = Speed_Nor_Low - (int)(0.3f * Speed_Down);break;
		
		case 0:Motor_Speed_Max = Speed_Nor_High; Motor_Speed_Min = Speed_Nor_Low; break;
		
		case 1:Motor_Speed_Max = Speed_Nor_High + (int)(0.5f * Speed_Up); Motor_Speed_Min = Speed_Nor_Low + (int)(0.5f * Speed_Up); break;
		
		case 2:Motor_Speed_Max = Speed_Nor_High + (int)(1.0f * Speed_Up); Motor_Speed_Min = Speed_Nor_Low + (int)(1.0f * Speed_Up); break;
		
		case 3:Motor_Speed_Max = Speed_Nor_High + (int)(2.5f * Speed_Up); Motor_Speed_Min = Speed_Nor_Low + (int)(2.5f * Speed_Up); break;
	}
}


float Err_k=-0.5;
void Speed_Measure()
{
Measure_Speed_L=ftm_quad_get(ftm2);   //左轮
ftm_quad_clean(ftm2);
Measure_Speed_R=-ftm_quad_get(ftm1);   //右轮
ftm_quad_clean(ftm1);

Target_Speed = (int)(Err_k * abs(Center_Deviation) + Motor_Speed_Max + 0.5f);

if(Target_Speed<Motor_Speed_Min)Target_Speed = Motor_Speed_Min;     //最小速度限制

}


void MOTOR_Control(int32 L_Duty,int32 R_Duty)
{
  if(L_Duty<0)
  {
    L_Duty= - L_Duty;
    gpio_set (C1, 0);
  }
  else
    gpio_set (C1, 1);
  
  if(R_Duty<0)
  {
    R_Duty= - R_Duty;
    gpio_set (C3, 1);
  }
  else
    gpio_set (C3, 0);
  
  ftm_pwm_duty(ftm0,ftm_ch1,L_Duty);   
  ftm_pwm_duty(ftm0,ftm_ch5,R_Duty);   
}


int32 range_protect(int32 duty, int32 min, int32 max)//限幅保护
{
	if (duty >= max)
	{
		return max;
        }
	if (duty <= min)
	{
		return min;
	}
	else
	{
		return duty;
	}
}
