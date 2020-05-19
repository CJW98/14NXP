#include "Control.h"
#include "math.h"
int32 MOTOR_Duty_Left  = 10;
int32 MOTOR_Duty_Right = 10;
int32 Direction_PWM = 0;
int32 Direction_Speed = 0;
int Speed[2];
int32 Radius,Speed_Min;
extern float sys_time;
char Turn_Flag=0;


char Target_Flag=0,              //标志位
Avoid_Flag;              //标志位

uint8 Ob_NUM=0;        //壁障个数
char ob_Flag=0;

char ob_Speed_Flag=0;
char ob_Speed_Flag2=0;


/***避障参数****/
float Obstacle_Angle[2]={30,50},              //第一次偏转角度  第二次角度
Obstacle_Line=0,                   //第一次角度的路径
Obstacle_Distance,        //需要走的距离
Attitude_Angle_Z;         //当前角度（角速度积分）

int32 Theory_Duty;
uint16_t DISS=0;
/***flash参数***/
uint8_t Ramp_DISTANCE=200;
uint8_t ceshi_data[4]={0,25,35,0};  //测距监测   起跑线阈值  坡道电感阈值
void Obstacle_Control()
{
	Speed_Measure();
	if(ceshi_data[0])
		DISS+=(Measure_Speed_L+Measure_Speed_R)/2*0.003*Ratio_Encoder;	
		
	jiaodu();
	Filter();
	if(Target_Flag==0)
	{

		ob_Speed_Flag=1;
		Attitude_Angle_Z+=Gyro_X*0.003;
		if(Attitude_Angle_Z>30)
		{
			ob_Speed_Flag=0;
			Attitude_Angle_Z=0;
			Target_Flag = 1;
			Turn_offset=0;
		}
		
	}
	
	if(Target_Flag==1)
	{    	  
		Turn_offset=0;		
		Obstacle_Distance+=(Measure_Speed_L+Measure_Speed_R)/2*0.003*Ratio_Encoder;	
		if(Obstacle_Distance>=Obstacle_Line)
		{	
			Obstacle_Distance=0;
			Target_Flag=4;
		}

	}
	
	
	
	
	if(Target_Flag==4)
		
	{
		
		Attitude_Angle_Z+=Gyro_X*0.003;
		ob_Speed_Flag2=1;
		if(!Ob_NUM)
		{
			if(Attitude_Angle_Z<=-Obstacle_Angle[0])
			{
				//ob_Flag=1;
				Attitude_Angle_Z=0;
				ob_Speed_Flag2=0;
				Target_Flag = 5;
				Turn_offset=0;
				
			}
		}
		if(Ob_NUM)
		{
			if(Attitude_Angle_Z<=-Obstacle_Angle[1])
			{
				//ob_Flag=1;
				Attitude_Angle_Z=0;
				ob_Speed_Flag2=0;
				Target_Flag = 5;
				Turn_offset=0;
				
			}
		}
		
		
		
	}
	
	
	
	if(Target_Flag == 5 && ADC_End[1]>500 && ADC_End[4]>500)//&&(ADC_End[4]-ADC_End[1])*100/(ADC_End[4]+ADC_End[1]+ADC_End[5])>0
	{
		ob_Speed_Flag=1;
		Attitude_Angle_Z+=Gyro_X*0.003;
		if(Attitude_Angle_Z>=10)
		{
			Attitude_Angle_Z=0;						
			ob_Speed_Flag=0;
			Target_Flag=0;
			Obstacle_Flag = 0;
			Ob_NUM+=1;
			
		}
	}
	
	
	g_fDirectionControlOut=-PID_Realize(&Direct_In_PID, Direct_In, Turn_offset, 0)-Gyro_X*0.1;
	Speed[0]=40+g_fDirectionControlOut;  
	Speed[1]=40-g_fDirectionControlOut;
	if(ob_Speed_Flag)
	{
		Speed[0]=60;  
		Speed[1]=0;
		
	}
	if(ob_Speed_Flag2)
	{
		Speed[0]=0;  
		Speed[1]=60;
		
	}
	if(ob_Flag)
	{
		Speed[0]=0;  
		Speed[1]=0;
		
	}	
	Speed_Control();

	MOTOR_Duty_Left=g_fSpeedControlOut_L;
	MOTOR_Duty_Left = range_protect(MOTOR_Duty_Left, -800, 800);
	MOTOR_Duty_Right=g_fSpeedControlOut_R;
	MOTOR_Duty_Right = range_protect(MOTOR_Duty_Right, -800, 800);
	
	MOTOR_Control(  MOTOR_Duty_Left,  MOTOR_Duty_Right);	// 控制左右电机 g_fSpeedControlOut_L
//	
//	      if(Flag_9ms)
//	{
//	    
//	    Flag_9ms=0;	
//	  //  Speed_Measure();
//		Theory_Duty  =  PID_Realize(&Angle_Speed_PID, Angle_Speed, (int32)(GYRO_Real.Y*10), (int32)(Tar_Ang_Vel.Y));	// 计算直立PWM
//		Theory_Duty  = range_protect(Theory_Duty, -1200, 1200);
//	    
//	}
}


char Obstacle_Meeting=0;  
void  Run_Control()
{
	if(Obstacle_Flag)
	{
		Obstacle_Control();
	}
	else
		//	Control();
	{
		Master_Control();
	}
	
}



/***方案一****/



//
char Speed_jiajia;
char  Ramp_Flag=0;   //坡道标志位
float Ramp_Distance=0;
char Speed_decrease=0;  //减速
float meeting_distance=0;
void Control(void)
{
	float Radius_Speed;
	
	
	if(Flag_3ms)
	{
		Flag_3ms=0;
		jiaodu();
		Speed_Measure();
		
		if(Ramp_Flag)
		{
			Ramp_Distance+=(Measure_Speed_L+Measure_Speed_R)/2*0.003*Ratio_Encoder;  //壁障停车保护
			
			Speed_decrease=1;
			
			if(Ramp_Distance>=ring_distance[3])
			{
				Ramp_Distance=0;
				Ramp_Flag=0;
				Speed_decrease=0;
	
				
			}
			
		}
		
		
		
		
		
		/***摄像头****/
		//	Direction_Speed=PID_Realize(&Direct_In_PID, Direct_In, Center_Deviation, 0)+Gyro_X*0.1;
		//	Direction_Speed=range_protect(Direction_Speed, -200, 200);
		Dierction_Control();
		
		Radius_Speed = 50*Car_Wheelbase/100;
		
		if(abs(g_fDirectionControlOut)>Radius_Speed)
		{
			g_fDirectionControlOut = (g_fDirectionControlOut>0?1:-1) * (int16_t)Radius_Speed;
		}
		
		if(Speed_Up>1)
		{
			if(Speed_jiajia==1)
				Car_Up(1,0,10);
//			if(Speed_jiajia==2)
//				Car_Up(1,3,20);
			Speed[0]=Target_Speed+g_fDirectionControlOut;  //Speed_Set
			Speed[1]=Target_Speed-g_fDirectionControlOut;//Target_Speed
			
//			if(Disable)
//			{
//				Speed[0]=40+g_fDirectionControlOut;  //Speed_Set
//				Speed[1]=40-g_fDirectionControlOut;//Target_Speed			
//			}
			
		}
		else
		{
			Speed[0]=Speed_Set+g_fDirectionControlOut;  //Speed_Set
			Speed[1]=Speed_Set-g_fDirectionControlOut;//Target_Speed
		}	
		if(Speed_Flag==1)
		{
			Speed[0]=40+g_fDirectionControlOut;  
			Speed[1]=40-g_fDirectionControlOut;			
		}
		if(Speed_decrease)
		{
			Speed[0]=45+g_fDirectionControlOut;  
			Speed[1]=45-g_fDirectionControlOut;			
		}
		
		/*****升速********/
		if(Speed_Flag==2)
		{
			Speed[0]=45+g_fDirectionControlOut;  
			Speed[1]=45-g_fDirectionControlOut;			
		}
		
		if(Speed_Flag==3)
		{
			Speed[0]=55+g_fDirectionControlOut;  
			Speed[1]=55-g_fDirectionControlOut;			
		}
		
		//	
		if(ADC_End[1]<200&&ADC_End[4]<200)  //停车保护
			//if(stop_Flag||A_Reach_End)
		{
			Speed[0]=0;  
			Speed[1]=0;	
		}
		if(A_Reach_End)      //终点线
		{
			Speed[0]=0;   
			Speed[1]=0;
			
			if(B_Reach_End)
			{
				meeting_distance+=(Measure_Speed_L+Measure_Speed_R)/2*0.003*Ratio_Encoder;  //壁障停车保护
				Speed[0]=30+g_fDirectionControlOut;  
				Speed[1]=30-g_fDirectionControlOut;	
				if(meeting_distance>=60)
				{
					Speed[0]=0;  
					Speed[1]=0;		
					
				}
			}
		}
		
		if(	Speed[0]<0)
			Speed[0]=0;
		if(	Speed[1]<0)
			Speed[1]=0;

		Speed_Control(); 
		
		if(A_Reach_End)
		{
		if(AngleSpeed[0]<0)
		{
		Theory_Duty  =  PID_Realize(&Angle_Speed_N_PID, Angle_Speed_N, (int32)(AngleSpeed[0]*100), 0);	// 计算直立PWM
		Theory_Duty  = range_protect(Theory_Duty, -800, 800);
		}
		else
			Theory_Duty=0;
		MOTOR_Duty_Left=Theory_Duty+g_fSpeedControlOut_L;
		MOTOR_Duty_Left = range_protect(MOTOR_Duty_Left, -950, 950);
		MOTOR_Duty_Right=Theory_Duty+g_fSpeedControlOut_R;
		MOTOR_Duty_Right = range_protect(MOTOR_Duty_Right, -950, 950);
		MOTOR_Control(  MOTOR_Duty_Left,  MOTOR_Duty_Right);	// 控制左右电机 g_fSpeedControlOut_L
		}
		else
		{
		MOTOR_Duty_Left=g_fSpeedControlOut_L;
		MOTOR_Duty_Left = range_protect(MOTOR_Duty_Left, -950, 950);
		MOTOR_Duty_Right=g_fSpeedControlOut_R;
		MOTOR_Duty_Right = range_protect(MOTOR_Duty_Right, -950, 950);
		MOTOR_Control(  MOTOR_Duty_Left,  MOTOR_Duty_Right);	// 控制左右电机 g_fSpeedControlOut_L
		}
	}
	
}

#define  No_Meeting        0
#define  Meeting_One       1
#define  Meeting_Circle    2
#define  Meeting_In        3
#define  Meeting_Out       4
#define  Real_Meeting_Out  5




uint8 Meeting_state=0;
float journey=0;
void Master_Control(void)
{
	
	switch(Meeting_state)
	{
		case No_Meeting:  
		Control();
		if(Broken_Flag)
			Meeting_state=Meeting_One;
		else
			Meeting_state=No_Meeting;
		
		break;
		
		
		case Meeting_One:            //第一阶段減速階段
		//Beep=2;
		Circle_Flag=1;
		Speed_Measure();
		journey+=(Measure_Speed_L+Measure_Speed_R)/2*0.003*Ratio_Encoder;
		Dierction_Control();
		Speed[0]=30+g_fDirectionControlOut;   
		Speed[1]=30-g_fDirectionControlOut;
		if(journey>10)
		{
			Speed[0]=20+g_fDirectionControlOut;   
			Speed[1]=20-g_fDirectionControlOut;
			if(journey>SAIDAO_DATA[3])
			{
			journey=0;
			Speed[0]=+g_fDirectionControlOut;   
			Speed[1]=-g_fDirectionControlOut; 			
			Meeting_state=Meeting_In;
			}
		}
		Speed_Control();
		MOTOR_Duty_Left = g_fSpeedControlOut_L;
		MOTOR_Duty_Left = range_protect(MOTOR_Duty_Left, -700, 700);
		MOTOR_Duty_Right = g_fSpeedControlOut_R;
		MOTOR_Duty_Right = range_protect(MOTOR_Duty_Right, -700, 700);
		MOTOR_Control(  MOTOR_Duty_Left,  MOTOR_Duty_Right);	// 控制左右电机 g_fSpeedControlOut_L
		break;
		
		
		
		case Meeting_In:          //停車转身階段
		// if(1)
		Meeting();
		//	 else
		//	 {
		//	 Speed_Measure();  
		//	 ADC_Read(); 
		//	 Direction_Speed=PID_Realize(&Direct_In_PID, Direct_In, atan(Turn_offset)*100, 0)+Gyro_X*0.1;
		//	 Speed[0]=0-Direction_Speed;   
		//	 Speed[1]=0+Direction_Speed;
		//	 Speed_Control();
		//	 MOTOR_Duty_Left=g_fSpeedControlOut_L;
		//	 MOTOR_Duty_Left = range_protect(MOTOR_Duty_Left, -300, 300);
		//	 MOTOR_Duty_Right=g_fSpeedControlOut_R;
		//	 MOTOR_Duty_Right = range_protect(MOTOR_Duty_Right, -300, 300);
		//	 MOTOR_Control(  MOTOR_Duty_Left,  MOTOR_Duty_Right);	// 控制左右电机 g_fSpeedControlOut_L   
		//	 }
		break;
		
		case Meeting_Out: 	
		Speed_Measure();
		journey+=(Measure_Speed_L+Measure_Speed_R)/2*0.003*Ratio_Encoder;
		Dierction_Control();
		
		Speed[0]=0+g_fDirectionControlOut;   
		Speed[1]=0-g_fDirectionControlOut;
		if(ADC_End[1]>800&&ADC_End[4]>800)   //防止转完之后立马跑冲出赛道
		{
			if(B_Reach_Break)//B_Reach_Break
			{
				Disable=10;
				Speed_jiajia=2;
				Meeting_state=Real_Meeting_Out;
				
			}
		}
		Speed_Control();
		MOTOR_Duty_Left = g_fSpeedControlOut_L;
		MOTOR_Duty_Left = range_protect(MOTOR_Duty_Left, -500, 500);
		MOTOR_Duty_Right = g_fSpeedControlOut_R;
		MOTOR_Duty_Right = range_protect(MOTOR_Duty_Right, -500, 500);
		MOTOR_Control(  MOTOR_Duty_Left,  MOTOR_Duty_Right);	// 控制左右电机 g_fSpeedControlOut_L
		
		
		break;
		
		case Real_Meeting_Out:
		
		Control();
		journey+=(Measure_Speed_L+Measure_Speed_R)/2*0.003*Ratio_Encoder;
		if(journey>100)
		{
			journey=0;
			//起跑线防止误判5s
			Way_Flag=Ob_NUM;
			Meeting_state=No_Meeting;
			Broken_Flag=0;

			
		}
		break;
		
		
		default:break;	
		
	}
	
	
}


/*************转180度**************/
float Turn_X=0;
char Meeting_circle=0;
void Meeting(void)
{
	Circle_Flag=0;
	jiaodu();
	Filter();
	Speed_Measure();
	Turn_X+=Gyro_X*0.003;
	Speed[0]=-20;   
	Speed[1]=20;
	if(abs(Turn_X)>=120)
	{
		Turn_X=0;
		Speed[0]=0;   
		Speed[1]=0; 
		Meeting_circle=0;
		A_Reach_Break=1;
	    Meeting_state=Meeting_Out;
		
	}
	
	Speed_Control();
	MOTOR_Duty_Left=g_fSpeedControlOut_L;
	MOTOR_Duty_Left = range_protect(MOTOR_Duty_Left, -800, 800);
	MOTOR_Duty_Right=g_fSpeedControlOut_R;
	MOTOR_Duty_Right = range_protect(MOTOR_Duty_Right, -800, 800);
	MOTOR_Control(  MOTOR_Duty_Left,  MOTOR_Duty_Right);	// 控制左右电机 g_fSpeedControlOut_L
	
	
}
//
//

/***方案2****/

//void Control(void)
//  {
//      float max_diff;
//      if(Flag_3ms)
//	{
//		Flag_3ms=0;
//		
//		jiaodu();
//		Speed_Measure();
//		Dierction_Control();
//		Speed[0]=Speed_Set; 
//		Speed[1]=Speed_Set;
//		if(ADC_End[1]<200&&ADC_End[4]<200)
//		{
//			Speed[0]=0; 
//			Speed[1]=0;
//		}  
//		g_fDirectionControlOut = range_protect(g_fDirectionControlOut,-900,900); 
//		MOTOR_Duty_Left=g_fSpeedControlOut_L+g_fDirectionControlOut;
//		MOTOR_Duty_Left = range_protect(MOTOR_Duty_Left, -900, 900);	
//		MOTOR_Duty_Right=g_fSpeedControlOut_R-g_fDirectionControlOut;
//		MOTOR_Duty_Right = range_protect(MOTOR_Duty_Right, -900, 900);
//		MOTOR_Control(  MOTOR_Duty_Left,  MOTOR_Duty_Right);	// 控制左右电机 g_fSpeedControlOut_L
//		
//
//		
//	    
//	}
//      if(Flag_6ms)
//	{
//	    Flag_6ms=0;
//		Speed_Control();
////	    Filter();
////		if(Turn_Flag)
////		{
////			Radius=PID_Realize(&Obstacle_Direct_N_PID, Obstacle_Direct_N, atan(Turn_offset)*100, 0);//+Gyro_X*0.2;
////			// Radius=PlacePID_Control(&Direct_In_N_PID, Direct_In_N, atan(Turn_offset)*100, 0);//+Gyro_X*0.2;
////		}
////		else
////			Radius=PID_Realize(&Obstacle_Direct_N_PID, Obstacle_Direct_N, atan(Turn_offset)*100, 0);//+Gyro_X*0.2;
////		Radius=range_protect(Radius, -700, 700);
//	}
//      if(Flag_9ms)
//	{
//	    
//	    Flag_9ms=0;	
//	  //  Speed_Measure();
//	    
//	    
//	}
//      if(Flag_18ms)
//	{	
//	    Flag_18ms=0;
////	    Speed[0]=Speed_Set; 
////	    Speed[1]=Speed_Set;
//
//
//	    Speed_Min = Speed_Min * 0.1 +  (Measure_Speed_L+Measure_Speed_R)/2 * 0.9;
//	    if (Speed_Min < 40)                
//		Speed_Min = 40; 
//	}
//  }