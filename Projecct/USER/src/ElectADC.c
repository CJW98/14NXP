#include "ElectADC.h"
#include "math.h"

uint16_t  ADC_End[6];
uint8 Ring_Speed[2];
float Turn_offset=0;
float Angle_Z;



uint16_t  ADC_End_N[3];
float g_fDirectionControlOut,g_fDirectionControlOutOld;	//方向控制输出
void ADC_Read(void)
{
	uint16  ad_valu[5];
	
	ADC_End_N[0]=adc_once(AD1, ADC_12bit);  			//水平左
	//  ADC_End_N[2]=adc_once(AD2, ADC_12bit);     		// 左2
	// ADC_End_N[3]=adc_once(AD3, ADC_12bit);  			// AD3右2
	ADC_End_N[1]=adc_once(AD4, ADC_12bit);     		// 右	
	ADC_End_N[2]=adc_once(AD5, ADC_12bit);     		// 
	
	ButterworthFilter(ADC_End_N);
	ADC_End[1]=ADC_End_N[0];
	//  ADC_End[2]=ADC_End_N[2];
	//  ADC_End[3]=ADC_End_N[3];
	ADC_End[4]=ADC_End_N[1];
	ADC_End[5]=ADC_End_N[2];
	
	
	/***環島調試****/
	Ring_Stable();
}

float g_fDirectionError_dot;//方向偏差微分（g_fDirectionError_dot[0]为一对水平电感的差比和偏差微分）

void Dierction_Control(void)
{
	static float g_fDirectionErrorTemp[2][5];
	
	Filter();		//获取电感值  
	//Turn_offset=(float)Center_Deviation/50;
	g_fDirectionErrorTemp[0][4] = g_fDirectionErrorTemp[0][3];
	g_fDirectionErrorTemp[0][3] = g_fDirectionErrorTemp[0][2];
	g_fDirectionErrorTemp[0][2] = g_fDirectionErrorTemp[0][1];
	g_fDirectionErrorTemp[0][1] = g_fDirectionErrorTemp[0][0];
	g_fDirectionErrorTemp[0][0] = atan(Turn_offset)*100;
	g_fDirectionError_dot = 5*(g_fDirectionErrorTemp[0][0]-g_fDirectionErrorTemp[0][3]);//水平电感的偏差微分
	g_fDirectionError_dot = (g_fDirectionError_dot> 70? 70:g_fDirectionError_dot);//偏差微分限幅
	g_fDirectionError_dot = (g_fDirectionError_dot<-70?-70:g_fDirectionError_dot);
	//g_fDirectionControlOut = atan(Turn_offset)*100*Direct_In[0] + g_fDirectionError_dot*Direct_In[2]-Gyro_X ;   //1  3
	g_fDirectionControlOut = atan(Turn_offset)*100*Direct_Out[0] + g_fDirectionError_dot*Direct_Out[2];//-Gyro_X*0.01;   //1  3
	if(abs(atan(Turn_offset)*100)>40)
		g_fDirectionControlOut = atan(Turn_offset)*100*Direct_Out[0] + g_fDirectionError_dot*2;   //1  3
	//g_fDirectionControlOut=-Gyro_X*0.5;
	if(Turn_Flag)
		//g_fDirectionControlOut = atan(Turn_offset)*100*Direct_In[0] + g_fDirectionError_dot*Direct_In[2]-Gyro_X*0.1;//0.25   
		g_fDirectionControlOut=-PID_Realize(&Direct_In_PID, Direct_In, Turn_offset, 0)-Gyro_X*0.1;
	
	
}


void DirectionControlOutput(void)
{
	
	static uint8 dk =0;
	dk++;
	g_fDirectionControlOut=(Radius-g_fDirectionControlOutOld)*dk/2.0+g_fDirectionControlOutOld;
	if(dk==2)
		dk=0;
	g_fDirectionControlOutOld = Radius;
	if(g_fDirectionControlOut > 1000)
		g_fDirectionControlOut = 1000;
	else if(g_fDirectionControlOut < -1000)
		g_fDirectionControlOut = -1000;
	
	
}

void Direction_PD()
{
	
	
}

/****环岛标志位****/
char Circle_Flag=0,Circle_In=0;

#define No_Ring              0
#define Ring_Flag            1
#define Ready_Ring           2
#define Star_Ring            3
#define In_Ring              4
#define Out_Ring             5
#define Real_Out_Ring        6

uint8 Ring[2];
uint8 Ring_Num=1;
uint8 Ring_state=0;
uint8 Ring_OutFlag=0;
float turn_buff[30]={0};
uint8 Camera_Flag=0;
char   Speed_Flag=0;
char   Size_Ring=0;   //大小环标志
char  ring_buff[2]=0;
float ring_angle=0;

/********flash参数*******/
uint8_t yuanhuan_line=110;
uint8_t ring_distance[4]={20,4,200,200}; 
void Ring_Stable(void)//只检测三个电感的阈值    任意赛道调一下就行
{
	uint8 i;
	uint8 way=0,way2=0;
	switch(Ring_state)
	{
		case No_Ring:
		{
			Turn_Flag=0;
			Turn_offset=(float)(ADC_End[4]-ADC_End[1])/(ADC_End[4]+ADC_End[1]+ADC_End[5]);	
			if(Sum_ADC[1]>9000&& Sum_ADC[0]>9000&&!Ramp_Flag&&ADC_End[5]>3500)
		//	if((ADC_End[1]>=3000 &&ADC_End[4]>=2500 )||(ADC_End[1]>=2500 &&ADC_End[4]>=3000)) 
			{
				Beep=2;
				Ring_state=Ring_Flag;
			}
			else
			{
				Ring_state=No_Ring;
			}
			break;
		}
		case Ring_Flag:
		{
			Turn_offset=(float)(ADC_End[4]-ADC_End[1])/(ADC_End[4]+ADC_End[1]+ADC_End[5]);	
			Distance+=(Measure_Speed_L+Measure_Speed_R)/2*0.003*Ratio_Encoder;
			if(Ring_Num==1||Ring_Num==4)
			{
				Buff_Flag=Ring[0];
				Size_Ring=Ring_Speed[0];
			}
			if(Ring_Num==2||Ring_Num==3)
			{
				Buff_Flag=Ring[1];
				Size_Ring=Ring_Speed[1];
			}
			if(Size_Ring==1)
			{
				Speed_Flag=1;
			}
			if(Size_Ring==2)
			{
				Speed_Flag=2;
			}
			if(Distance>ring_distance[1])
			{
				dis_circle=1;
				Distance=0;
				Ring_state=Ready_Ring;
			}
			break;
		}
		case Ready_Ring:		        /*******垂直电感入环********/
		{
			
			Camera_Flag=1;
			Turn_offset=(float)(ADC_End[4]-ADC_End[1])/(ADC_End[4]+ADC_End[1]+ADC_End[5]);
			Angle_Z+=Gyro_X*0.003;
			Distance+=(Measure_Speed_L+Measure_Speed_R)/2*0.003*Ratio_Encoder;
			
			
			//	    if(Buff_Flag==1)
			//		Turn_offset=0.1;
			//	    if(Buff_Flag==2)
			//		Turn_offset=-0.1;
			
			if(Distance>3)
			{
				
				Distance=0;
				Ring_state=Star_Ring;
				
			}
			break;
		}
		case  Star_Ring:
		{
			// Beep=2;
			Turn_Flag=1;			
//			if(Center_Deviation>80||Center_Deviation<-80)   //入环丢线
//			{
//				if(Buff_Flag==1)
//					Center_Deviation=60;
//				if(Buff_Flag==2)
//					Center_Deviation=-60;
//			}	
			Turn_offset=(float)Center_Deviation;
			//			if(abs(Center_Deviation)>35)
			//				Turn_offset=(float)Center_Deviation*1.5;
			Distance+=(Measure_Speed_L+Measure_Speed_R)/2*0.003*Ratio_Encoder;
			ring_angle+=Gyro_X*0.003;
			if(Distance>ring_distance[3])
				//if(abs(ring_angle)>=50) 
			{
				//Beep=2;
				// stop_Flag=1;
				ring_angle=0;
				Camera_Flag=0;
				Distance=0;
				Ring_state=In_Ring;
				
			}
			
			
			
			
			break;  
		}
		
		case In_Ring:
		{
			// Beep=2;
			//stop_Flag=1;
			if(Size_Ring==1)
			{
				Speed_Flag=1;
			}
			if(Size_Ring==2)
			{
				Speed_Flag=3;
			}
			Turn_Flag=0;
			Turn_offset=(float)(ADC_End[4]-ADC_End[1])/(ADC_End[4]+ADC_End[1]+ADC_End[5]);
			//if((ADC_End[1]>=2000 &&ADC_End[4]>=2300 )||(ADC_End[1]>=2300 &&ADC_End[4]>=2000 ))
		    if((Sum_ADC[1]>7000&& Sum_ADC[0]>7000)||(Sum_ADC[1]>7000&& Sum_ADC[0]>7000))
			{
				Beep=2;
				Ring_state=Out_Ring;
			}
			break;
		}
		
		case Out_Ring:
		{
//			
//	
			if(Buff_Flag==1)
				Turn_offset=(float)(ADC_End[4]-ADC_End[5 ])/(ADC_End[4]+ADC_End[1]);
			if(Buff_Flag==2)
				Turn_offset=(float)(ADC_End[5]-ADC_End[1])/(ADC_End[4]+ADC_End[1]);
		 //    Turn_offset=(float)(ADC_End[4]-ADC_End[1])/(ADC_End[4]+ADC_End[1]);
			Ring_OutFlag=1;
			//if(Distance>20) 
			if(Size_Ring==1)
			{
				Speed_Flag=1;
			}
			if(Size_Ring==2)
			{
				Speed_Flag=1;
			}
			if(L_Find||R_Find)
			{	
				Distance+=(Measure_Speed_L+Measure_Speed_R)/2*0.003*Ratio_Encoder;
				// if(Distance>10) 
				if(Distance>ring_distance[0]) 
				{
					//Beep=2;
					//Beep=2;
					dis_circle=0;
					Distance=0;	
					//A_Reach_End = 1;
					Ring_state=Real_Out_Ring;
				}
			}
			
			
			break;
		}
		
		case Real_Out_Ring:	
		{

			Turn_Flag=1;
			gpio_set (C5, 1);
			Distance+=(Measure_Speed_L+Measure_Speed_R)/2*0.003*Ratio_Encoder;
			Turn_offset=(float)Center_Deviation;
			//Turn_offset=(float)(ADC_End[4]-ADC_End[1])/(ADC_End[4]+ADC_End[1]+ADC_End[5])/2;
			//	    if(!L_Find&&!R_Find)
			//	    {
			//		if(Buff_Flag==1) Turn_offset=-0.1;
			//		if(Buff_Flag==2) Turn_offset=0.1;
			////		Turn_offset=-Turn_offset;
			//	    }	 
			if(Distance>yuanhuan_line)	    	    
			{
				//				if(L_Find==70&&R_Find==70)
				//				{
				   Beep=2;
				gpio_set (C5, 0);
				Distance=0;	
				Ring_Num++;
				Ring_OutFlag=0;
				Ring_state = No_Ring; 
				Speed_Flag=0;
				//			}
			}
			break;
		}
		default:break;	
	}
	
	
	
}



