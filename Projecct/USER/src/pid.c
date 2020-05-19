#include "pid.h"


float CONTROL[4]={1, 0, 30, 1000};
float CONTROL_N[4]={1, 0, 6, 1000};

float Direct_In[4] = {0.1, 0, 1, 1000};
float Direct_Out[4]={0, 0, 0, 1000};
float Direct_In_N[4] = {1, 0, 1, 1000};
float Obstacle_Direct[4] = {0.02,0,0.03,1000};
float Obstacle_Direct_N[4] = {20,0,1,1000};
float Angle_Speed_N[4] ={0.01, 0, 0.01, 1000};
PID Direct_In_PID,Direct_Out_PID,Direct_In_N_PID,OBSTACLE_DIRECT_PID,CONTROL_PID,CONTROL_N_PID,Obstacle_Direct_N_PID,Obstacle_Direct_PID,Angle_Speed_N_PID;	//定义舵机和电机的PID参数结构体


// PID参数初始化
void PID_Parameter_Init(PID *sptr)
{
	sptr->SumError  = 0;
	sptr->LastError = 0;	//Error[-1]
	sptr->PrevError = 0;	//Error[-2]	
	sptr->LastData  = 0;
}


void Motor_PID_Init(void)
{
    Motor_L.NowError  = 0;
    Motor_L.LastError = 0;	//Error[-1]
    Motor_L.PrevError = 0;	//Error[-2]
    
    
    Motor_R.NowError  = 0;
    Motor_R.LastError = 0;	//Error[-1]
    Motor_R.PrevError = 0;	//Error[-2]
    

    
}

/***位置式****/
int32 PID_Realize(PID *sptr, float *PID, int32 NowData, int32 Point)
{
	//当前误差，定义为寄存器变量，只能用于整型和字符型变量，提高运算速度
	int32 iError,	// 当前误差
		 Realize;	// 最后得出的实际增量

	iError = Point - NowData;	// 计算当前误差
	sptr->SumError += PID[KI] * iError;	// 误差积分
	if (sptr->SumError >= PID[KT])
	{
		sptr->SumError = PID[KT];
	}
	else if (sptr->SumError <= -PID[KT])
	{
		sptr->SumError = -PID[KT];
	}

	Realize = PID[KP] * iError
			+ sptr->SumError
			+ PID[KD] * (iError - sptr->LastError);
	sptr->PrevError = sptr->LastError;	// 更新前次误差
	sptr->LastError = iError;		  	// 更新上次误差
	sptr->LastData  = NowData;			// 更新上次数据

	return Realize;	// 返回实际值
}

/***增量式PID****/
int32 PID_Increase(PID *sptr, float *PID, int32 NowData, int32 Point)
{
	//当前误差，定义为寄存器变量，只能用于整型和字符型变量，提高运算速度
	int32 iError,	//当前误差
		Increase;	//最后得出的实际增量

	iError = Point - NowData;	// 计算当前误差

	Increase =  PID[KP] * (iError - sptr->LastError)
			  + PID[KI] * iError
			  + PID[KD] * (iError - 2 * sptr->LastError + sptr->PrevError);
	
	sptr->PrevError = sptr->LastError;	// 更新前次误差
	sptr->LastError = iError;		  	// 更新上次误差
	sptr->LastData  = NowData;			// 更新上次数据
	
	return Increase;	// 返回增量
}


int32 PlacePID_Control(PID *sprt, float *PID, float NowPiont, float SetPoint)
{
	//定义为寄存器变量，只能用于整型和字符型变量，提高运算速度
	float iError;	//当前误差
	int  Actual;	//最后得出的实际输出值
	float Kp;		//动态P
	
	iError = SetPoint - NowPiont;	//计算当前误差
	sprt->SumError += iError*0.01;
	if (sprt->SumError >= PID[KT])
	{
		sprt->SumError = PID[KT];
	}
	else if (sprt->SumError <= PID[KT])
	{
		sprt->SumError = -PID[KT];
	}
	//二次函数是为了达到 误差越大  反应越快 回复力越大 其中 KI值是误差为0时的P 也就是直道上的P值
	Kp = 1.0 * (iError*iError) / PID[KP] + PID[KI];	//P值与差值成二次函数关系，此处P和I不是PID参数，而是动态PID参数，要注意！！！
	
	Actual = Kp * iError
		   + PID[KD] * ((0.8*iError + 0.2*sprt->LastError) - sprt->LastError);//不完全微分  
	sprt->LastError = iError;		//更新上次误差

	Actual = range_protect(Actual, -260, 260);

	return Actual;
}


Motor Motor_L,Motor_R;
float fSpeedErrorInteg[2] = 0;
float g_fSpeedControlOut_R=0,g_fSpeedControlOut_L=0;
float g_fSpeedErrorTemp[2][5] = {0};
uint16 Speed_Set=0;
void Speed_Control()
{
	int8 index[2]={1,1};
	
	Motor_R.Kp=PID_Motor_R[0];
	Motor_L.Kp=PID_Motor_L[0];
	Motor_R.Ki=PID_Motor_R[1];
	Motor_L.Ki=PID_Motor_L[1];
	
	
	
	
	Motor_L.NowError = Speed[0] - Measure_Speed_L;
	Motor_R.NowError = Speed[1] - Measure_Speed_R;
	

    
    
//    if(Motor_R.NowError>=Motor_R.LastError)   
//	Motor_R.NowError = ((Motor_R.NowError-Motor_R.LastError)>2?(Motor_R.LastError+2):Motor_R.LastError); // 是速度偏差限幅
//    else  
//	Motor_R.NowError = ((Motor_R.NowError-Motor_R.LastError)<-2?(Motor_R.LastError-2):Motor_R.LastError); 
//        
//    if(Motor_L.NowError>=Motor_L.LastError)   
//	Motor_L.NowError = ((Motor_L.NowError-Motor_L.LastError)>2?(Motor_L.LastError+2):Motor_L.LastError); // 是速度偏差限幅
//    else  
//	Motor_L.NowError = ((Motor_L.NowError-Motor_L.LastError)<-2?(Motor_L.LastError-2):Motor_L.LastError); 
//    
//    
//  /**************右轮**********/
//     
// 
//    if((Motor_R.NowError<=15)&&(Motor_R.NowError>=-15))
//	index[0]=1;
//    else
//	index[0]=0;
//    fSpeedErrorInteg[0] = index[0]  * Motor_R.NowError * 1;
//    g_fSpeedControlOut_R +=10*(Motor_R.NowError-Motor_R.LastError) + fSpeedErrorInteg[0];
//    g_fSpeedControlOut_R = range_protect(g_fSpeedControlOut_R,-800,800); 
//    Motor_R.LastError = Motor_R.NowError;//记录上一次的偏差e[n-1]
//// 
////	
//	if((Motor_L.NowError<=15)&&(Motor_L.NowError>=-15))
//		index[1]=1;
//	else
//		index[1]=0;
//	fSpeedErrorInteg[1] = index[0]  * Motor_L.NowError * 1;
//	g_fSpeedControlOut_L +=10*(Motor_L.NowError-Motor_L.LastError) + fSpeedErrorInteg[1];
//	g_fSpeedControlOut_L = range_protect(g_fSpeedControlOut_L,-800,800); 
//	Motor_L.LastError = Motor_L.NowError;//记录上一次的偏差e[n-1]
	
    
//    if(A_Reach_End==1)
//	{
//
//    g_fSpeedControlOut_L += (5 + 5 ) * Motor_L.NowError	- (5  ) * Motor_L.LastError;//增量计算	
//    Motor_L.PrevError = Motor_L.LastError;//记录上上一次的偏差e[n-2]
//    Motor_L.LastError = Motor_L.NowError;//记录上一次的偏差e[n-1]
//    g_fSpeedControlOut_L = range_protect(g_fSpeedControlOut_L,-950,950); 
//    
//
// /**********左轮*******/   
//    
//
//    
//		
//	g_fSpeedControlOut_R += (5+ 5  ) * Motor_R.NowError	- (5 ) * Motor_R.LastError;//增量计算	
//	Motor_R.PrevError = Motor_R.LastError;//记录上上一次的偏差e[n-2]
//	Motor_R.LastError = Motor_R.NowError;//记录上一次的偏差e[n-1]
//	g_fSpeedControlOut_R = range_protect(g_fSpeedControlOut_R,-950,950);
//	}
//else
//{
    g_fSpeedControlOut_L += (Motor_L.Kp + Motor_L.Ki  + 0.1) * Motor_L.NowError	- (Motor_L.Kp  + 2 * 0.1) * Motor_L.LastError+ 0.1 * Motor_L.PrevError;//增量计算	
    Motor_L.PrevError = Motor_L.LastError;//记录上上一次的偏差e[n-2]
    Motor_L.LastError = Motor_L.NowError;//记录上一次的偏差e[n-1]
    g_fSpeedControlOut_L = range_protect(g_fSpeedControlOut_L,-980,980); 
    

 /**********左轮*******/   
    

    
		
	g_fSpeedControlOut_R += (Motor_R.Kp + Motor_R.Ki  + 0.1) * Motor_R.NowError	- (Motor_R.Kp  + 2 * 0.1) * Motor_R.LastError+ 0.1* Motor_R.PrevError;//增量计算	
	Motor_R.PrevError = Motor_R.LastError;//记录上上一次的偏差e[n-2]
	Motor_R.LastError = Motor_R.NowError;//记录上一次的偏差e[n-1]
	g_fSpeedControlOut_R = range_protect(g_fSpeedControlOut_R,-980,980);
    
//}   
//        

	

    
}





