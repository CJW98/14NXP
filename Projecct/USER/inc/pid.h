#ifndef _PID_H_
#define _PID_H_

#include "headfile.h"

#define KP 0
#define KI 1
#define KD 2
#define KT 3
#define KB 4
#define KF 5
extern uint16 Speed_Set;
extern float g_fSpeedControlOut_R,g_fSpeedControlOut_L;
typedef struct PID
{
	float SumError;	//误差累计	
	int32 LastError;	//Error[-1]
	int32 PrevError;	//Error[-2]	
	int32 LastData;	//Speed[-1]
} PID;

typedef struct Motor
{
    float Kp;
    float Ki;
    float Kd;
    
    	float NowError;	//Error[-1]
	int32 LastError;	//Error[-2]	
	int32 PrevError;	//Speed[-1]
    

}Motor;


extern PID  Direct_In_PID,Direct_Out_PID,Direct_In_N_PID,OBSTACLE_DIRECT_PID,CONTROL_PID,CONTROL_N_PID,Obstacle_Direct_N_PID,Obstacle_Direct_PID,Angle_Speed_N_PID;	//定义舵机和电机的PID参数结构体
//
extern float  Direct_In[4],Direct_Out[4],Direct_In_N[4],Obstacle_Direct[4],CONTROL[4],CONTROL_N[4],Obstacle_Direct_N[4],Obstacle_Direct[4],Angle_Speed_N[4];
extern Motor Motor_L,Motor_R;
extern int32 PlacePID_Control(PID *sprt, float *PID, float NowPiont, float SetPoint);
void PID_Parameter_Init(PID *sptr);
void Motor_PID_Init(void);
int32 PID_Realize(PID *sptr, float *PID, int32 NowData, int32 Point);
int32 PID_Increase(PID *sptr, float *PID, int32 NowData, int32 Point);
extern void Speed_Control();
extern void Motor_PID(void);




#endif 