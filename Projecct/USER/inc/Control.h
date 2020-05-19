#ifndef CONTROL_H_
#define CONTROL_H_

#include "headfile.h"
extern uint16_t DISS; 
extern uint8_t ceshi_data[4];  //测距监测   起跑线监测
extern uint8 Way_Flag ;
extern uint8 Ob_NUM;
extern float Attitude_Angle_Z;
extern char Speed_jiajia;
extern float Obstacle_Angle[2],Obstacle_Line;                   //第一次角度的路径
extern float Ramp_Distance;
extern char  Ramp_Flag;   //坡道标志位
extern int32 Radius,Speed_Min;
extern int32 Direction_PWM ;
extern int32 Direction_Speed ;
extern int32 Radius,Speed_Min;
extern int Speed[2]; 
extern char Turn_Flag;
 extern float Turn_X;
extern char Target_Flag;              //标志位
extern float journey;
#define Camear_Flag    0        //0为电磁控制  1为摄像头控制
#define Car_Wheelbase	153/2//胎中心距离一半(mm)
#define Ratio_Encoder  20.7/(1175*0.003);// 左轮速度=counter*左轮周长(cm)/(左轮转一圈对应的脉冲数*程序周期3ms)
extern void Control(void);
extern void Meeting(void);
void Master_Control(void);
extern void Obstacle_Control();
extern void  Run_Control();
#endif