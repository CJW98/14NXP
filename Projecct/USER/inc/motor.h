#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "headfile.h"


extern float Distance;
extern int Measure_Speed_L,Measure_Speed_R;
extern int Speed_Nor_Low,Speed_Nor_High;
extern int Speed_Up,Speed_Down;
extern void Motor_init();
extern void MOTOR_Control(int32 L_Duty,int32 R_Duty);
extern int32 range_protect(int32 duty, int32 min, int32 max);//限幅保护
extern void Speed_Measure();
extern void Speed_Change(int sta);//0减速,1常速,2加速,3全速
#endif 