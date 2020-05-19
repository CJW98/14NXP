#ifndef CONTROL_H_
#define CONTROL_H_

#include "headfile.h"
extern uint16_t DISS; 
extern uint8_t ceshi_data[4];  //�����   �����߼��
extern uint8 Way_Flag ;
extern uint8 Ob_NUM;
extern float Attitude_Angle_Z;
extern char Speed_jiajia;
extern float Obstacle_Angle[2],Obstacle_Line;                   //��һ�νǶȵ�·��
extern float Ramp_Distance;
extern char  Ramp_Flag;   //�µ���־λ
extern int32 Radius,Speed_Min;
extern int32 Direction_PWM ;
extern int32 Direction_Speed ;
extern int32 Radius,Speed_Min;
extern int Speed[2]; 
extern char Turn_Flag;
 extern float Turn_X;
extern char Target_Flag;              //��־λ
extern float journey;
#define Camear_Flag    0        //0Ϊ��ſ���  1Ϊ����ͷ����
#define Car_Wheelbase	153/2//̥���ľ���һ��(mm)
#define Ratio_Encoder  20.7/(1175*0.003);// �����ٶ�=counter*�����ܳ�(cm)/(����תһȦ��Ӧ��������*��������3ms)
extern void Control(void);
extern void Meeting(void);
void Master_Control(void);
extern void Obstacle_Control();
extern void  Run_Control();
#endif