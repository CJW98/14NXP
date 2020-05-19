#ifndef _ELECTADC_H_
#define _ELECTADC_H_

#include "headfile.h"


#define AD1    ADC0_SE11//A7   ADC0_SE10 
#define AD2    ADC1_SE7a//A8  ADC0_SE11   ADC1_SE7a
//#define AD3    ADC1_SE6a//E2   ADC0_SE11    ADC1_SE6a
#define AD3    ADC0_SE10//E3    ADC1_SE7a
#define AD4    ADC0_SE13// B2   ADC0_SE10
#define AD5    ADC0_SE12// B3   ADC0_SE13


extern uint8_t yuanhuan_line;
extern uint8_t ring_distance[4];
extern uint16_t  ADC_End[6];
extern float Turn_offset;
extern uint8 Ring_Speed[2];
extern uint8 Ring_OutFlag;
extern float g_fDirectionControlOut,g_fDirectionControlOutOld;	//方向控制输出
extern uint8 Camera_Flag;
extern uint8 Ring_state;
extern float Angle_Z;
extern void DirectionControlOutput(void);
extern uint8 Ring[2];
extern char Speed_Flag;
extern void ADC_Read(void);
extern char Circle_Flag;
extern void Ring_Stable(void);//只检测三个电感的阈值    任意赛道调一下就行
extern void Circle_Judge();
extern void Direction_PD();
extern void Dierction_Control(void);

#endif 