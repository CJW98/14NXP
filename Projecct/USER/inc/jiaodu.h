#ifndef __JIAODU_H__
#define __JIAODU_H__


#include "headfile.h"

#define ACCEL_SCALE		(1.0f/8191.875f)			//量程±4g,65535/(4*2)=8191.875f
#define ACCEL_SCALE2		(1.0f/16383.875f)			//量程±2g,65535/(4*2)=8191.875f
#define GYRO_SCALE		(1.0f/32.7675f)				//量程±1000dps,65535/(2*1000)=32.7675f
extern int16  Acc_X,Acc_Y,Acc_Z,Gyro_X,Gyro_Y,Gyro_Z;
extern float angle_offset_vertical,Gyro_Now,g_fCarAngle,g_fGyroscopeAngleIntegral;
extern float Angle1_Y,Angle1_Z,Angle_Speed_Y,Angle_Speed_Z,Angle_Speed,Car_Angle;
extern float Angle[2],AngleSpeed[2];
extern float m;
void jiaodu(void);
void Kalman_Filter(float angle_m,float gyro_m);

void AttitudeCalculation(float *euler,float *gyro);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);

#endif