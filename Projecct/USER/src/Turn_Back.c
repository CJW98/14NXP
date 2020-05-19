#include "Turn_Back.h"

float Turn_Angle,Target_Angle;



//************编码器积分acos（x/y）与陀螺仪角度融合
void Turn_Back()
{
    float   Gyro_X_Speed[2];
    Target_Angle=180;   
    
    
//    Gyro_X_Speed[1]=Gyro_X_Speed[0];    
//    Gyro_X_Speed[0]=Gyro_X;    
    //Angle_Speed_Z=Gyro_X_Speed[0]*0.7+Gyro_X_Speed[1]*0.3;//低筒滤波 新的值为本次的0.7个权重加上次0.3的权重 
    
   // Angle_Speed_Z+=Gyro_X*0.01;
    
 //   Gyro_X=(Gyro_X+230)*GYRO_SCALE;  //X轴角速度归一化
 //   Turn_Angle+=Gyro_X*0.01;
 
}