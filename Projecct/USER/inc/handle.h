#ifndef HANDLE_H_
#define HANDLE_H_

#include "headfile.h"

#define Near_Lines  10
#define IMAGE_W 188
#define IMAGE_H	80
#define Scan_Width 6        //追踪扫描行数


#define Lsm_Lines 10     //二乘法计算行数,应不大于Near_Line

#define L_Boundary 0
#define R_Boundary 1
extern uint8 jump_point;
extern uint8_t  SAIDAO_DATA[4];
extern uint8 Side_Thres;      //边界阈值
extern uint8 ob_Side_Thres;   //壁障边界阈值
extern int Gearshift;
extern char dis_circle;
extern char Broken_Flag;
extern uint8 L_Miss,R_Miss;//丢线起始行
extern float Obstacle_Miss_Average;
extern uint8 L_Find ,R_Find ;//找线标志
extern char Starting_Flag;
extern int Obstacle_Flag;
extern int Center_Deviation;//中心偏差
extern uint8 long_way[2];
extern int L_Side[80],R_Side[80],Mid_Line[80];//左右边线,中线
extern uint8 Searching_Nearby(void);
extern void Image_Handle(void);
extern void Side_Line(void);
extern void Midline_Extraction(void);
extern void Boundary_Analysis(void);
extern float Side_Angle(uint8 Boundary,uint8 Point,uint8 Size);//求角度
extern void Connection(uint8 Boundary,int Point0[2],int Point1[2]);
extern void Least_Square_Method(uint8 Boundary,uint8 Line);//最小二乘法
extern void Line(void);
extern void StartingLine(void);
extern void Rampway(void);
int16 Side_Thres_Count(int16 Pixel_A, int16 Pixel_B);
void StraightRoad(void);
#endif