#ifndef CAR_H_
#define CAR_H_
#include "headfile.h"
extern char LCD_Flag;
extern int8 Target_Speed;
extern int  stop_Flag;
extern int Stop_Up_Flag, DelayStop, SlowStop;
extern void Car_Run();
extern void LCD_Show();
extern void Car_Up(int set, int del_t, int slow_t);//延时时间,减速时间

#endif