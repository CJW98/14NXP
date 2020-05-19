#ifndef _FLASH_H
#define _FLASH_H

#include "headfile.h"
#include "common.h"
#define SectorNum 127
extern uint16  Int_Speed_set;
extern float PID_Motor_L[4],PID_Motor_R[4];
void Car_Debug();
void Mode_Set();


#endif