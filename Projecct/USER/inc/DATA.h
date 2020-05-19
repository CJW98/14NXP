#ifndef _DATA_H_
#define _DATA_H_

#include "headfile.h"
extern uint8 Meeting_Flag;
extern void DATA_init();
extern void vcan_sendware(void *wareaddr, uint32_t waresize);
extern void Twocar_Train_RX();
extern void Twocar_Train_TX();
extern char A_Start_Ok,A_Reach_Break,A_Reach_End,A_Finish_End,B_Start_Ok,B_Reach_Break,B_Reach_End,B_Finish_End;
#endif 