#ifndef PIT_H_
#define PIT_H_
#include "headfile.h"
extern int Disable;//������ʧ��ʱ��
extern int Beep;
extern char Flag_3ms,Flag_6ms,Flag_9ms,Flag_18ms;
extern void PIT0_IRQHandler(void);
extern void PIT1_IRQHandler(void);
#endif