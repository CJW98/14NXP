#ifndef OTSUMETHOD_H_
#define OTSUMETHOD_H_

#include "headfile.h"
extern uint8 Pixle[60][188];
#define IMAGE_W 188
#define IMAGE_H 80
#define IMAGE_SIZE 15040
extern uint8_t img[IMAGE_H][IMAGE_W];//二值化后的图像
void Otsu_Method(void);
//void Otsu(void);

#endif