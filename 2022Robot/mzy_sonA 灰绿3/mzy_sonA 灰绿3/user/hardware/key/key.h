#ifndef KEY_H
#define KEY_H

#include "main.h"
#include "delay.h"
#include "sys.h"

#define KEY0 GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4)
#define KEY1 GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_5)
#define KEY2 GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_6)
#define KEY3 GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_12)
#define WK_UP GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)
 
#define KEY0_PRESS 1		//KEY0按下
#define KEY1_PRESS 2		//KEY1按下
#define KEY2_PRESS 3		//KEY0按下
#define KEY3_PRESS 4		//KEY1按下
#define WK_UP_PRESS 5		//WK_UP按下

void key_Init(void);

int  KEY_Scan(u8 mode);
#endif
