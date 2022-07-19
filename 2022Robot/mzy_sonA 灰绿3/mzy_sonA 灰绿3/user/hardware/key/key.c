#include "key.h"
#include "stm32f4xx.h"


void key_Init(void)
{
	  GPIO_InitTypeDef GPIO_InitStructure;
   
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOI, &GPIO_InitStructure);
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1| GPIO_Pin_2| GPIO_Pin_3| GPIO_Pin_4| GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
		
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5| GPIO_Pin_6| GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
		
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOF, &GPIO_InitStructure);
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11| GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOH, &GPIO_InitStructure);
		
		
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOI, &GPIO_InitStructure);	


	  GPIO_WriteBit(GPIOA,GPIO_Pin_4,Bit_SET);
	  GPIO_WriteBit(GPIOA,GPIO_Pin_5,Bit_SET);

	  GPIO_ResetBits(GPIOB,GPIO_Pin_0);
	  GPIO_ResetBits(GPIOB,GPIO_Pin_1);
		
	  GPIO_ResetBits(GPIOC,GPIO_Pin_0);
	  GPIO_ResetBits(GPIOC,GPIO_Pin_1);
	  GPIO_ResetBits(GPIOC,GPIO_Pin_2);
	  GPIO_ResetBits(GPIOC,GPIO_Pin_3);

	  GPIO_ResetBits(GPIOC,GPIO_Pin_4);
	  GPIO_ResetBits(GPIOC,GPIO_Pin_5);

		GPIO_ResetBits(GPIOD,GPIO_Pin_12);
	  GPIO_ResetBits(GPIOD,GPIO_Pin_13);
		GPIO_ResetBits(GPIOD,GPIO_Pin_14);
	  //GPIO_ResetBits(GPIOD,GPIO_Pin_15);
		
	  GPIO_ResetBits(GPIOE,GPIO_Pin_4);
	  GPIO_ResetBits(GPIOE,GPIO_Pin_5);

	  GPIO_ResetBits(GPIOE,GPIO_Pin_6);
	  GPIO_ResetBits(GPIOE,GPIO_Pin_12);
		
    GPIO_ResetBits(GPIOF,GPIO_Pin_10);
		
	  GPIO_ResetBits(GPIOH,GPIO_Pin_10);
	  GPIO_ResetBits(GPIOH,GPIO_Pin_11);
		GPIO_ResetBits(GPIOH,GPIO_Pin_12);
	  GPIO_ResetBits(GPIOI,GPIO_Pin_2);

		GPIO_ResetBits(GPIOA,GPIO_Pin_0);
	  GPIO_ResetBits(GPIOA,GPIO_Pin_1);
		GPIO_ResetBits(GPIOA,GPIO_Pin_2);
	  GPIO_ResetBits(GPIOA,GPIO_Pin_3);
	  GPIO_ResetBits(GPIOA,GPIO_Pin_8);
	  GPIO_ResetBits(GPIOA,GPIO_Pin_14);
		
}
int KEY_Scan(u8 mode)
{
	static u8 key_up = 1;		//按键弹起标志
	
	if(mode)
		key_up = 1;		//支持按键长按核心代码
	
	if(key_up && (!KEY0 || !KEY1 || !KEY2 || !KEY3))		//检测到按键按下
	{
		delay_ms(10);			//延时10ms消抖
		key_up = 0;
		if(!KEY0)
			return KEY0_PRESS;
		else if(!KEY1)
			return KEY1_PRESS;
		else if(!KEY2)
			return KEY2_PRESS;
		else if(!KEY3)
			return KEY3_PRESS;
	}
	else if(KEY0 && KEY1 && KEY2 &&KEY3)
		key_up = 1;
	
	return 0;
}

