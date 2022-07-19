#include "fric.h"

#include "stm32f4xx.h"


/**
  * @brief  摩擦轮电机配置(初始化+解锁)
  * @param  void
  * @retval void
  * @attention PWM1\2直接定义为CCR寄存器,PA6,PA7
  */
void TIM4_Init(void)
{
	GPIO_InitTypeDef          gpio;
	TIM_TimeBaseInitTypeDef   tim;
	TIM_OCInitTypeDef         oc;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	

	gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD,&gpio);
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource12, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource13, GPIO_AF_TIM4);    
	
	tim.TIM_Prescaler = 84-1;//1MHz
	tim.TIM_CounterMode = TIM_CounterMode_Up;		
	tim.TIM_Period = 2499;   //25ms一个周期	,每+1代表时间+1微秒
	tim.TIM_ClockDivision = TIM_CKD_DIV1;		
	TIM_TimeBaseInit(TIM4,&tim);
	
	oc.TIM_OCMode = TIM_OCMode_PWM2;		
	oc.TIM_OutputState = TIM_OutputState_Enable;		
	oc.TIM_OutputNState = TIM_OutputState_Disable;	
	oc.TIM_Pulse = 0;		
	oc.TIM_OCPolarity = TIM_OCPolarity_Low;		
	oc.TIM_OCNPolarity = TIM_OCPolarity_High;		//输出极性高
	oc.TIM_OCIdleState = TIM_OCIdleState_Reset;		
	oc.TIM_OCNIdleState = TIM_OCIdleState_Set;		
	TIM_OC1Init(TIM4,&oc);		
	TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Enable);
	
	TIM_OC2Init(TIM4,&oc);		
	TIM_OC2PreloadConfig(TIM4,TIM_OCPreload_Enable);
				 
	TIM_ARRPreloadConfig(TIM4,ENABLE);
	
	TIM_CtrlPWMOutputs(TIM4,ENABLE);
	
	TIM_Cmd(TIM4,ENABLE);
	PWM1 = 1000;		//解锁摩擦轮,大于640
	PWM2 = 1000;
}

/**
  * @brief  摩擦轮输出函数
  * @param  void
  * @retval pwm1  pwm2
  * @attention 左右都是正,以后接线的时候注意三条线的接法
  */
void TIM4_FrictionPwmOutp(int16_t pwm1,int16_t pwm2)//PA8,PE14
{
	PWM1 = pwm1+1000;	//持续输出时要大于1ms
	PWM2 = pwm2+1000;
}
