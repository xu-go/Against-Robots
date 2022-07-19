#include "fric.h"

#include "stm32f4xx.h"


/**
  * @brief  Ħ���ֵ������(��ʼ��+����)
  * @param  void
  * @retval void
  * @attention PWM1\2ֱ�Ӷ���ΪCCR�Ĵ���,PA6,PA7
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
	tim.TIM_Period = 2499;   //25msһ������	,ÿ+1����ʱ��+1΢��
	tim.TIM_ClockDivision = TIM_CKD_DIV1;		
	TIM_TimeBaseInit(TIM4,&tim);
	
	oc.TIM_OCMode = TIM_OCMode_PWM2;		
	oc.TIM_OutputState = TIM_OutputState_Enable;		
	oc.TIM_OutputNState = TIM_OutputState_Disable;	
	oc.TIM_Pulse = 0;		
	oc.TIM_OCPolarity = TIM_OCPolarity_Low;		
	oc.TIM_OCNPolarity = TIM_OCPolarity_High;		//������Ը�
	oc.TIM_OCIdleState = TIM_OCIdleState_Reset;		
	oc.TIM_OCNIdleState = TIM_OCIdleState_Set;		
	TIM_OC1Init(TIM4,&oc);		
	TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Enable);
	
	TIM_OC2Init(TIM4,&oc);		
	TIM_OC2PreloadConfig(TIM4,TIM_OCPreload_Enable);
				 
	TIM_ARRPreloadConfig(TIM4,ENABLE);
	
	TIM_CtrlPWMOutputs(TIM4,ENABLE);
	
	TIM_Cmd(TIM4,ENABLE);
	PWM1 = 1000;		//����Ħ����,����640
	PWM2 = 1000;
}

/**
  * @brief  Ħ�����������
  * @param  void
  * @retval pwm1  pwm2
  * @attention ���Ҷ�����,�Ժ���ߵ�ʱ��ע�������ߵĽӷ�
  */
void TIM4_FrictionPwmOutp(int16_t pwm1,int16_t pwm2)//PA8,PE14
{
	PWM1 = pwm1+1000;	//�������ʱҪ����1ms
	PWM2 = pwm2+1000;
}
