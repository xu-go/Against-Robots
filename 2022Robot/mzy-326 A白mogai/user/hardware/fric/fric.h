#ifndef FRIC_H
#define FRIC_H
#include "main.h"

#define PWM1  TIM4->CCR1     //Ħ����,PD12
#define PWM2  TIM4->CCR2     //Ħ����,PD13

void TIM4_Init(void);
void TIM4_FrictionPwmOutp(int16_t pwm1,int16_t pwm2);//PA8,PE14

#endif
