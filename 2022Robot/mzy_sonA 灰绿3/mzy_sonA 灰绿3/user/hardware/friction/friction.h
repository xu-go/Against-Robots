#ifndef FRICTION_H
#define FRICTION_H

#include "main.h"
#include "fric.h"


//ң��ģʽ�µ�һЩ��־λ
#define    FRIC_STEP0    0
#define    FRIC_STEP1    1
#define    FRIC_STEP2    2

//�ٶ�ѡ��
#define FRI_OFF  	0
#define FRI_LOW  	1	
#define FRI_SENTRY  2		//�ڱ�����


void friction_RC_Ctrl(void);
void friction_AUTO_Ctrl(void);
void friction_Init(void);
void FRICTION_StopMotor(void);
bool FRIC_RcSwitch(void);
void Fric_mode(uint16_t Fric_Speed_Level);

/****Ħ���ָ�������*****/
void Friction_Ramp(void);
uint16_t Fric_GetHeatInc(void);
float Fric_GetSpeedReal(void);

#endif
