#ifndef SERVO_H
#define SERVO_H
#include "main.h"
#include "timer.h"


#define S1  TIM5->CCR4     //A×óI0
#define S2  TIM5->CCR3     //BÓÒH12
#define S3  TIM5->CCR2     //CÉÏH11

typedef enum
{
	
	servo1 = 0,  
	servo2 = 1,  
	servo3 = 2,  
	
} Servo;


void TIM5_Servo(int16_t s1,int16_t s2,int16_t s3);
void servo(int16_t s1,int16_t s2,int16_t s3);
void Servo_Init(void);
void Normal(void);
void Normal2(void);
void Tou1(void);
void Tou2(void);
void Tou3(void);
void Tou4(void);
void Tou5(void);
void Tou6(void);
void Nor(void);
void Raise(void);
void Raise2(void);

void Get(void);

void Place1(void);
void Place2(void);
void Place3(void);
void Place222(void);
void Place333(void);

void Place11(void);
void Place21(void);
void Place22(void);
void Place31(void);
void Place32(void);

void Placec1(void);
void Placec2(void);
void Placec3(void);
void Placec4(void);



void Servo_Ramp(Servo servo ,float x);

#endif
