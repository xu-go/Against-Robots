#ifndef SERVO_H
#define SERVO_H
#include "main.h"
#include "timer.h"

#define S2  TIM5->CCR3     //left--PH12--B
#define S3  TIM5->CCR2     //right--PH11--C
#define S4  TIM5->CCR1    //ph10-A
#define S5  TIM5->CCR4   //pi0-D
typedef enum
{
	  
	servo2 = 1,  
	servo3 = 2,  
	servo4 = 3,  
	servo5 = 4,  
} Servo;


void TIM5_Servo(int16_t s2,int16_t s3,int16_t s4,int16_t s5);
void servo(int16_t s2,int16_t s3,int16_t s4,int16_t s5);
void Servo_Init(void);
void Normal(void);
void Raise(void);
void boom(void);
void boom2(void);
void Raise11(void);
void Raise22(void);
void Raise33(void);
void Servo_Ramp(Servo servo ,float x);

#endif
