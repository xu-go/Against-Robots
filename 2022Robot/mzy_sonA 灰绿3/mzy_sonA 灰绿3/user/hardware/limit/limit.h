#ifndef LIMIT_H
#define LIMIT_H


#include <stdio.h>
#include "main.h"
#include "stdbool.h"
#include "string.h"
#include "stm32f4xx.h"

int constrain(int amt, int low, int high);
float constrain_float(float amt, float low, float high);
int16_t constrain_int16_t(int16_t amt, int16_t low, int16_t high);
int32_t constrain_int32_t(int32_t amt, int32_t low, int32_t high);


//Ð±ÆÂº¯Êý
float RAMP_float( float final, float now, float ramp );
float RampInc_float( float *buffer, float now, float ramp );

extern void abs_limit_num(fp32 num, fp32 Limit);

#endif
