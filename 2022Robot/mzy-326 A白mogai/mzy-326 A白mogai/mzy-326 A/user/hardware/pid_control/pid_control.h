#ifndef PIDCONTRLO_H
#define PIDCONTRLO_H
#include "main.h"
#include "stdbool.h"


bool PID_KP_Switch0(void);
bool PID_KP_Switch1(void);
bool PID_KI_Switch0(void);
bool PID_KI_Switch1(void);
void PID_KP_Ctrl(void);


#endif
