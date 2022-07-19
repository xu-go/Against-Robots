#ifndef GIMBALTASK_H
#define GIMBALTASK_H
#include "main.h"
#include "CAN_Receive.h"
#include "pid.h"
#include "remote_control.h"

#define YAW 0
#define mid_angle 3800
#define Cloud_MOVE_MAX 8191
#define Slope_Begin_Yaw 0.005
//云台模式选择
typedef enum
{
	CLOUD_MECH_MODE = 0,
	CLOUD_GYRO_MODE = 1,
} GimbalCtrlMode;


void GIMBAL_task(void *pvParameters);
void GIMBAL_InitCtrl(void);
void RC_Set_Mode(void);
void GIMBAL_Set_Control(void);
void GIMBAL_PositionLoop(void);
void GIMBAL_CanSend(void);

void GIMBAL_UpdateAngle(int16_t angle );
void GIMBAL_UpdateSpeed(int16_t speed );
void GIMBAL_UpdateCurrent(int16_t current );
void CloudAngleSum(void);

#endif
