/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       chassis.c/h
  * @brief      完成底盘控制任务。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef CHASSISTASK_H
#define CHASSISTASK_H
#include "main.h"
#include "CAN_Receive.h"
#include "pid.h"
#include "stdbool.h"



#define TEST 0
#define GAME 1
#define AVOI 2
#define MODE GAME  //(选择TEST是测试状态，选择GAME是比赛状态)



#define PALST_COMPS_YAW        (45)     //陀螺仪YAW角速度补偿


//底盘旋转跟随PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 140.0f //500
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.08f//0.001
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 8.5f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 7000.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 20.0f

//底盘运动过程最大旋转速度
#define NORMAL_MAX_CHASSIS_SPEED_Z 4200

#define Omni_Speed_Max 13000  //速度限幅

#define CHASSIS_DECELE_RATIO (1.0f/36.0f)  //电机减数比

#define LIMIT_CHASSIS_MAX         13000     //功率限制情况下底盘单个电机最大输出

//底盘电流限幅
#define iTermChassis_Max          3000     //积分限幅



//底盘电机ID
typedef enum
{	
	LEFT_FRON_201 = 0,  // 左前
	RIGH_FRON_202 = 1,  // 右前
	LEFT_BACK_203 = 2,  // 左后
	RIGH_BACK_204 = 3,  // 右后	
} ChassisWheel;





typedef enum
{	
	REST = 0,
	Stop,
	out,
	high_1,
	high_2,
	high_3,
	high_4,
	high_5,
	high_6,
	high_7,
	high_8,
	low3_1,
	half_1,
	low2_1,
	low2_0,
	low2_11,
	low_1,
	low_2,
	low_3,
	low_4,
	low_5,
	low_6,
	low_7,
	low_8,	
	AUTO,
	GYRO_east,
	GYRO_east88,
	BGYRO_east,
	BGYRO_east2,
	east_89,
	east_91,
	GYRO_west,
	BGYRO_west,
	GYRO_north,
	BGYRO_north,
	BGYRO_north1,
	GYRO_south,
	BGYRO_south,
	paw_init,
	paw_normal,
	paw_raise,
	booM,
	booM2,
	shou,
	zhua,
	tou,
	paw_get,
	ChassisPosiMode,
	STOP1000,
	STOP2000,
	STOP3000,
	Arm_flag,
	Forward_flag,
	Turn_flag,
	back_1,
	back_11,
	back_2,
	back_3,
	back_4,
	back_5,
} ChassisSetMode;


void Gyro_turn1(fp32 degree);
void chassis_task(void *pvParameters);
void Chassis_Set_Mode(void);
void Chassis_Set_Contorl(void);
void JUDGE(void);
void judge(void);
void MPU_Update_Starting(void);
void MPU_Update(void);
void Gyro_east(fp32 degree);
void chassis_feedback_update(fp32 angle);
float Chassis_GetOffsetAngle(fp32 angle);
void POSI_REST(void);
void STOPDELAY(fp32 time);
void arm_flag(void);
void turn_flag(void);
void forward_flag(void);


void avoi(void);

void Chassis_Motor_Speed_PID( ChassisWheel Wheel );
void Chassis_Motor_Angle_PID( ChassisWheel Wheel );
void MotorAngleSum( ChassisWheel Wheel );
void Chassis_MotorOutput(void);
void CHASSIS_CANSend(void);
void Chassis_Omni_Move_Calculate(void);
void Chassis_Angle_Move_Calculate(void);
void CHASSIS_UpdateMotorAngle( ChassisWheel Wheel, int16_t angle);
void CHASSIS_UpdateMotorSpeed( ChassisWheel Wheel, int16_t speed);
void CHASSIS_UpdateMotorCur( ChassisWheel Wheel, int16_t current);
void Chassis_Init(void);
void CHASSIS_REST(void);
void Chassis_RC_Ctrl(void);


void AUTO_mode(void);
#endif
