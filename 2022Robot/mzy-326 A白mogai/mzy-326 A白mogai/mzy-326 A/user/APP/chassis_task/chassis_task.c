/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       chassis.c/h
  * @brief      接地气写法，适合初学者使用
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
#include "chassis_task.h"
#include "start_task.h"

#include "main.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "led.h"
#include "pid.h"
#include "servo.h"
#include "laser.h"
#include "limit.h"
#include "arm_math.h"
#include "remote_control.h"
#include "motor_ctrl.h"

#include "CAN_Receive.h"
#include "pid.h"
#include "stdio.h"
#include <stdlib.h>
#include "mpu6050.h"
#include "inv_mpu.h"
#include "usart6.h"

uint8_t uncorrect_flag=0;
float current_angle=0;


#define abs(x) ((x)>0? (x):(-(x)))

/*--------------------高速1+格数------------------------*/ 
/*--------------------低速2+格数（221:超慢速一格）------*/	
/*--------------------后退6+格数------------------------*/
/*--------------------标志位手动开启函数（188直行，189转弯，190机械臂）------------------------*/
//***切记：连续执行同一路径的时候中间加一个标志位打开函数打开标志位，例如：机械臂（43，190，46）直行（11，188，221）
/* 具体度数根据实际修改


                          8180：北（180）
					  	8182:北(182)              8178北（178）
																|
8268:西（268）									|
8270：西（270）						                                  892:东(92)
<------------------------------   ------------------------->890：东（90）
8272:西（272）                                              888:东(88)
															  |
															  |
							 8358：南(358)		|        8362：南（362）
													 	8360：南（360）
*/


/**********************************************************/

//转向红色前加7，蓝色前加8，红蓝转向分两套写的
//	
//int lujin[]={12,43,190,44,221,45,190,422,11,831,12,833,11,83111,5,46,190,47,190,48,190,422,61,834,11,43,190,44,2211,45,190,422,83111,5,49,190,501,190,511,190,422,61,
//833,13,43,190,44,2211,45,190,422,8311,555,49,190,50,190,51,190,422,61,832,12,834,11,43,190,44,2211,45,190,422,11,831,12,83332,11,831,5,46,190,47,190,48,190,422,
//61,83332,13,8321,43,190,44,2211,45,190,422,8341,12,83122,11,8341,11,83122,5,46,190,47,190,48,190,422,
//61,8332,13,8322,12,43,190,44,221,45,190,422,8341,11,831221,12,8341,11,831221,11,83322,2211,831222,5,49,190,50,190,51,190,422,0};//完整路径planA蓝色
	
//int lujin[]={12,831,43,190,44,221,45,190,422,12,833,5,49,190,50,190,51,190,422,61,832,13,43,190,44,221,45,190,422,831,14,833,5,49,190,50,190,51,190,422,
//61,832,13,8331,43,190,44,221,45,190,422,13,831,13,834,5,46,190,47,190,48,190,422,611,83211,13,834,11,83211,190,52,190,53,5,54,190,55,190,422,61,8331,
//11,831111,13,83411,5,46,190,47,190,48,190,422,611,83211,13,834,12,83211,12,0};//蓝色区域对抗planA

//int lujin[]={12,831,43,190,44,221,45,190,422,12,8331,5,49,190,50,190,51,190,422,61,
//832,12,833,12,832,12,8331,5,52,190,53,190,54,190,55,190,422,61,831,12,8342,12,831,11,188,2211,8331,555,49,190,501,190,511,190,422,
//61,832,15,0};//暂时不用
	
//int lujin[]={12,43,190,44,221,45,190,422,11,831,13,8331,555,46,190,47,190,48,190,422,
//61,832,14,8331,190,52,190,53,5,54,190,55,190,422,61,8311,14,8331,555,46,190,47,190,48,190,422,
//61,832,12,8331,190,52,190,53,5,190,54,190,55,190,422,190,42,
//61,832,13,0};//蓝色区域对抗planC,三倍区满分后堵对面门(1)

//int lujin[]={12,43,190,44,221,45,190,422,13,831,13,834,5,46,190,47,190,48,190,422,
//611,83211,14,834,190,52,190,53,5,54,190,55,190,422,61,831,14,834,5,190,46,190,47,190,48,190,422,
//611,83211,13,834,12,83211,12,0};//lujin

/********************红色区域*********************************/
//int lujin[]={12,43,190,44,221,45,190,422,11,731,12,733,11,731,5,46,190,47,190,48,190,422,61,734,11,43,190,44,2211,45,190,422,7311,5,49,190,501,190,511,190,422,61,
//733,13,43,190,44,2211,45,190,422,7311,5,49,190,50,190,51,190,422,61,7321,12,7342,11,43,190,44,2211,45,190,422,11,731,12,7332,11,7311,5,46,190,47,190,48,190,422,
//61,7332,13,732,43,190,44,221,45,190,422,7342,12,73122,11,734,11,73122,5,46,190,47,190,48,190,422,
//61,73322,13,73222,12,43,190,44,2211,45,190,422,7341,11,731222,12,7341,11,731222,11,733222,2211,731222,5,49,190,50,190,51,190,422,0};//完整路径planA红色（带二个环纠正）

//int lujin[]={12,7311,43,190,44,221,45,190,422,12,733,5,49,190,50,190,51,190,422,61,732,13,43,190,44,221,45,190,422,731,14,733,5,49,190,50,190,51,190,422,
//61,732,13,733,43,190,44,221,45,190,422,13,7311,13,73411,5,46,190,471,190,481,190,422,611,
//83211,13,734,11,7321,190,52,190,53,5,54,190,55,190,422,61,7331,11,73111,13,73411,5,46,190,471,190,481,190,422,611,
//73211,13,734,12,73211,12,0};//红色区域对抗planA

//int lujin[]={12,43,190,44,221,45,190,422,13,731,13,734,5,46,190,47,190,48,190,422,
//611,732,14,734,190,52,190,53,5,54,190,55,190,422,61,731,14,734,5,190,46,190,47,190,48,190,422,
//611,732,13,734,12,732,12,0};

//int lujin[]={12,43,190,44,221,45,190,422,11,731,13,7331,555,46,190,47,190,48,190,422,
//61,732,14,7331,190,52,190,53,5,54,190,55,190,422,61,731,14,7331,555,46,190,47,190,48,190,422,
//61,732,12,7331,190,52,190,53,5,54,190,55,190,422,190,42,61,732,13,0};//planC 
/*------------------------------------3.17最新调试------------------------------------------------------*/


//int lujin[]={12,43,190,44,221,45,190,422,11,890,12,8180,11,8905,5,46,190,47,190,48,190,422,
//611,8359,11,43,190,44,2211,45,190,422,890,5,49,190,50,190,51,190,422,611,
//8180,13,43,190,44,2211,45,190,422,889,5,49,190,50,190,51,190,422,611,
//8268,12,8359,11,43,190,44,221,45,190,422,11,890,12,8180,11,891,5,46,190,47,190,48,190,422,611,
//8180,13,8270,43,190,44,2211,45,190,422,8360,12,890,11,8361,11,892,5,4S6,190,47,190,48,190,422,611,
//8180,13,8270,12,43,190,44,221,45,190,422,8361,12,890,13,8182,11,892,5,49,190,50,190,51,190,422,611,
//0};

//8359,15,8271,43,190,44,221,45,190,422,8181,12,890,11,8180,11,892,5,46,190,47,190,48,190,422,611,
////////8359,13,8271,12,43,190,44,221,45,190,422,8180,12,890,13,8360,11,890,5,49,190,50,190,51,190,422,611,

//int lujin[]={189,790,0};

//int lujin[]={14,0};
//int lujin[]={12,0};
// int lujin[]={190,46,190,471,190,481,190,42,0};

//int lujin[]={555,49,190,1000,50,190,1000,51,1000,190,42,1000,0};

//int lujin[]={12,
//  	42,190,43,190,44,221,45,190,422,
// 	11,890,12,188,222,8181,555,190,46,190,471,190,481,190,0};//,
//	//190,471,190,481,190,42,0};

//FIXME:比赛类型
#define RUNS 'T'

#if RUNS=='R'
#define PLANR 'A'
   int lujin[]={
//		 221,42,190,43,190,44,188,221,190,45,190,422,0
//		21,43,190,44,190,45,221,890,

//		 12,42,190,43,190,221,44,
//		 12,
//   	42,190,43,190,44,221,45,190,422,
//   	11,890,12,188,222,8181,555,
//   	190,46,190,471,190,481,190,42,
	
	#if PLANR=='A'
	611,8271,11,8360,
	43,190,44,221,45,190,422,//抓环
	889,555,
		 49,190,50,190,51,190,42,//放三倍区
	
	611,8359,11,8270,
	43,190,44,221,45,190,422,
	8180,12,890,11,188,222,8179,555,
	190,46,190,471,190,481,190,42,
	

	611,8270,11,8180,12,
	43,190,44,221,45,190,422,	
	888,555,49,190,50,190,51,190,42,
	#elif PLANR=='B'
	611,8271,13,8180,
	43,190,44,221,45,190,422,
	61,890,12,188,222,8182,555,
	190,46,190,471,190,481,190,42,//放二倍区

	611,8271,11,8359,
	43,190,44,221,45,190,422,891,555,
	49,190,50,190,51,190,42,
	
	611,8180,12,188,221,
	43,190,44,221,45,190,422,888,555,
	49,190,50,190,51,190,42,

	611,8270,11,
	43,190,44,221,45,190,422,
	890,11,188,221,555,
	49,190,50,190,51,190,42,

	611,8180,11,8270,


	43,190,44,221,45,190,422,
	61,8360,14,890,222,8180,555,
	190,46,190,471,190,481,190,42,

	#elif PLANR=='C'
	611,8271,13,8180,
	43,190,44,221,45,190,422,
	61,890,12,188,222,8181,555,
	190,46,190,471,190,481,190,42,

	611,8271,11,8360,
  43,190,44,221,45,190,422,
  889,555,49,190,50,190,51,190,42,

	611,8181,
	43,190,44,221,45,190,422,61,
	888,555,49,190,50,190,51,190,42,
	
	#elif PLANR=='D'
	
	#endif
	
   	/* 611,8180,221,8270,
   	43,190,44,221,45,190,422,
   	8360,221,892,221,
   	555,49,190,50,190,51,190,42,
	
   	611,8270,11,
   	43,190,44,221,45,190,422,
   	890,11,188,221,
   	555,49,190,50,190,51,190,42,
	
   	611,8271,11,188,221,8360,11,	
   	43,190,44,221,45,190,422,
   	221,890,12,188,221,8182,555,
   	190,46,190,471,190,481,190,42, */

     0};
#elif RUNS=='K'
#define PLANP 'A'
//TODO:淘汰赛
int lujin[]=
{
	12,
  42,190,43,190,44,221,45,190,422,
  11,890,12,188,222,8180,555,
  190,46,190,471,190,481,190,42,
#if PLANP=='A'
	611,8270,13,188,221,8181,555,
	190,52,190,53,190,54,190,422,
	611,890,13,188,222,8181,555,
	190,46,190,471,190,481,190,42,
	
	611,8271,11,8359,
	43,190,44,221,45,190,422,
	890,555,49,190,50,190,51,190,42,
	
	62,8359,
	43,190,44,221,45,190,422,
	890,11,188,221,8180,555,
	49,190,50,190,51,190,42,
	
	611,8270,15,8180,13,8270,555,
	190,52,190,53,190,54,190,422,
	
	611,890,42,

#elif PLANP=='B'
	611,8270,11,188,221,8180,555,
	190,52,190,53,190,54,190,422,
	611,42,
	
	8270,11,8180,
	43,190,44,221,45,190,422,
	61,890,12,188,222,8181,555,
	190,46,190,471,190,481,190,42,
	
	611,8270,12,8359,11,
	43,190,44,221,45,190,422,
	890,11,188,222,8180,555,
	9,190,50,190,51,190,42,
	
	611,8270,15,8180,13,8270,555,
	190,52,190,53,190,54,190,422,
	
	611,890,42
	
	
#endif
	
	0
};
//转身左偏用220duo b2左碰到为零
#elif RUNS=='T'
//偏左,左边重
//TODO: 测试
int lujin[]={
//	221,42,190,43,190,44,221,190,45,190,422,221,8270,555,7776,7777,49,190,50,190,51,190,42,
	//221,890,43,190,44,190,
	  //221,42,190,43,190,44,221,190,45,190,422,8270,555,//49,190,50,190,51,190,42,
//	221,890,42,190,43,190,44,221,45,190,422,
//221,890,42,190,43,190,44,221,45,190,422,8180,221,890,12,8180,555,46,190,47,190,48,
//	555,46,190,47,190,48,
//	221,555,52,190,53,190,54,190,55,//增益环
//	221,42,190,43,190,44,188,221,190,45,190,422,8270,555,7776,7777,46,190,47,190,48, //三倍区
//		221,890,221,42,190,43,190,44,188,221,190,45,190,422, // 抓环
//	12,43,190,44,221,45,190,422,221,890,12,8180,221,890,555,46,190,47,190,48,190,422,611,8360,
//	221,43,190,44,221,45,190,422,888,555,49,190,50,190,51,190,42,611,8180,22,8270,555,7776,7777,52,190,53,190,54,190,422,
12,43,190,44,221,45,190,422,221,890,
12,//
	8180,220,890,555,
46,190,47,190,48,190,422,61,8361,
221,43,190,44,221,45,190,422,62,889,555,46,190,47,190,48,190,42,61,8179,
	
221,43,190,44,2211,45,190,422,888,555,49,190,50,190,51,190,42,
//221,43,190,44,221,45,190,422,889,12,8915,555,49,190,50,190,51,190,42,
62,8180,43,190,44,221,45,190,422,890,221,188,2211,8359,555,49,190,50,190,51,190,42,
61,8270,12,188,221,43,190,44,221,45,190,422,890,13,188,221,8360,555,49,190,50,190,51,190,42,
61,8270,12,188,221,8360,43,190,44,221,45,190,422,221,890,12,188,221,8360,5,46,190,47,190,48,190,42,
71,8270,12,188,221,8360,43,190,44,221,45,190,422,221,890,12,188,221,8180,555,46,190,47,190,48,190,42,
//611,8270,




//	12,43,190,44,221,45,190,422,221,890,12,8180,221,890,555,
//	46,190,47,190,48,190,422,611,8359,
//	221,43,190,44,221,45,190,422,891,555,49,190,50,190,51,190,42,611,8179,
//	13,43,190,44,221,45,190,422,891,555,49,190,50,190,51,42,611,8270,
//	221,43,190,44,221,45,190,422,62,8361,12,891,555,7776,7777,46,190,47,190,48,190,42,
	//8360,12,188,221,889,12,8180,221,888,555,7776,7777,46,190,47,190,48,190,42,



//	611,8270,
//	221,890,221,8180,221,8270,221,
//   221,42,190,43,190,
//	12,188,221,890,221,8179,221,8270,221,
//   188,221,890,22,
// 22,188,221,890,22,8180,
	
	
//	12,
//	42,190,43,190,44,221,45,190,422,
//	221,890,12,188,221,8181,
//	555,190,46,190,47,190,48,190,42,
//	
//	611,8271,221,8360,
//	42,190,43,190,44,221,45,190,422,
//	61,890,221,8181,
//	555,190,46,190,47,190,48,190,42,

//	611,8271,221,8360,12,8270,
//	42,190,43,190,44,221,45,190,422,
//	8180,11,890,221,188,555,
//	190,49,190,50,190,51,190,42,
//	
//	611,8180,13,
//	42,190,43,190,44,221,45,190,422,
//	888,
//	555,190,49,190,50,190,51,190,42,
0
};
#endif

//int lujin[]={189,8270,0};
//int lujin[]={11,0};
// int lujin[]={13,0};
//int lujin[]={190,422,190,49,190,50,0};
//int lujin[]={11,8270,0};
//int lujin[]={555,422,190,46,190,471,0};
//int lujin[]={190,43,190,44,221,45,190,422,11,188,555,49,190,50,190,51,0};

//
//
//


portTickType gettime;

int i;
int target;
extern  RC_ctrl_t rc_ctrl;
bool mode_change = FALSE;

/*----------------------------------------------***定义底盘全向移动变量***-------------------------------------------------*/
float Chassis_Move_Z;//左右旋转
float Target_Angle = 0;

/*-----------------------------------------------***速度限幅***-------------------------------------------------------------*/
float Chassis_Standard_Move_Max;//底盘前后左右平移限速(13000最大值)
float Chassis_Revolve_Move_Max;//底盘左右旋转限速,根据不同运动模式实时更改,所以不能用宏定义
float Chassis_Final_Output_Max = LIMIT_CHASSIS_MAX;//限制PID运算最终输出值,底盘功率限制,根据电机转速实时变化
float vz_min_speed;  //旋转限幅（5600）
float vz_max_speed;  //旋转限幅（5600）

/*-----------------------------------------------***定义PID参数***--------------------------------------------------------------*/
float kRc_Mech_Chassis_Standard;
float kRc_Mech_Chassis_Revolve;
//底盘期望速度
float Chassis_Speed_Target[4];//ID
//底盘速度误差
float Chassis_Speed_Error[4];//ID
//底盘测量速度
float Chassis_Speed_Measure[4];
//底盘测量速度
float Chassis_Current_Measure[4];
//底盘速度误差和
float Chassis_Speed_Error_Sum[4];//ID
float Chassis_Speed_Error_NOW[4], Chassis_Speed_Error_LAST[4];
//单级PID参数
float Chassis_Speed_kpid[4][3];//	motorID kp/ki/kd

float   pTermChassis[4], iTermChassis[4], dTermChassis[4];//ID
float	  pidTermChassis[4];//ID,计算输出量


/*------------------------------------------------***位置PID***---------------------------------------------------------*/
float Angle_Measure_Sum[4];//底盘测量角度累计和,用于位置PID
int16_t Angle_Measure_Prev[4];//上次剩下的累加和角度,用于圈数计算判断

//底盘测量角度
float Chassis_Angle_Measure[4];

//底盘期望角度
float  Angle_Target_Sum[4];
float  Chassis_Buff_Target_Sum[4];
//拨盘电机输出量,正数逆时针
float Chasssis_Final_Output;

//拨盘角度误差
float Angle_Error[4][2];//  inner/outer

float pTermAngle[4][2], iTermAngle[4][2],pidTermAngle[4][2];//  inner/outer
float Chassis_Angle_kpid[4][2][3];//  inner/outer    kp/ki/kd
float iTermPosiMax;//位置环积分限幅

//陀螺仪模式下底盘偏差(相对YAW的中心分离机械角度)
fp32 Chassis_Gyro_Error;

PidTypeDef chassis_angle_pid;


//接地气写法，适合初学者使用
/*----------------------------------------------------------------------------------------------------------------------*/

extern float lf,rf,lb,rb;//四个轮子的速度
extern float f1,f2,f3,f4,f5,f6,f7,f8,l1,l2,l3,l4,l5,r1,r2,r3,r4,r5,b1,b2;//循迹
ChassisSetMode ChassisMode;//模式结构体

extern float num;
extern Flag flag;

//陀螺仪参数
float angleMpuPitch,	angleMpuYaw,	angleMpuRoll;//陀螺仪角度值
short palstanceMpuPitch,	palstanceMpuYaw,	palstanceMpuRoll;//陀螺仪角速度值
float angleMpu[3][2];
float AngleMpuYaw_Start;
float Cloud_Angle_Measure_Yaw;
float Cloud_Palstance_Measure_Yaw;
fp32 Angle_error;

#if MODE==TEST /*-------------------------遥控器调试模式-------------------------------*/
//去chassis_task.h里修改状态
void chassis_task(void *pvParameters)
{
//    //空闲一段时间
//    vTaskDelay(CHASSIS_TASK_INIT_TIME);
			for(;;)
			{
				if(SYSTEM_GetSystemState() == SYSTEM_STARTING)  //系统启动
				{
						CHASSIS_REST();
						Chassis_Init();	
						Servo_Init();					
				}
				else
				{
					if (SYSTEM_GetRemoteMode() == RC) //遥控调试模式
					{		
						flow_led_on(0);		
						chassis_feedback_update(Cloud_Angle_Measure_Yaw);	//返回欧拉角误差值与轮子转速
            Chassis_Set_Mode();
            Chassis_Set_Contorl();						
					}
				}
				Chassis_Omni_Move_Calculate();
				Chassis_MotorOutput();//底盘PID计算
				CHASSIS_CANSend();  //Can发送数据
				vTaskDelay(TIME_STAMP_2MS);
			} 
}

#else /*-------------------------正式比赛模式-------------------------------*/
void chassis_task(void *pvParameters)
{
	//    //空闲一段时间
	//    vTaskDelay(CHASSIS_TASK_INIT_TIME);
	for (;;)
	{
		if (SYSTEM_GetSystemState() == SYSTEM_STARTING) //系统启动
		{
			CHASSIS_REST();
			Chassis_Init();
			Servo_Init();
			sensor_update();
		}
		else //正式比赛路径
		{
			sensor_update(); //循迹初始化
			judge();
			Chassis_Set_Contorl();
		}
		Chassis_Omni_Move_Calculate();
		Chassis_MotorOutput(); //底盘PID计算
		CHASSIS_CANSend();	   //Can发送数据
		vTaskDelay(TIME_STAMP_2MS);
	}
}

#endif

//底盘电机输出量
float Chassis_Final_Output[4];
//舵机输出量
float Servo_Output[3];

void Chassis_Init(void)
{
	flag.get = TRUE;
	flag.back = TRUE;
	flag.open_flag = TRUE;
	flag.posi_go = TRUE;
/********************************************/
/**************PID参数*************************/	
	Chassis_Speed_kpid[LEFT_FRON_201][KP] = 2;//11 //4.5
	Chassis_Speed_kpid[LEFT_FRON_201][KI] = 0;//100;90
	Chassis_Speed_kpid[LEFT_FRON_201][KD] = 0;
	
	Chassis_Speed_kpid[RIGH_FRON_202][KP] = 21;//15 //5.5
	Chassis_Speed_kpid[RIGH_FRON_202][KI] = 0.15;//0.08;0 //0.01
	Chassis_Speed_kpid[RIGH_FRON_202][KD] = 0;
	
	Chassis_Speed_kpid[LEFT_BACK_203][KP] = 2;//11//19 //4.5
	Chassis_Speed_kpid[LEFT_BACK_203][KI] = 0;//0.08;0
	Chassis_Speed_kpid[LEFT_BACK_203][KD] = 0;
	
	Chassis_Speed_kpid[RIGH_BACK_204][KP] = 21;//15//5.5
	Chassis_Speed_kpid[RIGH_BACK_204][KI] = 0.15;//0.08; //0.01
	Chassis_Speed_kpid[RIGH_BACK_204][KD] = 0;
	
  for (uint8_t j=0; j<4; j++)
	{
		Chassis_Angle_kpid[j][OUTER][KP] = 0.08;//0.005
		Chassis_Angle_kpid[j][OUTER][KI] = 0;//0.005
		Chassis_Angle_kpid[j][OUTER][KD] = 0;//0.005
		Chassis_Angle_kpid[j][INNER][KP] = 6;//0.005
		Chassis_Angle_kpid[j][INNER][KI] = 0;//0.005
		Chassis_Angle_kpid[j][INNER][KD] = 0;//0.005
	}
	
/********************************************/
  Chassis_Standard_Move_Max = Omni_Speed_Max;
	iTermPosiMax   = 2500;
	const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};  //旋转PID参数设置(调节转向角度)
	PID_Init(&chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
	vz_max_speed = NORMAL_MAX_CHASSIS_SPEED_Z;
	vz_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Z;
	chassis_feedback_update(Cloud_Angle_Measure_Yaw);	//返回欧拉角误差值与轮子转速
}




void Chassis_Set_Mode(void)
{
	if(IF_RC_SW1_DOWN && IF_RC_SW2_UP)
	{
		ChassisMode = AUTO;  //遥控器模式
		//Get();
	}
	else	if(IF_RC_SW1_DOWN && IF_RC_SW2_MID)
	{
		ChassisMode = AUTO;  //遥控器模式
		//TO_Get();
	}
	else	if(IF_RC_SW1_DOWN && IF_RC_SW2_DOWN)
	{
		ChassisMode = AUTO;  //遥控器模式
		Normal();
  }
	else if(IF_RC_SW1_MID && IF_RC_SW2_UP)  
	{
		ChassisMode = AUTO;  //遥控器模式
		//Raise();				
	}
	else if(IF_RC_SW1_MID && IF_RC_SW2_MID)  
	{
		ChassisMode = REST;    //重置标志位
		//ChassisMode = AUTO;
    //drop();		
	}
	else if(IF_RC_SW1_MID && IF_RC_SW2_DOWN)  
	{
		//ChassisMode = REST;    //重置标志位
		ChassisMode = AUTO;	
	}
	else if(IF_RC_SW1_UP && IF_RC_SW2_UP)
	{
		judge();   //路径
		//Place();
	}
//	else if(IF_RC_SW1_UP && IF_RC_SW2_UP)
//	{
//		flag.turn = TRUE;
////		ChassisMode = GYRO_west; //陀螺仪左转
//		Servo_Init();
//	}
//	else if(IF_RC_SW1_UP && IF_RC_SW2_DOWN)
//	{
//		flag.turn = TRUE;
////		ChassisMode = GYRO_east; //陀螺仪右转
//		Servo_Init();
//	} 
//	else if(IF_RC_SW1_UP && IF_RC_SW2_MID)
//	{
//		flag.turn = TRUE;
////		ChassisMode = GYRO_north; 
//		Servo_Init();
//	}
//  else if(IF_RC_SW1_MID && (RC_CH1_RUD_OFFSET > 5))
//	{
//		flag.turn = TRUE;
////		ChassisMode = GYRO_west; 
//		Servo_Init();
//	}		
  else
  {
	  ChassisMode=Stop;
  }
}


/*-------------速度环重置--------------*/
void CHASSIS_REST(void)
{
  motor_speed(0,0,0,0);
	i=0;
	target=180;
	flag.get = FALSE;
	flag.turn = FALSE;
	flag.back = FALSE;
	flag.arm = FALSE;
	flag.back = FALSE;
}


/*-------------位置环重置--------------*/
void POSI_REST(void)
{
	motor_speed(0,0,0,0);
	for (uint8_t j=0; j<4 ;j++)
	{
	  Angle_Target_Sum[j]   = Chassis_Angle_Measure[j];//位置环目标角度重置
	  Angle_Measure_Sum[j]  = Chassis_Angle_Measure[j];//位置环转过角度重置
	  Angle_Measure_Prev[j] = Chassis_Angle_Measure[j];//上次位置重置
		Chassis_Buff_Target_Sum[j] = Angle_Target_Sum[j];
	}
}


/*---------------用陀螺仪转弯以出发位置面对方向为北方向------------------*/
/*
  * @brief  关于陀螺仪转弯部分，是通过debug调试出来，要搞清楚判断为什么这么写就debug试吧
  * @param  void
  * @retval void
  * @attention  degree：   90：正东    180：正北    270：正西      360：正南
  *              	
 */

void Gyro_turn(fp32 degree)
{
	mode_change=TRUE;
	if(flag.turn == TRUE)
	{
		flag.open_flag = FALSE;
		chassis_feedback_update(degree);
	 lf = rf = lb = rb = 0;
	 if(degree < Cloud_Angle_Measure_Yaw)	//判断哪个防线转弯转弯路程短
			Chassis_Move_Z = -PID_Calc(&chassis_angle_pid, 0.0f ,Chassis_Gyro_Error); //逆时针，通过PID结算得出 Chassis_Move_Z
	 else
	 {
		 if( Angle_error > 180)
			 Chassis_Move_Z = PID_Calc(&chassis_angle_pid, 0.0f ,Chassis_Gyro_Error); //顺时针
		 else
			 Chassis_Move_Z = -PID_Calc(&chassis_angle_pid, 0.0f ,Chassis_Gyro_Error); //逆时针
	 }
		Chassis_Move_Z = constrain_float(Chassis_Move_Z, vz_min_speed, vz_max_speed);
	 
	if(Chassis_Gyro_Error == 0)
	{
		stop();
		flag.turn = FALSE;//
		flag.delay = TRUE;
		flag.get=TRUE;
		flag.arm = TRUE;		
		flag.posi_go = TRUE;
		flag.open_flag = TRUE;
		flag.back = TRUE;
	}

	}
	if(flag.delay == TRUE)
		stop_forward(30);	
}

void update_uncorrect_flag(void)
{
	uncorrect_flag=1;
	current_angle=Cloud_Angle_Measure_Yaw;
	if(f3==0)
	{
		current_angle+=5.0f;//3.0
	}
	else if(f2==0)
	{
		current_angle+=5.0f;
	}
	else if(f6==0)
	{
		current_angle-=4.0f;//2.7
	}
	else if(f7==0)
	{
		current_angle-=4.0f;
	}
}

/*TODO:放环纠正*/
void ring_release_correction(void)
{
	extern float f1,f2,f3,f4,f5,f6,f7,f8;
	
	if(uncorrect_flag)
	{
		if(f4==0&&f5==0&&f3==1&&f6==1)
		{
			uncorrect_flag=0;
			i++;
		}
		else 
		{
			Gyro_turn(current_angle);
		}
	}
}

void Gyro_turn1(fp32 degree) //用于直行纠正
{
	mode_change=TRUE;
	if(flag.turn == TRUE)
	{
		flag.open_flag = FALSE;
		chassis_feedback_update(degree);
	 lf = rf = lb = rb = 0;
	 if(degree < Cloud_Angle_Measure_Yaw)	//判断哪个防线转弯转弯路程短
			Chassis_Move_Z = -PID_Calc(&chassis_angle_pid, 0.0f ,Chassis_Gyro_Error); //逆时针，通过PID结算得出 Chassis_Move_Z
	 else
	 {
		 if( Angle_error > 180)
			 Chassis_Move_Z = PID_Calc(&chassis_angle_pid, 0.0f ,Chassis_Gyro_Error); //顺时针
		 else
			 Chassis_Move_Z = -PID_Calc(&chassis_angle_pid, 0.0f ,Chassis_Gyro_Error); //逆时针
	 }
		Chassis_Move_Z = constrain_float(Chassis_Move_Z, vz_min_speed, vz_max_speed);
	 
//	if(Chassis_Gyro_Error == 0)
//	{
//		stop();
//		flag.turn = FALSE;
//		flag.delay = TRUE;
//		flag.get=TRUE;
//		flag.arm = TRUE;		
//		flag.posi_go = TRUE;
//		flag.open_flag = TRUE;
//		flag.back = TRUE;
//	}
	}
	
}


/*---------------------------------------什么模式下怎么动-----------------------------------------*/
 void Chassis_Set_Contorl(void)
{
	switch(ChassisMode)
	{
		case AUTO:
		  AUTO_mode();
		  mode_change = TRUE;
		  flag.arm = TRUE;
    break;

    case REST:
		  CHASSIS_REST();
		  Chassis_Init();
	   	mode_change = TRUE;
		break;
		
		case Stop:
			stop();
		  mode_change = TRUE;
		break;
		case adjust:
			Adjust();
		break;
			
/*-------------------高速-------------------------*/		
		case high_1:
			Forward(HIGH,1,1);
		break;
		
		case high_2:
			Forward(HIGH,2,1);
		break;

		case high_3:
			Forward(HIGH,3,1);
		break;

		case high_4:
			Forward(HIGH,4,1);
		break;

		case high_5:
			Forward(HIGH,5,1);
		break;

		case high_6:
			Forward(HIGH,6,1);
		break;

		case high_7:
			Forward(HIGH,7,1);
		break;

		case high_8:
			Forward(HIGH,8,1);
		break;
		case step:
			Step();
		break;
		case stepyou://左边循迹
			Step1();
		break;
/*--------------------低速------------------------*/	
		case low2_1:
			low_Forward();
		break;
		case low2_0:
			low_Forward0();
		break;
		
		case low_adv_stop:
		    Low_Forward_AdvanceStop();
		break;

		case low2_11:
			low_Forward1();
		break;
		
	 case low2_111:
			low_Forward11();
		break;
		
		case low_1:
			Forward(LOW,1,0);
		break; 

		case low_2:
			Forward(LOW,2,0);
		break;

		case low_3:
			Forward(LOW,3,0);
		break;

		case low_4:
			Forward(LOW,4,0);
		break;

		case low_5:
			Forward(LOW,5,0);
		break;

		case low_6:
			Forward(LOW,6,0);
		break;

		case low_7:
			Forward(LOW,7,0);
		break;		
		
		case low_8:
			Forward(LOW,8,0);
		break;
	
/*--------------------红色区域转向--------------------*/	
		case re90:
			Gyro_turn(90);    //东（90）
		break;
		case re91:
			Gyro_turn(91);    //东（91）
		break;
		case re92:
			Gyro_turn(92);    //东（92）
		break;
		case re89:
			Gyro_turn(89);    //东（89）
		break;
		case re88:
			Gyro_turn(88);    //东（88）
		break;
		/*--------------------------------*/
		case rw270:
			Gyro_turn(270);    //西（270）
		break;
		case rw271:
			Gyro_turn(271);    //西（271）
		break;
		case rw272:
			Gyro_turn(272);    //西（272）
		break;
		case rw269:
			Gyro_turn(269);    //西（269）
		break;
		case rw268:
			Gyro_turn(268);    //西（268）
		break;
    /*--------------------------------*/
    case rs360:
			Gyro_turn(360);     //南（0） 
		break;	
    case rs1:
			Gyro_turn(1);       //南（1） 
		break;	
    case rs2:
			Gyro_turn(2);       //南（2） 
		break;
    case rs359:
			Gyro_turn(359);     //南（359） 
		break;		
		case rs358:
			Gyro_turn(358);     //南（359） 
		break;
		/*--------------------------------*/
		case rn180:
			Gyro_turn(180);     //北（180）
		break;
		case rn181:
			Gyro_turn(181);     //北（181）
		break;
		case rn182:
			Gyro_turn(182);     //北（182）
		break;
		case rn179:
			Gyro_turn(179);     //北（179）
		break;
		case rn178:
			Gyro_turn(178);     //北（178）
		break;
		
		

/*------------------蓝色区域转向----------------------*/	
    case be90:
			Gyro_turn(90);    //东（90）
		break;
		case be905:
			Gyro_turn(90.5);    //东（90.5）
		break;
		case be91:
			Gyro_turn(91);    //东（91）
		break;
			case be915:
			Gyro_turn(91.5);    //东（91.5）
		break;
		case be92:
			Gyro_turn(92);    //东（92）
		break;
		case be89:
			Gyro_turn(89);    //东（89）
		break;
		case be895:
			Gyro_turn(89.5);    //东（89.5）
		break;
		case be88:
			Gyro_turn(88);    //东（88）
		break;
	/*--------------------------------*/
		case bw270:
			Gyro_turn(270);    //西（270）
		break;
		case bw271:
			Gyro_turn(271);    //西（271）
		break;
		case bw272:
			Gyro_turn(272);    //西（272）
		break;
		case bw269:
			Gyro_turn(269);    //西（269）
		break;
		case bw268:
			Gyro_turn(268);    //西（268）
		break;
  /*--------------------------------*/
    case bs360:
			Gyro_turn(360);     //南（0） 
		break;	
    case bs1:
			Gyro_turn(1);       //南（1） 
		break;	
    case bs2:
			Gyro_turn(2);       //南（2） 
		break;
    case bs359:
			Gyro_turn(359);     //南（359） 
		break;		
		case bs358:
			Gyro_turn(358);     //南（359） 
		break;
		/*--------------------------------*/
		case bn180:
			Gyro_turn(180);     //北（180）
		break;
		case bn181:
			Gyro_turn(181);     //北（181）
		break;
		case bn182:
			Gyro_turn(182);     //北（182）
		break;
		case bn179:
			Gyro_turn(179);     //北（179）
		break;
		case bn178:
			Gyro_turn(178);     //北（178）
		break;

/*--------------------转向备用------------------------*/
    case t1:
			Gyro_turn(180);     //备用1
		break;
		case t2:
			Gyro_turn(180);     //备用2
		break;
		case t3:
			Gyro_turn(180);     //备用3
		break;
		case t4:
			Gyro_turn(180);     //备用4
		break;
		case t5:
			Gyro_turn(180);     //备用5
		break;
		case t6:
			Gyro_turn(180);     //备用6
		break;
		case t7:
			Gyro_turn(180);     //备用7
		break;
		case t8:
			Gyro_turn(180);     //备用8
		break;

/*--------------------机械臂动作------------------------*/		
		case paw_init:    //初始化状态--41
			Nor();
		break;
		
		case paw_normal:  //正常行驶舵机状态(不带环)--42
			Normal();
		break;
		
		case paw_normal2:
			Normal2();      //正常行驶状态（带环）-422
		break;
/*--------------------机械臂抓环----------------------*/
		
		case paw_raise:  //举起爪子（去抓的第一步）--43
			Raise();
		break;
		
		case paw_raise2: //举起爪子（去抓的第二步）--44
			Raise2();
		break;
		
		case paw_get:    //爪子抓到环--45
			Get();
		break;
		
/*--------------------机械臂放环(三倍区)----------------------*/			
		
		case place1:
			Place1();   //放环（三倍区）的第一步--46
		break;
		
		case place2:
			Place2();   //放环（三倍区）的第二步--47
		break;
		
		case place222:
			Place222(); //放环（三倍区）的第二步--
		break;
		
		case place3:
			Place3();   //放环（三倍区）--48
		break;
		
		case place333:
			Place333(); //放环（三倍区）--
		break;
		
/*--------------------机械臂放环(二倍区)----------------------*/
		
		case place11:
			Place11(); //放环（二倍区）的第一步--49
		break;
		
		case place21:
			Place21(); //放环（二倍区）的第二步--50
		break;
		
		case place22:
			Place22(); //放环（二倍区）的第二步--501
		break;
		
		case place31:
			Place31(); //放环（二倍区）--51
		break;
		
		case place32:
			Place32(); //放环（二倍区）--511
		break;
		
/*--------------------机械臂放环(增益环)----------------------*/
		case c1:
			Placec1(); //放环（二倍区）--52
		break;
		case c2:
			Placec2();//放环（二倍区）--53
		break;
		case c3:
			Placec3();//放环（二倍区）--54
		break;
		case c4:
			Placec4() ;//放环（二倍区）--55
		break;
				
/*-----------------------标志位手动开启------------------------*/
		case Forward_flag:
			forward_flag();
		break; 
		
		case Arm_flag:
			arm_flag();
		break; 
		
		case Turn_flag:
			turn_flag();
		break; 
/*-----------------------后退------------------------*/	
		case back_1:
			Back(1);
		break;
   case back_11:
			Back1(1);
		break;

 case back_12:
			Back12(1);
		break;
   case back_112:
			Back112(1);
		break; 	 

		case back_2:
			Back(2);
		break;

		case back_3:
			Back(3);
		break;

		case back_4:
			Back(4);
		break;

		case back_5:
			Back(5);
		break;
    case bstep:
			BStep();
		break;	
		case delay1s:
			vTaskDelay(1000);
			i++;
			break;	
		/***********************放环纠正**************************/
		 case ring_release:
		 ring_release_correction();
		 break;
		 case update_uncorrect:
		 update_uncorrect_flag();
		 i++;
		 break;
	}
}



void judge(void)
{
  switch(lujin[i])
	{
		case 0:
			ChassisMode = Stop;
      break;	
		case 02:
			ChassisMode = adjust;
      break;
		
/*--------------------高速1+格数------------------------*/  		
		case 11:
			ChassisMode = high_1;
      break;			
		case 12:
			ChassisMode = high_2;
      break;	
		case 13:
			ChassisMode = high_3;
      break;	
		case 14:
			ChassisMode = high_4;
      break;	
		case 15:
			ChassisMode = high_5;
      break;	
		case 16:
			ChassisMode = high_6;
      break;	
		case 17:
			ChassisMode = high_7;
      break;	
		case 18:
			ChassisMode = high_8;
      break;
	  case  5:
			ChassisMode = step;
      break;
		case 555:
			ChassisMode = stepyou;
      break;
/*--------------------低速2+格数------------------------*/	
	 case 221:
			ChassisMode = low2_1;
	  break;
	 case 220:
			ChassisMode = low2_0;
	  break;
	  case 222:
	  ChassisMode = low_adv_stop;
	  break;
	 case 2211:
			ChassisMode = low2_11;//多走一个循迹灯
      break;
	 
	 case 22111:
			ChassisMode = low2_111;//再多走一个循迹灯
      break;
	 
		case 21:
			ChassisMode = low_1;
      break;			
		case 22:
			ChassisMode = low_2;
      break;	
		case 23:
			ChassisMode = low_3;
      break;	
		case 24:
			ChassisMode = low_4;
      break;	
		case 25:
			ChassisMode = low_5;
      break;	
		case 26:
			ChassisMode = low_6;
      break;	
		case 27:
			ChassisMode = low_7;
      break;	
		case 28:
			ChassisMode = low_8;
      break;	
/*--------------------红色区域转向(开头加7)--------------------*/		
    case 790:
	    ChassisMode = re90;   //东(90)
		  target=90;
     break;
		case 791:
	    ChassisMode = re91;   //东（91）
		  target=90;
     break;
		case 792:
	    ChassisMode = re92;   //东（92）
		  target=90;
     break;
		case 789:
	    ChassisMode = re89;   //东（89）
		  target=90;
      break;
		case 788:
	    ChassisMode = re88;   //东（88）
		  target=90;
      break;
		case 7270:
	    ChassisMode = rw270;  //西(270)
		  target=270;
      break;
    case 7271:
	    ChassisMode = rw271;  //西(271)
	  	 target=270;
      break;
		case 7272:
	    ChassisMode = rw272;  //西(272)
	  	target=270;
      break;
		case 7269:
	    ChassisMode = rw269;  //西(269)
		  target=270;
      break;
	  case 7268:
	    ChassisMode = rw268;  //西(268)
		 target=270;
      break;
		case 7360:
	    ChassisMode = rs360;  //南(360)
		 target=360;
      break;
		case 7361:
	    ChassisMode = rs1;  //南(361)
		 target=360;
      break;
		case 7362:
	    ChassisMode = rs2; //南(362)
		 target=360;
      break;
		case 7359:
	    ChassisMode = rs359; //南(359)
		 target=360;
      break;
		case 7358:
	    ChassisMode = rs358; //南(358)
		  target=360;
      break;
		case 7180:
	    ChassisMode = rn180; //北（180）
		 target=180;
      break;
		case 7181:
	    ChassisMode = rn181; //北（181）
		  target=180;
      break;	
		case 7182:
	    ChassisMode = rn182; //北（182）
		  target=180;
      break;	
		case 7179:
	    ChassisMode = rn179; //北（179）
		  target=180;
      break;	
		case 7178:
	    ChassisMode = rn178; //北（178）
		 target=180;
      break;
		
/*--------------------蓝色区域转向（加8）--------------------*/
		case 890:
	    ChassisMode = be90;   //东(90)
		 target=90;
      break;
		case 8905:
	    ChassisMode = be905;   //东（90.5）
		 target=91;
      break;
		case 891:
	    ChassisMode = be91;   //东（91）
		 target=91;
      break;
		case 8915:
	    ChassisMode = be915;   //东（91.5）
		 target=92;
      break;
		case 892:
	    ChassisMode = be92;   //东（92）
		 target=92;
      break;
		case 889:
	    ChassisMode = be89;   //东（89）
		 target=89;
      break;
		case 8895:
	    ChassisMode = be895;   //东（89.5）
		 target=90;
      break;
		case 888:
	    ChassisMode = be88;   //东（88）
		 target=88;
      break;
		case 8270:
	    ChassisMode = bw270;  //西(270)
		 target=270;
      break;
    case 8271:
	    ChassisMode = bw271;  //西(271)
		 target=271;
      break;
		case 8272:
	    ChassisMode = bw272;  //西(272)
		 target=272;
      break;
		case 8269:
	    ChassisMode = bw269;  //西(269)
		 target=269;
      break;
	  case 8268:
	    ChassisMode = bw268;  //西(268)
		 target=268;
      break;
		case 8360:
	    ChassisMode = bs360;  //南(360)
		 target=360;
      break;
		case 8361:
	    ChassisMode = bs1;  //南(361)
		target=361;
      break;
		case 8362:
	    ChassisMode = bs2; //南(362)
		target=362;
      break;
		case 8359:
	    ChassisMode = bs359; //南(359)
		target=359;
      break;
		case 8358:
	    ChassisMode = bs358; //南(358)
		target=358;
      break;
		case 8180:
	    ChassisMode = bn180; //北（180）
		target=180;
      break;
		case 8181:
	    ChassisMode = bn181; //北（181）
		target=180;
      break;	
		case 8182:
	    ChassisMode = bn182; //北（182）
		target=180;
      break;	
		case 8179:
	    ChassisMode = bn179; //北（179）
		target=180;
      break;	
		case 8178:
	    ChassisMode = bn178; //北（178）
		target=178;
      break;
		
/*--------------------转向备用（加9）--------------------*/			
		case 91:
	    ChassisMode = t1; //备用1 北（180）
		target=90;
      break;
		
		case 92:
	    ChassisMode = t2; //备用2
			target=90;
      break;
		case 93:
	    ChassisMode = t3; //备用3
			target=90;
      break;
    case 94:
	    ChassisMode = t4; //备用4
			target=90;
      break;
    case 95:
	    ChassisMode = t5; //备用5
			target=90;
      break;
		case 96:
	    ChassisMode = t6; //备用6
			target=90;
      break;
		case 97:
	    ChassisMode = t7; //备用7
			target=90;
      break;
		case 98:
	    ChassisMode = t8; //备用8
			target=90;
      break;
			
/*--------------------机械臂----------------------*/
		case 40:
      ChassisMode = paw_drop; //（之前先举起爪子）放开爪子	
		   break;
		
		case 41:
			ChassisMode = paw_init; //初始状态
		  break;
		
		case 42:
			ChassisMode = paw_normal; //正常行驶状态（不带环，用于启动后机械臂伸展）
		  break;
		
		case 422:
			ChassisMode = paw_normal2; //正常行驶状态（带环）
		   break;
/*--------------------机械臂抓地上的环----------------------*/
		
		case 43:
			ChassisMode = paw_raise; //举起爪子（去抓的第一步）
		   break;
		case 44:
			ChassisMode = paw_raise2; //举起爪子（去抓的第二步）
		   break;
		case 45:
			ChassisMode = paw_get; //爪子抓到环（去抓的第三步）
		  break;

/*--------------------机械臂放环(三倍区)----------------------*/	
		  
    case 46:
      ChassisMode = place1; //放环（三倍区）第一步	
		break;

    case 47:
      ChassisMode = place2;	//放环（三倍区）第二步
		break;	
		
		case 48:
      ChassisMode = place3;//放环（三倍区）第三步	
		break;
		
		case 471:
      ChassisMode = place222;	//放环（三倍区）（?）
		break;	
		case 481:
      ChassisMode = place333;//放环（三倍区）	（?）
		break;
		
/*--------------------机械臂放环(二倍区)----------------------*/	
    case 49:
      ChassisMode = place11; //放环（二倍区）的第一步	
		   break;
		
		case 501:
      ChassisMode = place22;	//放环（二倍区）的第二步
		   break;
		 
		case 511:
      ChassisMode = place32;//放环（二倍区）第三步	
		  break;

    case 50:
      ChassisMode = place21;	//放环（二倍区）的第二步（？）
		   break;	
			
		case 51:
      ChassisMode = place31;//放环（二倍区）第三步（？）
		  break;		
		
	
/*--------------------机械臂抓环(增益环)（先伸抓再前进抓）----------------------*/	
		case 52:
      ChassisMode = c1; //抓环（增益）的第一步	
		   break;
		
		case 53:
      ChassisMode = c2; //抓环（增益）的第二步	
		   break;
		
		case 54:
      ChassisMode = c3; //抓环（增益）的第三步	
		   break;
		case 55:
      ChassisMode = c4; //取环	
		   break;
		
/*-----------------------标志位手动开启函数------------------------*/
    case 188:
      ChassisMode = Forward_flag;
       break;
    case 189:
      ChassisMode = Turn_flag;
       break;
    case 190:
      ChassisMode = Arm_flag;
       break;	
/*-----------------------后退6+格数------------------------*/	
    case 61:
      ChassisMode = back_1;//左边3停 5
	  break;
	 case 611:
		ChassisMode = back_11;//左边4停 5
		break;
	
	 case 71:
      ChassisMode = back_12;//右边3停 555
	  break;
	 case 711:
		ChassisMode = back_112;//右边4停  555
		break;
	
	case 62:
		ChassisMode = back_2;
		break;
	case 63:
		ChassisMode = back_3;
		break;
	case 64:
		ChassisMode = back_4;
		break;
	case 65:
		ChassisMode = back_5;
		break;
	case 66:
		ChassisMode = bstep;
		break;
	case 1000:
		ChassisMode = delay1s;
		break;
	case 7777:
		ChassisMode=ring_release;
		break;
	case 7776:
		ChassisMode=update_uncorrect;
		break;
	}
}


/*-------标志位启用函数(按照实际情况是否使用,一般用于路径中连续执行同一个模块的程序，比如机械臂抓取、投放这两段路径中间需要打开标志位)--------*/

void arm_flag(void)
{
	if(flag.open_flag == TRUE)
	{
		flag.arm = TRUE;
		flag.get = FALSE;
		flag.turn = FALSE;
		flag.back = FALSE;
		i++;
	}
}

void forward_flag(void)
{
	if(flag.open_flag == TRUE)
	{
		flag.arm = FALSE;
		flag.get = TRUE;
		flag.turn = FALSE;
		flag.back = FALSE;
		i++;
	}
}

void turn_flag(void)
{
	if(flag.open_flag == TRUE)
	{
		flag.arm = FALSE;
		flag.get = FALSE;
		flag.turn = TRUE;
		flag.back = FALSE;
		i++;
	}
}
/*------------------------------------------人机交互模式，用于调试和玩耍---------------------------------------------*/
void AUTO_mode(void)
{
	float k_rc_z=1;
	kRc_Mech_Chassis_Standard = 15.f;  //调节摇杆灵敏度,调节速度
	kRc_Mech_Chassis_Revolve  = 11.4f; //调节机械模式摇杆扭头灵敏度(太小会影响最高速度)
	Chassis_Move_Z = constrain_float( kRc_Mech_Chassis_Revolve*RC_CH0_RLR_OFFSET, -Chassis_Standard_Move_Max, Chassis_Standard_Move_Max);//旋转
		
	if(fabs(Chassis_Move_Z) > 800)//扭头速度越快,前后速度越慢,防止转弯半径过大
		{
			k_rc_z = ( (Chassis_Revolve_Move_Max - fabs(Chassis_Move_Z) + 800) * (Chassis_Revolve_Move_Max - fabs(Chassis_Move_Z) + 800) )
						/ ( Chassis_Revolve_Move_Max * Chassis_Revolve_Move_Max );
			
			k_rc_z = constrain_float(k_rc_z,0,1);
		}
		else
		{
			k_rc_z = 1;
		}
		
		lf = constrain_float( kRc_Mech_Chassis_Standard*RC_CH3_LUD_OFFSET, -Chassis_Standard_Move_Max*k_rc_z, Chassis_Standard_Move_Max*k_rc_z);//前后
    rf=lb=rb=lf;		
}


/*-------------------------------------------算法------------------------------------------------*/
/**
  * @brief  底盘全向算法,计算各电机转速
  * @param  void
  * @retval void
  * @attention  (201/202/203/204 --> 左前/右前/左后/右后)
  *              	X前(+)后(-)     Y左(-)右(+)     Z扭头
 */

void Chassis_Omni_Move_Calculate(void)
{	
	float speed_max;
  static fp32 k_z = 1;
	
	if(fabs(Chassis_Move_Z) > 800)//旋转速度越快,前后速度越慢,防止矫正时半径过大
	{
		k_z = ( (speed_max - fabs(Chassis_Move_Z) + 800) * (speed_max - fabs(Chassis_Move_Z) + 800) )
					/ ( speed_max * speed_max );
		
		k_z = constrain_float(k_z,0,1);
	}
	else
	{
		k_z = 1;
	}
		
	speed_max = 13000;
	
	Chassis_Speed_Target[LEFT_FRON_201] = +(lf + Chassis_Move_Z);
	Chassis_Speed_Target[RIGH_FRON_202] = -(rf - Chassis_Move_Z);
	Chassis_Speed_Target[LEFT_BACK_203] = +(lb + Chassis_Move_Z);
	Chassis_Speed_Target[RIGH_BACK_204] = -(rb - Chassis_Move_Z);

	
	Chassis_Speed_Target[LEFT_FRON_201] = constrain_float( Chassis_Speed_Target[LEFT_FRON_201], -speed_max * k_z, speed_max * k_z);
	Chassis_Speed_Target[RIGH_FRON_202] = constrain_float( Chassis_Speed_Target[RIGH_FRON_202], -speed_max * k_z, speed_max * k_z);
	Chassis_Speed_Target[LEFT_BACK_203] = constrain_float( Chassis_Speed_Target[LEFT_BACK_203], -speed_max * k_z, speed_max * k_z);
	Chassis_Speed_Target[RIGH_BACK_204] = constrain_float( Chassis_Speed_Target[RIGH_BACK_204], -speed_max * k_z, speed_max * k_z);
}


/**
  * @brief  分别对4个电机做PID运算,计算最终输出电流(发送给电调的值)
  * @param  void
  * @retval void
  * @attention  (201/202/203/204 --> 左前/右前/左后/右后)
  */
void Chassis_MotorOutput(void)
{
	if(ChassisMode == ChassisPosiMode)
	{
		Chassis_Motor_Angle_PID(LEFT_FRON_201);
		Chassis_Motor_Angle_PID(RIGH_FRON_202);
		Chassis_Motor_Angle_PID(LEFT_BACK_203);
		Chassis_Motor_Angle_PID(RIGH_BACK_204);
	}
	else
	{
		Chassis_Motor_Speed_PID(LEFT_FRON_201);
		Chassis_Motor_Speed_PID(RIGH_FRON_202);
		Chassis_Motor_Speed_PID(LEFT_BACK_203);
		Chassis_Motor_Speed_PID(RIGH_BACK_204);
	}
}


/*---------------------------------------------速度环PID------------------------------------------*/
/**
  * @brief  底盘电机PID计算,单级
  * @param  电机ID
  * @retval void
  * @attention  (201/202/203/204 --> 左前/右前/左后/右后)
  */
void Chassis_Motor_Speed_PID( ChassisWheel Wheel ) 
{

	//计算速度误差
	Chassis_Speed_Error[Wheel] = Chassis_Speed_Target[Wheel] - Chassis_Speed_Measure[Wheel];
	//Chassis_Speed_Error[Wheel] = KalmanFilter(&Chassis_Speed_Kalman[Wheel], Chassis_Speed_Error[Wheel]);
	Chassis_Speed_Error_Sum[Wheel] += Chassis_Speed_Error[Wheel];
	
	pTermChassis[Wheel] =  Chassis_Speed_Error[Wheel]*Chassis_Speed_kpid[Wheel][KP];
	iTermChassis[Wheel] =  Chassis_Speed_Error_Sum[Wheel]*Chassis_Speed_kpid[Wheel][KI] * 0.002f;
	//积分限幅
	iTermChassis[Wheel] = constrain_float(iTermChassis[Wheel],-iTermChassis_Max,iTermChassis_Max);
	
	Chassis_Speed_Error_NOW[Wheel] = Chassis_Speed_Error[Wheel];
	dTermChassis[Wheel] = (Chassis_Speed_Error_NOW[Wheel] - Chassis_Speed_Error_LAST[Wheel])*Chassis_Speed_kpid[Wheel][KD];
	Chassis_Speed_Error_LAST[Wheel] = Chassis_Speed_Error_NOW[Wheel];
	
	//积分项缓慢减小,防止误差为0时突然失力
	if( pTermChassis[Wheel] * iTermChassis[Wheel] < 0 )
	{
		Chassis_Speed_Error_Sum[Wheel] = constrain_float(Chassis_Speed_Error_Sum[Wheel],
															-(3000/Chassis_Speed_kpid[Wheel][KI]/5.f),
															 (3000/Chassis_Speed_kpid[Wheel][KI]/5.f));
	}
	
	pidTermChassis[Wheel] = pTermChassis[Wheel] + iTermChassis[Wheel] + dTermChassis[Wheel];

	pidTermChassis[Wheel] = constrain_float(pidTermChassis[Wheel],-Chassis_Final_Output_Max,Chassis_Final_Output_Max);	
	
	//记录输出电流
	Chassis_Final_Output[Wheel] = pidTermChassis[Wheel];
}



/*----------------------------------------------位置环PID--------------------------------------------*/
void Chassis_Motor_Angle_PID( ChassisWheel Wheel ) 
{
	
	//获取转过的总角度值
	MotorAngleSum(LEFT_FRON_201);
	MotorAngleSum(RIGH_FRON_202);
	MotorAngleSum(LEFT_BACK_203);
	MotorAngleSum(RIGH_BACK_204);  
	//外环计算
	Angle_Error[Wheel][OUTER] = Angle_Target_Sum[Wheel] - Chassis_Angle_Measure[Wheel];
	pTermAngle[Wheel][OUTER] = Angle_Error[Wheel][OUTER] * Chassis_Angle_kpid[Wheel][OUTER][KP];
	iTermAngle[Wheel][OUTER] = Angle_Error[Wheel][OUTER] * Chassis_Angle_kpid[Wheel][OUTER][KI];
	pidTermAngle[Wheel][OUTER] = pTermAngle[Wheel][OUTER] + iTermAngle[Wheel][OUTER];
	//内环计算
	Angle_Error[Wheel][INNER]  =  pidTermAngle[Wheel][OUTER] - Chassis_Speed_Measure[Wheel];
	pTermAngle[Wheel][INNER]   = Angle_Error[Wheel][INNER] * Chassis_Speed_kpid[Wheel][KP];		
	iTermAngle[Wheel][INNER]  += Angle_Error[Wheel][INNER] * Chassis_Speed_kpid[Wheel][KI] * 0.001f;
	iTermAngle[Wheel][INNER]   = constrain_float( iTermAngle[Wheel][INNER], -iTermPosiMax, iTermPosiMax );

	Chassis_Final_Output[Wheel] = constrain_float( pTermAngle[Wheel][INNER] + iTermAngle[Wheel][INNER] , -6000, 6000);
}



void MotorAngleSum( ChassisWheel Wheel )
{		 
	//临界值判断法
	if (abs(Chassis_Angle_Measure[Wheel] - Angle_Measure_Prev[Wheel]) > 4095)//转过半圈
	{		
		//本次测量角度小于上次测量角度且过了半圈,则说明本次过了零点
		if (Chassis_Angle_Measure[Wheel] < Angle_Measure_Prev[Wheel])//过半圈且过零点
		{
			//已经转到下一圈,则累计转过 8191(一圈) - 上次 + 本次
			Angle_Measure_Sum[Wheel] += 8191 - Angle_Measure_Prev[Wheel] + Chassis_Angle_Measure[Wheel];
		}
		else
		{
			//超过了一圈
			Angle_Measure_Sum[Wheel] -= 8191 - Chassis_Angle_Measure[Wheel] + Angle_Measure_Prev[Wheel];
		}
	}
	else      
	{
		//未过临界值,累加上转过的角度差
		Angle_Measure_Sum[Wheel] += Chassis_Angle_Measure[Wheel] - Angle_Measure_Prev[Wheel];
	}

	//记录此时电机角度,下一次计算转过角度差用,用来判断是否转过1圈
	Angle_Measure_Prev[Wheel] = Chassis_Angle_Measure[Wheel];
}



/**
  * @brief  更新底盘电机数据 速度  
  * @param  void
  * @retval void
  * @attention  
  *              
  */
void chassis_feedback_update(fp32 angle)
{
	 Chassis_Gyro_Error = 	Chassis_GetOffsetAngle(angle);
}


/**
  * @brief  计算YAW偏离中心角度,底盘跟随模式用
  * @param  void
  * @retval sAngleError,偏离角度值,CAN反馈的机械角度
  */
float Chassis_GetOffsetAngle(fp32 angle)
{
	float sAngleError = 0;
	Target_Angle = angle;
	
	Angle_error = fabs(Target_Angle - Cloud_Angle_Measure_Yaw);	
	if( fabs(Target_Angle - Cloud_Angle_Measure_Yaw) <= 2.38 )
	{
		sAngleError = 0;
	}
	else
	{
		if(Angle_error > 180) //使最终的误差的绝对值小于180
		{
			if(Target_Angle < 180)
			{
			 sAngleError = 360 - (Cloud_Angle_Measure_Yaw - Target_Angle);
			}	
      else
			{
				sAngleError = 360 - (Target_Angle - Cloud_Angle_Measure_Yaw);
			}				
		}
    else 
		{
			sAngleError = Target_Angle - Cloud_Angle_Measure_Yaw;
		}			
	}
	
	return  sAngleError;
}

/**
  * @brief  获取电机角度
  * @param  ID,CAN数据
  * @retval void
  * @attention  (201/202/203/204 --> 左前/右前/左后/右后),CAN1中调用
  */
void CHASSIS_UpdateMotorAngle( ChassisWheel Wheel, int16_t angle )
{
    Chassis_Angle_Measure[Wheel] = angle;
}


/**
  * @brief  获取电机转速
  * @param  ID,CAN数据
  * @retval void
  * @attention  (201/202/203/204 --> 左前/右前/左后/右后),CAN1中调用
  */
void CHASSIS_UpdateMotorSpeed( ChassisWheel Wheel, int16_t speed )
{
	Chassis_Speed_Measure[ Wheel ] = speed;
}


/**
  * @brief  获取电机转矩电流
  * @param  ID,CAN数据
  * @retval void
  * @attention  (201/202/203/204 --> 左前/右前/左后/右后),CAN1中调用
  */
void CHASSIS_UpdateMotorCur( ChassisWheel Wheel, int16_t current )
{
	Chassis_Current_Measure[ Wheel ] = current;
}

uint8_t mpu_data_cnt=0;
uint8_t mpu_angle_data_cnt1=0;
uint8_t mpu_angle_data_cnt2=0;

void MPU_Update_Starting(void)
{
	#if 0
	mpu_dmp_get_data( &angleMpuRoll, &angleMpuPitch, &angleMpuYaw );
	AngleMpuYaw_Start = angleMpuYaw;
	#else
	
	while(Judge_Buf[0][mpu_data_cnt]!=0x53)
	{
		mpu_data_cnt++;
		mpu_data_cnt%=55;
	}
	mpu_angle_data_cnt1=(mpu_data_cnt+5)%55;
	mpu_angle_data_cnt2=(mpu_data_cnt+6)%55;

	AngleMpuYaw_Start=(int16_t)(Judge_Buf[0][mpu_angle_data_cnt2]<<8|
								Judge_Buf[0][mpu_angle_data_cnt1])*180.0f/32768.0f;
	#endif
}

float Raw_angleMpuYaw;

void MPU_Update()
{
	#if 0
	gettime = xTaskGetTickCount();	//获取当前系统时间
	//读取陀螺仪  角度   角速度   
	mpu_dmp_get_data( &angleMpuRoll, &angleMpuPitch, &angleMpuYaw );
	MPU_Get_Gyroscope( &palstanceMpuPitch, &palstanceMpuRoll, &palstanceMpuYaw );
	#endif
	Raw_angleMpuYaw=(int16_t)(Judge_Buf[0][mpu_angle_data_cnt2]<<8|
							  Judge_Buf[0][mpu_angle_data_cnt1])*180.0f/32768.0f;
	/* angleMpuYaw=Raw_angleMpuYaw*180.0f/32768.0f; */
	//将陀螺仪角度放大
	Cloud_Angle_Measure_Yaw = Raw_angleMpuYaw - AngleMpuYaw_Start + 180 ;
//  if(gettime>=50000)
//		Cloud_Angle_Measure_Yaw=Cloud_Angle_Measure_Yaw-0.4f;
//	else if(gettime>=80000)
//		Cloud_Angle_Measure_Yaw=Cloud_Angle_Measure_Yaw-0.5f;
	
		//角速度更新
	#if 0
  Cloud_Palstance_Measure_Yaw   = palstanceMpuYaw+PALST_COMPS_YAW;
  #endif
		
}


/**
  * @brief  发送电机最终电流值
  * @param  void
  * @retval void
  * @attention  CAN1发送
  */
void CHASSIS_CANSend(void)
{	 	
	CAN_CMD_CHASSIS(Chassis_Final_Output[0],Chassis_Final_Output[1],Chassis_Final_Output[2],Chassis_Final_Output[3]);
}

