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
#include "key.h"
#include "CAN_Receive.h"
#include "pid.h"
#include "stdio.h"
#include <stdlib.h>
#include "mpu6050.h"
#include "inv_mpu.h"


#define abs(x) ((x)>0? (x):(-(x)))

#define RADIUS     76      //麦轮半径
#define PERIMETER  478     //麦轮周长
#define WHEELTRACK 536     //左右轮距
#define WHEELBASE  368     //前后轴距
#define RADIAN_COEF 57.3f  
/*--------------------高速1+格数------------------------*/ 
/*--------------------低速2+格数（221:超慢速一格）------------------------*/	

//20:用于放砖

//10:用于出门


//41:初始状态 42:正常行驶状态 43:举起爪子
//51，52:停一会儿（具体时间可以在下面改）


/*-----------------------后退6+格数------------------------*/

/*-----------------------标志位手动开启函数（188直行，189转弯，190机械臂）------------------------*/

//******切记：连续执行同一路径的时候中间加一个标志位打开函数打开标志位，例如：机械臂（43，190，42）直行（11，188，221）


/*         33：北（180）
               |               |

               |
32：西（270）						 331:东北(91)
<------------     ------------->31：东（90）
                         332:东北(89)
               |
               |
               |
						34：南（360）
*/



/*----------------------------88红色区域---------------------------------------------------------------*/


//int lujin[]={12,42,221,52,21,731,15,43,52,732,12,734,42,12,731,21,733,12,7311,11,0};//红色区域排位赛
//int lujin[]={12,42,15,731,14,43,733,11,732,14,734,12,733,12,734,12,733,12,0};	//一路直接退后堵柱子


	
	/*---------------------------蓝色区域----------------------------------------------------------------*/
////	
//int lujin[]={12,42,221,52,21,831,15,43,52,832,12,834,42,12,831,21,833,12,831,11,0};//蓝色区域排位赛
////int lujin[]={12,42,15,83/1,14,834,11,831,11,43,832,12,833,12,832,13,834,12,
//833,12,834,12,833,12,834,12,0};/planA先推3个环的一路，直接退后堵柱子（主）
//int lujin[]={12,42,16,831,14,834,12,831,11,43,832,11,833,11,832,14,834,11,0};//planB先推两个环的一路，直接堵柱子（红蓝备用方案）
//int lujin[]={12,42,221,52,21,831,0};//蓝色区域排位赛
//12,890,42,12,8180,12,890,13,43,63,8360,42,12,8270,221,8180,13,188,221,890,13,0
//排位赛
extern int mode;
int nbut = 0;
//棒棒动作：42：张开最大 40：收入一个环后限制位置准备收第二个环(第二角度) 401：夹紧两个环(第三角度) 402：偷一个环(最小角度) 44：双爆炸 45：后爆炸
//底盘动作：51：停住一秒 31：前进到循迹倒数第二个灯亮(多走一点) 611：后退到第二个灯停止(后退一点)  (31 611)：配合保护放入一倍区 
//配合放好环后打散两个换：61,401,31,40,8270
//22,42,21,188,220,892,51,12,40,8179,12,401,890,13,42,64,0 排位赛
//22,42,21,188,220,892,12,40,8179,12,401,890,13,42,61,401,31,40,8270,13,0 小组赛初始化放环
//22,42,21,188,220,892,51,12,40,8179,12,401,890,12,188,31,42,611,0 小组赛放好环后用车保护
int lujin1[]={22,42,12,892,12,40,8179,22,401,890,13,42,61,401,31,40,8270,0};//22,42,21,188,220,892,51,21,40,21,8179,221,188,220,401,890,13,42,61,8179,21,401,8360,21,42,61,0排位赛
int lujin2[]={22,42,16,40,188,221,401,890,14,8360,21,890,21,42,61,401,31,40,8270,21,0};//S1 放布
int lujin3[]={22,42,890,12,40,21,401,8180,22,890,22,42,61,401,31,40,8270,0};//S2 放两环
int lujin4[]={22,188,12,42,8270,21,402,61,890,21,188,14,42,61,401,31,40,8270,0};//S3 左启动偷第一个环(近)
             //{22,188,12,188,22,42,8270,21,402,61,890,21,188,14,40,61,401,31,40,8270,0};//   左启动偷第二个环(远) 
						//{22,188,12,42,8270,14,188,21,402,61,890,21,188,14,40,61,401,31,40,8270,0};//右启动偷一
						//{22,188,12,188,22,42,8270,14,188,21,402,61,890,21,188,14,40,61,401,31,40,8270,0};//右启动偷二
int lujin5[]={22,188,14,44,0};//S4 爆炸 15,44,0 双爆炸19,42,8270,40,4360,21,270
int *lu[5]={lujin1,lujin2,lujin3,lujin4,lujin5};
int *cmd;
//int **lujin = lu;
//12,42,14,890,15,221,61,8270,13,188,221,8180,221,8360,221,890,/14,221,61,8270,13,188,221,8360,13,188,221,890,221,8180,/11,188,221,890,13,188,221,0
	
int i;
int target;
extern  RC_ctrl_t rc_ctrl;
long key1,key2;
bool mode_change = FALSE;
/*----------------------------------------------***定义底盘全向移动变量***-------------------------------------------------*/

float Chassis_Move_Z;//左右旋转
float Target_Angle = 0;
/*-----------------------------------------------***速度限幅***-------------------------------------------------------------*/

float Chassis_Standard_Move_Max;//底盘前后左右平移限速
float Chassis_Revolve_Move_Max;//底盘左右旋转限速,根据不同运动模式实时更改,所以不能用宏定义
float Chassis_Final_Output_Max = LIMIT_CHASSIS_MAX;//限制PID运算最终输出值,底盘功率限制,根据电机转速实时变化
float vz_min_speed;  //旋转限幅
float vz_max_speed;  //旋转限幅

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
extern float f1,f2,f3,f4,f5,f6,f7,f8,b1,b2,b3,b4,b5,b6,b7,b8,l1,l2,l3,l4,l5,r1,r2,r3,r4,r5;
ChassisSetMode ChassisMode;

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




#if MODE==TEST /*-------------------------遥控器调试模式，比赛不用-------------------------------*/
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


#elif MODE==AVOI/*-------------------------避障，比赛不需要用-------------------------------*/


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
								sensor_update();
				}
				else  //正式比赛路径
				{				
            sensor_update();//循迹初始化
						avoi();			
				}
			  Chassis_Omni_Move_Calculate();
				Chassis_MotorOutput();//底盘PID计算
				CHASSIS_CANSend();  //Can发送数据
				vTaskDelay(TIME_STAMP_2MS);
			} 
}


#else /*-------------------------正式比赛，比赛只用这段-------------------------------*/
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
								sensor_update();
				}
				else  //正式比赛路径
				{				
            sensor_update();//循迹初始化
						judge();	
            Chassis_Set_Contorl();	
				}
			  Chassis_Omni_Move_Calculate();
				Chassis_MotorOutput();//底盘PID计算
				CHASSIS_CANSend();  //Can发送数据
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
	flag.arm = TRUE;
	flag.open_flag = TRUE;
	flag.posi_go = TRUE;
/********************************************/
/**************PID参数*************************/	
	Chassis_Speed_kpid[LEFT_FRON_201][KP] = 5;//11
	Chassis_Speed_kpid[LEFT_FRON_201][KI] = 0.01;//100
	Chassis_Speed_kpid[LEFT_FRON_201][KD] = 0;
	
	Chassis_Speed_kpid[RIGH_FRON_202][KP] = 5;
	Chassis_Speed_kpid[RIGH_FRON_202][KI] = 0.01;//0.08;
	Chassis_Speed_kpid[RIGH_FRON_202][KD] = 0;
	
	Chassis_Speed_kpid[LEFT_BACK_203][KP] = 5;
	Chassis_Speed_kpid[LEFT_BACK_203][KI] = 0.01;//0.08;
	Chassis_Speed_kpid[LEFT_BACK_203][KD] = 0;
	
	Chassis_Speed_kpid[RIGH_BACK_204][KP] = 5;
	Chassis_Speed_kpid[RIGH_BACK_204][KI] = 0.01;//0.08;
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
	}
	else	if(IF_RC_SW1_DOWN && IF_RC_SW2_MID)
	{
		ChassisMode = AUTO;  //遥控器模式
		//Raise();
	}
	else	if(IF_RC_SW1_DOWN && IF_RC_SW2_DOWN)
	{
		ChassisMode = AUTO;  //遥控器模式
		//Normal();
  }
	else if(IF_RC_SW1_MID && IF_RC_SW2_UP)  
	{
		ChassisMode = AUTO;  //遥控器模式				
	}
	else if(IF_RC_SW1_MID && IF_RC_SW2_MID)  
	{
		ChassisMode = REST;    //重置标志位
    //Normal();		
	}
	else if(IF_RC_SW1_UP && IF_RC_SW2_UP)
	{
		judge();   //路径
	}	
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
		flag.turn = FALSE;
		flag.delay = TRUE;
		flag.get=TRUE;
		flag.arm = TRUE;		
		flag.posi_go = TRUE;
		flag.open_flag = TRUE;
		flag.back = TRUE;
	}
	}
	if(flag.delay == TRUE)
		stop_forward(100);	
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




/*----------------------------------停一会儿-----------------------------*/	
void STOPDELAY(fp32 time) 
{
  static fp32 posi_time;
  if(flag.posi_go == TRUE)	
	{
	  stop();
	  posi_time++;
	}		
	if(posi_time > time)
	{
		stop();
		posi_time = 0;
		flag.posi_go = FALSE;
		flag.get = TRUE;
		flag.arm = TRUE;
		flag.turn = TRUE;
		flag.back = TRUE;
		i++;
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
		
		case out:
			OUT();
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
/*--------------------低速------------------------*/
    case half_1:
      Half_Forward();
    break;
		case low2_0:
			low_Forward0();
		break;
		case low2_1:
			low_Forward();
		break;
		case low2_11:
			low_Forward1();
		break;
		case low3_1:
			low_Forward3();
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
/*--------------------红色区域转向（加7）--------------------*/
		case GYRO_east:
			Gyro_turn(89);
		break;
		case GYRO_east88:
			Gyro_turn(87);
		break;
		case GYRO_west:
			Gyro_turn(270);
		break;
		case GYRO_south:
			Gyro_turn(359);
		break;
		case GYRO_north:
			Gyro_turn(180);
		break;
		
/*--------------------蓝色区域转向（加8）--------------------*/
		case BGYRO_east:
			Gyro_turn(90);//90
		break;
		case BGYRO_east2:
			Gyro_turn(92);//90
		break;
		case BGYRO_west:
			Gyro_turn(270);//270
		break;
		case 	BGYRO_south:
			Gyro_turn(2);
		break;
		case BGYRO_north:
			Gyro_turn(180);
		break;
		case BGYRO_north1:
			Gyro_turn(179);
		break;
/*--------------------转向备用--------------------*/
		
	  case east_89:
			Gyro_turn(89);
		break;
		
	  case east_91:
			Gyro_turn(91);
		break;
	
/*--------------------机械臂动作------------------------*/		
		case paw_init:
			Servo_Init();
		break;
		
		case paw_normal:
			Normal();
		break;
		
		case paw_raise:
			Raise();
		break;
		case booM:
			boom();
		break;
		case booM2:
			boom2();
		break;
		case shou:
			Raise11();
		break;
		case zhua:
			Raise22();
		break;
		case tou:
			Raise33();
		break;
/*-----------------------写死电机编码值----------------------------*/
		case ChassisPosiMode:
			break;
		case STOP1000:
			STOPDELAY(1000);
		break;
		case STOP2000:
			STOPDELAY(2800);//2200  
		break; 
		case STOP3000:
			STOPDELAY(4800);//2200  
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
		case back_11:
			Back11(1);
		break;
	}
}
//void JUDGE(void)
//{ 
//		


//	
//	
//	judge();
//}

void judge(void)
{
  switch(*(lu[nbut]+i))
	{
		case 0:
			ChassisMode = Stop;
      break;
    case 10:
			ChassisMode = out;
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
		case 31:
			ChassisMode = low3_1;
      break;
/*--------------------低速2+格数------------------------*/	
		case 20:
			ChassisMode = half_1;
		  break;
		case 221:
			ChassisMode = low2_1;
      break;
		case 220:
			ChassisMode = low2_0;
      break;
		case 2211:
			ChassisMode = low2_11;//少走一个循迹
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
/*--------------------红色区域转向（加8）--------------------*/
		case 731:
	    ChassisMode = GYRO_east; //东
      break;
		case 7311:
	    ChassisMode = GYRO_east88; //东
      break;
		case 732:
	    ChassisMode = GYRO_west; //西
      break;
	 case 733:
	    ChassisMode = GYRO_north; //北
      break;	
		case 734:
	    ChassisMode = GYRO_south; //南
      break;	
		
/*--------------------蓝色区域转向（加8）--------------------*/
		case 890:
			ChassisMode = BGYRO_east; //东
		target =90;
      break;
		case 892:
			ChassisMode = BGYRO_east2; //东
		target =92;
      break;
		case 8270:
	    ChassisMode = BGYRO_west; //西
		target=270;
      break;
		case 8180:
	    ChassisMode = BGYRO_north; //北
		target=180;
		  break;
 		case 8179:
	    ChassisMode = BGYRO_north1; //北
		target=179;     
		   break;
		case 8360:
	    ChassisMode = BGYRO_south; //南
		target=2;
      break;
		

		
/*--------------------转向备用--------------------*/
		
		case 331:
	    ChassisMode = east_91; //西
      break;
		case 332:
	    ChassisMode = east_89; //西
      break;
		
		
/*--------------------机械臂----------------------*/
		case 41:
			ChassisMode = paw_init; //初始状态
		  break;
		case 42:
			ChassisMode = paw_normal; //正常行驶状态
		   break;
		case 43:
			ChassisMode = paw_raise; //举起爪子
		   break;
		case 44:
			ChassisMode = booM; //爆炸
		   break;
		case 45:
			ChassisMode = booM2; //后爆炸
		   break;
		case 40:
			ChassisMode = shou; //收两个环
		   break;
		case 401:
			ChassisMode = zhua; //抓两个环
		   break;
		case 402:
			ChassisMode = tou; //偷一个环
		   break;

/*-----------------------停几秒----------------------------*/
    case 51:
      ChassisMode = STOP1000;	
       break;
    case 52:
      ChassisMode = STOP2000;	
       break;
		case 53:
      ChassisMode = STOP3000;	
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
      ChassisMode = back_1;
       break;		
    case 611:
      ChassisMode = back_11;
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
	kRc_Mech_Chassis_Standard = 8.f;  //调节摇杆灵敏度,调节速度
	kRc_Mech_Chassis_Revolve  = 5.4f; //调节机械模式摇杆扭头灵敏度(太小会影响最高速度)
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
	if( fabs(Target_Angle - Cloud_Angle_Measure_Yaw) <= 1.0 )
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


void MPU_Update_Starting(void)
{
	mpu_dmp_get_data( &angleMpuRoll, &angleMpuPitch, &angleMpuYaw );

	AngleMpuYaw_Start = angleMpuYaw;
}



void MPU_Update()
{
	//读取陀螺仪  角度   角速度   
	mpu_dmp_get_data( &angleMpuRoll, &angleMpuPitch, &angleMpuYaw );
	MPU_Get_Gyroscope( &palstanceMpuPitch, &palstanceMpuRoll, &palstanceMpuYaw );
	
	//将陀螺仪角度放大
	Cloud_Angle_Measure_Yaw = angleMpuYaw - AngleMpuYaw_Start + 180;
	
		//角速度更新
  Cloud_Palstance_Measure_Yaw   = palstanceMpuYaw+PALST_COMPS_YAW;
		
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
	
	//CAN_CMD_CHASSIS(1000,1000,1000,1000);
}







