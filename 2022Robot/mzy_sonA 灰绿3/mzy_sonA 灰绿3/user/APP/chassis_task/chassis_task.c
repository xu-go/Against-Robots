/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       chassis.c/h
  * @brief      �ӵ���д�����ʺϳ�ѧ��ʹ��
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
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

#define RADIUS     76      //���ְ뾶
#define PERIMETER  478     //�����ܳ�
#define WHEELTRACK 536     //�����־�
#define WHEELBASE  368     //ǰ�����
#define RADIAN_COEF 57.3f  
/*--------------------����1+����------------------------*/ 
/*--------------------����2+������221:������һ��------------------------*/	

//20:���ڷ�ש

//10:���ڳ���


//41:��ʼ״̬ 42:������ʻ״̬ 43:����צ��
//51��52:ͣһ���������ʱ�����������ģ�


/*-----------------------����6+����------------------------*/

/*-----------------------��־λ�ֶ�����������188ֱ�У�189ת�䣬190��е�ۣ�------------------------*/

//******�мǣ�����ִ��ͬһ·����ʱ���м��һ����־λ�򿪺����򿪱�־λ�����磺��е�ۣ�43��190��42��ֱ�У�11��188��221��


/*         33������180��
               |               |

               |
32������270��						 331:����(91)
<------------     ------------->31������90��
                         332:����(89)
               |
               |
               |
						34���ϣ�360��
*/



/*----------------------------88��ɫ����---------------------------------------------------------------*/


//int lujin[]={12,42,221,52,21,731,15,43,52,732,12,734,42,12,731,21,733,12,7311,11,0};//��ɫ������λ��
//int lujin[]={12,42,15,731,14,43,733,11,732,14,734,12,733,12,734,12,733,12,0};	//һ·ֱ���˺������


	
	/*---------------------------��ɫ����----------------------------------------------------------------*/
////	
//int lujin[]={12,42,221,52,21,831,15,43,52,832,12,834,42,12,831,21,833,12,831,11,0};//��ɫ������λ��
////int lujin[]={12,42,15,83/1,14,834,11,831,11,43,832,12,833,12,832,13,834,12,
//833,12,834,12,833,12,834,12,0};/planA����3������һ·��ֱ���˺�����ӣ�����
//int lujin[]={12,42,16,831,14,834,12,831,11,43,832,11,833,11,832,14,834,11,0};//planB������������һ·��ֱ�Ӷ����ӣ��������÷�����
//int lujin[]={12,42,221,52,21,831,0};//��ɫ������λ��
//12,890,42,12,8180,12,890,13,43,63,8360,42,12,8270,221,8180,13,188,221,890,13,0
//��λ��
extern int mode;
int nbut = 0;
//����������42���ſ���� 40������һ����������λ��׼���յڶ�����(�ڶ��Ƕ�) 401���н�������(�����Ƕ�) 402��͵һ����(��С�Ƕ�) 44��˫��ը 45����ը
//���̶�����51��ͣסһ�� 31��ǰ����ѭ�������ڶ�������(����һ��) 611�����˵��ڶ�����ֹͣ(����һ��)  (31 611)����ϱ�������һ���� 
//��Ϸźû����ɢ��������61,401,31,40,8270
//22,42,21,188,220,892,51,12,40,8179,12,401,890,13,42,64,0 ��λ��
//22,42,21,188,220,892,12,40,8179,12,401,890,13,42,61,401,31,40,8270,13,0 С������ʼ���Ż�
//22,42,21,188,220,892,51,12,40,8179,12,401,890,12,188,31,42,611,0 С�����źû����ó�����
int lujin1[]={22,42,12,892,12,40,8179,22,401,890,13,42,61,401,31,40,8270,0};//22,42,21,188,220,892,51,21,40,21,8179,221,188,220,401,890,13,42,61,8179,21,401,8360,21,42,61,0��λ��
int lujin2[]={22,42,16,40,188,221,401,890,14,8360,21,890,21,42,61,401,31,40,8270,21,0};//S1 �Ų�
int lujin3[]={22,42,890,12,40,21,401,8180,22,890,22,42,61,401,31,40,8270,0};//S2 ������
int lujin4[]={22,188,12,42,8270,21,402,61,890,21,188,14,42,61,401,31,40,8270,0};//S3 ������͵��һ����(��)
             //{22,188,12,188,22,42,8270,21,402,61,890,21,188,14,40,61,401,31,40,8270,0};//   ������͵�ڶ�����(Զ) 
						//{22,188,12,42,8270,14,188,21,402,61,890,21,188,14,40,61,401,31,40,8270,0};//������͵һ
						//{22,188,12,188,22,42,8270,14,188,21,402,61,890,21,188,14,40,61,401,31,40,8270,0};//������͵��
int lujin5[]={22,188,14,44,0};//S4 ��ը 15,44,0 ˫��ը19,42,8270,40,4360,21,270
int *lu[5]={lujin1,lujin2,lujin3,lujin4,lujin5};
int *cmd;
//int **lujin = lu;
//12,42,14,890,15,221,61,8270,13,188,221,8180,221,8360,221,890,/14,221,61,8270,13,188,221,8360,13,188,221,890,221,8180,/11,188,221,890,13,188,221,0
	
int i;
int target;
extern  RC_ctrl_t rc_ctrl;
long key1,key2;
bool mode_change = FALSE;
/*----------------------------------------------***�������ȫ���ƶ�����***-------------------------------------------------*/

float Chassis_Move_Z;//������ת
float Target_Angle = 0;
/*-----------------------------------------------***�ٶ��޷�***-------------------------------------------------------------*/

float Chassis_Standard_Move_Max;//����ǰ������ƽ������
float Chassis_Revolve_Move_Max;//����������ת����,���ݲ�ͬ�˶�ģʽʵʱ����,���Բ����ú궨��
float Chassis_Final_Output_Max = LIMIT_CHASSIS_MAX;//����PID�����������ֵ,���̹�������,���ݵ��ת��ʵʱ�仯
float vz_min_speed;  //��ת�޷�
float vz_max_speed;  //��ת�޷�

/*-----------------------------------------------***����PID����***--------------------------------------------------------------*/
float kRc_Mech_Chassis_Standard;
float kRc_Mech_Chassis_Revolve;
//���������ٶ�
float Chassis_Speed_Target[4];//ID

//�����ٶ����
float Chassis_Speed_Error[4];//ID


//���̲����ٶ�
float Chassis_Speed_Measure[4];

//���̲����ٶ�
float Chassis_Current_Measure[4];

//�����ٶ�����
float Chassis_Speed_Error_Sum[4];//ID
float Chassis_Speed_Error_NOW[4], Chassis_Speed_Error_LAST[4];

//����PID����
float Chassis_Speed_kpid[4][3];//	motorID kp/ki/kd

float   pTermChassis[4], iTermChassis[4], dTermChassis[4];//ID
float	  pidTermChassis[4];//ID,���������


/*------------------------------------------------***λ��PID***---------------------------------------------------------*/
float Angle_Measure_Sum[4];//���̲����Ƕ��ۼƺ�,����λ��PID
int16_t Angle_Measure_Prev[4];//�ϴ�ʣ�µ��ۼӺͽǶ�,����Ȧ�������ж�

//���̲����Ƕ�
float Chassis_Angle_Measure[4];

//���������Ƕ�
float  Angle_Target_Sum[4];
float  Chassis_Buff_Target_Sum[4];
//���̵�������,������ʱ��
float Chasssis_Final_Output;

//���̽Ƕ����
float Angle_Error[4][2];//  inner/outer

float pTermAngle[4][2], iTermAngle[4][2],pidTermAngle[4][2];//  inner/outer
float Chassis_Angle_kpid[4][2][3];//  inner/outer    kp/ki/kd
float iTermPosiMax;//λ�û������޷�

//������ģʽ�µ���ƫ��(���YAW�����ķ����е�Ƕ�)
fp32 Chassis_Gyro_Error;

PidTypeDef chassis_angle_pid;






//�ӵ���д�����ʺϳ�ѧ��ʹ��
/*----------------------------------------------------------------------------------------------------------------------*/

extern float lf,rf,lb,rb;//�ĸ����ӵ��ٶ�
extern float f1,f2,f3,f4,f5,f6,f7,f8,b1,b2,b3,b4,b5,b6,b7,b8,l1,l2,l3,l4,l5,r1,r2,r3,r4,r5;
ChassisSetMode ChassisMode;

extern float num;
extern Flag flag;

//�����ǲ���
float angleMpuPitch,	angleMpuYaw,	angleMpuRoll;//�����ǽǶ�ֵ
short palstanceMpuPitch,	palstanceMpuYaw,	palstanceMpuRoll;//�����ǽ��ٶ�ֵ
float angleMpu[3][2];
float AngleMpuYaw_Start;
float Cloud_Angle_Measure_Yaw;
float Cloud_Palstance_Measure_Yaw;
fp32 Angle_error;




#if MODE==TEST /*-------------------------ң��������ģʽ����������-------------------------------*/
//ȥchassis_task.h���޸�״̬
void chassis_task(void *pvParameters)
{
//    //����һ��ʱ��
//    vTaskDelay(CHASSIS_TASK_INIT_TIME);
			for(;;)
			{
				if(SYSTEM_GetSystemState() == SYSTEM_STARTING)  //ϵͳ����
				{
						CHASSIS_REST();
						Chassis_Init();	
						Servo_Init();					
				}
				else
				{
					if (SYSTEM_GetRemoteMode() == RC) //ң�ص���ģʽ
					{		
						flow_led_on(0);		
						chassis_feedback_update(Cloud_Angle_Measure_Yaw);	//����ŷ�������ֵ������ת��
            Chassis_Set_Mode();
            Chassis_Set_Contorl();						
					}
				}
				Chassis_Omni_Move_Calculate();
				Chassis_MotorOutput();//����PID����
				CHASSIS_CANSend();  //Can��������
				vTaskDelay(TIME_STAMP_2MS);
			} 
}


#elif MODE==AVOI/*-------------------------���ϣ���������Ҫ��-------------------------------*/


void chassis_task(void *pvParameters)
{
//    //����һ��ʱ��
//    vTaskDelay(CHASSIS_TASK_INIT_TIME);
			for(;;)
			{
				if(SYSTEM_GetSystemState() == SYSTEM_STARTING)  //ϵͳ����
				{
								CHASSIS_REST();
                Chassis_Init();	
								sensor_update();
				}
				else  //��ʽ����·��
				{				
            sensor_update();//ѭ����ʼ��
						avoi();			
				}
			  Chassis_Omni_Move_Calculate();
				Chassis_MotorOutput();//����PID����
				CHASSIS_CANSend();  //Can��������
				vTaskDelay(TIME_STAMP_2MS);
			} 
}


#else /*-------------------------��ʽ����������ֻ�����-------------------------------*/
void chassis_task(void *pvParameters)
{
//    //����һ��ʱ��
//    vTaskDelay(CHASSIS_TASK_INIT_TIME);
			for(;;)
			{
				if(SYSTEM_GetSystemState() == SYSTEM_STARTING)  //ϵͳ����
				{
								CHASSIS_REST();
                Chassis_Init();	
					      Servo_Init();
								sensor_update();
				}
				else  //��ʽ����·��
				{				
            sensor_update();//ѭ����ʼ��
						judge();	
            Chassis_Set_Contorl();	
				}
			  Chassis_Omni_Move_Calculate();
				Chassis_MotorOutput();//����PID����
				CHASSIS_CANSend();  //Can��������
				vTaskDelay(TIME_STAMP_2MS);
			} 
}

#endif


		
//���̵�������
float Chassis_Final_Output[4];
//��������
float Servo_Output[3];


void Chassis_Init(void)
{
	flag.get = TRUE;
	flag.back = TRUE;
	flag.arm = TRUE;
	flag.open_flag = TRUE;
	flag.posi_go = TRUE;
/********************************************/
/**************PID����*************************/	
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
	const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};  //��תPID��������(����ת��Ƕ�)
	PID_Init(&chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
	vz_max_speed = NORMAL_MAX_CHASSIS_SPEED_Z;
	vz_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Z;
	chassis_feedback_update(Cloud_Angle_Measure_Yaw);	//����ŷ�������ֵ������ת��
}




void Chassis_Set_Mode(void)
{
	if(IF_RC_SW1_DOWN && IF_RC_SW2_UP)
	{
		ChassisMode = AUTO;  //ң����ģʽ
	}
	else	if(IF_RC_SW1_DOWN && IF_RC_SW2_MID)
	{
		ChassisMode = AUTO;  //ң����ģʽ
		//Raise();
	}
	else	if(IF_RC_SW1_DOWN && IF_RC_SW2_DOWN)
	{
		ChassisMode = AUTO;  //ң����ģʽ
		//Normal();
  }
	else if(IF_RC_SW1_MID && IF_RC_SW2_UP)  
	{
		ChassisMode = AUTO;  //ң����ģʽ				
	}
	else if(IF_RC_SW1_MID && IF_RC_SW2_MID)  
	{
		ChassisMode = REST;    //���ñ�־λ
    //Normal();		
	}
	else if(IF_RC_SW1_UP && IF_RC_SW2_UP)
	{
		judge();   //·��
	}	
  else
  { 
	  ChassisMode=Stop;
  }
}


/*-------------�ٶȻ�����--------------*/
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


/*-------------λ�û�����--------------*/
void POSI_REST(void)
{
	motor_speed(0,0,0,0);
	for (uint8_t j=0; j<4 ;j++)
	{
	  Angle_Target_Sum[j]   = Chassis_Angle_Measure[j];//λ�û�Ŀ��Ƕ�����
	  Angle_Measure_Sum[j]  = Chassis_Angle_Measure[j];//λ�û�ת���Ƕ�����
	  Angle_Measure_Prev[j] = Chassis_Angle_Measure[j];//�ϴ�λ������
		Chassis_Buff_Target_Sum[j] = Angle_Target_Sum[j];
	}
}


/*---------------��������ת���Գ���λ����Է���Ϊ������------------------*/
/*
  * @brief  ����������ת�䲿�֣���ͨ��debug���Գ�����Ҫ������ж�Ϊʲô��ôд��debug�԰�
  * @param  void
  * @retval void
  * @attention  degree��   90������    180������    270������      360������
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
	 if(degree < Cloud_Angle_Measure_Yaw)	//�ж��ĸ�����ת��ת��·�̶�
			Chassis_Move_Z = -PID_Calc(&chassis_angle_pid, 0.0f ,Chassis_Gyro_Error); //��ʱ�룬ͨ��PID����ó� Chassis_Move_Z
	 else
	 {
		 if( Angle_error > 180)
			 Chassis_Move_Z = PID_Calc(&chassis_angle_pid, 0.0f ,Chassis_Gyro_Error); //˳ʱ��
		 else
			 Chassis_Move_Z = -PID_Calc(&chassis_angle_pid, 0.0f ,Chassis_Gyro_Error); //��ʱ��
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

void Gyro_turn1(fp32 degree) //����ֱ�о���
{
	mode_change=TRUE;
	if(flag.turn == TRUE)
	{
		flag.open_flag = FALSE;
		chassis_feedback_update(degree);
	 lf = rf = lb = rb = 0;
	 if(degree < Cloud_Angle_Measure_Yaw)	//�ж��ĸ�����ת��ת��·�̶�
			Chassis_Move_Z = -PID_Calc(&chassis_angle_pid, 0.0f ,Chassis_Gyro_Error); //��ʱ�룬ͨ��PID����ó� Chassis_Move_Z
	 else
	 {
		 if( Angle_error > 180)
			 Chassis_Move_Z = PID_Calc(&chassis_angle_pid, 0.0f ,Chassis_Gyro_Error); //˳ʱ��
		 else
			 Chassis_Move_Z = -PID_Calc(&chassis_angle_pid, 0.0f ,Chassis_Gyro_Error); //��ʱ��
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




/*----------------------------------ͣһ���-----------------------------*/	
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




/*---------------------------------------ʲôģʽ����ô��-----------------------------------------*/
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
/*-------------------����-------------------------*/		
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
/*--------------------����------------------------*/
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
/*--------------------��ɫ����ת�򣨼�7��--------------------*/
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
		
/*--------------------��ɫ����ת�򣨼�8��--------------------*/
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
/*--------------------ת����--------------------*/
		
	  case east_89:
			Gyro_turn(89);
		break;
		
	  case east_91:
			Gyro_turn(91);
		break;
	
/*--------------------��е�۶���------------------------*/		
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
/*-----------------------д���������ֵ----------------------------*/
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
/*-----------------------��־λ�ֶ�����------------------------*/
		case Forward_flag:
			forward_flag();
		break; 
		
		case Arm_flag:
			arm_flag();
		break; 
		
		case Turn_flag:
			turn_flag();
		break; 
/*-----------------------����------------------------*/	
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
/*--------------------����1+����------------------------*/  		
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
/*--------------------����2+����------------------------*/	
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
			ChassisMode = low2_11;//����һ��ѭ��
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
/*--------------------��ɫ����ת�򣨼�8��--------------------*/
		case 731:
	    ChassisMode = GYRO_east; //��
      break;
		case 7311:
	    ChassisMode = GYRO_east88; //��
      break;
		case 732:
	    ChassisMode = GYRO_west; //��
      break;
	 case 733:
	    ChassisMode = GYRO_north; //��
      break;	
		case 734:
	    ChassisMode = GYRO_south; //��
      break;	
		
/*--------------------��ɫ����ת�򣨼�8��--------------------*/
		case 890:
			ChassisMode = BGYRO_east; //��
		target =90;
      break;
		case 892:
			ChassisMode = BGYRO_east2; //��
		target =92;
      break;
		case 8270:
	    ChassisMode = BGYRO_west; //��
		target=270;
      break;
		case 8180:
	    ChassisMode = BGYRO_north; //��
		target=180;
		  break;
 		case 8179:
	    ChassisMode = BGYRO_north1; //��
		target=179;     
		   break;
		case 8360:
	    ChassisMode = BGYRO_south; //��
		target=2;
      break;
		

		
/*--------------------ת����--------------------*/
		
		case 331:
	    ChassisMode = east_91; //��
      break;
		case 332:
	    ChassisMode = east_89; //��
      break;
		
		
/*--------------------��е��----------------------*/
		case 41:
			ChassisMode = paw_init; //��ʼ״̬
		  break;
		case 42:
			ChassisMode = paw_normal; //������ʻ״̬
		   break;
		case 43:
			ChassisMode = paw_raise; //����צ��
		   break;
		case 44:
			ChassisMode = booM; //��ը
		   break;
		case 45:
			ChassisMode = booM2; //��ը
		   break;
		case 40:
			ChassisMode = shou; //��������
		   break;
		case 401:
			ChassisMode = zhua; //ץ������
		   break;
		case 402:
			ChassisMode = tou; //͵һ����
		   break;

/*-----------------------ͣ����----------------------------*/
    case 51:
      ChassisMode = STOP1000;	
       break;
    case 52:
      ChassisMode = STOP2000;	
       break;
		case 53:
      ChassisMode = STOP3000;	
       break;
		
/*-----------------------��־λ�ֶ���������------------------------*/
    case 188:
      ChassisMode = Forward_flag;
       break;
    case 189:
      ChassisMode = Turn_flag;
       break;
    case 190:
      ChassisMode = Arm_flag;
       break;	
/*-----------------------����6+����------------------------*/	
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




/*-------��־λ���ú���(����ʵ������Ƿ�ʹ��,һ������·��������ִ��ͬһ��ģ��ĳ��򣬱����е��ץȡ��Ͷ��������·���м���Ҫ�򿪱�־λ)--------*/

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
/*------------------------------------------�˻�����ģʽ�����ڵ��Ժ���ˣ---------------------------------------------*/
void AUTO_mode(void)
{
	float k_rc_z=1;
	kRc_Mech_Chassis_Standard = 8.f;  //����ҡ��������,�����ٶ�
	kRc_Mech_Chassis_Revolve  = 5.4f; //���ڻ�еģʽҡ��Ťͷ������(̫С��Ӱ������ٶ�)
	Chassis_Move_Z = constrain_float( kRc_Mech_Chassis_Revolve*RC_CH0_RLR_OFFSET, -Chassis_Standard_Move_Max, Chassis_Standard_Move_Max);//��ת
		
	if(fabs(Chassis_Move_Z) > 800)//Ťͷ�ٶ�Խ��,ǰ���ٶ�Խ��,��ֹת��뾶����
		{
			k_rc_z = ( (Chassis_Revolve_Move_Max - fabs(Chassis_Move_Z) + 800) * (Chassis_Revolve_Move_Max - fabs(Chassis_Move_Z) + 800) )
						/ ( Chassis_Revolve_Move_Max * Chassis_Revolve_Move_Max );
			
			k_rc_z = constrain_float(k_rc_z,0,1);
		}
		else
		{
			k_rc_z = 1;
		}
		
		lf = constrain_float( kRc_Mech_Chassis_Standard*RC_CH3_LUD_OFFSET, -Chassis_Standard_Move_Max*k_rc_z, Chassis_Standard_Move_Max*k_rc_z);//ǰ��
    rf=lb=rb=lf;		
}


/*-------------------------------------------�㷨------------------------------------------------*/
/**
  * @brief  ����ȫ���㷨,��������ת��
  * @param  void
  * @retval void
  * @attention  (201/202/203/204 --> ��ǰ/��ǰ/���/�Һ�)
  *              	Xǰ(+)��(-)     Y��(-)��(+)     ZŤͷ
 */

void Chassis_Omni_Move_Calculate(void)
{	
	float speed_max;
  static fp32 k_z = 1;
	
	if(fabs(Chassis_Move_Z) > 800)//��ת�ٶ�Խ��,ǰ���ٶ�Խ��,��ֹ����ʱ�뾶����
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
  * @brief  �ֱ��4�������PID����,���������������(���͸������ֵ)
  * @param  void
  * @retval void
  * @attention  (201/202/203/204 --> ��ǰ/��ǰ/���/�Һ�)
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


/*---------------------------------------------�ٶȻ�PID------------------------------------------*/
/**
  * @brief  ���̵��PID����,����
  * @param  ���ID
  * @retval void
  * @attention  (201/202/203/204 --> ��ǰ/��ǰ/���/�Һ�)
  */
void Chassis_Motor_Speed_PID( ChassisWheel Wheel ) 
{

	//�����ٶ����
	Chassis_Speed_Error[Wheel] = Chassis_Speed_Target[Wheel] - Chassis_Speed_Measure[Wheel];
	//Chassis_Speed_Error[Wheel] = KalmanFilter(&Chassis_Speed_Kalman[Wheel], Chassis_Speed_Error[Wheel]);
	Chassis_Speed_Error_Sum[Wheel] += Chassis_Speed_Error[Wheel];
	
	pTermChassis[Wheel] =  Chassis_Speed_Error[Wheel]*Chassis_Speed_kpid[Wheel][KP];
	iTermChassis[Wheel] =  Chassis_Speed_Error_Sum[Wheel]*Chassis_Speed_kpid[Wheel][KI] * 0.002f;
	//�����޷�
	iTermChassis[Wheel] = constrain_float(iTermChassis[Wheel],-iTermChassis_Max,iTermChassis_Max);
	
	Chassis_Speed_Error_NOW[Wheel] = Chassis_Speed_Error[Wheel];
	dTermChassis[Wheel] = (Chassis_Speed_Error_NOW[Wheel] - Chassis_Speed_Error_LAST[Wheel])*Chassis_Speed_kpid[Wheel][KD];
	Chassis_Speed_Error_LAST[Wheel] = Chassis_Speed_Error_NOW[Wheel];
	
	//���������С,��ֹ���Ϊ0ʱͻȻʧ��
	if( pTermChassis[Wheel] * iTermChassis[Wheel] < 0 )
	{
		Chassis_Speed_Error_Sum[Wheel] = constrain_float(Chassis_Speed_Error_Sum[Wheel],
															-(3000/Chassis_Speed_kpid[Wheel][KI]/5.f),
															 (3000/Chassis_Speed_kpid[Wheel][KI]/5.f));
	}
	
	pidTermChassis[Wheel] = pTermChassis[Wheel] + iTermChassis[Wheel] + dTermChassis[Wheel];

	pidTermChassis[Wheel] = constrain_float(pidTermChassis[Wheel],-Chassis_Final_Output_Max,Chassis_Final_Output_Max);	
	
	//��¼�������
	Chassis_Final_Output[Wheel] = pidTermChassis[Wheel];
}



/*----------------------------------------------λ�û�PID--------------------------------------------*/
void Chassis_Motor_Angle_PID( ChassisWheel Wheel ) 
{
	
	//��ȡת�����ܽǶ�ֵ
	MotorAngleSum(LEFT_FRON_201);
	MotorAngleSum(RIGH_FRON_202);
	MotorAngleSum(LEFT_BACK_203);
	MotorAngleSum(RIGH_BACK_204);  
	//�⻷����
	Angle_Error[Wheel][OUTER] = Angle_Target_Sum[Wheel] - Chassis_Angle_Measure[Wheel];
	pTermAngle[Wheel][OUTER] = Angle_Error[Wheel][OUTER] * Chassis_Angle_kpid[Wheel][OUTER][KP];
	iTermAngle[Wheel][OUTER] = Angle_Error[Wheel][OUTER] * Chassis_Angle_kpid[Wheel][OUTER][KI];
	pidTermAngle[Wheel][OUTER] = pTermAngle[Wheel][OUTER] + iTermAngle[Wheel][OUTER];
	//�ڻ�����
	Angle_Error[Wheel][INNER]  =  pidTermAngle[Wheel][OUTER] - Chassis_Speed_Measure[Wheel];
	pTermAngle[Wheel][INNER]   = Angle_Error[Wheel][INNER] * Chassis_Speed_kpid[Wheel][KP];		
	iTermAngle[Wheel][INNER]  += Angle_Error[Wheel][INNER] * Chassis_Speed_kpid[Wheel][KI] * 0.001f;
	iTermAngle[Wheel][INNER]   = constrain_float( iTermAngle[Wheel][INNER], -iTermPosiMax, iTermPosiMax );

	Chassis_Final_Output[Wheel] = constrain_float( pTermAngle[Wheel][INNER] + iTermAngle[Wheel][INNER] , -6000, 6000);
}



void MotorAngleSum( ChassisWheel Wheel )
{		 
	//�ٽ�ֵ�жϷ�
	if (abs(Chassis_Angle_Measure[Wheel] - Angle_Measure_Prev[Wheel]) > 4095)//ת����Ȧ
	{		
		//���β����Ƕ�С���ϴβ����Ƕ��ҹ��˰�Ȧ,��˵�����ι������
		if (Chassis_Angle_Measure[Wheel] < Angle_Measure_Prev[Wheel])//����Ȧ�ҹ����
		{
			//�Ѿ�ת����һȦ,���ۼ�ת�� 8191(һȦ) - �ϴ� + ����
			Angle_Measure_Sum[Wheel] += 8191 - Angle_Measure_Prev[Wheel] + Chassis_Angle_Measure[Wheel];
		}
		else
		{
			//������һȦ
			Angle_Measure_Sum[Wheel] -= 8191 - Chassis_Angle_Measure[Wheel] + Angle_Measure_Prev[Wheel];
		}
	}
	else      
	{
		//δ���ٽ�ֵ,�ۼ���ת���ĽǶȲ�
		Angle_Measure_Sum[Wheel] += Chassis_Angle_Measure[Wheel] - Angle_Measure_Prev[Wheel];
	}

	//��¼��ʱ����Ƕ�,��һ�μ���ת���ǶȲ���,�����ж��Ƿ�ת��1Ȧ
	Angle_Measure_Prev[Wheel] = Chassis_Angle_Measure[Wheel];
}



/**
  * @brief  ���µ��̵������ �ٶ�  
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
  * @brief  ����YAWƫ�����ĽǶ�,���̸���ģʽ��
  * @param  void
  * @retval sAngleError,ƫ��Ƕ�ֵ,CAN�����Ļ�е�Ƕ�
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
		if(Angle_error > 180) //ʹ���յ����ľ���ֵС��180
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
  * @brief  ��ȡ����Ƕ�
  * @param  ID,CAN����
  * @retval void
  * @attention  (201/202/203/204 --> ��ǰ/��ǰ/���/�Һ�),CAN1�е���
  */
void CHASSIS_UpdateMotorAngle( ChassisWheel Wheel, int16_t angle )
{
    Chassis_Angle_Measure[Wheel] = angle;
}


/**
  * @brief  ��ȡ���ת��
  * @param  ID,CAN����
  * @retval void
  * @attention  (201/202/203/204 --> ��ǰ/��ǰ/���/�Һ�),CAN1�е���
  */
void CHASSIS_UpdateMotorSpeed( ChassisWheel Wheel, int16_t speed )
{
	Chassis_Speed_Measure[ Wheel ] = speed;
}


/**
  * @brief  ��ȡ���ת�ص���
  * @param  ID,CAN����
  * @retval void
  * @attention  (201/202/203/204 --> ��ǰ/��ǰ/���/�Һ�),CAN1�е���
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
	//��ȡ������  �Ƕ�   ���ٶ�   
	mpu_dmp_get_data( &angleMpuRoll, &angleMpuPitch, &angleMpuYaw );
	MPU_Get_Gyroscope( &palstanceMpuPitch, &palstanceMpuRoll, &palstanceMpuYaw );
	
	//�������ǽǶȷŴ�
	Cloud_Angle_Measure_Yaw = angleMpuYaw - AngleMpuYaw_Start + 180;
	
		//���ٶȸ���
  Cloud_Palstance_Measure_Yaw   = palstanceMpuYaw+PALST_COMPS_YAW;
		
}




/**
  * @brief  ���͵�����յ���ֵ
  * @param  void
  * @retval void
  * @attention  CAN1����
  */
void CHASSIS_CANSend(void)
{	 	
	CAN_CMD_CHASSIS(Chassis_Final_Output[0],Chassis_Final_Output[1],Chassis_Final_Output[2],Chassis_Final_Output[3]);
	
	//CAN_CMD_CHASSIS(1000,1000,1000,1000);
}







