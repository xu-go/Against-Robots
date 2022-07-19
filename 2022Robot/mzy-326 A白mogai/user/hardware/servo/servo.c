#include "servo.h"
#include "motor_ctrl.h"
#include "chassis_task.h"
#include "stm32f4xx.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

extern int i;
extern Flag flag;
fp32 Servo_Real[3];
fp32 Servo_Target[3];
extern float Cloud_Angle_Measure_Yaw;

/*---------------
------标志位开关(标志位使用规则：执行结束关闭自己的标志位，开启其他任务的标志位)---------------*/ 
//需要延时，防止还没有执行完就开始进行下一步
void TIM5_Servo(int16_t s1,int16_t s2,int16_t s3)//0°-180°   舵机
{
	chassis_feedback_update(Cloud_Angle_Measure_Yaw);
	static fp32 servo_time;
	S1=7.4*s1+500;
	S2=7.4*s2+500;
	S3=7.4*s3+500;	
	if(Servo_Real[0] == Servo_Target[0] && Servo_Real[1] == Servo_Target[1] && Servo_Real[2] == Servo_Target[2])
	{
		servo_time++;
		if(servo_time > 100)
		{
			flag.get = TRUE;
			flag.posi_go = TRUE;
			flag.back = TRUE;
			flag.turn = TRUE;
			flag.arm = FALSE;
			servo_time = 0;
		  i++;
		}
	}
	else
	{
	  flag.get = FALSE;
	  flag.turn = FALSE;
	}
}


//延时短，为了加快速度，用于无需延时的部分
void servo(int16_t s1,int16_t s2,int16_t s3)//0°-180°   舵机
{
	chassis_feedback_update(Cloud_Angle_Measure_Yaw);
	static fp32 servo_time;
	S1=7.4*s1+500;
	S2=7.4*s2+500;
	S3=7.4*s3+500;	
	if(Servo_Real[0] == Servo_Target[0] && Servo_Real[1] == Servo_Target[1] && Servo_Real[2] == Servo_Target[2])
	{
		servo_time++;
		if(servo_time > 340)//400
		{
			flag.get = TRUE;
			flag.back = TRUE;
			flag.posi_go = TRUE;
			flag.turn = TRUE;
			flag.arm = FALSE;
			servo_time = 0;
		  i++;
		}
	}
	else
	{
	  flag.get = FALSE;
	  flag.turn = FALSE;
	}
}

#define GET_RING 100
//190 40 200
void Servo_Init(void) //出发区舵机状态
{
  S1=7.4*190+500; //变大，往下190  （40-） 120 158
	S2=7.4*44+500; //变大，往前44（-180）     
	S3=7.4*165+500;//变小，夹紧165  (-167                 
}
//200
void Nor(void) //初始化状态
{
	if(flag.arm == TRUE)
	{
		Servo_Target[0] = 190;//150越往前越小
		Servo_Target[1] = 40;//40//20
		Servo_Target[2] = 100;//75
		Servo_Ramp(servo1,10);
		Servo_Ramp(servo2,10);
		Servo_Ramp(servo3,25);
		servo(Servo_Real[0],Servo_Real[1],Servo_Real[2]);
	}
}
//s1 越小越往前，s2 越大越往前
void Normal(void) //正常行驶舵机状态(不带环)
{
	if(flag.arm == TRUE)
	{
		Servo_Target[0] = 190;//+105
		Servo_Target[1] = 40;//40//+30
		Servo_Target[2] = 150;
		Servo_Ramp(servo1,10);
		Servo_Ramp(servo2,10);
		Servo_Ramp(servo3,10);
		servo(Servo_Real[0],Servo_Real[1],Servo_Real[2]);
	}
}

void Normal2(void) //正常行驶状态（带环）
{
	if(flag.arm == TRUE)
	{
		Servo_Target[0] = 190;
		Servo_Target[1] = 40;//40
		Servo_Target[2] = 100;   //GET_RING;150
		Servo_Ramp(servo1,10);
		Servo_Ramp(servo2,10);
		Servo_Ramp(servo3,10);
		servo(Servo_Real[0],Servo_Real[1],Servo_Real[2]);
	}
}




                                 /*--------------------机械臂抓环----------------------*/
void Raise(void) //举起爪子（去抓的第一步）
{
	if(flag.arm == TRUE)
	{
		Servo_Target[0] = 190;//180
		Servo_Target[1] = 180;//80 210
		Servo_Target[2] = 170;//140
		Servo_Ramp(servo1,10);
		Servo_Ramp(servo2,10);
		Servo_Ramp(servo3,10);
		servo(Servo_Real[0],Servo_Real[1],Servo_Real[2]);
	}
}

void Raise2(void) //举起爪子（去抓的第二步）
{
	if(flag.arm == TRUE)
	{
		Servo_Target[0] = 40;//30
		Servo_Target[1] = 180;//190
		Servo_Target[2] = 170;//140
		Servo_Ramp(servo1,10);
		Servo_Ramp(servo2,10);
		Servo_Ramp(servo3,10);
		TIM5_Servo(Servo_Real[0],Servo_Real[1],Servo_Real[2]);
	}
}


void Get(void) //抓到环
{
	if(flag.arm == TRUE)
	{
		Servo_Target[0] = 43;//110
		Servo_Target[1] = 180;
		Servo_Target[2] = 100;    //GET_RING;140      7.15:1002
		Servo_Ramp(servo1,10);
		Servo_Ramp(servo2,10);
		Servo_Ramp(servo3,10);
		TIM5_Servo(Servo_Real[0],Servo_Real[1],Servo_Real[2]);
	}
}
                                /*--------------------机械臂放环(三倍区)----------------------*/		
//190 44 165
void Place1(void) //放环（三倍区）的第一步
{
	if(flag.arm == TRUE)
	{
		Servo_Target[0] = 200;//150
		Servo_Target[1] = 137;//132 135
		Servo_Target[2] = 100;
		Servo_Ramp(servo1,10);
		Servo_Ramp(servo2,10);
		Servo_Ramp(servo3,10);
		servo(Servo_Real[0],Servo_Real[1],Servo_Real[2]);
	}
}

void Place2(void) //放环（三倍区）的第二步
{
	if(flag.arm == TRUE)
	{
		Servo_Target[0] = 141;//33-100 140
		Servo_Target[1] = 137;//132 135
		Servo_Target[2] = 100;
		Servo_Ramp(servo1,10);
		Servo_Ramp(servo2,10);
		Servo_Ramp(servo3,10);
		servo(Servo_Real[0],Servo_Real[1],Servo_Real[2]);
	}
}
void Place3(void) //放环（三倍区）需要根据第二步修改
{
	if(flag.arm == TRUE)
	{
		Servo_Target[0] = 141;//33-100 140
		Servo_Target[1] = 137;//132 135
		Servo_Target[2] = 165;
		Servo_Ramp(servo1,10);
		Servo_Ramp(servo2,10);
		Servo_Ramp(servo3,30);
		servo(Servo_Real[0],Servo_Real[1],Servo_Real[2]);
	}
}

void Place222(void) //放环（三倍区）对抗侧放
{
	if(flag.arm == TRUE)
	{
		Servo_Target[0] = 188;//140
		Servo_Target[1] = 148;//137
		Servo_Target[2] = GET_RING;
		Servo_Ramp(servo1,10);
		Servo_Ramp(servo2,10);
		Servo_Ramp(servo3,10);
		servo(Servo_Real[0],Servo_Real[1],Servo_Real[2]);
	}
}

void Place333(void) //放环（三倍区）需要根据第二步修改duikang
{
	if(flag.arm == TRUE)
	{
		Servo_Target[0] = 188;//140
		Servo_Target[1] = 147;//137
		Servo_Target[2] = 165;//75
		Servo_Ramp(servo1,10);
		Servo_Ramp(servo2,10);
		Servo_Ramp(servo3,30);
		servo(Servo_Real[0],Servo_Real[1],Servo_Real[2]);
	}
}
 
                            /*--------------------机械臂放环(二倍区)----------------------*/
void Place11(void) //放环（二倍区）的第一步
{
	if(flag.arm == TRUE)
	{
		Servo_Target[0] = 130;//130
		Servo_Target[1] = 149;//147
		Servo_Target[2] = 110;
		Servo_Ramp(servo1,10);
		Servo_Ramp(servo2,10);
		Servo_Ramp(servo3,10);
		servo(Servo_Real[0],Servo_Real[1],Servo_Real[2]);
	}
}

void Place21(void) //放环（二倍区）的第二步
{
	if(flag.arm == TRUE)
	{
		Servo_Target[0] = 110;//110
		Servo_Target[1] = 149;//147
		Servo_Target[2] = 110;
		Servo_Ramp(servo1,10);
		Servo_Ramp(servo2,10);
		Servo_Ramp(servo3,10);
		servo(Servo_Real[0],Servo_Real[1],Servo_Real[2]);
	}
}

void Place31(void) //放环（二倍区）需要根据第二步修改
{
	if(flag.arm == TRUE)
	{
		Servo_Target[0] =110;//100
		Servo_Target[1] = 148;//150
		Servo_Target[2] =165;
		Servo_Ramp(servo1,10);
		Servo_Ramp(servo2,10);
		Servo_Ramp(servo3,10);
		servo(Servo_Real[0],Servo_Real[1],Servo_Real[2]);
	}
}

void Place22(void) //放环（二倍区）的第二步(针对增益环)
{
	if(flag.arm == TRUE)
	{
		Servo_Target[0] =90;
		Servo_Target[1] =159;
		Servo_Target[2] =75;
		Servo_Ramp(servo1,10);
		Servo_Ramp(servo2,10);
		Servo_Ramp(servo3,10);
		servo(Servo_Real[0],Servo_Real[1],Servo_Real[2]);
	}
}




void Place32(void) //放环（二倍区）需要根据第二步修改
{
	if(flag.arm == TRUE)
	{
		Servo_Target[0] =90;
		Servo_Target[1] = 159;
		Servo_Target[2] =75;
		Servo_Ramp(servo1,10);
		Servo_Ramp(servo2,10);
		Servo_Ramp(servo3,10);
		servo(Servo_Real[0],Servo_Real[1],Servo_Real[2]);
	}
}

                            /*--------------------机械臂抓环(增益环)----------------------*/
//190 40 200
void Placec1(void) //抓环第一步
{
	if(flag.arm == TRUE)
	{//190 40 150
		Servo_Target[0] = 120;//220 50 140
		Servo_Target[1] = 158;//160
		Servo_Target[2] = 170;//180
		Servo_Ramp(servo1,10);
		Servo_Ramp(servo2,10);
		Servo_Ramp(servo3,10);
		servo(Servo_Real[0],Servo_Real[1],Servo_Real[2]);
	}
}

void Placec2(void) //抓环第二步
{
	if(flag.arm == TRUE)
	{
		Servo_Target[0] = 120;//68-115
		Servo_Target[1] = 158;//160
		Servo_Target[2] = 110;
		Servo_Ramp(servo1,10);
		Servo_Ramp(servo2,10);
		Servo_Ramp(servo3,10);
		servo(Servo_Real[0],Servo_Real[1],Servo_Real[2]);
	}
}

void Placec3(void) //抓环第三步夹紧
{
	if(flag.arm == TRUE)
	{
		Servo_Target[0] = 215;
		Servo_Target[1] = 140;
		Servo_Target[2] = 110;
		Servo_Ramp(servo1,10);
		Servo_Ramp(servo2,10);
		Servo_Ramp(servo3,10);
		servo(Servo_Real[0],Servo_Real[1],Servo_Real[2]);
	}
}

void Placec4(void) //取环
{
	if(flag.arm == TRUE)
	{
		Servo_Target[0] = 190;
		Servo_Target[1] = 40;
		Servo_Target[2] = 130;
		Servo_Ramp(servo1,10);
		Servo_Ramp(servo2,10);
		Servo_Ramp(servo3,10);
		servo(Servo_Real[0],Servo_Real[1],Servo_Real[2]);
	}
}
void Tou1(void) //举起爪子（去抓的第一步）
{
	if(flag.arm == TRUE)
	{//190 44 165
		Servo_Target[0] = 90;//220 50 140
		Servo_Target[1] = 160;//160
		Servo_Target[2] = 140;//180		170
		Servo_Ramp(servo1,10);
		Servo_Ramp(servo2,10);
		Servo_Ramp(servo3,10);
		servo(Servo_Real[0],Servo_Real[1],Servo_Real[2]);
	}
}
void Tou2(void) //举起爪子（去抓的第一步）
{
	if(flag.arm == TRUE)
	{
		Servo_Target[0] = 90;//180	
		Servo_Target[1] = 170;//80 210
		Servo_Target[2] = 110;//140 110
		Servo_Ramp(servo1,10);
		Servo_Ramp(servo2,10);
		Servo_Ramp(servo3,10);
		servo(Servo_Real[0],Servo_Real[1],Servo_Real[2]);
	}
}
void Tou3(void) //举起爪子（去抓的第一步）
{
	if(flag.arm == TRUE)
	{
		Servo_Target[0] = 210;//180
		Servo_Target[1] = 160;//80 210
		Servo_Target[2] = 110;//140
		Servo_Ramp(servo1,10);
		Servo_Ramp(servo2,10);
		Servo_Ramp(servo3,10);
		servo(Servo_Real[0],Servo_Real[1],Servo_Real[2]);
	}
}
void Tou4(void) //举起爪子（去抓的第一步）
{
	if(flag.arm == TRUE)
	{//190 40 150
		Servo_Target[0] = 120;//220 50 140
		Servo_Target[1] = 170;//160 178
		Servo_Target[2] = 140;//180
		Servo_Ramp(servo1,10);
		Servo_Ramp(servo2,10);
		Servo_Ramp(servo3,10);
		servo(Servo_Real[0],Servo_Real[1],Servo_Real[2]);
	}
}
void Tou5(void) //举起爪子（去抓的第一步）
{
	if(flag.arm == TRUE)
	{
		Servo_Target[0] = 120;//180
		Servo_Target[1] = 170;//80 210
		Servo_Target[2] = 140;//140
		Servo_Ramp(servo1,10);
		Servo_Ramp(servo2,10);
		Servo_Ramp(servo3,10);
		servo(Servo_Real[0],Servo_Real[1],Servo_Real[2]);
	}
}
void Tou6(void) //举起爪子（去抓的第一步）
{
	if(flag.arm == TRUE)
	{
		Servo_Target[0] = 210;//180
		Servo_Target[1] = 150;//80 210
		Servo_Target[2] = 110;//140
		Servo_Ramp(servo1,10);
		Servo_Ramp(servo2,10);
		Servo_Ramp(servo3,10);
		servo(Servo_Real[0],Servo_Real[1],Servo_Real[2]);
	}
}
/**
  * @brief  舵机输出斜坡函数
  * @param  void
  * @retval void
  * @attention 
  */
void Servo_Ramp(Servo servo ,float x)
{
	if (Servo_Real[servo] < Servo_Target[servo])
	{
		Servo_Real[servo] += x;
		if(Servo_Real[servo] > Servo_Target[servo])
		{
			Servo_Real[servo] = Servo_Target[servo];
		}
	}
	else if (Servo_Real[servo] > Servo_Target[servo])
	{
		Servo_Real[servo] -= x;
	}
	
	if (Servo_Real[servo] < 0)
	{
		Servo_Real[servo] = 0;
	}
}


