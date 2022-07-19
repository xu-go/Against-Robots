#include "servo.h"
#include "motor_ctrl.h"
#include "chassis_task.h"
#include "stm32f4xx.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

extern int i;
extern Flag flag;
fp32 Servo_Real[5];
fp32 Servo_Target[5];
extern float Cloud_Angle_Measure_Yaw;

/*---------------------标志位开关(标志位使用规则：执行结束关闭自己的标志位，开启其他任务的标志位)---------------*/ 

//延时短，为了加快速度，用于无需延时的部分
void servo(int16_t s2,int16_t s3,int16_t s4,int16_t s5)//0°-180°   舵机
{
	chassis_feedback_update(Cloud_Angle_Measure_Yaw);
	static fp32 servo_time;
	S2=7.4*s2+500;
	S3=7.4*s3+500;
  S4=7.4*s4+500;
  S5=7.4*s5+500;
	if(Servo_Real[1] == Servo_Target[1] && Servo_Real[2] == Servo_Target[2] && Servo_Real[3] == Servo_Target[3] && Servo_Real[4] == Servo_Target[4])
	{
		servo_time++;
		if(servo_time > 100)
		{
			flag.get = TRUE;
			flag.back = TRUE;
			flag.turn = TRUE;
			flag.posi_go = TRUE;
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


//10 180
//180 10
//20  170
void Servo_Init(void) //出发时舵机状态
{
	S2=7.4*205+500;//左边舵机--增大往前转   205
	S3=7.4*10+500;//右边舵机--减小往前转  10
	S4=7.4*180+500;//前
	S5=7.4*160+500;//后
}


void Normal(void) //正常行驶舵机状态
	
{
	if(flag.arm == TRUE)
	{
		Servo_Target[1] = 80;
		Servo_Target[2] = 200;
		Servo_Target[3] = 180;
		Servo_Target[4] = 160;
		Servo_Ramp(servo2,5);
		Servo_Ramp(servo3,5);
		Servo_Ramp(servo4,5);
		Servo_Ramp(servo5,5);
		servo(Servo_Real[1],Servo_Real[2],Servo_Real[3],Servo_Real[4]);
	}
}


void Raise(void) //举起爪子
{
	if(flag.arm == TRUE)
	{
		Servo_Target[1] = 205;
		Servo_Target[2] = 10;
		Servo_Target[3] = 180;
		Servo_Target[4] = 160;
		Servo_Ramp(servo2,5);
		Servo_Ramp(servo3,10);
		Servo_Ramp(servo4,5);
		Servo_Ramp(servo5,5);
		servo(Servo_Real[1],Servo_Real[2],Servo_Real[3],Servo_Real[4]);
	}
}

void Raise11(void) //抓两个环
{
	if(flag.arm == TRUE)
	{
		Servo_Target[1] = 120;
		Servo_Target[2] = 140;
		Servo_Target[3] = 180;
		Servo_Target[4] = 160;
		Servo_Ramp(servo2,5);
		Servo_Ramp(servo3,10);
		Servo_Ramp(servo4,5);
		Servo_Ramp(servo5,5);
		servo(Servo_Real[1],Servo_Real[2],Servo_Real[3],Servo_Real[4]);
	}
}

void Raise22(void) //收两个环
{
	if(flag.arm == TRUE)
	{
		Servo_Target[1] = 138;//130
		Servo_Target[2] = 122;//130
		Servo_Target[3] = 180;
		Servo_Target[4] = 160;
		Servo_Ramp(servo2,5);
		Servo_Ramp(servo3,10);
		Servo_Ramp(servo4,5);
		Servo_Ramp(servo5,5);
		servo(Servo_Real[1],Servo_Real[2],Servo_Real[3],Servo_Real[4]);
	}
}

void Raise33(void) //偷个环
{
	if(flag.arm == TRUE)
	{
		Servo_Target[1] = 140;
		Servo_Target[2] = 120;
		Servo_Target[3] = 180;
		Servo_Target[4] = 160;
		Servo_Ramp(servo2,5);
		Servo_Ramp(servo3,10);
		Servo_Ramp(servo4,5);
		Servo_Ramp(servo5,5);
		servo(Servo_Real[1],Servo_Real[2],Servo_Real[3],Servo_Real[4]);
	}
}

void boom(void)
{
	if(flag.arm == TRUE)
	{
		Servo_Target[1] = 205;
		Servo_Target[2] = 10;
		Servo_Target[3] = 70;
		Servo_Target[4] = 50;
		Servo_Ramp(servo2,5);
		Servo_Ramp(servo3,10);
		Servo_Ramp(servo4,5);
		Servo_Ramp(servo5,5);
		servo(Servo_Real[1],Servo_Real[2],Servo_Real[3],Servo_Real[4]);
	}
}

void boom2(void)
{
	if(flag.arm == TRUE)
	{
		Servo_Target[1] = 130;
		Servo_Target[2] = 130;
		Servo_Target[3] = 70;
		Servo_Target[4] = 160;
		Servo_Ramp(servo2,5);
		Servo_Ramp(servo3,10);
		Servo_Ramp(servo4,5);
		Servo_Ramp(servo5,5);
		servo(Servo_Real[1],Servo_Real[2],Servo_Real[3],Servo_Real[4]);
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


