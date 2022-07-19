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
------��־λ����(��־λʹ�ù���ִ�н����ر��Լ��ı�־λ��������������ı�־λ)---------------*/ 
//��Ҫ��ʱ����ֹ��û��ִ����Ϳ�ʼ������һ��
void TIM5_Servo(int16_t s1,int16_t s2,int16_t s3)//0��-180��   ���
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


//��ʱ�̣�Ϊ�˼ӿ��ٶȣ�����������ʱ�Ĳ���
void servo(int16_t s1,int16_t s2,int16_t s3)//0��-180��   ���
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
void Servo_Init(void) //���������״̬
{
  S1=7.4*190+500; //�������190  ��40-�� 120 158
	S2=7.4*44+500; //�����ǰ44��-180��     
	S3=7.4*165+500;//��С���н�165  (-167                 
}
//200
void Nor(void) //��ʼ��״̬
{
	if(flag.arm == TRUE)
	{
		Servo_Target[0] = 190;//150Խ��ǰԽС
		Servo_Target[1] = 40;//40//20
		Servo_Target[2] = 100;//75
		Servo_Ramp(servo1,10);
		Servo_Ramp(servo2,10);
		Servo_Ramp(servo3,25);
		servo(Servo_Real[0],Servo_Real[1],Servo_Real[2]);
	}
}
//s1 ԽСԽ��ǰ��s2 Խ��Խ��ǰ
void Normal(void) //������ʻ���״̬(������)
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

void Normal2(void) //������ʻ״̬��������
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




                                 /*--------------------��е��ץ��----------------------*/
void Raise(void) //����צ�ӣ�ȥץ�ĵ�һ����
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

void Raise2(void) //����צ�ӣ�ȥץ�ĵڶ�����
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


void Get(void) //ץ����
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
                                /*--------------------��е�۷Ż�(������)----------------------*/		
//190 44 165
void Place1(void) //�Ż������������ĵ�һ��
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

void Place2(void) //�Ż������������ĵڶ���
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
void Place3(void) //�Ż�������������Ҫ���ݵڶ����޸�
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

void Place222(void) //�Ż������������Կ����
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

void Place333(void) //�Ż�������������Ҫ���ݵڶ����޸�duikang
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
 
                            /*--------------------��е�۷Ż�(������)----------------------*/
void Place11(void) //�Ż������������ĵ�һ��
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

void Place21(void) //�Ż������������ĵڶ���
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

void Place31(void) //�Ż�������������Ҫ���ݵڶ����޸�
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

void Place22(void) //�Ż������������ĵڶ���(������滷)
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




void Place32(void) //�Ż�������������Ҫ���ݵڶ����޸�
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

                            /*--------------------��е��ץ��(���滷)----------------------*/
//190 40 200
void Placec1(void) //ץ����һ��
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

void Placec2(void) //ץ���ڶ���
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

void Placec3(void) //ץ���������н�
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

void Placec4(void) //ȡ��
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
void Tou1(void) //����צ�ӣ�ȥץ�ĵ�һ����
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
void Tou2(void) //����צ�ӣ�ȥץ�ĵ�һ����
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
void Tou3(void) //����צ�ӣ�ȥץ�ĵ�һ����
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
void Tou4(void) //����צ�ӣ�ȥץ�ĵ�һ����
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
void Tou5(void) //����צ�ӣ�ȥץ�ĵ�һ����
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
void Tou6(void) //����צ�ӣ�ȥץ�ĵ�һ����
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
  * @brief  ������б�º���
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


