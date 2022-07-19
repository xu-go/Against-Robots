#include "motor_ctrl.h"
#include "chassis_task.h"
#include "math.h"
#include "stdio.h"
#include <stdlib.h>
#include "mpu6050.h"
#include "inv_mpu.h"


#define high_speed 8500//9000 13000
#define mid_speed 6000
#define low_speed 6000
 
float lf,rf,lb,rb;//�ĸ����ӵ��ٶ�
extern int target;
extern int i;
float num;
extern float Cloud_Angle_Measure_Yaw;
extern bool mode_change;
Flag flag;
extern float Chassis_Move_Z;//������ת
/*----------------------------------------------*ѭ��*---------------------------------------------------------*/

float f1,f2,f3,f4,f5,f6,f7,f8,b1,b2,b3,b4,b5,b6,b7,b8,l1,l2,l3,l4,l5,r1,r2,r3,r4,r5;
float in;//����

/********************0ʶ�𵽣�1û��ʶ��************************/
#define left_1  ((f5==0&&f6==0&&f7==1)||(f5==1&&f6==0&&f7==1))//11110011����11111011  ����ƫ��̶�С
#define left_2  ((f6==0&&f7==0&&f8==1)||(f6==1&&f7==0&&f8==1))//11111001����11111101  ����ƫ��̶���||(f5==0&&f6==0&&f7==0)
#define left_3  ((f6==1&&f7==0&&f8==0)||(f6==1&&f7==1&&f8==0))//11111100����11111110  ����ƫ��̶ȴ�||(f6==0&&f7==0&&f8==0)
#define right_1 ((f2==1&&f3==0&&f4==0)||(f2==1&&f3==0&&f4==1))//11001111����11011111  ����ƫ�ҳ̶�С
#define right_2 ((f2==0&&f3==0&&f4==1)||(f2==0&&f3==1&&f4==1))//10011111����10111111  ����ƫ�ҳ̶���||(f2==0&&f3==0&&f4==0)
#define right_3 ((f1==0&&f2==0&&f3==1)||(f1==0&&f2==1&&f3==1))//00111111����01111111  ����ƫ�ҳ̶ȴ�||(f1==0&&f2==0&&f3==0)
#define middle  ((f1==0&&f2==0&&f3==0&&f4==0&&f5==0&&f6==0&&f7==0&&f8==0)||(f1==1&&f2==1&&f3==1&&f4==1&&f5==1&&f6==1&&f7==1&&f8==1)||(f3==1&&f4==0&&f5==1)||(f4==1&&f5==0&&f6==1)||(f3==1&&f4==0&&f5==0&&f6==1)||(f2==1&&f3==0&&f4==0&&f5==0&&f6==0&&f7==1))
//00000000����11111111����11101111����11110111����11100111  ����Ƚ���



/********************0ʶ�𵽣�1û��ʶ��************************/
 void sensor_update(void)
{
	f1=GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_2);//L1
  f2=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0);//L2
	f3=GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_3);//M1
	f4=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1);//M2
	f5=GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4);//N1
	f6=GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0);//N2
	f7=GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_5);//O1
	f8=GPIO_ReadInputDataBit(GPIOI,GPIO_Pin_2);//Z
		
	l1=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0);//S
	l2=GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_12);//H	
	l3=GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_13);//G
	l4=GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_14);//F
	l5=GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_15);//E		
	
	r1=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1);//T
	r2=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_2);//U
	r3=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3);//V
	r4=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8);//���PWM��
	r5=GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_14);//�ұ�PWM��
}



/*
***ǰ�� ����
***��һ������ȷ������һ���ٶ� LOW��HIGH
***�ڶ�������д�ܼ���
***�����������ж��Ƿ������٣���LOW����ֱ�ߣ���IFдΪ0����HIGH���ܣ���IF��ֵΪ1���������һ�������
*/

void Forward(Speed speed,float NUM,float IF) 
{
	chassis_feedback_update(Cloud_Angle_Measure_Yaw);
/*------------------------��־λ����(��־λʹ�ù���ִ�н����ر��Լ��ı�־λ��������������ı�־λ)---------------*/  
	mode_change=FALSE;
	if(flag.get == TRUE)  //��־λ��TRUEʱ��ʾ����ֱ��
	{
		flag.open_flag = FALSE; 		
		num=NUM;
		flag.get = FALSE;
    flag.senor = FALSE;		//��ֹ��ʮ��·��������num��һ
		if(speed == LOW) 
		{
			Low();
		}		
		else if(speed == HIGH)//l3����ʮ��·����speed=highʱ������
		{
			High();
		}
	}
	if(IF == 1) 
	{
		flag.mid = TRUE;
	}
	else if(IF == 0)
	{
		flag.mid = FALSE;
	}
/*------------------------------------------------------*/
	if(num>1)
	{
		if(l3==1 && speed == LOW) //l3����ʮ��·����speed=lowʱ������
		{
			Low();
			flag.senor = TRUE;
		}		
		else if(l3==1 && speed == HIGH)//l3����ʮ��·����speed=highʱ������
		{
			High();
			flag.senor = TRUE; //��־λΪ1	
		}
		if(l3==0 && flag.senor == TRUE)//l3��ʮ��·����ֱ�б�־λΪ1ʱ��num��1
		{
			num--;
			flag.senor = FALSE; //ֱ�б�־λΪ0����ʱnum���������ֹһ�����߼����
	  }
  }
	else if(num==1)
	{
		if(flag.mid == TRUE)
		{
		  Mid();  //IF=1���һ����MID����
		}
		else if(flag.mid == FALSE)
		{
			Low();  //IF=0���һ����LOW����
		}
		if((l1==0||l2==0)&& l3 == 1) //���һ��ѭ��Ѱ�����ߣ���ʱ��LOW����
		{
			Low();
			flag.senor = TRUE;
			flag.mid = FALSE;
		}	
		else if(l2==0 && l3 == 0)
		{
			Low();		
		}
		else if(l4==0 && l5 == 1 && flag.senor == TRUE) 
		{
			num--;
			stop();
		}
	}
	else if(num==0)
	{
		if(fabs(target-Cloud_Angle_Measure_Yaw)>350)
		{
		  target=0;
			if(Cloud_Angle_Measure_Yaw>350)
				target=360;
		}
		else if(fabs(target-Cloud_Angle_Measure_Yaw)>1.801)
			{
			  Gyro_turn1(target);
			}
		else if((fabs(target-Cloud_Angle_Measure_Yaw)<=1.8)&&(l2==0||l4==0||l3==0||l5==0||r2==0||r3==0))
		{
			stop_delay(150);
		}
		//stop_delay(100);
		flag.get = FALSE; //�ر�ֱ�б�־λ
		flag.arm = TRUE; //������е�� 
		flag.turn = TRUE; //����ת��
		mode_change=TRUE; 
		flag.posi_go = TRUE;
		flag.open_flag = TRUE;
		flag.back = TRUE;
	}
}

void low_Forward0(void) 
{
	chassis_feedback_update(Cloud_Angle_Measure_Yaw);
/*------------------------��־λ����(��־λʹ�ù���ִ�н����ر��Լ��ı�־λ��������������ı�־λ)---------------*/  
	mode_change=FALSE;
	if(flag.get == TRUE)  //��־λ��TRUEʱ��ʾ����ֱ��
	{
		flag.open_flag = FALSE; 		
		num=1;
		flag.get = FALSE;
    flag.senor = FALSE;		//��ֹ��ʮ��·��������num��һ
		Low();
	}
  if(num==1)
	{
		if(f3 == 0 && f4 == 0 && f5 == 0 && f6 == 0) //ǰ��ѭ��Ѱ�����ߣ���ʱ��LOW����
		{
			Low2();
			flag.senor = TRUE;
			flag.mid = FALSE;
		}	
		if(l4 == 0 && l3 == 1 && flag.senor == TRUE) 
		{
			num--;
			stop();
		}
	}
	else if(num==0)
	{
		if(fabs(target-Cloud_Angle_Measure_Yaw)>350)
		{
		  target=0;
			if(Cloud_Angle_Measure_Yaw>350)
				target=360;
		}
		else if(fabs(target-Cloud_Angle_Measure_Yaw)>1.801)
			{
			  Gyro_turn1(target);
			}
		else if((fabs(target-Cloud_Angle_Measure_Yaw)<=1.8)&&(l2==0||l4==0||l3==0||l5==0||r2==0||r3==0))
		{
			stop_delay(150);
		}
		//stop_delay(100);
		flag.get = FALSE; //�ر�ֱ�б�־λ
		flag.arm = TRUE; //������е��
		flag.turn = TRUE; //����ת��
		mode_change=TRUE; 
		flag.posi_go = TRUE;
		flag.open_flag = TRUE;
		flag.back = TRUE;
	}
}

//����һ��
void low_Forward(void) 
{
	chassis_feedback_update(Cloud_Angle_Measure_Yaw);
/*------------------------��־λ����(��־λʹ�ù���ִ�н����ر��Լ��ı�־λ��������������ı�־λ)---------------*/  
	mode_change=FALSE;
	if(flag.get == TRUE)  //��־λ��TRUEʱ��ʾ����ֱ��
	{
		flag.open_flag = FALSE; 		
		num=1;
		flag.get = FALSE;
    flag.senor = FALSE;		//��ֹ��ʮ��·��������num��һ
		Low();
	}
  if(num==1)
	{
		if(f3 == 0 && f4 == 0 && f5 == 0 && f6 == 0) //ǰ��ѭ��Ѱ�����ߣ���ʱ��LOW����
		{
			Low2();
			flag.senor = TRUE;
			flag.mid = FALSE;
		}	
		if(l3 == 0 && l2 == 1 && flag.senor == TRUE) 
		{
			num--;
			stop();
		}
	}
	else if(num==0)
	{
		if(fabs(target-Cloud_Angle_Measure_Yaw)>350)
		{
		  target=0;
			if(Cloud_Angle_Measure_Yaw>350)
				target=360;
		}
		else if(fabs(target-Cloud_Angle_Measure_Yaw)>1.801)
			{
			  Gyro_turn1(target);
			}
		else if((fabs(target-Cloud_Angle_Measure_Yaw)<=1.8)&&(l2==0||l4==0||l3==0||l5==0||r2==0||r3==0))
		{
			stop_delay(150);
		}
		//stop_delay(100);
		flag.get = FALSE; //�ر�ֱ�б�־λ
		flag.arm = TRUE; //������е��
		flag.turn = TRUE; //����ת��
		mode_change=TRUE; 
		flag.posi_go = TRUE;
		flag.open_flag = TRUE;
		flag.back = TRUE;
	}
}

void low_Forward1(void) //����
{
	chassis_feedback_update(Cloud_Angle_Measure_Yaw);
/*------------------------��־λ����(��־λʹ�ù���ִ�н����ر��Լ��ı�־λ��������������ı�־λ)---------------*/  
	mode_change=FALSE;
	if(flag.get == TRUE)  //��־λ��TRUEʱ��ʾ����ֱ��
	{
		flag.open_flag = FALSE; 		
		num=1;
		flag.get = FALSE;
    flag.senor = FALSE;		//��ֹ��ʮ��·��������num��һ
		Low();
	}
  if(num==1)
	{
			if(f3 == 0 && f4 == 0 && f5 == 0 && f6 == 0) //ǰ��ѭ��Ѱ�����ߣ���ʱ��LOW����
		{
			Low2();
			flag.senor = TRUE;
			flag.mid = FALSE;
		}	
		if(l2 == 0 && l1 == 1 && flag.senor == TRUE) 
		{
			num--;
			stop();
		}
	}
	else if(num==0)
	{
		stop_delay(100);
		flag.get = FALSE; //�ر�ֱ�б�־λ
		flag.arm = TRUE; //������е��
		flag.turn = TRUE; //����ת��
		mode_change=TRUE; 
		flag.posi_go = TRUE;
		flag.open_flag = TRUE;
		flag.back = TRUE;
	}
}

void low_Forward3(void) 
{
	chassis_feedback_update(Cloud_Angle_Measure_Yaw);
/*------------------------��־λ����(��־λʹ�ù���ִ�н����ر��Լ��ı�־λ��������������ı�־λ)---------------*/  
	mode_change=FALSE;
	if(flag.get == TRUE)  //��־λ��TRUEʱ��ʾ����ֱ��
	{
		flag.open_flag = FALSE; 		
		num=1;
		flag.get = FALSE;
    flag.senor = FALSE;		//��ֹ��ʮ��·��������num��һ
		Low();
	}
  if(num==1)
	{
		if(f3 == 0 && f4 == 0 && f5 == 0 && f6 == 0) //ǰ��ѭ��Ѱ�����ߣ���ʱ��LOW����
		{
			Low2();
			flag.senor = TRUE;
			flag.mid = FALSE;
		}	
		if(l2 == 0 && l3 == 1 && flag.senor == TRUE) 
		{
			num--;
			stop();
		}
	}
	else if(num==0)
	{
		if(fabs(target-Cloud_Angle_Measure_Yaw)>350)
		{
		  target=0;
			if(Cloud_Angle_Measure_Yaw>350)
				target=360;
		}
		else if(fabs(target-Cloud_Angle_Measure_Yaw)>1.801)
			{
			  Gyro_turn1(target);
			}
		else if((fabs(target-Cloud_Angle_Measure_Yaw)<=1.8)&&(l2==0||l4==0||l3==0||l5==0||r2==0||r3==0))
		{
			stop_delay(150);
		}
		//stop_delay(100);
		flag.get = FALSE; //�ر�ֱ�б�־λ
		flag.arm = TRUE; //������е��
		flag.turn = TRUE; //����ת��
		mode_change=TRUE; 
		flag.posi_go = TRUE;
		flag.open_flag = TRUE;
		flag.back = TRUE;
	}
}


//���ڳ���
void OUT(void) 
{
	chassis_feedback_update(Cloud_Angle_Measure_Yaw);
/*------------------------��־λ����(��־λʹ�ù���ִ�н����ر��Լ��ı�־λ��������������ı�־λ)---------------*/  
	mode_change=FALSE;
	if(flag.get == TRUE)  //��־λ��TRUEʱ��ʾ����ֱ��
	{
		flag.open_flag = FALSE; 		
		num=1;
		flag.get = FALSE;
    flag.senor = FALSE;		//��ֹ��ʮ��·��������num��һ
		Low();
	}
  if(num==1)
	{
		if((l1==0 || l2 ==0) && l3 == 1) //���һ��ѭ��Ѱ�����ߣ���ʱ��LOW����
		{
			Low();
			flag.senor = TRUE;
			flag.mid = FALSE;
		}	
		if(l5==0 && l4==1 && flag.senor == TRUE) 
		{
			num--;
			stop();
		}
	}
	else if(num==0)
	{
		stop_delay(0);
		flag.get = FALSE; //�ر�ֱ�б�־λ
		flag.arm = TRUE; //������е��
		flag.turn = TRUE; //����ת��
		mode_change=TRUE; 
		flag.posi_go = TRUE;
		flag.open_flag = TRUE;
		flag.back = TRUE;
	}
}


//ǰ��������ڷ�ש
void Half_Forward(void) 
{
	chassis_feedback_update(Cloud_Angle_Measure_Yaw);
/*------------------------��־λ����(��־λʹ�ù���ִ�н����ر��Լ��ı�־λ��������������ı�־λ)---------------*/  
	mode_change=FALSE;
	if(flag.get == TRUE)  //��־λ��TRUEʱ��ʾ����ֱ��
	{
		flag.open_flag = FALSE; 		
		num=1;
		flag.get = FALSE;
    flag.senor = FALSE;		//��ֹ��ʮ��·��������num��һ
		Low();
	}
    if(num==1)
	{
		if(f3 == 0 && f4 == 0 && f5==0) //ǰ��ѭ��Ѱ�����ߣ���ʱ��LOW����
		{
			Low2();
			flag.senor = TRUE;
			flag.mid = FALSE;
		}	
		if(l4==0 && l5 == 0 && flag.senor == TRUE) 
		{
			num--;
			stop();
		}
	}
	else if(num==0)
	{
		stop_delay(100);
		flag.get = FALSE; //�ر�ֱ�б�־λ
		flag.arm = TRUE; //������е��
		flag.turn = TRUE; //����ת��
		mode_change=TRUE; 
		flag.posi_go = TRUE;
		flag.open_flag = TRUE;
		flag.back = TRUE;
	}
}


void Back(float NUM) 
{
	chassis_feedback_update(Cloud_Angle_Measure_Yaw);
/*------------------------��־λ����(��־λʹ�ù���ִ�н����ر��Լ��ı�־λ��������������ı�־λ)---------------*/  
	mode_change=FALSE;
	if(flag.back == TRUE)  //��־λ��TRUEʱ��ʾ����ֱ��
	{
		back();
		flag.open_flag = FALSE; 		
		num=NUM;
		flag.back = FALSE;
		flag.senor = FALSE;
	}
/*------------------------------------------------------*/
	if(num>=1)
	{
		if(l3==1) //l3����ʮ��·����speed=lowʱ������
		{
			back();
			flag.senor = TRUE;
		}		
		if(l3==0 && flag.senor == TRUE)//l3��ʮ��·����ֱ�б�־λΪ1ʱ��num��1
		{
			num--;
			flag.senor = FALSE; //ֱ�б�־λΪ0����ʱnum���������ֹһ�����߼����
	  }
  }
	else if(num==0)
	{
		stop_delay(100);
		flag.arm = TRUE;
		flag.turn = TRUE;
		mode_change=TRUE;
		flag.posi_go = TRUE;
		flag.get = FALSE;
		flag.open_flag = TRUE;
	}
}

void Back11(float NUM) 
{
	chassis_feedback_update(Cloud_Angle_Measure_Yaw);
/*------------------------��־λ����(��־λʹ�ù���ִ�н����ر��Լ��ı�־λ��������������ı�־λ)---------------*/  
	mode_change=FALSE;
	if(flag.back == TRUE)  //��־λ��TRUEʱ��ʾ����ֱ��
	{
		back();
		flag.open_flag = FALSE; 		
		num=NUM;
		flag.back = FALSE;
		flag.senor = FALSE;
	}
/*------------------------------------------------------*/
	if(num>=1)
	{
		if(l4==1) //l3����ʮ��·����speed=lowʱ������
		{
			back();
			flag.senor = TRUE;
		}		
		if(l4==0 && flag.senor == TRUE)//l3��ʮ��·����ֱ�б�־λΪ1ʱ��num��1
		{
			num--;
			flag.senor = FALSE; //ֱ�б�־λΪ0����ʱnum���������ֹһ�����߼����
	  }
  }
	else if(num==0)
	{
		stop_delay(100);
		flag.arm = TRUE;
		flag.turn = TRUE;
		mode_change=TRUE;
		flag.posi_go = TRUE;
		flag.get = FALSE;
		flag.open_flag = TRUE;
	}
}


void stop_delay(fp32 time) //timeд���پ�ͣ����
{
	static fp32 delay_time;
	stop();
	delay_time++;
	if(delay_time > time)
	{
		delay_time = 0;
		flag.delay = FALSE;
		i++;
	}
}


void stop_forward(fp32 time) //д���پ�ͣ��ã����沿���Ƿ�ֹת���������ƫ�󣬷�ֹ��ѭһ��
{
	static fp32 delay_time;
	delay_time++;
	if(delay_time > time)
	{
		if(l1 == 0 || l2 == 0 || l3 == 0 || l4 == 0 || l5 == 0)
		{
			if(l3 == 0)
			{
				stop();
				delay_time = 0;
				flag.delay = FALSE;
				i++;
			}
			else if(l3 == 1)
			{
				Low2();				
			}
		}
		else 
		{
			delay_time = 0;
			flag.delay = FALSE;
			i++;
		}
	}
}



/*------------------------------------------�Ӻ���------------------------------------------------*/
/**
  * @brief  �����ٶ�
  * @param  void
  * @retval void
  * @attention  
  */

void motor_speed(float left_front,float right_front,float left_behind,float right_behind)
{
	lf=left_front;
	rf=right_front;
	lb=left_behind;
	rb=right_behind;
}

void left(float left_front,float right_front,float left_behind,float right_behind)
{
	lf=-left_front;
	rf=right_front;
	lb=-left_behind;
	rb=right_behind;
}

void right(float left_front,float right_front,float left_behind,float right_behind)
{
	lf=left_front;
	rf=-right_front;
	lb=left_behind;
	rb=-right_behind;
}



void stop(void)
{
  motor_speed(0,0,0,0);
	Chassis_Move_Z = 0;
}



void go_high(fp32 z)  //num�Ӷ����ٲ���Ƕ���
{
  motor_speed(high_speed,high_speed,high_speed,high_speed);
	Chassis_Move_Z = z;
}


void go_mid(fp32 z)
{
  motor_speed(mid_speed,mid_speed,mid_speed,mid_speed);
	Chassis_Move_Z = z;
}



void go_low(fp32 z)
{
  motor_speed(low_speed,low_speed,low_speed,low_speed);
	Chassis_Move_Z = z;
}

void go_low2(fp32 z)
{
  motor_speed(low_speed-3000,low_speed-3000,low_speed-3000,low_speed-3000);//-3000-3000
	Chassis_Move_Z = z;
}
void go_low2right1(fp32 z)
{
  motor_speed(low_speed-2000,low_speed-3000,low_speed-2000,low_speed-3000);//-3000-3000
	Chassis_Move_Z = z;
}



void go_back(fp32 z)
{
	motor_speed(-low_speed,-low_speed,-low_speed,-low_speed);
	Chassis_Move_Z = -z;
}

/*----------------------------------����----------------------------------*/
void Low(void) //����
{
  if(middle)
		go_low(30);

	else if(left_1) 
		go_low(300); //ƫ���һ����תֵ,ֵԽ���������Խ��200
	
	else if(left_2)//300
		go_low(300);
	
	else if(left_3) 
		go_low(300);//900
	
	else if(right_1) 
		go_low(-300); //ƫ�Ҽ�һ����תֵ(-300)

	else if(right_2) 
		go_low(-300);

	else if(right_3) 
		go_low(-300);
	else 
		go_low(0);
}

void Low2(void) //����
{
  if(middle)
		go_low2(10);
	else if(left_1) 
		go_low2(180); //ƫ���һ����תֵ,ֵԽ���������Խ��20��300
	else if(left_2)
		go_low2(180);//60
	else if(left_3) 
		go_low2(90);
	else if(right_1) 
		go_low2(-50); //ƫ�Ҽ�һ����תֵ(-20)
	else if(right_2) 
		go_low2(-80);
	else if(right_3) 
		go_low2(-90);
	else 
		go_low2(0);
}


void Mid(void) //����
{
	if(middle)
		go_mid(0);
	
	else if(left_1) 
		go_mid(350); //ƫ���һ����תֵ,ֵԽ���������Խ��300
	
	else if(left_2)
		go_mid(400);
	
	else if(left_3) 
		go_mid(600);
	
	else if(right_1) 
		go_mid(-300); //ƫ�Ҽ�һ����תֵ
	
	else if(right_2) 
		go_mid(-400);
	
	else if(right_3) 
		go_mid(-600);
	
	else 
		go_mid(0);
}


void High(void)//����
{
	if(middle)
		go_high(0);
	
	else if(left_1) 
		go_high(200);//ƫ���һ����תֵ,ֵԽ���������Խ��//4007001100
	
	else if(left_2) 
		go_high(300);

	else if(left_3) 
		go_high(400);
	
	else if(right_1) 
		go_high(-200);//ƫ�Ҽ�һ����תֵ,ֵԽ���������Խ��
	
	else if(right_2) 
		go_high(-300);
	
	else if(right_3) 
		go_high(-400);
	
	else 
		go_high(0);
}


void back(void)
{
	if(middle)
		go_back(0);
	
	else if(left_1) 
		go_back(200);//ƫ���һ����תֵ,ֵԽ���������Խ��
	
	else if(left_2) 
		go_back(300);

	else if(left_3) 
		go_back(500);
	
	else if(right_1) 
		go_back(-200);//ƫ�Ҽ�һ����תֵ,ֵԽ���������Խ��
	
	else if(right_2) 
		go_back(-300);
	
	else if(right_3) 
		go_back(-500);
	
	else
		go_back(0);
}






/*-----------------------------------------������Ҫ�������ò���������Ҫ��-----------------------------------------------*/
void turn(void)
{
	motor_speed(-low_speed-1000,low_speed+1000,-low_speed-1000,low_speed+1000);
}



void avoi(void)
{
	if(in == 1)
	{
		stop_delay(100);
		go_low2(0);
	}
	else if(in == 0)
	{
		stop_delay(100);
		turn();
	}
}

//void stop(void)
//{
//  motor_speed(0,0,0,0);
//	Chassis_Move_Z = 0;
//}
//void Adjust()
//{
//	
////		if(f2==0||f3==0||(f2==0&&f3==0)||(f1==0&&f2==0)||(f1==0&&f2==0&&f3==0))//��ƫ��
////		{
////			lf = rf = lb = rb = 0;
////			Chassis_Move_Z=-400;
////		}
////		else if(f6==0||f7==0||f8==0||(f6==0&&f7==0)||(f7==0&&f8==0))
////		{
////			lf = rf = lb = rb = 0;
////			Chassis_Move_Z=400;		
////		}
////	  if(f4==0 || f5==0)
////		{
//		stop();
//	  stop_delay(100);
////		}

//}

