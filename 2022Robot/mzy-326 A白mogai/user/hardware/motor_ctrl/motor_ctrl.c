#include "motor_ctrl.h"
#include "chassis_task.h"
#include "math.h"
#include "stdio.h"
#include <stdlib.h>
#include "mpu6050.h"
#include "inv_mpu.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"



#define high_speed 13000
#define mid_speed 8000
#define low_speed 5000

float lf,rf,lb,rb;//�ĸ����ӵ��ٶ�

extern int i;
float num;
extern float Cloud_Angle_Measure_Yaw;
extern bool mode_change;
Flag flag;
extern float Chassis_Move_Z;//������ת
/*----------------------------------------------*ѭ��*---------------------------------------------------------*/

float f1,f2,f3,f4,f5,f6,f7,f8,b1,b2,b3,b4,b5,b6,b7,b8,l1,l2,l3,l4,l5,r1,r2,r3,r4,r5,b1;
float in;//����
int flagH,flaghh;
extern int target;

/********************0ʶ�𵽣�1û��ʶ��************************/
#define left_1  ((f5==0&&f6==0&&f7==1)||(f5==1&&f6==0&&f7==1))//11110011����11111011  ����ƫ��̶��?
#define left_2  ((f6==0&&f7==0&&f8==1)||(f6==1&&f7==0&&f8==1)||(f5==0&&f6==0&&f7==0))//11111001����11111101 11110001  ����ƫ��̶���?
#define left_3  ((f6==1&&f7==0&&f8==0)||(f6==1&&f7==1&&f8==0)||(f6==0&&f7==0&&f8==0))//11111100����11111110 11111000  ����ƫ��̶ȴ�?
#define right_1 ((f2==1&&f3==0&&f4==0)||(f2==1&&f3==0&&f4==1))//11001111����11011111  ����ƫ�ҳ̶�С
#define right_2 ((f2==0&&f3==0&&f4==1)||(f2==0&&f3==1&&f4==1)||(f2==0&&f3==0&&f4==0))//10011111����10111111 10001111  ����ƫ�ҳ̶���
#define right_3 ((f1==0&&f2==0&&f3==1)||(f1==0&&f2==1&&f3==1)||(f1==0&&f2==0&&f3==0))//00111111����01111111 00011111����ƫ�ҳ̶ȴ�
#define middle  ((f1==0&&f2==0&&f3==0&&f4==0&&f5==0&&f6==0&&f7==0&&f8==0)|| \
(f1==1&&f2==1&&f3==1&&f4==1&&f5==1&&f6==1&&f7==1&&f8==1)||(f4==1&&f5==0&&f6==1)||(f3==1&&f4==0&&f5==1)|| \
(f3==1&&f4==0&&f5==0&&f6==1)||(f2==1&&f3==0&&f4==0&&f5==0&&f6==0&&f7==1)|| \
(f1==1&&f2==0&&f3==0&&f4==0&&f5==0&&f6==0&&f7==0&&f8==1))  

//#define left_1  (f1==1&&f2==1&&f3==0&&f4==0&f5==1)//11001  ����ƫ��̶��?
//#define left_2  (f1==1&&f2==1&&f3==1&&f4==0&f5==1)//11101  ����ƫ��̶���?
//#define left_3  ((f1==1&&f2==1&&f3==1&&f4==0&f5==0)||(f1==1&&f2==1&&f3==1&&f4==1&f5==0))//11100 11110  ����ƫ��̶ȴ�?
//#define right_1 (f1==1&&f2==0&&f3==0&&f4==1&f5==1)//10011  ����ƫ�ҳ̶�С
//#define right_2 (f1==1&&f2==0&&f3==1&&f4==1&f5==1)//10111  ����ƫ�ҳ̶���
//#define right_3 ((f1==0&&f2==0&&f3==1&&f4==1&f5==1)||(f1==0&&f2==1&&f3==1&&f4==1&f5==1))//00111 01111  ����ƫ�ҳ̶ȴ�
//#define middle  ((f1==1&&f2==1&&f3==0&&f4==1&f5==1)|| \
//(f1==1&&f2==1&&f3==1&&f4==1&f5==1)||(f1==0&&f2==0&&f3==0&&f4==0&f5==0)|| \
//(f1==1&&f2==0&&f3==0&&f4==0&f5==1))  


//00000000����11111111����11101111����11110111����11100111  �����Ƚ���



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
	
	b1=GPIO_ReadInputDataBit(GPIOI,GPIO_Pin_6);//X�Һ�
	b2=!GPIO_ReadInputDataBit(GPIOI,GPIO_Pin_5);//w������
	
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
/*------------------------???��????(???��??��?????��?????????????��???????????????????)---------------*/  
	mode_change=FALSE;
	if(flag.get == TRUE)  //???��??TRUE???????????
	{
		flag.open_flag = FALSE; 		
		num=NUM;
		flaghh=1;
		flag.get = FALSE;
    flag.senor = FALSE;		//????????��????????num???
		if(speed == LOW) 
		{
			Low();
		}		
		else if(speed == HIGH)//l3???????��????speed=high???????
		{
			High();
		}
	}
	if (flaghh)
	{
	   if(IF == 1) 
	   {
		   flag.mid = TRUE;
			 flaghh=0;
	   }
	   else if(IF == 0)
	  {
		  flag.mid = FALSE;
	   }
	}
/*------------------------------------------------------*/
	if(num>1)
	{
		if(l3==1 && speed == LOW) //l3???????��????speed=low???????
		{
			Low();
			flag.senor = TRUE;
		}		
		else if(l3==1 && speed == HIGH)//l3???????��????speed=high???????
		{
			High();
			flag.senor = TRUE; //???��?1	
		}
		if(l3==0 && flag.senor == TRUE)//l3?????��??????��??��?1???num??1
		{
			num--;
			flag.senor = FALSE; //??��??��?0?????num?????????????????????
	  }
  }
	else if(num==1)
	{
		if(flag.mid == TRUE)
		{
		  Mid();  //????????MID????
		}
		else if(flag.mid == FALSE)
		{
			Low();  
		}
		if((l1==0 || l2 ==0) && l3 == 1) //???????????????????????LOW????
		{
			Low();
			flag.senor = TRUE;
			flag.mid = FALSE;
		}	
		else if(l2==0 && l3 == 0)
		{
			Low();		
		}
	
		else if(l3 == 0 && flag.senor == TRUE) 	//l5else if(l4==0 && l5 == 1 && flag.senor == TRUE) 
		{
			num--;
			stop();
		}
	}
	else if(num==0)
	{
		//TODO:?????????????????????????

		/******************start***********************/

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

		/********************end***********************/
		
		//stop_delay(100);
		i++;
		flag.get = FALSE; //?????��??��
		flag.arm = TRUE; //??????��??
		flag.turn = TRUE; //???????
		mode_change=TRUE; 
		flag.posi_go = TRUE;
		flag.open_flag = TRUE;
		flag.back = TRUE;
		
	}
}
void Forward1(Speed speed,float NUM,float IF) 
{
	chassis_feedback_update(Cloud_Angle_Measure_Yaw);
/*------------------------��־λ����(��־λʹ�ù���ִ�н����ر��Լ��ı�־λ��������������ı�־�?)---------------*/  
	mode_change=FALSE;
	if(flag.get == TRUE)  //��־λ��TRUEʱ��ʾ����ֱ��
	{
		flag.open_flag = FALSE; 		
		num=NUM;
		flaghh=1;
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
	if (flaghh)
	{
	   if(IF == 1) 
	   {
		   flag.mid = TRUE;
			 flaghh=0;
	   }
	   else if(IF == 0)
	  {
		  flag.mid = FALSE;
	   }
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
		  Mid();  //���һ����MID����
		}
		else if(flag.mid == FALSE)
		{
			Low();  
		}
		if((l3==0 || l4 ==0) && l5 == 1) //���һ���?��Ѱ�����ߣ���ʱ��LOW����
		{
			Low();
			flag.senor = TRUE;
			flag.mid = FALSE;
		}	
		else if(l4==0 && l5 == 0)
		{
			Low();		
		}
	//	else if(l5==0 && flag.senor == TRUE) 
		else if(l5== 0 && flag.senor == TRUE) //5
		{
			num--;
			stop();
		}
	}
	else if(num==0)
	{
		//TODO:����ѭ������ʱҪע�͵�����Ĵ���

		/******************start***********************/

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

		/********************end***********************/
		
		//stop_delay(100);
		i++;
		flag.get = FALSE; //�ر�ֱ�б�־λ
		flag.arm = TRUE; //������е��
		flag.turn = TRUE; //����ת��
		mode_change=TRUE; 
		flag.posi_go = TRUE;
		flag.open_flag = TRUE;
		flag.back = TRUE;
		
	}
}

/*************************************************************************
 * @brief ����ǰ������Ŀ�����ǰɲ��?
 * @param void
 * @return void
**************************************************************************/
void Low_Forward_AdvanceStop()
{
	chassis_feedback_update(Cloud_Angle_Measure_Yaw);
/*------------------------��־λ����(��־λʹ�ù���ִ�н����ر��Լ��ı�־λ��������������ı�־�?)---------------*/  
	mode_change=FALSE;
	if(flag.get == TRUE)  //��־λ��TRUEʱ��ʾ����ֱ��
	{
		flag.open_flag = FALSE; 		
		num=1;
		flagH=0;
		flag.get = FALSE;
    flag.senor = FALSE;		//��ֹ��ʮ��·��������num��һ
		Low3();
		
	}
  if(num==1)
	{
		if(f3 == 0 && f4 == 0 && f5 == 0 && f6 == 0) //ǰ��ѭ��Ѱ�����ߣ���ʱ��LOW����
		{
			Low3();
			flag.senor = TRUE;
			flag.mid = FALSE;
			flagH=1;
		}	
		if(flagH)
		{
		  Low3();
		}
		if((l1 == 1 && l2 == 0) && flag.senor == TRUE) 
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
//		stop_delay(100);
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
/*------------------------��־λ����(��־λʹ�ù���ִ�н����ر��Լ��ı�־λ��������������ı�־�?)---------------*/  
	mode_change=FALSE;
	if(flag.get == TRUE)  //��־λ��TRUEʱ��ʾ����ֱ��
	{
		flag.open_flag = FALSE; 		
		num=1;
		flagH=0;
		flag.get = FALSE;
    flag.senor = FALSE;		//��ֹ��ʮ��·��������num��һ
		Low3();
		
	}
  if(num==1)
	{
		if(f3 == 0 && f4 == 0 && f5 == 0 && f6 == 0) //ǰ��ѭ��Ѱ�����ߣ���ʱ��LOW����
		{
			Low2();
			flag.senor = TRUE;
			flag.mid = FALSE;
			flagH=1;
		}	
		if(flagH)
		{
		  Low2();
		}
//	if(l3 == 0 && l2 == 1 && flag.senor == TRUE) 
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
		else if(fabs(target-Cloud_Angle_Measure_Yaw)>1.801)//1.801
			{
			  Gyro_turn1(target);
			}
		else if((fabs(target-Cloud_Angle_Measure_Yaw)<=1.8)&&(l2==0||l4==0||l3==0||l5==0||r2==0||r3==0))
		{
			stop_delay(150);
		}
//		stop_delay(100);
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
/*------------------------��־λ����(��־λʹ�ù���ִ�н����ر��Լ��ı�־λ��������������ı�־�?)---------------*/  
	mode_change=FALSE;
	if(flag.get == TRUE)  //��־λ��TRUEʱ��ʾ����ֱ��
	{
		flag.open_flag = FALSE; 		
		num=1;
		flagH=0;
		flag.get = FALSE;
    flag.senor = FALSE;		//��ֹ��ʮ��·��������num��һ
		Low3();
		
	}
  if(num==1)
	{
		if(f3 == 0 && f4 == 0 && f5 == 0 && f6 == 0) //ǰ��ѭ��Ѱ�����ߣ���ʱ��LOW����
		{
			Low2();
			flag.senor = TRUE;
			flag.mid = FALSE;
			flagH=1;
		}	
		if(flagH)
		{
		  Low2();
		}
		//	if(l4 == 0 && l2 == 1 && flag.senor == TRUE)
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
//		stop_delay(100);
		flag.get = FALSE; //�ر�ֱ�б�־λ
		flag.arm = TRUE; //������е��
		flag.turn = TRUE; //����ת��
		mode_change=TRUE; 
		flag.posi_go = TRUE;
		flag.open_flag = TRUE;
		flag.back = TRUE;
	}
}

//����һ��1
void low_Forward1(void) 
{
	chassis_feedback_update(Cloud_Angle_Measure_Yaw);
/*------------------------��־λ����(��־λʹ�ù���ִ�н����ر��Լ��ı�־λ��������������ı�־�?)---------------*/  
	mode_change=FALSE;
	if(flag.get == TRUE)  //��־λ��TRUEʱ��ʾ����ֱ��
	{
		flag.open_flag = FALSE; 		
		num=1;
		flagH=0;
		flag.get = FALSE;
    flag.senor = FALSE;		//��ֹ��ʮ��·��������num��һ
		Low3();
	}
  if(num==1)
	{
		if(f3 == 0 && f4 == 0 && f5 == 0 && f6 == 0) //ǰ��ѭ��Ѱ�����ߣ���ʱ��LOW����
		{
			Low2();
			flag.senor = TRUE;
			flag.mid = FALSE;
			flagH=1;
			
		}	
		if(flagH)
		{
		  Low2();
		}
		if(l2 == 0 && l1 == 1 && flag.senor == TRUE) 
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
			  Gyro_turn1(target-0.1);
			}
		else if((fabs(target-Cloud_Angle_Measure_Yaw)<=1.8)&&(l2==0||l4==0||l3==0||l5==0||r2==0||r3==0))
		{
			stop_delay(150);
		}
		flag.get = FALSE; //�ر�ֱ�б�־λ
		flag.arm = TRUE; //������е��
		flag.turn = TRUE; //����ת��
		mode_change=TRUE; 
		flag.posi_go = TRUE;
		flag.open_flag = TRUE;
		flag.back = TRUE;
	}
}

//����һ��11
void low_Forward11(void) 
{
	chassis_feedback_update(Cloud_Angle_Measure_Yaw);
/*------------------------��־λ����(��־λʹ�ù���ִ�н����ر��Լ��ı�־λ��������������ı�־�?)---------------*/  
	mode_change=FALSE;
	if(flag.get == TRUE)  //��־λ��TRUEʱ��ʾ����ֱ��
	{
		flag.open_flag = FALSE; 		
		num=1;
		flagH=0;
		flag.get = FALSE;
    flag.senor = FALSE;		//��ֹ��ʮ��·��������num��һ
		Low3();
	}
  if(num==1)
	{
		if(f3 == 0 && f4 == 0 && f5 == 0 && f6 == 0) //ǰ��ѭ��Ѱ�����ߣ���ʱ��LOW����
		{
			Low2();
			flag.senor = TRUE;
			flag.mid = FALSE;
			flagH=1;
		}	
		if(flagH)
		{
		  Low2();
		}
		if(l5 == 0 && flag.senor == TRUE) 
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


void Back0(float NUM) 
{
	chassis_feedback_update(Cloud_Angle_Measure_Yaw);
/*------------------------��־λ����(��־λʹ�ù���ִ�н����ر��Լ��ı�־λ��������������ı�־�?)---------------*/  
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
		if(l5==1) //l3
		{
			back();
			flag.senor = TRUE;
		}		
		if(l5==0 && flag.senor == TRUE)//l3��ʮ��·����ֱ�б�־λΪ1ʱ��num��1
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

void Back(float NUM) 
{
	chassis_feedback_update(Cloud_Angle_Measure_Yaw);
/*------------------------��־λ����(��־λʹ�ù���ִ�н����ر��Լ��ı�־λ��������������ı�־�?)---------------*/  
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
		if(l3==1) //l3
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
void Back12(float NUM) 
{
	chassis_feedback_update(Cloud_Angle_Measure_Yaw);
/*------------------------��־λ����(��־λʹ�ù���ִ�н����ر��Լ��ı�־λ��������������ı�־�?)---------------*/  
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
		if(r2==1) //l3����ʮ��·����speed=lowʱ������
		{
			back();
			flag.senor = TRUE;
		}		
		if(r3==0 && flag.senor == TRUE)//l3��ʮ��·����ֱ�б�־λΪ1ʱ��num��1
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

void Back112(float NUM) 
{
	chassis_feedback_update(Cloud_Angle_Measure_Yaw);
/*------------------------��־λ����(��־λʹ�ù���ִ�н����ر��Լ��ı�־λ��������������ı�־�?)---------------*/  
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
		if(r3==1) //l3����ʮ��·����speed=lowʱ������
		{
			back();
			flag.senor = TRUE;
	
		}		
		if(r4==0 && flag.senor == TRUE)//l3��ʮ��·����ֱ�б�־λΪ1ʱ��num��1
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
void Back1(float NUM) 
{
	chassis_feedback_update(Cloud_Angle_Measure_Yaw);
/*------------------------��־λ����(��־λʹ�ù���ִ�н����ر��Լ��ı�־λ��������������ı�־�?)---------------*/  
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
	{//l5 l4
		if(l2==1) //l3����ʮ��·����speed=lowʱ������
		{
			back();
			flag.senor = TRUE;
		}		
		if(l2==0 && flag.senor == TRUE)//l3��ʮ��·����ֱ�б�־λΪ1ʱ��num��1
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
volatile int ct=0;

void stop_forward(fp32 time) //д���پ�ͣ��ã����沿���Ƿ�ֹ�?����������?�󣬷�ֹ��ѭһ��
{
	static fp32 delay_time;
	delay_time++;
	if(delay_time > time)
	{
		if(l1 == 0 || l2 == 0 || l3 == 0 || l4 == 0 || l5 == 0)
		{
		 ct++;
		 if(l2 == 0 || l3 == 0 || l1 == 0 ||ct>950)//l5
			{
				stop();
				delay_time = 0;
				flag.delay = FALSE;
				i++;
				ct=0;
			}
			else if(12 == 1)//l2
			{
       Low4();				
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

void stop_forward1(fp32 time) //д���پ�ͣ��ã����沿���Ƿ�ֹ�?����������?�󣬷�ֹ��ѭһ��
{
	static fp32 delay_time;
	delay_time++;
	if(delay_time > time)
	{
		if(l1 == 0 || l2 == 0 || l3 == 0 || l4 == 0 || l5 == 0)
		{
			if(l4 == 0)//l5
			{
				stop();
				delay_time = 0;
				flag.delay = FALSE;
				i++;
			}
			else if(l4 == 1)//l5
			{
				//Low3();	
       Low2();				
			}
		}
		else 
		{
			delay_time = 0;
			flag.delay = FALSE;
		}
	}
}

void Step6()
{
	
	if(b1==1){
	
	
  if(f1 == 0 || f2 == 0 || f7 == 0 || f8 == 0){
		flag.turn = TRUE;
		flag.delay = TRUE;
		flag.get=FALSE;
		flag.arm = TRUE;		
		flag.posi_go = TRUE;
		flag.open_flag = TRUE;
		flag.back = TRUE;
    stop();
		i++;
	}
	else 
	{
		Low4();				
	}
	
	
	}
	else 
	{
		Low4();				
	}

}

void Step()
{
  if(b1==0){
		flag.turn = TRUE;
		flag.delay = TRUE;
		flag.get=FALSE;
		flag.arm = TRUE;		
		flag.posi_go = TRUE;
		flag.open_flag = TRUE;
		flag.back = TRUE;
    stop();
		i++;
	}
	else 
	{
		Low4();				
	}

}
//volatile int CC=0;

//����Ϊ0
void Step1()//555��
{
	
  if(b2==0){
		flag.turn = TRUE;
		flag.delay = TRUE;
		flag.get=FALSE;
		flag.arm = TRUE;		
		flag.posi_go = TRUE;
		flag.open_flag = TRUE;
		flag.back = TRUE;
    stop();
//		CC=0;
		i++;
	}
//	CC++;
	if(b2==1)//b2==0 || l2 == 0 || l3 == 0
	{
		Low4();				
	}

}

void BStep()
{
  if(b1==1&& l3==0){
		flag.turn = FALSE;
		flag.delay = TRUE;
		flag.get=TRUE;
		flag.arm = TRUE;		
		flag.posi_go = TRUE;
		flag.open_flag = TRUE;
		flag.back = TRUE;
    stop();
		i++;
	}
	else 
	{
		back();				
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
void Adjust()
{
	
//		if(f2==0||f3==0||f4==0||(f3==0&&f4==0)||(f2==0&&f3==0)||(f2==0&&f3==0&&f4==0)||(f1==0&&f2==0)||(f1==0&&f2==0&&f3==0))//��ƫ��
//		{
//			lf = rf = lb = rb = 0;
//			Chassis_Move_Z=-500;
//		}
//		else if(f5==0||f6==0||f7==0||f8==0||(f5==0&&f6==0)||(f5==0&&f6==0&&f7==0)||(f6==0&&f7==0)||(f7==0&&f8==0))
//		{
//			lf = rf = lb = rb = 0;
//			Chassis_Move_Z=500;		
//		}
//	  if(f4==0&&f5==0)
//		{
		stop();
	  stop_delay(100);
//	  stop_delay(100);
//		}

}




void go_high(fp32 z)  //num�Ӷ����ٲ���Ƕ���?
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
  motor_speed(low_speed-3000,low_speed-3000,low_speed-3000,low_speed-3000);//3000
	Chassis_Move_Z = z;
}

void go_low3(fp32 z)
{
  motor_speed(low_speed,low_speed,low_speed,low_speed);//3000
	Chassis_Move_Z = z;
}



void go_back(fp32 z)
{
	motor_speed(-low_speed,-low_speed,-low_speed,-low_speed);
	Chassis_Move_Z = -z;
}

/*----------------------------------����----------------------------------*/

void Low4(void) //ת�����?
{ 
		go_low2(50);//������ 50
	//zhong-464 505 480
}
void Low2(void) //����ֻ���ڻ���ǰ��һ�����ľ���
{
  if(middle)
		go_low2(5);//-4
	
	else if(left_1) 
		go_low2(439); //ƫ���һ�����?ֵ,ֵԽ���������Խ��?
	//430
	else if(left_2)
		go_low2(600);//250
	
	else if(left_3) 
		go_low2(700);//
	
	else if(right_1) 
		go_low2(-485); //ƫ�Ҽ�һ����תֵ
	//-455q
	else if(right_2) 
		go_low2(-500);//
	
	else if(right_3) 
		go_low2(-700);//
	
	else 
		go_low2(0);
}

//void Low2(void) //����ֻ���ڻ���ǰ��һ�����ľ���
//{
//  if(middle)
//		go_low2(330);//0 
//	
//	else if(left_1) 
//		go_low2(660); //ƫ���һ�����?ֵ,ֵԽ���������Խ��?
//	//100 295 790
//	else if(left_2)
//		go_low2(890);//
//	//250 680 900
//	else if(left_3) 
//		go_low2(1099);//919
//	
//	else if(right_1) 
//		go_low2(170); //ƫ�Ҽ�һ����תֵ-100
//	
//	else if(right_2) 
//		go_low2(20);//-200
//	
//	else if(right_3) 
//		go_low2(-10);//
//	
//	else 
//		go_low2(0);
//}
//221
void Low3(void) //����ֻ���ڻ���ǰ��һ��ģʽʱʹ��
{
 
	if(middle)
		go_low3(198);
	//198
	else if(left_1) 
		go_low3(480); //ƫ���һ�����?ֵ,ֵԽ���������Խ��?
	//855  450
	else if(left_2)
		go_low3(646);//900
	//900 917
	else if(left_3) 
		go_low3(800);//1100
	
	else if(right_1) 
		go_low3(-198); //ƫ�Ҽ�һ����תֵ650
	//-300 -238
	else if(right_2) 
		go_low3(-200);//900 -599
	
	else if(right_3) 
		go_low3(-330);//790
	
	else 
		go_low3(1100);
}

//void Low3(void) //����ֻ���ڻ���ǰ��һ��ģʽʱʹ��
//{
// 
//	if(middle)
//		go_low3(190);//230
//	
//	else if(left_1) 
//		go_low3(420); //ƫ���һ�����?ֵ,ֵԽ���������Խ��?
//	//300 420
//	else if(left_2)
//		go_low3(820);//820
//	//450 795 822 
//	else if(left_3) 
//		go_low3(1100);//1100
//	
//	else if(right_1) 
//		go_low3(-280); //ƫ�Ҽ�һ����תֵ100
//	//-230 -310
//	else if(right_2) 
//		go_low3(-700);//-710
//	
//	else if(right_3) 
//		go_low3(-970);//1010
//	
//	else 
//		go_low3(0);
//}

void Low5(void)
{
	if(middle)
		go_low2(0);
	
	else if(left_1) 
		go_low2(100); //ƫ���һ�����?ֵ,ֵԽ���������Խ��?
	
	else if(left_2)
		go_low2(200);
	
	else if(left_3) 
		go_low2(300);
	
	else if(right_1) 
		go_low2(-100); //ƫ�Ҽ�һ����תֵ
	
	else if(right_2) 
		go_low2(-200);
	
	else if(right_3) 
		go_low2(-300);
	
	else 
		go_low2(0);
}
//high1
//��ƫ ����ٶ��� ��ƫ ����ٶȿ�
void Low(void) //����
{
	if(middle)
		go_low(0);//0 q
	
	else if(left_1) 
		go_low(450); //ƫ���һ�����?ֵ,ֵԽ���������Խ��?
	//100 450
	else if(left_2)
		go_low(575);
	//200 560
	else if(left_3) 
		go_low(300);//700
	
	else if(right_1) 
		go_low(-200); //-100  -115ƫ�Ҽ�һ����תֵ
	//-100
	else if(right_2) 
		go_low(-200);
	
	else if(right_3) 
		go_low(-300);
	
	else 
		go_low(0);
}


void Mid(void) //����
{
	if(middle)
		go_mid(0);//0 145
	
	else if(left_1) 
		go_mid(600); //ƫ���һ�����?ֵ,ֵԽ���������Խ��?1800
	//600  1100
	else if(left_2)
		go_mid(1500);//1000 4500
	
	else if(left_3) 
		go_mid(1200);//1200 5000
	
	else if(right_1) 
		go_mid(-600); //ƫ�Ҽ�һ����תֵ
	
	else if(right_2) 
		go_mid(-1100);//
	
	else if(right_3) 
		go_mid(-1200);//��������
	
	else 
		go_mid(0);
}


void High(void)//����
{
	if(middle)
		go_high(438);//420
	//440
	else if(left_1) 
//		motor_speed(5600,-5600,5600,-5600);//lf rf lb rb
	  go_high(1200);//ƫ���һ�����?ֵ,ֵԽ���������Խ��?2200
	//300  1980 2020
	else if(left_2) 
		go_high(1400);//3000 2400

	else if(left_3) 
		go_high(1600);//��������
	
	else if(right_1) 
		go_high(-458);//ƫ�Ҽ�һ����תֵ,ֵԽ���������Խ��?-400
	//400
	else if(right_2) 
		go_high(-888);//-450
	
	else if(right_3) 
		go_high(-500);//��������1800
	
	else 
		go_high(0);
}

//right���Ǻ���ƫ�����ұ��ٶȾ�������ĸ�ֵ����ֵ    Խ���׾���ֵԽ��
//left���Ǻ���ƫ��������ٶȾ��Ǽ�С����ֵ����ֵ     Խ���׾�ֵԽxiao
//��ֵ��С����ٶ�
void back(void)//û�е���
{
	if(middle)
		go_back(78);
	//
	else if(left_1) 
		go_back(188);//ƫ���һ�����?ֵ,ֵԽ���������Խ��?100
	//168
	else if(left_2) 
		go_back(230);//200

	else if(left_3) 
		go_back(300);
	
	else if(right_1) 
		go_back(-128);//ƫ�Ҽ�һ����תֵ,ֵԽ���������Խ�50
	//128
	else if(right_2) 
		go_back(-160);
	//159
	else if(right_3) 
		go_back(-300);
	
	else
		go_back(0);
}





