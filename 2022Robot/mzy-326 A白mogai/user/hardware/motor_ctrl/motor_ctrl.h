#ifndef MOTORCTRL_H
#define MOTORCTRL_H
#include "main.h"




/*---------------------��־λ����(��־λʹ�ù���ִ�н����ر��Լ��ı�־λ��������������ı�־λ)---------------*/ 
typedef struct
{	
	bool get; //ֱ�б�־λ
  bool senor; //�ж�ʮ��·�ڵ���·ѭ����־λ
	bool mid; //�Ƿ�ʹ������
	bool turn; //ת���־λ
	bool back; //���˱�־λ
	bool posi_go; //λ�û�·����־λ
	bool arm; //��е�۱�־λ
	bool open_flag; //��־λ������־λ
	bool delay; //����ִ����֮��ͣ�������ı�־λ
} Flag;



typedef enum
{	
	LOW = 0,
	HIGH = 1, 
} Speed;



void avoi(void);
void sensor_update(void);
void motor_speed(float left_front,float right_front,float left_behind,float right_behind);
void left(float left_front,float right_front,float left_behind,float right_behind);
void right(float left_front,float right_front,float left_behind,float right_behind);
void go_low(fp32 z);
void go_low2(fp32 z);
void go_low3(fp32 z);
void go_mid(fp32 z);
void go_high(fp32 z);
void High(void);
void Mid(void);
void Low(void);
void Low2(void);
void Low3(void);
void Low4(void);
void Low5(void);
void stop(void);
void back(void);
void Forward(Speed speed,float NUM,float IF);
void Forward1(Speed speed,float NUM,float IF);
void low_Forward(void);
void low_Forward0(void);
void Low_Forward_AdvanceStop(void);
void low_Forward1(void);
void low_Forward11(void);
void Back0(float NUM);
void Back(float NUM);
void Back1(float NUM);
void stop_delay(fp32 time);
void stop_forward(fp32 time);
void Adjust(void);
void Back112(float NUM) ;
void Back12(float NUM) ;
void stop_forward1(fp32 time);
void Step(void);
void Step1(void);
void Step6(void);
void BStep(void);
void turn(void);
#endif
