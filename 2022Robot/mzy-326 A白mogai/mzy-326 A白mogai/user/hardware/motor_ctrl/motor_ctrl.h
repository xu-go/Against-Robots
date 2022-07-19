#ifndef MOTORCTRL_H
#define MOTORCTRL_H
#include "main.h"




/*---------------------标志位开关(标志位使用规则：执行结束关闭自己的标志位，开启其他任务的标志位)---------------*/ 
typedef struct
{	
	bool get; //直行标志位
  bool senor; //判断十字路口的那路循迹标志位
	bool mid; //是否使用中速
	bool turn; //转弯标志位
	bool back; //后退标志位
	bool posi_go; //位置环路径标志位
	bool arm; //机械臂标志位
	bool open_flag; //标志位开启标志位
	bool delay; //动作执行完之后停车函数的标志位
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
