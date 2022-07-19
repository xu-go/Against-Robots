#include "pid_control.h"
#include "remote_control.h"
#include "chassis_task.h"
//-------------------------------------------------PID快速调节法-----------------------------------------//

extern RC_ctrl_t rc_ctrl;
extern float Chassis_Speed_kpid[4][3];//	motorID kp/ki/kd


/**
  * @brief  
  * @param  void
  * @retval void
  * @attention 
  */
#define REVOL_STEP0    0		//失能标志
#define REVOL_STEP1    1		//SW1复位标志
#define REVOL_STEP2    2		//开关标志
uint8_t	PID_Switch0 = 0;//遥控模式开关标志位转换
bool PID_KP_Switch0(void)
{
	if (IF_RC_SW2_UP)//机械模式
	{
		if (rc_ctrl.rc.ch[0]>600)
		{
			if (PID_Switch0 == REVOL_STEP1)
			{
				PID_Switch0 = REVOL_STEP2;
			}
			else if (PID_Switch0 == REVOL_STEP2)
			{
				PID_Switch0 = REVOL_STEP0;
			}
		}
		else		
		{
			PID_Switch0 = REVOL_STEP1;
		}
	}
	else
	{
		PID_Switch0 = REVOL_STEP0;
	}
	
	if (PID_Switch0 == REVOL_STEP2)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}


uint8_t	PID_Switch1 = 0;//遥控模式开关标志位转换
bool PID_KP_Switch1(void)
{
	if (IF_RC_SW2_UP)
	{
		if (rc_ctrl.rc.ch[0]<-600)
		{
			if (PID_Switch1 == REVOL_STEP1)
			{
				PID_Switch1 = REVOL_STEP2;
			}
			else if (PID_Switch1 == REVOL_STEP2)
			{
				PID_Switch1 = REVOL_STEP0;
			}
		}
		else		
		{
			PID_Switch1 = REVOL_STEP1;
		}
	}
	else
	{
		PID_Switch1 = REVOL_STEP0;
	}
	
	if (PID_Switch1 == REVOL_STEP2)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}


uint8_t	PID_Switch3 = 0;//遥控模式开关标志位转换
bool PID_KI_Switch0(void)
{
	if (IF_RC_SW2_UP)
	{
		if (rc_ctrl.rc.ch[2]>600)
		{
			if (PID_Switch3 == REVOL_STEP1)
			{
				PID_Switch3 = REVOL_STEP2;
			}
			else if (PID_Switch3 == REVOL_STEP2)
			{
				PID_Switch3 = REVOL_STEP0;
			}
		}
		else		
		{
			PID_Switch3 = REVOL_STEP1;
		}
	}
	else
	{
		PID_Switch3 = REVOL_STEP0;
	}
	
	if (PID_Switch3 == REVOL_STEP2)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}



uint8_t	PID_Switch4 = 0;//遥控模式开关标志位转换
bool PID_KI_Switch1(void)
{
	if (IF_RC_SW2_UP)
	{
		if (rc_ctrl.rc.ch[2]<-600)
		{
			if (PID_Switch4 == REVOL_STEP1)
			{
				PID_Switch4 = REVOL_STEP2;
			}
			else if (PID_Switch4 == REVOL_STEP2)
			{
				PID_Switch4 = REVOL_STEP0;
			}
		}
		else		
		{
			PID_Switch4 = REVOL_STEP1;
		}
	}
	else
	{
		PID_Switch4 = REVOL_STEP0;
	}
	
	if (PID_Switch4 == REVOL_STEP2)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}



int16_t PID_ShootNum0;
int16_t PID_ShootNum1;
int16_t PID_ShootNum2;
int16_t PID_ShootNum3;
void PID_KP_Ctrl(void)
{
		if(PID_KP_Switch0() == TRUE)
		{
			PID_ShootNum0++;
		}
	  if(PID_ShootNum0 != 0)
		{
			PID_ShootNum0--;
			Chassis_Speed_kpid[RIGH_BACK_204][KP] = Chassis_Speed_kpid[RIGH_BACK_204][KP]+1;
		}
		
	  if(PID_KP_Switch1() == TRUE)
		{
			PID_ShootNum1++;
		}
	  if(PID_ShootNum1 != 0)
		{
			PID_ShootNum1--;
			Chassis_Speed_kpid[RIGH_BACK_204][KP] = Chassis_Speed_kpid[RIGH_BACK_204][KP]-1;
		}
		
		if(PID_KI_Switch0() == TRUE)
		{
			PID_ShootNum2++;
		}
	  if(PID_ShootNum2 != 0)
		{
			PID_ShootNum2--;
			Chassis_Speed_kpid[RIGH_BACK_204][KI] = Chassis_Speed_kpid[RIGH_BACK_204][KI]+5;
		}
		
		if(PID_KI_Switch1() == TRUE)
		{
			PID_ShootNum3++;
		}
	  if(PID_ShootNum3 != 0)
		{
			PID_ShootNum3--;
			Chassis_Speed_kpid[RIGH_BACK_204][KI] = Chassis_Speed_kpid[RIGH_BACK_204][KI]-5;
		}
}


