#include "fric.h"
#include "friction.h"
#include "main.h"
#include "Remote_Control.h"
#include "rc.h"
#include "delay.h"
#include "judge.h"
#include "stdbool.h"


#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

extern  RC_ctrl_t rc_ctrl;

/**************************Ħ���ֿ���***********************************/
float Friction_PWM_Output[3]     = {0, 460, 685};//�ر�  ����  �ڱ�

//Ħ���ֲ�ͬpwm�¶�Ӧ����������ֵ(����),��ñ�ʵ��ֵ��5
uint16_t Friction_PWM_HeatInc[5] = {0,  20,  26,  34,  36};//����ʱ��㶨���ٶ�,������Ը���


//�ٶȵȼ�ѡ��
uint16_t Fric_Speed_Level;

//ң��ģʽ�µĿ�����־λ
uint8_t Friction_Switch = 0;//�뵯�ֿ����ж�����

//Ħ����Ŀ���ٶ�
float Friction_Speed_Target;

//Ħ���ֵȼ�Ŀ��ת��
float Frict_Speed_Level_Target;

//Ħ����ʵ������ٶ�,������б�����
float Friction_Speed_Real;


/*********************************************************************************************************/


/**
  * @brief  Ħ����ʧ�ر���
  * @param  void
  * @retval void
  * @attention ������СPWM���,����ᱨ��
  */
void FRICTION_StopMotor(void)
{
	Friction_Speed_Target = 0;

	if (Friction_Speed_Real > 0)
	{
		Friction_Speed_Real -= 1;//3
	}

	if (Friction_Speed_Real < 0)
	{
		Friction_Speed_Real = 0;
	}

	TIM4_FrictionPwmOutp( Friction_Speed_Real, Friction_Speed_Real );

}

/************************Ħ�����ܿ���*****************************/

/**
  * @brief  Ħ���ֿ��ƺ���
  * @param  void
  * @retval void
  * @attention Ħ����ֹͣת��,����ر�,Ħ���ִӹرյ�����,��̨̧ͷ����
  */

void friction_Init(void)
{
	 Fric_Speed_Level = FRI_OFF;
	 Friction_Speed_Target = 0;
	 Friction_Speed_Real   = 0;
}





/**
  * @brief  Ħ����ң�ؿ��ƺ���
  * @param  void
  * @retval void
  * @attention 
  */
void friction_RC_Ctrl(void)
{
	Fric_Speed_Level = FRI_LOW;//ң��ģʽ�µ��ٶ�ѡ�񣬵����٣������¼���ⵯ
	
	if (FRIC_RcSwitch( ) == TRUE)//�ж�״̬�л�
	{	//�л�Ϊ��
	 if (Friction_Speed_Target > Friction_PWM_Output[FRI_OFF])
	 {
		 Friction_Speed_Target = Friction_PWM_Output[FRI_OFF];	
	 }
	 else//�л�Ϊ��
	 {
		 Friction_Speed_Target = Friction_PWM_Output[Fric_Speed_Level];//Ħ����Ŀ��ֵ����0,��־��ң��ģʽ�¿���,����pitchҪ̧ͷ
	 }
  }
	
	else
  {
	 if(Friction_Speed_Target > Friction_PWM_Output[Fric_Speed_Level])
	 {
		 Friction_Speed_Target = Friction_PWM_Output[Fric_Speed_Level];
 	 }
  }
			
	//Ħ�������б��,ע��Ҫ��̧ͷ���ܽ���б��
	Friction_Ramp();

	TIM4_FrictionPwmOutp(Friction_Speed_Real, Friction_Speed_Real);

}


void Fric_mode(uint16_t speed)
{
	
if (FRIC_RcSwitch( ) == TRUE)//�ж�״̬�л�
{
	if(speed == FRI_LOW)
	{
	  Friction_Speed_Target = Friction_PWM_Output[FRI_LOW];
	}
	
	else if(speed == FRI_SENTRY)
	{
		Friction_Speed_Target = Friction_PWM_Output[FRI_SENTRY];
	}
}
	if(speed == FRI_OFF)
	{
		Friction_Speed_Target = Friction_PWM_Output[FRI_OFF];
	}
	
	 
	Friction_Ramp();

	TIM4_FrictionPwmOutp(Friction_Speed_Real, Friction_Speed_Real);
}


/**
  * @brief  Ħ�����ڱ�ģʽ���ƺ���
  * @param  void
  * @retval void
  * @attention 
  */
void friction_AUTO_Ctrl(void)
{
	Friction_Speed_Target = Friction_PWM_Output[FRI_SENTRY];
	
	Friction_Ramp();

	TIM4_FrictionPwmOutp(Friction_Speed_Real, Friction_Speed_Real);
}





/***********Ħ����������̨̧ͷ�жϺ���*************/

/**
  * @brief  Ħ�����Ƿ��Ѿ�����
  * @param  void
  * @retval TRUE�ѿ���   FALSEδ����
  * @attention 
  */
uint8_t FRIC_IfWait( void )
{
    if (Friction_Speed_Target > 0)//ͨ���ı�Ħ����Ŀ��ֵ����־ң��ģʽ��pitch�Ƿ�Ҫ̧ͷ
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}


/**
  * @brief  Ħ�����Ƿ����ڿ���
  * @param  void
  * @retval TRUE���ڿ���   FALSE������Ŀ���ٶ�
  * @attention ���ڿ�����׼�ı���̨PITCH
  */
uint8_t FRIC_IfOpen( void )
{
	static  uint8_t  status = FALSE;
	static uint32_t  ulWait = 0;

	static portTickType ulCurrent = 0;

	ulCurrent = xTaskGetTickCount( );
	
    if (Friction_Speed_Real > 0)
	{
		if (ulCurrent >= ulWait + 1500)//̧ͷʱ��,1.5S
		{
			status = TRUE;
		}
	}
	else
	{
		ulWait = ulCurrent;
		
		status = FALSE;
	}
	
	return status;
}

/**
  * @brief  Ħ����ң�ؿ���
  * @param  void
  * @retval �Ƿ�ת����ǰ״̬
  * @attention �����ֿ����߼���ͬ
  */
bool FRIC_RcSwitch( void )
{
	if (IF_RC_SW2_UP) //ң��ģʽ
	{
		if (IF_RC_SW1_DOWN) //����Ħ��������1
		{
			if (Friction_Switch == FRIC_STEP1)
			{
				Friction_Switch = FRIC_STEP2;
			}
			else if (Friction_Switch == FRIC_STEP2)
			{
				Friction_Switch = FRIC_STEP0; //�ж���ϵ
			}
		}
		else //��־SW1�Ƿ��и�λ�����,�ڸ�λ������²����ٴν���STERP2
		{
			Friction_Switch = FRIC_STEP1; //����SW1���´α任֮ǰһֱ������
		}
	}
	else //s2��������,������Ħ���ֿ���
	{
		Friction_Switch = FRIC_STEP0; //������Ħ���ֿ���Ҳ�������л�������ģʽ
	}


	if (Friction_Switch == FRIC_STEP2)
	{
		return TRUE; //ֻ��SW1���±任��ʱ���ΪTRUE
	}
	else
	{
		return FALSE;
	}
}


/*************Ħ���ָ�������****************/

/**
  * @brief  Ħ�������б��
  * @param  void
  * @retval void
  * @attention 
  */
void Friction_Ramp(void)
{
	if (Friction_Speed_Real < Friction_Speed_Target)//����
	{
		Friction_Speed_Real += 5;
		if(Friction_Speed_Real > Friction_Speed_Target)
		{
			Friction_Speed_Real = Friction_Speed_Target;
		}
	}
	else if (Friction_Speed_Real > Friction_Speed_Target)//�ر�
	{
		Friction_Speed_Real -= 5;
	}
	
	if (Friction_Speed_Real < 0)
	{
		Friction_Speed_Real = 0;
	}
}

/**
  * @brief  ��ȡ��ǰĦ����PWM���ֵ
  * @param  void
  * @retval ʵ��PWMֵ
  * @attention ������ֹĦ�����ٶȹ��͵�����²��̵�ת��
  */
float Fric_GetSpeedReal(void)
{
	return Friction_Speed_Real;
}

