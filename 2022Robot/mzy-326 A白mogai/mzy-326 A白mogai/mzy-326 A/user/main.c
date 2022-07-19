/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       main.c/h
  * @brief      stm32��ʼ���Լ���ʼ����freeRTOS��h�ļ��������ȫ�ֺ궨���Լ�
  *             typedef һЩ������������
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
#include "main.h"

#include "stm32f4xx.h"//����ͷ�ļ�

#include "adc.h"
#include "buzzer.h"
#include "can.h"
#include "delay.h"
#include "led.h"
#include "servo.h"
#include "key.h"
#include "power_ctrl.h"
#include "sys.h"
#include "timer.h"
#include "limit.h"
#include "usart6.h"
#include "laser.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"


#include "remote_control.h"
#include "start_task.h"
#include "chassis_task.h"
#include "MPU_Temperature.h"
#include "mpu6050.h"
#include "myiic.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"



void BSP_init(void);


//�������Լ���
bool pass_flag=1;


int main(void)
{
    BSP_init();

    delay_ms(100);
    startTast();
    vTaskStartScheduler();	
    while (1)
    {
      ;
    }
}

//�ĸ�24v ��� ���ο��� ��� 709us
#define POWER_CTRL_ONE_BY_ONE_TIME 709

void BSP_init(void)
{
		static portTickType ulCurrentTime = 0;
  	static portTickType ulLoopTime    = 0;
	  static int16_t  	sTimeCnt      = 0;
    //�ж��� 4
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    //��ʼ��ʱ��
    delay_init(configTICK_RATE_HZ);
    //led�Ƴ�ʼ��
    led_configuration();
	  //IO�ڳ�ʼ��
	  key_Init();
    //24������ƿ� ��ʼ��
    power_ctrl_configuration();
    //��������ʼ��
    //buzzer_init(30000, 90);
    //��ʱ��6 ��ʼ��
    TIM6_Init(60000, 90);
	  //��ʱ��5 ��ʼ��
    TIM5_Init();
	    //ң������ʼ��
    remote_control_init();
    //CAN�ӿڳ�ʼ��
    CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
    CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
    usart6_init(115200);     //����ϵͳ���ڳ�ʼ��
    //24v ��� �����ϵ�
    for (uint8_t i = POWER1_CTRL_SWITCH; i < POWER4_CTRL_SWITCH + 1; i++)
    {
        power_ctrl_on(i);
        delay_us(POWER_CTRL_ONE_BY_ONE_TIME);
    }
//flash��ȡ��������У׼ֵ�Żض�Ӧ����
//		
#if 0
      MPU_Init();
			while (mpu_dmp_init()) 
	   {
			ulCurrentTime = xTaskGetTickCount();

			if (ulCurrentTime >= ulLoopTime)  
			{
				  /* 100MS��ʱ */
					ulLoopTime = ulCurrentTime + TIME_STAMP_100MS;
				
				  /* 300ms�����Լ� */
					if (sTimeCnt >= 2) 
					{
							pass_flag = 0;
							sTimeCnt  = 0;//10;
					}
					else
					{
							sTimeCnt++;
					}
			}
	    }
#endif

}

