/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       main.c/h
  * @brief      stm32初始化以及开始任务freeRTOS。h文件定义相关全局宏定义以及
  *             typedef 一些常用数据类型
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#include "main.h"

#include "stm32f4xx.h"//顶层头文件

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


//陀螺仪自检用
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

//四个24v 输出 依次开启 间隔 709us
#define POWER_CTRL_ONE_BY_ONE_TIME 709

void BSP_init(void)
{
		static portTickType ulCurrentTime = 0;
  	static portTickType ulLoopTime    = 0;
	  static int16_t  	sTimeCnt      = 0;
    //中断组 4
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    //初始化时钟
    delay_init(configTICK_RATE_HZ);
    //led灯初始化
    led_configuration();
	  //IO口初始化
	  key_Init();
    //24输出控制口 初始化
    power_ctrl_configuration();
    //蜂鸣器初始化
    //buzzer_init(30000, 90);
    //定时器6 初始化
    TIM6_Init(60000, 90);
	  //定时器5 初始化
    TIM5_Init();
	    //遥控器初始化
    remote_control_init();
    //CAN接口初始化
    CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
    CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
    usart6_init(115200);     //裁判系统串口初始化
    //24v 输出 依次上电
    for (uint8_t i = POWER1_CTRL_SWITCH; i < POWER4_CTRL_SWITCH + 1; i++)
    {
        power_ctrl_on(i);
        delay_us(POWER_CTRL_ONE_BY_ONE_TIME);
    }
//flash读取函数，把校准值放回对应参数
//		
#if 0
      MPU_Init();
			while (mpu_dmp_init()) 
	   {
			ulCurrentTime = xTaskGetTickCount();

			if (ulCurrentTime >= ulLoopTime)  
			{
				  /* 100MS延时 */
					ulLoopTime = ulCurrentTime + TIME_STAMP_100MS;
				
				  /* 300ms屏蔽自检 */
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

