/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       can_receive.c/h
  * @brief      完成can设备数据收发函数，该文件是通过can中断完成接收
  * @note       该文件不是freeRTOS任务
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

#include "CAN_Receive.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "stm32f4xx.h"
#include "can.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

//底盘电机数据读取
#define get_motor_measure(ptr, rx_message)                                                     \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);     \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]); \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
    }



//声明电机变量
static motor_measure_t motor_yaw, motor_pit, motor_trigger, motor_chassis[4];
static CanTxMsg GIMBAL_TxMessage;


/**
  * @brief  CAN1接收中断
  * @param  void
  * @retval void
  */
void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage;
	int16_t speed_measure,rota_measure,current_measure;

   // CAN_Receive(CAN1, 0, &RxMessage);
	if(CAN_GetITStatus(CAN1,CAN_IT_FMP0)!=RESET)
	{
		CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0);
	  CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
	}
	
				//底盘电机转速读取,机械角度暂时没用
	if(RxMessage.StdId == 0x203)//左
	{
		rota_measure  = ((int16_t)RxMessage.Data[0]<<8|RxMessage.Data[1]);
		GIMBAL_UpdateAngle(rota_measure);
		
		speed_measure = ((int16_t)RxMessage.Data[2]<<8|RxMessage.Data[3]);
		GIMBAL_UpdateSpeed(speed_measure);
		
		current_measure = ((int16_t)RxMessage.Data[4]<<8|RxMessage.Data[5]);
		GIMBAL_UpdateCurrent(current_measure);
	}
	
	if(RxMessage.StdId == 0x205)//左
	{
		rota_measure   = ((int16_t)RxMessage.Data[0]<<8|RxMessage.Data[1]);
		CHASSIS_UpdateMotorAngle(LEFT_FRON_201, rota_measure);
		
		speed_measure  = ((int16_t)RxMessage.Data[2]<<8|RxMessage.Data[3]);
		CHASSIS_UpdateMotorSpeed(LEFT_FRON_201, speed_measure);
		
		current_measure = ((int16_t)RxMessage.Data[4]<<8|RxMessage.Data[5]);
		CHASSIS_UpdateMotorCur(LEFT_FRON_201, current_measure);
	}
	
	if(RxMessage.StdId == 0x206)//右
	{
		rota_measure   = ((int16_t)RxMessage.Data[0]<<8|RxMessage.Data[1]);
		CHASSIS_UpdateMotorAngle(RIGH_FRON_202, rota_measure);
		
		speed_measure  = ((int16_t)RxMessage.Data[2]<<8|RxMessage.Data[3]);
		CHASSIS_UpdateMotorSpeed(RIGH_FRON_202, speed_measure);
		
		current_measure = ((int16_t)RxMessage.Data[4]<<8|RxMessage.Data[5]);
		CHASSIS_UpdateMotorCur(RIGH_FRON_202, current_measure);
	}
	
	if(RxMessage.StdId == 0x207)//左后
	{
		rota_measure   = ((int16_t)RxMessage.Data[0]<<8|RxMessage.Data[1]);
		CHASSIS_UpdateMotorAngle(LEFT_BACK_203, rota_measure);
		
		speed_measure  = ((int16_t)RxMessage.Data[2]<<8|RxMessage.Data[3]);
		CHASSIS_UpdateMotorSpeed(LEFT_BACK_203, speed_measure);
		
		current_measure = ((int16_t)RxMessage.Data[4]<<8|RxMessage.Data[5]);
		CHASSIS_UpdateMotorCur(LEFT_BACK_203, current_measure);
	}
	
	if(RxMessage.StdId == 0x208)//右后
	{
		rota_measure   = ((int16_t)RxMessage.Data[0]<<8|RxMessage.Data[1]);
		CHASSIS_UpdateMotorAngle(RIGH_BACK_204, rota_measure);
		
		speed_measure  = ((int16_t)RxMessage.Data[2]<<8|RxMessage.Data[3]);
		CHASSIS_UpdateMotorSpeed(RIGH_BACK_204, speed_measure);
		
		current_measure = ((int16_t)RxMessage.Data[4]<<8|RxMessage.Data[5]);
		CHASSIS_UpdateMotorCur(RIGH_BACK_204, current_measure);
	}

	}
	



//can2中断
void CAN2_RX0_IRQHandler(void)
{
    CanRxMsg RxMessage;
	  volatile float speed_measure,rota_measure,current_measure;
    if (CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
        CAN_Receive(CAN2, CAN_FIFO0, &RxMessage);
    }
		
}



//Chassis   0X1FF   4个2006   编号 5 6 7 8 //CAN1
//Gimbal    0x200   1个2006   编号 1     //CAN1



void TIM6_DAC_IRQHandler(void)
{
    if( TIM_GetITStatus( TIM6, TIM_IT_Update )!= RESET )
    {

        TIM_ClearFlag( TIM6, TIM_IT_Update );
#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
        CAN_Transmit( GIMBAL_CAN,  &GIMBAL_TxMessage );
#endif
        TIM_Cmd(TIM6,DISABLE);
    }
}
//CAN 发送 0x700的ID的数据，会引发M3508进入快速设置ID模式
void CAN_CMD_CHASSIS_RESET_ID(void)
{

    CanTxMsg TxMessage;
    TxMessage.StdId = 0x700;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = 0;
    TxMessage.Data[1] = 0;
    TxMessage.Data[2] = 0;
    TxMessage.Data[3] = 0;
    TxMessage.Data[4] = 0;
    TxMessage.Data[5] = 0;
    TxMessage.Data[6] = 0;
    TxMessage.Data[7] = 0;

    CAN_Transmit(CAN2, &TxMessage);
}

//发送底盘电机控制命令
void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CAN_CHASSIS_ALL_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = motor1 >> 8;
    TxMessage.Data[1] = motor1;
    TxMessage.Data[2] = motor2 >> 8;
    TxMessage.Data[3] = motor2;
    TxMessage.Data[4] = motor3 >> 8;
    TxMessage.Data[5] = motor3;
    TxMessage.Data[6] = motor4 >> 8;
    TxMessage.Data[7] = motor4;

    CAN_Transmit(CHASSIS_CAN, &TxMessage);
}

void CAN_CMD_GIMBAL(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    GIMBAL_TxMessage.StdId = CAN_GIMBAL_ALL_ID;
    GIMBAL_TxMessage.IDE = CAN_ID_STD;
    GIMBAL_TxMessage.RTR = CAN_RTR_DATA;
    GIMBAL_TxMessage.DLC = 0x08;
    GIMBAL_TxMessage.Data[0] = (yaw >> 8);
    GIMBAL_TxMessage.Data[1] = yaw;
    GIMBAL_TxMessage.Data[2] = (pitch >> 8);
    GIMBAL_TxMessage.Data[3] = pitch;
    GIMBAL_TxMessage.Data[4] = (shoot >> 8);
    GIMBAL_TxMessage.Data[5] = shoot;
    GIMBAL_TxMessage.Data[6] = 0;
    GIMBAL_TxMessage.Data[7] = 0;

    CAN_Transmit( CHASSIS_CAN,  &GIMBAL_TxMessage );

}


//返回yaw电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void)
{
    return &motor_yaw;
}
//返回pitch电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void)
{
    return &motor_pit;
}
//返回trigger电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Trigger_Motor_Measure_Point(void)
{
    return &motor_trigger;
}
//返回底盘电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}

