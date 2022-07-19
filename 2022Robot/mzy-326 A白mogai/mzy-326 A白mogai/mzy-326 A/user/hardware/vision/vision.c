#include <vision.h>
#include "math.h"
#include "remote_control.h"
#include "crc.h"
#include "uart7.h"
#include "kalman.h"
#include "chassis_task.h"
#include "FreeRTOS.h"
#include "task.h"

#include "led.h"

float Vision_Comps_Yaw   = 0;//
float Vision_Comps_Pitch = 0;//固定补偿，减小距离的影响
float Vision_Comps_Pitch_Dist = 0;//根据距离补偿





//视觉是否发了新数据,FALSE没有,TRUE发了新的
uint8_t Vision_Get_New_Data = FALSE;

typedef enum
{
	VISION_MANU =0,
	VISION_BUFF =1,
	VISION_AUTO =2,
}VisionActData_t;

VisionSendHeader_t VisionSendHeader;            //发送数据帧头

VisionRecvData_t VisionRecvData;                //接收数据结构体

VisionSendData_t VisionSendData;                //发送数据结构体

VisionRecvData_test VisionRecvDataTest;

xRGBHeader RGBHeader;

uint8_t Vision_New_Data = FALSE;



/**
  * @brief  读取视觉信息
  * @param  uart7缓存数据
  * @retval void
  * @attention  IRQ执行
  */
float b=0.0;
float *a;
uint8_t Vision_Time[2] = {0};            //当前数据和上一次数据
uint8_t Vision_Ping = 0;                 //发送时间间隔
void Vision_Read_Data(uint8_t *ReadFormUart7)
{
	//判断帧头数据是否为0xA5
	if(ReadFormUart7[0] == VISION_SOF)
	{
		//帧头CRC8校验
		if(Verify_CRC8_Check_Sum( ReadFormUart7, VISION_LEN_HEADER ) == TRUE)
		{
			//帧尾CRC16校验
			if(Verify_CRC16_Check_Sum( ReadFormUart7, VISION_LEN_PACKED ) == TRUE)
			{
				//接收数据拷贝
				*a=b;
				memcpy( &VisionRecvData, ReadFormUart7, VISION_LEN_PACKED);	
				Vision_New_Data = TRUE;//标记视觉数据更新了
				
				//帧计算
				Vision_Time[NOW] = xTaskGetTickCount();
				Vision_Ping = Vision_Time[NOW] - Vision_Time[LAST];//计算时间间隔
				Vision_Time[LAST] = Vision_Time[NOW];
			 
		  }
		}
	}
}

/**
  * @brief  发送视觉指令
  * @param  CmdID
  * @retval void
  * @attention  按协议打包好数据发送
  *				CmdID   0x00   关闭视觉
  *				CmdID   0x01   识别红色装甲
  *				CmdID   0x02   识别蓝色装甲
  *				CmdID   0x03   小符
  *				CmdID   0x04   大符
  */
uint8_t vision_send_pack[50] = {0};//大于17就行
void Vision_Send_Data(uint8_t CmdID)
{
	int i=0;
	
	VisionSendHeader.SOF = VISION_SOF;
	VisionSendHeader.CmdID = CmdID;  //遥控或者键盘控制模式
	
	memcpy(vision_send_pack,&VisionSendHeader,VISION_LEN_HEADER);  //写入帧头
	
	Append_CRC8_Check_Sum(vision_send_pack,VISION_LEN_HEADER);     //crc8校验
	
	
	
	for (i = 0; i < VISION_LEN_PACKED; i++)
	{
		Uart7_SendChar( vision_send_pack[i] );
	}
	
	memset(vision_send_pack, 0, 50);
}



void RGB_Read_Data(uint8_t *ReadFromUsart)
{
   memcpy( &RGBHeader, ReadFromUsart, RGB_LEN);
   if(RGBHeader.color == 'R')
	 {
		 led_red_on();
		 led_green_off();
	 }
	 else if(RGBHeader.color == 'G')
	 {
		 led_red_off();
		 led_green_on();		 
	 }
	 
	 
		Vision_New_Data = TRUE;//标记视觉数据更新了
		
		//帧计算
		Vision_Time[NOW] = xTaskGetTickCount();
		Vision_Ping = Vision_Time[NOW] - Vision_Time[LAST];//计算时间间隔
		Vision_Time[LAST] = Vision_Time[NOW];
	
}





void Vision_Get_Distance(float *distance)                     //获取距离
{
	*distance = VisionRecvData.distance;
	if(VisionRecvData.distance <0)
	{
		*distance = 0;
	}
}

/**
  * @brief  判断视觉数据更新了吗
  * @param  void
  * @retval TRUE更新了   FALSE没更新
  * @attention  为自瞄做准备,串口空闲中断每触发一次且通过校验,则Vision_Get_New_Data置TRUE
  */
bool Vision_If_Update(void)
{
	return Vision_Get_New_Data;
}

/**
  * @brief  视觉数据更新标志位手动置0(false)
  * @param  void
  * @retval void
  * @attention  记得要清零,在哪清零自己选,调用这个函数就行
  */
void Vision_Clean_Update_Flag(void)
{
	Vision_Get_New_Data = FALSE;
}







