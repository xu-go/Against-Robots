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
float Vision_Comps_Pitch = 0;//�̶���������С�����Ӱ��
float Vision_Comps_Pitch_Dist = 0;//���ݾ��벹��





//�Ӿ��Ƿ���������,FALSEû��,TRUE�����µ�
uint8_t Vision_Get_New_Data = FALSE;

typedef enum
{
	VISION_MANU =0,
	VISION_BUFF =1,
	VISION_AUTO =2,
}VisionActData_t;

VisionSendHeader_t VisionSendHeader;            //��������֡ͷ

VisionRecvData_t VisionRecvData;                //�������ݽṹ��

VisionSendData_t VisionSendData;                //�������ݽṹ��

VisionRecvData_test VisionRecvDataTest;

xRGBHeader RGBHeader;

uint8_t Vision_New_Data = FALSE;



/**
  * @brief  ��ȡ�Ӿ���Ϣ
  * @param  uart7��������
  * @retval void
  * @attention  IRQִ��
  */
float b=0.0;
float *a;
uint8_t Vision_Time[2] = {0};            //��ǰ���ݺ���һ������
uint8_t Vision_Ping = 0;                 //����ʱ����
void Vision_Read_Data(uint8_t *ReadFormUart7)
{
	//�ж�֡ͷ�����Ƿ�Ϊ0xA5
	if(ReadFormUart7[0] == VISION_SOF)
	{
		//֡ͷCRC8У��
		if(Verify_CRC8_Check_Sum( ReadFormUart7, VISION_LEN_HEADER ) == TRUE)
		{
			//֡βCRC16У��
			if(Verify_CRC16_Check_Sum( ReadFormUart7, VISION_LEN_PACKED ) == TRUE)
			{
				//�������ݿ���
				*a=b;
				memcpy( &VisionRecvData, ReadFormUart7, VISION_LEN_PACKED);	
				Vision_New_Data = TRUE;//����Ӿ����ݸ�����
				
				//֡����
				Vision_Time[NOW] = xTaskGetTickCount();
				Vision_Ping = Vision_Time[NOW] - Vision_Time[LAST];//����ʱ����
				Vision_Time[LAST] = Vision_Time[NOW];
			 
		  }
		}
	}
}

/**
  * @brief  �����Ӿ�ָ��
  * @param  CmdID
  * @retval void
  * @attention  ��Э���������ݷ���
  *				CmdID   0x00   �ر��Ӿ�
  *				CmdID   0x01   ʶ���ɫװ��
  *				CmdID   0x02   ʶ����ɫװ��
  *				CmdID   0x03   С��
  *				CmdID   0x04   ���
  */
uint8_t vision_send_pack[50] = {0};//����17����
void Vision_Send_Data(uint8_t CmdID)
{
	int i=0;
	
	VisionSendHeader.SOF = VISION_SOF;
	VisionSendHeader.CmdID = CmdID;  //ң�ػ��߼��̿���ģʽ
	
	memcpy(vision_send_pack,&VisionSendHeader,VISION_LEN_HEADER);  //д��֡ͷ
	
	Append_CRC8_Check_Sum(vision_send_pack,VISION_LEN_HEADER);     //crc8У��
	
	
	
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
	 
	 
		Vision_New_Data = TRUE;//����Ӿ����ݸ�����
		
		//֡����
		Vision_Time[NOW] = xTaskGetTickCount();
		Vision_Ping = Vision_Time[NOW] - Vision_Time[LAST];//����ʱ����
		Vision_Time[LAST] = Vision_Time[NOW];
	
}





void Vision_Get_Distance(float *distance)                     //��ȡ����
{
	*distance = VisionRecvData.distance;
	if(VisionRecvData.distance <0)
	{
		*distance = 0;
	}
}

/**
  * @brief  �ж��Ӿ����ݸ�������
  * @param  void
  * @retval TRUE������   FALSEû����
  * @attention  Ϊ������׼��,���ڿ����ж�ÿ����һ����ͨ��У��,��Vision_Get_New_Data��TRUE
  */
bool Vision_If_Update(void)
{
	return Vision_Get_New_Data;
}

/**
  * @brief  �Ӿ����ݸ��±�־λ�ֶ���0(false)
  * @param  void
  * @retval void
  * @attention  �ǵ�Ҫ����,���������Լ�ѡ,���������������
  */
void Vision_Clean_Update_Flag(void)
{
	Vision_Get_New_Data = FALSE;
}







