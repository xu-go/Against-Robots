#ifndef __VISION_H
#define __VISION_H

#include "main.h"

#define    VISION_DATA_ERROR      0          //�Ӿ����ݴ���
#define    VISION_DATA_CORRECT    1          //�Ӿ����ݴ���

#define    VISION_LEN_HEADER      3          //֡ͷ��?
#define    VISION_LEN_CMDID       1          //�����볤��
#define    VISIOV_LEN_TAIL        2	        //֡βCRC16

#define    VISION_LEN_DATA        17         //��Ч���ݶγ���
#define    VISION_LEN_PACKED      22         //���ݰ�����

#define    VISION_OFF             (0x00)     //�ر��Ӿ�
#define    VISION_RED             (0x01)     //���Ӿ�ʶ���ɫ
#define    VISION_BULE            (0x02)     //���Ӿ�ʶ����ɫ

#define    VISION_BUFF_ANTI       (0x03)     //��ʱ�����
#define    VISION_BUFF_CLOCKWISE  (0x04)     //˳ʱ�����
#define    VISION_BUFF_STAND      (0x05)     //��С��

//��ʼ�ֽ�,Э��̶�Ϊ0xA5
#define    VISION_SOF              (0xA5)     //֡ͷ �ɸ��ģ�
#define    VISION_WEI              (0xFF)     //֡β �ɸ��� 


#define    RGB_LEN_DATA        2         //��Ч���ݶγ���
#define    RGB_LEN             2




typedef __packed struct
{
	char  color;	
} xRGBHeader;


#define NOW  0
#define LAST 1


//typedef enum 
//{
//	FRAME_HEADER         = 0,
//	CMD_ID               = 5,
//	DATA                 = 7,
//	
//}VERSIONFrameOffset;

////5�ֽ�֡ͷ,ƫ��λ��
//typedef enum
//{
//	SOF          = 0,//��ʼλ
//	DATA_LENGTH  = 1,//֡�����ݳ���,�����������ȡ���ݳ���
//	SEQ          = 3,//�����
//	CRC8         = 4 //CRC8
//	
//}VERSIONHeaderOffset;

//STM32����,ֱ�ӽ����ڽ��յ������ݿ������ṹ��
typedef __packed struct
{
	/* ͷ */
	uint8_t   SOF;			//֡ͷ��ʼλ,�ݶ�0xA5
	uint8_t   CmdID;		//ָ��
	uint8_t   CRC8;			//֡ͷCRCУ��,��֤���͵�ָ������ȷ��
	
	/* ���� */
	float     pitch_angle;
	float     yaw_angle;     
	float     distance;			//����
	uint8_t   centre_lock;		//�Ƿ���׼�����м�  0û��  1��׼����
	uint8_t	  identify_target;	//��Ұ���Ƿ���Ŀ��/�Ƿ�ʶ����Ŀ��   0��  1��	
	uint8_t   identify_buff;	//���ʱ�Ƿ�ʶ����Ŀ�꣬1�ǣ�2ʶ���л���װ�ף�0ûʶ��
	
	uint8_t	  blank_b;			//Ԥ��
	uint8_t	  auto_too_close;   //Ŀ�����̫��,�Ӿ���1������0
	
	
	/* β */
	uint16_t  CRC16;     //֡β�ں�8λ     
	
}VisionRecvData_t;

typedef __packed struct
{
	/* ͷ */
	uint8_t   SOF;			//֡ͷ��ʼλ,�ݶ�0xA5
	uint8_t   CmdID;		//ָ��
	uint8_t   CRC8;			//֡ͷCRCУ��,��֤���͵�ָ������ȷ��
	
	/* ���� */
	float     pitch_angle[4];
	float     yaw_angle[4];     
	float     distance[4];			//����
//	uint8_t   centre_lock;		//�Ƿ���׼�����м�  0û��  1��׼����
//	uint8_t	  identify_target;	//��Ұ���Ƿ���Ŀ��/�Ƿ�ʶ����Ŀ��   0��  1��	
//	uint8_t   identify_buff;	//���ʱ�Ƿ�ʶ����Ŀ�꣬1�ǣ�2ʶ���л���װ�ף�0ûʶ��
//	
//	uint8_t	  blank_b;			//Ԥ��
//	uint8_t	  auto_too_close;   //Ŀ�����̫��,�Ӿ���1������0
	
	
	/* β */
	uint16_t  CRC16;     //֡β�ں�8λ     
	
}VisionRecvData_test;

typedef __packed struct
{
	/* ͷ */
	uint8_t   SOF;			//֡ͷ��ʼλ,�ݶ�0xA5
	uint8_t   CmdID;		//ָ��
	uint8_t   CRC8;			//֡ͷCRCУ��,��֤���͵�ָ������ȷ��
	
}VisionSendHeader_t;

//STM32����,ֱ�ӽ�����õ�����һ���ֽ�һ���ֽڵط��ͳ�ȥ
typedef struct
{
//	/* ͷ */
//	uint8_t   SOF;			//֡ͷ��ʼλ,�ݶ�0xA5
//	uint8_t   CmdID;		//ָ��
//	uint8_t   CRC8;			//֡ͷCRCУ��,��֤���͵�ָ������ȷ��
	
	/* ���� */
	float     pitch_angle;     //��ǰ�Ƕ�
	float     yaw_angle;       //��ǰ�Ƕ�                                              (��е?������?)�)???????????
//	float     distance;			//����
//	uint8_t   lock_sentry;		//�Ƿ���̧ͷʶ���ڱ�
//	uint8_t   base;				//����
//	
//	uint8_t   blank_a;		//Ԥ��
//	uint8_t	  blank_b;
//	uint8_t	  blank_c;	
//	
//	/* β */
//	uint16_t  CRC16;
	
}VisionSendData_t;


//�������д��CRCУ��ֵ
//���ǿ���ֱ�����ùٷ�����CRC����

//ע��,CRC8��CRC16��ռ�ֽڲ�һ��,8Ϊһ���ֽ�,16Ϊ2���ֽ�

//д��    CRC8 ����    Append_CRC8_Check_Sum( param1, param2)
//���� param1����д����֡ͷ���ݵ�����(֡ͷ������ݻ�ûдû�й�ϵ),
//     param2����CRC8д������ݳ���,���Ƕ������ͷ�����һλ,Ҳ����3

//д��    CRC16 ����   Append_CRC16_Check_Sum( param3, param4)
//���� param3����д����   ֡ͷ + ����  ������(��������ͬһ������)
//     param4����CRC16д������ݳ���,���Ƕ�����������ݳ�����22,������22

/*----------------------------------------------------------*/

//������ID,�����жϽ��յ���ʲô����

 
void Vision_Read_Data(uint8_t *ReadFormUart7);                 //���ϲ��������

void Vision_Send_Data(uint8_t CmdID);                          //�������ݵ��ϲ�

void Vision_Buff_Error_Angle_Pitch(float *error);              //���pitch�Ƕ�����ȡ
void Vision_Buff_Error_Angle_Yaw(float *error);				   //���yaw�Ƕ�����ȡ

void Vision_Error_Angle_Pitch(float *error);				   //����pitch�Ƕ�����ȡ
void Vision_Error_Angle_Yaw(float *error);					   //����yaw�Ƕ�����ȡ

void Vision_Get_Distance(float *distance);                     //��ȡ����

bool Vision_If_Update(void);
void Vision_Clean_Update_Flag(void);


void RGB_Read_Data(uint8_t *ReadFromUsart);
#endif


