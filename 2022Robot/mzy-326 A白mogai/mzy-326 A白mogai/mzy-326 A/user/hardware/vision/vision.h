#ifndef __VISION_H
#define __VISION_H

#include "main.h"

#define    VISION_DATA_ERROR      0          //ÊÓ¾õÊı¾İ´íÎó
#define    VISION_DATA_CORRECT    1          //ÊÓ¾õÊı¾İ´íÎó

#define    VISION_LEN_HEADER      3          //Ö¡Í·³¤?
#define    VISION_LEN_CMDID       1          //ÃüÁîÂë³¤¶È
#define    VISIOV_LEN_TAIL        2	        //Ö¡Î²CRC16

#define    VISION_LEN_DATA        17         //ÓĞĞ§Êı¾İ¶Î³¤¶È
#define    VISION_LEN_PACKED      22         //Êı¾İ°ü³¤¶È

#define    VISION_OFF             (0x00)     //¹Ø±ÕÊÓ¾õ
#define    VISION_RED             (0x01)     //´ò¿ªÊÓ¾õÊ¶±ğºìÉ«
#define    VISION_BULE            (0x02)     //´ò¿ªÊÓ¾õÊ¶±ğÀ¶É«

#define    VISION_BUFF_ANTI       (0x03)     //ÄæÊ±Õë´ò´ó·û
#define    VISION_BUFF_CLOCKWISE  (0x04)     //Ë³Ê±Õë´ò´ó·û
#define    VISION_BUFF_STAND      (0x05)     //´òĞ¡·û

//ÆğÊ¼×Ö½Ú,Ğ­Òé¹Ì¶¨Îª0xA5
#define    VISION_SOF              (0xA5)     //Ö¡Í· ¿É¸ü¸Ä£¿
#define    VISION_WEI              (0xFF)     //Ö¡Î² ¿É¸ü¸Ä 


#define    RGB_LEN_DATA        2         //ÓĞĞ§Êı¾İ¶Î³¤¶È
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

////5×Ö½ÚÖ¡Í·,Æ«ÒÆÎ»ÖÃ
//typedef enum
//{
//	SOF          = 0,//ÆğÊ¼Î»
//	DATA_LENGTH  = 1,//Ö¡ÄÚÊı¾İ³¤¶È,¸ù¾İÕâ¸öÀ´»ñÈ¡Êı¾İ³¤¶È
//	SEQ          = 3,//°üĞòºÅ
//	CRC8         = 4 //CRC8
//	
//}VERSIONHeaderOffset;

//STM32½ÓÊÕ,Ö±½Ó½«´®¿Ú½ÓÊÕµ½µÄÊı¾İ¿½±´½ø½á¹¹Ìå
typedef __packed struct
{
	/* Í· */
	uint8_t   SOF;			//Ö¡Í·ÆğÊ¼Î»,Ôİ¶¨0xA5
	uint8_t   CmdID;		//Ö¸Áî
	uint8_t   CRC8;			//Ö¡Í·CRCĞ£Ñé,±£Ö¤·¢ËÍµÄÖ¸ÁîÊÇÕıÈ·µÄ
	
	/* Êı¾İ */
	float     pitch_angle;
	float     yaw_angle;     
	float     distance;			//¾àÀë
	uint8_t   centre_lock;		//ÊÇ·ñÃé×¼µ½ÁËÖĞ¼ä  0Ã»ÓĞ  1Ãé×¼µ½ÁË
	uint8_t	  identify_target;	//ÊÓÒ°ÄÚÊÇ·ñÓĞÄ¿±ê/ÊÇ·ñÊ¶±ğµ½ÁËÄ¿±ê   0·ñ  1ÊÇ	
	uint8_t   identify_buff;	//´ò·ûÊ±ÊÇ·ñÊ¶±ğµ½ÁËÄ¿±ê£¬1ÊÇ£¬2Ê¶±ğµ½ÇĞ»»ÁË×°¼×£¬0Ã»Ê¶±ğµ½
	
	uint8_t	  blank_b;			//Ô¤Áô
	uint8_t	  auto_too_close;   //Ä¿±ê¾àÀëÌ«½ü,ÊÓ¾õ·¢1£¬·ñÔò·¢0
	
	
	/* Î² */
	uint16_t  CRC16;     //Ö¡Î²ÔÚºó8Î»     
	
}VisionRecvData_t;

typedef __packed struct
{
	/* Í· */
	uint8_t   SOF;			//Ö¡Í·ÆğÊ¼Î»,Ôİ¶¨0xA5
	uint8_t   CmdID;		//Ö¸Áî
	uint8_t   CRC8;			//Ö¡Í·CRCĞ£Ñé,±£Ö¤·¢ËÍµÄÖ¸ÁîÊÇÕıÈ·µÄ
	
	/* Êı¾İ */
	float     pitch_angle[4];
	float     yaw_angle[4];     
	float     distance[4];			//¾àÀë
//	uint8_t   centre_lock;		//ÊÇ·ñÃé×¼µ½ÁËÖĞ¼ä  0Ã»ÓĞ  1Ãé×¼µ½ÁË
//	uint8_t	  identify_target;	//ÊÓÒ°ÄÚÊÇ·ñÓĞÄ¿±ê/ÊÇ·ñÊ¶±ğµ½ÁËÄ¿±ê   0·ñ  1ÊÇ	
//	uint8_t   identify_buff;	//´ò·ûÊ±ÊÇ·ñÊ¶±ğµ½ÁËÄ¿±ê£¬1ÊÇ£¬2Ê¶±ğµ½ÇĞ»»ÁË×°¼×£¬0Ã»Ê¶±ğµ½
//	
//	uint8_t	  blank_b;			//Ô¤Áô
//	uint8_t	  auto_too_close;   //Ä¿±ê¾àÀëÌ«½ü,ÊÓ¾õ·¢1£¬·ñÔò·¢0
	
	
	/* Î² */
	uint16_t  CRC16;     //Ö¡Î²ÔÚºó8Î»     
	
}VisionRecvData_test;

typedef __packed struct
{
	/* Í· */
	uint8_t   SOF;			//Ö¡Í·ÆğÊ¼Î»,Ôİ¶¨0xA5
	uint8_t   CmdID;		//Ö¸Áî
	uint8_t   CRC8;			//Ö¡Í·CRCĞ£Ñé,±£Ö¤·¢ËÍµÄÖ¸ÁîÊÇÕıÈ·µÄ
	
}VisionSendHeader_t;

//STM32·¢ËÍ,Ö±½Ó½«´ò°üºÃµÄÊı¾İÒ»¸ö×Ö½ÚÒ»¸ö×Ö½ÚµØ·¢ËÍ³öÈ¥
typedef struct
{
//	/* Í· */
//	uint8_t   SOF;			//Ö¡Í·ÆğÊ¼Î»,Ôİ¶¨0xA5
//	uint8_t   CmdID;		//Ö¸Áî
//	uint8_t   CRC8;			//Ö¡Í·CRCĞ£Ñé,±£Ö¤·¢ËÍµÄÖ¸ÁîÊÇÕıÈ·µÄ
	
	/* Êı¾İ */
	float     pitch_angle;     //µ±Ç°½Ç¶È
	float     yaw_angle;       //µ±Ç°½Ç¶È                                              (»úĞµ?ÍÓÂİÒÇ?)¿)???????????
//	float     distance;			//¾àÀë
//	uint8_t   lock_sentry;		//ÊÇ·ñÔÚÌ§Í·Ê¶±ğÉÚ±ø
//	uint8_t   base;				//µõÉä
//	
//	uint8_t   blank_a;		//Ô¤Áô
//	uint8_t	  blank_b;
//	uint8_t	  blank_c;	
//	
//	/* Î² */
//	uint16_t  CRC16;
	
}VisionSendData_t;


//¹ØÓÚÈçºÎĞ´ÈëCRCĞ£ÑéÖµ
//ÎÒÃÇ¿ÉÒÔÖ±½ÓÀûÓÃ¹Ù·½¸øµÄCRC´úÂë

//×¢Òâ,CRC8ºÍCRC16ËùÕ¼×Ö½Ú²»Ò»Ñù,8ÎªÒ»¸ö×Ö½Ú,16Îª2¸ö×Ö½Ú

//Ğ´Èë    CRC8 µ÷ÓÃ    Append_CRC8_Check_Sum( param1, param2)
//ÆäÖĞ param1´ú±íĞ´ºÃÁËÖ¡Í·Êı¾İµÄÊı×é(Ö¡Í·ºóµÄÊı¾İ»¹Ã»Ğ´Ã»ÓĞ¹ØÏµ),
//     param2´ú±íCRC8Ğ´ÈëºóÊı¾İ³¤¶È,ÎÒÃÇ¶¨ÒåµÄÊÇÍ·µÄ×îºóÒ»Î»,Ò²¾ÍÊÇ3

//Ğ´Èë    CRC16 µ÷ÓÃ   Append_CRC16_Check_Sum( param3, param4)
//ÆäÖĞ param3´ú±íĞ´ºÃÁË   Ö¡Í· + Êı¾İ  µÄÊı×é(¸úÉÏÃæÊÇÍ¬Ò»¸öÊı×é)
//     param4´ú±íCRC16Ğ´ÈëºóÊı¾İ³¤¶È,ÎÒÃÇ¶¨ÒåµÄÕû¸öÊı¾İ³¤¶ÈÊÇ22,ËùÒÔÊÇ22

/*----------------------------------------------------------*/

//ÃüÁîÂëID,ÓÃÀ´ÅĞ¶Ï½ÓÊÕµÄÊÇÊ²Ã´Êı¾İ

 
void Vision_Read_Data(uint8_t *ReadFormUart7);                 //´ÓÉÏ²ã½ÓÊÕÊı¾İ

void Vision_Send_Data(uint8_t CmdID);                          //·¢ËÍÊı¾İµ½ÉÏ²ã

void Vision_Buff_Error_Angle_Pitch(float *error);              //´ò·ûpitch½Ç¶ÈÎó²î»ñÈ¡
void Vision_Buff_Error_Angle_Yaw(float *error);				   //´ò·ûyaw½Ç¶ÈÎó²î»ñÈ¡

void Vision_Error_Angle_Pitch(float *error);				   //×ÔÃépitch½Ç¶ÈÎó²î»ñÈ¡
void Vision_Error_Angle_Yaw(float *error);					   //×ÔÃéyaw½Ç¶ÈÎó²î»ñÈ¡

void Vision_Get_Distance(float *distance);                     //»ñÈ¡¾àÀë

bool Vision_If_Update(void);
void Vision_Clean_Update_Flag(void);


void RGB_Read_Data(uint8_t *ReadFromUsart);
#endif


