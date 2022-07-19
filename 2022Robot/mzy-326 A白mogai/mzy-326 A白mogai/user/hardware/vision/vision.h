#ifndef __VISION_H
#define __VISION_H

#include "main.h"

#define    VISION_DATA_ERROR      0          //视觉数据错误
#define    VISION_DATA_CORRECT    1          //视觉数据错误

#define    VISION_LEN_HEADER      3          //帧头长?
#define    VISION_LEN_CMDID       1          //命令码长度
#define    VISIOV_LEN_TAIL        2	        //帧尾CRC16

#define    VISION_LEN_DATA        17         //有效数据段长度
#define    VISION_LEN_PACKED      22         //数据包长度

#define    VISION_OFF             (0x00)     //关闭视觉
#define    VISION_RED             (0x01)     //打开视觉识别红色
#define    VISION_BULE            (0x02)     //打开视觉识别蓝色

#define    VISION_BUFF_ANTI       (0x03)     //逆时针打大符
#define    VISION_BUFF_CLOCKWISE  (0x04)     //顺时针打大符
#define    VISION_BUFF_STAND      (0x05)     //打小符

//起始字节,协议固定为0xA5
#define    VISION_SOF              (0xA5)     //帧头 可更改？
#define    VISION_WEI              (0xFF)     //帧尾 可更改 


#define    RGB_LEN_DATA        2         //有效数据段长度
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

////5字节帧头,偏移位置
//typedef enum
//{
//	SOF          = 0,//起始位
//	DATA_LENGTH  = 1,//帧内数据长度,根据这个来获取数据长度
//	SEQ          = 3,//包序号
//	CRC8         = 4 //CRC8
//	
//}VERSIONHeaderOffset;

//STM32接收,直接将串口接收到的数据拷贝进结构体
typedef __packed struct
{
	/* 头 */
	uint8_t   SOF;			//帧头起始位,暂定0xA5
	uint8_t   CmdID;		//指令
	uint8_t   CRC8;			//帧头CRC校验,保证发送的指令是正确的
	
	/* 数据 */
	float     pitch_angle;
	float     yaw_angle;     
	float     distance;			//距离
	uint8_t   centre_lock;		//是否瞄准到了中间  0没有  1瞄准到了
	uint8_t	  identify_target;	//视野内是否有目标/是否识别到了目标   0否  1是	
	uint8_t   identify_buff;	//打符时是否识别到了目标，1是，2识别到切换了装甲，0没识别到
	
	uint8_t	  blank_b;			//预留
	uint8_t	  auto_too_close;   //目标距离太近,视觉发1，否则发0
	
	
	/* 尾 */
	uint16_t  CRC16;     //帧尾在后8位     
	
}VisionRecvData_t;

typedef __packed struct
{
	/* 头 */
	uint8_t   SOF;			//帧头起始位,暂定0xA5
	uint8_t   CmdID;		//指令
	uint8_t   CRC8;			//帧头CRC校验,保证发送的指令是正确的
	
	/* 数据 */
	float     pitch_angle[4];
	float     yaw_angle[4];     
	float     distance[4];			//距离
//	uint8_t   centre_lock;		//是否瞄准到了中间  0没有  1瞄准到了
//	uint8_t	  identify_target;	//视野内是否有目标/是否识别到了目标   0否  1是	
//	uint8_t   identify_buff;	//打符时是否识别到了目标，1是，2识别到切换了装甲，0没识别到
//	
//	uint8_t	  blank_b;			//预留
//	uint8_t	  auto_too_close;   //目标距离太近,视觉发1，否则发0
	
	
	/* 尾 */
	uint16_t  CRC16;     //帧尾在后8位     
	
}VisionRecvData_test;

typedef __packed struct
{
	/* 头 */
	uint8_t   SOF;			//帧头起始位,暂定0xA5
	uint8_t   CmdID;		//指令
	uint8_t   CRC8;			//帧头CRC校验,保证发送的指令是正确的
	
}VisionSendHeader_t;

//STM32发送,直接将打包好的数据一个字节一个字节地发送出去
typedef struct
{
//	/* 头 */
//	uint8_t   SOF;			//帧头起始位,暂定0xA5
//	uint8_t   CmdID;		//指令
//	uint8_t   CRC8;			//帧头CRC校验,保证发送的指令是正确的
	
	/* 数据 */
	float     pitch_angle;     //当前角度
	float     yaw_angle;       //当前角度                                              (机械?陀螺仪?)�)???????????
//	float     distance;			//距离
//	uint8_t   lock_sentry;		//是否在抬头识别哨兵
//	uint8_t   base;				//吊射
//	
//	uint8_t   blank_a;		//预留
//	uint8_t	  blank_b;
//	uint8_t	  blank_c;	
//	
//	/* 尾 */
//	uint16_t  CRC16;
	
}VisionSendData_t;


//关于如何写入CRC校验值
//我们可以直接利用官方给的CRC代码

//注意,CRC8和CRC16所占字节不一样,8为一个字节,16为2个字节

//写入    CRC8 调用    Append_CRC8_Check_Sum( param1, param2)
//其中 param1代表写好了帧头数据的数组(帧头后的数据还没写没有关系),
//     param2代表CRC8写入后数据长度,我们定义的是头的最后一位,也就是3

//写入    CRC16 调用   Append_CRC16_Check_Sum( param3, param4)
//其中 param3代表写好了   帧头 + 数据  的数组(跟上面是同一个数组)
//     param4代表CRC16写入后数据长度,我们定义的整个数据长度是22,所以是22

/*----------------------------------------------------------*/

//命令码ID,用来判断接收的是什么数据

 
void Vision_Read_Data(uint8_t *ReadFormUart7);                 //从上层接收数据

void Vision_Send_Data(uint8_t CmdID);                          //发送数据到上层

void Vision_Buff_Error_Angle_Pitch(float *error);              //打符pitch角度误差获取
void Vision_Buff_Error_Angle_Yaw(float *error);				   //打符yaw角度误差获取

void Vision_Error_Angle_Pitch(float *error);				   //自瞄pitch角度误差获取
void Vision_Error_Angle_Yaw(float *error);					   //自瞄yaw角度误差获取

void Vision_Get_Distance(float *distance);                     //获取距离

bool Vision_If_Update(void);
void Vision_Clean_Update_Flag(void);


void RGB_Read_Data(uint8_t *ReadFromUsart);
#endif


