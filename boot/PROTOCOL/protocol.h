#ifndef  __PROTOCOL_H
#define  __PROTOCOL_H
#include "stm32f4xx.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 传输协议	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

//传输帧对象
typedef struct
{
	//最大帧长度
	#define MAX_FRAME_LENGTH (256+6)
	//最小帧长度
	#define MIN_FRAME_LENGTH  5	
	//设备地址
	u8 Device_Address;
	//帧功能
	u8 Function_Type;
	//帧序列
	u8 Sequence;
	//有效数据长度
	u8 Data_Length;
	//数据
	u8 *Data;
	//校验值
	u16 Checksum;

}TransportProtocol_Typedef;

//传输结果
typedef enum
{
	//帧格式错误
	FRAME_FORMAT_ERR = 1,		
	//校验值格式错误
	CHECK_FORMAR_ERR = 2,
	//校验值错位
	CHECK_ERR = 3,
	//解包成功
	UPACKED_SUCCESS = 4

}TransportProtocol_Result;

//协议管理器
typedef struct
{	
	//传输帧
	TransportProtocol_Typedef * TransportProtocol;
	//接收的字节数
	u32  RecieveByteCount;

	//传输帧缓存
	u8* Buf;
	//帧总长度
	u16 FrameTotalLength;
	//解包函数
	TransportProtocol_Result (*Unpacked)(void);
	//打包函数
	void (*Packed)(void);
	//校验函数
	u16 (*Check)(u8 *,u16 len);

}TransportProtocol_Manager_Typedef;

//外部声明协议管理器
extern TransportProtocol_Manager_Typedef TransportProtocol_Manager;
//初始化传输协议
void  TransportProtocol_Init(TransportProtocol_Typedef *TransportProtocol,u8 *buf,u16 (*check)(u8 *,u16 len));

#endif


