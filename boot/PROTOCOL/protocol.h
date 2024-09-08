#ifndef  __PROTOCOL_H
#define  __PROTOCOL_H
#include "stm32f4xx.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * ����Э��	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

//����֡����
typedef struct
{
	//���֡����
	#define MAX_FRAME_LENGTH (256+6)
	//��С֡����
	#define MIN_FRAME_LENGTH  5	
	//�豸��ַ
	u8 Device_Address;
	//֡����
	u8 Function_Type;
	//֡����
	u8 Sequence;
	//��Ч���ݳ���
	u8 Data_Length;
	//����
	u8 *Data;
	//У��ֵ
	u16 Checksum;

}TransportProtocol_Typedef;

//������
typedef enum
{
	//֡��ʽ����
	FRAME_FORMAT_ERR = 1,		
	//У��ֵ��ʽ����
	CHECK_FORMAR_ERR = 2,
	//У��ֵ��λ
	CHECK_ERR = 3,
	//����ɹ�
	UPACKED_SUCCESS = 4

}TransportProtocol_Result;

//Э�������
typedef struct
{	
	//����֡
	TransportProtocol_Typedef * TransportProtocol;
	//���յ��ֽ���
	u32  RecieveByteCount;

	//����֡����
	u8* Buf;
	//֡�ܳ���
	u16 FrameTotalLength;
	//�������
	TransportProtocol_Result (*Unpacked)(void);
	//�������
	void (*Packed)(void);
	//У�麯��
	u16 (*Check)(u8 *,u16 len);

}TransportProtocol_Manager_Typedef;

//�ⲿ����Э�������
extern TransportProtocol_Manager_Typedef TransportProtocol_Manager;
//��ʼ������Э��
void  TransportProtocol_Init(TransportProtocol_Typedef *TransportProtocol,u8 *buf,u16 (*check)(u8 *,u16 len));

#endif


