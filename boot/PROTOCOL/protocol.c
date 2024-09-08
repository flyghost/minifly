#include "protocol.h"
#include "check.h"
#include "string.h"

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

//����һ��Э�������
TransportProtocol_Manager_Typedef TransportProtocol_Manager;

//���
static TransportProtocol_Result TransportProtocol_Unpacked()
{	
	int check_len=0;    //У���ֽڵĳ���   
	u16 checksum=0;      //���յ���У��ֵ
	u16 checksum_cal=0;  //����õ���У��ֵ
	u8 *checksum_pos=0; //У��ֵ��ƫ��λ��


	//֡��ʽ���󣬷Ƿ�����
	if(TransportProtocol_Manager.RecieveByteCount > MAX_FRAME_LENGTH || TransportProtocol_Manager.RecieveByteCount<MIN_FRAME_LENGTH)
	{
		return FRAME_FORMAT_ERR;
	}
	//�õ���Ч���ݳ���
	TransportProtocol_Manager.TransportProtocol->Data_Length = TransportProtocol_Manager.Buf[3];
	//У���ֽڵĳ���=���յ�֡����-4���ֽڵ�ͷ-��Ч���ݵĳ���
	check_len = TransportProtocol_Manager.RecieveByteCount - 4 - TransportProtocol_Manager.TransportProtocol->Data_Length;
	//У��ֵ��ʽ���� У���ֽڸ�������
	if(check_len<1 || check_len>2)
	{
		return CHECK_FORMAR_ERR;
	}
	//�õ�У��ֵ��ƫ�Ƶ�ַ
	checksum_pos = (u8 *)(4+TransportProtocol_Manager.TransportProtocol->Data_Length +TransportProtocol_Manager.Buf);
	if(check_len==1)
	{
		checksum = (*checksum_pos)&0xff;
	}else if(check_len==2)
	{
		checksum  = (*checksum_pos)&0xff;
		checksum_pos++;
		checksum <<=8;
		checksum |= *checksum_pos;
	}
	//���� ͷ�����ݵ�У��ֵ
	checksum_cal = TransportProtocol_Manager.Check(TransportProtocol_Manager.Buf,4+TransportProtocol_Manager.TransportProtocol->Data_Length);
	//У�����
	if(checksum!=checksum_cal)
	{
		return CHECK_ERR;
	}
	
	//�豸��ַ
	TransportProtocol_Manager.TransportProtocol->Device_Address = TransportProtocol_Manager.Buf[0];
	//֡����
	TransportProtocol_Manager.TransportProtocol->Function_Type = TransportProtocol_Manager.Buf[1];
	//֡���� ��ǰ�ڼ�֡
	TransportProtocol_Manager.TransportProtocol->Sequence = TransportProtocol_Manager.Buf[2];
	//֡��Ч���ݳ��� (ǰ���ѻ�ȡ)
	
	//֡����
	TransportProtocol_Manager.TransportProtocol->Data = &TransportProtocol_Manager.Buf[4];
	
	//֡У��ֵ
	TransportProtocol_Manager.TransportProtocol->Checksum = checksum_cal;
	
	
	return UPACKED_SUCCESS;  
}


//���
static void TransportProtocol_Packed()
{
	u16 checksum=0;
	u8 *check_pos;  //У��ֵ��ƫ��λ��
	
	//�豸��ַ
	TransportProtocol_Manager.Buf[0] = TransportProtocol_Manager.TransportProtocol->Device_Address;
	//֡����
	TransportProtocol_Manager.Buf[1] = TransportProtocol_Manager.TransportProtocol->Function_Type;
	//֡����
	TransportProtocol_Manager.Buf[2] = TransportProtocol_Manager.TransportProtocol->Sequence;
	//֡��Ч���ݴ�С
	TransportProtocol_Manager.Buf[3] = TransportProtocol_Manager.TransportProtocol->Data_Length;
	//֡����
	memcpy(&TransportProtocol_Manager.Buf[4],TransportProtocol_Manager.TransportProtocol->Data,TransportProtocol_Manager.TransportProtocol->Data_Length);
	//����У��ֵ
	checksum = TransportProtocol_Manager.Check(TransportProtocol_Manager.Buf,4+TransportProtocol_Manager.TransportProtocol->Data_Length);
	//У��ֵ
	check_pos = TransportProtocol_Manager.Buf+4+TransportProtocol_Manager.TransportProtocol->Data_Length;
	
	if(checksum<256)
	{ 
		(*check_pos) = checksum&0xff;
		//��¼֡�ܳ���
		TransportProtocol_Manager.FrameTotalLength = 4+TransportProtocol_Manager.TransportProtocol->Data_Length+1;
	}else  //CRC16
	{	
		(*check_pos) = (u8)(checksum>>8)&0xff;  //���ֽ�
		check_pos++;
		(*check_pos) = (u8)(checksum&0xff);		//���ֽ�
		//��¼֡�ܳ���
		TransportProtocol_Manager.FrameTotalLength = 4+TransportProtocol_Manager.TransportProtocol->Data_Length+2;
	}	
}


//��ʼ������Э��
//TransportProtocol������֡
//buf:�շ�������
//check:У�鷽ʽ
//Checksum_Sum:��У��
//Checksum_XOR�����У��
//Checksum_CRC8:CRC8У��
//Checksum_CRC16:CRC16У��

void  TransportProtocol_Init(TransportProtocol_Typedef *TransportProtocol,u8 *buf,u16 (*check)(u8 *,u16 len))
{	
	//Э���
	TransportProtocol_Manager.TransportProtocol = TransportProtocol;
	//����Э�黺�� 
	TransportProtocol_Manager.Buf = buf;
	//ѡ��У�鷽ʽ 
	TransportProtocol_Manager.Check = check;
	//�������
	TransportProtocol_Manager.Packed = TransportProtocol_Packed;
	//�������
	TransportProtocol_Manager.Unpacked = TransportProtocol_Unpacked;	
}



