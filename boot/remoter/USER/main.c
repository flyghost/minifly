#include <string.h>
#include "sys.h"
#include "delay.h"
#include "config.h"
#include "led.h"
#include "iap.h"
#include "protocol.h"
#include "check.h"
#include "stmflash.h"
#include "hw_config.h"
#include "key.h"
#include "byte2string.h"
/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * main.c	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

extern _usb_usart_fifo uu_rxfifo;



//�豸��Ϣ����
u8 DeviceInfoBuffer[13];

#define DEVICE_INFO_BUFFER_SIZE   (sizeof(DeviceInfoBuffer)/sizeof(u8)) 

//ת���ַ�����buffer
u8 RecieveBuf[10];

#define IAP_BUFFER_SIZE 2048
//iap ����
u8 ipap_buf[IAP_BUFFER_SIZE];

//����һ֡Э��
TransportProtocol_Typedef TransportProtocol;


//�жϽ��й̼�����������תAPP
void isUpgradeFirmware(void)
{
	if(READ_KEY_L() == 0)
	{
		delay_ms(1500);
		if(READ_KEY_L() != 0)
		{
			iap_load_app(FIRMWARE_START_ADDR);	
		}
	}
	else
	{
		iap_load_app(FIRMWARE_START_ADDR);
	}
	LED_BLUE = 0;
}

//�õ��豸��Ϣ
void GetDeviceInfo(uint8_t *buffer, uint8_t len)
{
    uint8_t  i   = 0;
    uint16_t buf = 1;

    STMFLASH_Read(CONFIG_PARAM_ADDR, &buf, 1);
    buffer[0] = (uint8_t)buf;                  //����汾 11��ʾV1.1
    for (i = 1; i < len; i++)
        buffer[i] = *(volatile uint32_t *)(0x1FFFF7E8 + i); //������к�
}


//extern void usbsendData(uint8_t* buf, uint32_t len);

//������Ӧ��λ��
static void IAP_Response()
{
	TransportProtocol.Device_Address = 0x01;	//�豸��ַ
	TransportProtocol.Sequence = TransportProtocol.Sequence;	//֡���� ���յ���һ�£����ﲻ�ı�
	TransportProtocol_Manager.Packed();			//���	

	usbsendData(TransportProtocol_Manager.Buf, TransportProtocol_Manager.FrameTotalLength);	
}

int main()
{
	u16 t = 0;
	u32 timeOut = 0;
	u32  revcnt = 0;
	u16 oldcount = 0;
	u8 IsTransportOK = 0;   //��Ǵ����Ƿ������
	TransportProtocol_Result res;	//������
	u32 Flash_App_Pos = FIRMWARE_START_ADDR;//������¼������µĵ�ַ
	
	ledInit();
	keyInit();
	delay_init();
	isUpgradeFirmware(); 	/*�ж��Ƿ�Ҫ�̼�����*/
	usb_vcp_init();
	
	//��ʼ������Э��  ָ�򴮿ڽ��ջ�����  ѡ��sum��У�鷽ʽ
	TransportProtocol_Init(&TransportProtocol, uu_rxfifo.buffer, Checksum_Sum);
	
	while(1) 
	{	
		if(uu_rxfifo.writeptr)
		{
			if(oldcount == uu_rxfifo.writeptr)//��������,û���յ��κ�����,��Ϊ�������ݽ������.
			{
				//��ȡ���յ����ֽ���
				TransportProtocol_Manager.RecieveByteCount = uu_rxfifo.writeptr;
				//���
				res = TransportProtocol_Manager.Unpacked();
				if(res!=UPACKED_SUCCESS)  //���ʧ�ܵ�ʱ�򣬲���Ӧ��λ������λ�����Զ��ط�
				{	
					OneByteToStr(res,RecieveBuf);
				}
				else  //����ɹ�  ÿ���յ�2K�ֽ���д��һ��FLASH ������λ�������Ч���ݳ�����ú�2K�ɱ�����ϵ
				{	
					if(TransportProtocol.Function_Type==0x01)  //��֡Ϊ���͵��ļ�����
					{	
						if(TransportProtocol.Data_Length==0)
						{	
							IsTransportOK = 1;   //���봫�����

							iap_write_appbin(Flash_App_Pos,ipap_buf,revcnt);//����FLASH���� 
							revcnt = 0;

							Flash_App_Pos=FIRMWARE_START_ADDR;  //�ָ���ԭ������ʼ��ַ
						}else
						{
							if(IsTransportOK==0)  //���Խ�����д��falsh
							{
								memcpy(ipap_buf+revcnt,TransportProtocol.Data,TransportProtocol.Data_Length);
								revcnt += TransportProtocol.Data_Length;
								if(revcnt>=IAP_BUFFER_SIZE)
								{	
									revcnt =0;
									iap_write_appbin(Flash_App_Pos,ipap_buf,IAP_BUFFER_SIZE);//����FLASH����   
									Flash_App_Pos += IAP_BUFFER_SIZE;
								}
							}
						}	
						TransportProtocol.Data_Length = 0;            //��Ч���ݴ�С
						TransportProtocol.Data = 0;			     	      //Ҫ���͵�����       
						TransportProtocol.Function_Type = 0x01;				//֡����			

					}else if(TransportProtocol.Function_Type==0x05) //��֡Ϊ��ѯ�豸��Ϣ�Ĺ���
					{	
						GetDeviceInfo(DeviceInfoBuffer, DEVICE_INFO_BUFFER_SIZE);
						TransportProtocol.Data_Length = DEVICE_INFO_BUFFER_SIZE;       //��Ч���ݴ�С
						TransportProtocol.Data = (u8*)DeviceInfoBuffer;	            //Ҫ���͵��豸��Ϣ   
						TransportProtocol.Function_Type = 0x05;				             //֡����			           
					}	
					IAP_Response();  //��Ӧ��λ��						
				}	

				uu_rxfifo.writeptr=0;								
			}else 
			{
				oldcount = uu_rxfifo.writeptr;	
			}
			timeOut = 0;	//��ʱ���� 
			
		}else
		{
			delay_ms(60);
			if(IsTransportOK == 1 || timeOut++ > 1000)	/*60S ��ʱ�˳�bootloader*/
			{
				timeOut = 0;
				iap_load_app(FIRMWARE_START_ADDR);
			}			
			if(++t >= 10)
			{
				t = 0;
				LED_BLUE = !LED_BLUE;
			}							
		}		
	}
}
