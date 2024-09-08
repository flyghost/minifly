#include <string.h>
#include "sys.h"
#include "delay.h"
#include "config.h"
#include "led.h"
#include "iap.h"
#include "protocol.h"
#include "check.h"
#include "stmflash.h"
#include "usbd_cdc_vcp.h"

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

extern uint16_t usb_rx_ptr_in;
extern uint8_t usb_rx_buf[];

const u8 HexTable[] ={
'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'
};


#define IAP_BUFFER_SIZE 2048
#define DEVICE_INFO_BUFFER_SIZE   (sizeof(DeviceInfoBuffer)/sizeof(u8)) 

u8 RecieveBuf[10];//ת���ַ�����buffer

u8 DeviceInfoBuffer[13];//�豸��Ϣ����

u8 ipap_buf[IAP_BUFFER_SIZE];//iap ����

TransportProtocol_Typedef TransportProtocol;//����һ֡Э��


/*�ж��Ƿ���й̼�����*/
void isUpgradeFirmware(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1ENR_GPIOAEN, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;	/*PA0: NRF_FC  */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/*PA0==Bit_RESET: Firmware  PA0==Bit_SET Bootloader*/
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == Bit_RESET) 	
	{
		iap_load_app(FIRMWARE_START_ADDR);
	}
}

//�õ��豸��Ϣ
void getDeviceInfo(void)
{
	u32 buf;
	
	STMFLASH_Read(CONFIG_PARAM_ADDR, (u32 *)&buf, 1);
	DeviceInfoBuffer[0] = (u8)buf;  //����汾 11��ʾV1.1

	for(u8 i=1; i<13; i++)
		DeviceInfoBuffer[i]=*(vu32*)(0x1FFF7A10+i); //������к�
}


//һ���ֽ�ת��Ϊ16�����ַ���ʽ���ַ���
void  OneByteToStr(u8 byte,u8* str)
{
	*str = HexTable[byte/16];
	str++;
	*str = HexTable[byte%16];	
	str++;
	*str = 0;
}

//����ֽ�ת��Ϊ16�����ַ���ʽ���ַ������Կո����
void MultiByteToStr(u8 *byteBuf,u16 len,u8 *str)
{
	while(len--)
	{
		*str = HexTable[(*byteBuf)/16];
		str++;
		*str = HexTable[(*byteBuf)%16];
		str++;
		*str = ' ';
		str++;
		byteBuf++;
	}
	*str = 0;
}

extern void usbIapResponse(uint8_t* buf, uint32_t len);

//������Ӧ��λ��
void iapResponse()
{
	TransportProtocol.Device_Address = 0x01;	//�豸��ַ
	TransportProtocol.Sequence = TransportProtocol.Sequence;	//֡���� ���յ���һ�£����ﲻ�ı�
	TransportProtocol_Manager.Packed();			//���	

	usbIapResponse(TransportProtocol_Manager.Buf, TransportProtocol_Manager.FrameTotalLength);	
}

int main()
{
	u16 t = 0;
	u32 timeOut = 0;
	u32 revcnt = 0;
	u16 oldcount = 0;
	u8 isTransportOK = 0;   //��Ǵ����Ƿ������
	TransportProtocol_Result res;	//������
	u32 flashNowPos = FIRMWARE_START_ADDR;//flash�ĵ�ǰλ��
	
	isUpgradeFirmware(); 	/*�ж��Ƿ�Ҫ�̼�����*/
	
	delay_init(96);
	ledInit();
	usbd_cdc_vcp_Init();
	
	//��ʼ������Э��  ָ�򴮿ڽ��ջ�����  ѡ��sum��У�鷽ʽ
	TransportProtocol_Init(&TransportProtocol, usb_rx_buf, Checksum_Sum);
	
	while(1) 
	{	
		if(usb_rx_ptr_in)
		{
			if(oldcount == usb_rx_ptr_in)//��������,û���յ��κ�����,��Ϊ�������ݽ������.
			{				
				TransportProtocol_Manager.RecieveByteCount = usb_rx_ptr_in;//��ȡ���յ����ֽ���			
				res = TransportProtocol_Manager.Unpacked();//���
				
				if(res!=UPACKED_SUCCESS)  //���ʧ�ܵ�ʱ�򣬲���Ӧ��λ������λ�����Զ��ط�
				{	
					OneByteToStr(res,RecieveBuf);
				}else  //����ɹ�  ÿ���յ�2K�ֽ���д��һ��FLASH ������λ�������Ч���ݳ�����ú�2K�ɱ�����ϵ
				{	
					if(TransportProtocol.Function_Type==0x01)  //��֡Ϊ���͵��ļ�����
					{	
						if(TransportProtocol.Data_Length==0)
						{	
							isTransportOK = 1;   //���봫�����

							iap_write_appbin(flashNowPos,ipap_buf,revcnt);//����FLASH���� 
							revcnt = 0;

							flashNowPos=FIRMWARE_START_ADDR;  //�ָ���ԭ������ʼ��ַ
						}else
						{
							if(isTransportOK==0)  //���Խ�����д��falsh
							{
								memcpy(ipap_buf+revcnt,TransportProtocol.Data,TransportProtocol.Data_Length);
								revcnt += TransportProtocol.Data_Length;
								if(revcnt>=IAP_BUFFER_SIZE)
								{	
									revcnt =0;
									iap_write_appbin(flashNowPos,ipap_buf,IAP_BUFFER_SIZE);//����FLASH����   
									flashNowPos += IAP_BUFFER_SIZE;
								}
							}
						}	
						TransportProtocol.Data_Length = 0; 		//��Ч���ݴ�С
						TransportProtocol.Data = 0;				//Ҫ���͵�����       
						TransportProtocol.Function_Type = 0x01;	//֡����			

					}else if(TransportProtocol.Function_Type==0x05) //��֡Ϊ��ѯ�豸��Ϣ�Ĺ���
					{	
						getDeviceInfo();
						TransportProtocol.Data_Length = DEVICE_INFO_BUFFER_SIZE;       //��Ч���ݴ�С
						TransportProtocol.Data = (u8*)DeviceInfoBuffer;	            //Ҫ���͵��豸��Ϣ   
						TransportProtocol.Function_Type = 0x05;				             //֡����			           
					}	
					iapResponse();  //��Ӧ��λ��						
				}	

				usb_rx_ptr_in=0;								
			}else 
			{
				oldcount = usb_rx_ptr_in;	
			}
			timeOut = 0;	//��ʱ���� 
			
		}else
		{
			delay_ms(60);
			if(isTransportOK == 1 || timeOut++ > 1000)	/*60S ��ʱ�˳�bootloader*/
			{
				timeOut = 0;
				iap_load_app(FIRMWARE_START_ADDR);
			}			
			if(++t >= 10)
			{
				t = 0;
				LED_BLUE_L = !LED_BLUE_L;
			}							
		}		
	}
}



