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
#include "byte2string.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * main.c	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

extern uint16_t usb_rx_ptr_in;
extern uint8_t usb_rx_buf[];




#define IAP_BUFFER_SIZE 2048
#define DEVICE_INFO_BUFFER_SIZE   (sizeof(DeviceInfoBuffer)/sizeof(u8)) 

u8 RecieveBuf[10];//转换字符串的buffer

u8 DeviceInfoBuffer[13];//设备信息缓存

u8 ipap_buf[IAP_BUFFER_SIZE];//iap 缓存

TransportProtocol_Typedef TransportProtocol;//定义一帧协议


/*判断是否进行固件升级*/
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

//得到设备信息
void getDeviceInfo(uint8_t *buffer, uint8_t len)
{
    uint32_t buf;

    STMFLASH_Read(CONFIG_PARAM_ADDR, (uint32_t *)&buf, 1);
    buffer[0] = (uint8_t)buf; //软件版本 11表示V1.1

    for (uint8_t i = 1; i < len; i++)
        buffer[i] = *(volatile uint32_t *)(0x1FFF7A10 + i); //软件序列号
}


extern void usbsendData(uint8_t* buf, uint32_t len);

//用来响应上位机
static void IAP_Response()
{
	TransportProtocol.Device_Address = 0x01;	//设备地址
	TransportProtocol.Sequence = TransportProtocol.Sequence;	//帧序列 和收到的一致，这里不改变
	TransportProtocol_Manager.Packed();			//打包	

	usbsendData(TransportProtocol_Manager.Buf, TransportProtocol_Manager.FrameTotalLength);	
}

int main()
{
	u16 t = 0;
	u32 timeOut = 0;
	u32 revcnt = 0;
	u16 oldcount = 0;
	u8 isTransportOK = 0;   //标记代码是否传输完毕
	TransportProtocol_Result res;	//传输结果
	u32 flashNowPos = FIRMWARE_START_ADDR;//flash的当前位置
	
	isUpgradeFirmware(); 	/*判断是否要固件升级*/
	
	delay_init(96);
	ledInit();
	usbd_cdc_vcp_Init();
	
	//初始化传输协议  指向串口接收缓冲区  选择sum的校验方式
	TransportProtocol_Init(&TransportProtocol, usb_rx_buf, Checksum_Sum);
	
	while(1) 
	{	
		if(usb_rx_ptr_in)
		{
			if(oldcount == usb_rx_ptr_in)//新周期内,没有收到任何数据,认为本次数据接收完成.
			{				
				TransportProtocol_Manager.RecieveByteCount = usb_rx_ptr_in;//获取接收的总字节数			
				res = TransportProtocol_Manager.Unpacked();//解包
				
				if(res!=UPACKED_SUCCESS)  //解包失败的时候，不响应上位机，上位机会自动重发
				{	
					OneByteToStr(res,RecieveBuf);
				}else  //解包成功  每接收到2K字节则写入一次FLASH 所以上位机最大有效数据长度最好和2K成倍数关系
				{	
					if(TransportProtocol.Function_Type==0x01)  //该帧为发送的文件数据
					{	
						if(TransportProtocol.Data_Length==0)
						{	
							isTransportOK = 1;   //代码传输完毕

							iap_write_appbin(flashNowPos,ipap_buf,revcnt);//更新FLASH代码 
							revcnt = 0;

							flashNowPos=FIRMWARE_START_ADDR;  //恢复到原来的起始地址
						}else
						{
							if(isTransportOK==0)  //可以将代码写入falsh
							{
								memcpy(ipap_buf+revcnt,TransportProtocol.Data,TransportProtocol.Data_Length);
								revcnt += TransportProtocol.Data_Length;
								if(revcnt>=IAP_BUFFER_SIZE)
								{	
									revcnt =0;
									iap_write_appbin(flashNowPos,ipap_buf,IAP_BUFFER_SIZE);//更新FLASH代码   
									flashNowPos += IAP_BUFFER_SIZE;
								}
							}
						}	
						TransportProtocol.Data_Length = 0; 		//有效数据大小
						TransportProtocol.Data = 0;				//要发送的数据       
						TransportProtocol.Function_Type = 0x01;	//帧功能			

					}else if(TransportProtocol.Function_Type==0x05) //该帧为查询设备信息的功能
					{	
						getDeviceInfo(DeviceInfoBuffer, DEVICE_INFO_BUFFER_SIZE);
						TransportProtocol.Data_Length = DEVICE_INFO_BUFFER_SIZE;       //有效数据大小
						TransportProtocol.Data = (u8*)DeviceInfoBuffer;	            //要发送的设备信息   
						TransportProtocol.Function_Type = 0x05;				             //帧功能			           
					}	
					IAP_Response();  //响应上位机						
				}	

				usb_rx_ptr_in=0;								
			}else 
			{
				oldcount = usb_rx_ptr_in;	
			}
			timeOut = 0;	//超时清零 
			
		}else
		{
			delay_ms(60);
			if(isTransportOK == 1 || timeOut++ > 1000)	/*60S 超时退出bootloader*/
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



