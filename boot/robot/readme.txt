
最新资料下载地址:
	http://www.openedv.com/docs/index.html

MiniFly外形:

         HEAD
	  M4  ↑  M1
	   \     /
		\   /
		 \ /
		 / \
		/   \
	   /     \
	  M3     M2
	
硬件资源:
	1,MCU:STM32F411CEU6 (FLAH:512K, RAM:128K, 系统运行时钟频率:96MHz)
	2,9轴MPU9250连接在IIC1上(IMU_SCL:PB8, IMU_SDA:PB9, 通信方式:模拟IIC) 
	3,气压计BMP280连接在MPU9250的辅助IIC上(AUX_DA,AUX_CL)
	4,无线通信NFR51822连接在UART2上(NRF_RX:PA2, NRF_TX:PA3, NRF_FLOW_CTRL:PA0) 
	5,MOTOR1连接在TIM4_CH2上(PB7)
	6,MOTOR2连接在TIM4_CH1上(PB6)
	7,MOTOR3连接在TIM2_CH3上(PB10)
	8,MOTOR4连接在TIM2_CH1上(PA5)
	9,LED_BLUE_L连接在PB12上	(MORTOR3对应的蓝色LED, 高电平有效)
	10,LED_GREEN_L连接在PA6上	(MORTOR4对应的绿色LED, 低电平有效)
	11,LED_RED_L连接在PA7上		(MORTOR4对应的红色LED, 低电平有效)
	12,LED_GREEN_R连接在PC13上	(MORTOR1对应的绿色LED, 低电平有效)
	13,LED_RED_R连接在PC14上	(MORTOR1对应的红色LED, 低电平有效)
	14,扩展IIC接口(SDA:PB4, SCL:PB5) 
	15,扩展SPI2接口(SCK:PB13, MISO:PB14, MOSI:PB15)  
	16,扩展UART1接口(RX1:PB3, TX1:PA15, 外挂摄像头模块需用此接口)  
	17,扩展GPIO(CS0:PC15, CS1:PB0, CS2:PB1, CS3:PA8). 	
	18,USB_SLAVE接口(USB_ID:PA10, USB_DM:PA11, USB_DP:PA12)

实验现象:
	关机状态下，长按电源键3S左右，等待LED_BLUE_L闪烁，MiniFly进入bootloader，
	插上配送的USB线，打开固件升级软件FirmwarUpgrade.exe,找到对应的串口，
	打开F411的固件Firmware_F411.bin，点击开始，等待固件升级完成。

注意事项:
	代码下载和调试前，请将下载器开关拨到STM32档。
	Bootloader起始地址(BOOTLOADER_ADDR) 0x08000000;
	固件起始地址(FIRMWARE_START_ADDR) 	0x08008000;


固件更新记录:
	Bootloader V1.0 Release(硬件版本:V1.32, DATE:2017-06-30)

	Bootloader V1.1 Release(硬件版本:V1.4, DATE:2021-04-20)
		1.更改下载调试器为ATK Mini-CMSIS-DAP;


	

					正点原子@ALIENTEK
					2017-6-30
					广州市星翼电子科技有限公司
					电话：020-38271790
					传真：020-36773971
					购买：http://shop62103354.taobao.com
					http://shop62057469.taobao.com
					公司网站：www.alientek.com
					技术论坛：www.openedv.com