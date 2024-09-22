#include "nv.h"


// //得到ROS设备信息
// void getDeviceInfo(void)
// {
// 	u32 buf;
	
// 	STMFLASH_Read(CONFIG_PARAM_ADDR, (u32 *)&buf, 1);
// 	DeviceInfoBuffer[0] = (u8)buf;  //软件版本 11表示V1.1

// 	for(u8 i=1; i<13; i++)
// 		DeviceInfoBuffer[i]=*(vu32*)(0x1FFF7A10+i); //软件序列号
// }

// //得到REMOTER设备信息
// void GetDeviceInfo(void)
// {
// 	u8 i=0;
// 	u16 buf = 1;
	
// 	STMFLASH_Read(CONFIG_PARAM_ADDR, &buf, 1);
// 	DeviceInfoBuffer[0] = (u8)buf;  //软件版本 11表示V1.1
// 	for(i=1;i<=12;i++)
// 		DeviceInfoBuffer[i]=*(vu32*)(0x1FFFF7E8+i); //软件序列号
// }