#ifndef __CHECK_H
#define __CHECK_H
#include "stm32f4xx.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 校验文件	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

u16 Checksum_Sum(u8* buf,u16 len);	//和校验 所有字节之和为0
u16 Checksum_XOR(u8* buf, u16 len);	//异或校验，所有字节异或
u16 Checksum_CRC8(u8 *buf,u16 len);	//CRC8 校验
u16 Checksum_CRC16(u8 *buf,u16 len);//CRC16 校验

#endif

