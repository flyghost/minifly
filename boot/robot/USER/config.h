#ifndef __CONFIG_H
#define __CONFIG_H

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly	
 * �����ļ�
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

#define BOOTLOADER_SIZE		(16*1024)	
#define CONFIG_PARAM_SIZE	(16*1024)

#define CONFIG_PARAM_ADDR 	(FLASH_BASE + BOOTLOADER_SIZE)	/*16K bootloader*/
#define FIRMWARE_START_ADDR (FLASH_BASE + BOOTLOADER_SIZE + CONFIG_PARAM_SIZE)	/*16K bootloader+ 16 ģ��eeprom*/


#endif










