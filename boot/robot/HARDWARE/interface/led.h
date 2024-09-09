#ifndef __LED_H
#define __LED_H
#include "sys.h"
#include <stdbool.h>

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly	
 * LED��������
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/


#define LED_BLUE_L		PBout(12)	/*LED_BLUE_L PB12*/
#define LED_GREEN_L		PAout(6)	/*LED_GREEN_L PA6*/
#define LED_RED_L		PAout(7)	/*LED_RED_L PA7*/
#define LED_GREEN_R		PCout(13)	/*LED_GREEN_R PC13*/
#define LED_RED_R		PCout(14)	/*LED_RED_R PC14*/

void ledInit(void);	/* LED��ʼ�� */


#endif
