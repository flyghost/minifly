#ifndef __LED_H
#define __LED_H
#include "sys.h"
#include <stdbool.h>

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly	
 * LED驱动代码
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/


#define LED_BLUE_L		PBout(12)	/*LED_BLUE_L PB12*/
#define LED_GREEN_L		PAout(6)	/*LED_GREEN_L PA6*/
#define LED_RED_L		PAout(7)	/*LED_RED_L PA7*/
#define LED_GREEN_R		PCout(13)	/*LED_GREEN_R PC13*/
#define LED_RED_R		PCout(14)	/*LED_RED_R PC14*/

void ledInit(void);	/* LED初始化 */


#endif
