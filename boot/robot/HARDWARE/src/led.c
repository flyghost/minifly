#include "led.h"

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


/* LED初始化 */
void ledInit(void)
{	
	GPIO_InitTypeDef GPIO_InitStructure;

	/*使能led时钟*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	/*LED_GREEN_L PA6	LED_RED_L PA7*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;	
    GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	/*LED_BLUE_L PB12*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/*LED_GREEN_R PC13	LED_RED_R PC14*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	/*点亮blue LED*/
	LED_BLUE_L=1;	
	/*关闭其他LED*/
	LED_GREEN_L=1;
	LED_RED_L=1;
	LED_GREEN_R=1;
	LED_RED_R=1;
}



