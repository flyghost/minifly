#include "led.h"

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


/* LED��ʼ�� */
void ledInit(void)
{	
	GPIO_InitTypeDef GPIO_InitStructure;

	/*ʹ��ledʱ��*/
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
	
	/*����blue LED*/
	LED_BLUE_L=1;	
	/*�ر�����LED*/
	LED_GREEN_L=1;
	LED_RED_L=1;
	LED_GREEN_R=1;
	LED_RED_R=1;
}



