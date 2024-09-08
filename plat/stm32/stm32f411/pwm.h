#ifndef __PWM_H
#define __PWM_H


#include <stdbool.h>
#include <stdint.h>

#define MOTOR_M1      0
#define MOTOR_M2      1
#define MOTOR_M3      2
#define MOTOR_M4      3
#define NBR_OF_MOTORS 4

/* 96M主频下 8位精度输出375K PWM */
#define TIM_CLOCK_HZ        96000000
#define MOTORS_PWM_BITS     8                                   // 只是用寄存器的低8位，精度为8位
#define MOTORS_PWM_PERIOD   ((1 << MOTORS_PWM_BITS) - 1)        // 自动重装值，当前精度下：2^8 - 1=255
#define MOTORS_PWM_PRESCALE 0                                   // 定时器分频系数

void PWM_Init(uint32_t maxPeriod, uint32_t preScaler);
bool PWM_IsInit(void);

// 范围：0~65535
void PWM_SetValue(uint8_t id, uint16_t ratio);




#endif

