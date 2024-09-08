#ifndef __MOTORS_H
#define __MOTORS_H
#include "sys.h"
#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "drv_pwm.h"





void motorsInit(void);                    /*电机初始化*/
bool motorsTest(void);                    /*电机测试*/
void motorsSetRatio(u32 id, u16 ithrust); /*设置电机占空比*/


#endif

