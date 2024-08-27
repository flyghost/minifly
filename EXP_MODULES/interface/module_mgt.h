#ifndef __EXP_MODULE_H
#define __EXP_MODULE_H
#include "sys.h"
#include "module_detect.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 扩展模块管理驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

enum expModuleID getModuleID(void);
void             expModulePower(bool state);
void             expModuleMgtTask(void *param);


#endif /* __EXP_MODULE_H */

