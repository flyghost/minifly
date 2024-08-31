#ifndef __SENSORS_TYPES_H
#define __SENSORS_TYPES_H
#include "sys.h"
#include "axis.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * sensor参数类型定义	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

#if defined(__CC_ARM)
#pragma anon_unions
#endif

typedef union {
    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
    };
    int16_t axis[3];
} Axis3i16;

typedef union {
    struct
    {
        int32_t x;
        int32_t y;
        int32_t z;
    };
    int32_t axis[3];
} Axis3i32;

typedef union {
    struct
    {
        int64_t x;
        int64_t y;
        int64_t z;
    };
    int64_t axis[3];
} Axis3i64;

typedef union {
    struct
    {
        float x;
        float y;
        float z;
    };
    float axis[3];
} Axis3f;

#define AXIS_X(a)       ((a)->x)
#define AXIS_Y(a)       ((a)->y)
#define AXIS_Z(a)       ((a)->z)

#define AXIS_MEMSET(a, b) \
    do {                  \
        (a)->x = (b);     \
        (a)->y = (b);     \
        (a)->z = (b);     \
    } while (0)

// 向量自加
#define AXIS_SELF_ADD(a, b) \
    do {                    \
        (a)->x += (b)->x;   \
        (a)->y += (b)->y;   \
        (a)->z += (b)->z;   \
    } while (0)



// 向量标量除
#define AXIS_SCALAR_DIV(out, in, scalar)               \
    do {                                               \
        for (u8 i = 0; i < 3; i++)                     \
            (out)->axis[i] = (in)->axis[i] / (scalar); \
    } while (0)

#define AXIS_SCALAR_DIV(out, in, scalar)               \
    do {                                               \
        for (u8 i = 0; i < 3; i++)                     \
            (out)->axis[i] = (in)->axis[i] / (scalar); \
    } while (0)

#define AXIS_MEMCPY(des, src) \
    do {                      \
        (des)->x = (src)->x;  \
        (des)->y = (src)->y;  \
        (des)->z = (src)->z;  \
    } while (0)

// #define AXIS_SCALAR_DIV(out, in, scalar) 
//     do {                                 
//         (out)->x = (in)->x / (scalar);   
//         (out)->y = (in)->y / (scalar);   
//         (out)->z = (in)->z / (scalar);   
//     } while (0)

#endif /* __SENSORS_TYPES_H */
