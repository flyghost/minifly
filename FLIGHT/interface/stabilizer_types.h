#ifndef __STABILIZER_TYPES_H
#define __STABILIZER_TYPES_H
#include "sys.h"
#include <stdbool.h>
#include "sensors_types.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 结构体类型定义	
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

typedef struct
{
    u32 timestamp; /*时间戳*/

    float roll;
    float pitch;
    float yaw;
} attitude_t;

struct vec3_s
{
    u32 timestamp; /*时间戳*/

    float x;
    float y;
    float z;
};

typedef struct vec3_s point_t;
typedef struct vec3_s velocity_t;
typedef struct vec3_s acc_t;

/* Orientation as a quaternion */
typedef struct quaternion_s
{
    uint32_t timestamp;

    union {
        struct
        {
            float q0;
            float q1;
            float q2;
            float q3;
        };
        struct
        {
            float x;
            float y;
            float z;
            float w;
        };
    };
} quaternion_t;

typedef struct toaMeasurement_s
{
    int8_t  senderId;
    float   x, y, z;
    int64_t rx, tx;
} toaMeasurement_t;

typedef struct tdoaMeasurement_s
{
    point_t anchorPosition[2];
    float   distanceDiff;
    float   stdDev;
} tdoaMeasurement_t;

typedef struct positionMeasurement_s
{
    union {
        struct
        {
            float x;
            float y;
            float z;
        };
        float pos[3];
    };
    float stdDev;
} positionMeasurement_t;

typedef struct distanceMeasurement_s
{
    union {
        struct
        {
            float x;
            float y;
            float z;
        };
        float pos[3];
    };
    float distance;
    float stdDev;
} distanceMeasurement_t;

typedef struct zRange_s
{
    uint32_t timestamp; // 时间戳
    float    distance;  // 测量距离，单位[cm]
    float    quality;   // 可信度
} zRange_t;

/** Flow measurement**/
typedef struct flowMeasurement_s
{
    uint32_t timestamp;
    union {
        struct
        {
            float dpixelx; // Accumulated pixel count x
            float dpixely; // Accumulated pixel count y
        };
        float dpixel[2];   // Accumulated pixel count
    };
    float stdDevX;         // Measurement standard deviation
    float stdDevY;         // Measurement standard deviation
    float dt;              // Time during which pixels were accumulated
} flowMeasurement_t;


/** TOF measurement**/
typedef struct tofMeasurement_s
{
    uint32_t timestamp;
    float    distance;
    float    stdDev;
} tofMeasurement_t;

typedef struct
{
    float pressure;             // 气压：hPa
    float temperature;          // 温度：摄氏度
    float asl;                  // 海拔高度：米
} baro_t;

typedef struct
{
    Axis3f   acc;               // 加速度计数据：单位：g(9.8m/s^2)，此时已经处理了加速度缩放因子
                                // 接着再对该数据做二阶低通滤波
    Axis3f   gyro;              // 陀螺仪数据：度/秒（此时已经减去了角速度偏移）
    Axis3f   mag;               // 磁力计数据：单位：高斯
    baro_t   baro;              // 气压计数据
    point_t  position;          // 进行位置估算后的空间坐标
    zRange_t zrange;            // 激光测量到的高度
} sensorData_t;

typedef struct
{
    attitude_t   attitude;              // 姿态
    quaternion_t attitudeQuaternion;
    point_t      position;              // 位置
    velocity_t   velocity;              // 速度
    acc_t        acc;                   // 加速度
    bool         isRCLocked;            // 是否锁定遥控器
} state_t;

enum dir_e
{
    CENTER = 0,
    FORWARD,
    BACK,
    LEFT,
    RIGHT,
};

typedef struct
{
    s16        roll;
    s16        pitch;
    s16        yaw;
    float      thrust;
    enum dir_e flipDir; /*翻滚方向*/
} control_t;

typedef enum
{
    modeDisable = 0, /*关闭模式*/
    modeAbs,         /*绝对值模式*/
    modeVelocity     /*速率模式*/
} mode_e;

typedef struct
{
    mode_e x;       // 手动模式：关闭
    mode_e y;       // 手动模式：关闭
    mode_e z;
    mode_e roll;    // 一直关闭
    mode_e pitch;   // 一直关闭
    mode_e yaw;
} mode_t;

typedef struct
{
    attitude_t attitude;     // 姿态：deg
    attitude_t attitudeRate; // deg/s
    point_t    position;     // 位置：m
    velocity_t velocity;     // 速度：m/s
    mode_t     mode;         // 模式
    float      thrust;
} setpoint_t;


#define RATE_5_HZ    5
#define RATE_10_HZ   10
#define RATE_25_HZ   25
#define RATE_50_HZ   50
#define RATE_100_HZ  100
#define RATE_200_HZ  200
#define RATE_250_HZ  250
#define RATE_500_HZ  500
#define RATE_1000_HZ 1000

#define MAIN_LOOP_RATE RATE_1000_HZ
#define MAIN_LOOP_DT   (u32)(1000 / MAIN_LOOP_RATE) /*单位ms*/

#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (MAIN_LOOP_RATE / RATE_HZ)) == 0)

#endif


