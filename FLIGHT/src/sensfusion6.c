#include <math.h>
#include "sensfusion6.h"
#include "config.h"
#include "ledseq.h"
#include "maths.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 6轴数据融合代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
 *
 * 修改说明:
 * 版本V1.3 互补滤波代码移植于inav-1.9.0
********************************************************************************/

static float q0 = 1.0f; /*四元数*/
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;
static float rMat[3][3];                             /*旋转矩阵*/

static bool isGravityCalibrated  = false;            /*重力是否校校准完成*/
static Axis3f baseAcc = {.x = 0.f, .y = 0.f, .z = 1.0f};/*静态加速度,单位g*/

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
static float invSqrt(float x) /*快速开平方求倒*/
{
    float halfx = 0.5f * x;
    float y     = x;
    long  i     = *(long *)&y;
    i           = 0x5f3759df - (i >> 1);
    y           = *(float *)&i;
    y           = y * (1.5f - (halfx * y * y));
    return y;
}

/**
 * @brief 计算静态加速度
 * 
 * @param acc 地球坐标系下的加速度
 */
static bool calBaseAcc(Axis3f *inAcc, Axis3f *outBaseAcc)
{
#define ACCZ_SAMPLE 350

    static u16   cnt       = 0;
    static float accZMin   = 1.5;
    static float accZMax   = 0.5;
    static Axis3f sumAcc   = {0.f};

    Axis3f averageAcc = {0.f};
    bool isCalibrated = false;

    AXIS_SELF_ADD(&sumAcc, inAcc);

    if (AXIS_Z(inAcc) < accZMin) accZMin = AXIS_Z(inAcc);
    if (AXIS_Z(inAcc) > accZMax) accZMax = AXIS_Z(inAcc);

    if (++cnt >= ACCZ_SAMPLE) /*缓冲区满*/
    {
        // 最大误差(静止误差，太大的话说明在动，不准确)
        if ((accZMax - accZMin) < 0.015f)
        {
            AXIS_SCALAR_DIV(&averageAcc, &sumAcc, ACCZ_SAMPLE);

            AXIS_MEMCPY(outBaseAcc, &averageAcc);

            isCalibrated = true;

            ledseqRun(SYS_LED, seq_calibrated); /*校准通过指示灯*/
        }

        // 复位状态
        cnt      = 0;
        accZMin  = 1.5;
        accZMax  = 0.5;
        AXIS_MEMSET(&sumAcc, 0);
    }

    return isCalibrated;

#undef ACCZ_SAMPLE
}

static void imuAccCalibrate(Axis3f *acc)
{
    // float  earthAxis[3] = {0.f};

    Axis3f earthAxis = {0.f};

    if (!isGravityCalibrated) /*未校准*/
    {
        // 转到机体坐标系
        // earthAxis.x = acc->x * rMat[0][0] + acc->y * rMat[0][1] + acc->z * rMat[0][2]; /*accx*/
        // earthAxis.y = acc->x * rMat[1][0] + acc->y * rMat[1][1] + acc->z * rMat[1][2]; /*accy*/
        earthAxis.z = acc->x * rMat[2][0] + acc->y * rMat[2][1] + acc->z * rMat[2][2]; /*accz*/
        isGravityCalibrated = calBaseAcc(&earthAxis, &baseAcc);                        /*计算静态加速度*/
    }
}

/**
 * @brief 陀螺仪补偿
 * 
 * 数据融合 互补滤波
 * 
 * @param acc 
 * @param gyro 
 * @param dt 
 */
static void imuFusion(Axis3f *acc, Axis3f *gyro, Axis3f *out, float dt)
{
    static float Kp    = 0.4f;   /*比例增益*/
    static float Ki    = 0.001f; /*积分增益*/
    static float exInt = 0.0f;
    static float eyInt = 0.0f;
    static float ezInt = 0.0f;     /*积分误差累计*/

    float  normalise;
    float  ex, ey, ez;

    out->x = gyro->x;
    out->y = gyro->y;
    out->z = gyro->z;

    /* 加速度计输出有效时,利用加速度计补偿陀螺仪*/
    if ((acc->x != 0.0f) || (acc->y != 0.0f) || (acc->z != 0.0f))
    {
        /*单位化加速计测量值*/
        normalise  = invSqrt(acc->x * acc->x + acc->y * acc->y + acc->z * acc->z);
        acc->x     *= normalise;
        acc->y     *= normalise;
        acc->z     *= normalise;

        /*加速计读取的方向与重力加速计方向的差值，用向量叉乘计算*/
        ex = (acc->y * rMat[2][2] - acc->z * rMat[2][1]);
        ey = (acc->z * rMat[2][0] - acc->x * rMat[2][2]);
        ez = (acc->x * rMat[2][1] - acc->y * rMat[2][0]);

        /*误差累计，与积分常数相乘*/
        exInt += Ki * ex * dt;
        eyInt += Ki * ey * dt;
        ezInt += Ki * ez * dt;

        /*用叉积误差来做PI修正陀螺零偏，即抵消陀螺读数中的偏移量*/
        out->x += Kp * ex + exInt;
        out->y += Kp * ey + eyInt;
        out->z += Kp * ez + ezInt;
    }
}

/**
 * @brief 更新四元数
 * 
 * @param gyro 
 * @param dt 
 */
static void quaternionUpdate(Axis3f *gyro, float dt)
{
    /* 一阶近似算法，四元数运动学方程的离散化形式和积分 */
    float  normalise;
    float  halfT  = 0.5f * dt;
    float q0Last  = q0;
    float q1Last  = q1;
    float q2Last  = q2;
    float q3Last  = q3;

    q0           += (-q1Last * gyro->x - q2Last * gyro->y - q3Last * gyro->z) * halfT;
    q1           += (q0Last * gyro->x + q2Last * gyro->z - q3Last * gyro->y) * halfT;
    q2           += (q0Last * gyro->y - q1Last * gyro->z + q3Last * gyro->x) * halfT;
    q3           += (q0Last * gyro->z + q1Last * gyro->y - q2Last * gyro->x) * halfT;

    /*单位化四元数*/
    normalise  = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0        *= normalise;
    q1        *= normalise;
    q2        *= normalise;
    q3        *= normalise;
}

/*计算旋转矩阵*/
static void imuComputeRotationMatrix(void)
{
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3;

    rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    rMat[0][1] = 2.0f * (q1q2 + -q0q3);
    rMat[0][2] = 2.0f * (q1q3 - -q0q2);

    rMat[1][0] = 2.0f * (q1q2 - -q0q3);
    rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    rMat[1][2] = 2.0f * (q2q3 + -q0q1);

    rMat[2][0] = 2.0f * (q1q3 + -q0q2);
    rMat[2][1] = 2.0f * (q2q3 - -q0q1);
    rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}

/**
 * @brief 姿态解算
 * 
 * 数据融合 互补滤波
 * 
 * @param acc       加速度
 * @param gyro      陀螺仪
 * @param state     状态：计算得到的俯仰角，俯仰角，偏航角保存在里面
 * @param dt 
 */
void imuUpdate(Axis3f acc, Axis3f gyro, attitude_t *attitude, float dt)
{
    Axis3f tempacc = acc;
    Axis3f fusionGyro;

    gyro.x = gyro.x * DEG2RAD; /* 度转弧度 */
    gyro.y = gyro.y * DEG2RAD;
    gyro.z = gyro.z * DEG2RAD;

    imuFusion(&tempacc, &gyro, &fusionGyro, dt);

    quaternionUpdate(&fusionGyro, dt);/* 一阶近似算法，四元数运动学方程的离散化形式和积分 */

    imuComputeRotationMatrix(); /*计算旋转矩阵*/

    /*计算roll pitch yaw 欧拉角*/
    attitude->pitch = -asinf(rMat[2][0]) * RAD2DEG;
    attitude->roll  = atan2f(rMat[2][1], rMat[2][2]) * RAD2DEG;
    attitude->yaw   = atan2f(rMat[1][0], rMat[0][0]) * RAD2DEG;

    imuAccCalibrate(&acc);
}

/*机体到地球*/
void imuTransformVectorBodyToEarth(Axis3f *v)
{
    /* From body frame to earth frame */
    const float x = rMat[0][0] * AXIS_X(v) + rMat[0][1] * AXIS_Y(v) + rMat[0][2] * AXIS_Z(v);
    const float y = rMat[1][0] * AXIS_X(v) + rMat[1][1] * AXIS_Y(v) + rMat[1][2] * AXIS_Z(v);
    const float z = rMat[2][0] * AXIS_X(v) + rMat[2][1] * AXIS_Y(v) + rMat[2][2] * AXIS_Z(v);

    float yawRad = atan2f(rMat[1][0], rMat[0][0]);
    float cosy   = cosf(yawRad);
    float siny   = sinf(yawRad);
    float vx     = x * cosy + y * siny;
    float vy     = y * cosy - x * siny;

    AXIS_X(v) = vx;
    AXIS_Y(v) = -vy;
    AXIS_Z(v) = z - baseAcc.axis[2] * 980.f; /*去除重力加速度*/
}

/*地球到机体*/
void imuTransformVectorEarthToBody(Axis3f *v)
{
    AXIS_Y(v) = -AXIS_Y(v);

    /* From earth frame to body frame */
    const float x = rMat[0][0] * AXIS_X(v) + rMat[1][0] * AXIS_Y(v) + rMat[2][0] * AXIS_Z(v);
    const float y = rMat[0][1] * AXIS_X(v) + rMat[1][1] * AXIS_Y(v) + rMat[2][1] * AXIS_Z(v);
    const float z = rMat[0][2] * AXIS_X(v) + rMat[1][2] * AXIS_Y(v) + rMat[2][2] * AXIS_Z(v);

    AXIS_X(v) = x;
    AXIS_Y(v) = y;
    AXIS_Z(v) = z;
}

bool imuIsCalibrated(void)
{
    return isGravityCalibrated;
}

