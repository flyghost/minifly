#include "state_estimator.h"
#include "attitude_pid.h"
#include "position_pid.h"
#include "maths.h"
#include "vl53lxx.h"
#include "stabilizer.h"
#include "sensfusion6.h"
#include "optical_flow.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 姿态估测代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3 
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
 *
 * 修改说明:
 * 版本V1.3 位置估测代码移植于inav-1.9.0
********************************************************************************/

#define ACC_LIMIT          (1000.f)                           /*加速度限幅 单位cm/s/s*/
#define ACC_LIMIT_MAX      (1800.f)                           /*最大加速度限幅 单位cm/s/s*/
#define VELOCITY_LIMIT     (130.f)                            /*速度限幅 单位cm/s*/
#define VELOCITY_LIMIT_MAX (500.f)                            /*最大速度限幅 单位cm/s*/

#define GRAVITY_CMSS                   (980.f)                /*重力加速度 单位cm/s/s*/
#define INAV_ACC_BIAS_ACCEPTANCE_VALUE (GRAVITY_CMSS * 0.25f) // Max accepted bias correction of 0.25G - unlikely we are going to be that much off anyway


static float wBaro    = 0.35f;     /*气压校正权重*/
static float wOpflowP = 1.0f;      /*光流位置校正权重*/
static float wOpflowV = 2.0f;      /*光流速度校正权重*/


static bool isRstHeight = false;   /*复位高度*/
static bool isRstAll    = true;    /*复位估测*/


static float fusedHeightLpf = 0.f; /*融合高度，低通*/
static float startBaroAsl   = 0.f; /*起飞点海拔*/


/*估测系统*/
static estimator_t estimator =
    {
        .vAccDeadband = 4.0f,
        .accBias[0]   = 0.0f,
        .accBias[1]   = 0.0f,
        .accBias[2]   = 0.0f,
        .acc[0]       = 0.0f,
        .acc[1]       = 0.0f,
        .acc[2]       = 0.0f,
        .vel[0]       = 0.0f,
        .vel[1]       = 0.0f,
        .vel[2]       = 0.0f,
        .pos[0]       = 0.0f,
        .pos[1]       = 0.0f,
        .pos[2]       = 0.0f,
};

/* Inertial filter, implementation taken from PX4 implementation by Anton Babushkin <rk3dov@gmail.com> */
// pos = pos + vel * dt + acc * dt^2 / 2.0
// vel = vel + acc * dt
static void inavFilterPredict(int axis, float dt, float acc)
{
    estimator.pos[axis] += estimator.vel[axis] * dt + acc * dt * dt / 2.0f;
    estimator.vel[axis] += acc * dt;
}
/*位置校正*/
static void inavFilterCorrectPos(int axis, float dt, float e, float w)
{
    float ewdt           = e * w * dt;
    estimator.pos[axis] += ewdt;
    estimator.vel[axis] += w * ewdt;
}
/*速度校正*/
static void inavFilterCorrectVel(int axis, float dt, float e, float w)
{
    estimator.vel[axis] += e * w * dt;
}

// 高度融合
static float heightEstimate(baro_t *bara, zRange_t *laser, float *accLpf, float *wi)
{
    static float rangeLpf  = 0.f;
    static float fusedHeight; /*融合高度，起飞点为0*/
    float        weight    = wBaro;
    float        quality;

    float relateHight = bara->asl - startBaroAsl;      /*气压相对高度*/

    // 光流模块可用,且使用激光，则将气压计和激光数据融合
    if (getModuleID() == OPTICAL_FLOW && isEnableVl53lxx == true)
    {
        vl53lxxReadRange(laser);                    /*读取激光数据*/

        // rangeLpf = laser->distance;
        rangeLpf += (laser->distance - rangeLpf) * 0.1f; /*低通 单位cm*/

        quality = laser->quality;

        if (quality < 0.3f) /*低于这个可行度，激光数据不可用*/
        {
            quality = 0.f;
        }
        else
        {
            weight       = quality;
            startBaroAsl = bara->asl - rangeLpf;
        }
        fusedHeight = rangeLpf * quality + (1.0f - quality) * relateHight; /*融合高度*/
    }
    else                                                                   /*无光流模块*/
    {
        fusedHeight = relateHight;                                         /*融合高度*/
    }

    // 融合后的高度，进行低通滤波
    fusedHeightLpf += (fusedHeight - fusedHeightLpf) * 0.1f;               /*融合高度 低通*/

    if (isRstHeight)
    {
        isRstHeight = false;

        weight = 0.95f; /*增加权重，快速调整*/

        startBaroAsl = bara->asl;

        if (getModuleID() == OPTICAL_FLOW)
        {
            if (laser->distance < VL53L0X_MAX_RANGE)
            {
                startBaroAsl -= laser->distance;
                fusedHeight   = laser->distance;
            }
        }

        estimator.pos[Z] = fusedHeight;
    }
    else if (isRstAll)
    {
        isRstAll = false;

        accLpf[Z]      = 0.f;
        fusedHeight    = 0.f;
        fusedHeightLpf = 0.f;
        startBaroAsl   = bara->asl;

        if (getModuleID() == OPTICAL_FLOW)
        {
            if (laser->distance < VL53L0X_MAX_RANGE)
            {
                startBaroAsl -= laser->distance;
                fusedHeight   = laser->distance;
            }
        }

        estimator.vel[Z] = 0.f;
        estimator.pos[Z] = fusedHeight;
    }

    *wi = weight;

    return fusedHeight;
}

static void accEstimate(Axis3f *axisAcc, float *accLpf, acc_t *accOut)
{
    Axis3f accelBF;

    // 加速度校正
    accelBF.x = axisAcc->x * GRAVITY_CMSS - estimator.accBias[X];
    accelBF.y = axisAcc->y * GRAVITY_CMSS - estimator.accBias[Y];
    accelBF.z = axisAcc->z * GRAVITY_CMSS - estimator.accBias[Z];

    // 将矢量旋转到地球坐标系 -从前右下到北东上
    imuTransformVectorBodyToEarth(&accelBF);

    estimator.acc[X] = applyDeadbandf(accelBF.x, estimator.vAccDeadband); /*去除死区的加速度*/
    estimator.acc[Y] = applyDeadbandf(accelBF.y, estimator.vAccDeadband); /*去除死区的加速度*/
    estimator.acc[Z] = applyDeadbandf(accelBF.z, estimator.vAccDeadband); /*去除死区的加速度*/

    // 加速度低通
    for (u8 i = 0; i < 3; i++)
        accLpf[i] += (estimator.acc[i] - accLpf[i]) * 0.1f;

    // 定高飞或者降落状态，状态的加速度为滤波后的加速度
    if ((getCommanderKeyFlight() == true) || (getCommanderKeyland() == true))                   /*定高飞或者降落状态*/
    {
        accOut->x = constrainf(accLpf[X], -ACC_LIMIT, ACC_LIMIT);                               /*加速度限幅*/
        accOut->y = constrainf(accLpf[Y], -ACC_LIMIT, ACC_LIMIT);                               /*加速度限幅*/
        accOut->z = constrainf(accLpf[Z], -ACC_LIMIT, ACC_LIMIT);                               /*加速度限幅*/
    }
    // 否则使用原始加速度
    else
    {
        accOut->x = constrainf(estimator.acc[X], -ACC_LIMIT_MAX, ACC_LIMIT_MAX); /*最大加速度限幅*/
        accOut->y = constrainf(estimator.acc[Y], -ACC_LIMIT_MAX, ACC_LIMIT_MAX); /*最大加速度限幅*/
        accOut->z = constrainf(estimator.acc[Z], -ACC_LIMIT_MAX, ACC_LIMIT_MAX); /*最大加速度限幅*/
    }
}

static void calcAccBias(float errPosZ, float dt)
{
    static float wAccBias = 0.01f;     /*加速度校正权重*/

    float accelBiasCorrMagnitudeSq;

    /*加速度偏置校正*/
    Axis3f accelBiasCorr = {
        {0, 0, 0}
    };

    accelBiasCorr.z         -= errPosZ * sq(wBaro);
    accelBiasCorrMagnitudeSq = sq(accelBiasCorr.x) + sq(accelBiasCorr.y) + sq(accelBiasCorr.z);
    if (accelBiasCorrMagnitudeSq < sq(INAV_ACC_BIAS_ACCEPTANCE_VALUE))
    {
        // 将误差向量从NEU坐标系变换到body坐标系
        imuTransformVectorEarthToBody(&accelBiasCorr);

        // 纠正加速度偏差
        estimator.accBias[X] += accelBiasCorr.x * wAccBias * dt;
        estimator.accBias[Y] += accelBiasCorr.y * wAccBias * dt;
        estimator.accBias[Z] += accelBiasCorr.z * wAccBias * dt;
    }
}

/**
 * @brief 位置估计
 * 
 * @param sensorData        传感器数据
 * @param state             更新状态量：加速度、速度、位置
 * @param dt 
 */
void positionEstimate(sensorData_t *sensorData, acc_t *accOut, velocity_t *velocityOut, point_t *positionOut, float dt)
{
    static float accLpf[3] = {0.f}; /*加速度低通*/

    float weight;

    // 计算融合后的高度和估测的速度
    float fuseHeight = heightEstimate(&sensorData->baro, &sensorData->zrange, accLpf, &weight);

    accEstimate(&sensorData->acc, accLpf, accOut);


    float errPosZ = fuseHeight - estimator.pos[Z];

    /* 位置预估: Z-axis */
    inavFilterPredict(Z, dt, estimator.acc[Z]);
    /* 位置校正: Z-axis */
    inavFilterCorrectPos(Z, dt, errPosZ, weight);

    if (getModuleID() == OPTICAL_FLOW) /*光流模块可用*/
    {
        float opflowDt = dt;

        float opResidualX    = opFlow.posSum[X] - estimator.pos[X];
        float opResidualY    = opFlow.posSum[Y] - estimator.pos[Y];
        float opResidualXVel = opFlow.velLpf[X] - estimator.vel[X];
        float opResidualYVel = opFlow.velLpf[Y] - estimator.vel[Y];

        float opWeightScaler = 1.0f;

        float wXYPos = wOpflowP * opWeightScaler;
        float wXYVel = wOpflowV * sq(opWeightScaler);

        /* 位置预估: XY-axis */
        inavFilterPredict(X, opflowDt, estimator.acc[X]);
        inavFilterPredict(Y, opflowDt, estimator.acc[Y]);
        /* 位置校正: XY-axis */
        inavFilterCorrectPos(X, opflowDt, opResidualX, wXYPos);
        inavFilterCorrectPos(Y, opflowDt, opResidualY, wXYPos);
        /* 速度校正: XY-axis */
        inavFilterCorrectVel(X, opflowDt, opResidualXVel, wXYVel);
        inavFilterCorrectVel(Y, opflowDt, opResidualYVel, wXYVel);
    }

    calcAccBias(errPosZ, dt);

    // 对估测的速度做限幅，然后输出
    if ((getCommanderKeyFlight() == true) || (getCommanderKeyland() == true))           /*定高飞或者降落状态*/
    {
        velocityOut->x = constrainf(estimator.vel[X], -VELOCITY_LIMIT, VELOCITY_LIMIT); /*速度限幅 VELOCITY_LIMIT*/
        velocityOut->y = constrainf(estimator.vel[Y], -VELOCITY_LIMIT, VELOCITY_LIMIT); /*速度限幅 VELOCITY_LIMIT*/
        velocityOut->z = constrainf(estimator.vel[Z], -VELOCITY_LIMIT, VELOCITY_LIMIT); /*速度限幅 VELOCITY_LIMIT*/
    }
    else
    {
        velocityOut->x = constrainf(estimator.vel[X], -VELOCITY_LIMIT_MAX, VELOCITY_LIMIT_MAX); /*最大速度限幅 VELOCITY_LIMIT_MAX*/
        velocityOut->y = constrainf(estimator.vel[Y], -VELOCITY_LIMIT_MAX, VELOCITY_LIMIT_MAX); /*最大速度限幅 VELOCITY_LIMIT_MAX*/
        velocityOut->z = constrainf(estimator.vel[Z], -VELOCITY_LIMIT_MAX, VELOCITY_LIMIT_MAX); /*最大速度限幅 VELOCITY_LIMIT_MAX*/
    }

    // 直接输出估测的位置
    positionOut->x = estimator.pos[X];
    positionOut->y = estimator.pos[Y];
    positionOut->z = estimator.pos[Z];
}

/*读取融合高度 单位cm*/
float getFusedHeight(void)
{
    return fusedHeightLpf;
}

/*复位估测高度*/
void estRstHeight(void)
{
    isRstHeight = true;
}

/*复位所有估测*/
void estRstAll(void)
{
    isRstAll = true;
}


