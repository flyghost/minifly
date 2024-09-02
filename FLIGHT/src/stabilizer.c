#include "system.h"
#include "stabilizer.h"
#include "sensors.h"
#include "sensfusion6.h"
#include "commander.h"
#include "anomal_detec.h"
#include "state_control.h"
#include "state_estimator.h"
#include "power_control.h"
#include "position_pid.h"
#include "flip.h"
#include "optical_flow.h"
#include "vl53lxx.h"
#include "maths.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 四轴自稳控制代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

static bool isInit;

static setpoint_t   g_targetState;  /*设置目标状态*/
static sensorData_t g_sensorData;   /*传感器数据*/
static rosState_t   g_rosState;     /*四轴姿态*/
static rosControl_t g_rosControl;   /*四轴控制参数*/

static u16   velModeTimes = 0;   /*速率模式次数*/
static u16   absModeTimes = 0;   /*绝对值模式次数*/
static float setHeight    = 0.f; /*设定目标高度 单位cm*/
static float baroLast     = 0.f;
static float baroVelLpf   = 0.f;


void stabilizerTask(void *param);

void stabilizerInit(void)
{
    if (isInit) return;

    stateControlInit(); /*姿态PID初始化*/
    powerControlInit(); /*电机初始化*/

    isInit = true;
}

bool stabilizerTest(void)
{
    bool pass = true;

    pass &= stateControlTest();
    pass &= powerControlTest();

    return pass;
}


/*设置快速调整参数*/
void setFastAdjustPosParam(u16 velTimes, u16 absTimes, float height)
{
    if (velTimes != 0 && velModeTimes == 0)
    {
        baroLast   = g_sensorData.baro.asl;
        baroVelLpf = 0.f;

        velModeTimes = velTimes;
    }
    if (absTimes != 0 && absModeTimes == 0)
    {
        setHeight    = height;
        absModeTimes = absTimes;
    }
}

/*快速调整高度*/
static void fastAdjustPosZ(void)
{
    if (velModeTimes > 0)
    {
        velModeTimes--;
        estRstHeight();                                             /*复位估测高度*/

        float baroVel  = (g_sensorData.baro.asl - baroLast) / 0.004f; /*250Hz*/
        baroLast       = g_sensorData.baro.asl;
        baroVelLpf    += (baroVel - baroVelLpf) * 0.35f;

        g_targetState.mode.z     = modeVelocity;
        g_rosState.velocity.z    = baroVelLpf; /*气压计融合*/
        g_targetState.velocity.z = -1.0f * baroVelLpf;

        if (velModeTimes == 0)
        {
            if (getModuleID() == OPTICAL_FLOW)
                setHeight = getFusedHeight();
            else
                setHeight = g_rosState.position.z;
        }
    }
    else if (absModeTimes > 0)
    {
        absModeTimes--;
        estRstAll(); /*复位估测*/
        g_targetState.mode.z     = modeAbs;
        g_targetState.position.z = setHeight;
    }
}

void stabilizerTask(void *param)
{
    u32 tick         = 0;
    u32 lastWakeTime = getSysTickCnt();

    ledseqRun(SYS_LED, seq_alive);

    while (!sensorsAreCalibrated())
    {
        vTaskDelayUntil(&lastWakeTime, MAIN_LOOP_DT);
    }

    while (1)
    {
        vTaskDelayUntil(&lastWakeTime, MAIN_LOOP_DT); /*1ms周期延时*/

        //获取6轴和气压数据（500Hz）
        if (RATE_DO_EXECUTE(RATE_500_HZ, tick))
        {
            sensorsAcquire(&g_sensorData, tick); /*获取6轴和气压数据*/
        }

        //四元数和欧拉角计算（250Hz）
        if (RATE_DO_EXECUTE(ATTITUDE_ESTIMAT_RATE, tick))
        {
            imuUpdate(g_sensorData.acc, g_sensorData.gyro, &g_rosState.attitude, ATTITUDE_ESTIMAT_DT);
        }

        //位置预估计算（250Hz）
        if (RATE_DO_EXECUTE(POSITION_ESTIMAT_RATE, tick))
        {
            positionEstimate(&g_sensorData, &g_rosState.acc, &g_rosState.velocity, &g_rosState.position, POSITION_ESTIMAT_DT);
        }

        //目标姿态和飞行模式设定（100Hz）
        if (RATE_DO_EXECUTE(RATE_100_HZ, tick) && imuIsCalibrated() == true)
        {
            commanderGetSetpoint(&g_targetState, &g_rosState); /*目标数据和飞行模式设定*/
        }

        if (RATE_DO_EXECUTE(RATE_250_HZ, tick))
        {
            fastAdjustPosZ(); /*快速调整高度*/
        }

        /*读取光流数据(100Hz)*/
        if (RATE_DO_EXECUTE(RATE_100_HZ, tick))
        {
            getOpFlowData(&g_rosState, 0.01f);
        }

        /*翻滚检测(500Hz) 非定点模式*/
        if (RATE_DO_EXECUTE(RATE_500_HZ, tick) && (getCommanderCtrlMode() != 0x03))
        {
            flyerFlipCheck(&g_targetState, &g_rosControl, &g_rosState);
        }

        /*异常检测*/
        anomalDetec(&g_sensorData, &g_rosState, &g_rosControl);

        /*PID控制*/

        stateControl(&g_rosControl, &g_sensorData, &g_rosState, &g_targetState, tick);


        //控制电机输出（500Hz）
        if (RATE_DO_EXECUTE(RATE_500_HZ, tick))
        {
            powerControl(&g_rosControl);
        }

        tick++;
    }
}


void getAttitudeData(attitude_t *get)
{
    get->pitch = -g_rosState.attitude.pitch;
    get->roll  = g_rosState.attitude.roll;
    get->yaw   = -g_rosState.attitude.yaw;
}

float getBaroData(void)
{
    return g_sensorData.baro.asl;
}

void getSensorData(sensorData_t *get)
{
    *get = g_sensorData;
}

void getStateData(Axis3f *acc, Axis3f *vel, Axis3f *pos)
{
    acc->x = 1.0f * g_rosState.acc.x;
    acc->y = 1.0f * g_rosState.acc.y;
    acc->z = 1.0f * g_rosState.acc.z;
    vel->x = 1.0f * g_rosState.velocity.x;
    vel->y = 1.0f * g_rosState.velocity.y;
    vel->z = 1.0f * g_rosState.velocity.z;
    pos->x = 1.0f * g_rosState.position.x;
    pos->y = 1.0f * g_rosState.position.y;
    pos->z = 1.0f * g_rosState.position.z;
}


