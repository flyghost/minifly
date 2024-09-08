#include "sys.h"
#include "delay.h"
#include "motors.h"
#include "pm.h"
#include "drv_pwm.h"



#define ENABLE_THRUST_BAT_COMPENSATED /*使能电池油门补偿*/

#define MOTORS_TEST_RATIO         (uint32_t)(0.2 * (1 << 16)) // 最大值为65535，当前值为13107，所以占空比 = 20%
#define MOTORS_TEST_ON_TIME_MS    50
#define MOTORS_TEST_DELAY_TIME_MS 150



extern bool isExitFlip;

static const u32 MOTORS[] = {MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4};

static uint16_t thrustBatCompensated(uint16_t ithrust)
{
#ifdef ENABLE_THRUST_BAT_COMPENSATED
    u16 ratio = ithrust;

    if (isExitFlip == true) /*500Hz*/
    {
        float thrust         = ((float)ithrust / 65536.0f) * 60;
        float volts          = -0.0006239f * thrust * thrust + 0.088f * thrust;
        float supply_voltage = pmGetBatteryVoltage();
        float percentage     = volts / supply_voltage;
        percentage           = percentage > 1.0f ? 1.0f : percentage;
        ratio                = percentage * UINT16_MAX;
    }

    return ratio;
#else
    return ithrust;
#endif
}

void motorsInit(void) /*电机初始化*/
{
    PWM_Init(MOTORS_PWM_PERIOD, MOTORS_PWM_PRESCALE);
}

/*电机测试*/
bool motorsTest(void)
{
    int i;

    for (i = 0; i < sizeof(MOTORS) / sizeof(*MOTORS); i++)
    {
        motorsSetRatio(MOTORS[i], MOTORS_TEST_RATIO);
        delay_xms(MOTORS_TEST_ON_TIME_MS);
        motorsSetRatio(MOTORS[i], 0);
        delay_xms(MOTORS_TEST_DELAY_TIME_MS);
    }

    return PWM_IsInit();
}

/*设置电机PWM占空比*/
void motorsSetRatio(u32 id, u16 ithrust)
{
    u16 ratio = ithrust;

    if (!PWM_IsInit()) {
        return;
    }

    ratio = thrustBatCompensated(ithrust);

    PWM_SetValue(id, ratio);
}

