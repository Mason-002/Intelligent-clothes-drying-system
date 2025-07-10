/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-07-07     Administrator       the first version
 */
#include "mg90s.h"
#include <rtdbg.h>

static struct rt_device_pwm *pwm_dev = RT_NULL;
static struct rt_device_pwm *pwm_dev2 = RT_NULL;
int mg90s_init(void)
{
    /* 查找PWM设备 */
    pwm_dev = (struct rt_device_pwm *)rt_device_find(MG90S_PWM_DEVICE);
    if (pwm_dev == RT_NULL)
    {
        LOG_E("Can't find %s device!", MG90S_PWM_DEVICE);
        return -RT_ERROR;
    }

    /* 设置PWM周期为20ms(50Hz) */
    rt_pwm_set(pwm_dev, MG90S_PWM_CHANNEL, 20000000, 1500000);
    rt_pwm_enable(pwm_dev, MG90S_PWM_CHANNEL);


    return RT_EOK;
}

int mg90s_set_angle(int angle)
{
    rt_uint32_t pulse;
    rt_pwm_enable(pwm_dev, MG90S_PWM_CHANNEL);
    /* 角度限制 */
    if (angle < MG90S_MIN_ANGLE) angle = MG90S_MIN_ANGLE;
    if (angle > MG90S_MAX_ANGLE) angle = MG90S_MAX_ANGLE;

    /* 角度转脉冲宽度 */
    pulse = MG90S_MIN_PULSE + (angle * (MG90S_MAX_PULSE - MG90S_MIN_PULSE)) / MG90S_MAX_ANGLE;

    /* 设置PWM脉宽 */
    rt_pwm_set(pwm_dev, MG90S_PWM_CHANNEL, 20000000, pulse * 1000);

    return RT_EOK;
}

int mg90s2_init(void)
{
    /* 查找PWM设备 */
    pwm_dev2 = (struct rt_device_pwm *)rt_device_find(MG90S_PWM_DEVICE1);
    if (pwm_dev == RT_NULL)
    {
        LOG_E("Can't find %s device!", MG90S_PWM_DEVICE1);
        return -RT_ERROR;
    }

    /* 设置PWM周期为20ms(50Hz) */
    rt_pwm_set(pwm_dev, MG90S_PWM_CHANNEL1, 20000000, 1500000);
    rt_pwm_enable(pwm_dev2, MG90S_PWM_CHANNEL1);


    return RT_EOK;
}

int mg90s2_set_angle(int angle)
{
    rt_uint32_t pulse;
    rt_pwm_enable(pwm_dev, MG90S_PWM_CHANNEL1);
    /* 角度限制 */
    if (angle < MG90S_MIN_ANGLE) angle = MG90S_MIN_ANGLE;
    if (angle > MG90S_MAX_ANGLE) angle = MG90S_MAX_ANGLE;

    /* 角度转脉冲宽度 */
    pulse = MG90S_MIN_PULSE + (angle * (MG90S_MAX_PULSE - MG90S_MIN_PULSE)) / MG90S_MAX_ANGLE;

    /* 设置PWM脉宽 */
    rt_pwm_set(pwm_dev2, MG90S_PWM_CHANNEL1, 20000000, pulse * 1000);

    return RT_EOK;
}
