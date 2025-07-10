/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-07-07     Administrator       the first version
 */
#ifndef APPLICATIONS_MG90S_H_
#define APPLICATIONS_MG90S_H_

#ifndef __MG90S_H__
#define __MG90S_H__

#include <rtthread.h>
#include <rtdevice.h>

#define MG90S_PWM_DEVICE    "pwm1"  // PWM设备名称
#define MG90S_PWM_CHANNEL   "A8"      // PWM通道

#define MG90S_PWM_DEVICE1    "pwm2"  // PWM设备名称
#define MG90S_PWM_CHANNEL1   "A7"      // PWM通道

/* 舵机角度范围定义 */
#define MG90S_MIN_ANGLE     0       // 最小角度(度)
#define MG90S_MAX_ANGLE     180     // 最大角度(度)
#define MG90S_MIN_PULSE     500     // 0度对应的脉冲宽度(us)
#define MG90S_MAX_PULSE     2500    // 180度对应的脉冲宽度(us)

/* 舵机控制函数 */
int mg90s_init(void);
int mg90s_set_angle(int angle);

int mg90s2_init(void);
int mg90s2_set_angle(int angle);

#endif

#endif /* APPLICATIONS_MG90S_H_ */
