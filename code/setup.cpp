/*
 * setup.cpp
 *
 *  Created on: 2026Äê9ÔÂ12ÈÕ
 *      Author: neowa
 */

#include "setup.hpp"
#include "zf_common_headfile.h"

#pragma section all "cpu0_dsram"


// Assumed servo: 180 degrees over 500-2500 us, neutral at 1500 us.
#define SERVO_PWM_CHANNEL       (ATOM0_CH1_P33_9)
#define SERVO_PWM_FREQ_HZ       (50U)
#define SERVO_NEUTRAL_US        (1500)
#define SERVO_PULSE_SPAN_US     (2000)
#define SERVO_ANGLE_SPAN_DEG    (180)
#define SERVO_LEFT_OFFSET_DEG   (10)
// Left uses a smaller angle and a shorter pulse than neutral.
#define SERVO_LEFT_DIRECTION    (-1)

void setup(void)
{
    const uint32 pulse_us = (uint32)(SERVO_NEUTRAL_US
        + SERVO_LEFT_DIRECTION * SERVO_LEFT_OFFSET_DEG
        * SERVO_PULSE_SPAN_US / SERVO_ANGLE_SPAN_DEG);
    const uint32 period_us = 1000000U / SERVO_PWM_FREQ_HZ;
    const uint32 duty = (uint32)(((uint64)pulse_us * PWM_DUTY_MAX
        + period_us / 2U) / period_us);

    // Initialize once; hardware keeps outputting the target position pulse.
    pwm_init(SERVO_PWM_CHANNEL, SERVO_PWM_FREQ_HZ, duty);
}

#pragma section all restore
