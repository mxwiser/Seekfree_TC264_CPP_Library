#include "motor.hpp"
#include "zf_common_headfile.h"

#pragma section all "cpu0_dsram"

#define RIGHT_MOTOR_DIR_PIN       (P02_6)
#define RIGHT_MOTOR_PWM_CHANNEL   (ATOM0_CH7_P02_7)

#define LEFT_MOTOR_DIR_PIN        (P02_4)
#define LEFT_MOTOR_PWM_CHANNEL    (ATOM0_CH5_P02_5)

#define MOTOR_PWM_FREQ_HZ         (2000U)

static uint32 motor_percent_to_duty(int duty_percent)
{
    if (duty_percent < 0)
    {
        duty_percent = 0;
    }
    else if (duty_percent > 100)
    {
        duty_percent = 100;
    }

    return (uint32)duty_percent * PWM_DUTY_MAX / 100U;
}

void right_motor_init(int duty_percent)
{
    gpio_init(RIGHT_MOTOR_DIR_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(RIGHT_MOTOR_PWM_CHANNEL, MOTOR_PWM_FREQ_HZ,
        motor_percent_to_duty(duty_percent));
}

void right_motor_set_duty(int duty_percent)
{
    pwm_set_duty(RIGHT_MOTOR_PWM_CHANNEL,
        motor_percent_to_duty(duty_percent));
}

void right_motor_stop(void)
{
    pwm_set_duty(RIGHT_MOTOR_PWM_CHANNEL, 0);
}

void left_motor_init(int duty_percent)
{
    gpio_init(LEFT_MOTOR_DIR_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(LEFT_MOTOR_PWM_CHANNEL, MOTOR_PWM_FREQ_HZ,
        motor_percent_to_duty(duty_percent));
}

void left_motor_set_duty(int duty_percent)
{
    pwm_set_duty(LEFT_MOTOR_PWM_CHANNEL,
        motor_percent_to_duty(duty_percent));
}

void left_motor_stop(void)
{
    pwm_set_duty(LEFT_MOTOR_PWM_CHANNEL, 0);
}

#pragma section all restore
