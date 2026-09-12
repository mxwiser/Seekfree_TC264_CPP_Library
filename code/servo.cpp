#include "servo.hpp"
#include "zf_common_headfile.h"



// 500-2500 us corresponds to 0-180 degrees; 1500 us is neutral.
#define SERVO_PWM_CHANNEL       (ATOM0_CH1_P33_9)
#define SERVO_PWM_FREQ_HZ       (50U)
#define SERVO_MIN_PULSE_US      (500)
#define SERVO_PULSE_SPAN_US     (2000)
#define SERVO_ANGLE_SPAN_DEG    (180)


//min 60 max 100 center 80
#define SERVO_CAR_CENTER_ANGLE    (80)
#define SERVO_CAR_LEFT_ANGLE    (100)
#define SERVO_CAR_RIGHT_ANGLE    (60)

static uint32 servo_angle_to_duty(int angle_deg)
{
    if (angle_deg < 0)
    {
        angle_deg = 0;
    }
    else if (angle_deg > SERVO_ANGLE_SPAN_DEG)
    {
        angle_deg = SERVO_ANGLE_SPAN_DEG;
    }

    const uint32 pulse_us = (uint32)(SERVO_MIN_PULSE_US
        + (angle_deg * SERVO_PULSE_SPAN_US + SERVO_ANGLE_SPAN_DEG / 2)
        / SERVO_ANGLE_SPAN_DEG);
    const uint32 period_us = 1000000U / SERVO_PWM_FREQ_HZ;
    return (uint32)(((uint64)pulse_us * PWM_DUTY_MAX
        + period_us / 2U) / period_us);
}

void servo_init()
{
    pwm_init(SERVO_PWM_CHANNEL, SERVO_PWM_FREQ_HZ,
        servo_angle_to_duty(SERVO_CAR_CENTER_ANGLE));
}

void servo_set_angle(int angle_deg)
{
    pwm_set_duty(SERVO_PWM_CHANNEL, servo_angle_to_duty(angle_deg));
}

void car_angle(float i){
    int angle=SERVO_CAR_CENTER_ANGLE;
    if(i>0)
        angle -= (SERVO_CAR_CENTER_ANGLE-SERVO_CAR_RIGHT_ANGLE)*i;
    else
        angle += (SERVO_CAR_CENTER_ANGLE-SERVO_CAR_LEFT_ANGLE)*i;
    servo_set_angle(angle);
}


