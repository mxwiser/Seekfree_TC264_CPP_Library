/*
 * setup.cpp
 *
 *  Created on: 2026Äê9ÔÂ12ÈÕ
 *      Author: neowa
 */

#include "setup.hpp"
#include "zf_common_headfile.h"

#pragma section all "cpu0_dsram"


#define CHANNEL_NUMBER          (4)
#define PWM_CH1                 (ATOM1_CH5_P20_9)
#define PWM_CH2                 (ATOM0_CH7_P20_8)
#define PWM_CH3                 (ATOM0_CH3_P21_5)
#define PWM_CH4                 (ATOM0_CH2_P21_4)


int16 duty = 0;
int16 duty_temp = 0;
uint8 channel_index = 0;
pwm_channel_enum channel_list[CHANNEL_NUMBER] = {PWM_CH1, PWM_CH2, PWM_CH3, PWM_CH4};


void setup(void)
{

    pwm_init(PWM_CH1, 17000, 0);                                                // ??? PWM ?? ?? 17KHz ????? 0%
    pwm_init(PWM_CH2, 17000, 0);                                                // ??? PWM ?? ?? 17KHz ????? 0%
    pwm_init(PWM_CH3, 17000, 0);                                                // ??? PWM ?? ?? 17KHz ????? 0%
    pwm_init(PWM_CH4, 17000, 0);                                                // ??? PWM ?? ?? 17KHz ????? 0%
    while (TRUE)
    {
        // ?????????????

        for(duty = 0; duty <= PWM_DUTY_MAX / 2; duty ++)                        // ???????? 50%
        {
			// ?????
            for(channel_index = 0; channel_index < CHANNEL_NUMBER; channel_index++) 
            {
                duty_temp = (duty + channel_index * PWM_DUTY_MAX / 8) % (PWM_DUTY_MAX / 2) + (PWM_DUTY_MAX / 2); 
                pwm_set_duty(channel_list[channel_index], duty_temp);           // ?????????
            }
            system_delay_us(200);
        }

        // ?????????????
    }
}

#pragma section all restore
