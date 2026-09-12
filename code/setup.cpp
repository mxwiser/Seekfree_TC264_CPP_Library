/*
 * setup.cpp
 *
 *  Created on: 2026Äê9ÔÂ12ÈÕ
 *      Author: neowa
 */

#include "setup.hpp"
#include "servo.hpp"
#include "camera.hpp"
#include "motor.hpp"
#include "zf_common_headfile.h"

void setup(void)
{
    servo_init();
    camera_init();
    right_motor_init(20);
    system_delay_ms(5000);
    right_motor_stop();
}

void loop(void)
{
    camera_display_frame();
}
