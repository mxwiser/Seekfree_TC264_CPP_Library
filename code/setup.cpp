/*
 * setup.cpp
 *
 *  Created on: 2026Äê9ÔÂ12ÈÕ
 *      Author: neowa
 */

#include "setup.hpp"
#include "servo.hpp"
#include "camera.hpp"

void setup(void)
{
    servo_init();
    camera_init();
}

void loop(void)
{
    camera_send_frame();
}
