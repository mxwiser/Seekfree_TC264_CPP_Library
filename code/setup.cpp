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
#include "encoder.hpp"
#include "route_process/route_process.hpp"
#include "route_process/route_config.hpp"
#include "zf_common_headfile.h"

void setup(void)
{
    servo_init();
    camera_init();
    route_process_init();
    encoder_init();
    system_delay_ms(ROUTE_START_DELAY_MS);
    route_process_start();
}

void loop(void)
{
    route_process_frame();
}
