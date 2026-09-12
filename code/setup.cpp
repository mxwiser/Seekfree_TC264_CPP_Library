/*
 * setup.cpp
 *
 *  Created on: 2026Äê9ÔÂ12ÈÕ
 *      Author: neowa
 */

#include "setup.hpp"
#include "servo.hpp"
#include "zf_common_headfile.h"





void setup(void)
{
    servo_init();
    while(1){
        system_delay_ms(5000);
        //car_angle(0.2);
        system_delay_ms(5000);
        //car_angle(-0.2);

    };
}



