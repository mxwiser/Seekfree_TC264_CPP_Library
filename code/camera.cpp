#include "camera.hpp"
#include "zf_common_headfile.h"

#pragma section all "cpu0_dsram"

void camera_init(void)
{
    ips200_init(IPS200_TYPE_SPI);
    ips200_show_string(0, 0, "mt9v03x init.");

    while (mt9v03x_init())
    {
        ips200_show_string(0, 80, "mt9v03x reinit.");
        system_delay_ms(500);
    }

    ips200_show_string(0, 16, "init success.");
}

void camera_display_frame(void)
{
    if (mt9v03x_finish_flag)
    {
        ips200_displayimage03x((const uint8 *)mt9v03x_image,
            MT9V03X_W, MT9V03X_H);
        mt9v03x_finish_flag = 0;
    }
}

#pragma section all restore
