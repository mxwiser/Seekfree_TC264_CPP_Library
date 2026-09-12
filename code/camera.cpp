#include "camera.hpp"
#include "zf_common_headfile.h"

#pragma section all "cpu0_dsram"

// Keep a separate snapshot so UART transmission does not read a changing frame.
static uint8 image_copy[MT9V03X_H][MT9V03X_W];
static seekfree_assistant_camera_struct camera_information;

void camera_init(void)
{
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);

    while (mt9v03x_init())
    {
        // The camera driver reports initialization errors through the debug UART.
        system_delay_ms(500);
    }

    seekfree_assistant_camera_config(&camera_information,
        SEEKFREE_ASSISTANT_CAMERA_TYPE_MT9V03X,
        MT9V03X_W, MT9V03X_H, image_copy[0]);
}

void camera_send_frame(void)
{
    if (mt9v03x_finish_flag)
    {
        mt9v03x_finish_flag = 0;
        memcpy(image_copy[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
        seekfree_assistant_camera_send(&camera_information);
    }
}

#pragma section all restore
