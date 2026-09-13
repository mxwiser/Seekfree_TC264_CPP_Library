#ifndef CODE_ROUTE_PROCESS_ROUTE_IMAGE_HPP_
#define CODE_ROUTE_PROCESS_ROUTE_IMAGE_HPP_

#include "zf_common_typedef.h"
#include "zf_device_mt9v03x.h"

typedef struct
{
    uint8 white_min;
    uint8 white_max;
    uint8 reference_col;
    uint8 reference_row;

    uint8 left_edge[MT9V03X_H];
    uint8 right_edge[MT9V03X_H];
    uint8 center_line[MT9V03X_H];
    uint8 edge_flags[MT9V03X_H];

    int16 steering_error;
    uint8 track_valid;
} route_image_result_t;

void route_image_reset(route_image_result_t *result);
void route_image_process(const uint8 *image, route_image_result_t *result);

#endif /* CODE_ROUTE_PROCESS_ROUTE_IMAGE_HPP_ */
