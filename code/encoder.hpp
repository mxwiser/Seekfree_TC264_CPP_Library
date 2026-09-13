#ifndef CODE_ENCODER_HPP_
#define CODE_ENCODER_HPP_

#include "zf_common_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

// Direction encoders:
// Left encoder count=P33.7, direction=P33.6.
// Right encoder count=P10.3, direction=P10.1.
void encoder_init(void);

// Called by the 100 ms PIT interrupt.
void encoder_update_100ms(void);

// Displays the latest 100 ms pulse counts below the camera image.
void encoder_display(void);

// Latest signed pulse counts captured by encoder_update_100ms().
// Vehicle-forward rotation is positive for both wheels.
int16 encoder_get_left_count_100ms(void);
int16 encoder_get_right_count_100ms(void);

#ifdef __cplusplus
}
#endif

#endif /* CODE_ENCODER_HPP_ */
