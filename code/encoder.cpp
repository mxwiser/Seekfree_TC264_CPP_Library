#include "encoder.hpp"
#include "zf_common_headfile.h"

#pragma section all "cpu0_dsram"

#define LEFT_ENCODER           (TIM2_ENCODER)
#define LEFT_ENCODER_A         (TIM2_ENCODER_CH1_P33_7)
#define LEFT_ENCODER_B         (TIM2_ENCODER_CH2_P33_6)

#define RIGHT_ENCODER          (TIM5_ENCODER)
#define RIGHT_ENCODER_A        (TIM5_ENCODER_CH1_P10_3)
#define RIGHT_ENCODER_B        (TIM5_ENCODER_CH2_P10_1)

#define ENCODER_SAMPLE_MS      (100U)

static volatile int16 right_encoder_count = 0;
static volatile int16 left_encoder_count = 0;

void encoder_init(void)
{
    encoder_dir_init(RIGHT_ENCODER, RIGHT_ENCODER_A, RIGHT_ENCODER_B);
    encoder_dir_init(LEFT_ENCODER, LEFT_ENCODER_A, LEFT_ENCODER_B);
    pit_ms_init(CCU60_CH0, ENCODER_SAMPLE_MS);
}

void encoder_update_100ms(void)
{
    // This encoder's direction level is opposite to the vehicle forward direction.
    right_encoder_count = (int16)(-encoder_get_count(RIGHT_ENCODER));
    encoder_clear_count(RIGHT_ENCODER);

    left_encoder_count = encoder_get_count(LEFT_ENCODER);
    encoder_clear_count(LEFT_ENCODER);
}

void encoder_display(void)
{
    const int16 right_count = right_encoder_count;
    const int16 left_count = left_encoder_count;

    ips200_show_string(0, 200, "ENC R:");
    ips200_show_int(64, 200, right_count, 6);
    ips200_show_string(0, 216, "ENC L:");
    ips200_show_int(64, 216, left_count, 6);
}

int16 encoder_get_left_count_100ms(void)
{
    return left_encoder_count;
}

int16 encoder_get_right_count_100ms(void)
{
    return right_encoder_count;
}

#pragma section all restore
