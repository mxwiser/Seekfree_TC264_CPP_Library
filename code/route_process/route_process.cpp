#include "route_process.hpp"
#include "route_config.hpp"
#include "route_image.hpp"
#include "encoder.hpp"
#include "motor.hpp"
#include "servo.hpp"
#include "zf_common_headfile.h"

#pragma section all "cpu0_dsram"

typedef struct
{
    float last_error;
    float previous_error;
    float output;
} route_speed_pid_t;

static route_image_result_t route_image_result;
static route_speed_pid_t left_speed_pid;
static route_speed_pid_t right_speed_pid;

static volatile uint8 route_enabled = 0;
static volatile uint8 route_track_valid = 0;
static volatile uint8 route_ramp_active = 0;
static volatile int16 route_steering_permille = 0;
static volatile int16 route_left_duty = 0;
static volatile int16 route_right_duty = 0;
static volatile int16 route_normal_speed_ticks
    = ROUTE_NORMAL_SPEED_TICKS_100MS;
static volatile int16 route_ramp_speed_ticks
    = ROUTE_RAMP_SPEED_TICKS_100MS;

static float route_last_image_error = 0.0f;
static float route_filtered_steering = 0.0f;

static int route_limit_int(int value, int minimum, int maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static float route_limit_float(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static void route_reset_speed_pid(route_speed_pid_t *pid)
{
    pid->last_error = 0.0f;
    pid->previous_error = 0.0f;
    pid->output = 0.0f;
}

static int route_update_speed_pid(route_speed_pid_t *pid,
    int target, int actual)
{
    const float error = (float)(target - actual);
    pid->output += ROUTE_SPEED_KP * (error - pid->last_error)
        + ROUTE_SPEED_KI * error
        + ROUTE_SPEED_KD
            * (error - 2.0f * pid->last_error + pid->previous_error);
    pid->output = route_limit_float(pid->output, 0.0f,
        (float)ROUTE_MOTOR_MAX_DUTY_PERCENT);
    pid->previous_error = pid->last_error;
    pid->last_error = error;
    return (int)(pid->output + 0.5f);
}

static void route_update_steering(void)
{
    if (!route_image_result.track_valid)
    {
        route_track_valid = 0;
        route_steering_permille = 0;
        route_last_image_error = 0.0f;
        route_filtered_steering = 0.0f;
        car_angle(0.0f);
        return;
    }

    const float error = (float)route_image_result.steering_error;
    const float raw_steering = ROUTE_STEERING_KP * error
        + ROUTE_STEERING_KD * (error - route_last_image_error);
    route_filtered_steering += ROUTE_STEERING_FILTER
        * (raw_steering - route_filtered_steering);
    route_filtered_steering = route_limit_float(route_filtered_steering,
        -1.0f, 1.0f);
    route_last_image_error = error;

    route_steering_permille = (int16)(route_filtered_steering * 1000.0f);
    route_track_valid = 1;
    route_ramp_active = route_image_result.ramp_active;
    if (route_enabled)
    {
        car_angle(route_filtered_steering);
    }
}

static void route_draw_result(void)
{
    ips200_displayimage03x((const uint8 *)mt9v03x_image,
        MT9V03X_W, MT9V03X_H);

    for (int row = route_image_result.reference_row;
        row < MT9V03X_H; row += 2)
    {
        if (route_image_result.edge_flags[row] & 0x01U)
        {
            ips200_draw_point(route_image_result.left_edge[row], row,
                RGB565_RED);
        }
        if (route_image_result.edge_flags[row] & 0x02U)
        {
            ips200_draw_point(route_image_result.right_edge[row], row,
                RGB565_RED);
        }
        if (route_image_result.edge_flags[row])
        {
            ips200_draw_point(route_image_result.center_line[row], row,
                RGB565_GREEN);
        }
    }

    ips200_show_string(0, 128, "LINE ERR:");
    ips200_show_int(80, 128, route_image_result.steering_error, 4);
    ips200_show_string(0, 144, "THRESH:");
    ips200_show_uint(64, 144, route_image_result.white_min, 3);
    ips200_show_string(0, 160, "RAMP:");
    ips200_show_uint(48, 160, route_image_result.ramp_state, 1);
    ips200_show_string(72, 160, "DW:");
    ips200_show_int(96, 160, route_image_result.ramp_width_delta, 3);
    ips200_show_string(0, 176, "DUTY L/R:");
    ips200_show_int(80, 176, route_left_duty, 3);
    ips200_show_int(112, 176, route_right_duty, 3);
    encoder_display();
}

void route_process_init(void)
{
    route_image_reset(&route_image_result);
    route_reset_speed_pid(&left_speed_pid);
    route_reset_speed_pid(&right_speed_pid);
    route_track_valid = 0;
    route_ramp_active = 0;
    route_steering_permille = 0;
    route_left_duty = 0;
    route_right_duty = 0;
    route_last_image_error = 0.0f;
    route_filtered_steering = 0.0f;
    route_enabled = 0;

    left_motor_init(0);
    right_motor_init(0);
}

void route_process_start(void)
{
    route_reset_speed_pid(&left_speed_pid);
    route_reset_speed_pid(&right_speed_pid);
    route_enabled = 1;
}

void route_process_stop(void)
{
    route_enabled = 0;
    route_left_duty = 0;
    route_right_duty = 0;
    left_motor_stop();
    right_motor_stop();
    car_angle(0.0f);
    route_reset_speed_pid(&left_speed_pid);
    route_reset_speed_pid(&right_speed_pid);
}

void route_process_set_speed(int normal_ticks, int ramp_ticks)
{
    route_normal_speed_ticks = (int16)route_limit_int(normal_ticks, 0, 30000);
    route_ramp_speed_ticks = (int16)route_limit_int(ramp_ticks, 0, 30000);
}

void route_process_frame(void)
{
    if (!mt9v03x_finish_flag)
    {
        return;
    }

    route_image_process((const uint8 *)mt9v03x_image,
        &route_image_result);
    route_update_steering();
    route_draw_result();
    mt9v03x_finish_flag = 0;
}

void route_speed_control_100ms(void)
{
    if (!route_enabled || !route_track_valid)
    {
        route_left_duty = 0;
        route_right_duty = 0;
        left_motor_stop();
        right_motor_stop();
        route_reset_speed_pid(&left_speed_pid);
        route_reset_speed_pid(&right_speed_pid);
        return;
    }

    int left_target = route_ramp_active
        ? route_ramp_speed_ticks : route_normal_speed_ticks;
    int right_target = left_target;
    const int steering = route_steering_permille;
    const int reduction = ((steering < 0 ? -steering : steering)
        * ROUTE_TURN_SPEED_REDUCTION_TICKS) / 1000;

    if (steering > 0)
    {
        right_target -= reduction;
    }
    else
    {
        left_target -= reduction;
    }
    left_target = route_limit_int(left_target, 0, 30000);
    right_target = route_limit_int(right_target, 0, 30000);

    //route_left_duty = (int16)route_update_speed_pid(&left_speed_pid,
    //    left_target, encoder_get_left_count_100ms());
    //route_right_duty = (int16)route_update_speed_pid(&right_speed_pid,
    //    right_target, encoder_get_right_count_100ms());
    left_motor_set_duty(25);
    right_motor_set_duty(25);
}

#pragma section all restore
