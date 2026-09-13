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
    float integral;
    float last_error;
} route_speed_pid_t;

static route_image_result_t route_image_result;
static route_speed_pid_t left_speed_pid;
static route_speed_pid_t right_speed_pid;

static volatile uint8 route_enabled = 0;
static volatile uint8 route_track_valid = 0;
static volatile int16 route_steering_permille = 0;
// PWM duty in tenths of one percent, so 83 means 8.3%.
static volatile int16 route_left_duty_x10 = 0;
static volatile int16 route_right_duty_x10 = 0;
static volatile int16 route_normal_speed_ticks
    = ROUTE_NORMAL_SPEED_TICKS_100MS;

static float route_last_image_error = 0.0f;

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
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
}

static int route_update_speed_pid(route_speed_pid_t *pid,
    int target, int actual)
{
    actual = actual < 0 ? -actual : actual;
    const float error = (float)target - (float)actual;
    pid->integral += ROUTE_SPEED_KI * error;
    const float raw_output = ROUTE_SPEED_KP * error
        + pid->integral
        + ROUTE_SPEED_KD * (error - pid->last_error);
    const float output = route_limit_float(raw_output, 0.0f,
        (float)ROUTE_MOTOR_MAX_DUTY_PERCENT);

    // Back-calculation prevents integral windup without filtering the output.
    pid->integral += output - raw_output;
    pid->last_error = error;
    return (int)(output * 10.0f + 0.5f);
}

static void route_update_steering(void)
{
    if (!route_image_result.track_valid)
    {
        route_track_valid = 0;
        route_steering_permille = 0;
        route_last_image_error = 0.0f;
        car_angle(0.0f);
        return;
    }

    const float error = (float)route_image_result.steering_error;
    const float steering = route_limit_float(
        ROUTE_STEERING_KP * error
            + ROUTE_STEERING_KD * (error - route_last_image_error),
        -1.0f, 1.0f);
    route_last_image_error = error;

    route_steering_permille = (int16)(steering * 1000.0f);
    route_track_valid = 1;
    if (route_enabled)
    {
        car_angle(steering);
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
    ips200_show_string(0, 160, "DUTY L/R:");
    ips200_show_float(80, 160, (double)route_left_duty_x10 / 10.0,
        2, 1);
    ips200_show_float(128, 160, (double)route_right_duty_x10 / 10.0,
        2, 1);
    encoder_display();
}

void route_process_init(void)
{
    route_image_reset(&route_image_result);
    route_reset_speed_pid(&left_speed_pid);
    route_reset_speed_pid(&right_speed_pid);
    route_track_valid = 0;
    route_steering_permille = 0;
    route_left_duty_x10 = 0;
    route_right_duty_x10 = 0;
    route_last_image_error = 0.0f;
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
    route_left_duty_x10 = 0;
    route_right_duty_x10 = 0;
    left_motor_stop();
    right_motor_stop();
    car_angle(0.0f);
    route_reset_speed_pid(&left_speed_pid);
    route_reset_speed_pid(&right_speed_pid);
}

void route_process_set_speed(int target_ticks)
{
    route_normal_speed_ticks = (int16)route_limit_int(target_ticks, 0, 30000);
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
    if (!route_enabled)
    {
        route_left_duty_x10 = 0;
        route_right_duty_x10 = 0;
        left_motor_stop();
        right_motor_stop();
        route_reset_speed_pid(&left_speed_pid);
        route_reset_speed_pid(&right_speed_pid);
        return;
    }

    if (!route_track_valid)
    {
        route_left_duty_x10 = 0;
        route_right_duty_x10 = 0;
        left_motor_stop();
        right_motor_stop();
        route_reset_speed_pid(&left_speed_pid);
        route_reset_speed_pid(&right_speed_pid);
        return;
    }

    int steering_abs = route_steering_permille;
    steering_abs = steering_abs < 0 ? -steering_abs : steering_abs;
    steering_abs = route_limit_int(steering_abs, 0, 1000);

    int target = route_normal_speed_ticks;
    const int minimum_turn_target = route_normal_speed_ticks
        < ROUTE_MIN_TURN_SPEED_TICKS_100MS
        ? route_normal_speed_ticks : ROUTE_MIN_TURN_SPEED_TICKS_100MS;
    const int speed_reduction_range = route_normal_speed_ticks
        - minimum_turn_target;
    if (speed_reduction_range > 0)
    {
        target -= steering_abs * speed_reduction_range / 1000;
    }
    target = route_limit_int(target,
        minimum_turn_target, route_normal_speed_ticks);

    // The servo handles direction; both motors keep the same target speed.
    int left_target = target;
    int right_target = target;

    route_left_duty_x10 = (int16)route_update_speed_pid(&left_speed_pid,
        left_target, encoder_get_left_count_100ms());
    route_right_duty_x10 = (int16)route_update_speed_pid(&right_speed_pid,
        right_target, encoder_get_right_count_100ms());
    left_motor_set_duty_x10(route_left_duty_x10);
    right_motor_set_duty_x10(route_right_duty_x10);
}

#pragma section all restore
