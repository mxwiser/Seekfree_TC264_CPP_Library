#ifndef CODE_ROUTE_PROCESS_ROUTE_PROCESS_HPP_
#define CODE_ROUTE_PROCESS_ROUTE_PROCESS_HPP_

#ifdef __cplusplus
extern "C" {
#endif

// Initializes the route controller and both motor PWM channels at zero duty.
void route_process_init(void);

// Enables/disables automatic steering and speed control.
void route_process_start(void);
void route_process_stop(void);

// Processes and displays one completed camera frame. Call in loop().
void route_process_frame(void);

// Runs the two encoder speed loops. Call after encoder_update_100ms().
void route_speed_control_100ms(void);

// Targets are encoder pulse counts per 100 ms.
void route_process_set_speed(int normal_ticks, int ramp_ticks);

#ifdef __cplusplus
}
#endif

#endif /* CODE_ROUTE_PROCESS_ROUTE_PROCESS_HPP_ */
