#ifndef CODE_ROUTE_PROCESS_ROUTE_CONFIG_HPP_
#define CODE_ROUTE_PROCESS_ROUTE_CONFIG_HPP_

// Image parameters ported from the reference vehicle project.
#define ROUTE_REFERENCE_ROWS               (5)
#define ROUTE_BLACK_FLOOR                  (50)
#define ROUTE_WHITE_MAX_RATIO_X10          (13)
#define ROUTE_WHITE_MIN_RATIO_X10          (7)
#define ROUTE_CONTRAST_OFFSET              (3)
#define ROUTE_CONTRAST_THRESHOLD           (30)
#define ROUTE_SEED_SEARCH_RANGE            (24)
#define ROUTE_MIN_TRACK_WIDTH              (12)
#define ROUTE_MAX_CENTER_JUMP               (15)

// A positive image error means that the track center is to the right.
// car_angle(+1.0f) steers to the right and car_angle(-1.0f) to the left.
#define ROUTE_STEERING_KP                  (0.015f)
#define ROUTE_STEERING_KD                  (0.02f)

// Encoder targets are pulse counts measured during one 100 ms interval.
// Tune this value after observing ENC L / ENC R on the IPS display.
#define ROUTE_NORMAL_SPEED_TICKS_100MS     (3500)
#define ROUTE_MIN_TURN_SPEED_TICKS_100MS   (1000)

// Direct positional PI/PID output is motor duty in percent.
#define ROUTE_SPEED_KP                     (0.006f)
#define ROUTE_SPEED_KI                     (0.0020f)
#define ROUTE_SPEED_KD                     (0.00f)
#define ROUTE_MOTOR_MAX_DUTY_PERCENT       (80)

// Give the operator time to place the vehicle after reset.
#define ROUTE_START_DELAY_MS               (3000U)

#endif /* CODE_ROUTE_PROCESS_ROUTE_CONFIG_HPP_ */
