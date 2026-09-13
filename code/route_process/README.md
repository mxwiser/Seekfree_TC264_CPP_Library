# Route process

This module follows a bright (white) track on a darker blue background using
the MT9V034 grayscale image. It ports the reference project's dynamic white
threshold and left/right edge tracing.

Runtime order:

1. `route_process_frame()` consumes each completed camera frame, traces both
   edges, calculates the weighted center error, drives the steering servo, and
   draws red edges plus a green center line on the IPS display.
2. `route_speed_control_100ms()` runs after the two encoder counts are sampled.
   Both motors use the same direct positional PI/PID speed target over the
   whole track. The raw encoder error is converted directly to motor PWM.
3. If no usable white track is found, both motors stop automatically.

Tune `route_config.hpp` first. `ROUTE_NORMAL_SPEED_TICKS_100MS` is an encoder
pulse count per 100 ms, not a PWM percentage. The screen shows `ENC L`,
`ENC R`, `LINE ERR`, threshold, and both PWM duties.
Motor PWM is applied and displayed with 0.1% resolution.
