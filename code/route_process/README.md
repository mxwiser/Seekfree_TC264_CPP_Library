# Route process

This module follows a bright (white) track on a darker blue background using
the MT9V034 grayscale image. It ports the reference project's dynamic white
threshold, left/right edge tracing, and row-40/row-50 ramp state machine.

Runtime order:

1. `route_process_frame()` consumes each completed camera frame, traces both
   edges, calculates the weighted center error, drives the steering servo, and
   draws red edges plus a green center line on the IPS display.
2. `route_speed_control_100ms()` runs after the two encoder counts are sampled.
   Both motors use incremental PID. On a detected ramp, the ramp speed target
   is selected; turning reduces the inner wheel target.
3. If no usable white track is found, both motors stop automatically.

Tune `route_config.hpp` first. `ROUTE_NORMAL_SPEED_TICKS_100MS` and
`ROUTE_RAMP_SPEED_TICKS_100MS` are encoder pulse counts per 100 ms, not PWM
percentages. The screen shows `ENC L`, `ENC R`, `LINE ERR`, threshold, ramp
state, width delta, and the two PWM duty values.

Ramp states follow the source example: `0` normal, `1` candidate/entering,
`2` climbing, `3` crest, `4` leaving, and `5` exit hold. States 1 through 4
select the closed-loop ramp speed target.
