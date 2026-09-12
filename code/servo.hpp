#ifndef CODE_SERVO_HPP_
#define CODE_SERVO_HPP_

// Absolute angle in degrees, from 0 to 180. No center offset.
// Values outside the supported range are clamped to 0 .. 180.
// Call once after clock_init(); the PWM output continues in hardware.
void servo_init();

// Call after servo_init() to change the target position.
void servo_set_angle(int angle_deg);

void car_angle(float i);



#endif /* CODE_SERVO_HPP_ */
