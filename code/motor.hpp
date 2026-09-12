#ifndef CODE_MOTOR_HPP_
#define CODE_MOTOR_HPP_

// Right motor: DIR=P02.6, PWM=P02.7, PWM frequency=2 kHz.
void right_motor_init(int duty_percent);
void right_motor_set_duty(int duty_percent);
void right_motor_stop(void);

#endif /* CODE_MOTOR_HPP_ */
