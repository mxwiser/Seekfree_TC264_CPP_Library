#ifndef CODE_CAMERA_HPP_
#define CODE_CAMERA_HPP_

// Call after clock_init() and debug_init(); retries until the camera is ready.
void camera_init(void);

// Call repeatedly. Sends a completed frame through the debug UART.
// Transmission is blocking: about two seconds per frame at 115200 baud.
void camera_send_frame(void);

#endif /* CODE_CAMERA_HPP_ */
