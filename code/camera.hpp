#ifndef CODE_CAMERA_HPP_
#define CODE_CAMERA_HPP_

// Call after clock_init() and debug_init(); retries until the camera is ready.
void camera_init(void);

// Call repeatedly to display each completed MT9V034 frame on IPS200.
void camera_display_frame(void);

#endif /* CODE_CAMERA_HPP_ */
