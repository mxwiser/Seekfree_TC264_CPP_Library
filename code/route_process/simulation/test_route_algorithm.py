import numpy as np

from route_algorithm import (
    IMAGE_H,
    IMAGE_W,
    NORMAL_SPEED,
    SPEED_KI,
    SPEED_KP,
    STEERING_KD,
    STEERING_KP,
    RouteController,
    RouteImageAlgorithm,
    RouteImageResult,
    SpeedPid,
    c_div,
)


def make_track(far_shift=0, ramp_shape=False):
    image = np.full((IMAGE_H, IMAGE_W), 35, dtype=np.uint8)
    for row in range(IMAGE_H):
        width = (56 + row * 3 // 5) if ramp_shape else (
            30 + row * 130 // 119
        )
        center = 94 + c_div(far_shift * (119 - row), 119)
        left = center - width // 2
        right = center + width // 2
        image[row, max(left, 0):min(right + 1, IMAGE_W)] = 210
    return image


def main():
    algorithm = RouteImageAlgorithm()
    straight = algorithm.process(make_track())
    assert (straight.track_valid, straight.steering_error,
            straight.reference_col, straight.reference_row,
            int(straight.track_width[40]), int(straight.track_width[50])) == (
                1, 0, 93, 3, 68, 80
            )
    assert algorithm.process(make_track(30)).steering_error == 15
    assert algorithm.process(make_track(-30)).steering_error == -15

    algorithm.reset()
    for _ in range(6):
        ramp = algorithm.process(make_track(ramp_shape=True))
    assert (ramp.ramp_width_delta, ramp.ramp_state, ramp.ramp_active) == (
        6, 1, 1
    )

    algorithm.reset()
    assert algorithm.process(np.full((IMAGE_H, IMAGE_W), 35,
                                     dtype=np.uint8)).track_valid == 0
    assert algorithm.process(np.full((IMAGE_H, IMAGE_W), 210,
                                     dtype=np.uint8)).track_valid == 0

    controller = RouteController()
    controller.steering_permille = 1000
    assert controller.speed_targets(False) == (NORMAL_SPEED, NORMAL_SPEED)
    assert controller.speed_targets(True) == (NORMAL_SPEED, NORMAL_SPEED)

    steering_result = RouteImageResult(track_valid=1, steering_error=10)
    expected_steering = min(max(
        STEERING_KP * 10.0 + STEERING_KD * 10.0, -1.0
    ), 1.0)
    assert controller.update_steering(steering_result) == expected_steering

    pid = SpeedPid()
    first_duty = pid.update(NORMAL_SPEED, 0)
    expected_direct_duty = (
        (SPEED_KP + SPEED_KI) * NORMAL_SPEED
    )
    assert first_duty == int(expected_direct_duty * 10.0 + 0.5) / 10.0
    assert pid.update(NORMAL_SPEED, 0) > first_duty
    assert SpeedPid().update(NORMAL_SPEED, 3000) == 0.0

    pid = SpeedPid()
    plant_speed = 0.0
    peak_speed = 0.0
    for _ in range(80):
        duty = pid.update(NORMAL_SPEED, int(plant_speed + 0.5))
        # Hardware observation: 25% PWM is about 3000 ticks / 100 ms.
        plant_speed += (duty * 120.0 - plant_speed) * 0.35
        peak_speed = max(peak_speed, plant_speed)
    assert abs(plant_speed - NORMAL_SPEED) < 20.0
    assert peak_speed < NORMAL_SPEED * 1.10

    controller = RouteController()
    valid = RouteImageResult(track_valid=1)
    controller.update_speed(valid, 0, 0)
    invalid = RouteImageResult(track_valid=0)
    assert controller.update_speed(invalid, 0, 0) == (0, 0, 0, 0)
    print("Python route algorithm matches the C++ synthetic reference cases.")


if __name__ == "__main__":
    main()
