import numpy as np

from route_algorithm import IMAGE_H, IMAGE_W, RouteImageAlgorithm, c_div


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
    print("Python route algorithm matches the C++ synthetic reference cases.")


if __name__ == "__main__":
    main()
