"""Python translation of the embedded route image and control algorithms."""

from dataclasses import dataclass, field
from pathlib import Path
import math
import re

import numpy as np


IMAGE_W = 188
IMAGE_H = 120
LEFT_FOUND = 0x01
RIGHT_FOUND = 0x02


def _cpp_config_number(name, default):
    config_path = Path(__file__).resolve().parents[1] / "route_config.hpp"
    if not config_path.exists():
        return default
    text = config_path.read_text(encoding="utf-8")
    match = re.search(
        rf"^\s*#define\s+{re.escape(name)}\s+\(\s*"
        rf"([-+]?\d+(?:\.\d+)?)\s*[fFuU]*\s*\)",
        text,
        flags=re.MULTILINE,
    )
    return type(default)(match.group(1)) if match else default


REFERENCE_ROWS = _cpp_config_number("ROUTE_REFERENCE_ROWS", 5)
BLACK_FLOOR = _cpp_config_number("ROUTE_BLACK_FLOOR", 50)
WHITE_MAX_RATIO_X10 = _cpp_config_number("ROUTE_WHITE_MAX_RATIO_X10", 13)
WHITE_MIN_RATIO_X10 = _cpp_config_number("ROUTE_WHITE_MIN_RATIO_X10", 7)
CONTRAST_OFFSET = _cpp_config_number("ROUTE_CONTRAST_OFFSET", 3)
CONTRAST_THRESHOLD = _cpp_config_number("ROUTE_CONTRAST_THRESHOLD", 30)
SEED_SEARCH_RANGE = _cpp_config_number("ROUTE_SEED_SEARCH_RANGE", 24)
MIN_TRACK_WIDTH = _cpp_config_number("ROUTE_MIN_TRACK_WIDTH", 12)
MAX_CENTER_JUMP = _cpp_config_number("ROUTE_MAX_CENTER_JUMP", 15)

STEERING_KP = _cpp_config_number("ROUTE_STEERING_KP", 0.025)
STEERING_KD = _cpp_config_number("ROUTE_STEERING_KD", 0.012)

NORMAL_SPEED = _cpp_config_number("ROUTE_NORMAL_SPEED_TICKS_100MS", 3000)
MIN_TURN_SPEED = _cpp_config_number(
    "ROUTE_MIN_TURN_SPEED_TICKS_100MS", 1000
)
SPEED_KP = _cpp_config_number("ROUTE_SPEED_KP", 0.006)
SPEED_KI = _cpp_config_number("ROUTE_SPEED_KI", 0.0020)
SPEED_KD = _cpp_config_number("ROUTE_SPEED_KD", 0.00)
MAX_DUTY = _cpp_config_number("ROUTE_MOTOR_MAX_DUTY_PERCENT", 45)


def c_div(numerator, denominator):
    """C/C++ integer division, which truncates toward zero."""
    return math.trunc(numerator / denominator)


def limit_int(value, minimum, maximum):
    return min(max(value, minimum), maximum)


def contrast(inside, outside):
    total = int(inside) + int(outside)
    if total == 0:
        return 0
    return c_div((int(inside) - int(outside)) * 200, total)


@dataclass
class RouteImageResult:
    white_min: int = BLACK_FLOOR
    white_max: int = 255
    reference_col: int = IMAGE_W // 2
    reference_row: int = IMAGE_H - 1
    left_edge: np.ndarray = field(
        default_factory=lambda: np.zeros(IMAGE_H, dtype=np.uint8)
    )
    right_edge: np.ndarray = field(
        default_factory=lambda: np.full(IMAGE_H, IMAGE_W - 1, dtype=np.uint8)
    )
    center_line: np.ndarray = field(
        default_factory=lambda: np.full(IMAGE_H, IMAGE_W // 2, dtype=np.uint8)
    )
    edge_flags: np.ndarray = field(
        default_factory=lambda: np.zeros(IMAGE_H, dtype=np.uint8)
    )
    steering_error: int = 0
    track_valid: int = 0


class RouteImageAlgorithm:
    def __init__(self):
        self.result = RouteImageResult()

    def reset(self):
        self.__init__()

    def _get_white_reference(self, image):
        first_row = IMAGE_H - REFERENCE_ROWS
        reference = int(
            np.sum(image[first_row:IMAGE_H], dtype=np.uint32)
            // (REFERENCE_ROWS * IMAGE_W)
        )
        self.result.white_min = limit_int(
            c_div(reference * WHITE_MIN_RATIO_X10, 10), BLACK_FLOOR, 255
        )
        self.result.white_max = limit_int(
            c_div(reference * WHITE_MAX_RATIO_X10, 10), BLACK_FLOOR, 255
        )

    def _find_reference_column(self, image):
        center = IMAGE_W // 2
        best_col = center
        best_row = IMAGE_H - 1
        best_center_distance = IMAGE_W

        for col in range(CONTRAST_OFFSET, IMAGE_W - CONTRAST_OFFSET,
                         CONTRAST_OFFSET):
            remote_row = CONTRAST_OFFSET
            for row in range(IMAGE_H - 1, CONTRAST_OFFSET, -CONTRAST_OFFSET):
                current = int(image[row, col])
                ahead = int(image[row - CONTRAST_OFFSET, col])
                if current < self.result.white_min:
                    remote_row = row
                    break
                if (ahead <= self.result.white_max
                        and contrast(current, ahead) > CONTRAST_THRESHOLD):
                    remote_row = row
                    break

            center_distance = abs(col - center)
            if (remote_row < best_row
                    or (remote_row == best_row
                        and center_distance < best_center_distance)):
                best_row = remote_row
                best_col = col
                best_center_distance = center_distance

        self.result.reference_col = limit_int(
            best_col, CONTRAST_OFFSET, IMAGE_W - CONTRAST_OFFSET - 1
        )
        self.result.reference_row = limit_int(
            best_row, CONTRAST_OFFSET, IMAGE_H - 20
        )

    def _find_white_seed(self, line, predicted):
        predicted = limit_int(
            predicted, CONTRAST_OFFSET, IMAGE_W - CONTRAST_OFFSET - 1
        )
        if int(line[predicted]) >= self.result.white_min:
            return predicted
        for offset in range(1, SEED_SEARCH_RANGE + 1):
            left = predicted - offset
            right = predicted + offset
            if (left >= CONTRAST_OFFSET
                    and int(line[left]) >= self.result.white_min):
                return left
            if (right < IMAGE_W - CONTRAST_OFFSET
                    and int(line[right]) >= self.result.white_min):
                return right
        return -1

    def _find_left_edge(self, line, seed):
        for col in range(seed, CONTRAST_OFFSET - 1, -1):
            inside = int(line[col])
            outside = int(line[col - CONTRAST_OFFSET])
            if inside < self.result.white_min:
                return col + 1, 1
            if (outside < self.result.white_min
                    or (outside <= self.result.white_max
                        and contrast(inside, outside) > CONTRAST_THRESHOLD)):
                return col, 1
        return 0, 0

    def _find_right_edge(self, line, seed):
        last = IMAGE_W - CONTRAST_OFFSET - 1
        for col in range(seed, last + 1):
            inside = int(line[col])
            outside = int(line[col + CONTRAST_OFFSET])
            if inside < self.result.white_min:
                return col - 1, 1
            if (outside < self.result.white_min
                    or (outside <= self.result.white_max
                        and contrast(inside, outside) > CONTRAST_THRESHOLD)):
                return col, 1
        return IMAGE_W - 1, 0

    def _trace_edges(self, image):
        predicted_center = self.result.reference_col
        estimated_width = IMAGE_W - 20
        center_initialized = False
        for row in range(IMAGE_H - 1, self.result.reference_row - 1, -1):
            line = image[row]
            seed = self._find_white_seed(line, predicted_center)
            if seed < 0:
                continue

            left, left_found = self._find_left_edge(line, seed)
            right, right_found = self._find_right_edge(line, seed)

            center = predicted_center
            if left_found and right_found and right - left >= MIN_TRACK_WIDTH:
                center = c_div(left + right, 2)
            elif left_found and not right_found:
                center = left + c_div(estimated_width, 2)
            elif not left_found and right_found:
                center = right - c_div(estimated_width, 2)
            else:
                continue

            center = limit_int(center, 0, IMAGE_W - 1)
            if (center_initialized
                    and abs(center - predicted_center) > MAX_CENTER_JUMP):
                continue

            if left_found:
                self.result.edge_flags[row] |= LEFT_FOUND
            if right_found:
                self.result.edge_flags[row] |= RIGHT_FOUND
            self.result.left_edge[row] = left
            self.result.right_edge[row] = right
            self.result.center_line[row] = center
            if left_found and right_found:
                estimated_width = right - left
            predicted_center = center
            center_initialized = True

    def _calculate_steering_error(self):
        weighted_error = 0
        weight_sum = 0
        usable_rows = 0
        first_row = max(self.result.reference_row + 5, 25)
        for row in range(first_row, min(90, IMAGE_H - 1) + 1, 5):
            if self.result.edge_flags[row] == 0:
                continue
            weight = 1 + c_div(90 - row, 10)
            weighted_error += (int(self.result.center_line[row])
                               - IMAGE_W // 2) * weight
            weight_sum += weight
            usable_rows += 1

        if usable_rows >= 4 and weight_sum > 0:
            self.result.steering_error = limit_int(
                c_div(weighted_error, weight_sum), -60, 60
            )
            self.result.track_valid = 1

    def process(self, image):
        image = np.asarray(image, dtype=np.uint8)
        if image.shape != (IMAGE_H, IMAGE_W):
            raise ValueError(f"expected {(IMAGE_H, IMAGE_W)}, got {image.shape}")

        self.result.left_edge.fill(0)
        self.result.right_edge.fill(IMAGE_W - 1)
        self.result.center_line.fill(IMAGE_W // 2)
        self.result.edge_flags.fill(0)
        self.result.steering_error = 0
        self.result.track_valid = 0

        self._get_white_reference(image)
        self._find_reference_column(image)
        self._trace_edges(image)
        self._calculate_steering_error()
        return self.result


@dataclass
class SpeedPid:
    integral: float = 0.0
    last_error: float = 0.0

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0

    def update(self, target, actual):
        actual = abs(actual)
        error = float(target) - float(actual)
        self.integral += SPEED_KI * error
        raw_output = (
            SPEED_KP * error
            + self.integral
            + SPEED_KD * (error - self.last_error)
        )
        output = min(max(raw_output, 0.0), float(MAX_DUTY))
        self.integral += output - raw_output
        self.last_error = error
        return int(output * 10.0 + 0.5) / 10.0


class RouteController:
    """Python translation of route_process.cpp's steering and speed control."""

    def __init__(self):
        self.last_image_error = 0.0
        self.steering_permille = 0
        self.left_target = 0
        self.right_target = 0
        self.left_duty = 0
        self.right_duty = 0
        self.left_pid = SpeedPid()
        self.right_pid = SpeedPid()

    def update_steering(self, result):
        if not result.track_valid:
            self.last_image_error = 0.0
            self.steering_permille = 0
            return 0.0
        error = float(result.steering_error)
        steering = STEERING_KP * error + STEERING_KD * (
            error - self.last_image_error
        )
        steering = min(max(steering, -1.0), 1.0)
        self.last_image_error = error
        self.steering_permille = int(steering * 1000.0)
        return steering

    def speed_targets(self):
        steering_abs = min(abs(self.steering_permille), 1000)
        minimum_target = min(NORMAL_SPEED, MIN_TURN_SPEED)
        reduction_range = NORMAL_SPEED - minimum_target
        target = NORMAL_SPEED - c_div(
            steering_abs * reduction_range, 1000
        )
        target = limit_int(target, minimum_target, NORMAL_SPEED)
        return target, target

    def update_speed(self, result, left_actual, right_actual):
        if not result.track_valid:
            self.left_pid.reset()
            self.right_pid.reset()
            self.left_target = 0
            self.right_target = 0
            self.left_duty = 0
            self.right_duty = 0
            return 0, 0, 0, 0

        self.left_target, self.right_target = self.speed_targets()
        self.left_duty = self.left_pid.update(
            self.left_target, left_actual
        )
        self.right_duty = self.right_pid.update(
            self.right_target, right_actual
        )
        return (self.left_target, self.right_target,
                self.left_duty, self.right_duty)
