"""Python translation of code/route_process/route_image.cpp.

The loop order, integer limits, C/C++ integer division, edge flags, steering
weights, and ramp state transitions intentionally match the embedded code.
"""

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

STEERING_KP = _cpp_config_number("ROUTE_STEERING_KP", 0.025)
STEERING_KD = _cpp_config_number("ROUTE_STEERING_KD", 0.012)
STEERING_FILTER = _cpp_config_number("ROUTE_STEERING_FILTER", 0.35)
STEERING_DEADBAND_PIXELS = _cpp_config_number(
    "ROUTE_STEERING_DEADBAND_PIXELS", 0
)
STEERING_MAX_STEP = _cpp_config_number("ROUTE_STEERING_MAX_STEP", 1.0)
STEERING_LOST_HOLD_FRAMES = _cpp_config_number(
    "ROUTE_STEERING_LOST_HOLD_FRAMES", 0
)

NORMAL_SPEED = _cpp_config_number("ROUTE_NORMAL_SPEED_TICKS_100MS", 80)
RAMP_SPEED = _cpp_config_number("ROUTE_RAMP_SPEED_TICKS_100MS", 65)
TURN_REDUCTION = _cpp_config_number("ROUTE_TURN_SPEED_REDUCTION_TICKS", 20)
SPEED_KP = _cpp_config_number("ROUTE_SPEED_KP", 0.30)
SPEED_KI = _cpp_config_number("ROUTE_SPEED_KI", 0.05)
SPEED_KD = _cpp_config_number("ROUTE_SPEED_KD", 0.00)
MAX_DUTY = _cpp_config_number("ROUTE_MOTOR_MAX_DUTY_PERCENT", 60)


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
    track_width: np.ndarray = field(
        default_factory=lambda: np.zeros(IMAGE_H, dtype=np.uint8)
    )
    edge_flags: np.ndarray = field(
        default_factory=lambda: np.zeros(IMAGE_H, dtype=np.uint8)
    )
    steering_error: int = 0
    ramp_width_delta: int = 0
    track_valid: int = 0
    ramp_state: int = 0
    ramp_active: int = 0


class RouteImageAlgorithm:
    def __init__(self):
        self.result = RouteImageResult()
        self.ramp_confirm_count = 0
        self.ramp_timeout_count = 0
        self.ramp_exit_count = 0

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
        for row in range(IMAGE_H - 1, self.result.reference_row - 1, -1):
            line = image[row]
            seed = self._find_white_seed(line, predicted_center)
            if seed < 0:
                continue

            left, left_found = self._find_left_edge(line, seed)
            right, right_found = self._find_right_edge(line, seed)
            if left_found:
                self.result.edge_flags[row] |= LEFT_FOUND
            if right_found:
                self.result.edge_flags[row] |= RIGHT_FOUND
            self.result.left_edge[row] = left
            self.result.right_edge[row] = right

            center = predicted_center
            if left_found and right_found and right - left >= MIN_TRACK_WIDTH:
                estimated_width = right - left
                center = c_div(left + right, 2)
            elif left_found and not right_found:
                center = left + c_div(estimated_width, 2)
            elif not left_found and right_found:
                center = right - c_div(estimated_width, 2)
            else:
                continue

            center = limit_int(center, 0, IMAGE_W - 1)
            self.result.center_line[row] = center
            self.result.track_width[row] = limit_int(
                right - left, 0, IMAGE_W - 1
            )
            predicted_center = center

    def _calculate_steering_error(self):
        weighted_error = 0
        weight_sum = 0
        usable_rows = 0
        first_row = max(self.result.reference_row + 5, 30)
        for row in range(first_row, min(100, IMAGE_H - 1) + 1, 5):
            if self.result.edge_flags[row] == 0:
                continue
            weight = 1 + c_div(100 - row, 20)
            weighted_error += (int(self.result.center_line[row])
                               - IMAGE_W // 2) * weight
            weight_sum += weight
            usable_rows += 1

        if usable_rows >= 4 and weight_sum > 0:
            self.result.steering_error = limit_int(
                c_div(weighted_error, weight_sum), -60, 60
            )
            self.result.track_valid = 1

    def _update_ramp_state(self):
        both_edges = LEFT_FOUND | RIGHT_FOUND
        if (self.result.edge_flags[40] != both_edges
                or self.result.edge_flags[50] != both_edges):
            self.result.ramp_active = int(1 <= self.result.ramp_state <= 4)
            return

        delta = (int(self.result.track_width[50])
                 - int(self.result.track_width[40]))
        self.result.ramp_width_delta = delta
        state = self.result.ramp_state

        if state == 0:
            if 3 < delta < 9 and self.result.track_valid:
                self.ramp_confirm_count += 1
                if self.ramp_confirm_count > 5:
                    self.result.ramp_state = 1
                    self.ramp_confirm_count = 0
                    self.ramp_timeout_count = 0
            else:
                self.ramp_confirm_count = 0
        elif state == 1:
            self.ramp_timeout_count += 1
            if self.ramp_timeout_count >= 100:
                self.result.ramp_state = 0
                self.ramp_timeout_count = 0
            elif delta > 15:
                self.result.ramp_state = 2
        elif state == 2:
            if 5 < delta < 9:
                self.result.ramp_state = 3
        elif state == 3:
            if delta > 10:
                self.result.ramp_state = 4
                self.ramp_confirm_count = 0
        elif state == 4:
            if delta > 10:
                self.ramp_confirm_count += 1
                if self.ramp_confirm_count > 5:
                    self.result.ramp_state = 5
                    self.ramp_confirm_count = 0
                    self.ramp_exit_count = 0
            else:
                self.ramp_confirm_count = 0
        else:
            self.ramp_exit_count += 1
            if self.ramp_exit_count >= 20:
                self.result.ramp_state = 0
                self.ramp_exit_count = 0

        self.result.ramp_active = int(1 <= self.result.ramp_state <= 4)

    def process(self, image):
        image = np.asarray(image, dtype=np.uint8)
        if image.shape != (IMAGE_H, IMAGE_W):
            raise ValueError(f"expected {(IMAGE_H, IMAGE_W)}, got {image.shape}")

        ramp_state = self.result.ramp_state
        self.result.left_edge.fill(0)
        self.result.right_edge.fill(IMAGE_W - 1)
        self.result.center_line.fill(IMAGE_W // 2)
        self.result.track_width.fill(0)
        self.result.edge_flags.fill(0)
        self.result.steering_error = 0
        self.result.ramp_width_delta = 0
        self.result.track_valid = 0
        self.result.ramp_state = ramp_state

        self._get_white_reference(image)
        self._find_reference_column(image)
        self._trace_edges(image)
        self._calculate_steering_error()
        self._update_ramp_state()
        return self.result


@dataclass
class IncrementalPid:
    last_error: float = 0.0
    previous_error: float = 0.0
    output: float = 0.0

    def reset(self):
        self.last_error = 0.0
        self.previous_error = 0.0
        self.output = 0.0

    def update(self, target, actual):
        error = float(target - actual)
        self.output += (
            SPEED_KP * (error - self.last_error)
            + SPEED_KI * error
            + SPEED_KD * (error - 2.0 * self.last_error
                          + self.previous_error)
        )
        self.output = min(max(self.output, 0.0), float(MAX_DUTY))
        self.previous_error = self.last_error
        self.last_error = error
        return int(self.output + 0.5)


class RouteController:
    """Python translation of route_process.cpp's steering and speed control."""

    def __init__(self):
        self.last_image_error = 0.0
        self.filtered_steering = 0.0
        self.steering_permille = 0
        self.track_valid = 0
        self.ramp_active = 0
        self.lost_frame_count = 0
        self.left_pid = IncrementalPid()
        self.right_pid = IncrementalPid()

    def _move_steering_toward(self, target):
        target = min(max(target, -1.0), 1.0)
        filtered_target = self.filtered_steering + STEERING_FILTER * (
            target - self.filtered_steering
        )
        step = min(max(
            filtered_target - self.filtered_steering,
            -STEERING_MAX_STEP,
        ), STEERING_MAX_STEP)
        self.filtered_steering = min(max(
            self.filtered_steering + step, -1.0
        ), 1.0)
        self.steering_permille = int(self.filtered_steering * 1000.0)

    def update_steering(self, result):
        if not result.track_valid:
            if (self.track_valid
                    and self.lost_frame_count < STEERING_LOST_HOLD_FRAMES):
                self.lost_frame_count += 1
                return self.filtered_steering
            self.track_valid = 0
            self.ramp_active = 0
            self.last_image_error = 0.0
            self._move_steering_toward(0.0)
            return self.filtered_steering

        self.lost_frame_count = 0
        error = float(result.steering_error)
        if -STEERING_DEADBAND_PIXELS <= error <= STEERING_DEADBAND_PIXELS:
            error = 0.0
        raw = STEERING_KP * error + STEERING_KD * (
            error - self.last_image_error
        )
        self._move_steering_toward(raw)
        self.last_image_error = error
        self.track_valid = 1
        self.ramp_active = result.ramp_active
        return self.filtered_steering

    def speed_targets(self, ramp_active):
        target = RAMP_SPEED if ramp_active else NORMAL_SPEED
        left_target = target
        right_target = target
        reduction = c_div(
            abs(self.steering_permille) * TURN_REDUCTION, 1000
        )
        if self.steering_permille > 0:
            right_target -= reduction
        else:
            left_target -= reduction
        return max(left_target, 0), max(right_target, 0)

    def update_speed(self, result, left_actual, right_actual):
        if not self.track_valid:
            self.left_pid.reset()
            self.right_pid.reset()
            return 0, 0, 0, 0
        left_target, right_target = self.speed_targets(self.ramp_active)
        left_duty = self.left_pid.update(left_target, left_actual)
        right_duty = self.right_pid.update(right_target, right_actual)
        return left_target, right_target, left_duty, right_duty
