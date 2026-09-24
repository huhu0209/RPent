import math

import pytest

from lynsense_webots_sim.geometry import (
    Pose2,
    angular_error,
    bearing_to_goal,
    normalize_angle,
    webots_motor_rate,
    wheel_rates,
)
from lynsense_webots_sim.state_machine import MotionProfile


def test_angle_normalization_and_shortest_error():
    assert normalize_angle(math.pi + 0.1) == pytest.approx(-math.pi + 0.1)
    assert angular_error(0.1, -0.1) == pytest.approx(-0.2)
    assert angular_error(math.pi - 0.1, -math.pi + 0.1) == pytest.approx(0.2)


def test_bearing_uses_map_coordinates():
    pose = Pose2(0, 0, 0)
    assert bearing_to_goal(pose, Pose2(1, 1, 0)) == pytest.approx(math.pi / 4)


def test_wheel_rates_are_bounded_and_ordered_for_left_turn():
    profile = MotionProfile()
    left, right = wheel_rates(2.0, 1.5, profile)
    assert abs(left) <= profile.max_wheel_rate_radps + 1e-9
    assert abs(right) <= profile.max_wheel_rate_radps + 1e-9
    # Positive yaw is CCW; the right wheel must spin faster for the same axis sign.
    assert abs(right) > abs(left)


def test_negative_turn_reverses_differential_wheel_order():
    profile = MotionProfile()
    left, right = wheel_rates(0.0, -1.0, profile)
    assert left > 0.0
    assert right < 0.0
    assert left > right


def test_webots_motor_axis_preserves_logical_rate_sign():
    assert webots_motor_rate(1.25) == pytest.approx(1.25)
    assert webots_motor_rate(0.0) == 0.0
