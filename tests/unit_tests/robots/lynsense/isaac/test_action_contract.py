# Copyright 2026 The RPent Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import annotations

import json
import math
from dataclasses import replace

import pytest

from robots.lynsense.isaac.action_contract import (
    ACTION_JOINT_NAMES,
    DriveConfiguration,
    DriveGains,
    DriveLimits,
    JointActionPlan,
    JointSample,
    TransformSample,
    base_motion_to_dict,
    evaluate_base_motion,
    evaluate_joint_action,
    joint_trajectories_to_dict,
    plan_joint_action,
    validate_step_state,
)
from robots.lynsense.isaac.contracts import IsaacProbeError


def test_drive_configuration_separates_gains_from_physical_limits() -> None:
    configuration = DriveConfiguration(
        gains=DriveGains(stiffness=200.0, damping=16.6),
        limits=DriveLimits(
            max_effort_nm=200.0,
            max_velocity_rad_s=0.314,
        ),
        verified=True,
    )
    assert configuration.to_dict() == {
        "gains": {"stiffness": 200.0, "damping": 16.6},
        "limits": {"max_effort_nm": 200.0, "max_velocity_rad_s": 0.314},
        "verified": True,
    }


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("stiffness", -1.0),
        ("stiffness", math.nan),
        ("damping", math.inf),
        ("max_effort_nm", -0.1),
        ("max_velocity_rad_s", math.nan),
    ],
)
def test_drive_configuration_rejects_invalid_numeric_values(
    field: str, value: float
) -> None:
    gains = DriveGains(
        stiffness=value if field == "stiffness" else 200.0,
        damping=value if field == "damping" else 16.6,
    )
    limits = DriveLimits(
        max_effort_nm=value if field == "max_effort_nm" else 200.0,
        max_velocity_rad_s=(value if field == "max_velocity_rad_s" else 0.314),
    )
    with pytest.raises(IsaacProbeError, match="non-negative and finite"):
        DriveConfiguration(gains=gains, limits=limits)


def test_action_target_is_initial_plus_one_tenth() -> None:
    plan = plan_joint_action("left_joint1", 0.2, -1.0, 1.0)
    assert plan == JointActionPlan(
        joint_name="left_joint1",
        initial_position=0.2,
        target_position=0.30000000000000004,
        lower_limit=-1.0,
        upper_limit=1.0,
    )


@pytest.mark.parametrize(
    ("lower", "upper", "initial"),
    [(-1.0, 1.0, 0.95), (-1.0, 1.0, 1.0), (-1.0, 1.0, -2.0)],
)
def test_out_of_limit_or_invalid_target_fails(
    lower: float, upper: float, initial: float
) -> None:
    with pytest.raises(IsaacProbeError, match="target"):
        plan_joint_action("left_joint1", initial, lower, upper)


@pytest.mark.parametrize(
    ("lower", "upper", "initial"),
    [
        (math.nan, 1.0, 0.0),
        (-1.0, math.inf, 0.0),
        (-1.0, math.nan, math.nan),
        (1.0, -1.0, 0.0),
    ],
)
def test_plan_rejects_nonfinite_or_reversed_limits(
    lower: float, upper: float, initial: float
) -> None:
    with pytest.raises(IsaacProbeError, match="limits"):
        plan_joint_action("left_joint1", initial, lower, upper)


def _trajectory(target: float, final_time: float = 1.0) -> tuple[JointSample, ...]:
    return (
        JointSample(0.0, 0.0, 0.0),
        JointSample(0.2, 0.05, 0.2),
        JointSample(0.6, target, 0.01),
        JointSample(final_time, target, 0.01),
    )


def _plans() -> dict[str, JointActionPlan]:
    return {
        name: plan_joint_action(name, 0.0, -1.0, 1.0)
        for name in ("left_joint1", "right_joint1")
    }


def test_complete_settled_action_passes() -> None:
    plans = _plans()
    trajectories = {name: _trajectory(0.1) for name in plans}
    result = evaluate_joint_action(trajectories, plans)
    assert result["passed"] is True
    assert result["final_time_s"] == 1.0
    assert result["joints"]["left_joint1"]["progress_rad"] == pytest.approx(0.1)


def test_insufficient_progress_fails() -> None:
    plans = _plans()
    trajectories = {
        "left_joint1": _trajectory(0.1),
        "right_joint1": _trajectory(0.04),
    }
    with pytest.raises(IsaacProbeError, match="right_joint1.*progress"):
        evaluate_joint_action(trajectories, plans)


def test_final_window_speed_and_overshoot_fail() -> None:
    plans = _plans()
    trajectories = {name: _trajectory(0.1) for name in plans}
    moving = dict(trajectories)
    moving["left_joint1"] = list(_trajectory(0.1))
    moving["left_joint1"][-1] = JointSample(1.0, 0.1, 0.2)
    with pytest.raises(IsaacProbeError, match="speed"):
        evaluate_joint_action(
            {name: tuple(trajectory) for name, trajectory in moving.items()},
            plans,
        )

    overshoot = dict(trajectories)
    overshoot["left_joint1"] = list(_trajectory(0.1))
    overshoot["left_joint1"][-2] = JointSample(0.6, 0.13, 0.01)
    with pytest.raises(IsaacProbeError, match="overshoot"):
        evaluate_joint_action(
            {name: tuple(trajectory) for name, trajectory in overshoot.items()},
            plans,
        )


def test_intermediate_position_must_stay_within_plan_limits() -> None:
    plans = _plans()
    trajectories = {name: list(_trajectory(0.1)) for name in plans}
    trajectories["left_joint1"][1] = JointSample(0.2, 1.1, 0.2)
    with pytest.raises(IsaacProbeError, match="position.*plan limits"):
        evaluate_joint_action(
            {name: tuple(trajectory) for name, trajectory in trajectories.items()},
            plans,
        )


def test_intermediate_speed_must_stay_below_action_limit() -> None:
    plans = _plans()
    trajectories = {name: list(_trajectory(0.1)) for name in plans}
    trajectories["right_joint1"][1] = JointSample(0.2, 0.05, 0.315)
    with pytest.raises(IsaacProbeError, match="speed"):
        evaluate_joint_action(
            {name: tuple(trajectory) for name, trajectory in trajectories.items()},
            plans,
        )


def test_base_motion_translation_and_rotation_limits() -> None:
    identity = TransformSample(
        0.0,
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )
    within = TransformSample(
        1.0,
        (0.0005, 0.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )
    assert evaluate_base_motion((identity, within))["passed"] is True

    outside = TransformSample(
        1.0,
        (0.0011, 0.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )
    with pytest.raises(IsaacProbeError, match="translation"):
        evaluate_base_motion((identity, outside))


def test_base_rotation_is_relative_to_initial_orientation() -> None:
    initial = TransformSample(
        0.0,
        (0.0, 0.0, 0.0),
        (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)),
    )
    within = TransformSample(
        1.0,
        (0.0, 0.0, 0.0),
        (
            0.0,
            0.0,
            math.sin((math.pi / 2 + 0.001) / 2),
            math.cos((math.pi / 2 + 0.001) / 2),
        ),
    )
    result = evaluate_base_motion((initial, within))
    assert result["passed"] is True
    assert result["max_rotation_rad"] == pytest.approx(0.001)

    outside = TransformSample(
        1.0,
        (0.0, 0.0, 0.0),
        (
            0.0,
            0.0,
            math.sin((math.pi / 2 + 0.01) / 2),
            math.cos((math.pi / 2 + 0.01) / 2),
        ),
    )
    with pytest.raises(IsaacProbeError, match="rotation"):
        evaluate_base_motion((initial, outside))


def test_large_finite_quaternions_do_not_overflow_normalization() -> None:
    initial = TransformSample(
        0.0,
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 1e308),
    )
    opposite_w = TransformSample(
        1.0,
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, -1e308),
    )
    result = evaluate_base_motion((initial, opposite_w))
    assert result["passed"] is True
    assert result["max_rotation_rad"] == pytest.approx(0.0)

    one_degree = math.radians(1.0)
    moved = TransformSample(
        1.0,
        (0.0, 0.0, 0.0),
        (
            0.0,
            0.0,
            math.sin(one_degree / 2.0) * 1e308,
            math.cos(one_degree / 2.0) * 1e308,
        ),
    )
    with pytest.raises(IsaacProbeError, match="rotation"):
        evaluate_base_motion((initial, moved))


def test_nonfinite_action_and_base_state_fail() -> None:
    plans = _plans()
    trajectories = {name: _trajectory(0.1) for name in plans}
    bad = dict(trajectories)
    bad["left_joint1"] = list(_trajectory(0.1))
    bad["left_joint1"][-1] = JointSample(1.0, math.nan, 0.0)
    with pytest.raises(IsaacProbeError, match="finite"):
        evaluate_joint_action(
            {name: tuple(trajectory) for name, trajectory in bad.items()},
            plans,
        )

    identity = TransformSample(0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    bad_base = TransformSample(1.0, (0.0, math.inf, 0.0), (0.0, 0.0, 0.0, 1.0))
    with pytest.raises(IsaacProbeError, match="finite"):
        evaluate_base_motion((identity, bad_base))


def test_streaming_step_check_fails_immediately() -> None:
    plans = _plans()
    samples = {
        "left_joint1": JointSample(0.1, 0.05, 0.1),
        "right_joint1": JointSample(0.1, math.nan, 0.1),
    }
    base = (
        TransformSample(0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        TransformSample(0.1, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
    )
    with pytest.raises(IsaacProbeError, match="finite"):
        validate_step_state(plans, samples, base)

    samples["right_joint1"] = JointSample(0.1, 0.05, 0.1)
    outside_base = base + (
        TransformSample(0.2, (0.0011, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
    )
    with pytest.raises(IsaacProbeError, match="translation"):
        validate_step_state(plans, samples, outside_base)


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("target_position", 0.09, "target"),
        ("joint_name", "wrong_joint1", "joint name"),
        ("initial_position", math.nan, "finite"),
        ("target_position", math.inf, "finite"),
        ("lower_limit", math.nan, "finite"),
        ("upper_limit", math.nan, "finite"),
    ],
)
def test_both_gates_reject_invalid_action_plans(
    field: str, value: object, message: str
) -> None:
    plans = _plans()
    plans["left_joint1"] = replace(plans["left_joint1"], **{field: value})
    trajectories = {name: _trajectory(0.1) for name in plans}
    with pytest.raises(IsaacProbeError, match=message):
        evaluate_joint_action(trajectories, plans)

    samples = {name: JointSample(0.1, 0.05, 0.1) for name in ACTION_JOINT_NAMES}
    base = (
        TransformSample(0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        TransformSample(0.1, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
    )
    with pytest.raises(IsaacProbeError, match=message):
        validate_step_state(plans, samples, base)


def test_trajectory_initial_position_must_match_plan() -> None:
    plans = _plans()
    trajectories = {name: _trajectory(0.1) for name in plans}
    mismatched = list(trajectories["left_joint1"])
    mismatched[0] = JointSample(0.0, 0.01, 0.0)
    trajectories["left_joint1"] = tuple(mismatched)
    with pytest.raises(IsaacProbeError, match="initial position"):
        evaluate_joint_action(trajectories, plans)


def test_empty_or_singleton_joint_trajectory_is_rejected() -> None:
    plans = _plans()
    with pytest.raises(IsaacProbeError, match="at least two"):
        trajectories = dict.fromkeys(plans, ())
        evaluate_joint_action(trajectories, plans)
    with pytest.raises(IsaacProbeError, match="at least two"):
        trajectories = dict.fromkeys(plans, (JointSample(0.0, 0.0, 0.0),))
        evaluate_joint_action(trajectories, plans)


def test_unordered_or_duplicate_joint_times_are_rejected() -> None:
    plans = _plans()
    unordered = (
        JointSample(0.0, 0.0, 0.0),
        JointSample(0.2, 0.05, 0.2),
        JointSample(1.0, 0.1, 0.01),
    )
    duplicate = (
        JointSample(0.0, 0.0, 0.0),
        JointSample(1.0, 0.1, 0.01),
        JointSample(1.0, 0.1, 0.01),
    )
    evaluate_joint_action(dict.fromkeys(plans, unordered), plans)
    with pytest.raises(IsaacProbeError, match="strictly increasing"):
        evaluate_joint_action(dict.fromkeys(plans, duplicate), plans)


def test_duration_over_ten_seconds_is_rejected() -> None:
    plans = _plans()
    trajectory = (
        JointSample(0.0, 0.0, 0.0),
        JointSample(10.0001, 0.1, 0.01),
    )
    with pytest.raises(IsaacProbeError, match="duration"):
        evaluate_joint_action(dict.fromkeys(plans, trajectory), plans)


def test_missing_final_half_second_window_is_rejected() -> None:
    plans = _plans()
    trajectory = (
        JointSample(0.0, 0.0, 0.0),
        JointSample(0.4, 0.1, 0.01),
    )
    with pytest.raises(IsaacProbeError, match="final settle window"):
        evaluate_joint_action(dict.fromkeys(plans, trajectory), plans)


def test_base_motion_requires_ordered_finite_time_and_nonzero_quaternions() -> None:
    first = TransformSample(0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    with pytest.raises(IsaacProbeError, match="strictly increasing"):
        evaluate_base_motion(
            (
                first,
                TransformSample(0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            )
        )
    with pytest.raises(IsaacProbeError, match="finite"):
        evaluate_base_motion(
            (
                first,
                TransformSample(math.nan, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            )
        )
    with pytest.raises(IsaacProbeError, match="quaternion norm"):
        evaluate_base_motion(
            (
                first,
                TransformSample(1.0, (0.0, 0.0, 0.0), (1e-10, 0.0, 0.0, 0.0)),
            )
        )


def test_contract_serialization_contains_sequences_not_tuples() -> None:
    trajectories = {
        "left_joint1": _trajectory(0.1),
        "right_joint1": _trajectory(0.1),
    }
    joint_document = joint_trajectories_to_dict(trajectories)
    base_document = base_motion_to_dict(
        (
            TransformSample(0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            TransformSample(0.1, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        )
    )
    assert list(joint_document) == ["left_joint1", "right_joint1"]
    assert joint_document["left_joint1"][0] == {
        "time_s": 0.0,
        "position_rad": 0.0,
        "velocity_rad_s": 0.0,
    }
    assert base_document[1] == {
        "time_s": 0.1,
        "translation": [0.0, 0.0, 0.0],
        "rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
    }
    json.dumps(joint_document, allow_nan=False)
    json.dumps(base_document, allow_nan=False)
