from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import pytest

from robots.lynsense_real_box.atomic_profile import (
    AtomicProfileError,
    atomic_profile_hash,
    load_atomic_capability_profile,
    validate_atomic_capability_profile,
)


def valid_atomic_profile() -> dict[str, Any]:
    return {
        "profile_id": "robot1-atomic-offline-v1",
        "version": 1,
        "mode": "dry_run",
        "joint_axes": 6,
        "joint_limits_deg": [
            [-10.0, 10.0],
            [-20.0, 20.0],
            [-10.0, 10.0],
            [-20.0, 20.0],
            [-10.0, 10.0],
            [-20.0, 20.0],
        ],
        "dual_arm_profiles": {
            "safe": {
                "left_joints_deg": [
                    [0.0, 1.0, -1.0, 2.0, -2.0, 0.0],
                    [1.0, 2.0, 0.0, 0.0, 0.0, 1.0],
                ],
                "right_joints_deg": [
                    [0.0, -1.0, 1.0, -2.0, 2.0, 0.0],
                    [1.0, 0.0, 0.0, 0.0, 0.0, -1.0],
                ],
                "tolerance_deg": 0.5,
                "max_joint_step_deg": 2.0,
                "start_tolerance_deg": 2.0,
                "timeout_s": 10.0,
            }
        },
        "waist_profiles": {
            "safe": {
                "position_mm": 200.0,
                "tolerance_mm": 3.0,
                "timeout_s": 10.0,
            }
        },
        "gripper_profiles": {
            "open": {
                "position_rad": 0.5,
                "tolerance_rad": 0.01,
                "timeout_s": 4.0,
            },
            "close": {
                "position_rad": 0.1,
                "tolerance_rad": 0.01,
                "timeout_s": 4.0,
            },
        },
        "chassis_profiles": {
            "pickup": {
                "kind": "nav_goal",
                "goal": "pickup",
                "timeout_s": 30.0,
            },
            "retreat": {
                "kind": "move_distance",
                "distance_m": -0.6,
                "angle_deg": 0.0,
                "timeout_s": 20.0,
            },
        },
        "review": {
            "status": "offline_fixture_only",
            "review_id": "atomic-offline-v1",
            "safety_evidence": [],
            "reviewer": None,
        },
        "force_gates": {
            "max_abs_force_n": [30.0, 30.0, 30.0],
            "max_abs_torque_nm": [3.0, 3.0, 3.0],
            "hold_s": 0.05,
        },
        "budgets": {
            "chassis_s": 30.0,
            "waist_s": 10.0,
            "dual_arms_s": 10.0,
            "grippers_s": 4.0,
            "stop_s": 2.0,
        },
        "robot_state_max_age_s": 0.5,
        "max_chassis_distance_m": 1.0,
        "max_chassis_angle_deg": 90.0,
    }


def live_profile() -> dict[str, Any]:
    source = valid_v2_profile()
    source["mode"] = "live"
    source["review"] = {
        "status": "site_confirmed",
        "review_id": "robot1-atomic-live-v1",
        "safety_evidence": ["reviewed-lynrotcontrol-atomic-profile"],
        "reviewer": "site-reviewer",
    }
    return source


def valid_v2_profile() -> dict[str, Any]:
    source = valid_atomic_profile()
    source["profile_id"] = "robot1-atomic-offline-v2"
    source["version"] = 2
    source["chassis_profiles"]["pickup"]["carries_box"] = False
    source["chassis_profiles"]["retreat"]["carries_box"] = True
    source["runtime_binding"] = {
        "lynrotcontrol_instance": "robot-one-reviewed",
        "lynrotcontrol_config_sha256": "a" * 64,
        "execution_host": "offline-fixture-host",
        "ros_domain_id": 3,
        "source_artifacts": ["offline-runtime-fixture"],
    }
    source["chassis_runtime"] = {
        "kind": "bounded_odom_cmd_vel",
        "odom_topic": "/odom",
        "cmd_vel_topic": "/cmd_vel",
        "max_linear_speed_m_s": 0.15,
        "max_angular_speed_rad_s": 0.25,
        "max_linear_accel_m_s2": 0.1,
        "max_angular_accel_rad_s2": 0.2,
        "command_rate_hz": 20.0,
        "watchdog_timeout_s": 1.0,
        "stop_settle_timeout_s": 1.0,
        "zero_twist_confirm_samples": 3,
        "odom_max_age_s": 0.25,
        "source_artifacts": ["offline-chassis-fixture"],
    }
    source["gripper_runtime"] = {
        "failure_policy": "confirmed_stop_then_operator_review",
        "requires_stop_api": True,
        "stop_timeout_s": 1.0,
        "zero_motion_confirm_samples": 2,
        "source_artifacts": ["offline-gripper-fixture"],
    }
    source["perception_binding"] = {
        "pose_topic": "/industrial_box/pose_base",
        "status_topic": "/industrial_box/status",
        "trigger_service": "/industrial_box/trigger",
        "expected_frame": "base_link",
        "pose_max_age_s": 0.5,
        "trigger_timeout_s": 5.0,
        "result_deadline_s": 8.0,
        "source_artifacts": ["offline-perception-fixture"],
    }
    source["subsystem_freshness"] = {
        "left_arm_state_s": 0.4,
        "right_arm_state_s": 0.4,
        "left_arm_force_s": 0.3,
        "right_arm_force_s": 0.3,
        "left_gripper_s": 0.4,
        "right_gripper_s": 0.4,
        "waist_s": 0.4,
        "chassis_s": 0.25,
    }
    source["carry_guards"] = {
        "retreat": {
            "arm_profile": "safe",
            "gripper_command": "close",
            "box_control_status": "held",
            "force_monitor": "both_arms",
            "source_artifacts": ["offline-carry-fixture"],
        }
    }
    return source


def test_valid_profile_normalizes_all_site_values():
    profile = validate_atomic_capability_profile(valid_atomic_profile())
    assert profile.profile_id == "robot1-atomic-offline-v1"
    assert profile.joint_axes == 6
    assert profile.dual_arm_profiles["safe"].left_joints_deg == (
        (0.0, 1.0, -1.0, 2.0, -2.0, 0.0),
        (1.0, 2.0, 0.0, 0.0, 0.0, 1.0),
    )
    assert profile.dual_arm_profiles["safe"].right_joints_deg == (
        (0.0, -1.0, 1.0, -2.0, 2.0, 0.0),
        (1.0, 0.0, 0.0, 0.0, 0.0, -1.0),
    )
    assert profile.gripper_profiles["open"].position_rad == 0.5
    assert profile.gripper_profiles["close"].position_rad == 0.1
    assert profile.chassis_profiles["retreat"].distance_m == -0.6
    assert profile.review.status == "offline_fixture_only"


def test_profile_hash_is_stable_and_json_order_independent(tmp_path: Path):
    profile = validate_atomic_capability_profile(valid_atomic_profile())
    expected = atomic_profile_hash(profile)
    reordered = json.loads(
        json.dumps(valid_atomic_profile(), sort_keys=True, ensure_ascii=False)
    )
    assert atomic_profile_hash(
        validate_atomic_capability_profile(reordered)
    ) == expected

    path = tmp_path / "atomic-profile.json"
    path.write_text(
        json.dumps(valid_atomic_profile(), ensure_ascii=False),
        encoding="utf-8",
    )
    assert load_atomic_capability_profile(path) == profile


def test_joint_paths_must_match_axes_limits_and_step_bound():
    source = valid_atomic_profile()
    source["dual_arm_profiles"]["safe"]["left_joints_deg"] = [
        [0.0, 30.0, 0.0, 0.0, 0.0, 0.0]
    ]
    with pytest.raises(AtomicProfileError, match="outside joint limits"):
        validate_atomic_capability_profile(source)

    source = valid_atomic_profile()
    source["dual_arm_profiles"]["safe"]["start_tolerance_deg"] = 2.1
    with pytest.raises(AtomicProfileError, match="start_tolerance_deg"):
        validate_atomic_capability_profile(source)

    source = valid_atomic_profile()
    source["dual_arm_profiles"]["safe"]["left_joints_deg"] = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [2.1, 0.0, 0.0, 0.0, 0.0, 0.0],
    ]
    with pytest.raises(AtomicProfileError, match="max_joint_step"):
        validate_atomic_capability_profile(source)

    source = valid_atomic_profile()
    source["dual_arm_profiles"]["safe"]["right_joints_deg"] = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ]
    with pytest.raises(AtomicProfileError, match="equal path lengths"):
        validate_atomic_capability_profile(source)


def test_gripper_targets_must_be_distinct_and_nonoverlapping():
    source = valid_atomic_profile()
    source["gripper_profiles"]["close"]["position_rad"] = 0.5
    with pytest.raises(AtomicProfileError, match="exceed close"):
        validate_atomic_capability_profile(source)

    source = valid_atomic_profile()
    source["gripper_profiles"]["open"]["tolerance_rad"] = 0.21
    source["gripper_profiles"]["close"]["tolerance_rad"] = 0.21
    with pytest.raises(AtomicProfileError, match="tolerances must not overlap"):
        validate_atomic_capability_profile(source)


def test_chassis_profiles_are_bounded_and_discriminated():
    source = valid_atomic_profile()
    source["chassis_profiles"]["retreat"]["distance_m"] = -1.1
    with pytest.raises(AtomicProfileError, match="distance_m exceeds"):
        validate_atomic_capability_profile(source)

    source = valid_atomic_profile()
    source["chassis_profiles"]["retreat"]["kind"] = "cmd_vel"
    with pytest.raises(AtomicProfileError, match="nav_goal or move_distance"):
        validate_atomic_capability_profile(source)

    source = valid_atomic_profile()
    source["chassis_profiles"]["retreat"].update(
        {"distance_m": 0.0, "angle_deg": 0.0}
    )
    with pytest.raises(AtomicProfileError, match="move distance or angle"):
        validate_atomic_capability_profile(source)


@pytest.mark.parametrize(
    "mutation",
    [
        lambda p: p.update(version=2),
        lambda p: p.update(joint_axes=1),
        lambda p: p.update(joint_axes=8),
        lambda p: p.update(joint_limits_deg=[[-10.0, 10.0]] * 5),
        lambda p: p["force_gates"].update(max_abs_force_n=[30.0, 30.0]),
        lambda p: p["force_gates"].update(max_abs_force_n=[30.0, 30.0, math.inf]),
        lambda p: p["budgets"].update(stop_s=0.0),
        lambda p: p.update(robot_state_max_age_s=-0.1),
        lambda p: p["dual_arm_profiles"].update(extra=None),
        lambda p: p.pop("chassis_profiles"),
    ],
)
def test_invalid_profiles_fail_closed(mutation):
    source = valid_atomic_profile()
    mutation(source)
    with pytest.raises(AtomicProfileError):
        validate_atomic_capability_profile(source)


def test_budgets_must_cover_profile_timeouts():
    source = valid_atomic_profile()
    source["budgets"]["dual_arms_s"] = 9.9
    with pytest.raises(AtomicProfileError, match="dual-arm budget"):
        validate_atomic_capability_profile(source)


def test_live_review_requires_site_evidence_and_reviewer():
    profile = validate_atomic_capability_profile(live_profile())
    assert profile.review.status == "site_confirmed"
    assert profile.review.reviewer == "site-reviewer"

    source = live_profile()
    source["review"]["safety_evidence"] = []
    with pytest.raises(AtomicProfileError, match="safety evidence"):
        validate_atomic_capability_profile(source)

    source = valid_atomic_profile()
    source["review"]["reviewer"] = "unreviewed"
    with pytest.raises(AtomicProfileError, match="dry_run review.reviewer"):
        validate_atomic_capability_profile(source)


def test_version_one_cannot_be_used_for_live() -> None:
    source = valid_atomic_profile()
    source["mode"] = "live"
    source["review"] = live_profile()["review"]

    with pytest.raises(AtomicProfileError, match="live profiles require version 2"):
        validate_atomic_capability_profile(source)


def test_v2_profile_normalizes_runtime_transport_and_carry_bindings() -> None:
    profile = validate_atomic_capability_profile(valid_v2_profile())

    assert profile.version == 2
    assert profile.runtime_binding is not None
    assert profile.runtime_binding.ros_domain_id == 3
    assert profile.chassis_runtime is not None
    assert profile.chassis_runtime.kind == "bounded_odom_cmd_vel"
    assert profile.gripper_runtime is not None
    assert profile.gripper_runtime.requires_stop_api is True
    assert profile.perception_binding is not None
    assert profile.perception_binding.expected_frame == "base_link"
    assert profile.subsystem_freshness is not None
    assert profile.subsystem_freshness.chassis_s == 0.25
    assert profile.carry_guards["retreat"].arm_profile == "safe"


def test_v2_hash_includes_runtime_and_transport_identity() -> None:
    source = valid_v2_profile()
    profile = validate_atomic_capability_profile(source)
    original_hash = atomic_profile_hash(profile)

    source["runtime_binding"]["lynrotcontrol_config_sha256"] = "b" * 64
    changed_hash = atomic_profile_hash(
        validate_atomic_capability_profile(source)
    )

    assert original_hash != changed_hash


@pytest.mark.parametrize(
    "mutation",
    [
        lambda p: p["runtime_binding"].update(
            lynrotcontrol_config_sha256="not-a-sha"
        ),
        lambda p: p["chassis_runtime"].update(
            max_angular_speed_rad_s=0.0,
            max_linear_speed_m_s=0.0,
        ),
        lambda p: p["chassis_runtime"].update(stop_settle_timeout_s=2.1),
        lambda p: p["gripper_runtime"].update(requires_stop_api=False),
        lambda p: p["gripper_runtime"].update(stop_timeout_s=2.1),
        lambda p: p["perception_binding"].update(result_deadline_s=4.0),
        lambda p: p["subsystem_freshness"].update(chassis_s=0.0),
        lambda p: p["carry_guards"]["retreat"].update(arm_profile="missing"),
        lambda p: p["carry_guards"]["retreat"].update(
            box_control_status="released"
        ),
        lambda p: p["carry_guards"]["retreat"].update(source_artifacts=[]),
        lambda p: p.pop("runtime_binding"),
        lambda p: p["carry_guards"].pop("retreat"),
        lambda p: p["carry_guards"].update(
            pickup=dict(p["carry_guards"]["retreat"])
        ),
    ],
)
def test_v2_runtime_and_safety_fields_fail_closed(mutation) -> None:
    source = valid_v2_profile()
    mutation(source)
    with pytest.raises(AtomicProfileError):
        validate_atomic_capability_profile(source)
