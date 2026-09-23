from __future__ import annotations

import copy
import math
from dataclasses import replace

import pytest

from robots.lynsense_real_box.site_profile import (
    Calibration,
    MoveTransition,
    validate_site_profile,
)
from robots.lynsense_real_box.dry_run_adapter import OfflineDryRunAdapter


def interface(role: str, kind: str = "topic") -> dict:
    return {
        "role": role,
        "kind": kind,
        "name": f"/{role.replace('_', '/')}",
        "type": "example/Type",
        "direction": "subscribe" if kind == "topic" else "call",
    }


def valid_profile() -> dict:
    return {
        "profile_id": "robot1-single-box-v1",
        "version": 1,
        "ros_domain_id": 3,
        "mode": "dry_run",
        "interfaces": [
            interface("left_arm_state"),
            interface("right_arm_state"),
            interface("waist_state"),
            interface("chassis_state"),
            interface("localization_state"),
            interface("left_gripper_feedback"),
            interface("right_gripper_feedback"),
            interface("box_perception_pose"),
            interface("box_perception_status"),
            interface("chassis_nav", "action"),
            interface("chassis_move", "action"),
            interface("waist_control", "action"),
            interface("dual_arm_controller", "action"),
            interface("left_gripper", "action"),
            interface("right_gripper", "action"),
            interface("dual_arm_grasp_release", "action"),
            interface("box_perception_trigger", "service"),
        ],
        "navigation_goals": {"pickup": "搬箱子1", "placement": "放箱子1_1"},
        "move_transitions": [
            {
                "name": "retreat_after_pick",
                "from_state": "box_grasped",
                "to_state": "carrying",
                "distance_m": -0.6,
                "angle_deg": 0.0,
            }
        ],
        "waist_profiles": [{"name": "pick_ready", "height_mm": 200.0}],
        "dual_arm_configs": [{"name": "pick_ready"}, {"name": "place_ready"}],
        "gripper_commands": ["open", "close"],
        "grasp_evidence": {
            "kind": "reviewed_controller_signal",
            "left_signal": "held",
            "right_signal": "held",
        },
        "release_evidence": {
            "kind": "reviewed_controller_signal",
            "left_signal": "released",
            "right_signal": "released",
        },
        "controller_review": {
            "status": "offline_fixture_only",
            "review_id": "phase-1-offline-contract",
            "safety_evidence": [],
            "reviewer": None,
        },
        "action_budgets": {
            "navigation_s": 120.0,
            "chassis_move_s": 30.0,
            "waist_s": 20.0,
            "dual_arms_s": 60.0,
            "grippers_s": 10.0,
            "pick_place_s": 120.0,
        },
        "gripper_timing": {
            "command_timeout_s": 5.0,
            "feedback_timeout_s": 2.0,
            "position_tolerance": 0.05,
        },
        "perception": {
            "frame": "offline_box_fixture",
            "trigger_timeout_s": 5.0,
            "result_timeout_s": 15.0,
        },
        "calibration": {
            "status": "offline_fixture_confirmed",
            "perception_to_robot_transform": "offline_fixture_transform",
        },
        "limits": {
            "max_linear_speed_m_s": 0.2,
            "max_yaw_speed_rad_s": 0.5,
            "max_payload_kg": 1.0,
        },
        "placement": {
            "target_pose": {
                "x": 2.810559,
                "y": -2.734170,
                "z": 0.1085,
                "yaw_rad": 0.006126,
            },
            "supported_region": {
                "min_x": 2.70,
                "max_x": 2.92,
                "min_y": -2.84,
                "max_y": -2.63,
            },
            "position_tolerance_m": 0.08,
            "yaw_tolerance_rad": 0.0872665,
            "settle_s": 2.0,
            "stability_max_speed_m_s": 0.01,
        },
        "failure": {
            "partially_held_box": "hold_grippers",
            "stop_timeout_s": 2.0,
        },
        "freshness": {
            "robot_state_max_age_s": 0.5,
            "perception_max_age_s": 1.0,
        },
    }


PROFILE = validate_site_profile(valid_profile())
LIVE_PROFILE = validate_site_profile(
    {
        **valid_profile(),
        "mode": "live",
        "calibration": {
            "status": "site_confirmed",
            "perception_to_robot_transform": "offline_fixture_transform",
        },
        "controller_review": {
            "status": "site_confirmed",
            "review_id": "phase-1-live-review",
            "safety_evidence": ["approved-robot-one-inventory"],
            "reviewer": "site-reviewer",
        },
    }
)


def robot_state() -> dict:
    return {
        "age_s": 0.1,
        "healthy": True,
        "mode": "dry_run",
        "left_arm_error": False,
        "right_arm_error": False,
    }


def box_pose() -> dict:
    return {
        "age_s": 0.2,
        "frame": "offline_box_fixture",
        "transform_valid": True,
        "pose": {"x": 1.0, "y": -2.0, "z": 0.1, "yaw_rad": 0.0},
    }


def placement_result() -> dict:
    return {
        "pose": {
            "x": 2.81,
            "y": -2.73,
            "z": 0.108,
            "yaw_rad": 0.006,
        },
        "left_gripper": {"state": "open", "position_error": 0.01},
        "right_gripper": {"state": "open", "position_error": 0.01},
        "release_evidence": {
            "kind": "reviewed_controller_signal",
            "left_signal": "released",
            "right_signal": "released",
        },
        "settle_elapsed_s": 2.1,
        "max_observed_speed_m_s": 0.0,
    }


def adapter(**changes):
    fixtures = {
        "robot_state": robot_state(),
        "box_pose": box_pose(),
        "placement_result": placement_result(),
    }
    fixtures.update(changes)
    item = OfflineDryRunAdapter(PROFILE, **fixtures)
    assert item.connect()["status"] == "ok"
    return item


def call_physical_methods(item) -> list[dict]:
    return [
        item.navigate("pickup"),
        item.move_distance("retreat_after_pick"),
        item.move_waist("pick_ready"),
        item.move_dual_arms("pick_ready"),
        item.set_dual_grippers("open"),
        item.pick_box(),
        item.place_box(),
        item.stop_motion(),
    ]


def intended_call(method: str, role: str, request: dict) -> dict:
    return {
        "method": method,
        "interface_role": role,
        "interface_name": next(
            item.name for item in PROFILE.interfaces if item.role == role
        ),
        "request": request,
    }


def test_connect_reports_profile_without_recording_a_call():
    item = adapter()
    assert item.connect() == {
        "status": "ok",
        "mode": "dry_run",
        "profile_id": "robot1-single-box-v1",
    }
    assert item.calls == []


def test_read_state_returns_an_isolated_copy_of_supplied_state():
    supplied = robot_state()
    supplied["pose"] = {"x": [1.0]}
    item = OfflineDryRunAdapter(PROFILE, robot_state=supplied, now_s=1000.0)
    result = item.read_state()
    result["robot_state"]["pose"]["x"].append(2.0)
    assert supplied["pose"]["x"] == [1.0]
    assert result["robot_state"]["age_s"] == 0.1
    assert item.calls == []


def test_detect_box_supplies_frame_and_receipt_time():
    pose = box_pose()
    item = OfflineDryRunAdapter(PROFILE, box_pose=pose, now_s=1234.5)
    result = item.detect_box()
    assert result == {
        "status": "ok",
        "frame": "offline_box_fixture",
        "received_at": 1234.5,
        "pose": pose["pose"],
    }
    result["pose"]["x"] = 99.0
    assert pose["pose"]["x"] == 1.0
    assert item.calls == []


def test_motion_precondition_guards_reject_without_an_intended_request():
    missing = OfflineDryRunAdapter(PROFILE)
    missing.connect()
    assert missing.move_waist("pick_ready") == {
        "status": "rejected",
        "reason": "robot_state_missing",
    }

    stale = OfflineDryRunAdapter(
        PROFILE,
        robot_state={**robot_state(), "age_s": 0.51},
        now_s=1000.0,
    )
    stale.connect()
    assert stale.move_waist("pick_ready") == {
        "status": "rejected",
        "reason": "robot_state_stale",
    }

    unhealthy = OfflineDryRunAdapter(
        PROFILE,
        robot_state={**robot_state(), "healthy": False},
    )
    unhealthy.connect()
    assert unhealthy.move_waist("pick_ready")["reason"] == "robot_unhealthy"

    arm_error = OfflineDryRunAdapter(
        PROFILE,
        robot_state={**robot_state(), "right_arm_error": True},
    )
    arm_error.connect()
    assert arm_error.move_waist("pick_ready")["reason"] == "arm_error"

    assert all(item.get("dispatched") is False for item in missing.calls)
    assert all(item.get("dispatched") is False for item in stale.calls)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("age_s", "0.1"),
        ("age_s", math.nan),
        ("healthy", "yes"),
        ("mode", "live"),
        ("left_arm_error", "no"),
    ],
)
def test_invalid_robot_state_is_rejected(field, value):
    item = OfflineDryRunAdapter(
        PROFILE,
        robot_state={**robot_state(), field: value},
        box_pose=box_pose(),
    )
    item.connect()
    result = item.move_distance("retreat_after_pick")
    assert result["status"] == "rejected"
    assert item.calls[-1]["dispatched"] is False


@pytest.mark.parametrize(
    ("mutation", "reason"),
    [
        (lambda p: p.pop("transform_valid"), "transform_invalid"),
        (lambda p: p.update(frame="wrong_frame"), "wrong_frame"),
        (lambda p: p.update(age_s=math.inf), "perception_stale"),
    ],
)
def test_invalid_perception_is_rejected(mutation, reason):
    pose = box_pose()
    mutation(pose)
    item = OfflineDryRunAdapter(
        PROFILE,
        robot_state=robot_state(),
        box_pose=pose,
    )
    item.connect()
    assert item.pick_box() == {"status": "rejected", "reason": reason}
    assert item.calls[-1]["dispatched"] is False


def test_every_motion_method_records_exact_intended_request():
    item = adapter()
    methods_and_requests = [
        ("navigate", "pickup", "chassis_nav", {"goal": "搬箱子1"}),
        (
            "move_distance",
            "retreat_after_pick",
            "chassis_move",
            {"distance_m": -0.6, "angle_deg": 0.0},
        ),
        ("move_waist", "pick_ready", "waist_control", {"height_mm": 200.0}),
        (
            "move_dual_arms",
            "pick_ready",
            "dual_arm_controller",
            {"config": "pick_ready"},
        ),
        ("set_dual_grippers", "open", "right_gripper", {"command": "open"}),
        ("set_dual_grippers", "close", "right_gripper", {"command": "close"}),
    ]
    for method, argument, role, request in methods_and_requests:
        if method == "navigate":
            result = item.navigate(argument)
        elif method == "move_distance":
            result = item.move_distance(argument)
        elif method == "move_waist":
            result = item.move_waist(argument)
        elif method == "move_dual_arms":
            result = item.move_dual_arms(argument)
        else:
            result = item.set_dual_grippers(argument)
        assert result == {"status": "dry_run"}
        assert item.calls[-1] == intended_call(method, role, request)
    assert item.calls[-4] == intended_call(
        "set_dual_grippers", "left_gripper", {"command": "open"}
    )


def test_unknown_profile_names_are_rejected_without_intended_request():
    item = adapter()
    assert item.navigate("home") == {
        "status": "rejected",
        "reason": "unknown_profile",
    }
    assert item.move_distance("unknown")["reason"] == "unknown_profile"
    assert item.move_waist("unknown")["reason"] == "unknown_profile"
    assert item.move_dual_arms("unknown")["reason"] == "unknown_profile"
    assert item.set_dual_grippers("half")["reason"] == "unknown_profile"
    assert len(item.calls) == 5
    assert all(call["dispatched"] is False for call in item.calls)


def test_fail_next_is_consumed_once():
    item = adapter()
    failure = {"status": "failed", "failure_kind": "timeout"}
    item.fail_next("move_distance", failure)
    assert item.move_distance("retreat_after_pick") == failure
    assert item.calls[-1] == intended_call(
        "move_distance",
        "chassis_move",
        {"distance_m": -0.6, "angle_deg": 0.0},
    )
    assert item.move_distance("retreat_after_pick") == {"status": "dry_run"}


def test_live_profile_rejects_every_physical_method_without_connection():
    item = OfflineDryRunAdapter(
        LIVE_PROFILE,
        robot_state=robot_state(),
        box_pose=box_pose(),
        placement_result=placement_result(),
    )
    assert item.connect() == {"status": "rejected", "reason": "not_dry_run"}
    assert call_physical_methods(item) == [
        {"status": "rejected", "reason": "not_dry_run"}
    ] * 8
    assert len(item.calls) == 8
    assert all(call["dispatched"] is False for call in item.calls)


def test_dry_profile_requires_successful_connection_before_dispatch():
    item = OfflineDryRunAdapter(
        PROFILE,
        robot_state=robot_state(),
        box_pose=box_pose(),
        placement_result=placement_result(),
    )
    assert call_physical_methods(item) == [
        {"status": "rejected", "reason": "not_connected"}
    ] * 8
    assert all(call["dispatched"] is False for call in item.calls)
    assert item.connect()["status"] == "ok"
    assert item.navigate("pickup") == {"status": "dry_run"}


def test_invalid_calibration_is_independent_of_connection():
    item = OfflineDryRunAdapter(
        replace(
            PROFILE,
            calibration=Calibration(
                status="unknown",
                perception_to_robot_transform="offline_fixture_transform",
            ),
        ),
        robot_state=robot_state(),
    )
    assert item.connect() == {
        "status": "rejected",
        "reason": "calibration_invalid",
    }
    assert item.move_waist("pick_ready") == {
        "status": "rejected",
        "reason": "calibration_invalid",
    }
    assert item.calls[-1]["dispatched"] is False


def test_robot_state_mode_must_be_dry_run():
    item = adapter(
        robot_state={**robot_state(), "mode": "live"},
    )
    assert item.move_waist("pick_ready") == {
        "status": "rejected",
        "reason": "mode_mismatch",
    }
    assert item.calls[-1]["dispatched"] is False


def test_scripted_failure_is_bound_to_its_named_method():
    item = adapter()
    failure = {"status": "failed", "failure_kind": "localization_loss"}
    item.fail_next("navigate", failure)
    assert item.move_waist("pick_ready") == {"status": "dry_run"}
    assert item.navigate("pickup") == failure
    assert item.navigate("placement") == {"status": "dry_run"}


@pytest.mark.parametrize(
    "failure_kind", ["timeout", "canceled", "cancel_unknown"]
)
def test_scripted_motion_outcomes_preserve_failure_kinds(failure_kind):
    item = adapter()
    item.fail_next(
        "move_distance",
        {"status": "failed", "failure_kind": failure_kind},
    )
    result = item.move_distance("retreat_after_pick")
    assert result == {"status": "failed", "failure_kind": failure_kind}
    assert result["status"] != "dry_run"


def test_dual_arm_disagreement_has_separate_results():
    item = adapter()
    item.fail_next(
        "move_dual_arms",
        {
            "status": "failed",
            "failure_kind": "dual_arm_disagreement",
            "left_result": {"status": "succeeded"},
            "right_result": {"status": "failed"},
        },
    )
    result = item.move_dual_arms("pick_ready")
    assert result["failure_kind"] == "dual_arm_disagreement"
    assert result["left_result"]["status"] == "succeeded"
    assert result["right_result"]["status"] == "failed"
    assert result["status"] == "failed"


def test_one_sided_dual_arm_completion_is_not_success():
    item = adapter()
    item.fail_next(
        "move_dual_arms",
        {
            "status": "failed",
            "failure_kind": "dual_arm_disagreement",
            "left_result": {"status": "succeeded"},
        },
    )
    result = item.move_dual_arms("pick_ready")
    assert result["status"] == "failed"
    assert result["right_result"]["status"] == "failed"


def test_gripper_disagreement_has_separate_feedback_states():
    item = adapter()
    item.fail_next(
        "set_dual_grippers",
        {
            "status": "failed",
            "failure_kind": "gripper_disagreement",
            "left_feedback": {"state": "open"},
            "right_feedback": {"state": "closed"},
        },
    )
    result = item.set_dual_grippers("open")
    assert result["failure_kind"] == "gripper_disagreement"
    assert result["left_feedback"] == {"state": "open"}
    assert result["right_feedback"] == {"state": "closed"}


def test_missing_grasp_evidence_sets_box_hazard_when_held():
    item = adapter()
    item.fail_next(
        "pick_box",
        {
            "status": "failed",
            "failure_kind": "missing_grasp_evidence",
            "left_gripper": {"state": "released"},
            "right_gripper": {"state": "held"},
        },
    )
    result = item.pick_box()
    assert result["failure_kind"] == "missing_grasp_evidence"
    assert result["box_hazard"] is True


def test_place_failure_variants_are_preserved():
    for failure_kind in (
        "release_verification_failed",
        "placement_settle_failed",
        "placement_stability_failed",
    ):
        item = adapter()
        item.fail_next(
            "place_box",
            {"status": "failed", "failure_kind": failure_kind},
        )
        assert item.place_box() == {
            "status": "failed",
            "failure_kind": failure_kind,
        }


@pytest.mark.parametrize(
    ("mutation", "reason"),
    [
        (lambda p: p.pop("settle_elapsed_s"), "incomplete_placement_result"),
        (lambda p: p["pose"].update(x=2.69), "outside_supported_region"),
        (lambda p: p["pose"].update(y=-2.821), "position_tolerance"),
        (lambda p: p["pose"].update(yaw_rad=0.096), "yaw_tolerance"),
        (lambda p: p["left_gripper"].update(state="closed"), "gripper_not_open"),
        (lambda p: p["right_gripper"].update(state="held"), "gripper_not_open"),
        (
            lambda p: p["release_evidence"].update(left_signal="held"),
            "release_evidence_mismatch",
        ),
        (lambda p: p.update(settle_elapsed_s=1.99), "settle_time"),
        (lambda p: p.update(max_observed_speed_m_s=0.02), "stability_speed"),
    ],
)
def test_place_box_acceptance_gates(mutation, reason):
    evidence = placement_result()
    mutation(evidence)
    item = OfflineDryRunAdapter(
        PROFILE,
        robot_state=robot_state(),
        box_pose=box_pose(),
        placement_result=evidence,
    )
    assert item.connect()["status"] == "ok"
    assert item.place_box() == {"status": "rejected", "reason": reason}
    assert item.calls[-1]["dispatched"] is False


def test_localization_loss_while_carrying_is_scripted_not_success():
    item = adapter()
    item.fail_next(
        "navigate",
        {"status": "failed", "failure_kind": "localization_loss"},
    )
    result = item.navigate("placement")
    assert result == {"status": "failed", "failure_kind": "localization_loss"}


def test_stop_motion_is_best_effort_and_not_estop():
    item = adapter()
    assert item.stop_motion() == {
        "status": "best_effort",
        "estop": False,
        "stop_timeout_s": 2.0,
    }
    assert item.calls[-1] == intended_call("stop_motion", "chassis_move", {})


def test_close_is_idempotent_and_subsequent_calls_are_closed():
    item = adapter()
    assert item.close() is None
    assert item.close() is None
    assert item.connect() == {"status": "closed"}
    assert item.read_state() == {"status": "closed"}
    assert item.detect_box() == {"status": "closed"}
    assert item.navigate("pickup") == {"status": "closed"}
    assert item.move_distance("retreat_after_pick") == {"status": "closed"}
    assert item.stop_motion() == {"status": "closed"}


def test_composite_requests_match_the_reviewed_contract():
    item = adapter()
    item.pick_box()
    assert item.calls[-1] == intended_call(
        "pick_box",
        "dual_arm_grasp_release",
        {
            "box_action": "pick",
            "left_gripper": "held",
            "right_gripper": "held",
            "grasp_evidence": PROFILE.grasp_evidence,
        },
    )
    item.place_box()
    assert item.calls[-1] == intended_call(
        "place_box",
        "dual_arm_grasp_release",
        {
            "box_action": "place",
            "left_gripper": "released",
            "right_gripper": "released",
            "release_evidence": PROFILE.release_evidence,
            "placement": PROFILE.placement,
        },
    )
