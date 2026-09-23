from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

from robots.lynsense_real_box.site_profile import (
    SiteProfileError,
    load_site_profile,
    site_profile_hash,
    validate_site_profile,
)


def interface(
    role: str,
    kind: str = "topic",
    name: str | None = None,
    type_name: str = "example/Type",
) -> dict:
    return {
        "role": role,
        "kind": kind,
        "name": name or f"/{role.replace('_', '/')}",
        "type": type_name,
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
            interface("chassis_nav", "action", type_name="example/NavToPose"),
            interface("chassis_move", "action", type_name="example/MoveDistance"),
            interface("waist_control", "action"),
            interface("dual_arm_controller", "action"),
            interface("left_gripper", "action"),
            interface("right_gripper", "action"),
            interface(
                "dual_arm_grasp_release",
                "action",
                type_name="example/BoxPhase",
            ),
            interface(
                "box_perception_trigger",
                "service",
                type_name="std_srvs/srv/Trigger",
            ),
        ],
        "navigation_goals": {
            "pickup": "搬箱子1",
            "placement": "放箱子1_1",
        },
        "move_transitions": [
            {
                "name": "retreat_after_pick",
                "from_state": "box_grasped",
                "to_state": "carrying",
                "distance_m": -0.6,
                "angle_deg": 0.0,
            },
            {
                "name": "turn_after_pick",
                "from_state": "carrying",
                "to_state": "carrying",
                "distance_m": 0.0,
                "angle_deg": 90.0,
            },
            {
                "name": "retreat_after_place",
                "from_state": "box_released",
                "to_state": "withdrawn",
                "distance_m": -0.6,
                "angle_deg": 0.0,
            },
        ],
        "waist_profiles": [
            {"name": "pick_ready", "height_mm": 200.0},
            {"name": "safe", "height_mm": 200.0},
        ],
        "dual_arm_configs": [
            {"name": "pick_ready"},
            {"name": "place_ready"},
            {"name": "safe"},
        ],
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


def test_valid_profile_normalizes_and_hashes_stably(tmp_path: Path):
    source = valid_profile()
    profile = validate_site_profile(source)
    assert profile.profile_id == "robot1-single-box-v1"
    assert profile.ros_domain_id == 3
    assert profile.mode == "dry_run"
    assert len(profile.interfaces) == 17
    assert [item.role for item in profile.interfaces] == [
        item["role"] for item in source["interfaces"]
    ]
    assert profile.navigation_goals["pickup"] == "搬箱子1"
    assert [item.name for item in profile.waist_profiles] == [
        "pick_ready",
        "safe",
    ]

    with pytest.raises((AttributeError, TypeError)):
        profile.mode = "live"

    digest = site_profile_hash(profile)
    assert len(digest) == 64
    assert digest == site_profile_hash(validate_site_profile(valid_profile()))
    reordered = json.dumps(valid_profile(), ensure_ascii=False, sort_keys=True)
    assert digest == site_profile_hash(
        validate_site_profile(json.loads(reordered)),
    )

    path = tmp_path / "site.json"
    path.write_text(json.dumps(valid_profile()), encoding="utf-8")
    assert load_site_profile(path) == profile


def test_live_mode_is_represented_but_requires_controller_evidence():
    source = valid_profile()
    source["mode"] = "live"
    source["controller_review"].update(
        status="site_confirmed",
        safety_evidence=["approved-robot-one-inventory"],
        reviewer="site-reviewer",
    )
    source["calibration"]["status"] = "site_confirmed"
    profile = validate_site_profile(source)
    assert profile.mode == "live"
    assert any(
        item.role == "dual_arm_grasp_release" for item in profile.interfaces
    )


def rejected_mutations() -> list[tuple[str, object]]:
    def change(*path: object, value: object = None) -> object:
        def mutation(profile: dict) -> None:
            item = profile
            for key in path[:-1]:
                item = item[key]  # type: ignore[index]
            item[path[-1]] = value  # type: ignore[index]

        return mutation

    return [
        ("empty profile id", lambda p: p.update(profile_id="")),
        ("wrong version", lambda p: p.update(version=2)),
        ("wrong domain", lambda p: p.update(ros_domain_id=42)),
        ("bad mode", lambda p: p.update(mode="rehearsal")),
        ("extra root key", lambda p: p.update(extra_field="forbidden")),
        ("duplicate interface", lambda p: p["interfaces"].append(p["interfaces"][0].copy())),
        ("empty interfaces", lambda p: p["interfaces"].clear()),
        ("missing action", lambda p: p["interfaces"].remove(
            next(i for i in p["interfaces"] if i["role"] == "dual_arm_controller")
        )),
        ("missing feedback", lambda p: p["interfaces"].remove(
            next(i for i in p["interfaces"] if i["role"] == "left_gripper_feedback")
        )),
        ("bad interface kind", change("interfaces", 0, "kind", value="topic_stream")),
        ("bad interface direction", change("interfaces", 0, "direction", value="publish")),
        ("empty interface name", change("interfaces", 0, "name", value="")),
        ("empty interface type", change("interfaces", 0, "type", value="")),
        ("navigation extra", lambda p: p["navigation_goals"].update(extra="home")),
        ("nonfinite move", lambda p: p["move_transitions"][0].update(distance_m=math.nan)),
        ("duplicate move", lambda p: p["move_transitions"].append(p["move_transitions"][0].copy())),
        ("duplicate waist", lambda p: p["waist_profiles"].append(
            {"name": "pick_ready", "height_mm": 300.0}
        )),
        ("nonfinite waist", lambda p: p["waist_profiles"][0].update(height_mm=math.inf)),
        ("duplicate arm config", lambda p: p["dual_arm_configs"].append({"name": "safe"})),
        ("bad gripper commands", lambda p: p["gripper_commands"].append("half")),
        ("bad grasp kind", lambda p: p["grasp_evidence"].update(kind="invented_by_model")),
        ("bad release kind", lambda p: p["release_evidence"].update(kind="invented_by_model")),
        ("empty grasp signal", lambda p: p["grasp_evidence"].update(left_signal="")),
        ("bad review", lambda p: p["controller_review"].update(status="unreviewed")),
        ("zero budget", lambda p: p["action_budgets"].update(navigation_s=0.0)),
        ("zero feedback timeout", lambda p: p["gripper_timing"].update(feedback_timeout_s=0.0)),
        ("empty perception frame", lambda p: p["perception"].update(frame="")),
        ("zero perception timeout", lambda p: p["perception"].update(result_timeout_s=0.0)),
        ("unknown calibration", lambda p: p["calibration"].update(status="unknown")),
        ("zero payload", lambda p: p["limits"].update(max_payload_kg=0.0)),
        ("negative tolerance", lambda p: p["placement"].update(position_tolerance_m=-0.1)),
        ("nonfinite pose", lambda p: p["placement"]["target_pose"].update(x=math.nan)),
        ("inverted region", lambda p: p["placement"]["supported_region"].update(min_x=99.0)),
        ("bad hold policy", lambda p: p["failure"].update(partially_held_box="model_decides")),
        ("zero freshness", lambda p: p["freshness"].update(robot_state_max_age_s=0.0)),
    ]


@pytest.mark.parametrize(("label", "mutation"), rejected_mutations(), ids=[item[0] for item in rejected_mutations()])
def test_invalid_profiles_are_rejected(label: str, mutation):
    source = valid_profile()
    mutation(source)
    with pytest.raises(SiteProfileError):
        validate_site_profile(source)


def test_oversized_integer_is_a_contract_error():
    source = valid_profile()
    source["move_transitions"][0]["distance_m"] = 10**10000
    with pytest.raises(SiteProfileError):
        validate_site_profile(source)


def test_live_without_site_evidence_is_rejected():
    source = valid_profile()
    source["mode"] = "live"
    with pytest.raises(SiteProfileError):
        validate_site_profile(source)


def test_non_object_source_is_rejected():
    with pytest.raises(SiteProfileError):
        validate_site_profile([])  # type: ignore[arg-type]
