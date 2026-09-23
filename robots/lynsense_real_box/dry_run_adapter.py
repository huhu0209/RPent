from __future__ import annotations

import copy
import math
from collections.abc import Mapping
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any

from robots.lynsense_real_box.site_profile import (
    GraspReleaseEvidence,
    MoveTransition,
    NamedProfile,
    RosInterface,
    SiteProfile,
)


@dataclass(frozen=True, slots=True)
class _ScriptedFailure:
    method: str
    result: dict[str, Any]


def _finite(value: Any, label: str) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    try:
        number = float(value)
    except (OverflowError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    return number


def _pose(value: Any) -> dict[str, float] | None:
    if not isinstance(value, dict) or set(value) != {
        "x",
        "y",
        "z",
        "yaw_rad",
    }:
        return None
    result: dict[str, float] = {}
    for key in ("x", "y", "z", "yaw_rad"):
        number = _finite(value[key], f"pose.{key}")
        if number is None:
            return None
        result[key] = number
    return result


def _evidence(value: GraspReleaseEvidence) -> dict[str, str]:
    return {
        "kind": value.kind,
        "left_signal": value.left_signal,
        "right_signal": value.right_signal,
    }


class OfflineDryRunAdapter:
    """Offline execution of the complete guard sequence without ROS transport."""

    def __init__(
        self,
        profile: SiteProfile,
        robot_state: dict | None = None,
        box_pose: dict | None = None,
        placement_result: dict | None = None,
        now_s: float = 1000.0,
    ) -> None:
        self._profile = profile
        self._robot_state = copy.deepcopy(robot_state)
        self._box_pose = copy.deepcopy(box_pose)
        self._placement_result = copy.deepcopy(placement_result)
        self._now_s = now_s
        self._closed = False
        self._connected = False
        self._scripted_failure: _ScriptedFailure | None = None
        self.calls: list[dict[str, Any]] = []

        self._navigation_goals: Mapping[str, str] = MappingProxyType(
            dict(profile.navigation_goals)
        )
        self._move_transitions: Mapping[str, MoveTransition] = MappingProxyType(
            {item.name: item for item in profile.move_transitions}
        )
        self._waist_profiles: Mapping[str, NamedProfile] = MappingProxyType(
            {item.name: item for item in profile.waist_profiles}
        )
        self._dual_arm_configs: Mapping[str, NamedProfile] = MappingProxyType(
            {item.name: item for item in profile.dual_arm_configs}
        )
        self._gripper_commands: Mapping[str, str] = MappingProxyType(
            {item: item for item in profile.gripper_commands}
        )
        self._interfaces: Mapping[str, RosInterface] = MappingProxyType(
            {item.role: item for item in profile.interfaces}
        )

    def connect(self) -> dict[str, Any]:
        if self._closed:
            return {"status": "closed"}
        if self._profile.mode != "dry_run":
            return {"status": "rejected", "reason": "not_dry_run"}
        if self._profile.calibration.status != "offline_fixture_confirmed":
            return {"status": "rejected", "reason": "calibration_invalid"}
        self._connected = True
        return {
            "status": "ok",
            "mode": "dry_run",
            "profile_id": "robot1-single-box-v1",
        }

    def read_state(self) -> dict[str, Any]:
        if self._closed:
            return {"status": "closed"}
        reason = self._robot_state_reason()
        if reason is not None:
            return {"status": "rejected", "reason": reason}
        return {"status": "ok", "robot_state": copy.deepcopy(self._robot_state)}

    def detect_box(self) -> dict[str, Any]:
        if self._closed:
            return {"status": "closed"}
        reason = self._perception_reason()
        if reason is not None:
            return {"status": "rejected", "reason": reason}
        assert self._box_pose is not None
        return {
            "status": "ok",
            "frame": self._profile.perception.frame,
            "received_at": self._now_s,
            "pose": copy.deepcopy(self._box_pose["pose"]),
        }

    def navigate(self, goal_name: str) -> dict[str, Any]:
        return self._motion(
            "navigate",
            "chassis_nav",
            lambda: {"goal": self._navigation_goals[goal_name]},
            profile_known=goal_name in self._navigation_goals,
        )

    def move_distance(self, profile_name: str) -> dict[str, Any]:
        def request() -> dict[str, Any]:
            item = self._move_transitions[profile_name]
            return {"distance_m": item.distance_m, "angle_deg": item.angle_deg}

        return self._motion(
            "move_distance",
            "chassis_move",
            request,
            profile_known=profile_name in self._move_transitions,
        )

    def move_waist(self, profile_name: str) -> dict[str, Any]:
        def request() -> dict[str, Any]:
            return {"height_mm": self._waist_profiles[profile_name].height_mm}

        return self._motion(
            "move_waist",
            "waist_control",
            request,
            profile_known=profile_name in self._waist_profiles,
        )

    def move_dual_arms(self, profile_name: str) -> dict[str, Any]:
        def request() -> dict[str, Any]:
            return {"config": self._dual_arm_configs[profile_name].name}

        return self._motion(
            "move_dual_arms",
            "dual_arm_controller",
            request,
            profile_known=profile_name in self._dual_arm_configs,
        )

    def set_dual_grippers(self, command: str) -> dict[str, Any]:
        if self._closed:
            return {"status": "closed"}
        reason = self._physical_dispatch_reason()
        if reason is not None:
            return self._rejection("set_dual_grippers", "left_gripper", {}, reason)
        if command not in self._gripper_commands:
            return self._rejection(
                "set_dual_grippers", "left_gripper", {}, "unknown_profile"
            )
        request = {"command": command}
        reason = self._robot_state_reason()
        if reason is not None:
            return self._rejection(
                "set_dual_grippers", "left_gripper", request, reason
            )
        self._record("set_dual_grippers", "left_gripper", request)
        self._record("set_dual_grippers", "right_gripper", request)
        return self._scripted_or_success("set_dual_grippers")

    def pick_box(self) -> dict[str, Any]:
        request: dict[str, Any] = {
            "box_action": "pick",
            "left_gripper": "held",
            "right_gripper": "held",
            "grasp_evidence": self._profile.grasp_evidence,
        }
        if self._closed:
            return {"status": "closed"}
        reason = self._physical_dispatch_reason()
        if reason is not None:
            return self._rejection("pick_box", "dual_arm_grasp_release", {}, reason)
        reason = self._robot_state_reason() or self._perception_reason()
        if reason is not None:
            return self._rejection(
                "pick_box", "dual_arm_grasp_release", request, reason
            )
        self._record("pick_box", "dual_arm_grasp_release", request)
        return self._scripted_pick_result("pick_box")

    def place_box(self) -> dict[str, Any]:
        request: dict[str, Any] = {
            "box_action": "place",
            "left_gripper": "released",
            "right_gripper": "released",
            "release_evidence": self._profile.release_evidence,
            "placement": self._profile.placement,
        }
        if self._closed:
            return {"status": "closed"}
        reason = self._physical_dispatch_reason()
        if reason is not None:
            return self._rejection("place_box", "dual_arm_grasp_release", {}, reason)
        reason = self._robot_state_reason() or self._perception_reason()
        if reason is None:
            reason = self._placement_reason()
        if reason is not None:
            return self._rejection(
                "place_box", "dual_arm_grasp_release", request, reason
            )
        self._record("place_box", "dual_arm_grasp_release", request)
        return self._scripted_or_success("place_box")

    def stop_motion(self) -> dict[str, Any]:
        if self._closed:
            return {"status": "closed"}
        reason = self._physical_dispatch_reason()
        if reason is not None:
            return self._rejection("stop_motion", "chassis_move", {}, reason)
        self._record("stop_motion", "chassis_move", {})
        return {
            "status": "best_effort",
            "estop": False,
            "stop_timeout_s": self._profile.failure["stop_timeout_s"],
        }

    def close(self) -> None:
        self._closed = True

    def fail_next(self, method_name: str, result: dict[str, Any]) -> None:
        allowed = {
            "navigate",
            "move_distance",
            "move_waist",
            "move_dual_arms",
            "set_dual_grippers",
            "pick_box",
            "place_box",
        }
        if method_name not in allowed:
            raise ValueError("fail_next does not support this method")
        if not isinstance(result, dict):
            raise ValueError("failure result must be a dict")
        if result.get("status") != "failed" or not isinstance(
            result.get("failure_kind"), str
        ):
            raise ValueError("failure result must have failed status and kind")
        if self._scripted_failure is not None:
            raise ValueError("a scripted failure is already pending")
        self._scripted_failure = _ScriptedFailure(method_name, result)

    def _motion(
        self,
        method: str,
        role: str,
        request_builder,
        *,
        profile_known: bool,
    ) -> dict[str, Any]:
        if self._closed:
            return {"status": "closed"}
        reason = self._physical_dispatch_reason()
        if reason is not None:
            return self._rejection(method, role, {}, reason)
        if not profile_known:
            return self._rejection(method, role, {}, "unknown_profile")
        request = request_builder()
        reason = self._robot_state_reason()
        if reason is not None:
            return self._rejection(method, role, request, reason)
        self._record(method, role, request)
        return self._scripted_or_success(method)

    def _record(self, method: str, role: str, request: dict[str, Any]) -> None:
        self.calls.append(
            {
                "method": method,
                "interface_role": role,
                "interface_name": self._interfaces[role].name,
                "request": copy.deepcopy(request),
            }
        )

    def _rejection(
        self,
        method: str,
        role: str,
        request: dict[str, Any],
        reason: str,
    ) -> dict[str, Any]:
        self.calls.append(
            {
                "method": method,
                "interface_role": role,
                "interface_name": self._interfaces[role].name,
                "request": copy.deepcopy(request),
                "dispatched": False,
                "reason": reason,
            }
        )
        return {"status": "rejected", "reason": reason}

    def _robot_state_reason(self) -> str | None:
        state = self._robot_state
        if state is None:
            return "robot_state_missing"
        required = {
            "age_s",
            "healthy",
            "mode",
            "left_arm_error",
            "right_arm_error",
        }
        if not required.issubset(state):
            return "robot_state_invalid"
        age_s = _finite(state["age_s"], "age_s")
        if age_s is None or age_s < 0.0:
            return "robot_state_invalid"
        if age_s > self._profile.freshness["robot_state_max_age_s"]:
            return "robot_state_stale"
        if state["healthy"] is not True:
            return "robot_unhealthy"
        if state["mode"] != "dry_run":
            return "mode_mismatch"
        if state["left_arm_error"] is not False:
            return "arm_error"
        if state["right_arm_error"] is not False:
            return "arm_error"
        return None

    def _physical_dispatch_reason(self) -> str | None:
        if self._profile.mode != "dry_run":
            return "not_dry_run"
        if self._profile.calibration.status != "offline_fixture_confirmed":
            return "calibration_invalid"
        if not self._connected:
            return "not_connected"
        return None

    def _perception_reason(self) -> str | None:
        pose = self._box_pose
        if pose is None:
            return "perception_missing"
        if "transform_valid" not in pose:
            return "transform_invalid"
        required = {"age_s", "frame", "transform_valid", "pose"}
        if not required.issubset(pose):
            return "perception_invalid"
        age_s = _finite(pose["age_s"], "age_s")
        raw_age = pose["age_s"]
        if age_s is None and (
            not isinstance(raw_age, bool)
            and isinstance(raw_age, (int, float))
            and not math.isfinite(float(raw_age))
        ):
            return "perception_stale"
        if age_s is None or age_s < 0.0:
            return "perception_invalid"
        if pose["frame"] != self._profile.perception.frame:
            return "wrong_frame"
        if pose["transform_valid"] is not True:
            return "transform_invalid"
        if age_s > self._profile.freshness["perception_max_age_s"]:
            return "perception_stale"
        if _pose(pose["pose"]) is None:
            return "perception_invalid"
        return None

    def _placement_reason(self) -> str | None:
        result = self._placement_result
        required = {
            "pose",
            "left_gripper",
            "right_gripper",
            "release_evidence",
            "settle_elapsed_s",
            "max_observed_speed_m_s",
        }
        if not isinstance(result, dict) or not required.issubset(result):
            return "incomplete_placement_result"
        pose = _pose(result["pose"])
        if pose is None:
            return "placement_pose_invalid"
        region = self._profile.placement.supported_region
        if not (
            region.min_x <= pose["x"] <= region.max_x
            and region.min_y <= pose["y"] <= region.max_y
        ):
            return "outside_supported_region"
        target = self._profile.placement.target_pose
        position_error = math.sqrt(
            (pose["x"] - target.x) ** 2
            + (pose["y"] - target.y) ** 2
            + (pose["z"] - target.z) ** 2
        )
        if position_error > self._profile.placement.position_tolerance_m:
            return "position_tolerance"
        if abs(pose["yaw_rad"] - target.yaw_rad) > (
            self._profile.placement.yaw_tolerance_rad
        ):
            return "yaw_tolerance"
        for side in ("left", "right"):
            gripper = result[f"{side}_gripper"]
            if not isinstance(gripper, dict) or gripper.get("state") != "open":
                return "gripper_not_open"
            error = _finite(
                gripper.get("position_error"), f"{side}_position_error"
            )
            if error is None or error < 0.0:
                return "placement_result_invalid"
        if result["release_evidence"] != _evidence(
            self._profile.release_evidence
        ):
            return "release_evidence_mismatch"
        settle = _finite(result["settle_elapsed_s"], "settle_elapsed_s")
        if settle is None or settle < self._profile.placement.settle_s:
            return "settle_time"
        speed = _finite(
            result["max_observed_speed_m_s"], "max_observed_speed_m_s"
        )
        if speed is None or speed > self._profile.placement.stability_max_speed_m_s:
            return "stability_speed"
        return None

    def _scripted_or_success(self, method: str) -> dict[str, Any]:
        if self._scripted_failure is None:
            return {"status": "dry_run"}
        scripted = self._scripted_failure
        if scripted.method != method:
            return {"status": "dry_run"}
        self._scripted_failure = None
        result = copy.deepcopy(scripted.result)
        if scripted.result.get("failure_kind") == "dual_arm_disagreement":
            result.setdefault(
                "left_result", {"status": "failed", "reason": "result_missing"}
            )
            result.setdefault(
                "right_result", {"status": "failed", "reason": "result_missing"}
            )
        if scripted.result.get("failure_kind") == "gripper_disagreement":
            result.setdefault(
                "left_feedback", {"state": "unknown", "reason": "feedback_missing"}
            )
            result.setdefault(
                "right_feedback", {"state": "unknown", "reason": "feedback_missing"}
            )
        return result

    def _scripted_pick_result(self, method: str) -> dict[str, Any]:
        if self._scripted_failure is None:
            return {"status": "dry_run"}
        scripted = self._scripted_failure
        if scripted.method != method:
            return {"status": "dry_run"}
        self._scripted_failure = None
        result = copy.deepcopy(scripted.result)
        left = result.get("left_gripper", {})
        right = result.get("right_gripper", {})
        if (
            isinstance(left, dict) and left.get("state") == "held"
        ) or (
            isinstance(right, dict) and right.get("state") == "held"
        ):
            result["box_hazard"] = True
        return result
