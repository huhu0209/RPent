from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any


class SiteProfileError(ValueError):
    """Raised when a Phase 1 site profile is not valid."""


@dataclass(frozen=True, slots=True)
class RosInterface:
    role: str
    kind: str
    name: str
    type_name: str
    direction: str


@dataclass(frozen=True, slots=True)
class MoveTransition:
    name: str
    from_state: str
    to_state: str
    distance_m: float
    angle_deg: float


@dataclass(frozen=True, slots=True)
class NamedProfile:
    name: str
    height_mm: float | None = None


@dataclass(frozen=True, slots=True)
class GraspReleaseEvidence:
    kind: str
    left_signal: str
    right_signal: str


@dataclass(frozen=True, slots=True)
class ControllerReview:
    status: str
    review_id: str
    safety_evidence: tuple[str, ...]
    reviewer: str | None


@dataclass(frozen=True, slots=True)
class ActionBudgets:
    navigation_s: float
    chassis_move_s: float
    waist_s: float
    dual_arms_s: float
    grippers_s: float
    pick_place_s: float


@dataclass(frozen=True, slots=True)
class GripperTiming:
    command_timeout_s: float
    feedback_timeout_s: float
    position_tolerance: float


@dataclass(frozen=True, slots=True)
class PerceptionContract:
    frame: str
    trigger_timeout_s: float
    result_timeout_s: float


@dataclass(frozen=True, slots=True)
class Calibration:
    status: str
    perception_to_robot_transform: str


@dataclass(frozen=True, slots=True)
class PhysicalLimits:
    max_linear_speed_m_s: float
    max_yaw_speed_rad_s: float
    max_payload_kg: float


@dataclass(frozen=True, slots=True)
class TargetPose:
    x: float
    y: float
    z: float
    yaw_rad: float


@dataclass(frozen=True, slots=True)
class SupportedRegion:
    min_x: float
    max_x: float
    min_y: float
    max_y: float


@dataclass(frozen=True, slots=True)
class PlacementAcceptance:
    target_pose: TargetPose
    supported_region: SupportedRegion
    position_tolerance_m: float
    yaw_tolerance_rad: float
    settle_s: float
    stability_max_speed_m_s: float


@dataclass(frozen=True, slots=True)
class SiteProfile:
    profile_id: str
    version: int
    ros_domain_id: int
    mode: str
    interfaces: tuple[RosInterface, ...]
    navigation_goals: Mapping[str, str]
    move_transitions: tuple[MoveTransition, ...]
    waist_profiles: tuple[NamedProfile, ...]
    dual_arm_configs: tuple[NamedProfile, ...]
    gripper_commands: tuple[str, ...]
    grasp_evidence: GraspReleaseEvidence
    release_evidence: GraspReleaseEvidence
    controller_review: ControllerReview
    action_budgets: ActionBudgets
    gripper_timing: GripperTiming
    perception: PerceptionContract
    calibration: Calibration
    limits: PhysicalLimits
    placement: PlacementAcceptance
    failure: Mapping[str, Any]
    freshness: Mapping[str, float]


_TOP_LEVEL_KEYS = frozenset(
    {
        "profile_id",
        "version",
        "ros_domain_id",
        "mode",
        "interfaces",
        "navigation_goals",
        "move_transitions",
        "waist_profiles",
        "dual_arm_configs",
        "gripper_commands",
        "grasp_evidence",
        "release_evidence",
        "controller_review",
        "action_budgets",
        "gripper_timing",
        "perception",
        "calibration",
        "limits",
        "placement",
        "failure",
        "freshness",
    }
)
_REQUIRED_ROLES = (
    "left_arm_state",
    "right_arm_state",
    "waist_state",
    "chassis_state",
    "localization_state",
    "left_gripper_feedback",
    "right_gripper_feedback",
    "box_perception_pose",
    "box_perception_status",
    "chassis_nav",
    "chassis_move",
    "waist_control",
    "dual_arm_controller",
    "left_gripper",
    "right_gripper",
    "dual_arm_grasp_release",
    "box_perception_trigger",
)
_INTERFACE_KEYS = frozenset({"role", "kind", "name", "type", "direction"})
_INTERFACE_DIRECTIONS = {"topic": "subscribe", "service": "call", "action": "call"}
_EVIDENCE_KEYS = frozenset({"kind", "left_signal", "right_signal"})
_EVIDENCE_KINDS = frozenset(
    {"contact", "force_current", "reviewed_controller_signal"}
)
_BUDGET_KEYS = frozenset(
    {
        "navigation_s",
        "chassis_move_s",
        "waist_s",
        "dual_arms_s",
        "grippers_s",
        "pick_place_s",
    }
)
_POSE_KEYS = frozenset({"x", "y", "z", "yaw_rad"})
_REGION_KEYS = frozenset({"min_x", "max_x", "min_y", "max_y"})


def _object(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise SiteProfileError(f"{label} must be an object")
    return value


def _exact_keys(value: Any, expected: frozenset[str], label: str) -> dict[str, Any]:
    item = _object(value, label)
    actual = item.keys()
    missing = expected - actual
    extra = actual - expected
    if missing or extra:
        raise SiteProfileError(
            f"{label} has missing keys {sorted(missing)} and extra keys {sorted(extra)}"
        )
    return item


def _string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise SiteProfileError(f"{label} must be a nonempty string")
    return value


def _finite(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise SiteProfileError(f"{label} must be numeric")
    try:
        result = float(value)
    except (OverflowError, ValueError) as error:
        raise SiteProfileError(f"{label} cannot be represented as a float") from error
    if not math.isfinite(result):
        raise SiteProfileError(f"{label} must be finite")
    return result


def _positive(value: Any, label: str) -> float:
    result = _finite(value, label)
    if result <= 0.0:
        raise SiteProfileError(f"{label} must be greater than zero")
    return result


def _string_list(value: Any, label: str) -> tuple[str, ...]:
    if not isinstance(value, list):
        raise SiteProfileError(f"{label} must be an array")
    return tuple(_string(item, f"{label}[{index}]") for index, item in enumerate(value))


def _unique_names(items: Any, label: str, value_key: str | None = None) -> tuple[NamedProfile, ...]:
    if not isinstance(items, list):
        raise SiteProfileError(f"{label} must be an array")
    names: set[str] = set()
    result: list[NamedProfile] = []
    for index, item in enumerate(items):
        path = f"{label}[{index}]"
        if value_key is None:
            if not isinstance(item, dict) or item.keys() != {"name"}:
                raise SiteProfileError(f"{path} must have only a name")
            name = _string(item["name"], f"{path}.name")
            result.append(NamedProfile(name=name))
        else:
            fields = _exact_keys(item, {"name", value_key}, path)
            name = _string(fields["name"], f"{path}.name")
            value = _positive(fields[value_key], f"{path}.{value_key}")
            result.append(NamedProfile(name=name, height_mm=value))
        if name in names:
            raise SiteProfileError(f"duplicate {label} name {name!r}")
        names.add(name)
    return tuple(result)


def _validate_interfaces(value: Any) -> tuple[RosInterface, ...]:
    if not isinstance(value, list):
        raise SiteProfileError("interfaces must be an array")
    by_role: dict[str, RosInterface] = {}
    for index, item in enumerate(value):
        fields = _exact_keys(item, _INTERFACE_KEYS, f"interfaces[{index}]")
        role = _string(fields["role"], f"interfaces[{index}].role")
        name = _string(fields["name"], f"interfaces[{index}].name")
        type_name = _string(fields["type"], f"interfaces[{index}].type")
        kind = fields["kind"]
        direction = fields["direction"]
        if kind not in _INTERFACE_DIRECTIONS:
            raise SiteProfileError(f"interfaces[{index}].kind is invalid")
        if direction != _INTERFACE_DIRECTIONS[kind]:
            raise SiteProfileError(f"interfaces[{index}].direction is invalid")
        if role in by_role:
            raise SiteProfileError(f"duplicate interface role {role!r}")
        by_role[role] = RosInterface(
            role=role,
            kind=kind,
            name=name,
            type_name=type_name,
            direction=direction,
        )
    missing = [role for role in _REQUIRED_ROLES if role not in by_role]
    if missing:
        raise SiteProfileError(f"missing interface roles {missing}")
    if len(by_role) != len(_REQUIRED_ROLES):
        extra = sorted(set(by_role) - set(_REQUIRED_ROLES))
        raise SiteProfileError(f"extra interface roles {extra}")
    return tuple(by_role[role] for role in _REQUIRED_ROLES)


def _validate_moves(value: Any) -> tuple[MoveTransition, ...]:
    if not isinstance(value, list):
        raise SiteProfileError("move_transitions must be an array")
    names: set[str] = set()
    result: list[MoveTransition] = []
    for index, item in enumerate(value):
        fields = _exact_keys(
            item,
            frozenset({"name", "from_state", "to_state", "distance_m", "angle_deg"}),
            f"move_transitions[{index}]",
        )
        name = _string(fields["name"], f"move_transitions[{index}].name")
        if name in names:
            raise SiteProfileError(f"duplicate move transition {name!r}")
        names.add(name)
        result.append(
            MoveTransition(
                name=name,
                from_state=_string(fields["from_state"], f"move_transitions[{index}].from_state"),
                to_state=_string(fields["to_state"], f"move_transitions[{index}].to_state"),
                distance_m=_finite(fields["distance_m"], f"move_transitions[{index}].distance_m"),
                angle_deg=_finite(fields["angle_deg"], f"move_transitions[{index}].angle_deg"),
            )
        )
    return tuple(result)


def _validate_evidence(value: Any, label: str) -> GraspReleaseEvidence:
    fields = _exact_keys(value, _EVIDENCE_KEYS, label)
    kind = _string(fields["kind"], f"{label}.kind")
    if kind not in _EVIDENCE_KINDS:
        raise SiteProfileError(f"{label}.kind is invalid")
    return GraspReleaseEvidence(
        kind=kind,
        left_signal=_string(fields["left_signal"], f"{label}.left_signal"),
        right_signal=_string(fields["right_signal"], f"{label}.right_signal"),
    )


def _positive_fields(
    value: Any,
    expected: frozenset[str],
    label: str,
) -> dict[str, float]:
    fields = _exact_keys(value, expected, label)
    return {key: _positive(fields[key], f"{label}.{key}") for key in expected}


def _public_document(profile: SiteProfile) -> dict[str, Any]:
    return {
        "profile_id": profile.profile_id,
        "version": profile.version,
        "ros_domain_id": profile.ros_domain_id,
        "mode": profile.mode,
        "interfaces": [
            {
                "role": item.role,
                "kind": item.kind,
                "name": item.name,
                "type": item.type_name,
                "direction": item.direction,
            }
            for item in profile.interfaces
        ],
        "navigation_goals": dict(profile.navigation_goals),
        "move_transitions": [
            {
                "name": item.name,
                "from_state": item.from_state,
                "to_state": item.to_state,
                "distance_m": item.distance_m,
                "angle_deg": item.angle_deg,
            }
            for item in profile.move_transitions
        ],
        "waist_profiles": [
            {"name": item.name, "height_mm": item.height_mm}
            for item in profile.waist_profiles
        ],
        "dual_arm_configs": [{"name": item.name} for item in profile.dual_arm_configs],
        "gripper_commands": list(profile.gripper_commands),
        "grasp_evidence": {
            "kind": profile.grasp_evidence.kind,
            "left_signal": profile.grasp_evidence.left_signal,
            "right_signal": profile.grasp_evidence.right_signal,
        },
        "release_evidence": {
            "kind": profile.release_evidence.kind,
            "left_signal": profile.release_evidence.left_signal,
            "right_signal": profile.release_evidence.right_signal,
        },
        "controller_review": {
            "status": profile.controller_review.status,
            "review_id": profile.controller_review.review_id,
            "safety_evidence": list(profile.controller_review.safety_evidence),
            "reviewer": profile.controller_review.reviewer,
        },
        "action_budgets": {
            "navigation_s": profile.action_budgets.navigation_s,
            "chassis_move_s": profile.action_budgets.chassis_move_s,
            "waist_s": profile.action_budgets.waist_s,
            "dual_arms_s": profile.action_budgets.dual_arms_s,
            "grippers_s": profile.action_budgets.grippers_s,
            "pick_place_s": profile.action_budgets.pick_place_s,
        },
        "gripper_timing": {
            "command_timeout_s": profile.gripper_timing.command_timeout_s,
            "feedback_timeout_s": profile.gripper_timing.feedback_timeout_s,
            "position_tolerance": profile.gripper_timing.position_tolerance,
        },
        "perception": {
            "frame": profile.perception.frame,
            "trigger_timeout_s": profile.perception.trigger_timeout_s,
            "result_timeout_s": profile.perception.result_timeout_s,
        },
        "calibration": {
            "status": profile.calibration.status,
            "perception_to_robot_transform": (
                profile.calibration.perception_to_robot_transform
            ),
        },
        "limits": {
            "max_linear_speed_m_s": profile.limits.max_linear_speed_m_s,
            "max_yaw_speed_rad_s": profile.limits.max_yaw_speed_rad_s,
            "max_payload_kg": profile.limits.max_payload_kg,
        },
        "placement": {
            "target_pose": {
                "x": profile.placement.target_pose.x,
                "y": profile.placement.target_pose.y,
                "z": profile.placement.target_pose.z,
                "yaw_rad": profile.placement.target_pose.yaw_rad,
            },
            "supported_region": {
                "min_x": profile.placement.supported_region.min_x,
                "max_x": profile.placement.supported_region.max_x,
                "min_y": profile.placement.supported_region.min_y,
                "max_y": profile.placement.supported_region.max_y,
            },
            "position_tolerance_m": profile.placement.position_tolerance_m,
            "yaw_tolerance_rad": profile.placement.yaw_tolerance_rad,
            "settle_s": profile.placement.settle_s,
            "stability_max_speed_m_s": profile.placement.stability_max_speed_m_s,
        },
        "failure": dict(profile.failure),
        "freshness": dict(profile.freshness),
    }


def validate_site_profile(source: dict[str, Any]) -> SiteProfile:
    item = _exact_keys(source, _TOP_LEVEL_KEYS, "site profile")

    profile_id = _string(item["profile_id"], "profile_id")
    if isinstance(item["version"], bool) or item["version"] != 1:
        raise SiteProfileError("version must be 1")
    if (
        isinstance(item["ros_domain_id"], bool)
        or item["ros_domain_id"] != 3
        or isinstance(item["ros_domain_id"], str)
    ):
        raise SiteProfileError("ros_domain_id must be 3")
    mode = _string(item["mode"], "mode")
    if mode not in {"dry_run", "live"}:
        raise SiteProfileError("mode must be dry_run or live")

    interfaces = _validate_interfaces(item["interfaces"])

    goals = _exact_keys(item["navigation_goals"], {"pickup", "placement"}, "navigation_goals")
    navigation_goals = MappingProxyType(
        {
            "pickup": _string(goals["pickup"], "navigation_goals.pickup"),
            "placement": _string(goals["placement"], "navigation_goals.placement"),
        }
    )

    moves = _validate_moves(item["move_transitions"])
    waist = _unique_names(item["waist_profiles"], "waist_profiles", "height_mm")
    arm_configs = _unique_names(item["dual_arm_configs"], "dual_arm_configs")
    commands = _string_list(item["gripper_commands"], "gripper_commands")
    if commands != ("open", "close"):
        raise SiteProfileError("gripper_commands must be ['open', 'close']")

    grasp = _validate_evidence(item["grasp_evidence"], "grasp_evidence")
    release = _validate_evidence(item["release_evidence"], "release_evidence")

    review = _exact_keys(
        item["controller_review"],
        frozenset({"status", "review_id", "safety_evidence", "reviewer"}),
        "controller_review",
    )
    review_status = _string(review["status"], "controller_review.status")
    reviewer = review["reviewer"]
    if mode == "live":
        if review_status != "site_confirmed":
            raise SiteProfileError("live controller_review must be site_confirmed")
        reviewer = _string(reviewer, "controller_review.reviewer")
        evidence = _string_list(
            review["safety_evidence"],
            "controller_review.safety_evidence",
        )
        if not evidence:
            raise SiteProfileError("live controller_review requires safety evidence")
    else:
        if review_status != "offline_fixture_only":
            raise SiteProfileError("dry_run controller_review must be offline_fixture_only")
        if reviewer is not None:
            raise SiteProfileError("dry_run controller_review.reviewer must be null")
        evidence = _string_list(
            review["safety_evidence"],
            "controller_review.safety_evidence",
        )
    controller_review = ControllerReview(
        status=review_status,
        review_id=_string(review["review_id"], "controller_review.review_id"),
        safety_evidence=evidence,
        reviewer=reviewer,
    )

    budgets = _positive_fields(item["action_budgets"], _BUDGET_KEYS, "action_budgets")
    timing = _positive_fields(
        item["gripper_timing"],
        frozenset({"command_timeout_s", "feedback_timeout_s", "position_tolerance"}),
        "gripper_timing",
    )

    perception_item = _exact_keys(
        item["perception"],
        frozenset({"frame", "trigger_timeout_s", "result_timeout_s"}),
        "perception",
    )
    perception = PerceptionContract(
        frame=_string(perception_item["frame"], "perception.frame"),
        trigger_timeout_s=_positive(
            perception_item["trigger_timeout_s"], "perception.trigger_timeout_s"
        ),
        result_timeout_s=_positive(
            perception_item["result_timeout_s"], "perception.result_timeout_s"
        ),
    )

    calibration_item = _exact_keys(
        item["calibration"],
        frozenset({"status", "perception_to_robot_transform"}),
        "calibration",
    )
    calibration_status = _string(calibration_item["status"], "calibration.status")
    if mode == "live" and calibration_status != "site_confirmed":
        raise SiteProfileError("live calibration must be site_confirmed")
    if mode == "dry_run" and calibration_status != "offline_fixture_confirmed":
        raise SiteProfileError(
            "dry_run calibration must be offline_fixture_confirmed"
        )
    calibration = Calibration(
        status=calibration_status,
        perception_to_robot_transform=_string(
            calibration_item["perception_to_robot_transform"],
            "calibration.perception_to_robot_transform",
        ),
    )

    limits = _positive_fields(
        item["limits"],
        frozenset({"max_linear_speed_m_s", "max_yaw_speed_rad_s", "max_payload_kg"}),
        "limits",
    )

    placement_item = _exact_keys(
        item["placement"],
        frozenset(
            {
                "target_pose",
                "supported_region",
                "position_tolerance_m",
                "yaw_tolerance_rad",
                "settle_s",
                "stability_max_speed_m_s",
            }
        ),
        "placement",
    )
    pose_fields = _exact_keys(placement_item["target_pose"], _POSE_KEYS, "target_pose")
    target_pose = TargetPose(
        **{key: _finite(pose_fields[key], f"target_pose.{key}") for key in _POSE_KEYS}
    )
    region_fields = _exact_keys(
        placement_item["supported_region"], _REGION_KEYS, "supported_region"
    )
    region_values = {
        key: _finite(region_fields[key], f"supported_region.{key}")
        for key in _REGION_KEYS
    }
    if region_values["min_x"] >= region_values["max_x"]:
        raise SiteProfileError("supported_region requires min_x < max_x")
    if region_values["min_y"] >= region_values["max_y"]:
        raise SiteProfileError("supported_region requires min_y < max_y")
    placement = PlacementAcceptance(
        target_pose=target_pose,
        supported_region=SupportedRegion(**region_values),
        position_tolerance_m=_positive(
            placement_item["position_tolerance_m"], "placement.position_tolerance_m"
        ),
        yaw_tolerance_rad=_positive(
            placement_item["yaw_tolerance_rad"], "placement.yaw_tolerance_rad"
        ),
        settle_s=_positive(placement_item["settle_s"], "placement.settle_s"),
        stability_max_speed_m_s=_positive(
            placement_item["stability_max_speed_m_s"],
            "placement.stability_max_speed_m_s",
        ),
    )

    failure_fields = _exact_keys(
        item["failure"],
        frozenset({"partially_held_box", "stop_timeout_s"}),
        "failure",
    )
    hold_policy = _string(failure_fields["partially_held_box"], "failure.partially_held_box")
    if hold_policy not in {"hold_grippers", "site_reviewed_open"}:
        raise SiteProfileError("failure.partially_held_box is invalid")
    failure = MappingProxyType(
        {
            "partially_held_box": hold_policy,
            "stop_timeout_s": _positive(failure_fields["stop_timeout_s"], "failure.stop_timeout_s"),
        }
    )
    freshness_values = _positive_fields(
        item["freshness"],
        frozenset({"robot_state_max_age_s", "perception_max_age_s"}),
        "freshness",
    )
    freshness = MappingProxyType(freshness_values)

    return SiteProfile(
        profile_id=profile_id,
        version=1,
        ros_domain_id=3,
        mode=mode,
        interfaces=interfaces,
        navigation_goals=navigation_goals,
        move_transitions=moves,
        waist_profiles=waist,
        dual_arm_configs=arm_configs,
        gripper_commands=commands,
        grasp_evidence=grasp,
        release_evidence=release,
        controller_review=controller_review,
        action_budgets=ActionBudgets(**budgets),
        gripper_timing=GripperTiming(**timing),
        perception=perception,
        calibration=calibration,
        limits=PhysicalLimits(**limits),
        placement=placement,
        failure=failure,
        freshness=freshness,
    )


def load_site_profile(path: Path) -> SiteProfile:
    if not isinstance(path, Path):
        raise SiteProfileError("path must be a Path")
    try:
        source = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise SiteProfileError(f"could not load site profile: {error}") from error
    return validate_site_profile(source)


def site_profile_hash(profile: SiteProfile) -> str:
    if not isinstance(profile, SiteProfile):
        raise SiteProfileError("profile must be a SiteProfile")
    canonical = json.dumps(
        _public_document(profile),
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    )
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()
