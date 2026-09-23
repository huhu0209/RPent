"""Pure-Python contract for the LynrotControl atomic capability boundary."""

from __future__ import annotations

from collections.abc import Mapping
import math
from typing import Any

from robots.lynsense_real_box.atomic_profile import AtomicCapabilityProfile


ATOMIC_TOOL_NAMES = (
    "read_capability_state",
    "detect_box",
    "move_chassis",
    "move_waist",
    "move_dual_arms",
    "set_dual_grippers",
    "stop_all",
    "finish",
)

ATOMIC_MOTION_TOOLS = frozenset(
    {
        "move_chassis",
        "move_waist",
        "move_dual_arms",
        "set_dual_grippers",
    }
)

ATOMIC_CAPABILITY_MODES = frozenset(
    {
        "idle",
        "perception_running",
        "chassis_moving",
        "waist_moving",
        "dual_arms_moving",
        "grippers_moving",
        "stopping",
        "operator_review",
        "operator_review_acknowledged",
        "closed",
    }
)

_READ_ONLY_TOOLS = frozenset({"read_capability_state"})
_SAFE_IDLE_TOOLS = frozenset({"detect_box", *ATOMIC_MOTION_TOOLS})
_ARGUMENT_TOOLS = frozenset(
    {
        "move_chassis",
        "move_waist",
        "move_dual_arms",
        "set_dual_grippers",
        "finish",
    }
)


def atomic_tool_specs(
    profile: AtomicCapabilityProfile,
) -> list[dict[str, Any]]:
    """Return the exact model-facing atomic capability schema."""

    chassis_profiles = profile.chassis_profiles.keys()
    properties: dict[str, dict[str, Any]] = {
        "move_chassis": {
            "profile_name": _enum_property(chassis_profiles)
        },
        "move_waist": {
            "profile_name": _enum_property(profile.waist_profiles.keys())
        },
        "move_dual_arms": {
            "profile_name": _enum_property(
                profile.dual_arm_profiles.keys()
            )
        },
        "set_dual_grippers": {
            "command": _enum_property(profile.gripper_profiles.keys())
        },
        "finish": {
            "status": _enum_property(("success", "failure"))
        },
    }
    descriptions = {
        "read_capability_state": (
            "Read guarded subsystem state without changing robot hardware."
        ),
        "detect_box": "Run one reviewed box-perception request.",
        "move_chassis": "Execute one site-reviewed chassis capability.",
        "move_waist": "Execute one site-reviewed waist capability.",
        "move_dual_arms": (
            "Execute one paired dual-arm capability; single-arm motion is absent."
        ),
        "set_dual_grippers": (
            "Command both grippers as one reviewed paired capability."
        ),
        "stop_all": (
            "Request bounded cancellation across reviewed motion subsystems."
        ),
        "finish": "Finish only from an explicitly safe capability mode.",
    }

    specs: list[dict[str, Any]] = []
    for name in ATOMIC_TOOL_NAMES:
        item_properties = properties.get(name, {})
        specs.append(
            {
                "name": name,
                "description": descriptions[name],
                "input_schema": {
                    "type": "object",
                    "properties": copy_mapping(item_properties),
                    "required": list(item_properties),
                    "additionalProperties": False,
                },
            }
        )
    return specs


def atomic_call_rejection(
    *,
    tool: str,
    arguments: Mapping[str, Any],
    profile: AtomicCapabilityProfile,
    capability_mode: str,
    robot_state: Mapping[str, Any] | None,
) -> str | None:
    """Return the first capability-boundary rejection reason, if any."""

    if tool not in ATOMIC_TOOL_NAMES:
        return "unknown_tool"
    if capability_mode not in ATOMIC_CAPABILITY_MODES:
        return "unknown_capability_mode"
    if capability_mode == "closed":
        return "closed"
    if not isinstance(arguments, Mapping):
        return "arguments_invalid"

    expected = _expected_arguments(profile, tool)
    if set(arguments) != set(expected):
        return "arguments_invalid"
    for name, allowed in expected.items():
        if arguments[name] not in allowed:
            return "unknown_profile"

    if tool in _READ_ONLY_TOOLS:
        return None
    if tool == "stop_all":
        return None

    state_reason = _robot_state_reason(
        robot_state,
        profile.robot_state_max_age_s,
        profile.mode,
    )
    if state_reason is not None:
        return state_reason
    if tool == "finish":
        status = arguments["status"]
        if status == "success":
            return None if capability_mode == "idle" else "unsafe_finish"
        return (
            None
            if capability_mode == "operator_review_acknowledged"
            else "unsafe_finish"
        )
    if capability_mode in {
        "operator_review",
        "operator_review_acknowledged",
    }:
        return "operator_review_open"
    if tool in _SAFE_IDLE_TOOLS:
        return None if capability_mode == "idle" else "capability_busy"
    return "unknown_tool"


def _expected_arguments(
    profile: AtomicCapabilityProfile,
    tool: str,
) -> dict[str, frozenset[Any]]:
    if tool not in _ARGUMENT_TOOLS:
        return {}
    if tool == "move_chassis":
        return {"profile_name": frozenset(profile.chassis_profiles.keys())}
    if tool == "move_waist":
        return {"profile_name": frozenset(profile.waist_profiles.keys())}
    if tool == "move_dual_arms":
        return {"profile_name": frozenset(profile.dual_arm_profiles.keys())}
    if tool == "set_dual_grippers":
        return {"command": frozenset(profile.gripper_profiles.keys())}
    return {"status": frozenset(("success", "failure"))}


def _robot_state_reason(
    state: Mapping[str, Any] | None,
    max_age_s: float,
    expected_mode: str,
) -> str | None:
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
    age = state["age_s"]
    if isinstance(age, bool) or not isinstance(age, (int, float)):
        return "robot_state_invalid"
    if not math.isfinite(float(age)):
        return "robot_state_invalid"
    if age < 0.0:
        return "robot_state_invalid"
    if age > max_age_s:
        return "robot_state_stale"
    if state["healthy"] is not True:
        return "robot_unhealthy"
    if state["mode"] != expected_mode:
        return "mode_mismatch"
    if state["left_arm_error"] is not False:
        return "arm_error"
    if state["right_arm_error"] is not False:
        return "arm_error"
    return None


def _enum_property(values: Any) -> dict[str, Any]:
    return {"type": "string", "enum": list(values)}


def copy_mapping(value: Mapping[str, Any]) -> dict[str, Any]:
    """Copy schema fragments without exposing the profile's mutable mappings."""

    return dict(value)
