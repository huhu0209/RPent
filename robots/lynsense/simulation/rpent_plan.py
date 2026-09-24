from __future__ import annotations

import copy
import json
import math
from pathlib import Path
from typing import Any


TASK_ID = "lynsense_single_box_v1"
PLAN_VERSION = 1

_CANONICAL_ACTIONS: tuple[dict[str, Any], ...] = (
    {"action": "nav_to_pose", "goal_name": "搬箱子1"},
    {"action": "move_waist", "height_mm": 200.0},
    {"action": "set_gripper", "position": 0.0},
    {"action": "move_named_config", "target": "dualjo:joints_br"},
    {
        "action": "box_phase",
        "box_action": "pick",
        "flow": "flow",
        "config": "box1",
    },
    {"action": "move_distance", "distance": -0.6, "angle": 0.0},
    {"action": "move_distance", "distance": 0.0, "angle": 90.0},
    {"action": "move_named_config", "target": "dualposi_armbase_abso:pt_1f1_ready"},
    {"action": "nav_to_pose", "goal_name": "放箱子1_1"},
    {
        "action": "box_phase",
        "box_action": "place",
        "flow": "flow",
        "config": "box1",
    },
    {"action": "move_distance", "distance": -0.6, "angle": 0.0},
    {"action": "move_named_config", "target": "dualposi_armbase_abso:pt_up"},
    {"action": "move_waist", "height_mm": 200.0},
    {"action": "set_gripper", "position": 0.0},
    {"action": "move_named_config", "target": "dualjo:joints_s"},
)

_ACTION_FIELDS: dict[str, tuple[str, ...]] = {
    "nav_to_pose": ("goal_name",),
    "move_waist": ("height_mm",),
    "set_gripper": ("position",),
    "move_named_config": ("target",),
    "box_phase": ("box_action", "flow", "config"),
    "move_distance": ("distance", "angle"),
}
_NUMERIC_FIELDS = {
    "move_waist": ("height_mm",),
    "set_gripper": ("position",),
    "move_distance": ("distance", "angle"),
}


class PlanError(ValueError):
    """A simulation plan does not satisfy the accepted task contract."""


def task_description() -> dict[str, Any]:
    return {
        "task_id": TASK_ID,
        "version": PLAN_VERSION,
        "environment": "webots-isolated",
        "execution": "serial",
        "required_action_count": len(_CANONICAL_ACTIONS),
        "initial_state": {
            "robot_pose": {
                "x": 2.024931,
                "y": -2.493846,
                "yaw_rad": 3.124139,
            },
            "box_pose": {
                "x": 1.674931,
                "y": -2.493846,
                "z": 0.1085,
                "yaw_rad": 0.0,
            },
        },
        "goal": {
            "box_pose": {
                "x": 2.810559,
                "y": -2.734170,
                "z": 0.1085,
                "yaw_rad": 0.006126,
            },
            "final_robot_pose": {
                "x": 1.8606561495236394,
                "y": -2.742422444761675,
                "yaw_rad": 0.006126105582217833,
            },
        },
        "required_action_order": tuple(
            action["action"] for action in _CANONICAL_ACTIONS
        ),
        "known_goal_names": ("搬箱子1", "放箱子1_1"),
        "known_action_values": {
            "move_waist": {"height_mm": (200.0,)},
            "set_gripper": {"position": (0.0,)},
            "move_named_config": {
                "target": tuple(
                    action["target"]
                    for action in _CANONICAL_ACTIONS
                    if action["action"] == "move_named_config"
                )
            },
            "move_distance": {"distance": (-0.6,), "angle": (0.0, 90.0)},
            "box_phase": {
                "action": ("pick", "place"),
                "flow": ("flow",),
                "config": ("box1",),
            },
        },
        "allowed_actions": {
            name: {"fields": fields}
            for name, fields in _ACTION_FIELDS.items()
        },
        "planning_rules": (
            "Submit exactly one accepted 15-action plan for the single-box task.",
            "Use only the listed action names and fields; extra fields are rejected.",
            "Pick must precede place and navigation remains strictly serial.",
        ),
    }


def canonical_single_box_plan() -> tuple[dict[str, Any], ...]:
    return copy.deepcopy(_CANONICAL_ACTIONS)


def validate_plan(source: Any) -> tuple[dict[str, Any], ...]:
    if not isinstance(source, dict):
        raise PlanError("plan must be a JSON object")
    expected_keys = {"task_id", "version", "actions"}
    if set(source) != expected_keys:
        raise PlanError(
            "plan fields must be exactly task_id, version, and actions"
        )
    if source["task_id"] != TASK_ID:
        raise PlanError(f"unsupported task_id: {source['task_id']!r}")
    version = source["version"]
    if isinstance(version, bool) or not isinstance(version, int) or version != PLAN_VERSION:
        raise PlanError(f"unsupported plan version: {version!r}")
    actions = source["actions"]
    if not isinstance(actions, list):
        raise PlanError("actions must be a JSON array")
    if len(actions) != len(_CANONICAL_ACTIONS):
        raise PlanError(
            f"plan must contain exactly {len(_CANONICAL_ACTIONS)} actions; "
            f"got {len(actions)}"
        )

    normalized: list[dict[str, Any]] = []
    for index, action in enumerate(actions):
        normalized.append(_validate_action(index, action))
    if tuple(normalized) != _CANONICAL_ACTIONS:
        raise PlanError("actions do not match the accepted single-box sequence")
    return tuple(normalized)


def load_plan(path: Path) -> tuple[dict[str, Any], ...]:
    try:
        source = json.loads(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise PlanError(f"cannot read plan {path}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise PlanError(f"plan is not valid JSON: {exc}") from exc
    return validate_plan(source)


def _validate_action(index: int, action: Any) -> dict[str, Any]:
    if not isinstance(action, dict):
        raise PlanError(f"action {index} must be an object")
    name = action.get("action")
    if not isinstance(name, str) or name not in _ACTION_FIELDS:
        raise PlanError(f"action {index} has an unsupported action name: {name!r}")
    fields = _ACTION_FIELDS[name]
    if set(action) != {"action", *fields}:
        raise PlanError(
            f"action {index} ({name}) fields must be exactly action and {list(fields)}"
        )
    normalized = {"action": name}
    for field in fields:
        value = action[field]
        if field in _NUMERIC_FIELDS.get(name, ()):  # type: ignore[arg-type]
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise PlanError(f"action {index} field {field} must be numeric")
            if not math.isfinite(float(value)):
                raise PlanError(f"action {index} field {field} must be finite")
            normalized[field] = float(value)
            continue
        if not isinstance(value, str) or not value:
            raise PlanError(f"action {index} field {field} must be a non-empty string")
        normalized[field] = value
    return normalized
