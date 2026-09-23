from __future__ import annotations

import math
from typing import Any

import pytest

from robots.lynsense_real_box.atomic_contract import (
    ATOMIC_CAPABILITY_MODES,
    ATOMIC_TOOL_NAMES,
    atomic_call_rejection,
    atomic_tool_specs,
)
from robots.lynsense_real_box.atomic_profile import (
    validate_atomic_capability_profile,
)
from tests.unit_tests.robots.lynsense_real_box.test_atomic_profile import (
    valid_atomic_profile,
)


def profile():
    return validate_atomic_capability_profile(valid_atomic_profile())


def robot_state(**changes: Any) -> dict[str, Any]:
    state = {
        "age_s": 0.1,
        "healthy": True,
        "mode": "dry_run",
        "left_arm_error": False,
        "right_arm_error": False,
    }
    state.update(changes)
    return state


_DEFAULT_STATE = object()


def rejection(
    tool: str,
    arguments: dict[str, Any] | None = None,
    *,
    capability_mode: str = "idle",
    state: Any = _DEFAULT_STATE,
) -> str | None:
    return atomic_call_rejection(
        tool=tool,
        arguments=arguments or {},
        profile=profile(),
        capability_mode=capability_mode,
        robot_state=state if state is not _DEFAULT_STATE else robot_state(),
    )


def test_atomic_tool_table_has_no_task_flow_or_single_arm_tools():
    assert ATOMIC_TOOL_NAMES == (
        "read_capability_state",
        "detect_box",
        "move_chassis",
        "move_waist",
        "move_dual_arms",
        "set_dual_grippers",
        "stop_all",
        "finish",
    )
    assert "pick_box" not in ATOMIC_TOOL_NAMES
    assert "place_box" not in ATOMIC_TOOL_NAMES
    assert "move_left_arm" not in ATOMIC_TOOL_NAMES
    assert "move_right_arm" not in ATOMIC_TOOL_NAMES


def test_schemas_use_only_atomic_profile_enums_and_are_closed():
    specs = atomic_tool_specs(profile())
    assert [item["name"] for item in specs] == list(ATOMIC_TOOL_NAMES)
    by_name = {item["name"]: item for item in specs}

    assert by_name["move_chassis"]["input_schema"]["properties"][
        "profile_name"
    ]["enum"] == ["pickup", "retreat"]
    assert by_name["move_waist"]["input_schema"]["properties"]["profile_name"][
        "enum"
    ] == ["safe"]
    assert by_name["move_dual_arms"]["input_schema"]["properties"][
        "profile_name"
    ]["enum"] == ["safe"]
    assert by_name["set_dual_grippers"]["input_schema"]["properties"][
        "command"
    ]["enum"] == ["open", "close"]

    for spec in specs:
        schema = spec["input_schema"]
        assert schema["additionalProperties"] is False
        assert schema["required"] == list(schema["properties"])


def test_argument_whitelists_are_exact():
    assert rejection("move_waist", {"profile_name": "safe"}) is None
    assert rejection("move_waist", {"profile_name": "invented"}) == "unknown_profile"
    assert rejection("move_waist", {}) == "arguments_invalid"
    assert (
        rejection("move_waist", {"profile_name": "safe", "speed": 1})
        == "arguments_invalid"
    )
    assert rejection("set_dual_grippers", {"command": "half"}) == "unknown_profile"


def test_read_and_stop_remain_available_while_motion_is_interlocked():
    assert rejection("read_capability_state", capability_mode="dual_arms_moving") is None
    assert rejection("stop_all", capability_mode="dual_arms_moving") is None
    assert rejection("move_waist", {"profile_name": "safe"}, capability_mode="dual_arms_moving") == "capability_busy"
    assert rejection("detect_box", capability_mode="perception_running") == "capability_busy"


@pytest.mark.parametrize(
    ("state", "reason"),
    [
        (None, "robot_state_missing"),
        (robot_state(age_s="0.1"), "robot_state_invalid"),
        (robot_state(age_s=math.inf), "robot_state_invalid"),
        (robot_state(age_s=0.51), "robot_state_stale"),
        (robot_state(healthy=False), "robot_unhealthy"),
        (robot_state(mode="live"), "mode_mismatch"),
        (robot_state(left_arm_error=True), "arm_error"),
        (robot_state(right_arm_error=True), "arm_error"),
    ],
)
def test_unhealthy_or_invalid_state_blocks_every_capability_effect(state, reason):
    assert rejection("detect_box", state=state) == reason
    assert rejection("move_dual_arms", {"profile_name": "safe"}, state=state) == reason


def test_missing_state_is_rejected_without_helper_default():
    item = profile()
    assert (
        atomic_call_rejection(
            tool="detect_box",
            arguments={},
            profile=item,
            capability_mode="idle",
            robot_state=None,
        )
        == "robot_state_missing"
    )


def test_finish_and_operator_review_have_explicit_modes():
    assert rejection("finish", {"status": "success"}) is None
    assert (
        rejection("finish", {"status": "success"}, capability_mode="stopping")
        == "unsafe_finish"
    )
    assert (
        rejection(
            "finish", {"status": "failure"}, capability_mode="operator_review"
        )
        == "unsafe_finish"
    )
    assert (
        rejection(
            "finish",
            {"status": "failure"},
            capability_mode="operator_review_acknowledged",
        )
        is None
    )
    assert rejection("move_waist", {"profile_name": "safe"}, capability_mode="operator_review") == "operator_review_open"


def test_unknown_tool_and_mode_fail_closed():
    assert rejection("pick_box", {}) == "unknown_tool"
    assert rejection("detect_box", capability_mode="running") == "unknown_capability_mode"
    assert rejection("detect_box", capability_mode="closed") == "closed"
    assert "operator_review" in ATOMIC_CAPABILITY_MODES
    assert "operator_review_acknowledged" in ATOMIC_CAPABILITY_MODES
