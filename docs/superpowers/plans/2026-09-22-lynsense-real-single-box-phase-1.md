# Lynsense Real Single-Box Phase 1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the offline, testable RPent decision and safety framework required before any Robot One motion adapter, then prepare the separately authorized read-only ROS inventory gate.

**Architecture:** Add a `lynsense_real_box` backend whose site profile, task state machine, evidence recorder, dry-run adapter, and Toolkit are pure Python and testable without ROS. The API Planner will make successive tool calls against a strict execution state machine; this phase constructs no ROS client and dispatches no hardware request. The real ROS adapter is deliberately deferred until the approved Robot One inventory supplies exact interface names and types.

**Tech Stack:** Python 3.10, RPent API Planner/Toolkit, pydantic-ai FunctionModel tests, JSON/JSONL artifacts, pytest.

**Spec:** `docs/superpowers/specs/2026-09-22-lynsense-real-single-box-design.md`

## Global Constraints

- This is Phase 1 only: implement the offline framework and inventory runbook, not the real ROS adapter.
- Do not connect to Robot One, use `ROS_DOMAIN_ID=3`, SSH to `robot1`, source a robot workspace, or call any live ROS graph during implementation.
- Do not import `rclpy`, ROS messages, `lynsense_pytrees`, or any company control package from Phase 1 production modules.
- Do not modify `robots/lynsense` or `robots/lynsense_simulation`.
- Do not add dependencies or make ROS a base import requirement.
- Keep the exact real-box tool table task-level; expose no image, file, shell, arbitrary trajectory, or left/right-only actuator tool.
- Dry-run is the only executable adapter mode in Phase 1; live mode may be represented in configuration but must be rejected by this runtime.
- Any Robot One inventory execution requires a new explicit user authorization after this plan is approved.
- No commit, push, PR, dependency installation, or physical-robot operation unless separately authorized.

---

### Task 1: Site Profile Contract

**Files:**

- Create: `robots/lynsense_real_box/__init__.py`
- Create: `robots/lynsense_real_box/site_profile.py`
- Test: `tests/unit_tests/robots/lynsense_real_box/__init__.py`
- Test: `tests/unit_tests/robots/lynsense_real_box/test_site_profile.py`

**Interfaces:**

- Produces `SiteProfileError(ValueError)`.
- Produces `SiteProfile`, `RosInterface`, `MoveTransition`, `NamedProfile`,
  `GraspReleaseEvidence`, `ControllerReview`, `ActionBudgets`,
  `GripperTiming`, `PerceptionContract`, `Calibration`, `PhysicalLimits`, and
  `PlacementAcceptance` immutable values; `PlacementAcceptance` contains the
  immutable nested `TargetPose` and `SupportedRegion` values.
- Produces `validate_site_profile(source: dict[str, Any]) -> SiteProfile`.
- Produces `load_site_profile(path: Path) -> SiteProfile`.
- Produces `site_profile_hash(profile: SiteProfile) -> str`; later evidence and adapters rely on this exact SHA-256 hexadecimal string.

- [x] **Step 1: Write failing contract tests**

Create tests that establish the complete profile shape and every rejection rule:

```python
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


def interface(role: str, kind: str = "topic", name: str | None = None,
              type_name: str = "example/Type") -> dict:
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
            interface("dual_arm_grasp_release", "action", type_name="example/BoxPhase"),
            interface("box_perception_trigger", "service", type_name="std_srvs/srv/Trigger"),
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
    profile = validate_site_profile(valid_profile())
    assert profile.profile_id == "robot1-single-box-v1"
    assert profile.ros_domain_id == 3
    assert profile.mode == "dry_run"
    assert len(profile.interfaces) == 17
    digest = site_profile_hash(profile)
    assert len(digest) == 64
    assert digest == site_profile_hash(validate_site_profile(valid_profile()))

    path = tmp_path / "site.json"
    path.write_text(json.dumps(valid_profile()), encoding="utf-8")
    assert load_site_profile(path) == profile


@pytest.mark.parametrize(
    "mutation",
    [
        lambda p: p.update(profile_id=""),
        lambda p: p.update(version=2),
        lambda p: p.update(ros_domain_id=42),
        lambda p: p.update(mode="live"),
        lambda p: p["interfaces"].append(p["interfaces"][0].copy()),
        lambda p: p["interfaces"].clear(),
        lambda p: p["interfaces"].remove(next(i for i in p["interfaces"] if i["role"] == "dual_arm_controller")),
        lambda p: p["interfaces"].remove(next(i for i in p["interfaces"] if i["role"] == "left_gripper_feedback")),
        lambda p: p["navigation_goals"].update(extra="home"),
        lambda p: p["move_transitions"][0].update(distance_m=math.nan),
        lambda p: p["move_transitions"].append(p["move_transitions"][0].copy()),
        lambda p: p["waist_profiles"].append({"name": "pick_ready", "height_mm": 300.0}),
        lambda p: p["dual_arm_configs"].append({"name": "safe"}),
        lambda p: p["gripper_commands"].append("half"),
        lambda p: p["grasp_evidence"].update(kind="invented_by_model"),
        lambda p: p["release_evidence"].update(kind="invented_by_model"),
        lambda p: p["controller_review"].update(status="unreviewed"),
        lambda p: p["action_budgets"].update(navigation_s=0.0),
        lambda p: p["gripper_timing"].update(feedback_timeout_s=0.0),
        lambda p: p["perception"].update(result_timeout_s=0.0),
        lambda p: p["calibration"].update(status="unknown"),
        lambda p: p["limits"].update(max_payload_kg=0.0),
        lambda p: p["placement"].update(position_tolerance_m=-0.1),
        lambda p: p["placement"]["target_pose"].update(x=math.nan),
        lambda p: p["placement"]["supported_region"].update(min_x=99.0),
        lambda p: p["failure"].update(partially_held_box="model_decides"),
        lambda p: p["freshness"].update(robot_state_max_age_s=0.0),
        lambda p: p.update(extra_field="forbidden"),
    ],
)
def test_invalid_profiles_are_rejected(mutation):
    source = valid_profile()
    mutation(source)
    with pytest.raises(SiteProfileError):
        validate_site_profile(source)


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
    assert any(item.role == "dual_arm_grasp_release" for item in profile.interfaces)
```

- [x] **Step 2: Run the site-profile tests and verify RED**

Run:

```bash
.venv/bin/pytest -q tests/unit_tests/robots/lynsense_real_box/test_site_profile.py
```

Expected result before implementation: collection fails with `ModuleNotFoundError` or `ImportError` for `robots.lynsense_real_box.site_profile`.

- [x] **Step 3: Implement the pure-Python profile contract**

Implement immutable dataclasses with exact field sets. Normalize at load time, never on every tool call. Hash the normalized public document with `json.dumps(document, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False)` before `hashlib.sha256`.

Enforce these rules:

- exact top-level keys are `profile_id`, `version`, `ros_domain_id`, `mode`, `interfaces`, `navigation_goals`, `move_transitions`, `waist_profiles`, `dual_arm_configs`, `gripper_commands`, `grasp_evidence`, `release_evidence`, `controller_review`, `action_budgets`, `gripper_timing`, `perception`, `calibration`, `limits`, `placement`, `failure`, and `freshness`;
- `version == 1`, `ros_domain_id == 3`, and `mode in {"dry_run", "live"}`;
- all 17 Phase 1 roles are required exactly once, including localization, both gripper feedback streams, and the composite dual-arm grasp/release interface;
- interface kinds are exactly `topic`, `service`, or `action`; topic direction is `subscribe`, service/action direction is `call`;
- navigation goal keys are exactly `pickup` and `placement`;
- move-transition and named-profile names are unique and nonempty;
- all numeric values are finite and physical thresholds are strictly positive;
- `gripper_commands == ["open", "close"]`;
- accepted grasp evidence kinds are `contact`, `force_current`, and `reviewed_controller_signal`;
- accepted release evidence kinds are the same three kinds;
- dry-run controller review may be `offline_fixture_only`; live requires `site_confirmed`, a nonempty reviewer, and nonempty safety evidence;
- dry-run calibration may be `offline_fixture_confirmed`; live requires `site_confirmed`;
- every action budget, timeout, tolerance, speed limit, payload limit, freshness threshold, settle interval, and stability threshold must be finite and strictly positive;
- placement target coordinates and yaw must be finite; `min_x < max_x` and
  `min_y < max_y` in the supported region;
- the partial-hold policy is `hold_grippers` or `site_reviewed_open`;
- source extras and duplicate names/roles are rejected.

- [x] **Step 4: Run the focused test and contract edge cases**

Run the command from Step 2. Expected result: every test passes.

- [x] **Step 5: Review the diff; do not commit**

Inspect the new module and tests for ROS imports, network calls, file writes outside `load_site_profile`, or unhashed mutable state. Do not run `git commit`.

---

### Task 2: Execution State Machine

**Files:**

- Create: `robots/lynsense_real_box/task_state.py`
- Test: `tests/unit_tests/robots/lynsense_real_box/test_task_state.py`

**Interfaces:**

- Produces `TaskState(str, Enum)` with:
  `initialized`, `box_localized`, `at_pick_approach`,
  `dual_pick_prepared`, `box_grasped`, `carrying`, `at_place_approach`,
  `dual_place_prepared`, `box_released`, `withdrawn`, `safe_complete`,
  `failed_stopping`, `failed_stopped`, `operator_review`, and
  `operator_acknowledged_stopped`.
- Produces `ToolName(str, Enum)` with the 12 model-visible tool names from the spec.
- Produces `TransitionError(ValueError)`.
- Produces `TaskStateMachine(initial_state: TaskState = TaskState.INITIALIZED, move_transitions: Mapping[str, MoveTransition] = MappingProxyType({}))` with:
  `state`, `allowed_tools() -> frozenset[ToolName]`,
  `pick_preparations: frozenset[str]`, `safe_preparations: frozenset[str]`,
  `failure_kind: str | None`, `requires_operator_review: bool`,
  `apply(tool: ToolName, *, profile_name: str | None = None, success: bool = True, failure_kind: str | None = None, box_hazard: bool = False) -> TaskState`,
  `mark_external_estop() -> TaskState`, `mark_operator_review(reason: str) -> TaskState`,
  `mark_operator_stopped() -> TaskState`, and non-mutating
  `assert_may_finish(status: str) -> None`.

- [x] **Step 1: Write failing state-machine tests**

```python
from __future__ import annotations

import pytest

from collections.abc import Mapping

from robots.lynsense_real_box.site_profile import MoveTransition
from robots.lynsense_real_box.task_state import (
    TaskState,
    TaskStateMachine,
    ToolName,
    TransitionError,
)


def move(name: str, from_state: str, to_state: str,
         distance_m: float, angle_deg: float) -> MoveTransition:
    return MoveTransition(
        name=name,
        from_state=from_state,
        to_state=to_state,
        distance_m=distance_m,
        angle_deg=angle_deg,
    )


MOVE_TRANSITIONS: Mapping[str, MoveTransition] = {
    item.name: item
    for item in (
        move("retreat_after_pick", "box_grasped", "carrying", -0.6, 0.0),
        move("turn_after_pick", "carrying", "carrying", 0.0, 90.0),
        move("retreat_after_place", "box_released", "withdrawn", -0.6, 0.0),
    )
}


def test_normal_success_path_is_serial():
    machine = TaskStateMachine(move_transitions=MOVE_TRANSITIONS)
    steps = [
        (ToolName.DETECT_BOX, None),
        (ToolName.NAV_TO_POSE, "pickup"),
        (ToolName.MOVE_WAIST, "pick_ready"),
        (ToolName.SET_DUAL_GRIPPERS, "close"),
        (ToolName.MOVE_DUAL_ARMS, "pick_ready"),
        (ToolName.PICK_BOX, None),
        (ToolName.MOVE_DISTANCE, "retreat_after_pick"),
        (ToolName.MOVE_DISTANCE, "turn_after_pick"),
        (ToolName.NAV_TO_POSE, "placement"),
        (ToolName.MOVE_DUAL_ARMS, "place_ready"),
        (ToolName.PLACE_BOX, None),
        (ToolName.MOVE_DISTANCE, "retreat_after_place"),
        (ToolName.MOVE_DUAL_ARMS, "safe"),
        (ToolName.SET_DUAL_GRIPPERS, "open"),
        (ToolName.MOVE_WAIST, "safe"),
    ]
    expected = [
        TaskState.BOX_LOCALIZED,
        TaskState.AT_PICK_APPROACH,
        TaskState.AT_PICK_APPROACH,
        TaskState.AT_PICK_APPROACH,
        TaskState.DUAL_PICK_PREPARED,
        TaskState.BOX_GRASPED,
        TaskState.CARRYING,
        TaskState.CARRYING,
        TaskState.AT_PLACE_APPROACH,
        TaskState.DUAL_PLACE_PREPARED,
        TaskState.BOX_RELEASED,
        TaskState.WITHDRAWN,
        TaskState.WITHDRAWN,
        TaskState.WITHDRAWN,
        TaskState.SAFE_COMPLETE,
    ]
    assert len(steps) == len(expected)
    for index, (tool, profile_name) in enumerate(steps):
        machine.apply(tool, profile_name=profile_name)
        assert machine.state is expected[index]
    assert machine.state is TaskState.SAFE_COMPLETE


def test_read_tools_do_not_advance_state():
    machine = TaskStateMachine(TaskState.AT_PICK_APPROACH)
    for tool in (ToolName.READ_TASK_STATE, ToolName.READ_ROBOT_STATE,
                 ToolName.DETECT_BOX):
        machine.apply(tool)
    assert machine.state is TaskState.AT_PICK_APPROACH


def test_motion_failure_enters_stopping_then_stopped():
    machine = TaskStateMachine(TaskState.DUAL_PICK_PREPARED)
    machine.apply(ToolName.PICK_BOX, success=False)
    assert machine.state is TaskState.FAILED_STOPPING
    machine.apply(ToolName.STOP_TASK, success=True)
    assert machine.state is TaskState.FAILED_STOPPED


def test_held_box_hazard_requires_operator_review():
    machine = TaskStateMachine(TaskState.DUAL_PICK_PREPARED)
    machine.apply(ToolName.PICK_BOX, success=False, box_hazard=True)
    assert machine.state is TaskState.FAILED_STOPPING
    machine.apply(ToolName.STOP_TASK, success=True, box_hazard=True)
    assert machine.state is TaskState.OPERATOR_REVIEW


@pytest.mark.parametrize(
    ("initial_state", "tool", "failure_kind"),
    [
        (TaskState.DUAL_PICK_PREPARED, ToolName.PICK_BOX, "cancel_unknown"),
        (TaskState.DUAL_PICK_PREPARED, ToolName.PICK_BOX, "dual_arm_disagreement"),
        (TaskState.DUAL_PICK_PREPARED, ToolName.PICK_BOX, "gripper_disagreement"),
        (TaskState.DUAL_PICK_PREPARED, ToolName.PICK_BOX, "missing_grasp_evidence"),
        (TaskState.DUAL_PLACE_PREPARED, ToolName.PLACE_BOX, "release_verification_failed"),
        (TaskState.CARRYING, ToolName.NAV_TO_POSE, "localization_loss"),
    ],
)
def test_structured_physical_failures_require_operator_review(
    initial_state, tool, failure_kind
):
    machine = TaskStateMachine(
        initial_state,
        move_transitions=MOVE_TRANSITIONS,
    )
    machine.apply(
        tool,
        success=False,
        failure_kind=failure_kind,
    )
    assert machine.failure_kind == failure_kind
    assert machine.requires_operator_review is True
    machine.apply(ToolName.STOP_TASK, success=True)
    assert machine.state is TaskState.OPERATOR_REVIEW


@pytest.mark.parametrize("failure_kind", ["action_failed", "timeout", "canceled"])
def test_confirmed_stop_is_sufficient_for_ordinary_failures(failure_kind):
    machine = TaskStateMachine(
        TaskState.DUAL_PICK_PREPARED,
        move_transitions=MOVE_TRANSITIONS,
    )
    machine.apply(
        ToolName.PICK_BOX,
        success=False,
        failure_kind=failure_kind,
    )
    assert machine.requires_operator_review is False
    machine.apply(ToolName.STOP_TASK, success=True)
    assert machine.state is TaskState.FAILED_STOPPED


def test_stop_from_ordinary_state_is_idempotent_after_confirmation():
    machine = TaskStateMachine(TaskState.AT_PICK_APPROACH)
    machine.apply(ToolName.STOP_TASK, success=True)
    assert machine.state is TaskState.FAILED_STOPPED
    machine.apply(ToolName.STOP_TASK, success=True)
    assert machine.state is TaskState.FAILED_STOPPED
    machine.mark_operator_stopped()
    assert machine.state is TaskState.OPERATOR_ACKNOWLEDGED_STOPPED


def test_stop_failure_requires_operator_review():
    machine = TaskStateMachine(
        TaskState.CARRYING,
        move_transitions=MOVE_TRANSITIONS,
    )
    machine.apply(
        ToolName.MOVE_DISTANCE,
        profile_name="turn_after_pick",
        success=False,
    )
    machine.apply(ToolName.STOP_TASK, success=False)
    assert machine.state is TaskState.OPERATOR_REVIEW


def test_external_estop_and_asynchronous_fault_enter_operator_review():
    estop_machine = TaskStateMachine(TaskState.CARRYING)
    assert estop_machine.mark_external_estop() is TaskState.OPERATOR_REVIEW

    fault_machine = TaskStateMachine(TaskState.AT_PICK_APPROACH)
    assert fault_machine.mark_operator_review(
        "asynchronous_driver_fault"
    ) is TaskState.OPERATOR_REVIEW


def test_failure_states_allow_reads_and_stop_but_no_motion():
    machine = TaskStateMachine(TaskState.FAILED_STOPPED)
    allowed = machine.allowed_tools()
    assert ToolName.READ_ROBOT_STATE in allowed
    assert ToolName.STOP_TASK in allowed
    assert ToolName.MOVE_DUAL_ARMS not in allowed
    assert ToolName.PICK_BOX not in allowed


def test_pick_requires_all_three_pick_preparations():
    machine = TaskStateMachine(TaskState.AT_PICK_APPROACH)
    machine.apply(ToolName.MOVE_WAIST, profile_name="pick_ready")
    with pytest.raises(TransitionError):
        machine.apply(ToolName.PICK_BOX)
    machine.apply(ToolName.SET_DUAL_GRIPPERS, profile_name="close")
    with pytest.raises(TransitionError):
        machine.apply(ToolName.PICK_BOX)
    machine.apply(ToolName.MOVE_DUAL_ARMS, profile_name="pick_ready")
    assert machine.state is TaskState.DUAL_PICK_PREPARED
    assert machine.apply(ToolName.PICK_BOX) is TaskState.BOX_GRASPED


def test_move_transition_profile_must_match_current_state():
    machine = TaskStateMachine(
        TaskState.BOX_GRASPED,
        move_transitions=MOVE_TRANSITIONS,
    )
    with pytest.raises(TransitionError):
        machine.apply(
            ToolName.MOVE_DISTANCE,
            profile_name="turn_after_pick",
        )


@pytest.mark.parametrize(
    ("state", "tool"),
    [
        (TaskState.INITIALIZED, ToolName.PICK_BOX),
        (TaskState.BOX_GRASPED, ToolName.PLACE_BOX),
        (TaskState.SAFE_COMPLETE, ToolName.MOVE_DUAL_ARMS),
        (TaskState.OPERATOR_ACKNOWLEDGED_STOPPED, ToolName.MOVE_DUAL_ARMS),
    ],
)
def test_illegal_motion_is_rejected(state, tool):
    machine = TaskStateMachine(state)
    with pytest.raises(TransitionError):
        machine.apply(tool, profile_name="pick_ready")


def test_operator_acknowledgement_has_source_restrictions():
    machine = TaskStateMachine(TaskState.SAFE_COMPLETE)
    with pytest.raises(TransitionError):
        machine.mark_operator_stopped()


def test_finish_legality_is_explicit():
    machine = TaskStateMachine()
    with pytest.raises(TransitionError):
        machine.assert_may_finish("success")
    machine.apply(ToolName.DETECT_BOX)
    with pytest.raises(TransitionError):
        machine.assert_may_finish("failure")
    safe_machine = TaskStateMachine(TaskState.SAFE_COMPLETE)
    safe_machine.assert_may_finish("success")
    stopped_machine = TaskStateMachine(TaskState.OPERATOR_ACKNOWLEDGED_STOPPED)
    stopped_machine.assert_may_finish("failure")
```

- [x] **Step 2: Run the state-machine tests and verify RED**

Run:

```bash
.venv/bin/pytest -q tests/unit_tests/robots/lynsense_real_box/test_task_state.py
```

Expected result before implementation: import failure for `task_state`.

- [x] **Step 3: Implement explicit transition tables**

Use two mappings, not conditional string parsing:

```python
_SUCCESS_TRANSITIONS: dict[tuple[TaskState, ToolName], TaskState] = {
    (TaskState.INITIALIZED, ToolName.DETECT_BOX): TaskState.BOX_LOCALIZED,
    (TaskState.BOX_LOCALIZED, ToolName.NAV_TO_POSE): TaskState.AT_PICK_APPROACH,
    (TaskState.CARRYING, ToolName.NAV_TO_POSE): TaskState.AT_PLACE_APPROACH,
    (TaskState.AT_PLACE_APPROACH, ToolName.MOVE_DUAL_ARMS): TaskState.DUAL_PLACE_PREPARED,
    (TaskState.DUAL_PICK_PREPARED, ToolName.PICK_BOX): TaskState.BOX_GRASPED,
    (TaskState.DUAL_PLACE_PREPARED, ToolName.PLACE_BOX): TaskState.BOX_RELEASED,
}
```

Preparation tools do not invent new task states. In `AT_PICK_APPROACH`,
successful `move_waist`, `set_dual_grippers`, and `move_dual_arms` add
`waist`, `grippers`, and `arms` to `pick_preparations`; completion of the third
preparation changes the state to `DUAL_PICK_PREPARED`. In `WITHDRAWN`, the same
three tool families populate `safe_preparations`; completion of the third
changes the state to `SAFE_COMPLETE`. `pick_box` is legal only when
`pick_preparations == {"waist", "grippers", "arms"}`.

`move_distance` is resolved only through the constructor-supplied
`move_transitions[profile_name]`; its `from_state` must equal the current state
and its reviewed `to_state` is authoritative. `nav_to_pose` accepts only pickup from
`BOX_LOCALIZED` and placement from `CARRYING`. `move_dual_arms(place_ready)` is
legal only from `AT_PLACE_APPROACH`. Read tools and legal observation-only
`detect_box` calls keep the current state. `assert_may_finish()` is
non-mutating and accepts only `success` in `SAFE_COMPLETE` and only `failure`
in `OPERATOR_ACKNOWLEDGED_STOPPED`.

Failure semantics are tool-dependent: failures of `read_task_state`,
`read_robot_state`, and `detect_box` retain the current state; failures of
motion-capable tools enter `FAILED_STOPPING`. `mark_external_estop()` and
`mark_operator_review()` are runtime/operator methods, never model tools.
Successful `stop_task` from `FAILED_STOPPING` enters `FAILED_STOPPED` only when
`box_hazard=False` and `requires_operator_review=False`; either flag being true
enters `OPERATOR_REVIEW`. `requires_operator_review` is true for
`cancel_unknown`, `dual_arm_disagreement`, `gripper_disagreement`,
`missing_grasp_evidence`, `release_verification_failed`, `localization_loss`,
or `box_hazard=True`; it is false for a confirmed `action_failed`, `timeout`,
or `canceled` outcome. From an ordinary nonterminal state, successful
`stop_task` enters `FAILED_STOPPED`; repeated successful stops are idempotent.
`mark_operator_stopped()` is legal only from `FAILED_STOPPED` or
`OPERATOR_REVIEW`.
`mark_external_estop()` and `mark_operator_review(reason)` are legal only from
nonterminal states; both are rejected after `SAFE_COMPLETE` or
`OPERATOR_ACKNOWLEDGED_STOPPED`. Adapter closure is enforced by the Toolkit and
adapter lifecycle rather than by the state machine. Terminal states expose no
method that restores motion.

- [x] **Step 4: Run focused tests**

Run Step 2 command. Expected result: all tests pass.

- [x] **Step 5: Review transition coverage**

Confirm every motion tool has at least one legal success transition, every nonterminal state can enter `FAILED_STOPPING`, and no model-callable method sets `OPERATOR_ACKNOWLEDGED_STOPPED`.

---

### Task 3: Offline Dry-Run Adapter

**Files:**

- Create: `robots/lynsense_real_box/dry_run_adapter.py`
- Test: `tests/unit_tests/robots/lynsense_real_box/test_dry_run_adapter.py`

**Interfaces:**

- Consumes `SiteProfile`.
- Produces `OfflineDryRunAdapter(profile: SiteProfile, robot_state: dict | None = None, box_pose: dict | None = None, placement_result: dict | None = None, now_s: float = 1000.0)`.
- Produces all spec adapter methods except a ROS import:
  `connect`, `read_state`, `detect_box`, `navigate`, `move_distance`, `move_waist`, `move_dual_arms`, `set_dual_grippers`, `pick_box`, `place_box`, `stop_motion`, and `close`.
- Produces `calls: list[dict[str, Any]]` for exact dry-run assertions.
- Produces `fail_next(method_name: str, result: dict[str, Any]) -> None`.

- [x] **Step 1: Write failing dry-run tests**

Cover:

- `connect()` returns `{"status": "ok", "mode": "dry_run", "profile_id": "robot1-single-box-v1"}` and records no call;
- `read_state()` returns the supplied state without copying mutable internals by reference;
- `detect_box()` returns supplied pose plus `status`, `frame`, and `received_at`;
- robot-state inputs include finite `age_s`, `healthy`, `mode`, and left/right
  error fields; motion methods reject missing state, stale state, unhealthy
  state, or either arm error before recording an intended request;
- perception inputs include finite `age_s`, valid `frame`, and
  `transform_valid`; pick/place reject missing or stale perception, wrong
  frame, and invalid transforms before recording an intended request;
- every motion method records exactly `method`, `interface_role`, `interface_name`, and normalized request;
- unknown profile names return `{"status": "rejected", "reason": "unknown_profile"}` and record no intended request;
- `fail_next()` supplies one failure result then clears itself;
- scripted timeout, canceled, and `cancel_unknown` outcomes retain the three
  literal `failure_kind` values and never report success;
- `move_dual_arms()` can return `dual_arm_disagreement` with separate
  `left_result` and `right_result`, and one-sided completion is never success;
- `set_dual_grippers()` can return `gripper_disagreement` with separate
  feedback states;
- `pick_box()` can return `missing_grasp_evidence` and sets `box_hazard=True`
  when either gripper reports `held`;
- `place_box()` can return `release_verification_failed`,
  `placement_settle_failed`, or `placement_stability_failed`;
- successful `place_box()` requires a complete `placement_result` containing
  target pose, both gripper open feedback states, release evidence,
  `settle_elapsed_s`, and `max_observed_speed_m_s`; it rejects a pose outside
  the supported region, position/yaw tolerance failure, either gripper not
  open, release evidence mismatch, insufficient settle time, and excessive
  stability speed before reporting `dry_run`;
- `navigate()` can return `localization_loss` while carrying;
- `stop_motion()` returns a configured best-effort result and never claims E-stop;
- `close()` is idempotent and makes later calls return `closed`.
- `pick_box()` records one composite request through `dual_arm_grasp_release`:
  `{"box_action": "pick", "left_gripper": "held", "right_gripper": "held", "grasp_evidence": profile.grasp_evidence}`;
- `place_box()` records one composite request through `dual_arm_grasp_release`:
  `{"box_action": "place", "left_gripper": "released", "right_gripper": "released", "release_evidence": profile.release_evidence, "placement": profile.placement}`;

Use this assertion shape for a motion call:

```python
robot_state = {
    "age_s": 0.1,
    "healthy": True,
    "left_arm_error": False,
    "right_arm_error": False,
}
box_pose = {
    "age_s": 0.2,
    "frame": "offline_box_fixture",
    "transform_valid": True,
    "pose": {"x": 1.0, "y": -2.0, "z": 0.1, "yaw_rad": 0.0},
}
placement_result = {
    "pose": {"x": 2.81, "y": -2.73, "z": 0.108, "yaw_rad": 0.006},
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
adapter = OfflineDryRunAdapter(
    profile,
    robot_state=robot_state,
    box_pose=box_pose,
    placement_result=placement_result,
)
result = adapter.move_distance("retreat_after_pick")
assert result["status"] == "dry_run"
assert adapter.calls == [
    {
        "method": "move_distance",
        "interface_role": "chassis_move",
        "interface_name": next(
            item.name for item in profile.interfaces if item.role == "chassis_move"
        ),
        "request": {"distance_m": -0.6, "angle_deg": 0.0},
    }
]
```

- [x] **Step 2: Run and verify RED**

```bash
.venv/bin/pytest -q tests/unit_tests/robots/lynsense_real_box/test_dry_run_adapter.py
```

Expected: import failure for `dry_run_adapter`.

- [x] **Step 3: Implement the adapter**

Resolve profiles through prebuilt dictionaries from `SiteProfile`, not string search at call time. Never construct a client. Return `dry_run` only after argument, state, perception, health, calibration, frame, and transform checks pass. On a rejected argument or failed precondition, append no successful intended request; append a rejection event with `dispatched=False` so offline evidence still explains the guard result.

- [x] **Step 4: Run focused tests**

Expected: all dry-run adapter tests pass.

---

### Task 4: Evidence Recorder

**Files:**

- Create: `robots/lynsense_real_box/evidence.py`
- Test: `tests/unit_tests/robots/lynsense_real_box/test_evidence.py`

**Interfaces:**

- Produces `EvidenceRecorder(path: Path, profile: SiteProfile)`.
- Produces `record_tool_call(tool: str, arguments: dict, result: dict, state_before: str, state_after: str) -> str`.
- Produces `record_operator_acknowledgement(operator: str) -> None`.
- Produces `close() -> None`.

- [x] **Step 1: Write failing evidence tests**

Require:

- the first JSONL line records `schema_version`, `event`, `profile_id`, `profile_sha256`, `created_at`, and operator or tool fields;
- `record_tool_call()` returns a stable evidence identifier such as
  `evt-000001` and every tool result carries that value as `evidence_id`;
- tool events record tool arguments and result but recursively reject keys
  whose normalized lowercase name is `password`, `token`, `api_key`, `secret`,
  `private_key`, or `ssh_config`; ambiguous credential-like nested keys fail
  closed;
- each line is valid UTF-8 JSON with sorted keys and no NaN;
- failed writes leave prior complete lines intact;
- `close()` is idempotent.

- [x] **Step 2: Run and verify RED**

```bash
.venv/bin/pytest -q tests/unit_tests/robots/lynsense_real_box/test_evidence.py
```

- [x] **Step 3: Implement append-only JSONL recording**

Create the parent directory, open in append mode with UTF-8, write one complete line per event, flush, and `os.fsync`. Do not rewrite or truncate existing evidence. Use UTC ISO-8601 timestamps.

- [x] **Step 4: Run focused tests**

Expected: all evidence tests pass.

---

### Task 5: Real-Box Toolkit

**Files:**

- Create: `robots/lynsense_real_box/toolkit.py`
- Test: `tests/unit_tests/robots/lynsense_real_box/test_toolkit.py`

**Interfaces:**

- Consumes `SiteProfile`, `TaskStateMachine`, `OfflineDryRunAdapter`, and `EvidenceRecorder`.
- Produces `LynsenseRealBoxToolkit(*, adapter: OfflineDryRunAdapter, state_machine: TaskStateMachine, evidence: EvidenceRecorder, dashboard_events: DashboardEventSink, memory: MemoryManager, profile: SiteProfile)`.
- Exposes exactly the 12 spec tools in this order:
  `read_task_state`, `read_robot_state`, `detect_box`, `nav_to_pose`, `move_distance`, `move_waist`, `move_dual_arms`, `set_dual_grippers`, `pick_box`, `place_box`, `stop_task`, and `finish`.

- [x] **Step 1: Write failing Toolkit/API-boundary tests**

Cover:

- `include_image_reader is False` and `_register_common_tools()` registers nothing;
- actual `_build_tools(toolkit)` returns exactly the 12 names and no `read_image`;
- every JSON schema has `additionalProperties=False`;
- `finish` status enum is exactly `["success", "failure"]`;
- valid state transitions invoke adapter and evidence;
- every successful, rejected, failed, or exceptional call produces one
  evidence event and one returned `evidence_id`;
- every returned tool result contains `status`, `reason` (`null` on success),
  `task_state`, and `evidence_id` in addition to tool-specific fields;
- illegal tool, unknown profile, wrong `from_state`, extra argument, and unsafe
  finish return structured errors, record the rejection, do not advance the
  state, and do not append an intended request;
- adapter motion failure applies `success=False` and any adapter-reported
  `box_hazard` to the state machine;
- `finish(success)` is rejected before `safe_complete`;
- `finish(failure)` is rejected before `operator_acknowledged_stopped`;
- `close()` closes adapter and evidence exactly once.
- an overridden `execute_tool(name, input_dict)` records unknown tool names,
  schema-invalid extra fields, and concurrent-operation rejections before
  returning; base-class dispatch results lacking `evidence_id` are also passed
  through the rejection recorder;
- `read_task_state()` includes `profile_id` and `profile_sha256`, making the
  profile hash part of the model transcript.

Include an exact tool-table assertion:

```python
EXPECTED_TOOLS = [
    "read_task_state",
    "read_robot_state",
    "detect_box",
    "nav_to_pose",
    "move_distance",
    "move_waist",
    "move_dual_arms",
    "set_dual_grippers",
    "pick_box",
    "place_box",
    "stop_task",
    "finish",
]
assert [item["name"] for item in toolkit.get_tools_spec()] == EXPECTED_TOOLS
assert [tool.name for tool in _build_tools(toolkit)] == EXPECTED_TOOLS
```

- [x] **Step 2: Run and verify RED**

```bash
.venv/bin/pytest -q tests/unit_tests/robots/lynsense_real_box/test_toolkit.py
```

- [x] **Step 3: Implement guarded tool handlers**

Every handler follows the same order:

1. strict argument/profile normalization, including `from_state` for every
   move transition and goal kind for navigation;
2. state-machine legality;
3. adapter call;
4. state transition with `success=result.get("status") in {"ok", "dry_run"}`
   plus `failure_kind=result.get("failure_kind")` and
   `box_hazard=result.get("box_hazard", False)`;
5. evidence recording for accepted, rejected, failed, and exceptional paths;
6. structured return with `status`, `reason`, `task_state`, and `evidence_id`.

Implement the dispatch wrapper explicitly. Before calling
`super().execute_tool()`, validate the top-level argument keys against the
registered JSON schema and record unknown-tool or extra-field rejections. After
the base call, if the returned result contains `error` but no `evidence_id`,
record that result with unchanged state and attach the evidence identifier.
Never acquire the robot adapter while holding the evidence recorder's file lock.

Do not let an adapter exception cross the model boundary. For read/observation tools, return a failure result whose `task_state` is `machine.state.value` without advancing. For motion-capable tools, return `{"status": "failed", "reason": "adapter_exception", "task_state": "failed_stopping"}` and apply the failure transition. `stop_task` handles its own stopping state and never reports E-stop.

- [x] **Step 4: Run focused tests**

Expected: all Toolkit tests pass.

---

### Task 6: Robot Registration And Runtime Gate

**Files:**

- Create: `robots/lynsense_real_box/robot_spec.py`
- Modify: `robots/lynsense_real_box/__init__.py`
- Modify: `tests/unit_tests/rpent/robots/test_registry_contracts.py`
- Test: `tests/unit_tests/robots/lynsense_real_box/test_extension.py`

**Interfaces:**

- Produces `get_robot_spec() -> RobotSpec` named `lynsense_real_box`.
- Produces `get_toolkit(*, runtime_kwargs: dict[str, Any], dashboard_events: DashboardEventSink, config: RunConfig) -> LynsenseRealBoxToolkit`, using the validated site profile and Phase 1 dry-run adapter.
- Adds `lynsense_real_box` to expected robots and prompt variables.

- [x] **Step 1: Write failing extension tests**

Cover:

- importing `robot_spec` does not import ROS modules;
- name, `is_real_robot=True`, and `supports_exploration=False`;
- only API Planner, local memory, and ordinary TTY are accepted;
- Dashboard, interactive, exploration, and alternate memory profiles are rejected;
- `--site-profile` is required and must point to a valid JSON profile;
- Phase 1 rejects `mode="live"` during `init_runtime` even if profile validation permits representing it;
- runtime components must be empty;
- construction/connection failure closes adapter and evidence resources;
- normal close closes both exactly once.

- [x] **Step 2: Update registry RED**

Add `lynsense_real_box` to `EXPECTED_ROBOTS` in sorted position and `PROMPT_VARIABLES` with `{}`. Run:

```bash
.venv/bin/pytest -q tests/unit_tests/rpent/robots/test_registry_contracts.py
```

Expected before implementation: registry discovery does not yet include the backend.

- [x] **Step 3: Implement `RobotSpec`**

Use the simulation backend's lazy import pattern and the real backend's strict mode gates. Prompt text must state that tools are globally visible but only currently legal calls execute, dry-run sends no ROS request, and `stop_task` is not an E-stop.

- [x] **Step 4: Run focused extension and registry tests**

```bash
.venv/bin/pytest -q \
  tests/unit_tests/robots/lynsense_real_box/test_extension.py \
  tests/unit_tests/rpent/robots/test_registry_contracts.py
```

Expected: both suites pass.

---

### Task 7: Stepwise API Planner And CLI Proof

**Files:**

- Test: `tests/unit_tests/robots/lynsense_real_box/test_api_planner.py`
- Test: `tests/unit_tests/robots/lynsense_real_box/test_cli.py`

**Interfaces:**

- Consumes the Task 6 backend and Task 5 Toolkit.
- Produces transcript evidence that RPent makes multiple tool calls rather than one static plan submission.

- [x] **Step 1: Write failing FunctionModel tests**

Use the actual `ApiAgentLoop` and `FunctionModel`. Provide a model sequence that:

1. reads task state;
2. reads robot state;
3. detects the box;
4. navigates to pickup;
5. prepares waist, grippers, and dual arms;
6. picks;
7. executes both carry transitions;
8. navigates while carrying and then prepares placement;
9. places and withdraws;
10. restores safe posture;
11. finishes with success.

Assert:

- more than ten model turns occur;
- every tool call is represented in evidence;
- dry-run adapter has intended requests for chassis, waist, dual arms, and dual grippers;
- no intended request is a left-only or right-only actuator command;
- final state is `safe_complete`;
- transcript path exists and contains profile hash.

Use `max_turns=32` for the direct `ApiAgentLoop` test. Then add a CLI test
using the actual CLI lifecycle, `--max-turns 32`, and a `FunctionModel`
replacement for `infer_model`. Assert exit success, generated evidence, and no
ROS/network import.

- [x] **Step 2: Run and verify RED**

```bash
.venv/bin/pytest -q \
  tests/unit_tests/robots/lynsense_real_box/test_api_planner.py \
  tests/unit_tests/robots/lynsense_real_box/test_cli.py
```

- [x] **Step 3: Fix only integration defects**

Do not weaken Toolkit or state-machine contracts to make the loop pass. If the model sequence exposes a state bug, add the failing case to Task 2 or Task 5 and fix it there first.

- [x] **Step 4: Run both tests again**

Expected: both tests pass through the real planner/CLI boundary.

---

### Task 8: Robot One Read-Only Inventory Runbook

**Files:**

- Create: `docs/superpowers/plans/2026-09-22-robot-one-readonly-inventory.md`
- Test: `tests/unit_tests/robots/lynsense_real_box/test_inventory_runbook.py`

**Interfaces:**

- Produces an operational runbook and artifact schema for the separately authorized Robot One inventory.
- Does not execute the inventory in this task.

- [x] **Step 1: Write a failing static runbook test**

Require that the runbook contains:

- an explicit “do not run without user authorization” gate;
- Asia/Shanghai time check before SSH;
- read-only command allowlist;
- forbidden command list containing `ros2 topic pub`, `ros2 service call`, `ros2 action send_goal`, `ros2 run`, `ros2 launch`, `ros2 param set`, `sudo`, and file/permission changes;
- sections for chassis, localization, waist, left arm, right arm, dual-arm controller, left gripper, right gripper, and box perception;
- artifact directory and SHA-256 manifest schema;
- stop condition when the reviewed dual-arm grasp/place controller or safety evidence is absent.
- every observation command is wrapped in a bounded timeout;
- sourcing `/home/rpp/rpp_ws/install/setup.bash` is allowed only after the
  operator confirms its path, ownership, and review status without changing it.

- [x] **Step 2: Run and verify RED**

```bash
.venv/bin/pytest -q tests/unit_tests/robots/lynsense_real_box/test_inventory_runbook.py
```

- [x] **Step 3: Write the runbook**

Use these observation-only command families after the user authorizes the inventory:

Before either `source` line, the operator must record the resolved workspace
path, file ownership, mode, and review decision using `readlink` and `stat`
output. If the workspace file is unreviewed or ownership is unexpected, stop the
inventory; do not source it and do not change permissions.

```bash
date -Is
readlink -f /home/rpp/rpp_ws
stat -c '%U:%G %a %n' /opt/ros/humble/setup.bash /home/rpp/rpp_ws/install/setup.bash
source /opt/ros/humble/setup.bash
source /home/rpp/rpp_ws/install/setup.bash
printenv ROS_DOMAIN_ID ROS_LOCALHOST_ONLY
timeout 10s ros2 node list
timeout 10s ros2 topic list -v
timeout 10s ros2 service list -v
timeout 10s ros2 action list -v
topic=/right_xarm/joint_states
timeout 10s ros2 topic info -v "$topic"
timeout 10s ros2 topic echo --once "$topic"
type=sensor_msgs/msg/JointState
timeout 10s ros2 interface show "$type"
```

For every additional observed state topic, repeat the four lines with that
topic and its observed type. The runbook must record each repetition; it must
not turn the example into an assumed interface for another subsystem.

Store only command output, timestamps, selected environment flags, interface names/types, and operator notes. If an interface is absent, record `present: false` and evidence; do not guess an alternate controller. The inventory run itself is not part of implementation.

- [x] **Step 4: Run static test**

Expected: runbook test passes.

---

### Task 9: Phase 1 Regression, Documentation, And Independent Review

**Files:**

- Create: `robots/lynsense_real_box/README.md`
- Modify: `docs/superpowers/plans/2026-09-22-lynsense-real-single-box-phase-1.md`

**Interfaces:**

- Produces Phase 1 completion evidence and explicit handoff requirements for the future ROS adapter plan.

- [x] **Step 1: Document Phase 1 limits**

README must state:

- this backend is offline dry-run only in Phase 1;
- no ROS client is created;
- live configuration is rejected by runtime;
- Robot One inventory and every live component require separate authorization;
- `stop_task` is not an E-stop;
- Webots and Isaac values are not real trajectories.

- [x] **Step 2: Run all focused tests**

```bash
.venv/bin/pytest -q tests/unit_tests/robots/lynsense_real_box \
  tests/unit_tests/rpent/robots/test_registry_contracts.py
```

Expected: all pass. Record exact count and duration.

Also run the established broad offline suite if its optional test dependencies
are present:

```bash
.venv/bin/pytest -q tests/unit_tests
```

If a broad-suite dependency is unavailable, record the exact missing dependency
and the focused suite result; do not install anything without authorization.

- [x] **Step 3: Run static and untracked-file checks**

```bash
.venv/bin/python -m compileall -q robots/lynsense_real_box \
  tests/unit_tests/robots/lynsense_real_box
git diff --check
while IFS= read -r -d '' file; do
  git diff --no-index --check /dev/null "$file"
done < <(git ls-files --others --exclude-standard \
  -- robots/lynsense_real_box tests/unit_tests/robots/lynsense_real_box \
  docs/superpowers/plans/2026-09-22-robot-one-readonly-inventory.md \
  docs/superpowers/plans/2026-09-22-lynsense-real-single-box-phase-1.md)
```

Run Ruff when it is available in the approved project environment:

```bash
.venv/bin/ruff format --check robots/lynsense_real_box \
  tests/unit_tests/robots/lynsense_real_box
.venv/bin/ruff check robots/lynsense_real_box \
  tests/unit_tests/robots/lynsense_real_box
```

If `.venv/bin/ruff` is absent, record `Ruff NOT RUN (not installed)`; do not
install it during this task.

Also scan new files for lines longer than 100 characters and trailing whitespace.

- [x] **Step 4: Negative import/network audit**

Add or run a focused test proving importing `robots.lynsense_real_box` and constructing the backend with a dry-run profile does not import `rclpy`, `lynsense_pytrees`, or open a socket.

- [x] **Step 5: Independent read-only implementation review**

Send the full tracked diff plus every new file to an independent reviewer. Required review points:

- no real ROS/hardware path;
- exact model tool table;
- execution legality rather than visibility-only safety;
- dual-arm/dual-gripper operation remains composite;
- dry-run dispatches nothing;
- evidence cannot leak secrets;
- state/finish/failure paths match the spec;
- inventory runbook contains no motion command.

Fix every Critical/Important finding, add focused regressions, rerun Step 2 and Step 3, and have the same reviewer re-review until APPROVED.

---

## Phase 1 Completion Definition

Phase 1 is complete only when:

1. all focused tests, compile checks, and diff checks pass;
2. independent implementation review is approved;
3. the README explicitly preserves the no-hardware boundary;
4. the inventory runbook exists but has not been executed;
5. the user has separately authorized the Robot One read-only inventory.

The next implementation plan may introduce a real ROS adapter only after the inventory record confirms exact interfaces and the user approves that Phase 2 plan.
