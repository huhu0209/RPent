from __future__ import annotations

import json
import threading
from pathlib import Path
from typing import Any

import pytest

from robots.lynsense_real_box.dry_run_adapter import OfflineDryRunAdapter
from robots.lynsense_real_box.evidence import EvidenceRecorder
from robots.lynsense_real_box.site_profile import SiteProfile, site_profile_hash
from robots.lynsense_real_box.task_state import TaskState, TaskStateMachine
from robots.lynsense_real_box.toolkit import LynsenseRealBoxToolkit
from tests.unit_tests.robots.lynsense_real_box.test_site_profile import (
    valid_profile,
)


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


class DashboardSink:
    def __init__(self) -> None:
        self.events: list[Any] = []

    def emit(self, event: Any) -> None:
        self.events.append(event)


class Memory:
    pass


class BlockingAdapter(OfflineDryRunAdapter):
    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self.pick_started = threading.Event()
        self.release_pick = threading.Event()

    def pick_box(self) -> dict[str, Any]:
        self.pick_started.set()
        self.release_pick.wait(timeout=2.0)
        return super().pick_box()


def robot_state() -> dict[str, Any]:
    return {
        "age_s": 0.1,
        "healthy": True,
        "mode": "dry_run",
        "left_arm_error": False,
        "right_arm_error": False,
    }


def box_pose() -> dict[str, Any]:
    return {
        "age_s": 0.2,
        "frame": "offline_box_fixture",
        "transform_valid": True,
        "pose": {"x": 1.0, "y": -2.0, "z": 0.1, "yaw_rad": 0.0},
    }


def placement_result() -> dict[str, Any]:
    return {
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


def make_adapter(**changes: Any) -> OfflineDryRunAdapter:
    profile = make_profile()
    fixtures: dict[str, Any] = {
        "robot_state": robot_state(),
        "box_pose": box_pose(),
        "placement_result": placement_result(),
    }
    fixtures.update(changes)
    adapter = OfflineDryRunAdapter(profile, **fixtures)
    assert adapter.connect()["status"] == "ok"
    return adapter


def make_profile() -> SiteProfile:
    from robots.lynsense_real_box.site_profile import validate_site_profile

    return validate_site_profile(valid_profile())


def evidence_path(tmp_path: Path) -> Path:
    return tmp_path / "evidence" / "run.jsonl"


def make_toolkit(
    tmp_path: Path,
    *,
    adapter: OfflineDryRunAdapter | None = None,
    machine: TaskStateMachine | None = None,
) -> tuple[LynsenseRealBoxToolkit, OfflineDryRunAdapter, TaskStateMachine, EvidenceRecorder]:
    profile = make_profile()
    actual_adapter = adapter or make_adapter()
    actual_machine = machine or TaskStateMachine(move_transitions={
        item.name: item for item in profile.move_transitions
    })
    recorder = EvidenceRecorder(evidence_path(tmp_path), profile)
    toolkit = LynsenseRealBoxToolkit(
        adapter=actual_adapter,
        state_machine=actual_machine,
        evidence=recorder,
        dashboard_events=DashboardSink(),
        memory=Memory(),
        profile=profile,
    )
    return toolkit, actual_adapter, actual_machine, recorder


def events(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text().splitlines()]


def call(
    toolkit: LynsenseRealBoxToolkit,
    name: str,
    arguments: dict[str, Any] | None = None,
) -> dict[str, Any]:
    return toolkit.execute_tool(name, arguments or {}).result


def advance(
    toolkit: LynsenseRealBoxToolkit,
    name: str,
    arguments: dict[str, Any] | None = None,
) -> dict[str, Any]:
    result = call(toolkit, name, arguments)
    assert result["status"] == "ok", result
    assert result["reason"] is None
    return result


def assert_common_result(result: dict[str, Any], state: str) -> None:
    assert {"status", "reason", "task_state", "evidence_id"} <= set(result)
    assert result["task_state"] == state


def strict_schemas(value: Any) -> None:
    if isinstance(value, dict):
        if value.get("type") == "object":
            assert value.get("additionalProperties") is False
        for child in value.values():
            strict_schemas(child)
    elif isinstance(value, list):
        for child in value:
            strict_schemas(child)


def prepared_pick_toolkit(tmp_path: Path):
    toolkit, adapter, machine, recorder = make_toolkit(tmp_path)
    advance(toolkit, "detect_box")
    advance(toolkit, "nav_to_pose", {"goal": "pickup"})
    advance(toolkit, "move_waist", {"profile_name": "pick_ready"})
    advance(toolkit, "set_dual_grippers", {"command": "close"})
    advance(toolkit, "move_dual_arms", {"profile_name": "pick_ready"})
    assert machine.state is TaskState.DUAL_PICK_PREPARED
    return toolkit, adapter, machine, recorder


def withdrawn_toolkit(tmp_path: Path):
    toolkit, adapter, machine, recorder = prepared_pick_toolkit(tmp_path)
    advance(toolkit, "pick_box")
    advance(toolkit, "move_distance", {"profile_name": "retreat_after_pick"})
    advance(toolkit, "move_distance", {"profile_name": "turn_after_pick"})
    advance(toolkit, "nav_to_pose", {"goal": "placement"})
    advance(toolkit, "move_dual_arms", {"profile_name": "place_ready"})
    advance(toolkit, "place_box")
    advance(toolkit, "move_distance", {"profile_name": "retreat_after_place"})
    assert machine.state is TaskState.WITHDRAWN
    return toolkit, adapter, machine, recorder


def test_tool_table_is_exact_and_has_no_common_or_image_tools(tmp_path: Path) -> None:
    from rpent.planner.api_loop import _build_tools

    toolkit, _, _, _ = make_toolkit(tmp_path)
    assert toolkit.include_image_reader is False
    toolkit._register_common_tools()
    assert [item["name"] for item in toolkit.get_tools_spec()] == EXPECTED_TOOLS
    assert [tool.name for tool in _build_tools(toolkit)] == EXPECTED_TOOLS
    assert "read_image" not in EXPECTED_TOOLS


def test_schemas_are_closed_and_finish_has_exact_status_enum(tmp_path: Path) -> None:
    toolkit, _, _, _ = make_toolkit(tmp_path)
    specs = toolkit.get_tools_spec()
    strict_schemas(specs)
    finish = next(item for item in specs if item["name"] == "finish")
    assert finish["input_schema"]["properties"]["status"]["enum"] == [
        "success",
        "failure",
    ]


def test_valid_transition_calls_adapter_and_evidence_once(tmp_path: Path) -> None:
    toolkit, adapter, machine, recorder = make_toolkit(tmp_path)
    adapter_calls: list[bool] = []
    original_detect = adapter.detect_box

    def spy_detect() -> dict[str, Any]:
        adapter_calls.append(True)
        return original_detect()

    monkeypatch = pytest.MonkeyPatch()
    monkeypatch.setattr(adapter, "detect_box", spy_detect)
    before_state = machine.state.value
    result = advance(toolkit, "detect_box")
    monkeypatch.undo()

    assert machine.state is TaskState.BOX_LOCALIZED
    assert len(adapter_calls) == 1
    assert_common_result(result, machine.state.value)
    assert len(events(evidence_path(tmp_path))) == 1
    assert recorder._event_count == 1
    assert before_state == TaskState.INITIALIZED.value


def test_successful_motion_evidence_preserves_tool_arguments(tmp_path: Path) -> None:
    toolkit, _, machine, _ = make_toolkit(tmp_path)
    advance(toolkit, "detect_box")
    advance(toolkit, "nav_to_pose", {"goal": "pickup"})

    motion_events = [
        event
        for event in events(evidence_path(tmp_path))
        if event["tool"] == "nav_to_pose"
    ]

    assert machine.state is TaskState.AT_PICK_APPROACH
    assert len(motion_events) == 1
    assert motion_events[0]["arguments"] == {"goal": "pickup"}


def test_read_task_state_contains_the_model_profile_transcript(tmp_path: Path) -> None:
    toolkit, _, machine, _ = make_toolkit(tmp_path)
    result = advance(toolkit, "read_task_state")

    profile = make_profile()
    assert result["profile_id"] == profile.profile_id
    assert result["profile_sha256"] == site_profile_hash(profile)
    assert result["task_state"] == machine.state.value


def test_rejections_are_evidenced_without_adapter_or_transition(
    tmp_path: Path,
) -> None:
    toolkit, adapter, machine, _ = make_toolkit(tmp_path)
    result = call(toolkit, "pick_box")

    assert result["status"] == "rejected"
    assert result["reason"]
    assert_common_result(result, "initialized")
    assert machine.state is TaskState.INITIALIZED
    assert adapter.calls == []
    assert len(events(evidence_path(tmp_path))) == 1


def test_unknown_move_profile_and_wrong_from_state_are_rejected(
    tmp_path: Path,
) -> None:
    toolkit, adapter, machine, _ = make_toolkit(tmp_path)
    advance(toolkit, "detect_box")
    advance(toolkit, "nav_to_pose", {"goal": "pickup"})
    adapter.calls.clear()

    unknown = call(toolkit, "move_distance", {"profile_name": "retreat_after_pick"})
    assert unknown["status"] == "rejected"
    assert unknown["task_state"] == "at_pick_approach"

    advance(toolkit, "move_waist", {"profile_name": "pick_ready"})
    advance(toolkit, "set_dual_grippers", {"command": "close"})
    advance(toolkit, "move_dual_arms", {"profile_name": "pick_ready"})
    advance(toolkit, "pick_box")
    intended_calls = list(adapter.calls)
    wrong = call(toolkit, "move_distance", {"profile_name": "retreat_after_place"})
    assert wrong["status"] == "rejected"
    assert machine.state is TaskState.BOX_GRASPED
    assert adapter.calls == intended_calls


def test_unsafe_finish_and_extra_argument_are_rejected(tmp_path: Path) -> None:
    toolkit, _, machine, _ = make_toolkit(tmp_path)
    unsafe = call(
        toolkit,
        "finish",
        {"status": "success"},
    )
    extra = call(toolkit, "detect_box", {"unexpected": True})

    assert unsafe["status"] == "rejected"
    assert unsafe["reason"] == "unsafe_finish"
    assert extra["status"] == "rejected"
    assert machine.state is TaskState.INITIALIZED
    assert len(events(evidence_path(tmp_path))) == 2


def test_failure_finish_is_rejected_before_operator_acknowledgement(
    tmp_path: Path,
) -> None:
    machine = TaskStateMachine(TaskState.OPERATOR_REVIEW)
    toolkit, _, machine, _ = make_toolkit(tmp_path, machine=machine)

    executed = toolkit.execute_tool(
        "finish",
        {"status": "failure"},
    )
    result = executed.result

    assert result["status"] == "rejected"
    assert result["reason"] == "unsafe_finish"
    assert result["task_state"] == "operator_review"
    assert result["evidence_id"]
    assert "_finish" not in result
    assert executed.is_finish is False
    assert machine.state is TaskState.OPERATOR_REVIEW
    assert len(events(evidence_path(tmp_path))) == 1


def test_adapter_failure_applies_failure_kind_and_hazard(tmp_path: Path) -> None:
    toolkit, adapter, machine, _ = prepared_pick_toolkit(tmp_path)
    adapter.fail_next(
        "pick_box",
        {
            "status": "failed",
            "failure_kind": "action_failed",
            "box_hazard": True,
        },
    )
    result = call(toolkit, "pick_box")

    assert result["status"] == "failed"
    assert result["reason"] == "action_failed"
    assert result["box_hazard"] is True
    assert machine.state is TaskState.FAILED_STOPPING
    assert_common_result(result, machine.state.value)


def test_adapter_exception_is_structured_and_enters_stopping(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    toolkit, adapter, machine, _ = prepared_pick_toolkit(tmp_path)

    def fail() -> dict[str, Any]:
        raise RuntimeError("adapter exploded")

    monkeypatch.setattr(adapter, "pick_box", fail)
    result = call(toolkit, "pick_box")
    assert result == {
        "status": "failed",
        "reason": "adapter_exception",
        "task_state": "failed_stopping",
        **{key: result[key] for key in ("evidence_id",)},
    }
    assert machine.state is TaskState.FAILED_STOPPING


def test_successful_and_failure_finish_lifecycles(tmp_path: Path) -> None:
    toolkit, _, machine, _ = make_toolkit(tmp_path)
    advance(toolkit, "detect_box")
    advance(toolkit, "nav_to_pose", {"goal": "pickup"})
    advance(toolkit, "move_waist", {"profile_name": "pick_ready"})
    advance(toolkit, "set_dual_grippers", {"command": "close"})
    advance(toolkit, "move_dual_arms", {"profile_name": "pick_ready"})
    advance(toolkit, "pick_box")
    advance(toolkit, "move_distance", {"profile_name": "retreat_after_pick"})
    advance(toolkit, "move_distance", {"profile_name": "turn_after_pick"})
    advance(toolkit, "nav_to_pose", {"goal": "placement"})
    advance(toolkit, "move_dual_arms", {"profile_name": "place_ready"})
    advance(toolkit, "place_box")
    advance(toolkit, "move_distance", {"profile_name": "retreat_after_place"})
    advance(toolkit, "move_dual_arms", {"profile_name": "safe"})
    advance(toolkit, "set_dual_grippers", {"command": "open"})
    advance(toolkit, "move_waist", {"profile_name": "safe"})
    finish = advance(
        toolkit,
        "finish",
        {"status": "success"},
    )

    assert machine.state is TaskState.SAFE_COMPLETE
    assert finish["_finish"] is True


def test_stop_task_reports_a_best_effort_stop_not_an_estop(tmp_path: Path) -> None:
    toolkit, _, machine, _ = prepared_pick_toolkit(tmp_path)
    result = advance(toolkit, "stop_task")

    assert result["estop"] is False
    assert machine.state is TaskState.FAILED_STOPPED


@pytest.mark.parametrize(
    ("initial_state", "name", "arguments"),
    [
        (TaskState.AT_PICK_APPROACH, "move_waist", {"profile_name": "safe"}),
        (TaskState.AT_PICK_APPROACH, "move_dual_arms", {"profile_name": "safe"}),
        (TaskState.AT_PICK_APPROACH, "set_dual_grippers", {"command": "open"}),
        (TaskState.AT_PLACE_APPROACH, "move_dual_arms", {"profile_name": "safe"}),
        (TaskState.WITHDRAWN, "move_waist", {"profile_name": "pick_ready"}),
        (TaskState.WITHDRAWN, "move_dual_arms", {"profile_name": "pick_ready"}),
        (TaskState.WITHDRAWN, "set_dual_grippers", {"command": "close"}),
    ],
)
def test_context_mismatched_preparations_are_rejected_before_dispatch(
    tmp_path: Path,
    initial_state: TaskState,
    name: str,
    arguments: dict[str, str],
) -> None:
    profile = make_profile()
    machine = TaskStateMachine(
        initial_state,
        move_transitions={item.name: item for item in profile.move_transitions},
    )
    toolkit, adapter, machine, _ = make_toolkit(tmp_path, machine=machine)

    def forbidden_profile() -> dict[str, str]:
        raise AssertionError("a context-mismatched preparation reached the adapter")

    monkeypatch = pytest.MonkeyPatch()
    monkeypatch.setattr(adapter, name, forbidden_profile)
    result = call(toolkit, name, arguments)
    monkeypatch.undo()

    assert result["status"] == "rejected"
    assert result["reason"] == "wrong_profile_for_state"
    assert result["task_state"] == machine.state.value
    assert adapter.calls == []


def test_withdrawn_wrong_preparation_cannot_enable_success_finish(
    tmp_path: Path,
) -> None:
    toolkit, _, machine, _ = withdrawn_toolkit(tmp_path)

    result = call(toolkit, "move_waist", {"profile_name": "pick_ready"})

    assert result["status"] == "rejected"
    assert machine.state is TaskState.WITHDRAWN
    unsafe_finish = call(toolkit, "finish", {"status": "success"})
    assert unsafe_finish["status"] == "rejected"
    assert unsafe_finish["reason"] == "unsafe_finish"


@pytest.mark.parametrize(
    "initial_state",
    [TaskState.BOX_GRASPED, TaskState.CARRYING],
)
def test_stop_from_held_box_requires_operator_confirmation(
    tmp_path: Path,
    initial_state: TaskState,
) -> None:
    profile = make_profile()
    machine = TaskStateMachine(
        initial_state,
        move_transitions={item.name: item for item in profile.move_transitions},
    )
    toolkit, _, machine, _ = make_toolkit(tmp_path, machine=machine)

    result = advance(toolkit, "stop_task")

    assert result["status"] == "ok"
    assert result["task_state"] == TaskState.OPERATOR_REVIEW.value
    assert result["estop"] is False
    assert machine.requires_operator_review is True

    unsafe_finish = call(toolkit, "finish", {"status": "success"})
    assert unsafe_finish["status"] == "rejected"
    assert unsafe_finish["reason"] == "unsafe_finish"

    machine.mark_operator_stopped()
    failure_finish = advance(toolkit, "finish", {"status": "failure"})
    assert failure_finish["_finish"] is True


def test_close_calls_adapter_and_evidence_exactly_once(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    toolkit, adapter, _, recorder = make_toolkit(tmp_path)
    adapter_calls = 0
    evidence_calls = 0
    original_adapter_close = adapter.close
    original_evidence_close = recorder.close

    def count_adapter() -> None:
        nonlocal adapter_calls
        adapter_calls += 1
        original_adapter_close()

    def count_evidence() -> None:
        nonlocal evidence_calls
        evidence_calls += 1
        original_evidence_close()

    monkeypatch.setattr(adapter, "close", count_adapter)
    monkeypatch.setattr(recorder, "close", count_evidence)
    toolkit.close()
    toolkit.close()
    assert (adapter_calls, evidence_calls) == (1, 1)


def test_dispatch_records_unknown_concurrent_and_unrecorded_results(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    toolkit, adapter, _, _ = make_toolkit(tmp_path)
    unknown = call(toolkit, "not_a_tool")
    assert "error" in unknown and unknown["evidence_id"]

    spec, _ = toolkit._tools["read_task_state"]
    monkeypatch.setitem(
        toolkit._tools,
        "read_task_state",
        (spec, lambda: {"error": "handler forgot evidence"}),
    )
    forgotten = call(toolkit, "read_task_state")
    assert forgotten["evidence_id"]

    started = threading.Event()
    release = threading.Event()

    def blocked() -> dict[str, Any]:
        started.set()
        release.wait(timeout=2.0)
        return {"status": "ok", "reason": None}

    monkeypatch.setitem(toolkit._tools, "read_task_state", (spec, blocked))
    first = threading.Thread(
        target=lambda: call(toolkit, "read_task_state"),
        daemon=True,
    )
    first.start()
    assert started.wait(timeout=2.0)
    concurrent = call(toolkit, "read_task_state")
    assert concurrent["error"] == "another tool operation is still active"
    assert concurrent["evidence_id"]
    release.set()
    first.join(timeout=2.0)
    assert not first.is_alive()
    unknown_profile = call(toolkit, "nav_to_pose", {"goal": "unknown"})
    assert unknown_profile["status"] == "rejected"


def test_all_successful_results_carry_the_common_envelope(tmp_path: Path) -> None:
    toolkit, _, machine, _ = make_toolkit(tmp_path)
    steps = [
        ("detect_box", None),
        ("nav_to_pose", {"goal": "pickup"}),
        ("move_waist", {"profile_name": "pick_ready"}),
        ("set_dual_grippers", {"command": "close"}),
        ("move_dual_arms", {"profile_name": "pick_ready"}),
        ("pick_box", None),
        ("move_distance", {"profile_name": "retreat_after_pick"}),
        ("move_distance", {"profile_name": "turn_after_pick"}),
        ("nav_to_pose", {"goal": "placement"}),
        ("move_dual_arms", {"profile_name": "place_ready"}),
        ("place_box", None),
        ("move_distance", {"profile_name": "retreat_after_place"}),
        ("move_dual_arms", {"profile_name": "safe"}),
        ("set_dual_grippers", {"command": "open"}),
        ("move_waist", {"profile_name": "safe"}),
        (
            "finish",
            {"status": "success"},
        ),
    ]
    for name, arguments in steps:
        result = advance(toolkit, name, arguments)
        assert_common_result(result, machine.state.value)
