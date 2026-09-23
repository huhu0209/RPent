from __future__ import annotations

import ast
import json
from pathlib import Path
from threading import Event, Thread
from typing import Any

import pytest

from robots.lynsense_real_box.atomic_adapter import LynrotControlAtomicAdapter
from robots.lynsense_real_box.atomic_profile import (
    atomic_profile_hash,
    validate_atomic_capability_profile,
)
from robots.lynsense_real_box.atomic_toolkit import LynsenseAtomicToolkit
from robots.lynsense_real_box.evidence import EvidenceRecorder
from tests.unit_tests.robots.lynsense_real_box.test_atomic_adapter import (
    MutableClock,
    make_runtime,
)
from tests.unit_tests.robots.lynsense_real_box.test_atomic_profile import (
    live_profile,
    valid_atomic_profile,
    valid_v2_profile,
)


EXPECTED_TOOLS = [
    "read_capability_state",
    "detect_box",
    "move_chassis",
    "move_waist",
    "move_dual_arms",
    "set_dual_grippers",
    "stop_all",
    "finish",
]


class DashboardSink:
    def __init__(self) -> None:
        self.events: list[Any] = []

    def emit(self, event: Any) -> None:
        self.events.append(event)


class Memory:
    pass


class FakePerception:
    def __init__(self) -> None:
        self.detect_calls = 0
        self.close_calls = 0
        self.observed_mode: str | None = None

    def detect_box(self) -> dict[str, Any]:
        self.detect_calls += 1
        return {
            "status": "ok",
            "frame": "base_link",
            "pose": {
                "x": 0.887,
                "y": -0.036,
                "z": 0.428,
                "yaw_rad": 0.0,
            },
        }

    def close(self) -> None:
        self.close_calls += 1


class FlakyClosePerception(FakePerception):
    def __init__(self) -> None:
        super().__init__()
        self.remaining_failures = 1

    def close(self) -> None:
        if self.remaining_failures > 0:
            self.remaining_failures -= 1
            raise OSError("temporary close failure")
        super().close()


class FakePerceptionIdentity:
    def __init__(
        self,
        profile: Any,
        *,
        changes: dict[str, Any] | None = None,
    ) -> None:
        self.profile = profile
        self.changes = changes or {}

    def read(self) -> dict[str, Any]:
        binding = self.profile.perception_binding
        assert binding is not None
        receipt = {
            "profile_sha256": atomic_profile_hash(self.profile),
            "pose_topic": binding.pose_topic,
            "status_topic": binding.status_topic,
            "trigger_service": binding.trigger_service,
            "expected_frame": binding.expected_frame,
        }
        receipt.update(self.changes)
        return receipt


def profile():
    return validate_atomic_capability_profile(valid_atomic_profile())


def evidence_path(tmp_path: Path) -> Path:
    return tmp_path / "evidence" / "atomic.jsonl"


def make_toolkit(
    tmp_path: Path,
) -> tuple[LynsenseAtomicToolkit, LynrotControlAtomicAdapter, FakePerception, Path]:
    clock = MutableClock()
    robot, chassis = make_runtime(clock)
    adapter = LynrotControlAtomicAdapter(
        profile=profile(),
        robot=robot,
        chassis=chassis,
        now_s=clock.now,
        monotonic_s=clock.monotonic,
        sleep_s=clock.sleep,
        poll_interval_s=0.02,
    )
    assert adapter.connect()["status"] == "ok"
    perception = FakePerception()
    toolkit = LynsenseAtomicToolkit(
        adapter=adapter,
        perception=perception,
        evidence=EvidenceRecorder(evidence_path(tmp_path), profile()),
        dashboard_events=DashboardSink(),
        memory=Memory(),
        profile=profile(),
    )
    return toolkit, adapter, perception, evidence_path(tmp_path)


def events(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    return [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
    ]


def call(
    toolkit: LynsenseAtomicToolkit,
    name: str,
    arguments: dict[str, Any] | None = None,
) -> dict[str, Any]:
    return toolkit.execute_tool(name, arguments or {}).result


def test_atomic_tool_table_is_exact_and_has_no_injected_tools(tmp_path: Path) -> None:
    from rpent.planner.api_loop import _build_tools

    toolkit, _, _, _ = make_toolkit(tmp_path)
    assert toolkit.include_image_reader is False
    assert [item["name"] for item in toolkit.get_tools_spec()] == EXPECTED_TOOLS
    assert [tool.name for tool in _build_tools(toolkit)] == EXPECTED_TOOLS
    assert not {"read_image", "pick_box", "place_box"} & set(EXPECTED_TOOLS)


def test_successful_atomic_call_is_sealed_with_atomic_profile_evidence(
    tmp_path: Path,
) -> None:
    toolkit, _, _, path = make_toolkit(tmp_path)
    digest = atomic_profile_hash(profile())

    result = call(toolkit, "move_dual_arms", {"profile_name": "safe"})

    assert result["status"] == "ok"
    assert result["capability_mode"] == "idle"
    assert result["profile_sha256"] == digest
    recorded = events(path)
    assert [event["event"] for event in recorded] == [
        "tool_intent",
        "tool_call",
    ]
    assert recorded[0]["tool"] == "move_dual_arms"
    assert recorded[0]["arguments"] == {"profile_name": "safe"}
    assert recorded[0]["state_before"] == "idle"
    assert recorded[0]["profile_sha256"] == digest
    assert recorded[1]["tool"] == "move_dual_arms"
    assert recorded[1]["arguments"] == {"profile_name": "safe"}
    assert recorded[1]["state_before"] == "idle"
    assert recorded[1]["state_after"] == "idle"
    assert recorded[1]["profile_sha256"] == digest
    assert result["evidence_id"] == recorded[1]["evidence_id"]
    assert result["intent_evidence_id"] == recorded[0]["evidence_id"]
    assert recorded[1]["intent_evidence_id"] == recorded[0]["evidence_id"]


def test_detect_box_uses_owned_perception_once(tmp_path: Path) -> None:
    toolkit, _, perception, _ = make_toolkit(tmp_path)

    result = call(toolkit, "detect_box")

    assert result["status"] == "ok"
    assert result["frame"] == "base_link"
    assert result["capability_mode"] == "idle"
    assert perception.detect_calls == 1


def test_detect_box_uses_dedicated_running_mode(tmp_path: Path) -> None:
    toolkit, adapter, perception, _ = make_toolkit(tmp_path)

    original = perception.detect_box

    def observe_mode() -> dict[str, Any]:
        perception.observed_mode = adapter.capability_mode
        return original()

    perception.detect_box = observe_mode
    result = call(toolkit, "detect_box")

    assert result["status"] == "ok"
    assert perception.observed_mode == "perception_running"
    assert result["capability_mode"] == "idle"


def test_v2_toolkit_requires_exact_perception_identity_receipt(
    tmp_path: Path,
) -> None:
    v2 = validate_atomic_capability_profile(valid_v2_profile())
    clock = MutableClock()
    robot, chassis = make_runtime(clock)
    adapter = LynrotControlAtomicAdapter(
        profile=v2,
        robot=robot,
        chassis=chassis,
    )

    def build(identity: FakePerceptionIdentity | None) -> LynsenseAtomicToolkit:
        return LynsenseAtomicToolkit(
            adapter=adapter,
            perception=FakePerception(),
            perception_identity=identity,
            evidence=EvidenceRecorder(evidence_path(tmp_path), v2),
            dashboard_events=DashboardSink(),
            memory=Memory(),
            profile=v2,
        )

    with pytest.raises(ValueError, match="perception_identity_missing"):
        build(None)
    with pytest.raises(ValueError, match="perception_identity_mismatch"):
        build(
            FakePerceptionIdentity(
                v2,
                changes={"expected_frame": "camera_link"},
            )
        )
    valid = build(FakePerceptionIdentity(v2))
    assert [item["name"] for item in valid.get_tools_spec()] == EXPECTED_TOOLS
    valid.close()


def test_schema_or_profile_rejections_are_evidenced_without_dispatch(
    tmp_path: Path,
) -> None:
    toolkit, adapter, _, path = make_toolkit(tmp_path)
    original = adapter.move_waist
    adapter_calls: list[bool] = []

    def spy_move_waist(profile_name: str) -> dict[str, Any]:
        adapter_calls.append(True)
        return original(profile_name)

    adapter.move_waist = spy_move_waist
    extra = call(toolkit, "move_waist", {"profile_name": "safe", "speed": 0.1})
    unknown = call(toolkit, "move_waist", {"profile_name": "invented"})
    unknown_tool = call(toolkit, "move_left_arm", {})

    assert extra["status"] == "rejected"
    assert extra["reason"] == "schema_invalid"
    assert unknown["status"] == "rejected"
    assert unknown["reason"] == "unknown_profile"
    assert unknown_tool["status"] == "rejected"
    assert unknown_tool["reason"] == "unknown_tool"
    assert adapter_calls == []
    assert [event["tool"] for event in events(path)] == [
        "move_waist",
        "move_waist",
        "move_left_arm",
    ]


def test_intent_evidence_failure_blocks_physical_dispatch(tmp_path: Path) -> None:
    toolkit, adapter, _, _ = make_toolkit(tmp_path)
    original = toolkit._evidence.record_tool_intent

    def fail_intent(*args: Any, **kwargs: Any) -> None:
        raise OSError("evidence unavailable")

    toolkit._evidence.record_tool_intent = fail_intent
    before = list(adapter._robot.arms.left.move_calls)

    result = call(toolkit, "move_dual_arms", {"profile_name": "safe"})

    toolkit._evidence.record_tool_intent = original
    assert result["status"] == "rejected"
    assert result["reason"] == "handler_error"
    assert adapter._robot.arms.left.move_calls == before
    assert adapter._robot.arms.right.move_calls == []


def test_completion_evidence_failure_stops_and_records_terminal_failure(
    tmp_path: Path,
) -> None:
    toolkit, adapter, _, path = make_toolkit(tmp_path)
    original = toolkit._evidence.record_tool_call

    def fail_completion(*args: Any, **kwargs: Any) -> None:
        raise OSError("completion evidence unavailable")

    toolkit._evidence.record_tool_call = fail_completion

    result = call(toolkit, "move_dual_arms", {"profile_name": "safe"})

    toolkit._evidence.record_tool_call = original
    assert result["status"] == "failed"
    assert result["reason"] == "evidence_exception"
    assert result["stop_status"] == "ok"
    assert adapter.capability_mode == "operator_review"
    assert adapter._robot.arms.left.cancel_calls == 1
    assert adapter._robot.arms.right.cancel_calls == 1
    recorded = events(path)
    assert [event["event"] for event in recorded] == [
        "tool_intent",
        "tool_completion_failure",
    ]
    assert recorded[1]["intent_evidence_id"] == recorded[0]["evidence_id"]
    assert recorded[1]["reason"] == "completion_evidence_failed"
    assert recorded[1]["stop_status"] == "ok"


def test_perception_completion_evidence_failure_is_terminal(
    tmp_path: Path,
) -> None:
    toolkit, adapter, perception, path = make_toolkit(tmp_path)
    original = toolkit._evidence.record_tool_call

    def fail_completion(*args: Any, **kwargs: Any) -> None:
        raise OSError("completion evidence unavailable")

    toolkit._evidence.record_tool_call = fail_completion

    result = call(toolkit, "detect_box")

    toolkit._evidence.record_tool_call = original
    assert result["status"] == "failed"
    assert result["reason"] == "evidence_exception"
    assert result["stop_status"] == "ok"
    assert perception.detect_calls == 1
    assert adapter.capability_mode == "operator_review"
    assert [event["event"] for event in events(path)] == [
        "tool_intent",
        "tool_completion_failure",
    ]


def test_operator_acknowledgement_evidence_failure_does_not_unlock_review(
    tmp_path: Path,
) -> None:
    toolkit, adapter, _, _ = make_toolkit(tmp_path)
    assert call(toolkit, "stop_all")["status"] == "ok"
    original = toolkit._evidence.record_operator_acknowledgement

    def fail_ack(*args: Any, **kwargs: Any) -> None:
        raise OSError("ack evidence unavailable")

    toolkit._evidence.record_operator_acknowledgement = fail_ack
    result = toolkit.acknowledge_operator_review("huhu")
    toolkit._evidence.record_operator_acknowledgement = original

    assert result["status"] == "failed"
    assert result["reason"] == "evidence_exception"
    assert adapter.capability_mode == "operator_review"


def test_success_finish_is_not_solved_when_completion_evidence_fails(
    tmp_path: Path,
) -> None:
    toolkit, adapter, _, _ = make_toolkit(tmp_path)
    original = toolkit._evidence.record_tool_call

    def fail_finish(*args: Any, **kwargs: Any) -> None:
        raise OSError("finish evidence unavailable")

    toolkit._evidence.record_tool_call = fail_finish
    with pytest.raises(OSError, match="finish evidence unavailable"):
        toolkit.finish(status="success")
    toolkit._evidence.record_tool_call = original

    assert toolkit.solved() is False
    assert adapter.capability_mode == "idle"


def test_adapter_failure_opens_review_and_blocks_success_finish(tmp_path: Path) -> None:
    toolkit, adapter, _, path = make_toolkit(tmp_path)
    adapter._robot.arms.left.force_high_after = 2

    failed = call(toolkit, "move_dual_arms", {"profile_name": "safe"})
    finish_success = call(toolkit, "finish", {"status": "success"})
    finish_failure = call(toolkit, "finish", {"status": "failure"})

    assert failed["status"] == "failed"
    assert failed["reason"] == "force_threshold"
    assert failed["capability_mode"] == "operator_review"
    assert finish_success["status"] == "rejected"
    assert finish_success["reason"] == "unsafe_finish"
    assert finish_failure["status"] == "rejected"
    assert finish_failure["reason"] == "unsafe_finish"
    assert finish_failure["error"] == "finish refused"
    assert toolkit.solved() is False

    invalid_acknowledgement = toolkit.acknowledge_operator_review(" ")
    assert invalid_acknowledgement["status"] == "rejected"
    assert invalid_acknowledgement["reason"] == "operator_invalid"

    acknowledgement = toolkit.acknowledge_operator_review("huhu")
    assert acknowledgement["status"] == "ok"
    assert acknowledgement["capability_mode"] == "operator_review_acknowledged"

    acknowledged_failure = call(toolkit, "finish", {"status": "failure"})
    assert acknowledged_failure["status"] == "ok"
    assert acknowledged_failure["_finish"] is True
    assert toolkit.solved() is False
    acknowledgements = [
        event
        for event in events(path)
        if event["event"] == "operator_acknowledgement"
    ]
    assert acknowledgements[-1]["operator"] == "huhu"
    finish_events = [
        event
        for event in events(path)
        if event["event"] == "tool_call" and event["tool"] == "finish"
    ]
    assert finish_events[0]["result"]["error"] == "finish refused"


def test_stop_is_evidenced_with_mode_transition(tmp_path: Path) -> None:
    toolkit, _, _, path = make_toolkit(tmp_path)

    result = call(toolkit, "stop_all")

    assert result["status"] == "ok"
    assert result["gripper_stop_requested"] is False
    assert result["capability_mode"] == "operator_review"
    recorded = events(path)
    assert recorded[-1]["tool"] == "stop_all"
    assert recorded[-1]["state_before"] == "idle"
    assert recorded[-1]["state_after"] == "operator_review"


def test_cancel_active_and_wait_is_safe_without_active_operation(
    tmp_path: Path,
) -> None:
    toolkit, _, _, _ = make_toolkit(tmp_path)

    toolkit.cancel_active_and_wait()


def test_cancel_active_and_wait_rejects_unconfirmed_stop(tmp_path: Path) -> None:
    toolkit, adapter, _, _ = make_toolkit(tmp_path)
    adapter._capability_mode = "grippers_moving"
    adapter.stop_all = lambda: {
        "status": "failed",
        "reason": "stop_unconfirmed",
    }

    with pytest.raises(RuntimeError, match="stop_unconfirmed"):
        toolkit.cancel_active_and_wait()


def test_cancel_stops_active_tool_before_transport_dispatch(tmp_path: Path) -> None:
    toolkit, adapter, _, _ = make_toolkit(tmp_path)
    entered = Event()
    release = Event()
    results: list[dict[str, Any]] = []
    stop_calls: list[bool] = []
    original_check = adapter._dispatch_identity_rejection
    original_stop = adapter.stop_all

    def pause_before_dispatch() -> None:
        entered.set()
        assert release.wait(timeout=5)
        return original_check()

    def stop_then_release() -> dict[str, Any]:
        stop_calls.append(True)
        stopped = original_stop()
        release.set()
        return stopped

    adapter._dispatch_identity_rejection = pause_before_dispatch
    adapter.stop_all = stop_then_release
    worker = Thread(
        target=lambda: results.append(
            toolkit.execute_tool("move_waist", {"profile_name": "safe"}).result
        )
    )
    worker.start()
    try:
        assert entered.wait(timeout=5)
        toolkit.cancel_active_and_wait()
    finally:
        release.set()
        worker.join(timeout=5)
    assert not worker.is_alive()
    assert stop_calls == [True]
    assert results[0]["status"] != "ok"
    assert adapter._robot.waist.lift.move_calls == []


def test_close_is_idempotent_and_closes_owned_resources(tmp_path: Path) -> None:
    toolkit, adapter, perception, path = make_toolkit(tmp_path)

    toolkit.close()
    toolkit.close()

    assert perception.close_calls == 1
    assert adapter.capability_mode == "closed"
    with pytest.raises(Exception, match="closed"):
        toolkit._evidence.record_tool_call(
            "read_capability_state",
            {},
            {},
            "closed",
            "closed",
        )
    assert path.exists()


def test_profile_hash_disagreement_fails_before_tool_registration(
    tmp_path: Path,
) -> None:
    clock = MutableClock()
    robot, chassis = make_runtime(clock)
    mismatched_source = valid_atomic_profile()
    mismatched_source["profile_id"] = "robot1-atomic-mismatched-v1"
    adapter = LynrotControlAtomicAdapter(
        profile=validate_atomic_capability_profile(mismatched_source),
        robot=robot,
        chassis=chassis,
    )
    evidence = EvidenceRecorder(evidence_path(tmp_path), profile())
    try:
        with pytest.raises(ValueError, match="same SHA-256"):
            LynsenseAtomicToolkit(
                adapter=adapter,
                perception=FakePerception(),
                evidence=evidence,
                dashboard_events=DashboardSink(),
                memory=Memory(),
                profile=profile(),
            )
    finally:
        evidence.close()
        adapter.close()


def test_active_atomic_capability_blocks_close(tmp_path: Path) -> None:
    toolkit, adapter, _, _ = make_toolkit(tmp_path)
    adapter._capability_mode = "dual_arms_moving"

    with pytest.raises(RuntimeError, match="capability is active"):
        toolkit.close()

    assert adapter.capability_mode == "dual_arms_moving"
    adapter._capability_mode = "idle"
    toolkit.close()


def test_partial_toolkit_close_can_be_retried(tmp_path: Path) -> None:
    toolkit, adapter, _, _ = make_toolkit(tmp_path)
    perception = FlakyClosePerception()
    toolkit._perception = perception

    with pytest.raises(RuntimeError, match="atomic Toolkit close failed"):
        toolkit.close()

    toolkit.close()

    assert perception.remaining_failures == 0
    assert perception.close_calls == 1
    assert adapter.capability_mode == "closed"


def test_live_atomic_profile_cannot_construct_offline_toolkit(tmp_path: Path) -> None:
    clock = MutableClock()
    robot, chassis = make_runtime(clock)
    adapter = LynrotControlAtomicAdapter(
        profile=validate_atomic_capability_profile(live_profile()),
        robot=robot,
        chassis=chassis,
    )
    evidence = EvidenceRecorder(
        evidence_path(tmp_path),
        validate_atomic_capability_profile(live_profile()),
    )

    try:
        with pytest.raises(ValueError, match="rejects mode='live'"):
            LynsenseAtomicToolkit(
                adapter=adapter,
                perception=FakePerception(),
                evidence=evidence,
                dashboard_events=DashboardSink(),
                memory=Memory(),
                profile=validate_atomic_capability_profile(live_profile()),
            )
    finally:
        evidence.close()
        adapter.close()


def test_atomic_toolkit_has_no_transport_or_task_flow_imports() -> None:
    source = Path("robots/lynsense_real_box/atomic_toolkit.py").read_text(
        encoding="utf-8"
    )
    tree = ast.parse(source)
    imports = {
        node.names[0].name.split(".")[0]
        for node in ast.walk(tree)
        if isinstance(node, ast.Import)
    }
    imports |= {
        (node.module or "").split(".")[0]
        for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom)
    }
    assert not imports & {
        "rclpy",
        "rosidl",
        "lynrotcontrol",
        "lynsense_pytrees",
        "socket",
    }
