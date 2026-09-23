from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from pydantic_ai.messages import (
    ModelResponse,
    TextPart,
    ToolCallPart,
    ToolReturnPart,
)
from pydantic_ai.models.function import FunctionModel
from pydantic_ai.usage import RequestUsage

from rpent.dashboard.events import TranscriptEvent
from rpent.planner.api_loop import ApiAgentLoop
from tests.unit_tests.robots.lynsense_real_box.test_atomic_profile import (
    valid_atomic_profile,
)
from robots.lynsense_real_box.atomic_profile import (
    atomic_profile_hash,
    validate_atomic_capability_profile,
)
from tests.unit_tests.robots.lynsense_real_box.test_atomic_toolkit import (
    make_toolkit,
)


MAX_TURNS = 16

ATOMIC_STRATEGY: tuple[tuple[str, dict[str, Any]], ...] = (
    ("read_capability_state", {}),
    ("detect_box", {}),
    ("move_chassis", {"profile_name": "pickup"}),
    ("move_waist", {"profile_name": "safe"}),
    ("move_dual_arms", {"profile_name": "safe"}),
    ("set_dual_grippers", {"command": "close"}),
    ("move_chassis", {"profile_name": "retreat"}),
    ("move_dual_arms", {"profile_name": "safe"}),
    ("set_dual_grippers", {"command": "open"}),
    ("finish", {"status": "success"}),
)


class RecordingSink:
    def __init__(self) -> None:
        self.events: list[Any] = []

    @property
    def enabled(self) -> bool:
        return True

    def emit(self, event: Any) -> None:
        self.events.append(event)


def scripted_atomic_model(messages: list[Any], info: Any) -> ModelResponse:
    """Choose one atomic capability at a time after checking its result."""

    del info
    tool_returns = [
        part
        for message in messages
        for part in getattr(message, "parts", ())
        if isinstance(part, ToolReturnPart)
    ]
    call_index = sum(
        1
        for message in messages
        for part in getattr(message, "parts", ())
        if isinstance(part, ToolCallPart)
    )
    assert call_index <= len(ATOMIC_STRATEGY)
    if tool_returns:
        content = tool_returns[-1].content
        if isinstance(content, str):
            content = json.loads(content)
        assert isinstance(content, dict)
        assert content.get("status") == "ok", content
        assert content.get("reason") is None, content

    name, arguments = ATOMIC_STRATEGY[call_index]
    return ModelResponse(
        parts=[
            ToolCallPart(
                name,
                arguments,
                f"atomic-call-{call_index + 1}",
            )
        ],
        usage=RequestUsage(input_tokens=12, output_tokens=6),
    )


def test_real_api_planner_composes_atomic_capabilities_stepwise(tmp_path: Path) -> None:
    toolkit, adapter, _, evidence_path = make_toolkit(tmp_path)
    sink = RecordingSink()
    planner = ApiAgentLoop(
        FunctionModel(scripted_atomic_model),
        max_tokens=512,
        dashboard_events=sink,
        timeout_s=10,
    )

    try:
        result = planner.solve(
            system_prompt=(
                "Compose the box task from only the supplied atomic tools. "
                "The robot provides reviewed capabilities; it does not provide "
                "pick_box or place_box task flows. stop_all is a software stop, "
                "not an E-stop."
            ),
            user_message=(
                "Use the atomic capabilities to approach, coordinate both arms, "
                "carry with the chassis, release, and finish safely."
            ),
            toolkit=toolkit,
            max_turns=MAX_TURNS,
        )
    finally:
        toolkit.close()

    expected_calls = [
        {"tool": name, "args": arguments}
        for name, arguments in ATOMIC_STRATEGY
    ]
    actual_calls = [
        {
            "tool": event.payload["tool"],
            "args": event.payload["args"],
        }
        for event in sink.events
        if isinstance(event, TranscriptEvent)
        and event.payload.get("type") == "tool_call"
    ]
    assert result.error is None, result.error
    assert result.finish_result == {"_finish": True, "status": "success"}
    assert result.stats["turns_used"] == len(ATOMIC_STRATEGY)
    assert result.stats["tool_calls"] == len(ATOMIC_STRATEGY)
    assert actual_calls == expected_calls
    assert toolkit.solved() is True

    evidence = [
        json.loads(line)
        for line in evidence_path.read_text(encoding="utf-8").splitlines()
    ]
    profile = validate_atomic_capability_profile(valid_atomic_profile())
    digest = atomic_profile_hash(profile)
    completions = [
        event for event in evidence if event["event"] == "tool_call"
    ]
    assert [
        (event["tool"], event["arguments"]) for event in completions
    ] == list(ATOMIC_STRATEGY)
    assert all(event["profile_sha256"] == digest for event in evidence)
    assert completions[-1]["result"]["_finish"] is True

    robot = adapter._robot
    chassis = adapter._chassis
    dual_profile = profile.dual_arm_profiles["safe"]
    expected_left = [
        [list(point) for point in dual_profile.left_joints_deg]
    ] * 2
    expected_right = [
        [list(point) for point in dual_profile.right_joints_deg]
    ] * 2
    assert len(robot.arms.left.move_calls) == 2
    assert len(robot.arms.right.move_calls) == 2
    assert robot.arms.left.move_calls == expected_left
    assert robot.arms.right.move_calls == expected_right
    assert robot.grippers.left.move_calls == [0.1, 0.5]
    assert robot.grippers.right.move_calls == [0.1, 0.5]
    assert chassis.navigate_calls == ["pickup"]
    assert chassis.move_distance_calls == [(-0.6, 0.0)]

    transcript_path = tmp_path / "transcript-atomic-api-planner-proof.json"
    transcript_path.write_text(
        json.dumps(
            {
                "finish": result.finish_result,
                "stats": result.stats,
                "messages": result.messages,
            },
            default=str,
        ),
        encoding="utf-8",
    )
    assert digest in transcript_path.read_text(encoding="utf-8")


def test_rejected_atomic_finish_does_not_end_api_planner_run(
    tmp_path: Path,
) -> None:
    toolkit, adapter, _, _ = make_toolkit(tmp_path)
    adapter._robot.arms.left.force_high_after = 2
    calls: list[tuple[str, dict[str, Any]]] = []

    def model(messages: list[Any], info: Any) -> ModelResponse:
        del info
        tool_returns = [
            part
            for message in messages
            for part in getattr(message, "parts", ())
            if isinstance(part, ToolReturnPart)
        ]
        if tool_returns:
            content = tool_returns[-1].content
            if isinstance(content, str):
                content = json.loads(content)
            if len(calls) == 1:
                assert content["status"] == "failed"
                assert content["reason"] == "force_threshold"
            elif len(calls) == 2:
                assert content["status"] == "rejected"
                assert content["error"] == "finish refused"
            else:
                assert content["status"] == "ok"

        if len(calls) < 3:
            strategy = (
                ("move_dual_arms", {"profile_name": "safe"}),
                ("finish", {"status": "success"}),
                ("read_capability_state", {}),
            )
            name, arguments = strategy[len(calls)]
            calls.append((name, arguments))
            return ModelResponse(
                parts=[ToolCallPart(name, arguments, f"review-call-{len(calls)}")]
            )
        return ModelResponse(parts=[TextPart("Waiting for operator review.")])

    planner = ApiAgentLoop(
        FunctionModel(model),
        max_tokens=512,
        dashboard_events=RecordingSink(),
        timeout_s=10,
    )

    try:
        result = planner.solve(
            system_prompt="Use only the reviewed atomic tools.",
            user_message="Try to finish after the failed atomic motion.",
            toolkit=toolkit,
            max_turns=4,
        )
        mode_before_close = adapter.capability_mode
    finally:
        toolkit.close()

    assert result.error is None, result.error
    assert result.finish_result is None
    assert result.stats["tool_calls"] == 3
    assert calls[-1] == ("read_capability_state", {})
    assert mode_before_close == "operator_review"
