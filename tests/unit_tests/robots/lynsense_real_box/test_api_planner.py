from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from pydantic_ai.messages import (
    ModelResponse,
    ToolCallPart,
    ToolReturnPart,
)
from pydantic_ai.models.function import FunctionModel
from pydantic_ai.usage import RequestUsage

from robots.lynsense_real_box.site_profile import site_profile_hash
from rpent.dashboard.events import TranscriptEvent
from rpent.planner.api_loop import ApiAgentLoop
from tests.unit_tests.robots.lynsense_real_box.test_site_profile import (
    valid_profile,
)


MAX_TURNS = 32

REAL_BOX_CALLS: tuple[tuple[str, dict[str, Any]], ...] = (
    ("read_task_state", {}),
    ("read_robot_state", {}),
    ("detect_box", {}),
    ("nav_to_pose", {"goal": "pickup"}),
    ("move_waist", {"profile_name": "pick_ready"}),
    ("set_dual_grippers", {"command": "close"}),
    ("move_dual_arms", {"profile_name": "pick_ready"}),
    ("pick_box", {}),
    ("move_distance", {"profile_name": "retreat_after_pick"}),
    ("move_distance", {"profile_name": "turn_after_pick"}),
    ("nav_to_pose", {"goal": "placement"}),
    ("move_dual_arms", {"profile_name": "place_ready"}),
    ("place_box", {}),
    ("move_distance", {"profile_name": "retreat_after_place"}),
    ("move_dual_arms", {"profile_name": "safe"}),
    ("set_dual_grippers", {"command": "open"}),
    ("move_waist", {"profile_name": "safe"}),
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


def write_profile(tmp_path: Path) -> Path:
    path = tmp_path / "site-profile.json"
    path.write_text(json.dumps(valid_profile()), encoding="utf-8")
    return path


def tool_call_events(sink: RecordingSink) -> list[dict[str, Any]]:
    return [
        event.payload
        for event in sink.events
        if isinstance(event, TranscriptEvent)
        and event.payload.get("type") == "tool_call"
    ]


def scripted_real_box_model(messages: list[Any], info: Any) -> ModelResponse:
    """Return the next workflow step after checking the last guarded result."""

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
    assert call_index <= len(REAL_BOX_CALLS)
    if tool_returns:
        content = tool_returns[-1].content
        if isinstance(content, str):
            content = json.loads(content)
        assert isinstance(content, dict)
        assert content.get("status") == "ok", content
        assert content.get("reason") is None, content

    name, arguments = REAL_BOX_CALLS[call_index]
    return ModelResponse(
        parts=[ToolCallPart(name, arguments, f"real-box-call-{call_index + 1}")],
        usage=RequestUsage(input_tokens=10, output_tokens=5),
    )


def offline_backend_arguments(tmp_path: Path) -> argparse.Namespace:
    output_dir = tmp_path / "output"
    return argparse.Namespace(
        planner="api",
        memory_profile="local",
        memory_dir=tmp_path / "memory",
        output_dir=output_dir,
        dashboard=False,
        interactive=False,
        explore=False,
        site_profile=write_profile(tmp_path),
    )


def read_evidence(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]


def test_api_loop_drives_the_complete_single_box_workflow(tmp_path: Path) -> None:
    from rpent.robots import get_robot_spec, get_toolkit

    args = offline_backend_arguments(tmp_path)
    spec = get_robot_spec("lynsense_real_box")
    config = spec.parse_config(args)
    _, runtime_kwargs = spec.init_runtime(args, config.output_dir, RecordingSink(), None)
    sink = RecordingSink()
    toolkit = get_toolkit(
        "lynsense_real_box",
        runtime_kwargs=runtime_kwargs,
        dashboard_events=sink,
        config=config,
    )
    planner = ApiAgentLoop(
        FunctionModel(scripted_real_box_model),
        max_tokens=512,
        dashboard_events=sink,
        timeout_s=10,
    )

    try:
        result = planner.solve(
            system_prompt=spec.prompts.render("system", variables={}),
            user_message=spec.prompts.render("user", variables={}),
            toolkit=toolkit,
            max_turns=MAX_TURNS,
        )
    finally:
        toolkit.close()

    expected_calls = [
        {"tool": name, "args": arguments} for name, arguments in REAL_BOX_CALLS
    ]
    actual_calls = [
        {"tool": event["tool"], "args": event["args"]}
        for event in tool_call_events(sink)
    ]
    assert result.error is None, result.error
    assert result.finish_result == {"_finish": True, "status": "success"}
    assert result.stats["turns_used"] == len(REAL_BOX_CALLS)
    assert result.stats["turns_used"] > 10
    assert result.stats["tool_calls"] == len(REAL_BOX_CALLS)
    assert actual_calls == expected_calls

    evidence_path = config.output_dir / "events-lynsense_real_box.jsonl"
    evidence = read_evidence(evidence_path)
    profile = runtime_kwargs["profile"]
    digest = site_profile_hash(profile)
    assert all(event["profile_sha256"] == digest for event in evidence)
    assert [
        (event["tool"], event["arguments"]) for event in evidence
    ] == list(REAL_BOX_CALLS)
    assert evidence[-1]["tool"] == "finish"
    assert evidence[-1]["result"]["task_state"] == "safe_complete"

    adapter_calls = toolkit._adapter.calls
    methods = {call["method"] for call in adapter_calls}
    assert {"navigate", "move_distance", "move_waist", "move_dual_arms",
            "set_dual_grippers"} <= methods
    gripper_calls = [
        call for call in adapter_calls if call["method"] == "set_dual_grippers"
    ]
    assert len(gripper_calls) % 2 == 0
    for left, right in zip(gripper_calls[::2], gripper_calls[1::2]):
        assert left["interface_role"] == "left_gripper"
        assert right["interface_role"] == "right_gripper"
        assert left["request"] == right["request"]
        assert set(left["request"]) == {"command"}

    transcript_path = config.output_dir / "transcript-api-planner-proof.json"
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
    assert transcript_path.exists()
    assert digest in transcript_path.read_text(encoding="utf-8")
