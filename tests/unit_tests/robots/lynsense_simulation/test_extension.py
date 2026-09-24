"""Offline contracts for the simulation-only Lynsense RPent backend."""

from __future__ import annotations

import argparse
import copy
import importlib
import json
import os
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
from pydantic_ai.messages import (
    ModelResponse,
    TextPart,
    ToolCallPart,
    ToolReturnPart,
)
from pydantic_ai.models.function import FunctionModel

from robots.lynsense.simulation.rpent_plan import (
    PLAN_VERSION,
    TASK_ID,
    PlanError,
    canonical_single_box_plan,
    task_description,
)
from robots.lynsense_simulation import get_robot_spec, get_toolkit
from robots.lynsense_simulation import robot_spec
from rpent.dashboard.events import NullDashboardEventSink
from rpent.planner.api_loop import ApiAgentLoop, _build_tools
from rpent.robots import enumerate_robots


@pytest.fixture(autouse=True)
def block_outbound_connections(monkeypatch):
    def reject(*args, **kwargs):
        pytest.fail("simulation backend attempted a network connection")

    monkeypatch.setattr("socket.socket.connect", reject)
    monkeypatch.setattr("socket.socket.connect_ex", reject)
    monkeypatch.setattr("socket.getaddrinfo", reject)


@pytest.fixture(autouse=True)
def output_directory(tmp_path, monkeypatch):
    from rpent.utils import logging

    monkeypatch.setattr(logging, "_output_dir", tmp_path)


def args_for(tmp_path: Path, **changes: object) -> argparse.Namespace:
    args = argparse.Namespace(
        planner="api",
        memory_profile="local",
        memory_dir=None,
        output_dir=str(tmp_path),
        dashboard=False,
        interactive=False,
        explore=False,
    )
    for name, value in changes.items():
        setattr(args, name, value)
    return args


def make_toolkit(tmp_path: Path, **changes: object):
    spec = get_robot_spec()
    args = args_for(tmp_path, **changes)
    config = spec.parse_config(args)
    daemons, runtime_kwargs = spec.init_runtime(
        args, tmp_path, NullDashboardEventSink(), None
    )
    assert daemons == []
    return get_toolkit(
        runtime_kwargs=runtime_kwargs,
        config=config,
        dashboard_events=NullDashboardEventSink(),
    )


def _tool(toolkit, name: str) -> dict:
    return next(item for item in toolkit.get_tools_spec() if item["name"] == name)


def test_registration_is_lazy_and_simulation_only(monkeypatch):
    for module_name in ("rclpy", "sensor_msgs", "std_msgs", "action_msgs"):
        monkeypatch.setitem(sys.modules, module_name, None)
    importlib.reload(robot_spec)

    assert "lynsense_simulation" in enumerate_robots()
    spec = get_robot_spec()
    assert spec.name == "lynsense_simulation"
    assert spec.is_real_robot is False
    assert spec.supports_exploration is False
    assert spec.supports_human_interactive_exploration is False
    assert "read_simulation_task" in spec.prompts.render("system")
    assert all(
        sys.modules.get(module) is None
        for module in ("rclpy", "action_msgs")
    )


@pytest.mark.parametrize(
    "change",
    [
        {"planner": "codex"},
        {"planner": "claude_code"},
        {"planner": "flash"},
        {"memory_profile": "hf"},
        {"memory_profile": None},
        {"dashboard": True},
        {"interactive": True},
        {"explore": True},
    ],
)
def test_parse_config_rejects_unsupported_run_modes(tmp_path, change):
    with pytest.raises(ValueError):
        get_robot_spec().parse_config(args_for(tmp_path, **change))


def test_runtime_has_no_resources_and_exact_prompt_plan_path(tmp_path):
    spec = get_robot_spec()
    args = args_for(tmp_path)
    assert spec.init_runtime(args, tmp_path, NullDashboardEventSink(), set()) == ([], {})
    with pytest.raises(ValueError, match="component"):
        spec.init_runtime(args, tmp_path, NullDashboardEventSink(), {"env"})

    config = spec.parse_config(args)
    assert config.recipe_tag == "lynsense_simulation_single_box"
    assert config.task_desc == {
        "task_id": TASK_ID,
        "environment": "webots-isolated",
    }
    assert config.prompt_vars["plan_path"] == str(tmp_path / "plan.json")


def test_tool_table_and_schemas_are_strict(tmp_path):
    toolkit = make_toolkit(tmp_path)
    try:
        names = [item["name"] for item in toolkit.get_tools_spec()]
        assert names == ["read_simulation_task", "submit_simulation_plan", "finish"]
        assert [tool.name for tool in _build_tools(toolkit)] == names

        read_schema = _tool(toolkit, "read_simulation_task")["input_schema"]
        assert read_schema == {
            "type": "object",
            "properties": {},
            "additionalProperties": False,
        }

        submit_schema = _tool(toolkit, "submit_simulation_plan")["input_schema"]
        assert submit_schema["additionalProperties"] is False
        assert submit_schema["required"] == ["plan"]
        assert set(submit_schema["properties"]) == {"plan"}

        finish_schema = _tool(toolkit, "finish")["input_schema"]
        assert finish_schema["additionalProperties"] is False
        assert finish_schema["required"] == ["status", "summary"]
        assert finish_schema["properties"]["status"]["enum"] == [
            "success",
            "failure",
            "stuck",
        ]
    finally:
        toolkit.close()


def test_read_simulation_task_is_readonly(tmp_path):
    toolkit = make_toolkit(tmp_path)
    try:
        result = toolkit.execute_tool("read_simulation_task", {})
        assert result.result == {"task": task_description()}
        assert result.result["task"]["required_action_count"] == 15
        with pytest.raises(TypeError):
            toolkit.read_simulation_task(unexpected=True)
    finally:
        toolkit.close()


def test_submit_rejects_invalid_plan_without_replacing_output(tmp_path):
    toolkit = make_toolkit(tmp_path)
    plan_path = Path(toolkit._plan_path)
    plan_path.parent.mkdir(parents=True, exist_ok=True)
    plan_path.write_text("preserve-me", encoding="utf-8")
    original_stat = plan_path.stat()
    try:
        invalid = {
            "task_id": TASK_ID,
            "version": PLAN_VERSION,
            "actions": canonical_single_box_plan()[:-1],
        }
        result = toolkit.execute_tool("submit_simulation_plan", {"plan": invalid})
        assert "error" in result.result
        assert plan_path.read_text(encoding="utf-8") == "preserve-me"
        assert plan_path.stat().st_mtime_ns == original_stat.st_mtime_ns
    finally:
        toolkit.close()


def test_submit_writes_accepted_plan_atomically(tmp_path, monkeypatch):
    toolkit = make_toolkit(tmp_path)
    plan_path = Path(toolkit._plan_path)
    replacements = []
    original_replace = os.replace

    def tracked_replace(source, target):
        replacements.append((Path(source), Path(target)))
        return original_replace(source, target)

    monkeypatch.setattr(os, "replace", tracked_replace)
    try:
        document = {
            "task_id": TASK_ID,
            "version": PLAN_VERSION,
            "actions": list(canonical_single_box_plan()),
        }
        result = toolkit.execute_tool("submit_simulation_plan", {"plan": document})
        assert result.result == {
            "accepted": True,
            "action_count": 15,
            "path": str(plan_path),
        }
        assert json.loads(plan_path.read_text(encoding="utf-8")) == document
        assert len(replacements) == 1
        source, target = replacements[0]
        assert source.parent == plan_path.parent
        assert target == plan_path
        assert not source.exists()
        rejected_plan = copy.deepcopy(document)
        rejected_plan["actions"] = rejected_plan["actions"][:-1]
        rejected = toolkit.submit_simulation_plan(rejected_plan)
        assert "error" in rejected
        assert len(replacements) == 1
        original_mtime_ns = plan_path.stat().st_mtime_ns
        duplicate = toolkit.submit_simulation_plan(document)
        assert duplicate == {
            "accepted": False,
            "error": "simulation plan has already been accepted for this session",
            "error_code": "plan_already_submitted",
        }
        assert len(replacements) == 1
        assert json.loads(plan_path.read_text(encoding="utf-8")) == document
        assert plan_path.stat().st_mtime_ns == original_mtime_ns
    finally:
        toolkit.close()


def test_finish_success_requires_session_accepted_plan(tmp_path):
    toolkit = make_toolkit(tmp_path)
    try:
        blocked = toolkit.finish(
            status="success", summary="no plan has been submitted"
        )
        assert blocked == {
            "error": "finish(status='success') requires an accepted simulation plan",
            "error_code": "success_requires_accepted_plan",
        }

        invalid = {
            "task_id": TASK_ID,
            "version": PLAN_VERSION,
            "actions": canonical_single_box_plan()[:-1],
        }
        assert "error" in toolkit.submit_simulation_plan(invalid)
        assert "error" in toolkit.finish(
            status="success", summary="only validation was attempted"
        )

        plan = {
            "task_id": TASK_ID,
            "version": PLAN_VERSION,
            "actions": list(canonical_single_box_plan()),
        }
        accepted = toolkit.submit_simulation_plan(plan)
        assert accepted["accepted"] is True
        finished = toolkit.finish(
            status="success", summary="single-box plan accepted"
        )
        assert finished == {
            "_finish": True,
            "status": "success",
            "summary": "single-box plan accepted",
        }
    finally:
        toolkit.close()


def test_failure_and_stuck_finish_without_an_accepted_plan(tmp_path):
    toolkit = make_toolkit(tmp_path)
    try:
        assert toolkit.finish(status="failure", summary="task context unavailable") == {
            "_finish": True,
            "status": "failure",
            "summary": "task context unavailable",
        }
    finally:
        toolkit.close()

    toolkit = make_toolkit(tmp_path)
    try:
        assert toolkit.finish(status="stuck", summary="planning is blocked") == {
            "_finish": True,
            "status": "stuck",
            "summary": "planning is blocked",
        }
    finally:
        toolkit.close()


def test_actual_api_loop_reads_submits_and_finishes(tmp_path):
    toolkit = make_toolkit(tmp_path)
    plan_path = Path(toolkit._plan_path)
    calls = []

    def model(messages, info):
        calls.append([tool.name for tool in info.function_tools])
        if any(
            isinstance(part, ToolReturnPart)
            for message in messages
            for part in message.parts
            if getattr(part, "tool_name", "") == "finish"
        ):
            return ModelResponse(parts=[TextPart("Plan submitted.")])
        if any(
            isinstance(part, ToolReturnPart)
            for message in messages
            for part in message.parts
            if getattr(part, "tool_name", "") == "submit_simulation_plan"
        ):
            return ModelResponse(
                parts=[
                    ToolCallPart(
                        "finish",
                        {"status": "success", "summary": "single-box plan accepted"},
                        "finish-1",
                    )
                ]
            )
        if any(
            isinstance(part, ToolReturnPart)
            for message in messages
            for part in message.parts
            if getattr(part, "tool_name", "") == "read_simulation_task"
        ):
            return ModelResponse(
                parts=[
                    ToolCallPart(
                        "submit_simulation_plan",
                        {
                            "plan": {
                                "task_id": TASK_ID,
                                "version": PLAN_VERSION,
                                "actions": list(canonical_single_box_plan()),
                            }
                        },
                        "submit-1",
                    )
                ]
            )
        return ModelResponse(
            parts=[ToolCallPart("read_simulation_task", {}, "read-1")]
        )

    try:
        planner = ApiAgentLoop(
            FunctionModel(model),
            dashboard_events=NullDashboardEventSink(),
            timeout_s=5,
        )
        result = planner.solve(
            system_prompt="Submit the simulation plan.",
            user_message="Read the task, submit exactly one plan, then finish.",
            toolkit=toolkit,
            max_turns=4,
        )
        assert result.error is None
        assert result.finish_result == {
            "_finish": True,
            "status": "success",
            "summary": "single-box plan accepted",
        }
        assert result.stats["tool_calls"] == 3
        assert plan_path.exists()
    finally:
        toolkit.close()
    assert calls == [[
        "read_simulation_task",
        "submit_simulation_plan",
        "finish",
    ]] * 3


def test_real_cli_generates_the_plan_file(tmp_path, monkeypatch):
    from rpent.cli import main as cli
    import pydantic_ai.models as models

    tool_lists = []

    def model(messages, info):
        tool_lists.append([tool.name for tool in info.function_tools])
        if any(
            isinstance(part, ToolReturnPart)
            for message in messages
            for part in message.parts
            if getattr(part, "tool_name", "") == "finish"
        ):
            return ModelResponse(parts=[TextPart("Finished offline.")])
        if any(
            isinstance(part, ToolReturnPart)
            for message in messages
            for part in message.parts
            if getattr(part, "tool_name", "") == "submit_simulation_plan"
        ):
            return ModelResponse(
                parts=[
                    ToolCallPart(
                        "finish",
                        {"status": "success", "summary": "offline plan generated"},
                        "finish-cli",
                    )
                ]
            )
        if any(isinstance(part, ToolReturnPart) for message in messages for part in message.parts):
            return ModelResponse(
                parts=[
                    ToolCallPart(
                        "submit_simulation_plan",
                        {
                            "plan": {
                                "task_id": TASK_ID,
                                "version": PLAN_VERSION,
                                "actions": list(canonical_single_box_plan()),
                            }
                        },
                        "submit-cli",
                    )
                ]
            )
        return ModelResponse(
            parts=[ToolCallPart("read_simulation_task", {}, "read-cli")]
        )

    original_infer = models.infer_model

    def infer(candidate, **kwargs):
        if not isinstance(candidate, str):
            return original_infer(candidate, **kwargs)
        assert candidate == "openai:offline-test"
        return FunctionModel(model)

    monkeypatch.setattr(models, "infer_model", infer)
    monkeypatch.setattr(models, "ALLOW_MODEL_REQUESTS", False)
    monkeypatch.setattr(sys, "stdin", SimpleNamespace(isatty=lambda: True))
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "rpent",
            "--robot",
            "lynsense_simulation",
            "--planner",
            "api",
            "--memory-profile",
            "local",
            "--output-dir",
            str(tmp_path),
            "--max-turns",
            "4",
            "--model",
            "openai:offline-test",
        ],
    )

    assert cli.main() == 0
    assert tool_lists == [
        ["read_simulation_task", "submit_simulation_plan", "finish"]
    ] * 3
    plan_path = tmp_path / "plan.json"
    assert json.loads(plan_path.read_text(encoding="utf-8")) == {
        "task_id": TASK_ID,
        "version": PLAN_VERSION,
        "actions": list(canonical_single_box_plan()),
    }
    transcript_path = tmp_path / "transcript_lynsense_simulation_single_box.json"
    transcript = json.loads(transcript_path.read_text(encoding="utf-8"))
    assert transcript["stats"]["tool_calls"] == 3
    assert transcript["finish"]["status"] == "success"
