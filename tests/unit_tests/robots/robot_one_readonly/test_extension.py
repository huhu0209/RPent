"""Offline contracts for the Robot One read-only backend."""

from __future__ import annotations

import argparse
import hashlib
from typing import Any

import pytest

from robots.robot_one_readonly import get_robot_spec, get_toolkit
from robots.robot_one_readonly import runtime as runtime_module
from rpent.dashboard.events import NullDashboardEventSink
from rpent.dashboard.state import DashboardState
from rpent.planner.api_loop import _build_tools
from rpent.robots import enumerate_robots


@pytest.fixture(autouse=True)
def output_directory(tmp_path, monkeypatch):
    from rpent.utils import logging

    monkeypatch.setattr(logging, "_output_dir", tmp_path)


class FakeReader:
    def __init__(self) -> None:
        self.connected = False
        self.closed = False

    def connect(self) -> dict[str, Any]:
        self.connected = True
        return {"status": "ok"}

    def read(self) -> dict[str, Any]:
        assert self.connected and not self.closed
        return {"status": "ok", "state": {"left_arm": {"sequence": 1}}}

    def close(self) -> None:
        self.closed = True


def args_for(tmp_path, **changes):
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


def make_toolkit(tmp_path, reader=None):
    spec = get_robot_spec()
    args = args_for(tmp_path)
    config = spec.parse_config(args)
    reader = reader or FakeReader()
    toolkit = get_toolkit(
        runtime_kwargs={"reader": reader},
        dashboard_events=NullDashboardEventSink(),
        config=config,
    )
    return toolkit, reader


def test_registration_is_exact_read_only_real_robot() -> None:
    assert "robot_one_readonly" in enumerate_robots()
    spec = get_robot_spec()
    assert spec.is_real_robot is True
    assert spec.supports_dashboard is True
    assert spec.dashboard is not None
    assert spec.dashboard["primitives"] == ("read_robot_state",)


def test_exact_tool_table_reads_and_releases(tmp_path) -> None:
    toolkit, reader = make_toolkit(tmp_path)
    try:
        specs = toolkit.get_tools_spec()
        assert [item["name"] for item in specs] == ["read_robot_state"]
        assert [tool.name for tool in _build_tools(toolkit)] == ["read_robot_state"]
        result = toolkit.execute_tool("read_robot_state", {}).result
        assert result["status"] == "ok"
        assert "error" in toolkit.execute_tool("move_dual_arms", {}).result
    finally:
        toolkit.close()
    assert reader.closed is True


@pytest.mark.parametrize(
    "change",
    [
        {"planner": "codex"},
        {"memory_profile": "hf"},
        {"interactive": True},
        {"explore": True},
    ],
)
def test_rejects_non_readonly_run_modes(tmp_path, change) -> None:
    with pytest.raises(ValueError):
        get_robot_spec().parse_config(args_for(tmp_path, **change))


def test_factory_connect_failure_closes_reader(tmp_path) -> None:
    class FailedReader(FakeReader):
        def connect(self):
            self.connected = True
            return {"status": "rejected", "reason": "blocked"}

    reader = FailedReader()
    with pytest.raises(RuntimeError, match="blocked"):
        make_toolkit(tmp_path, reader)
    assert reader.closed is True


def test_dashboard_primitive_executes_only_read_robot_state(tmp_path) -> None:
    toolkit, reader = make_toolkit(tmp_path)
    state = DashboardState(output_dir=tmp_path, dashboard_spec=get_robot_spec().dashboard)
    state.shared_services_ready()
    state.request_task({})
    assert state.wait_for_task(timeout=0.1) is not None
    state.begin_planner_session(video_path=None)
    state.bind_toolkit(toolkit)
    state.set_planner_activity("idle", accepting_input=True)
    try:
        result = state.execute_primitive("read_robot_state", {})
        assert result.result["status"] == "ok"
        with pytest.raises(ValueError, match="primitive is not allowed"):
            state.execute_primitive("move_dual_arms", {})
    finally:
        toolkit.close()
    assert reader.closed is True


def test_runtime_preflight_fails_closed_before_dependency_import(
    tmp_path,
    monkeypatch,
) -> None:
    config = tmp_path / "ea200.yaml"
    config.write_bytes(b"robot-one-config")
    digest = hashlib.sha256(config.read_bytes()).hexdigest()
    monkeypatch.setattr(runtime_module, "CONFIG_PATH", config)
    monkeypatch.setattr(runtime_module, "CONFIG_SHA256", "0" * 64)
    monkeypatch.setattr(runtime_module.platform, "node", lambda: "rpp-PC")
    monkeypatch.setenv("ROS_DOMAIN_ID", "3")

    reader = runtime_module.OwnedRobotOneStateReader()
    with pytest.raises(runtime_module.RobotOneReadOnlyError, match="config digest"):
        reader.connect()
    reader.close()

    monkeypatch.setattr(runtime_module.platform, "node", lambda: "other-host")
    reader = runtime_module.OwnedRobotOneStateReader()
    with pytest.raises(runtime_module.RobotOneReadOnlyError, match="execution host"):
        reader.connect()

    monkeypatch.setattr(runtime_module.platform, "node", lambda: "rpp-PC")
    monkeypatch.setenv("ROS_DOMAIN_ID", "4")
    with pytest.raises(runtime_module.RobotOneReadOnlyError, match="ROS domain"):
        reader.connect()
    with pytest.raises(runtime_module.RobotOneReadOnlyError, match="not connected"):
        reader.read()
    reader.close()
