"""Robot One owned read-only backend registration."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
from pathlib import Path
from typing import TYPE_CHECKING, Any

from rpent.dashboard.spec import DashboardSpec
from rpent.robots.prompt_bundle import PromptBundle
from rpent.robots.robot_spec import RobotSpec, RunConfig
from rpent.utils.config import get_memory_dir, get_repo_root

if TYPE_CHECKING:
    from rpent.dashboard.events import DashboardEventSink
    from rpent.utils.daemon import ProcessDaemon


DASHBOARD_SPEC: DashboardSpec = {
    "task": {
        "command": "/rpent-task",
        "usage": "/rpent-task",
        "fields": (),
        "display": "Robot One read-only state",
        "output_slug": "robot_one_readonly",
    },
    "runtime_components": (),
    "frame_channels": (),
    "primitives": ("read_robot_state",),
}


def _system_prompt(variables=None) -> str:
    return (
        "You observe Robot One through a read-only owned LynrotControl runtime. "
        "Your only tool is read_robot_state. It takes no arguments and cannot "
        "move arms, grippers, waist, chassis, or perception. Call it before "
        "reporting state. Report readable state and limitations; do not infer "
        "safety or motion readiness. There is no finish tool and no motion "
        "capability."
    )


def _user_prompt(variables=None) -> str:
    return "Call read_robot_state once and summarize Robot One's current state."


def get_robot_spec() -> RobotSpec:
    return RobotSpec(
        name="robot_one_readonly",
        prompts=PromptBundle(system=_system_prompt, user=_user_prompt),
        add_cli_args=_add_cli_args,
        parse_config=_parse_config,
        init_runtime=_init_runtime,
        dashboard=DASHBOARD_SPEC,
        is_real_robot=True,
        supports_dashboard=True,
        supports_exploration=False,
    )


def _add_cli_args(parser: argparse.ArgumentParser, use_dashboard: bool) -> None:
    return None


def _parse_config(args: argparse.Namespace) -> RunConfig:
    if getattr(args, "planner", None) != "api":
        raise ValueError("Robot One read-only requires --planner api")
    if getattr(args, "memory_profile", None) != "local":
        raise ValueError("Robot One read-only requires --memory-profile local")
    if any(getattr(args, mode, False) for mode in ("interactive", "explore")):
        raise ValueError(
            "Robot One read-only does not support interactive or exploration"
        )
    requested_output = getattr(args, "output_dir", None)
    output_dir = (
        Path(requested_output).expanduser().resolve()
        if requested_output
        else get_repo_root()
        / "logs"
        / (
            "robot_one_readonly_"
            + datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%f")
        )
    )
    memory_dir = getattr(args, "memory_dir", None)
    return RunConfig(
        recipe_tag="robot_one_readonly",
        output_dir=output_dir,
        prompt_vars={"memory_dir": str(memory_dir)}
        if memory_dir
        else {},
        task_desc={"operation": "read_robot_state", "runtime": "owned"},
    )


def _init_runtime(
    args: argparse.Namespace,
    output_dir: Path,
    dashboard_events: DashboardEventSink,
    components: set[str] | None,
) -> tuple[list[ProcessDaemon], dict[str, Any]]:
    if components not in (None, set()):
        raise ValueError(
            f"Robot One read-only has no runtime components: {sorted(components)}"
        )
    from robots.robot_one_readonly.runtime import OwnedRobotOneStateReader

    return [], {"reader": OwnedRobotOneStateReader()}


def get_toolkit(
    *,
    runtime_kwargs: dict[str, Any],
    dashboard_events: DashboardEventSink,
    config: RunConfig,
) -> Any:
    from robots.robot_one_readonly.toolkit import RobotOneReadOnlyToolkit

    reader = runtime_kwargs.pop("reader")
    try:
        connection = reader.connect()
        if connection.get("status") != "ok":
            raise RuntimeError(connection.get("reason", "connection rejected"))
        return RobotOneReadOnlyToolkit(
            reader=reader,
            dashboard_events=dashboard_events,
            memory=_memory(config),
        )
    except BaseException:
        reader.close()
        raise


def _memory(config: RunConfig) -> Any:
    from rpent.memory import MemoryManager

    return MemoryManager(
        config.prompt_vars.get("memory_dir")
        or get_memory_dir("robot_one_readonly")
    )
