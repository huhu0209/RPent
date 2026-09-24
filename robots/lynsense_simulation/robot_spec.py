"""RPent native backend for producing the isolated Webots single-box plan."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import TYPE_CHECKING, Any

from robots.lynsense.simulation.rpent_plan import TASK_ID, task_description
from rpent.dashboard.events import DashboardEventSink
from rpent.dashboard.spec import DashboardSpec
from rpent.memory import MemoryManager
from rpent.robots.prompt_bundle import PromptBundle
from rpent.robots.robot_spec import RobotSpec, RunConfig
from rpent.utils.config import get_memory_dir, get_repo_root

if TYPE_CHECKING:
    from robots.lynsense_simulation.toolkit import LynsenseSimulationToolkit


LYNSENSE_SIMULATION_DASHBOARD_SPEC: DashboardSpec = {
    "task": {
        "command": "/rpent-task",
        "usage": "/rpent-task",
        "fields": ({"name": "task_id", "label": "Task"},),
        "display": "Lynsense simulation: {task_id}",
        "output_slug": "lynsense_simulation",
    },
    "runtime_components": (),
    "frame_channels": (),
    "primitives": (),
}


def _system_prompt(variables=None) -> str:
    return (
        "You plan one isolated Webots single-box simulation task. Your tools "
        "are exactly read_simulation_task, submit_simulation_plan, and finish. "
        "Call read_simulation_task first, then submit one JSON plan that "
        "matches the published task contract. submit_simulation_plan returns "
        "structured validation feedback; on success, call finish with status "
        "success. Never invent actions, request perception, or connect to a "
        "real robot. This run writes only the simulation plan; it does not "
        "execute robot motion."
    )


def _user_prompt(variables=None) -> str:
    return (
        "Read the simulation task, submit exactly one accepted "
        "single-box plan, and finish with success."
    )


def get_robot_spec() -> RobotSpec:
    return RobotSpec(
        name="lynsense_simulation",
        prompts=PromptBundle(system=_system_prompt, user=_user_prompt),
        add_cli_args=_add_cli_args,
        parse_config=_parse_config,
        init_runtime=_init_runtime,
        dashboard=LYNSENSE_SIMULATION_DASHBOARD_SPEC,
        is_real_robot=False,
        supports_exploration=False,
    )


def _add_cli_args(parser: argparse.ArgumentParser, use_dashboard: bool) -> None:
    parser.add_argument("--plan-filename", default="plan.json")


def _parse_config(args: argparse.Namespace) -> RunConfig:
    if getattr(args, "planner", None) != "api":
        raise ValueError("lynsense simulation requires --planner api")
    if getattr(args, "memory_profile", None) != "local":
        raise ValueError("lynsense simulation requires --memory-profile local")
    if any(
        getattr(args, mode, False)
        for mode in ("dashboard", "interactive", "explore")
    ):
        raise ValueError(
            "lynsense simulation does not support dashboard, interactive, "
            "or exploration"
        )
    requested_output = getattr(args, "output_dir", None)
    output_dir = (
        Path(requested_output).expanduser().resolve()
        if requested_output
        else get_repo_root()
        / "logs"
        / "lynsense_simulation_single_box"
    )
    memory_dir = getattr(args, "memory_dir", None)
    prompt_vars: dict[str, Any] = {
        "plan_path": str(output_dir / getattr(args, "plan_filename", "plan.json"))
    }
    if memory_dir:
        prompt_vars["memory_dir"] = str(Path(memory_dir).expanduser().resolve())
    return RunConfig(
        recipe_tag="lynsense_simulation_single_box",
        output_dir=output_dir,
        prompt_vars=prompt_vars,
        task_desc={"task_id": TASK_ID, "environment": "webots-isolated"},
    )


def _init_runtime(
    args: argparse.Namespace,
    output_dir: Path,
    dashboard_events: DashboardEventSink,
    components: set[str] | None,
) -> tuple[list[Any], dict[str, Any]]:
    selected = set() if components is None else components
    if selected:
        raise ValueError(
            f"unsupported lynsense_simulation runtime components: {sorted(selected)}"
        )
    return [], {}


def get_toolkit(
    *,
    runtime_kwargs: dict[str, Any],
    dashboard_events: DashboardEventSink,
    config: RunConfig,
) -> LynsenseSimulationToolkit:
    """Reject stale runtime resources and build the plan-writing toolkit."""
    if runtime_kwargs:
        raise ValueError(f"unsupported runtime resources: {sorted(runtime_kwargs)}")
    from robots.lynsense_simulation.toolkit import LynsenseSimulationToolkit

    memory_dir = config.prompt_vars.get("memory_dir") or get_memory_dir(
        "lynsense_simulation"
    )
    return LynsenseSimulationToolkit(
        dashboard_events=dashboard_events,
        memory=MemoryManager(memory_dir),
        plan_path=Path(config.prompt_vars["plan_path"]),
    )
