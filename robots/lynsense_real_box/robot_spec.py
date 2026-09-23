"""Phase 1 real-box extension and strict dry-run registration gates."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
from pathlib import Path
from typing import TYPE_CHECKING, Any

from robots.lynsense_real_box.site_profile import (
    SiteProfile,
    load_site_profile,
)
from rpent.dashboard.spec import DashboardSpec
from rpent.robots.prompt_bundle import PromptBundle
from rpent.robots.robot_spec import RobotSpec, RunConfig
from rpent.utils.config import get_repo_root

if TYPE_CHECKING:
    from rpent.dashboard.events import DashboardEventSink
    from rpent.utils.daemon import ProcessDaemon


REAL_BOX_DASHBOARD_SPEC: DashboardSpec = {
    "task": {
        "command": "/rpent-task",
        "usage": "/rpent-task",
        "fields": (),
        "display": "Lynsense real-box dry run",
        "output_slug": "lynsense_real_box_phase1",
    },
    "runtime_components": (),
    "frame_channels": (),
    "primitives": (),
}


def _system_prompt(variables=None) -> str:
    return (
        "You operate the Lynsense real box workflow through exactly the provided "
        "tools. The tools are globally visible, but only currently legal calls "
        "execute. This Phase 1 dry-run sends no ROS request. stop_task is a "
        "best-effort software stop; stop_task is not an E-stop. A physical "
        "operator remains responsible for the E-stop and the work envelope. "
        "Do not claim that a rejected call moved hardware. Finish only when the "
        "task state says it is safe."
    )


def _user_prompt(variables=None) -> str:
    return (
        "Call read_task_state first, then execute only the next legal real-box "
        "workflow step."
    )


def get_robot_spec() -> RobotSpec:
    return RobotSpec(
        name="lynsense_real_box",
        prompts=PromptBundle(system=_system_prompt, user=_user_prompt),
        add_cli_args=_add_cli_args,
        parse_config=_parse_config,
        init_runtime=_init_runtime,
        dashboard=REAL_BOX_DASHBOARD_SPEC,
        is_real_robot=True,
        supports_exploration=False,
    )


def _add_cli_args(parser: argparse.ArgumentParser, use_dashboard: bool) -> None:
    parser.add_argument(
        "--site-profile",
        type=Path,
        required=True,
        help="Path to the validated Lynsense real-box site-profile JSON.",
    )


def _parse_config(args: argparse.Namespace) -> RunConfig:
    if (
        getattr(args, "planner", None) != "api"
        or getattr(args, "memory_profile", None) != "local"
        or any(
            getattr(args, mode, False)
            for mode in ("dashboard", "interactive", "explore")
        )
    ):
        raise ValueError(
            "lynsense real-box requires API planner, local memory, and an "
            "ordinary TTY"
        )

    profile = load_site_profile(Path(args.site_profile))
    requested_output = getattr(args, "output_dir", None)
    output_dir = (
        Path(requested_output).expanduser().resolve()
        if requested_output
        else get_repo_root()
        / "logs"
        / (
            "lynsense_real_box_"
            + datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%f")
        )
    )
    memory_dir = getattr(args, "memory_dir", None)
    return RunConfig(
        recipe_tag="lynsense_real_box_phase1",
        output_dir=output_dir,
        prompt_vars={
            "memory_dir": str(Path(memory_dir).expanduser().resolve())
        }
        if memory_dir
        else {},
        task_desc={
            "operation": "single_box_dry_run",
            "profile_id": profile.profile_id,
        },
    )


def _init_runtime(
    args: argparse.Namespace,
    output_dir: Path,
    dashboard_events: DashboardEventSink,
    components: set[str] | None,
) -> tuple[list[ProcessDaemon], dict[str, Any]]:
    if components not in (None, set()):
        raise ValueError(
            f"lynsense real-box has no runtime components: {sorted(components)}"
        )
    profile = load_site_profile(Path(args.site_profile))
    if profile.mode != "dry_run":
        raise ValueError(
            "Phase 1 lynsense real-box rejects mode='live'; only mode='dry_run' "
            "is enabled"
        )
    return [], {"profile": profile}


def _offline_fixtures(profile: SiteProfile) -> dict[str, Any]:
    """Build deterministic sensor responses for the offline-only factory."""

    target = profile.placement.target_pose
    return {
        "robot_state": {
            "age_s": 0.1,
            "healthy": True,
            "mode": "dry_run",
            "left_arm_error": False,
            "right_arm_error": False,
        },
        "box_pose": {
            "age_s": 0.2,
            "frame": profile.perception.frame,
            "transform_valid": True,
            "pose": {
                "x": 1.0,
                "y": -2.0,
                "z": 0.1,
                "yaw_rad": 0.0,
            },
        },
        "placement_result": {
            "pose": {
                "x": target.x,
                "y": target.y,
                "z": target.z,
                "yaw_rad": target.yaw_rad,
            },
            "left_gripper": {"state": "open", "position_error": 0.01},
            "right_gripper": {"state": "open", "position_error": 0.01},
            "release_evidence": {
                "kind": profile.release_evidence.kind,
                "left_signal": profile.release_evidence.left_signal,
                "right_signal": profile.release_evidence.right_signal,
            },
            "settle_elapsed_s": profile.placement.settle_s + 0.1,
            "max_observed_speed_m_s": 0.0,
        },
    }


def get_toolkit(
    *,
    runtime_kwargs: dict[str, Any],
    dashboard_events: DashboardEventSink,
    config: RunConfig,
) -> Any:
    """Build guarded resources and release both on any startup failure."""
    from robots.lynsense_real_box.dry_run_adapter import OfflineDryRunAdapter
    from robots.lynsense_real_box.evidence import EvidenceRecorder
    from robots.lynsense_real_box.task_state import TaskStateMachine
    from robots.lynsense_real_box.toolkit import LynsenseRealBoxToolkit

    profile = runtime_kwargs.get("profile")
    if not isinstance(profile, SiteProfile):
        raise ValueError("runtime_kwargs must contain a validated SiteProfile")

    adapter = OfflineDryRunAdapter(profile, **_offline_fixtures(profile))
    evidence: EvidenceRecorder | None = None
    try:
        evidence = EvidenceRecorder(
            config.output_dir / "events-lynsense_real_box.jsonl",
            profile,
        )
        state_machine = TaskStateMachine(
            move_transitions={
                item.name: item for item in profile.move_transitions
            }
        )
        memory = _memory(config)
        toolkit = LynsenseRealBoxToolkit(
            adapter=adapter,
            state_machine=state_machine,
            evidence=evidence,
            dashboard_events=dashboard_events,
            memory=memory,
            profile=profile,
        )
        connection = adapter.connect()
        if connection["status"] != "ok":
            raise RuntimeError(connection["reason"])
        return toolkit
    except BaseException:
        adapter.close()
        if evidence is not None:
            evidence.close()
        raise


def _memory(config: RunConfig) -> Any:
    from rpent.memory import MemoryManager
    from rpent.utils.config import get_memory_dir

    return MemoryManager(
        config.prompt_vars.get("memory_dir") or get_memory_dir("lynsense_real_box")
    )
