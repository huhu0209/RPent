from __future__ import annotations

import argparse
import ast
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

import pytest

from robots.lynsense_real_box.site_profile import SiteProfile
from tests.unit_tests.robots.lynsense_real_box.test_site_profile import (
    valid_profile,
)


class DashboardSink:
    pass


def add_global_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--planner", choices=["api", "claude"], default=None)
    parser.add_argument("--memory-profile", choices=["hf", "local"], default=None)
    parser.add_argument("--memory-dir")
    parser.add_argument("--output-dir")
    parser.add_argument("--dashboard", action="store_true")
    parser.add_argument("--interactive", action="store_true")
    parser.add_argument("--explore", action="store_true")


def write_profile(tmp_path: Path, *, mode: str = "dry_run") -> Path:
    source = valid_profile()
    if mode == "live":
        source["mode"] = "live"
        source["controller_review"].update(
            status="site_confirmed",
            safety_evidence=["approved-robot-one-inventory"],
            reviewer="site-reviewer",
        )
        source["calibration"]["status"] = "site_confirmed"
    path = tmp_path / "site-profile.json"
    path.write_text(json.dumps(source), encoding="utf-8")
    return path


def make_args(
    tmp_path: Path,
    *,
    planner: str = "api",
    memory_profile: str = "local",
    mode: str = "dry_run",
    profile_path: Path | None = None,
) -> argparse.Namespace:
    return argparse.Namespace(
        planner=planner,
        memory_profile=memory_profile,
        memory_dir=tmp_path / "memory",
        output_dir=tmp_path / "output",
        dashboard=False,
        interactive=False,
        explore=False,
        site_profile=profile_path or write_profile(tmp_path, mode=mode),
    )


def make_config(spec_module: Any, tmp_path: Path, **changes: Any):
    args = make_args(tmp_path, **changes)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    return spec_module._parse_config(args), args


def test_importing_robot_spec_does_not_import_forbidden_modules() -> None:
    code = (
        "import sys; import robots.lynsense_real_box.robot_spec; "
        "forbidden = ('rclpy', 'rosidl', 'lynsense_pytrees'); "
        "found = [name for name in forbidden "
        "if any(module == name or module.startswith(name + '.') "
        "for module in sys.modules)]; "
        "print(','.join(found)); "
        "raise SystemExit(bool(found))"
    )
    result = subprocess.run(
        [sys.executable, "-c", code],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, (result.stdout, result.stderr)
    assert result.stdout.strip() == ""


def test_task6_production_modules_do_not_import_sockets_directly() -> None:
    production_dir = Path("robots/lynsense_real_box")
    forbidden = {"socket", "rclpy", "lynsense_pytrees"}
    for path in production_dir.glob("*.py"):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        imports = {
            node.names[0].name.split(".")[0]
            for node in ast.walk(tree)
            if isinstance(node, (ast.Import, ast.ImportFrom))
            if isinstance(node, ast.Import)
        }
        imports |= {
            (node.module or "").split(".")[0]
            for node in ast.walk(tree)
            if isinstance(node, ast.ImportFrom)
        }
        assert not imports & forbidden, (path, imports & forbidden)


def test_robot_identity_and_prompt_contract() -> None:
    from robots.lynsense_real_box import robot_spec

    spec = robot_spec.get_robot_spec()
    assert spec.name == "lynsense_real_box"
    assert spec.is_real_robot is True
    assert spec.supports_exploration is False
    assert spec.supports_human_interactive_exploration is False
    system = spec.prompts.render("system", variables={})
    user = spec.prompts.render("user", variables={})
    assert "tools are globally visible" in system
    assert "only currently legal calls execute" in system
    assert "dry-run sends no ROS request" in system
    assert "stop_task is not an E-stop" in system
    assert system.strip() and user.strip()


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("planner", "claude"),
        ("memory_profile", "hf"),
        ("dashboard", True),
        ("interactive", True),
        ("explore", True),
    ],
)
def test_non_phase1_modes_are_rejected(tmp_path: Path, field: str, value: Any) -> None:
    from robots.lynsense_real_box import robot_spec

    args = make_args(tmp_path)
    setattr(args, field, value)
    with pytest.raises(ValueError, match="lynsense real-box requires"):
        robot_spec._parse_config(args)


def test_api_planner_with_local_memory_is_accepted(tmp_path: Path) -> None:
    from robots.lynsense_real_box import robot_spec

    config, args = make_config(robot_spec, tmp_path)
    assert config.recipe_tag == "lynsense_real_box_phase1"
    assert config.output_dir == args.output_dir
    assert config.prompt_vars == {"memory_dir": str(args.memory_dir)}


def test_site_profile_is_required_and_must_be_valid(tmp_path: Path) -> None:
    from robots.lynsense_real_box import robot_spec

    parser = argparse.ArgumentParser(exit_on_error=False)
    add_global_args(parser)
    robot_spec.get_robot_spec().add_cli_args(parser, use_dashboard=False)
    with pytest.raises(SystemExit):
        parser.parse_args([])

    from robots.lynsense_real_box.site_profile import SiteProfileError

    missing = make_args(tmp_path)
    missing.site_profile = tmp_path / "missing.json"
    with pytest.raises(SiteProfileError):
        robot_spec._parse_config(missing)

    invalid = tmp_path / "invalid.json"
    invalid.write_text("{", encoding="utf-8")
    args = make_args(tmp_path, profile_path=invalid)
    with pytest.raises(ValueError):
        robot_spec._parse_config(args)


def test_init_runtime_rejects_live_after_site_profile_validation(
    tmp_path: Path,
) -> None:
    from robots.lynsense_real_box import robot_spec

    args = make_args(tmp_path, mode="live")
    with pytest.raises(ValueError, match="rejects mode='live'"):
        robot_spec._init_runtime(args, args.output_dir, DashboardSink(), None)


def test_runtime_components_are_empty(tmp_path: Path) -> None:
    from robots.lynsense_real_box import robot_spec

    spec = robot_spec.get_robot_spec()
    assert spec.dashboard is not None
    assert spec.dashboard["runtime_components"] == ()
    args = make_args(tmp_path)
    daemons, runtime_kwargs = robot_spec._init_runtime(
        args, args.output_dir, DashboardSink(), None
    )
    assert daemons == []
    assert set(runtime_kwargs) == {"profile"}
    assert isinstance(runtime_kwargs["profile"], SiteProfile)
    with pytest.raises(ValueError, match="runtime components"):
        robot_spec._init_runtime(
            args, args.output_dir, DashboardSink(), {"unexpected"}
        )


def test_toolkit_construction_failure_closes_owned_resources(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    import robots.lynsense_real_box.dry_run_adapter as adapter_module
    import robots.lynsense_real_box.evidence as evidence_module
    import robots.lynsense_real_box.robot_spec as robot_spec
    import robots.lynsense_real_box.toolkit as toolkit_module
    from robots.lynsense_real_box.dry_run_adapter import OfflineDryRunAdapter
    from robots.lynsense_real_box.evidence import EvidenceRecorder

    class SpyAdapter(OfflineDryRunAdapter):
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            super().__init__(*args, **kwargs)
            self.close_calls = 0
            created.append(self)

        def close(self) -> None:
            self.close_calls += 1

    class SpyEvidence(EvidenceRecorder):
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            super().__init__(*args, **kwargs)
            self.close_calls = 0
            created.append(self)

        def close(self) -> None:
            self.close_calls += 1

    class FailingToolkit:
        def __init__(self, **kwargs: Any) -> None:
            raise RuntimeError("construction failed")

    created: list[Any] = []
    config, _ = make_config(robot_spec, tmp_path)
    _, runtime_kwargs = robot_spec._init_runtime(
        make_args(tmp_path), config.output_dir, DashboardSink(), None
    )
    monkeypatch.setattr(adapter_module, "OfflineDryRunAdapter", SpyAdapter)
    monkeypatch.setattr(evidence_module, "EvidenceRecorder", SpyEvidence)
    monkeypatch.setattr(toolkit_module, "LynsenseRealBoxToolkit", FailingToolkit)

    with pytest.raises(RuntimeError, match="construction failed"):
        robot_spec.get_toolkit(
            runtime_kwargs=runtime_kwargs,
            dashboard_events=DashboardSink(),
            config=config,
        )
    assert [item.close_calls for item in created] == [1, 1]


def test_connection_failure_closes_adapter_and_evidence(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    import robots.lynsense_real_box.dry_run_adapter as adapter_module
    import robots.lynsense_real_box.evidence as evidence_module
    import robots.lynsense_real_box.robot_spec as robot_spec
    from robots.lynsense_real_box.dry_run_adapter import OfflineDryRunAdapter
    from robots.lynsense_real_box.evidence import EvidenceRecorder

    class RejectedAdapter(OfflineDryRunAdapter):
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            super().__init__(*args, **kwargs)
            self.close_calls = 0
            created.append(self)

        def connect(self) -> dict[str, Any]:
            return {"status": "rejected", "reason": "test_rejected"}

        def close(self) -> None:
            self.close_calls += 1

    class SpyEvidence(EvidenceRecorder):
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            super().__init__(*args, **kwargs)
            self.close_calls = 0
            created.append(self)

        def close(self) -> None:
            self.close_calls += 1

    created: list[Any] = []
    config, _ = make_config(robot_spec, tmp_path)
    _, runtime_kwargs = robot_spec._init_runtime(
        make_args(tmp_path), config.output_dir, DashboardSink(), None
    )
    monkeypatch.setattr(adapter_module, "OfflineDryRunAdapter", RejectedAdapter)
    monkeypatch.setattr(evidence_module, "EvidenceRecorder", SpyEvidence)

    with pytest.raises(RuntimeError, match="test_rejected"):
        robot_spec.get_toolkit(
            runtime_kwargs=runtime_kwargs,
            dashboard_events=DashboardSink(),
            config=config,
        )
    assert [item.close_calls for item in created] == [1, 1]


def test_registered_dry_run_toolkit_supplies_offline_sensor_fixtures(
    tmp_path: Path,
) -> None:
    import robots.lynsense_real_box.robot_spec as robot_spec

    config, _ = make_config(robot_spec, tmp_path)
    _, runtime_kwargs = robot_spec._init_runtime(
        make_args(tmp_path), config.output_dir, DashboardSink(), None
    )
    toolkit = robot_spec.get_toolkit(
        runtime_kwargs=runtime_kwargs,
        dashboard_events=DashboardSink(),
        config=config,
    )

    try:
        task_result = toolkit.execute_tool("read_task_state", {}).result
        robot_result = toolkit.execute_tool("read_robot_state", {}).result
        detection_result = toolkit.execute_tool("detect_box", {}).result
    finally:
        toolkit.close()

    assert task_result["status"] == "ok"
    assert robot_result["status"] == "ok"
    assert robot_result["robot_state"]["healthy"] is True
    assert detection_result["status"] == "ok"
    assert detection_result["frame"] == "offline_box_fixture"


def test_normal_close_closes_adapter_and_evidence_exactly_once(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    import robots.lynsense_real_box.dry_run_adapter as adapter_module
    import robots.lynsense_real_box.evidence as evidence_module
    import robots.lynsense_real_box.robot_spec as robot_spec
    from robots.lynsense_real_box.dry_run_adapter import OfflineDryRunAdapter
    from robots.lynsense_real_box.evidence import EvidenceRecorder

    class SpyAdapter(OfflineDryRunAdapter):
        def close(self) -> None:
            self.close_calls = getattr(self, "close_calls", 0) + 1

    class SpyEvidence(EvidenceRecorder):
        def close(self) -> None:
            self.close_calls = getattr(self, "close_calls", 0) + 1

    monkeypatch.setattr(adapter_module, "OfflineDryRunAdapter", SpyAdapter)
    monkeypatch.setattr(evidence_module, "EvidenceRecorder", SpyEvidence)
    config, _ = make_config(robot_spec, tmp_path)
    _, runtime_kwargs = robot_spec._init_runtime(
        make_args(tmp_path), config.output_dir, DashboardSink(), None
    )
    toolkit = robot_spec.get_toolkit(
        runtime_kwargs=runtime_kwargs,
        dashboard_events=DashboardSink(),
        config=config,
    )
    toolkit.close()
    toolkit.close()
    assert toolkit._adapter.close_calls == 1
    assert toolkit._evidence.close_calls == 1
