# Copyright 2026 The RPent Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import annotations

import json
import sys
import types
from pathlib import Path
from types import SimpleNamespace

import imageio.v3 as imageio
import numpy as np
import pytest

from robots.lynsense.isaac import probe
from robots.lynsense.isaac.action_contract import (
    DriveConfiguration,
    DriveGains,
    DriveLimits,
    JointActionPlan,
    JointSample,
    TransformSample,
)
from robots.lynsense.isaac.artifacts import load_json_object
from robots.lynsense.isaac.contracts import IsaacEnvironment, VisualReview
from robots.lynsense.isaac.import_contract import ImporterEvent
from robots.lynsense.isaac.runtime import IsaacRuntimeResult
from robots.lynsense.isaac.urdf_contract import (
    REQUIRED_GRIPPER_JOINT_NAMES,
    REQUIRED_GRIPPER_LINK_NAMES,
    REQUIRED_JOINT_NAMES,
    REQUIRED_LINK_NAMES,
)


def _write_screenshot(path: Path, *, blank: bool = False) -> None:
    if blank:
        pixels = np.zeros((180, 320, 3), dtype=np.uint8)
    else:
        row = np.tile(np.arange(320, dtype=np.uint8), (180, 1))
        pixels = np.stack((row, np.roll(row, 3), np.roll(row, 6)), axis=2)
    imageio.imwrite(path, pixels)


def _valid_result() -> IsaacRuntimeResult:
    action_plans = {
        JointActionPlan(
            joint_name=name,
            initial_position=0.0,
            target_position=0.1,
            lower_limit=-1.0,
            upper_limit=1.0,
        )
        for name in ("left_joint1", "right_joint1")
    }
    samples = (
        JointSample(0.0, 0.0, 0.0),
        JointSample(0.2, 0.05, 0.2),
        JointSample(0.6, 0.1, 0.01),
        JointSample(1.0, 0.1, 0.01),
    )
    return IsaacRuntimeResult(
        environment=IsaacEnvironment(
            executable=Path("/opt/isaac/python.sh"),
            semantic_version="2023.1.1",
            kit_version="106.0.0",
            python_version="3.10.14",
            renderer="RTX",
            run_mode="desktop",
            capabilities=frozenset(
                {
                    "urdf_importer",
                    "articulation",
                    "viewport_screenshot",
                    "physx_joint_limits",
                }
            ),
        ),
        importer_events=(),
        articulation_joint_names=REQUIRED_JOINT_NAMES,
        loaded_joint_names=(
            *REQUIRED_JOINT_NAMES,
            *REQUIRED_GRIPPER_JOINT_NAMES,
        ),
        loaded_link_names=(
            *REQUIRED_LINK_NAMES,
            *REQUIRED_GRIPPER_LINK_NAMES,
        ),
        action_plans=action_plans,
        drive_configuration=DriveConfiguration(
            gains=DriveGains(stiffness=200.0, damping=16.6),
            limits=DriveLimits(
                max_effort_nm=200.0,
                max_velocity_rad_s=0.314,
            ),
            verified=True,
        ),
        trajectories={"left_joint1": samples, "right_joint1": samples},
        base_motion=(
            TransformSample(0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            TransformSample(1.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        ),
    )


def _complete_child_summary() -> dict[str, object]:
    result = _valid_result()
    return {
        "outcome": "passed",
        "is_pass": True,
        "max_uncompressed_bytes": 2 * 1024**3,
        "generated_usd": ".artifacts/run/robot_1.usd",
        "run_directory": ".artifacts/run",
        "render_metrics": {"all_nonblank": True},
        **probe._runtime_evidence(result),
    }


class FakeRuntime:
    def __init__(
        self,
        *,
        result: IsaacRuntimeResult | None = None,
        error: Exception | None = None,
        blank_render: bool = False,
        run_directory: Path | None = None,
    ) -> None:
        self.result = result or _valid_result()
        self.error = error
        self.blank_render = blank_render
        self.run_directory = run_directory

    def run(self, paths: object) -> IsaacRuntimeResult:
        assert self.run_directory is None or paths.run_directory == self.run_directory
        if self.error is not None:
            raise self.error
        for path in paths.screenshots:
            _write_screenshot(path, blank=self.blank_render)
        return self.result


def test_parser_requires_explicit_isaac_python_and_rejects_environments() -> None:
    parser = probe.build_parser()
    args = parser.parse_args([])
    assert args.isaac_python is None
    assert args.mode == "host"

    args = parser.parse_args(["--isaac-python", "/opt/isaac/python.sh"])
    assert args.mode == "host"
    assert args.archive == Path("../URDF/URDF_robot_1.tar.gz").resolve()
    assert probe.DEFAULT_ARTIFACT_ROOT == Path(".artifacts/lynsense-isaac").resolve()
    assert args.visual_review_status in {"accepted", "rejected"}
    assert args.max_uncompressed_bytes == 2 * 1024**3


def test_parser_rejects_an_extraction_limit_above_the_hard_ceiling() -> None:
    parser = probe.build_parser()
    with pytest.raises(SystemExit):
        parser.parse_args(["--max-uncompressed-bytes", str(2 * 1024**3 + 1)])


def test_host_probe_invalid_archive_fails(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    archive = tmp_path / "robot.tar.gz"
    archive.write_bytes(b"not-a-tar")
    monkeypatch.setattr(probe, "DEFAULT_ARCHIVE", archive)
    monkeypatch.setattr(probe, "DEFAULT_ARTIFACT_ROOT", tmp_path / "runs")
    assert probe.execute_host_probe([]) == 1
    summary = json.loads(
        next((tmp_path / "runs").glob("*/technical-summary.json")).read_text()
    )
    assert summary["outcome"] == "failed"


def test_host_probe_without_explicit_runtime_is_explicit_skip(
    valid_probe_archive: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    artifact_root = tmp_path / "runs"
    monkeypatch.setattr(probe, "DEFAULT_ARCHIVE", valid_probe_archive)
    monkeypatch.setattr(probe, "DEFAULT_ARTIFACT_ROOT", artifact_root)

    assert probe.execute_host_probe([]) == probe.EXPLICIT_SKIP_EXIT_CODE
    run_directory = next(artifact_root.iterdir())
    summary = json.loads(
        (run_directory / "technical-summary.json").read_text(encoding="utf-8")
    )
    assert summary["outcome"] == "skipped"
    assert summary["is_pass"] is False
    assert "No explicit Isaac Sim Python executable was supplied" in summary["reason"]
    assert (run_directory / "urdf-inventory.json").is_file()
    assert not (run_directory / "environment.json").exists()
    assert not (run_directory / "robot_1.usd").exists()
    assert not any(path.suffix == ".png" for path in run_directory.iterdir())


def test_host_probe_propagates_explicit_isaac_python_to_child(
    valid_probe_archive: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    artifact_root = tmp_path / "runs"
    monkeypatch.setattr(probe, "DEFAULT_ARCHIVE", valid_probe_archive)
    monkeypatch.setattr(probe, "DEFAULT_ARTIFACT_ROOT", artifact_root)
    captured: dict[str, object] = {}

    def fake_run(command, **kwargs):
        captured["command"] = command
        run_directory = Path(command[command.index("--run-directory") + 1])
        probe.atomic_write_json(
            run_directory / "technical-summary.json",
            {"outcome": "failed", "is_pass": False},
        )
        return SimpleNamespace(returncode=1)

    monkeypatch.setattr(probe.subprocess, "run", fake_run)
    assert probe.execute_host_probe(["--isaac-python", "/opt/isaac/python.sh"]) == 1
    command = captured["command"]
    assert isinstance(command, list)
    assert command[0] == "/opt/isaac/python.sh"
    assert command[command.index("--isaac-python") + 1] == "/opt/isaac/python.sh"


@pytest.mark.parametrize(
    ("summary", "return_code", "expected"),
    [
        (_complete_child_summary(), 0, 0),
        ({"outcome": "passed", "is_pass": True}, 1, 1),
        ({"outcome": "passed", "is_pass": False}, 0, 1),
        ({"outcome": "failed", "is_pass": False}, 0, 1),
        ({"outcome": "skipped", "is_pass": False}, 0, 1),
        ({"outcome": "unknown", "is_pass": True}, 0, 1),
        (None, 0, 1),
    ],
)
def test_host_probe_requires_passed_child_summary(
    valid_probe_archive: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    summary: dict[str, object] | None,
    return_code: int,
    expected: int,
) -> None:
    artifact_root = tmp_path / "runs"
    monkeypatch.setattr(probe, "DEFAULT_ARCHIVE", valid_probe_archive)
    monkeypatch.setattr(probe, "DEFAULT_ARTIFACT_ROOT", artifact_root)

    def fake_run(command, **kwargs):
        if summary is not None:
            run_directory = Path(command[command.index("--run-directory") + 1])
            probe.atomic_write_json(run_directory / "technical-summary.json", summary)
        return SimpleNamespace(returncode=return_code)

    monkeypatch.setattr(probe.subprocess, "run", fake_run)
    assert (
        probe.execute_host_probe(["--isaac-python", "/opt/isaac/python.sh"]) == expected
    )
    if expected:
        stored = load_json_object(next(artifact_root.glob("*/technical-summary.json")))
        assert stored["stage"] == "child"


def test_host_probe_nonzero_child_overwrites_existing_summary(
    valid_probe_archive: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    artifact_root = tmp_path / "runs"
    monkeypatch.setattr(probe, "DEFAULT_ARCHIVE", valid_probe_archive)
    monkeypatch.setattr(probe, "DEFAULT_ARTIFACT_ROOT", artifact_root)

    child_summary = _complete_child_summary()
    child_summary["importer_events"] = [
        {
            "category": "material_change",
            "severity": "warning",
            "source": "isaac-log",
            "message": "material changed",
            "decision": "recorded",
        }
    ]

    def fake_run(command, **kwargs):
        run_directory = Path(command[command.index("--run-directory") + 1])
        probe.atomic_write_json(run_directory / "technical-summary.json", child_summary)
        return SimpleNamespace(returncode=3)

    monkeypatch.setattr(probe.subprocess, "run", fake_run)
    assert probe.execute_host_probe(["--isaac-python", "/opt/isaac/python.sh"]) == 1
    summary = load_json_object(next(artifact_root.glob("*/technical-summary.json")))
    assert summary["outcome"] == "failed"
    assert summary["is_pass"] is False
    assert summary["stage"] == "child"
    assert summary["environment"] == child_summary["environment"]
    assert summary["importer_events"] == child_summary["importer_events"]


def test_host_probe_rejects_minimal_passing_child_summary_and_preserves_evidence(
    valid_probe_archive: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    artifact_root = tmp_path / "runs"
    monkeypatch.setattr(probe, "DEFAULT_ARCHIVE", valid_probe_archive)
    monkeypatch.setattr(probe, "DEFAULT_ARTIFACT_ROOT", artifact_root)
    child_summary = {
        "outcome": "passed",
        "is_pass": True,
        "importer_events": [
            {
                "category": "material_change",
                "severity": "warning",
                "source": "isaac-log",
                "message": "material changed",
                "decision": "recorded",
            }
        ],
    }

    def fake_run(command, **kwargs):
        run_directory = Path(command[command.index("--run-directory") + 1])
        probe.atomic_write_json(run_directory / "technical-summary.json", child_summary)
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(probe.subprocess, "run", fake_run)
    assert probe.execute_host_probe(["--isaac-python", "/opt/isaac/python.sh"]) == 1
    summary = load_json_object(next(artifact_root.glob("*/technical-summary.json")))
    assert summary["stage"] == "child"
    assert summary["outcome"] == "failed"
    assert summary["is_pass"] is False
    assert summary["importer_events"] == child_summary["importer_events"]


@pytest.mark.parametrize("launch_error", [OSError("no executable"), None])
def test_host_probe_records_launch_failure_summary(
    valid_probe_archive: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    launch_error: OSError | None,
) -> None:
    artifact_root = tmp_path / "runs"
    monkeypatch.setattr(probe, "DEFAULT_ARCHIVE", valid_probe_archive)
    monkeypatch.setattr(probe, "DEFAULT_ARTIFACT_ROOT", artifact_root)

    def fake_run(command, **kwargs):
        if launch_error is not None:
            raise launch_error
        return SimpleNamespace(returncode=1)

    monkeypatch.setattr(probe.subprocess, "run", fake_run)
    assert probe.execute_host_probe(["--isaac-python", "/opt/isaac/python.sh"]) == 1
    summary = load_json_object(next(artifact_root.glob("*/technical-summary.json")))
    assert summary["outcome"] == "failed"
    assert summary["stage"] == ("launch" if launch_error is not None else "child")


def test_real_isaac_probe_requires_explicit_executable(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    artifact_root = tmp_path / "runs"
    monkeypatch.setattr(probe, "DEFAULT_ARTIFACT_ROOT", artifact_root)
    run_directory = _prepared_run(artifact_root)
    assert (
        probe.execute_isaac_probe(
            ["--mode", "isaac", "--run-directory", str(run_directory)]
        )
        == 1
    )
    summary = load_json_object(run_directory / "technical-summary.json")
    assert summary["stage"] == "validation"
    assert "--isaac-python" in summary["reason"]


def test_observed_environment_records_exact_executable(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from robots.lynsense.isaac import runtime

    omni = types.ModuleType("omni")
    omni.__path__ = []
    kit = types.ModuleType("omni.kit")
    kit.__path__ = []
    app_module = types.ModuleType("omni.kit.app")
    usd_module = types.ModuleType("omni.usd")
    pxr = types.ModuleType("pxr")
    pxr.__path__ = []
    physx_module = types.ModuleType("pxr.PhysxSchema")
    carb = types.ModuleType("carb")

    class App:
        def get_app_version(self):
            return "2023.1.1"

        def get_kit_version(self):
            return "106.0.0"

    class Settings:
        def get(self, key):
            assert key == "/rtx/rendermode"
            return "RTX"

    app_module.get_app = lambda: App()
    usd_module.get_context = lambda: SimpleNamespace(get_stage=lambda: object())
    physx_module.PhysxJointAPI = object
    carb.settings = SimpleNamespace(get_settings=lambda: Settings())
    omni.kit = kit
    omni.usd = usd_module
    kit.app = app_module
    pxr.PhysxSchema = physx_module
    for name, module in {
        "omni": omni,
        "omni.kit": kit,
        "omni.kit.app": app_module,
        "omni.usd": usd_module,
        "pxr": pxr,
        "pxr.PhysxSchema": physx_module,
        "carb": carb,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)

    executable = Path("/opt/isaac/python.sh")
    assert runtime._observed_environment(executable).executable == executable


def test_isaac_probe_passes_fake_runtime(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(probe, "DEFAULT_ARTIFACT_ROOT", tmp_path)
    run_directory = tmp_path / "run"
    run_directory.mkdir()
    (run_directory / "urdf-inventory.json").write_text("{}", encoding="utf-8")
    (run_directory / "extracted" / "URDF_robot_1").mkdir(parents=True)
    (run_directory / "extracted" / "URDF_robot_1" / "robot.urdf").write_text(
        "<robot/>", encoding="utf-8"
    )

    return_code = probe.execute_isaac_probe(
        [
            "--mode",
            "isaac",
            "--run-directory",
            str(run_directory),
        ],
        runtime_factory=lambda: FakeRuntime(run_directory=run_directory),
    )

    assert return_code == 0
    assert (run_directory / "environment.json").is_file()
    summary = load_json_object(run_directory / "technical-summary.json")
    assert summary["outcome"] == "passed"
    assert summary["action_metrics"]["passed"] is True
    assert summary["base_metrics"]["passed"] is True
    assert set(summary["action_plans"]) == {"left_joint1", "right_joint1"}
    assert summary["importer_events"] == []
    assert (run_directory / "visual-review.json").exists() is False


def test_isaac_probe_rejects_run_outside_configured_artifact_root(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    artifact_root = tmp_path / "artifacts"
    external_run = tmp_path / "external" / "run"
    monkeypatch.setattr(probe, "DEFAULT_ARTIFACT_ROOT", artifact_root)
    existing_default_run = artifact_root / "run"
    existing_default_run.mkdir(parents=True)
    sentinel = existing_default_run / "sentinel.txt"
    sentinel.write_text("keep", encoding="utf-8")
    _prepared_run(external_run.parent)

    assert (
        probe.execute_isaac_probe(
            ["--mode", "isaac", "--run-directory", str(external_run)],
            runtime_factory=FakeRuntime,
        )
        == 1
    )
    assert not (external_run / "technical-summary.json").exists()
    assert sentinel.read_text(encoding="utf-8") == "keep"
    failure_runs = [
        path
        for path in artifact_root.iterdir()
        if path.is_dir() and path != existing_default_run
    ]
    assert len(failure_runs) == 1
    summary = load_json_object(failure_runs[0] / "technical-summary.json")
    assert summary["stage"] == "validation"
    assert "artifact root" in summary["reason"]


@pytest.mark.parametrize(
    ("error", "stage"),
    [
        (
            probe.IsaacRuntimeFailure(
                "unsupported runtime",
                stage="environment",
                partial_result=_valid_result(),
            ),
            "environment",
        ),
        (
            probe.IsaacRuntimeFailure(
                "importer blocked",
                stage="import",
                importer_events=(
                    ImporterEvent(
                        category="dropped_camera",
                        severity="warning",
                        source="urdf",
                        message="camera dropped",
                        decision="blocking",
                    ),
                ),
            ),
            "import",
        ),
        (RuntimeError("runtime stopped"), "runtime"),
        (
            probe.IsaacRuntimeFailure("shutdown failed", stage="shutdown"),
            "shutdown",
        ),
    ],
)
def test_isaac_probe_runtime_failures_are_recorded(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    error: Exception,
    stage: str,
) -> None:
    monkeypatch.setattr(probe, "DEFAULT_ARTIFACT_ROOT", tmp_path)
    run_directory = tmp_path / "run"
    run_directory.mkdir()
    (run_directory / "urdf-inventory.json").write_text("{}", encoding="utf-8")
    (run_directory / "extracted" / "URDF_robot_1").mkdir(parents=True)
    (run_directory / "extracted" / "URDF_robot_1" / "robot.urdf").write_text(
        "<robot/>", encoding="utf-8"
    )

    assert (
        probe.execute_isaac_probe(
            ["--mode", "isaac", "--run-directory", str(run_directory)],
            runtime_factory=lambda: FakeRuntime(error=error),
        )
        == 1
    )
    summary = load_json_object(run_directory / "technical-summary.json")
    assert summary["outcome"] == "failed"
    assert summary["is_pass"] is False
    assert summary["stage"] == stage


def test_failed_summary_preserves_and_deduplicates_importer_events() -> None:
    partial = _valid_result()
    partial = IsaacRuntimeResult(
        **{
            **partial.__dict__,
            "importer_events": (
                ImporterEvent(
                    category="dropped_camera",
                    severity="warning",
                    source="isaac-log",
                    message="camera dropped",
                ),
                ImporterEvent(
                    category="material_change",
                    severity="warning",
                    source="isaac-log",
                    message="material changed",
                ),
            ),
        }
    )
    explicit = [partial.importer_events[0].to_dict()]
    summary = probe._failed_summary(
        "blocked",
        "import",
        importer_events=explicit,
        partial_result=partial,
    )
    assert summary["importer_events"] == [
        explicit[0],
        partial.importer_events[1].to_dict(),
    ]


def test_failed_summary_replaces_nonfinite_partial_samples_with_null(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(probe, "DEFAULT_ARTIFACT_ROOT", tmp_path)
    result = _valid_result()
    partial = IsaacRuntimeResult(
        **{
            **result.__dict__,
            "importer_events": (
                ImporterEvent(
                    category="runtime_failure",
                    severity="error",
                    source="isaac-runtime",
                    message="partial sample was invalid",
                ),
            ),
            "trajectories": {
                "left_joint1": (
                    JointSample(float("nan"), float("inf"), float("-inf")),
                ),
            },
            "base_motion": (
                TransformSample(
                    float("nan"),
                    (float("inf"), 0.0, 0.0),
                    (0.0, 0.0, float("-inf"), 1.0),
                ),
            ),
        }
    )
    run_directory = _prepared_run(tmp_path)

    assert (
        probe.execute_isaac_probe(
            ["--mode", "isaac", "--run-directory", str(run_directory)],
            runtime_factory=lambda: FakeRuntime(
                error=probe.IsaacRuntimeFailure(
                    "partial runtime failure",
                    stage="action",
                    partial_result=partial,
                    importer_events=partial.importer_events,
                )
            ),
        )
        == 1
    )

    summary = load_json_object(run_directory / "technical-summary.json")
    assert summary["outcome"] == "failed"
    assert summary["reason"] == "partial runtime failure"
    assert summary["stage"] == "action"
    assert summary["importer_events"] == [partial.importer_events[0].to_dict()]
    assert summary["trajectories"]["left_joint1"][0] == {
        "time_s": None,
        "position_rad": None,
        "velocity_rad_s": None,
    }
    assert summary["base_motion"][0] == {
        "time_s": None,
        "translation": [None, 0.0, 0.0],
        "rotation_xyzw": [0.0, 0.0, None, 1.0],
    }


def test_isaac_probe_rejects_unsupported_fake_version(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(probe, "DEFAULT_ARTIFACT_ROOT", tmp_path)
    result = _valid_result()
    result.environment.capabilities  # Keep the fake result aligned with one mutation helper.
    mutated = IsaacRuntimeResult(
        **{
            **result.__dict__,
            "environment": IsaacEnvironment(
                executable=result.environment.executable,
                semantic_version="2023.2.0",
                kit_version=result.environment.kit_version,
                python_version=result.environment.python_version,
                renderer=result.environment.renderer,
                run_mode=result.environment.run_mode,
                capabilities=result.environment.capabilities,
            ),
        }
    )
    run_directory = _prepared_run(tmp_path)
    assert (
        probe.execute_isaac_probe(
            ["--mode", "isaac", "--run-directory", str(run_directory)],
            runtime_factory=lambda: FakeRuntime(result=mutated),
        )
        == 1
    )
    assert (
        load_json_object(run_directory / "technical-summary.json")["outcome"]
        == "failed"
    )


def test_isaac_probe_rejects_failed_action_evaluator(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(probe, "DEFAULT_ARTIFACT_ROOT", tmp_path)
    result = _valid_result()
    failed_plans = {
        plan.joint_name: JointActionPlan(
            joint_name=plan.joint_name,
            initial_position=0.0,
            target_position=0.2,
            lower_limit=plan.lower_limit,
            upper_limit=plan.upper_limit,
        )
        for plan in result.action_plans
    }
    mutated = IsaacRuntimeResult(**{**result.__dict__, "action_plans": failed_plans})
    run_directory = _prepared_run(tmp_path)
    _write_runtime_images(run_directory)
    assert (
        probe.execute_isaac_probe(
            ["--mode", "isaac", "--run-directory", str(run_directory)],
            runtime_factory=lambda: FakeRuntime(result=mutated),
        )
        == 1
    )
    assert (
        load_json_object(run_directory / "technical-summary.json")["outcome"]
        == "failed"
    )


@pytest.mark.parametrize(("stiffness", "damping"), [(201.0, 16.6), (200.0, 17.6)])
def test_isaac_probe_rejects_nonfixed_drive_gains(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    stiffness: float,
    damping: float,
) -> None:
    monkeypatch.setattr(probe, "DEFAULT_ARTIFACT_ROOT", tmp_path)
    result = _valid_result()
    mutated = IsaacRuntimeResult(
        **{
            **result.__dict__,
            "drive_configuration": DriveConfiguration(
                gains=DriveGains(stiffness=stiffness, damping=damping),
                limits=result.drive_configuration.limits,
                verified=True,
            ),
        }
    )
    run_directory = _prepared_run(tmp_path)
    assert (
        probe.execute_isaac_probe(
            ["--mode", "isaac", "--run-directory", str(run_directory)],
            runtime_factory=lambda: FakeRuntime(result=mutated),
        )
        == 1
    )
    summary = load_json_object(run_directory / "technical-summary.json")
    assert summary["stage"] == "validation"
    assert summary["drive_configuration"]["gains"]["stiffness"] == stiffness
    assert summary["drive_configuration"]["gains"]["damping"] == damping


def test_isaac_probe_failure_preserves_returned_runtime_evidence(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(probe, "DEFAULT_ARTIFACT_ROOT", tmp_path)
    result = _valid_result()
    event = ImporterEvent(
        category="material_change",
        severity="warning",
        source="isaac-log",
        message="material changed",
    )
    result = IsaacRuntimeResult(
        **{
            **result.__dict__,
            "importer_events": (event,),
            "drive_configuration": DriveConfiguration(
                gains=DriveGains(stiffness=201.0, damping=16.6),
                limits=result.drive_configuration.limits,
                verified=True,
            ),
        }
    )
    run_directory = _prepared_run(tmp_path)
    assert (
        probe.execute_isaac_probe(
            ["--mode", "isaac", "--run-directory", str(run_directory)],
            runtime_factory=lambda: FakeRuntime(result=result),
        )
        == 1
    )
    summary = load_json_object(run_directory / "technical-summary.json")
    assert summary["stage"] == "validation"
    assert summary["environment"] == result.environment.to_dict()
    assert summary["importer_events"] == [event.to_dict()]


def test_isaac_probe_rejects_blank_render(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(probe, "DEFAULT_ARTIFACT_ROOT", tmp_path)
    run_directory = _prepared_run(tmp_path)
    assert (
        probe.execute_isaac_probe(
            ["--mode", "isaac", "--run-directory", str(run_directory)],
            runtime_factory=lambda: FakeRuntime(blank_render=True),
        )
        == 1
    )
    assert (
        load_json_object(run_directory / "technical-summary.json")["outcome"]
        == "failed"
    )


def test_record_visual_review_writes_exact_contract(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(probe, "DEFAULT_ARTIFACT_ROOT", tmp_path)
    run_directory = _prepared_run(tmp_path)
    _write_runtime_images(run_directory)
    assert (
        probe.record_visual_review(
            [
                "--visual-review",
                str(run_directory),
                "--status",
                "rejected",
                "--reviewer",
                "human",
                "--notes",
                "not acceptable",
            ]
        )
        == 0
    )
    review = load_json_object(run_directory / "visual-review.json")
    assert set(review) == set(VisualReview.__dataclass_fields__)


def test_record_visual_review_rejects_run_outside_artifact_root(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    artifact_root = tmp_path / "artifacts"
    external_run = tmp_path / "external" / "run"
    monkeypatch.setattr(probe, "DEFAULT_ARTIFACT_ROOT", artifact_root)
    external_run.mkdir(parents=True)
    _write_runtime_images(external_run)

    with pytest.raises(probe.IsaacProbeError, match="outside the artifact root"):
        probe.record_visual_review(
            [
                "--visual-review",
                str(external_run),
                "--status",
                "accepted",
                "--reviewer",
                "human",
                "--notes",
                "not acceptable",
            ]
        )
    assert not (external_run / "visual-review.json").exists()


def _prepared_run(tmp_path: Path) -> Path:
    run_directory = tmp_path / "run"
    (run_directory / "extracted" / "URDF_robot_1").mkdir(parents=True)
    (run_directory / "extracted" / "URDF_robot_1" / "robot.urdf").write_text(
        "<robot/>", encoding="utf-8"
    )
    (run_directory / "urdf-inventory.json").write_text("{}", encoding="utf-8")
    (run_directory / "robot_1.usd").write_bytes(b"USD")
    return run_directory


def _write_runtime_images(run_directory: Path) -> None:
    for name in ("scene-initial.png", "scene-action.png", "scene-final.png"):
        _write_screenshot(run_directory / name)


def _accepted_review(run_directory: Path) -> None:
    probe.atomic_write_json(
        run_directory / "visual-review.json",
        {
            "schema_version": 1,
            "status": "accepted",
            "screenshots": [
                "scene-initial.png",
                "scene-action.png",
                "scene-final.png",
            ],
            "visible_subsystems": [
                "chassis",
                "dual_arms",
                "left_gripper",
                "right_gripper",
                "head_camera",
                "wrist_cameras",
            ],
            "reviewer": "human",
            "reviewed_at": "2026-09-22T00:00:00Z",
            "notes": "all required subsystems visible",
        },
    )


def _passing_run(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    monkeypatch.setattr(probe, "DEFAULT_ARTIFACT_ROOT", tmp_path)
    run_directory = _prepared_run(tmp_path)
    _write_runtime_images(run_directory)
    _accepted_review(run_directory)
    assert (
        probe.execute_isaac_probe(
            ["--mode", "isaac", "--run-directory", str(run_directory)],
            runtime_factory=lambda: FakeRuntime(),
        )
        == 0
    )
    return run_directory


def test_milestone_gate_accepts_complete_evidence(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    assert probe.verify_milestone(_passing_run(tmp_path, monkeypatch)) == 0


def test_milestone_gate_mutations_fail_without_changing_review(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    run_directory = _passing_run(tmp_path, monkeypatch)
    original_review = (run_directory / "visual-review.json").read_bytes()
    summary = load_json_object(run_directory / "technical-summary.json")
    environment = load_json_object(run_directory / "environment.json")

    probe.atomic_write_json(
        run_directory / "visual-review.json",
        {
            **load_json_object(run_directory / "visual-review.json"),
            "status": "rejected",
        },
    )
    assert probe.verify_milestone(run_directory) == 1
    (run_directory / "visual-review.json").unlink()
    assert probe.verify_milestone(run_directory) == 1
    _accepted_review(run_directory)

    for key in probe.TECHNICAL_SUMMARY_KEYS:
        missing_key = json.loads(json.dumps(summary))
        del missing_key[key]
        probe.atomic_write_json(run_directory / "technical-summary.json", missing_key)
        assert probe.verify_milestone(run_directory) == 1
    probe.atomic_write_json(run_directory / "technical-summary.json", summary)

    probe.atomic_write_json(
        run_directory / "technical-summary.json", {**summary, "outcome": "failed"}
    )
    assert probe.verify_milestone(run_directory) == 1
    probe.atomic_write_json(run_directory / "technical-summary.json", summary)
    mutated_evidence = json.loads(json.dumps(summary))
    mutated_evidence["trajectories"]["left_joint1"][1]["position_rad"] = 1.1
    mutated_evidence["action_metrics"] = {"passed": True}
    mutated_evidence["base_metrics"] = {"passed": True}
    probe.atomic_write_json(run_directory / "technical-summary.json", mutated_evidence)
    assert probe.verify_milestone(run_directory) == 1
    probe.atomic_write_json(run_directory / "technical-summary.json", summary)
    (run_directory / "robot_1.usd").unlink()
    assert probe.verify_milestone(run_directory) == 1
    (run_directory / "robot_1.usd").write_bytes(b"USD")

    unsupported = {**environment, "semantic_version": "2023.2.0"}
    probe.atomic_write_json(run_directory / "environment.json", unsupported)
    assert probe.verify_milestone(run_directory) == 1
    probe.atomic_write_json(run_directory / "environment.json", environment)
    _write_screenshot(run_directory / "scene-initial.png", blank=True)
    assert probe.verify_milestone(run_directory) == 1
    _write_runtime_images(run_directory)

    assert probe.verify_milestone(tmp_path.parent / f"{tmp_path.name}-outside") == 1
    blocking = {
        **summary,
        "importer_events": [
            {
                "category": "dropped_camera",
                "severity": "warning",
                "source": "urdf",
                "message": "camera dropped",
                "decision": "blocking",
            }
        ],
    }
    probe.atomic_write_json(run_directory / "technical-summary.json", blocking)
    assert probe.verify_milestone(run_directory) == 1
    assert (run_directory / "visual-review.json").read_bytes() == original_review


@pytest.mark.parametrize(
    "gains",
    [
        {"stiffness": 201.0, "damping": 16.6},
        {"stiffness": 200.0, "damping": 17.6},
    ],
)
def test_milestone_gate_rejects_nonfixed_drive_gains(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    gains: dict[str, float],
) -> None:
    run_directory = _passing_run(tmp_path, monkeypatch)
    summary = load_json_object(run_directory / "technical-summary.json")
    technical = {
        **summary,
        "drive_configuration": {
            **summary["drive_configuration"],
            "gains": gains,
        },
    }
    probe.atomic_write_json(run_directory / "technical-summary.json", technical)
    assert probe.verify_milestone(run_directory) == 1
