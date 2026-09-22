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

import argparse
import json
import math
import os
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from uuid import uuid4

from robots.lynsense.isaac.action_contract import (
    JointActionPlan,
    JointSample,
    TransformSample,
    evaluate_base_motion,
    evaluate_joint_action,
)
from robots.lynsense.isaac.archive import MAX_UNCOMPRESSED_BYTES, extract_archive
from robots.lynsense.isaac.artifacts import (
    ArtifactPaths,
    atomic_write_json,
    create_artifact_paths,
    load_json_object,
)
from robots.lynsense.isaac.contracts import (
    SCREENSHOT_NAMES,
    IsaacProbeError,
    environment_from_dict,
    validate_environment,
    validate_visual_review,
)
from robots.lynsense.isaac.import_contract import (
    classify_importer_events,
    require_no_blocking_events,
)
from robots.lynsense.isaac.render_evidence import validate_render_evidence
from robots.lynsense.isaac.runtime import (
    Isaac3Runtime,
    IsaacRuntimeFailure,
    IsaacRuntimeResult,
)
from robots.lynsense.isaac.urdf_contract import inspect_urdf

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_ARTIFACT_ROOT = Path(".artifacts/lynsense-isaac").resolve()
DEFAULT_ARCHIVE = Path("../URDF/URDF_robot_1.tar.gz").resolve()
EXPLICIT_SKIP_EXIT_CODE = 77
MAX_UNCOMPRESSED_BYTES_CEILING = MAX_UNCOMPRESSED_BYTES
TECHNICAL_SUMMARY_KEYS = frozenset(
    {
        "outcome",
        "is_pass",
        "environment",
        "importer_events",
        "articulation_joint_names",
        "loaded_joint_names",
        "loaded_link_names",
        "drive_configuration",
        "action_plans",
        "action_metrics",
        "base_metrics",
        "trajectories",
        "base_motion",
        "render_metrics",
        "generated_usd",
        "run_directory",
        "max_uncompressed_bytes",
    }
)


def _json_safe(value: Any) -> Any:
    """Replace non-finite numeric values before writing failure evidence."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        safe: dict[Any, Any] = {}
        for key, item in value.items():
            if isinstance(key, float) and not math.isfinite(key):
                continue
            safe[key] = _json_safe(item)
        return safe
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


def _positive_integer(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be an integer") from exc
    if parsed < 1:
        raise argparse.ArgumentTypeError("must be at least 1")
    if parsed > MAX_UNCOMPRESSED_BYTES_CEILING:
        raise argparse.ArgumentTypeError(
            f"must not exceed {MAX_UNCOMPRESSED_BYTES_CEILING}"
        )
    return parsed


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("host", "isaac"), default="host")
    parser.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    parser.add_argument("--isaac-python", type=Path)
    parser.add_argument(
        "--max-uncompressed-bytes",
        type=_positive_integer,
        default=MAX_UNCOMPRESSED_BYTES,
    )
    parser.add_argument("--run-directory", type=Path)
    parser.add_argument("--visual-review", type=Path)
    parser.add_argument("--status", choices=("accepted", "rejected"))
    parser.add_argument(
        "--visual-review-status",
        choices=("accepted", "rejected"),
        default="accepted",
    )
    parser.add_argument("--reviewer")
    parser.add_argument("--notes")
    parser.add_argument("--verify-milestone", type=Path)
    return parser


def _run_id() -> str:
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return f"{timestamp}-{uuid4().hex}"


def _paths(root: Path, run_id: str | None = None) -> tuple[ArtifactPaths, str]:
    identifier = run_id or _run_id()
    return create_artifact_paths(root, identifier), identifier


def _failed_summary(
    reason: str,
    stage: str,
    *,
    importer_events: list[dict[str, object]] | None = None,
    partial_result: IsaacRuntimeResult | None = None,
) -> dict[str, Any]:
    merged_importer_events: list[dict[str, object]] = []
    seen_events: set[str] = set()
    event_documents = list(importer_events or ())
    if partial_result is not None:
        event_documents.extend(
            event.to_dict() for event in partial_result.importer_events
        )
    for event in event_documents:
        event_key = json.dumps(event, sort_keys=True, separators=(",", ":"))
        if event_key in seen_events:
            continue
        seen_events.add(event_key)
        merged_importer_events.append(event)
    summary: dict[str, Any] = {
        "outcome": "failed",
        "is_pass": False,
        "reason": reason,
        "stage": stage,
        "importer_events": merged_importer_events,
    }
    if partial_result is not None:
        summary["environment"] = partial_result.environment.to_dict()
        summary["loaded_joint_names"] = list(partial_result.loaded_joint_names)
        summary["loaded_link_names"] = list(partial_result.loaded_link_names)
        summary["action_metrics"] = {"passed": False}
        summary["base_metrics"] = {"passed": False}
        summary["drive_configuration"] = partial_result.drive_configuration.to_dict()
        summary["trajectories"] = {
            name: [sample.__dict__ for sample in trajectory]
            for name, trajectory in partial_result.trajectories.items()
        }
        summary["base_motion"] = [
            sample.__dict__ for sample in partial_result.base_motion
        ]
    return _json_safe(summary)


def _write_failure(
    paths: ArtifactPaths,
    reason: str,
    stage: str,
    *,
    importer_events: list[dict[str, object]] | None = None,
    partial_result: IsaacRuntimeResult | None = None,
) -> None:
    paths.run_directory.mkdir(mode=0o700, parents=True, exist_ok=True)
    os.chmod(paths.run_directory, 0o700)
    atomic_write_json(
        paths.technical_summary_json,
        _failed_summary(
            reason,
            stage,
            importer_events=importer_events,
            partial_result=partial_result,
        ),
    )


def _write_child_failure(paths: ArtifactPaths, reason: str) -> None:
    """Mark a child failure without discarding evidence it already wrote."""
    try:
        summary = load_json_object(paths.technical_summary_json)
    except ValueError:
        summary = {}
    summary.update(
        {
            "outcome": "failed",
            "is_pass": False,
            "reason": reason,
            "stage": "child",
        }
    )
    paths.run_directory.mkdir(mode=0o700, parents=True, exist_ok=True)
    os.chmod(paths.run_directory, 0o700)
    atomic_write_json(paths.technical_summary_json, summary)


def _is_complete_passing_summary(summary: dict[str, Any]) -> bool:
    return (
        summary.get("outcome") == "passed"
        and summary.get("is_pass") is True
        and TECHNICAL_SUMMARY_KEYS.issubset(summary)
    )


def _runtime_evidence(result: IsaacRuntimeResult) -> dict[str, Any]:
    action_plan_items = (
        result.action_plans.items()
        if isinstance(result.action_plans, dict)
        else ((plan.joint_name, plan) for plan in result.action_plans)
    )
    action_plan_mapping = dict(action_plan_items)
    action_plans = {
        name: {
            "joint_name": plan.joint_name,
            "initial_position": plan.initial_position,
            "target_position": plan.target_position,
            "lower_limit": plan.lower_limit,
            "upper_limit": plan.upper_limit,
        }
        for name, plan in action_plan_mapping.items()
    }
    return {
        "environment": result.environment.to_dict(),
        "importer_events": [event.to_dict() for event in result.importer_events],
        "articulation_joint_names": list(result.articulation_joint_names),
        "loaded_joint_names": list(result.loaded_joint_names),
        "loaded_link_names": list(result.loaded_link_names),
        "drive_configuration": result.drive_configuration.to_dict(),
        "action_plans": action_plans,
        "action_metrics": evaluate_joint_action(
            result.trajectories, action_plan_mapping
        ),
        "base_metrics": evaluate_base_motion(result.base_motion),
        "trajectories": {
            name: [sample.__dict__ for sample in trajectory]
            for name, trajectory in result.trajectories.items()
        },
        "base_motion": [sample.__dict__ for sample in result.base_motion],
    }


def execute_host_probe(argv: list[str]) -> int:
    args = build_parser().parse_args(argv)
    DEFAULT_ARTIFACT_ROOT.mkdir(mode=0o700, parents=True, exist_ok=True)
    paths, _ = _paths(DEFAULT_ARTIFACT_ROOT)
    paths.run_directory.mkdir(mode=0o700)
    os.chmod(paths.run_directory, 0o700)
    try:
        extract_archive(
            args.archive,
            paths.extracted_root,
            max_uncompressed_bytes=args.max_uncompressed_bytes,
        )
        atomic_write_json(
            paths.run_directory / "urdf-inventory.json",
            inspect_urdf(paths.robot_urdf).to_dict(),
        )
    except (IsaacProbeError, OSError) as exc:
        _write_failure(paths, str(exc), "host")
        return 1

    if args.isaac_python is None:
        atomic_write_json(
            paths.technical_summary_json,
            {
                "outcome": "skipped",
                "is_pass": False,
                "reason": "No explicit Isaac Sim Python executable was supplied",
            },
        )
        return EXPLICIT_SKIP_EXIT_CODE

    command = [
        str(args.isaac_python),
        str(Path(__file__).resolve()),
        "--mode",
        "isaac",
        "--isaac-python",
        str(args.isaac_python),
        "--run-directory",
        str(paths.run_directory),
        "--max-uncompressed-bytes",
        str(args.max_uncompressed_bytes),
    ]
    environment = os.environ.copy()
    existing_path = environment.get("PYTHONPATH")
    environment["PYTHONPATH"] = (
        f"{REPOSITORY_ROOT}{os.pathsep}{existing_path}"
        if existing_path
        else str(REPOSITORY_ROOT)
    )
    try:
        with paths.isaac_log.open("wb") as log_file:
            completed = subprocess.run(
                command,
                stdout=log_file,
                stderr=log_file,
                env=environment,
                check=False,
            )
    except OSError as exc:
        _write_failure(paths, f"Isaac child launch failed: {exc}", "launch")
        return 1
    if completed.returncode != 0:
        _write_child_failure(
            paths,
            f"Isaac child exited with code {completed.returncode}",
        )
        return 1

    try:
        summary = load_json_object(paths.technical_summary_json)
    except ValueError:
        _write_child_failure(
            paths,
            "Isaac child produced no valid technical summary",
        )
        return 1
    if _is_complete_passing_summary(summary):
        return 0
    _write_child_failure(
        paths,
        "Isaac child did not report a passed technical summary",
    )
    return 1


def _candidate_run_directory(args: argparse.Namespace) -> ArtifactPaths:
    if args.run_directory is None:
        return create_artifact_paths(DEFAULT_ARTIFACT_ROOT, _run_id())
    run_directory = args.run_directory.resolve()
    artifact_root = DEFAULT_ARTIFACT_ROOT.resolve()
    if run_directory.parent != artifact_root:
        return create_artifact_paths(DEFAULT_ARTIFACT_ROOT, _run_id())
    return create_artifact_paths(DEFAULT_ARTIFACT_ROOT, run_directory.name)


def _validated_run_directory(run_directory: Path) -> tuple[Path, ArtifactPaths]:
    resolved_run = run_directory.resolve()
    artifact_root = DEFAULT_ARTIFACT_ROOT.resolve()
    try:
        relative = resolved_run.relative_to(artifact_root)
    except ValueError as exc:
        raise IsaacProbeError("run directory is outside the artifact root") from exc
    if len(relative.parts) != 1:
        raise IsaacProbeError("run directory is not a probe run")
    return resolved_run, create_artifact_paths(artifact_root, resolved_run.name)


def _parse_action_plans(document: Any) -> dict[str, JointActionPlan]:
    fields = {
        "joint_name",
        "initial_position",
        "target_position",
        "lower_limit",
        "upper_limit",
    }
    if not isinstance(document, dict):
        raise IsaacProbeError("action_plans must be an object")
    plans: dict[str, JointActionPlan] = {}
    for name, value in document.items():
        if not isinstance(name, str) or not isinstance(value, dict):
            raise IsaacProbeError("action_plans entries are malformed")
        if set(value) != fields:
            raise IsaacProbeError(f"action plan for {name!r} is malformed")
        try:
            plans[name] = JointActionPlan(**value)
        except TypeError as exc:
            raise IsaacProbeError(f"action plan for {name!r} is malformed") from exc
    return plans


def _parse_trajectories(document: Any) -> dict[str, tuple[JointSample, ...]]:
    fields = {"time_s", "position_rad", "velocity_rad_s"}
    if not isinstance(document, dict):
        raise IsaacProbeError("trajectories must be an object")
    trajectories: dict[str, tuple[JointSample, ...]] = {}
    for name, values in document.items():
        if not isinstance(name, str) or not isinstance(values, list):
            raise IsaacProbeError("trajectory entries are malformed")
        samples: list[JointSample] = []
        for value in values:
            if not isinstance(value, dict) or set(value) != fields:
                raise IsaacProbeError(f"trajectory for {name!r} is malformed")
            try:
                samples.append(JointSample(**value))
            except TypeError as exc:
                raise IsaacProbeError(f"trajectory for {name!r} is malformed") from exc
        trajectories[name] = tuple(samples)
    return trajectories


def _parse_base_motion(document: Any) -> tuple[TransformSample, ...]:
    fields = {"time_s", "translation", "rotation_xyzw"}
    if not isinstance(document, list):
        raise IsaacProbeError("base_motion must be a list")
    samples: list[TransformSample] = []
    for value in document:
        if not isinstance(value, dict) or set(value) != fields:
            raise IsaacProbeError("base motion sample is malformed")
        translation = value["translation"]
        rotation = value["rotation_xyzw"]
        if (
            not isinstance(translation, list)
            or len(translation) != 3
            or not isinstance(rotation, list)
            or len(rotation) != 4
        ):
            raise IsaacProbeError("base motion sample is malformed")
        try:
            samples.append(
                TransformSample(
                    time_s=value["time_s"],
                    translation=tuple(translation),
                    rotation_xyzw=tuple(rotation),
                )
            )
        except TypeError as exc:
            raise IsaacProbeError("base motion sample is malformed") from exc
    return tuple(samples)


def _prepared_run_directory(args: argparse.Namespace) -> ArtifactPaths:
    if args.run_directory is None:
        raise IsaacProbeError("Isaac mode requires --run-directory")
    run_directory = args.run_directory.resolve()
    run_id = run_directory.name
    if run_directory.parent != DEFAULT_ARTIFACT_ROOT.resolve():
        raise IsaacProbeError("run directory must be under the artifact root")
    return create_artifact_paths(run_directory.parent, run_id)


def _write_passed_summary(
    args: argparse.Namespace, paths: ArtifactPaths, result: IsaacRuntimeResult
) -> None:
    validate_environment(result.environment)
    classified = classify_importer_events(result.importer_events)
    require_no_blocking_events(classified)
    from robots.lynsense.isaac.urdf_contract import (
        REQUIRED_GRIPPER_JOINT_NAMES,
        REQUIRED_GRIPPER_LINK_NAMES,
        REQUIRED_JOINT_NAMES,
        REQUIRED_LINK_NAMES,
    )

    required_joints = set(REQUIRED_JOINT_NAMES)
    missing_joints = required_joints - set(result.loaded_joint_names)
    if missing_joints:
        raise IsaacProbeError(
            f"generated robot is missing joints: {sorted(missing_joints)[0]}"
        )

    missing = set(REQUIRED_LINK_NAMES) - set(result.loaded_link_names)
    missing |= set(REQUIRED_GRIPPER_JOINT_NAMES) - set(result.loaded_joint_names)
    missing |= set(REQUIRED_GRIPPER_LINK_NAMES) - set(result.loaded_link_names)
    if missing:
        raise IsaacProbeError("generated robot is missing required names")
    drive = result.drive_configuration
    if (
        not drive.verified
        or drive.gains.stiffness != 200.0
        or drive.gains.damping != 16.6
        or drive.limits.max_effort_nm != 200.0
        or drive.limits.max_velocity_rad_s != 0.314
    ):
        raise IsaacProbeError("drive gains and limits were not verified")
    render_metrics = validate_render_evidence(paths.run_directory)
    summary: dict[str, Any] = {
        "outcome": "passed",
        "is_pass": True,
        "max_uncompressed_bytes": args.max_uncompressed_bytes,
        "generated_usd": str(paths.generated_usd),
        "run_directory": str(paths.run_directory),
        "render_metrics": render_metrics,
        **_runtime_evidence(result),
    }
    atomic_write_json(paths.environment_json, result.environment.to_dict())
    atomic_write_json(paths.technical_summary_json, summary)


def execute_isaac_probe(
    argv: list[str],
    runtime_factory: type[Isaac3Runtime] | Any = Isaac3Runtime,
) -> int:
    args = build_parser().parse_args(argv)
    paths = _candidate_run_directory(args)
    result: IsaacRuntimeResult | None = None
    try:
        paths = _prepared_run_directory(args)
        if not (paths.run_directory / "urdf-inventory.json").is_file():
            raise IsaacProbeError("offline URDF inventory is required")
        if not paths.robot_urdf.is_file():
            raise IsaacProbeError("extracted robot URDF is required")

        if runtime_factory is Isaac3Runtime:
            if args.isaac_python is None:
                raise IsaacProbeError(
                    "Isaac mode requires an explicit --isaac-python executable"
                )

            def persist_before_app_close(
                callback_result: IsaacRuntimeResult | None,
                callback_failure: IsaacRuntimeFailure | None,
            ) -> None:
                try:
                    if callback_failure is not None:
                        raise callback_failure
                    if callback_result is None:
                        raise IsaacProbeError("Isaac runtime returned no result")
                    _write_passed_summary(args, paths, callback_result)
                except IsaacRuntimeFailure as exc:
                    _write_failure(
                        paths,
                        str(exc),
                        exc.stage,
                        importer_events=[
                            event.to_dict() for event in exc.importer_events
                        ],
                        partial_result=exc.partial_result,
                    )
                except Exception as exc:
                    stage = (
                        "validation" if isinstance(exc, IsaacProbeError) else "runtime"
                    )
                    _write_failure(
                        paths,
                        str(exc),
                        stage,
                        importer_events=(
                            [
                                event.to_dict()
                                for event in callback_result.importer_events
                            ]
                            if callback_result is not None
                            else None
                        ),
                        partial_result=callback_result,
                    )

            runtime = Isaac3Runtime.with_default_drive_gains(
                args.isaac_python,
                before_app_close=persist_before_app_close,
            )
        else:
            runtime = runtime_factory()
        result = runtime.run(paths)
        _write_passed_summary(args, paths, result)
        return 0
    except IsaacRuntimeFailure as exc:
        _write_failure(
            paths,
            str(exc),
            exc.stage,
            importer_events=[event.to_dict() for event in exc.importer_events],
            partial_result=exc.partial_result,
        )
    except Exception as exc:
        stage = "validation" if isinstance(exc, IsaacProbeError) else "runtime"
        _write_failure(
            paths,
            str(exc),
            stage,
            importer_events=(
                [event.to_dict() for event in result.importer_events]
                if result is not None
                else None
            ),
            partial_result=result,
        )
    return 1


def record_visual_review(argv: list[str]) -> int:
    args = build_parser().parse_args(argv)
    if args.visual_review is None:
        raise IsaacProbeError("visual review requires --visual-review")
    run_directory, paths = _validated_run_directory(args.visual_review)
    if not args.status or not args.reviewer or not args.notes:
        raise IsaacProbeError("status, reviewer, and notes are required")
    validate_render_evidence(run_directory)
    review = {
        "schema_version": 1,
        "status": args.status,
        "screenshots": list(SCREENSHOT_NAMES),
        "visible_subsystems": [
            "chassis",
            "dual_arms",
            "left_gripper",
            "right_gripper",
            "head_camera",
            "wrist_cameras",
        ],
        "reviewer": args.reviewer,
        "reviewed_at": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "notes": args.notes,
    }
    validate_visual_review(review)
    atomic_write_json(paths.visual_review_json, review)
    return 0


def _milestone_failure(message: str) -> int:
    print(f"milestone rejected: {message}", file=sys.stderr)
    return 1


def verify_milestone(run_directory: Path) -> int:
    try:
        run_directory, paths = _validated_run_directory(run_directory)
        environment = environment_from_dict(load_json_object(paths.environment_json))
        technical = load_json_object(paths.technical_summary_json)
        review = load_json_object(paths.visual_review_json)
        validate_visual_review(review)
        validate_environment(environment)
        if environment.semantic_version.startswith("2023.1.") is False:
            return _milestone_failure("unsupported Isaac version")
        missing_keys = TECHNICAL_SUMMARY_KEYS.difference(technical)
        if missing_keys:
            return _milestone_failure(
                f"technical summary is missing {sorted(missing_keys)[0]}"
            )
        if technical.get("outcome") != "passed" or technical.get("is_pass") is not True:
            return _milestone_failure("technical probe did not pass")
        drive_configuration = technical.get("drive_configuration", {})
        gains = drive_configuration.get("gains", {})
        drive = drive_configuration.get("limits", {})
        if (
            drive_configuration.get("verified") is not True
            or gains.get("stiffness") != 200.0
            or gains.get("damping") != 16.6
            or drive.get("max_effort_nm") != 200.0
            or drive.get("max_velocity_rad_s") != 0.314
        ):
            return _milestone_failure("drive gains and limits were not verified")
        if technical.get("action_metrics", {}).get("passed") is not True:
            return _milestone_failure("joint action did not pass")
        if technical.get("base_metrics", {}).get("passed") is not True:
            return _milestone_failure("fixed-base evidence did not pass")
        action_plans = _parse_action_plans(technical.get("action_plans"))
        trajectories = _parse_trajectories(technical.get("trajectories"))
        base_motion = _parse_base_motion(technical.get("base_motion"))
        evaluate_joint_action(trajectories, action_plans)
        evaluate_base_motion(base_motion)
        events = tuple(
            __import__(
                "robots.lynsense.isaac.import_contract",
                fromlist=["ImporterEvent"],
            ).ImporterEvent(**event)
            for event in technical.get("importer_events", ())
        )
        classified = classify_importer_events(events)
        if any(event.decision == "blocking" for event in classified):
            return _milestone_failure(
                "technical probe contains a blocking importer event"
            )
        if review.get("status") != "accepted":
            return _milestone_failure("human review is not accepted")
        if not paths.generated_usd.is_file():
            return _milestone_failure("generated USD is missing")
        if any(not (run_directory / name).is_file() for name in SCREENSHOT_NAMES):
            return _milestone_failure("a screenshot is missing")
        render = validate_render_evidence(run_directory)
        if render.get("all_nonblank") is not True:
            return _milestone_failure("render evidence is blank")
        return 0
    except (IsaacProbeError, ValueError, KeyError, TypeError, AttributeError) as exc:
        return _milestone_failure(str(exc))


def main() -> int:
    args = build_parser().parse_args()
    if args.verify_milestone is not None:
        return verify_milestone(args.verify_milestone)
    if args.visual_review is not None:
        return record_visual_review(sys.argv[1:])
    if args.mode == "isaac":
        return execute_isaac_probe(sys.argv[1:])
    return execute_host_probe(sys.argv[1:])


if __name__ == "__main__":
    raise SystemExit(main())
