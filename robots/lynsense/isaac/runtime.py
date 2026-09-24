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

import asyncio
import inspect
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Protocol

from robots.lynsense.isaac.action_contract import (
    DriveConfiguration,
    DriveGains,
    DriveLimits,
    JointActionPlan,
    JointSample,
    TransformSample,
    evaluate_base_motion,
    evaluate_joint_action,
    plan_joint_action,
    validate_step_state,
)
from robots.lynsense.isaac.artifacts import ArtifactPaths
from robots.lynsense.isaac.contracts import (
    IsaacEnvironment,
    IsaacProbeError,
    validate_environment,
    validate_environment_version,
)
from robots.lynsense.isaac.import_contract import (
    ImporterContractError,
    ImporterEvent,
    classify_importer_events,
    require_no_blocking_events,
)
from robots.lynsense.isaac.urdf_contract import (
    REQUIRED_GRIPPER_JOINT_NAMES,
    REQUIRED_GRIPPER_LINK_NAMES,
    REQUIRED_JOINT_NAMES,
    REQUIRED_LINK_NAMES,
)


@dataclass(frozen=True)
class IsaacRuntimeResult:
    environment: IsaacEnvironment
    importer_events: tuple[ImporterEvent, ...]
    articulation_joint_names: tuple[str, ...]
    loaded_joint_names: tuple[str, ...]
    loaded_link_names: tuple[str, ...]
    action_plans: dict[str, JointActionPlan]
    drive_configuration: DriveConfiguration
    trajectories: dict[str, tuple[JointSample, ...]]
    base_motion: tuple[TransformSample, ...]


class IsaacRuntimeFailure(IsaacProbeError):
    """An Isaac failure that preserves partial evidence."""

    def __init__(
        self,
        message: str,
        *,
        stage: str,
        partial_result: IsaacRuntimeResult | None = None,
        importer_events: tuple[ImporterEvent, ...] = (),
    ) -> None:
        super().__init__(message)
        self.stage = stage
        self.partial_result = partial_result
        self.importer_events = importer_events


class IsaacRuntime(Protocol):
    def run(self, paths: ArtifactPaths) -> IsaacRuntimeResult:
        """Import and exercise the prepared robot."""


def _await_isaac_task(task: object, output_path: Path) -> None:
    waiter = getattr(task, "wait_for_result", None)
    if callable(waiter):
        result = task.wait_for_result()  # type: ignore[attr-defined]
        if inspect.isawaitable(result):
            asyncio.run(result)
    elif inspect.isawaitable(task):
        asyncio.run(task)
    else:
        raise IsaacProbeError(
            "viewport capture must return wait_for_result or an awaitable"
        )
    if not output_path.is_file() or output_path.stat().st_size == 0:
        raise IsaacProbeError(
            f"viewport capture did not create a nonempty file: {output_path.name}"
        )


def _required_joint_events(robot_model: object) -> tuple[ImporterEvent, ...]:
    joints = getattr(robot_model, "joints", ())
    if isinstance(joints, dict):
        available = set(joints)
        joint_values = list(joints.values())
    else:
        names: list[str] = []
        for joint in joints:
            names.append(str(getattr(joint, "name", "")))
        available = set(names)
        joint_values = list(joints)

    events: list[ImporterEvent] = []
    for name in REQUIRED_JOINT_NAMES:
        if name not in available:
            events.append(
                ImporterEvent(
                    category="missing_required_joint",
                    severity="error",
                    source="urdf-model",
                    message=f"required joint {name!r} is absent",
                )
            )
            continue
        joint = next(
            (item for item in joint_values if getattr(item, "name", "") == name),
            None,
        )
        if joint is None:
            continue
        limit = getattr(joint, "limit", None)
        lower = getattr(joint, "lower", None)
        upper = getattr(joint, "upper", None)
        if lower is None and limit is not None:
            lower = getattr(limit, "lower", None)
        if upper is None and limit is not None:
            upper = getattr(limit, "upper", None)
        if lower is None or upper is None:
            events.append(
                ImporterEvent(
                    category="invalid_joint_limit",
                    severity="error",
                    source="urdf-model",
                    message=f"required joint {name!r} has an incomplete limit",
                )
            )
    return tuple(events)


def _log_event(message: str) -> ImporterEvent | None:
    if "[omni.importer.urdf]" not in message:
        return None
    lowered = message.lower()
    if "has no colliders" in lowered:
        return None
    severity = (
        "error"
        if "[error]" in lowered
        else ("warning" if "[warning]" in lowered else None)
    )
    if severity is None:
        return None
    category = "uncategorized"
    if "required joint" in lowered or "joint limit" in lowered:
        category = "missing_required_joint"
    elif "mesh" in lowered and ("unreadable" in lowered or "substitut" in lowered):
        category = "unreadable_mesh"
    elif "gripper" in lowered and ("dropped" in lowered or "removed" in lowered):
        category = "dropped_gripper_link"
    elif ("camera" in lowered or "sensor" in lowered) and (
        "dropped" in lowered or "removed" in lowered
    ):
        category = "dropped_camera"
    elif "mimic" in lowered:
        category = "mimic_joint_decision"
    elif any(word in lowered for word in ("material", "color", "texture", "fixed")):
        category = "material_change"
    return ImporterEvent(
        category=category,
        severity=severity,
        source="isaac-log",
        message=message,
    )


def _log_events(paths: ArtifactPaths) -> tuple[ImporterEvent, ...]:
    if not paths.isaac_log.is_file():
        return ()
    events = []
    for line in paths.isaac_log.read_text(
        encoding="utf-8", errors="replace"
    ).splitlines()[-500:]:
        event = _log_event(line)
        if event is not None:
            events.append(event)
    return tuple(events)


def _observed_environment(isaac_python: Path) -> IsaacEnvironment:
    import platform

    import carb
    import omni.kit.app
    import omni.usd

    kit_app = omni.kit.app.get_app()
    if kit_app is None:
        raise IsaacProbeError("Isaac Kit application is unavailable")
    semantic_version = kit_app.get_app_version()
    kit_version = kit_app.get_kit_version()
    python_version = platform.python_version()
    settings = carb.settings.get_settings()
    renderer = settings.get("/rtx/rendermode")
    if not semantic_version or not kit_version or not renderer:
        raise IsaacProbeError("a required Isaac runtime metadata value is absent")
    if omni.usd.get_context() is None or omni.usd.get_context().get_stage() is None:
        raise IsaacProbeError("Isaac did not expose an active USD stage")
    capabilities = {"urdf_importer", "articulation", "viewport_screenshot"}
    try:
        import pxr.PhysxSchema

        if pxr.PhysxSchema.PhysxJointAPI is None:
            raise IsaacProbeError("PhysX joint API is unavailable")
    except (ImportError, AttributeError) as exc:
        raise IsaacProbeError("PhysX joint API is unavailable") from exc
    capabilities.add("physx_joint_limits")
    return IsaacEnvironment(
        executable=isaac_python,
        semantic_version=semantic_version,
        kit_version=kit_version,
        python_version=python_version,
        renderer=str(renderer),
        run_mode="desktop",
        capabilities=frozenset(capabilities),
    )


def _partial_result(
    environment: IsaacEnvironment | None,
    importer_events: tuple[ImporterEvent, ...],
    *,
    executable: Path | None = None,
    articulation_joint_names: tuple[str, ...] = (),
    loaded_joint_names: tuple[str, ...] = (),
    loaded_link_names: tuple[str, ...] = (),
    action_plans: dict[str, JointActionPlan] | None = None,
    drive_configuration: DriveConfiguration | None = None,
    trajectories: dict[str, tuple[JointSample, ...]] | None = None,
    base_motion: tuple[TransformSample, ...] = (),
) -> IsaacRuntimeResult:
    return IsaacRuntimeResult(
        environment=environment
        or IsaacEnvironment(
            executable=executable or Path("unobserved"),
            semantic_version="unobserved",
            kit_version="unobserved",
            python_version="unobserved",
            renderer="unobserved",
            capabilities=frozenset(),
        ),
        importer_events=importer_events,
        articulation_joint_names=articulation_joint_names,
        loaded_joint_names=loaded_joint_names,
        loaded_link_names=loaded_link_names,
        action_plans=action_plans or {},
        drive_configuration=drive_configuration
        or DriveConfiguration(gains=DriveGains(), limits=DriveLimits(), verified=False),
        trajectories=trajectories or {},
        base_motion=base_motion,
    )


class Isaac3Runtime:
    """The supported 2023.1 Isaac Sim execution boundary."""

    @classmethod
    def with_default_drive_gains(
        cls,
        isaac_python: Path,
        before_app_close: Callable[
            [IsaacRuntimeResult | None, IsaacRuntimeFailure | None],
            None,
        ]
        | None = None,
    ) -> "Isaac3Runtime":
        return cls(
            drive_gains=DriveGains(stiffness=200.0, damping=16.6),
            isaac_python=isaac_python,
            before_app_close=before_app_close,
        )

    def __init__(
        self,
        drive_gains: DriveGains = DriveGains(),
        *,
        isaac_python: Path | None = None,
        before_app_close: Callable[
            [IsaacRuntimeResult | None, IsaacRuntimeFailure | None],
            None,
        ]
        | None = None,
    ) -> None:
        self.isaac_python = isaac_python
        self.drive_gains = drive_gains
        self._before_app_close = before_app_close
        self._app = None
        self._world = None
        self._latest_partial_result: IsaacRuntimeResult | None = None
        self._latest_importer_events: tuple[ImporterEvent, ...] = ()

    def _remember_partial(
        self,
        partial_result: IsaacRuntimeResult,
        importer_events: tuple[ImporterEvent, ...] | None = None,
    ) -> IsaacRuntimeResult:
        self._latest_partial_result = partial_result
        self._latest_importer_events = (
            partial_result.importer_events
            if importer_events is None
            else importer_events
        )
        return partial_result

    def run(self, paths: ArtifactPaths) -> IsaacRuntimeResult:
        self._latest_partial_result = None
        self._latest_importer_events = ()
        original_failure: IsaacRuntimeFailure | None = None
        shutdown_failure: IsaacRuntimeFailure | None = None
        result: IsaacRuntimeResult | None = None
        try:
            result = self._run(paths)
            return result
        except Exception as exc:
            if isinstance(exc, IsaacRuntimeFailure):
                original_failure = exc
            else:
                original_failure = IsaacRuntimeFailure(
                    str(exc),
                    stage="runtime",
                    partial_result=self._latest_partial_result,
                    importer_events=self._latest_importer_events,
                )
            if isinstance(exc, IsaacRuntimeFailure):
                raise
            raise original_failure from exc
        finally:
            if self._world is not None:
                try:
                    stop = getattr(self._world, "stop", None)
                    if callable(stop):
                        stop()
                except Exception as exc:
                    shutdown_failure = IsaacRuntimeFailure(
                        f"Isaac world shutdown failed: {exc}", stage="shutdown"
                    )
                try:
                    clear_instance = getattr(self._world, "clear_instance", None)
                    if callable(clear_instance):
                        clear_instance()
                except Exception as exc:
                    if shutdown_failure is None:
                        shutdown_failure = IsaacRuntimeFailure(
                            f"Isaac world shutdown failed: {exc}", stage="shutdown"
                        )
            # Isaac 3's SimulationApp.close() terminates the Python process.
            # Persist caller-owned evidence while the interpreter is still live.
            if self._app is not None and self._before_app_close is not None:
                self._before_app_close(result, original_failure or shutdown_failure)
            try:
                if self._app is not None:
                    self._app.close()
            except Exception as exc:
                if shutdown_failure is None:
                    shutdown_failure = IsaacRuntimeFailure(
                        f"Isaac shutdown failed: {exc}", stage="shutdown"
                    )
            if shutdown_failure is not None and original_failure is None:
                raise shutdown_failure

    def _run(self, paths: ArtifactPaths) -> IsaacRuntimeResult:
        if self.isaac_python is None:
            raise IsaacProbeError(
                "Isaac3Runtime requires an explicit Isaac Python executable"
            )
        from omni.isaac.kit import SimulationApp

        self._app = SimulationApp({"headless": False})

        import math

        import omni.kit.commands
        import omni.usd
        from omni.importer.urdf import _urdf
        from omni.isaac.core.articulations import Articulation
        from omni.isaac.core.utils.stage import add_reference_to_stage
        from omni.isaac.core.utils.types import ArticulationAction
        from omni.isaac.core.world import World
        from omni.kit.viewport.utility import (
            capture_viewport_to_file,
            get_active_viewport,
        )
        from pxr import PhysxSchema, UsdPhysics

        environment = _observed_environment(self.isaac_python)
        validate_environment(environment)
        validate_environment_version(environment.semantic_version)
        partial_result = self._remember_partial(_partial_result(environment, ()))

        _urdf.acquire_urdf_interface()
        import_config = _urdf.ImportConfig()
        import_config.merge_fixed_joints = False
        import_config.convex_decomp = False
        import_config.fix_base = True
        import_config.make_default_prim = True
        import_config.self_collision = False
        import_config.create_physics_scene = True
        import_config.import_inertia_tensor = True
        import_config.default_drive_strength = 200.0
        import_config.default_position_drive_damping = 16.6
        import_config.default_drive_type = (
            _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
        )
        import_config.distance_scale = 1.0
        import_config.density = 0.0

        parsed, robot_model = omni.kit.commands.execute(
            "URDFParseFile",
            urdf_path=str(paths.robot_urdf),
            import_config=import_config,
        )
        if not parsed:
            raise IsaacRuntimeFailure(
                "Isaac URDF parser rejected robot.urdf",
                stage="import",
                partial_result=partial_result,
            )
        pre_import_events = _required_joint_events(robot_model)

        imported, _prim_path = omni.kit.commands.execute(
            "URDFParseAndImportFile",
            urdf_path=str(paths.robot_urdf),
            import_config=import_config,
            dest_path=str(paths.generated_usd),
        )
        importer_events = pre_import_events + _log_events(paths)
        self._latest_importer_events = importer_events
        classified = classify_importer_events(importer_events)
        try:
            require_no_blocking_events(classified)
        except ImporterContractError as exc:
            partial_result = self._remember_partial(
                _partial_result(environment, exc.events), exc.events
            )
            raise IsaacRuntimeFailure(
                str(exc),
                stage="import",
                partial_result=partial_result,
                importer_events=exc.events,
            ) from exc
        partial_result = self._remember_partial(
            _partial_result(environment, classified), classified
        )

        if not imported or not paths.generated_usd.is_file():
            raise IsaacRuntimeFailure(
                "Isaac did not generate the expected USD file",
                stage="import",
                partial_result=partial_result,
                importer_events=classified,
            )

        world = World(
            stage_units_in_meters=1.0,
            physics_dt=1.0 / 60.0,
            rendering_dt=1.0 / 60.0,
        )
        self._world = world
        world.scene.add_default_ground_plane()
        robot_prim_path = "/lynsense_robot_1"
        add_reference_to_stage(str(paths.generated_usd), robot_prim_path)

        stage = omni.usd.get_context().get_stage()
        joint_prims: dict[str, tuple[str, object]] = {}
        loaded_joint_name_set: set[str] = set()
        loaded_link_name_set: set[str] = set()
        robot_prefix = f"{robot_prim_path}/"
        for prim in stage.Traverse():
            prim_path = str(prim.GetPath())
            if prim_path != robot_prim_path and not prim_path.startswith(robot_prefix):
                continue
            name = str(prim.GetName())
            if prim.IsA(UsdPhysics.Joint):
                loaded_joint_name_set.add(name)
                joint_prims[name] = (prim_path, prim)
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                loaded_link_name_set.add(name)

        loaded_joint_names = tuple(sorted(loaded_joint_name_set))
        loaded_link_names = tuple(sorted(loaded_link_name_set))
        partial_result = self._remember_partial(
            _partial_result(
                environment,
                classified,
                loaded_joint_names=loaded_joint_names,
                loaded_link_names=loaded_link_names,
            ),
            classified,
        )
        for name in (*REQUIRED_JOINT_NAMES, *REQUIRED_GRIPPER_JOINT_NAMES):
            joint_entry = joint_prims.get(name)
            if joint_entry is None:
                raise IsaacRuntimeFailure(
                    f"required USD joint {name!r} is absent under {robot_prim_path}",
                    stage="import",
                    partial_result=partial_result,
                    importer_events=classified,
                )
            joint_path, joint_prim = joint_entry
            if name not in {"left_joint1", "right_joint1"}:
                continue
            drive = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
            drive.GetMaxForceAttr().Set(200.0)
            drive.GetStiffnessAttr().Set(self.drive_gains.stiffness)
            drive.GetDampingAttr().Set(self.drive_gains.damping)
            physx_joint = PhysxSchema.PhysxJointAPI.Apply(joint_prim)
            physx_joint.GetMaxJointVelocityAttr().Set(math.degrees(0.314))
            if (
                drive.GetMaxForceAttr().Get() != 200.0
                or drive.GetStiffnessAttr().Get() != self.drive_gains.stiffness
                or drive.GetDampingAttr().Get() != self.drive_gains.damping
                or not math.isclose(
                    physx_joint.GetMaxJointVelocityAttr().Get(),
                    math.degrees(0.314),
                    rel_tol=0.0,
                    abs_tol=1e-5,
                )
            ):
                raise IsaacRuntimeFailure(
                    f"drive values did not round-trip for {joint_path}",
                    stage="drive",
                    partial_result=partial_result,
                    importer_events=classified,
                )
        stage.Save()

        robot = Articulation(prim_path=robot_prim_path)
        world.scene.add(robot)
        world.reset()

        missing_gripper_joints = set(REQUIRED_GRIPPER_JOINT_NAMES) - set(
            loaded_joint_names
        )
        missing_gripper_links = set(REQUIRED_GRIPPER_LINK_NAMES) - set(
            loaded_link_names
        )
        missing_links = set(REQUIRED_LINK_NAMES) - set(loaded_link_names)
        dof_names = tuple(str(name) for name in robot.dof_properties["names"])
        missing_articulation = set(REQUIRED_JOINT_NAMES) - set(dof_names)
        if (
            missing_gripper_joints
            or missing_gripper_links
            or missing_links
            or missing_articulation
        ):
            raise IsaacRuntimeFailure(
                "generated USD is missing required robot structure",
                stage="articulation",
                partial_result=partial_result,
                importer_events=classified,
            )

        indices: dict[str, int] = {}
        plans: dict[str, JointActionPlan] = {}
        for name in ("left_joint1", "right_joint1"):
            index = dof_names.index(name)
            indices[name] = index
            positions = robot.get_joint_positions()
            limits = robot.dof_properties["lower"], robot.dof_properties["upper"]
            plans[name] = plan_joint_action(
                name,
                float(positions[index]),
                float(limits[0][index]),
                float(limits[1][index]),
            )
        drive_configuration = DriveConfiguration(
            gains=self.drive_gains,
            limits=DriveLimits(),
            verified=True,
        )
        trajectories = dict.fromkeys(plans, ())
        base_motion: tuple[TransformSample, ...] = ()
        partial_result = self._remember_partial(
            _partial_result(
                environment,
                classified,
                articulation_joint_names=dof_names,
                loaded_joint_names=loaded_joint_names,
                loaded_link_names=loaded_link_names,
                action_plans=plans,
                drive_configuration=drive_configuration,
                trajectories=trajectories,
                base_motion=base_motion,
            ),
            classified,
        )

        def capture(name: str) -> None:
            viewport = get_active_viewport()
            output = paths.run_directory / name
            capture_task = capture_viewport_to_file(
                viewport,
                file_path=str(output.resolve()),
            )
            _await_isaac_task(capture_task, output)

        def add_sample(step: int) -> None:
            nonlocal base_motion
            positions = robot.get_joint_positions()
            velocities = robot.get_joint_velocities()
            translation, rotation = robot.get_local_pose()
            quaternion = (
                float(rotation[1]),
                float(rotation[2]),
                float(rotation[3]),
                float(rotation[0]),
            )
            base_motion = (
                *base_motion,
                TransformSample(
                    step / 60.0,
                    tuple(float(value) for value in translation),
                    quaternion,
                ),
            )
            latest = {}
            for name, index in indices.items():
                sample = JointSample(
                    step / 60.0,
                    float(positions[index]),
                    float(velocities[index]),
                )
                trajectories[name] = (*trajectories[name], sample)
                latest[name] = sample
            validate_step_state(plans, latest, base_motion)

        try:
            add_sample(0)
            capture("scene-initial.png")
            action = ArticulationAction(
                joint_positions=[
                    plans["left_joint1"].target_position,
                    plans["right_joint1"].target_position,
                ],
                joint_velocities=[0.314, 0.314],
                joint_indices=[
                    indices["left_joint1"],
                    indices["right_joint1"],
                ],
            )
            action_captured = False
            for step in range(1, 601):
                robot.apply_action(action)
                world.step(render=True)
                add_sample(step)
                if not action_captured and (
                    all(
                        trajectories[name][-1].position_rad
                        - plans[name].initial_position
                        >= 0.05
                        for name in plans
                    )
                    or step == 300
                ):
                    capture("scene-action.png")
                    action_captured = True
            capture("scene-final.png")
            evaluate_joint_action(trajectories, plans)
            evaluate_base_motion(base_motion)
        except Exception as exc:
            partial_result = self._remember_partial(
                _partial_result(
                    environment,
                    classified,
                    articulation_joint_names=dof_names,
                    loaded_joint_names=loaded_joint_names,
                    loaded_link_names=loaded_link_names,
                    action_plans=plans,
                    drive_configuration=drive_configuration,
                    trajectories=trajectories,
                    base_motion=base_motion,
                ),
                classified,
            )
            raise IsaacRuntimeFailure(
                str(exc), stage="action", partial_result=partial_result
            ) from exc

        return IsaacRuntimeResult(
            environment=environment,
            importer_events=classified,
            articulation_joint_names=dof_names,
            loaded_joint_names=loaded_joint_names,
            loaded_link_names=loaded_link_names,
            action_plans=plans,
            drive_configuration=drive_configuration,
            trajectories=trajectories,
            base_motion=base_motion,
        )
