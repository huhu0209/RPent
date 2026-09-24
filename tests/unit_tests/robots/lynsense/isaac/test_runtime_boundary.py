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

from pathlib import Path
from types import SimpleNamespace

import pytest

from robots.lynsense.isaac import runtime
from robots.lynsense.isaac.import_contract import ImporterEvent
from robots.lynsense.isaac.runtime import Isaac3Runtime, IsaacRuntimeFailure


def test_isaac_imports_happen_only_after_simulation_app_starts() -> None:
    source = Path(runtime.__file__).read_text(encoding="utf-8")
    app_start = source.index("SimulationApp(")
    module_imports = [
        source.index(text)
        for text in (
            "from omni.importer.urdf import _urdf",
            "import omni.kit.commands",
            "from omni.isaac.core.world import World",
        )
    ]
    assert all(app_start < index for index in module_imports)


def test_isaac_3_runtime_uses_company_importer_and_fixed_base() -> None:
    source = Path(runtime.__file__).read_text(encoding="utf-8")
    assert 'SimulationApp({"headless": False})' in source
    assert "omni.importer.urdf" in source
    assert "config.fix_base = True" in source
    assert "config.merge_fixed_joints = False" in source
    assert "JOINT_DRIVE_POSITION" in source
    assert "config.default_drive_strength = 200.0" in source
    assert "config.default_position_drive_damping = 16.6" in source
    assert "GetMaxForceAttr().Set(200.0)" in source
    assert "GetMaxJointVelocityAttr().Set(math.degrees(0.314))" in source
    assert "drive_gains=" in source
    assert "capture_viewport_to_file(" in source
    assert "wait_for_result()" in source
    assert "capture_screenshot" not in source
    assert "self._app.close()" in source


def test_required_joint_limits_use_the_isaac_3_nested_limit_api() -> None:
    joints = {
        name: SimpleNamespace(
            name=name,
            limit=SimpleNamespace(lower=-1.0, upper=1.0),
        )
        for name in runtime.REQUIRED_JOINT_NAMES
    }
    robot_model = SimpleNamespace(joints=joints)

    assert runtime._required_joint_events(robot_model) == ()


def test_importer_log_ignores_startup_noise_and_massless_camera_links() -> None:
    assert (
        runtime._log_event("[Warning] [gpu.foundation.plugin] IOMMU is enabled.")
        is None
    )
    assert (
        runtime._log_event(
            "[Warning] [omni.importer.urdf] "
            "Link head_camera_color_frame has no colliders"
        )
        is None
    )
    event = runtime._log_event(
        "[Warning] [omni.importer.urdf] camera sensor was dropped"
    )
    assert event is not None
    assert event.category == "dropped_camera"


def test_importer_log_is_gated_after_import_before_stage_or_world() -> None:
    source = Path(runtime.__file__).read_text(encoding="utf-8")
    import_call = source.index('"URDFParseAndImportFile"')
    event_gate = source.index(
        "importer_events = pre_import_events + _log_events(paths)"
    )
    stage_open = source.index("stage = omni.usd.get_context().get_stage()")
    world_open = source.index("world = World(")
    reference = source.index("add_reference_to_stage(")
    stage_traverse = source.index("for prim in stage.Traverse():")
    assert (
        import_call < event_gate < world_open < reference < stage_open < stage_traverse
    )


def test_usd_inventories_are_typed_not_all_traversed_names() -> None:
    source = Path(runtime.__file__).read_text(encoding="utf-8")
    assert "prim.IsA(UsdPhysics.Joint)" in source
    assert "prim.HasAPI(UsdPhysics.RigidBodyAPI)" in source
    assert "loaded_link_names = loaded_joint_names" not in source


def test_viewport_capture_helper_waits_for_result_protocol(tmp_path: Path) -> None:
    class FakeCaptureHelper:
        completed = False

        async def wait_for_result(self) -> None:
            self.completed = True

    output = tmp_path / "scene-initial.png"
    output.write_bytes(b"png")
    helper = FakeCaptureHelper()
    runtime._await_isaac_task(helper, output)
    assert helper.completed is True
    fallback_completed = False

    async def fallback_task() -> None:
        nonlocal fallback_completed
        fallback_completed = True

    runtime._await_isaac_task(fallback_task(), output)
    assert fallback_completed is True


def test_runtime_wraps_raw_failure_with_latest_partial_evidence() -> None:
    event = ImporterEvent(
        category="material_change",
        severity="warning",
        source="isaac-log",
        message="material changed",
    )
    instance = Isaac3Runtime(isaac_python=Path("/opt/isaac/python.sh"))

    def fail(_paths: object) -> object:
        partial = runtime._partial_result(
            None,
            (event,),
            executable=Path("/opt/isaac/python.sh"),
        )
        instance._remember_partial(partial, (event,))
        raise RuntimeError("raw runtime failure")

    instance._run = fail  # type: ignore[method-assign]
    with pytest.raises(IsaacRuntimeFailure) as raised:
        instance.run(SimpleNamespace())  # type: ignore[arg-type]

    assert str(raised.value) == "raw runtime failure"
    assert raised.value.partial_result is not None
    assert raised.value.importer_events == (event,)


def test_runtime_cleanup_attempts_clear_and_app_close_after_stop_failure() -> None:
    calls: list[str] = []

    class World:
        def stop(self) -> None:
            calls.append("stop")
            raise RuntimeError("stop failed")

        def clear_instance(self) -> None:
            calls.append("clear")

    class App:
        def close(self) -> None:
            calls.append("close")

    instance = Isaac3Runtime(isaac_python=Path("/opt/isaac/python.sh"))
    instance._world = World()
    instance._app = App()

    def fail(_paths: object) -> object:
        raise RuntimeError("original failure")

    instance._run = fail  # type: ignore[method-assign]
    with pytest.raises(IsaacRuntimeFailure) as raised:
        instance.run(SimpleNamespace())  # type: ignore[arg-type]

    assert str(raised.value) == "original failure"
    assert raised.value.stage == "runtime"
    assert calls == ["stop", "clear", "close"]


def test_runtime_reports_shutdown_failure_when_run_succeeds() -> None:
    class World:
        def stop(self) -> None:
            raise RuntimeError("stop failed")

        def clear_instance(self) -> None:
            pass

    instance = Isaac3Runtime(isaac_python=Path("/opt/isaac/python.sh"))
    instance._world = World()
    instance._run = lambda _paths: object()  # type: ignore[method-assign]

    with pytest.raises(IsaacRuntimeFailure) as raised:
        instance.run(SimpleNamespace())  # type: ignore[arg-type]

    assert raised.value.stage == "shutdown"
    assert "stop failed" in str(raised.value)


def test_runtime_calls_evidence_hook_before_terminal_app_close() -> None:
    events: list[str] = []

    class TerminalApp:
        def close(self) -> None:
            events.append("close")
            raise SystemExit(0)

    def run(_paths: object) -> object:
        instance._app = TerminalApp()
        return object()

    instance = Isaac3Runtime(isaac_python=Path("/opt/isaac/python.sh"))
    instance._run = run  # type: ignore[method-assign]

    def persist(result: object | None, failure: IsaacRuntimeFailure | None) -> None:
        assert result is not None
        assert failure is None
        events.append("persist")

    instance._before_app_close = persist
    with pytest.raises(SystemExit):
        instance.run(SimpleNamespace())
    assert events == ["persist", "close"]
