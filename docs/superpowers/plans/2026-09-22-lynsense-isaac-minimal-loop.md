# Lynsense Isaac Sim Minimal Loop Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the approved company Isaac Sim 3-era URDF-to-USD inspection and bounded dual-arm action loop while preserving the Webots baseline.

**Architecture:** Keep all Isaac work in a new `robots.lynsense.isaac` package beside, not inside, the Webots simulation package. The offline contract layer safely stages and inspects the company archive, evaluates trajectories and evidence, and writes artifacts; the runtime boundary uses the Isaac 2023.1-era `omni.importer.urdf` API only inside an explicitly selected company Isaac Python process. No Isaac installation is required for unit tests, and a missing explicit runtime is an explicit skip rather than a pass.

**Tech Stack:** Python 3.10+, standard-library `tarfile`/`xml.etree.ElementTree`, `numpy` and `imageio` already present in RPent, pytest, Isaac Sim `2023.1.*` with `omni.importer.urdf` and `omni.isaac.core`.

**Spec:** `docs/superpowers/specs/2026-09-22-lynsense-isaac-minimal-loop-design.md`

## Global Constraints

- The passing Isaac environment allowlist is exactly Isaac Sim `2023.1.*`; any other version fails the environment gate.
- A passing Isaac run requires an explicitly supplied company Isaac Python executable; discovery may only produce diagnostics.
- Preserve all Webots behavior and suites; do not modify `robots/lynsense/simulation` except for documentation-free test compatibility if a shared contract truly requires it.
- Do not connect to the real robot, SSH state, ROS Domain 3, robot workspace paths, or `.env.lynsense`.
- Do not add ROS, `py_trees`, RPent policy, perception, localization, navigation, or box-task logic to this milestone.
- Keep extracted company assets, generated USD, screenshots, logs, summaries, and visual review files under `.artifacts/lynsense-isaac/`; do not commit or redistribute them.
- The fixed robot root is `base_link`; gravity is `-9.81 m/s^2` on world Z.
- Required drivable joints are `connector_joint`, `left_joint1` through `left_joint6`, and `right_joint1` through `right_joint6`.
- Check left/right CTAG gripper subtrees structurally; do not require every mimic joint to be independently drivable.
- The action targets are `initial + 0.1 rad` for both `left_joint1` and `right_joint1`; reject an out-of-limit target before actuation.
- Use a recorded symmetric position-drive configuration, URDF effort limit `200 N*m`, probe speed limit `0.314 rad/s`, and imported damping/friction.
- During action, require finite state at every physics step, at least `0.05 rad` progress, final-window target error at most `0.02 rad`, final-window speed at most `0.05 rad/s`, overshoot at most `0.02 rad`, and completion within 10 simulated seconds.
- The fixed `base_link` may move no more than `0.001 m` or `0.5 deg`.
- Required screenshots are `scene-initial.png`, `scene-action.png`, and `scene-final.png`; they must be nonblank, while company-model fidelity requires `visual-review.json` human acceptance.
- Reject absolute paths, parent traversal, links, special files, over 100,000 entries, or over 2 GiB uncompressed content during archive extraction.
- Do not commit, push, clean, reset, or modify unrelated uncommitted work. Commits require separate explicit authorization.

---

### Task 1: Isaac Contracts, Version Gate, And Artifact Store

**Files:**
- Create: `robots/lynsense/isaac/__init__.py`
- Create: `robots/lynsense/isaac/contracts.py`
- Create: `robots/lynsense/isaac/artifacts.py`
- Test: `tests/unit_tests/robots/lynsense/isaac/test_contracts.py`
- Test: `tests/unit_tests/robots/lynsense/isaac/test_artifacts.py`

**Interfaces:**
- Consumes: none.
- Produces: `IsaacProbeError`, `IsaacEnvironment`, `environment_from_dict`, `validate_environment`, `validate_environment_version`, `VisualReview`, `validate_visual_review`, `ArtifactPaths`, `create_artifact_paths`, `atomic_write_json`, and `load_json_object`.

- [ ] **Step 1: Write failing contract tests**

Create the package tests first. These tests use no Isaac modules and no company archive.

```python
from __future__ import annotations

import json
from pathlib import Path

import pytest

from robots.lynsense.isaac.contracts import (
    IsaacEnvironment,
    IsaacProbeError,
    environment_from_dict,
    validate_environment,
    validate_environment_version,
    validate_visual_review,
)
from robots.lynsense.isaac.artifacts import (
    ArtifactPaths,
    atomic_write_json,
    create_artifact_paths,
    load_json_object,
)


def _environment(version: str = "2023.1.1") -> IsaacEnvironment:
    return IsaacEnvironment(
        executable=Path("/opt/isaac-sim/python.sh"),
        semantic_version=version,
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
    )


@pytest.mark.parametrize("version", ["2023.1.0", "2023.1.1", "2023.1.9"])
def test_environment_accepts_only_the_company_2023_1_family(version: str) -> None:
    assert validate_environment_version(version) is None


@pytest.mark.parametrize(
    "version",
    ["2022.2.1", "2023.2.0", "4.2.0", "2023.1", "2023.1.rc", "latest"],
)
def test_environment_rejects_unapproved_versions(version: str) -> None:
    with pytest.raises(IsaacProbeError, match="2023.1"):
        validate_environment_version(version)


def test_environment_serializes_without_secret_values() -> None:
    document = _environment().to_dict()
    assert document == {
        "executable": "/opt/isaac-sim/python.sh",
        "semantic_version": "2023.1.1",
        "kit_version": "106.0.0",
        "python_version": "3.10.14",
        "renderer": "RTX",
        "run_mode": "desktop",
        "capabilities": [
            "articulation",
            "physx_joint_limits",
            "urdf_importer",
            "viewport_screenshot",
        ],
    }
    json.dumps(document, allow_nan=False)


def test_environment_json_round_trip_rejects_extra_fields() -> None:
    document = _environment().to_dict()
    assert environment_from_dict(document) == _environment()
    document["license_token"] = "secret"
    with pytest.raises(IsaacProbeError, match="fields"):
        environment_from_dict(document)


def test_environment_metadata_and_capabilities_must_be_complete() -> None:
    complete = _environment()
    assert validate_environment(complete) is None
    for field in (
        "executable", "semantic_version", "kit_version",
        "python_version", "renderer", "run_mode",
    ):
        changes = {field: Path("") if field == "executable" else ""}
        with pytest.raises(IsaacProbeError, match=field):
            validate_environment(
                IsaacEnvironment(**{**complete.__dict__, **changes})
            )
    with pytest.raises(IsaacProbeError, match="capabilities"):
        validate_environment(
            IsaacEnvironment(
                **{**complete.__dict__, "capabilities": frozenset()}
            )
        )


def _visual_review(**changes: object) -> dict[str, object]:
    document: dict[str, object] = {
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
        "reviewer": "company-reviewer",
        "reviewed_at": "2026-09-22T10:30:00Z",
        "notes": "Company model matches the supplied URDF.",
    }
    document.update(changes)
    return document


def test_visual_review_accepts_the_complete_contract() -> None:
    assert validate_visual_review(_visual_review()) is None


def test_visual_review_requires_all_visual_and_metadata_fields() -> None:
    invalid = [
        _visual_review(status="passed"),
        _visual_review(screenshots=["scene-initial.png"]),
        _visual_review(visible_subsystems=["chassis"]),
        _visual_review(reviewer=""),
        _visual_review(reviewed_at="not-a-timestamp"),
        _visual_review(status="rejected", notes=""),
    ]
    for document in invalid:
        with pytest.raises(IsaacProbeError):
            validate_visual_review(document)


def test_rejected_visual_review_requires_reason() -> None:
    document = _visual_review(
        status="rejected",
        notes="The left gripper geometry does not match.",
    )
    assert validate_visual_review(document) is None
```

```python
from __future__ import annotations

import json
from pathlib import Path

import pytest

from robots.lynsense.isaac.artifacts import (
    ArtifactPaths,
    atomic_write_json,
    create_artifact_paths,
    load_json_object,
)


def test_artifact_paths_are_complete_and_run_scoped(tmp_path: Path) -> None:
    paths = create_artifact_paths(tmp_path, "probe-run")
    assert paths.run_directory == tmp_path / "probe-run"
    assert paths.extracted_root == tmp_path / "probe-run" / "extracted"
    assert paths.robot_urdf == (
        tmp_path / "probe-run" / "extracted" / "URDF_robot_1" / "robot.urdf"
    )
    assert paths.generated_usd == tmp_path / "probe-run" / "robot_1.usd"
    assert paths.environment_json.name == "environment.json"
    assert paths.technical_summary_json.name == "technical-summary.json"
    assert paths.visual_review_json.name == "visual-review.json"
    assert [path.name for path in paths.screenshots] == [
        "scene-initial.png",
        "scene-action.png",
        "scene-final.png",
    ]
    assert paths.isaac_log.name == "isaac.log"


def test_atomic_json_replaces_only_after_success(tmp_path: Path) -> None:
    target = tmp_path / "summary.json"
    atomic_write_json(target, {"ok": True})
    assert json.loads(target.read_text(encoding="utf-8")) == {"ok": True}
    assert list(tmp_path.glob(".summary.json.tmp-*")) == []

    with pytest.raises(ValueError, match="non-finite"):
        atomic_write_json(target, {"value": float("nan")})
    assert json.loads(target.read_text(encoding="utf-8")) == {"ok": True}


def test_json_object_loader_rejects_non_objects_and_invalid_json(
    tmp_path: Path,
) -> None:
    valid = tmp_path / "valid.json"
    valid.write_text(json.dumps({"value": 1}), encoding="utf-8")
    assert load_json_object(valid) == {"value": 1}

    array = tmp_path / "array.json"
    array.write_text("[]", encoding="utf-8")
    malformed = tmp_path / "malformed.json"
    malformed.write_text("{", encoding="utf-8")
    for path in (array, malformed):
        with pytest.raises(ValueError):
            load_json_object(path)
```

- [ ] **Step 2: Run tests and confirm import failures**

Run:

```bash
.venv/bin/pytest -q \
  tests/unit_tests/robots/lynsense/isaac/test_contracts.py \
  tests/unit_tests/robots/lynsense/isaac/test_artifacts.py
```

Expected: both files fail because `robots.lynsense.isaac.contracts` and
`robots.lynsense.isaac.artifacts` do not exist.

- [ ] **Step 3: Implement the package contracts**

Create `robots/lynsense/isaac/__init__.py` with only the package docstring:

```python
"""Offline and Isaac Sim probes for the Lynsense company robot model."""
```

Create `robots/lynsense/isaac/contracts.py` with these exact public constants and interfaces:

```python
from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any


SUPPORTED_ISAAC_VERSION = re.compile(r"^2023\.1\.\d+$")
SCREENSHOT_NAMES = (
    "scene-initial.png",
    "scene-action.png",
    "scene-final.png",
)
VISIBLE_SUBSYSTEMS = (
    "chassis",
    "dual_arms",
    "left_gripper",
    "right_gripper",
    "head_camera",
    "wrist_cameras",
)


class IsaacProbeError(RuntimeError):
    """A Lynsense Isaac probe contract was violated."""


@dataclass(frozen=True)
class IsaacEnvironment:
    executable: Path
    semantic_version: str
    kit_version: str
    python_version: str
    renderer: str
    run_mode: str = "desktop"
    capabilities: frozenset[str] = frozenset()

    def to_dict(self) -> dict[str, str]:
        return {
            "executable": str(self.executable),
            "semantic_version": self.semantic_version,
            "kit_version": self.kit_version,
            "python_version": self.python_version,
            "renderer": self.renderer,
            "run_mode": self.run_mode,
            "capabilities": sorted(self.capabilities),
        }


def validate_environment_version(version: str) -> None:
    if not SUPPORTED_ISAAC_VERSION.fullmatch(version):
        raise IsaacProbeError(
            f"unsupported Isaac Sim version {version!r}; expected 2023.1.*"
        )


def validate_environment(environment: IsaacEnvironment) -> None:
    """Require non-empty runtime metadata and all probe capabilities."""

Required capabilities are `urdf_importer`, `articulation`, `viewport_screenshot`,
and `physx_joint_limits`. `validate_environment` also calls
`validate_environment_version`, rejects an executable with an empty name, and
rejects empty Kit, Python, renderer, or run-mode strings. It does not claim a
capability unless the Isaac runtime successfully imported and exercised that
API during the probe.

Implement `environment_from_dict(document: Any) -> IsaacEnvironment` with exact
keys matching `IsaacEnvironment.to_dict()`. Convert `executable` to `Path`,
capabilities to a `frozenset`, reject extra or missing fields, and delegate the
result to `validate_environment`.


@dataclass(frozen=True)
class VisualReview:
    status: str
    screenshots: tuple[str, ...]
    visible_subsystems: tuple[str, ...]
    reviewer: str
    reviewed_at: str
    notes: str
```

Implement `validate_visual_review(document: Any) -> None` with these rules:

- The top-level keys are exactly `schema_version`, `status`, `screenshots`, `visible_subsystems`, `reviewer`, `reviewed_at`, and `notes`.
- `schema_version` is integer `1`.
- `status` is `accepted` or `rejected`.
- `screenshots` equals `list(SCREENSHOT_NAMES)`.
- `visible_subsystems` equals `list(VISIBLE_SUBSYSTEMS)`.
- `reviewer` and `notes` are non-empty strings.
- `reviewed_at` matches `^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z$`.
- A rejected review must explain the reason in `notes`.

Create `robots/lynsense/isaac/artifacts.py`:

```python
from __future__ import annotations

import json
import os
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from robots.lynsense.isaac.contracts import SCREENSHOT_NAMES


@dataclass(frozen=True)
class ArtifactPaths:
    run_directory: Path
    extracted_root: Path
    robot_urdf: Path
    generated_usd: Path
    environment_json: Path
    technical_summary_json: Path
    visual_review_json: Path
    isaac_log: Path
    screenshots: tuple[Path, ...]


def create_artifact_paths(root: Path, run_id: str) -> ArtifactPaths:
    run_directory = root / run_id
    extracted_root = run_directory / "extracted"
    return ArtifactPaths(
        run_directory=run_directory,
        extracted_root=extracted_root,
        robot_urdf=extracted_root / "URDF_robot_1" / "robot.urdf",
        generated_usd=run_directory / "robot_1.usd",
        environment_json=run_directory / "environment.json",
        technical_summary_json=run_directory / "technical-summary.json",
        visual_review_json=run_directory / "visual-review.json",
        isaac_log=run_directory / "isaac.log",
        screenshots=tuple(run_directory / name for name in SCREENSHOT_NAMES),
    )
```

Implement `atomic_write_json(path: Path, value: Any) -> None` by serializing with
`json.dumps(value, indent=2, sort_keys=True, allow_nan=False)`, writing a
same-directory temporary file, flushing, calling `os.fsync`, replacing the
target, and removing the temporary file in a `finally` block. Implement
`load_json_object(path: Path) -> dict[str, Any]` with UTF-8 JSON parsing that
rejects unreadable files, malformed JSON, and non-object roots.

- [ ] **Step 4: Run focused tests**

Repeat the Step 2 command. Expected: all tests pass.

- [ ] **Step 5: Review the task diff**

Run:

```bash
git diff --check
git status --short robots/lynsense/isaac \
  tests/unit_tests/robots/lynsense/isaac
```

Do not commit; commit authorization has not been given.

### Task 2: Safe Archive Staging And Offline URDF Contract

**Files:**
- Create: `robots/lynsense/isaac/archive.py`
- Create: `robots/lynsense/isaac/urdf_contract.py`
- Create: `tests/unit_tests/robots/lynsense/isaac/fixtures.py`
- Create: `tests/unit_tests/robots/lynsense/isaac/conftest.py`
- Test: `tests/unit_tests/robots/lynsense/isaac/test_archive.py`
- Test: `tests/unit_tests/robots/lynsense/isaac/test_urdf_contract.py`

**Interfaces:**
- Consumes: `IsaacProbeError`, `atomic_write_json`.
- Produces: `ArchiveInventory`, `extract_archive`, `sha256_file`, `UrdfJoint`, `UrdfMesh`, `UrdfInventory`, `inspect_urdf`, `REQUIRED_JOINT_NAMES`, `REQUIRED_LINK_NAMES`, `REQUIRED_GRIPPER_JOINT_NAMES`, `REQUIRED_GRIPPER_LINK_NAMES`, `resolve_mesh_reference`, and the test helper `write_valid_probe_archive`.

- [ ] **Step 1: Write failing archive safety tests**

```python
from __future__ import annotations

import io
import tarfile
from pathlib import Path

import pytest

from robots.lynsense.isaac.archive import ArchiveInventory, extract_archive


def _write_archive(path: Path, members: list[tuple[str, bytes]]) -> None:
    with tarfile.open(path, "w:gz") as archive:
        for name, payload in members:
            info = tarfile.TarInfo(name)
            if name.endswith("/"):
                info.type = tarfile.DIRTYPE
                info.mode = 0o700
                archive.addfile(info)
                continue
            info.size = len(payload)
            archive.addfile(info, io.BytesIO(payload))


def test_archive_extracts_expected_robot_and_records_inventory(tmp_path: Path) -> None:
    archive = tmp_path / "robot.tar.gz"
    _write_archive(
        archive,
        [
            ("URDF_robot_1/", b""),
            ("URDF_robot_1/robot.urdf", b"<robot/>"),
            ("URDF_robot_1/pkg/mesh.stl", b"mesh"),
        ],
    )
    destination = tmp_path / "out"
    inventory = extract_archive(archive, destination)

    assert isinstance(inventory, ArchiveInventory)
    assert inventory.file_count == 2
    assert inventory.total_bytes == 12
    assert inventory.archive_digest.startswith("sha256:")
    assert inventory.robot_urdf_sha256.startswith("sha256:")
    assert inventory.robot_urdf == destination / "URDF_robot_1" / "robot.urdf"
    assert inventory.robot_urdf.read_bytes() == b"<robot/>"


def test_archive_rejects_missing_robot_urdf(tmp_path: Path) -> None:
    archive = tmp_path / "robot.tar.gz"
    _write_archive(archive, [("URDF_robot_1/other.txt", b"x")])
    with pytest.raises(Exception, match="robot.urdf"):
        extract_archive(archive, tmp_path / "out")


def _special_archive(path: Path, name: str, kind: str) -> None:
    with tarfile.open(path, "w:gz") as archive:
        info = tarfile.TarInfo(name)
        info.type = {
            "symlink": tarfile.SYMTYPE,
            "hardlink": tarfile.LNKTYPE,
            "device": tarfile.CHRTYPE,
        }[kind]
        info.linkname = "/outside" if kind == "symlink" else "URDF_robot_1/other"
        archive.addfile(info)


@pytest.mark.parametrize("kind", ["symlink", "hardlink", "device"])
def test_archive_rejects_special_entries(tmp_path: Path, kind: str) -> None:
    archive = tmp_path / "robot.tar.gz"
    _special_archive(archive, f"URDF_robot_1/{kind}", kind)
    with pytest.raises(Exception, match=kind):
        extract_archive(archive, tmp_path / "out")


def test_archive_rejects_parent_and_absolute_paths(tmp_path: Path) -> None:
    for name in ("../escape.txt", "/absolute.txt", "URDF_robot_1/a\\b.txt"):
        archive = tmp_path / f"{len(name)}.tar.gz"
        _write_archive(archive, [(name, b"x")])
        with pytest.raises(Exception, match="unsafe"):
            extract_archive(archive, tmp_path / "out")


def test_archive_rejects_entry_and_size_limits(tmp_path: Path, monkeypatch) -> None:
    from robots.lynsense.isaac import archive as archive_module

    archive = tmp_path / "robot.tar.gz"
    _write_archive(archive, [("URDF_robot_1/robot.urdf", b"<robot/>")])
    monkeypatch.setattr(archive_module, "MAX_ARCHIVE_ENTRIES", 1)
    with pytest.raises(Exception, match="entries"):
        extract_archive(archive, tmp_path / "out")

    monkeypatch.setattr(archive_module, "MAX_ARCHIVE_ENTRIES", 100_000)
    monkeypatch.setattr(archive_module, "MAX_ARCHIVE_ENTRIES", 100_000)
    with pytest.raises(Exception, match="2 GiB"):
        extract_archive(
            archive,
            tmp_path / "another-out",
            max_uncompressed_bytes=1,
        )
```

- [ ] **Step 2: Run archive tests and confirm failure**

Run:

```bash
.venv/bin/pytest -q tests/unit_tests/robots/lynsense/isaac/test_archive.py
```

Expected: FAIL because `robots.lynsense.isaac.archive` does not exist.

- [ ] **Step 3: Implement safe archive staging**

Create `robots/lynsense/isaac/archive.py`. Use constants `MAX_ARCHIVE_ENTRIES = 100_000` and `MAX_UNCOMPRESSED_BYTES = 2 * 1024**3`.

Required implementation behavior:

```python
@dataclass(frozen=True)
class ArchiveInventory:
    archive_path: Path
    robot_urdf: Path
    file_count: int
    total_bytes: int
    archive_digest: str
    robot_urdf_sha256: str
```

Implement
`extract_archive(archive: Path, destination: Path, *, max_uncompressed_bytes: int = MAX_UNCOMPRESSED_BYTES) -> ArchiveInventory`:

1. Reject a missing or non-file archive.
2. Open with `tarfile.open(archive, "r:gz")`.
3. Validate all members before creating any output.
4. Reject more than `MAX_ARCHIVE_ENTRIES`.
5. Require a positive integer `max_uncompressed_bytes` no greater than `MAX_UNCOMPRESSED_BYTES`, and reject cumulative regular-file sizes over that limit.
6. Reject backslashes, empty names, absolute POSIX paths, any `.` or `..` component, and non-relative names.
7. Accept only directory and regular-file members; reject symbolic links, hard links, FIFOs, character devices, block devices, and unknown types.
8. Resolve every proposed output under `destination.resolve()` using `Path.is_relative_to`.
9. Create directories with mode `0o700`, then copy each regular member through `tar.extractfile(member)` in chunks; use mode `0o600`.
10. Require `URDF_robot_1/robot.urdf` to be an extracted regular file.
11. Return the inventory with a `sha256:` archive digest from `sha256_file(archive)` and a separate `sha256:` digest of the extracted `robot.urdf`.

Use `ArchiveError(IsaacProbeError)` as the exception type. Keep all path logic in standard-library `pathlib.PurePosixPath`; do not invoke a shell.

- [ ] **Step 4: Write failing URDF contract tests**

The fixture must be a small synthetic URDF, not the company archive.

```python
from __future__ import annotations

from pathlib import Path

import pytest

from robots.lynsense.isaac.urdf_contract import (
    REQUIRED_JOINT_NAMES,
    inspect_urdf,
    resolve_mesh_reference,
)


def _joint(
    name: str,
    joint_type: str = "revolute",
    child_link: str | None = None,
) -> str:
    child = child_link or f"{name}_link"
    if joint_type == "fixed":
        return f'<joint name="{name}" type="fixed"><parent link="base_link"/><child link="{child}"/></joint>'
    return f'''<joint name="{name}" type="{joint_type}">
      <parent link="base_link"/><child link="{child}"/><axis xyz="0 0 1"/>
      <limit lower="-1.0" upper="1.0" effort="200" velocity="3.14"/>
      <dynamics damping="16.6" friction="9.6"/>
    </joint>'''


def _urdf(mesh_reference: str = "package://pkg/meshes/part.stl") -> str:
    gripper_names = (
        "left_R1", "left_R2", "left_R3", "left_L1", "left_L2", "left_L3",
        "right_R1", "right_R2", "right_R3", "right_L1", "right_L2", "right_L3",
    )
    joints = "\n".join(_joint(name) for name in REQUIRED_JOINT_NAMES)
    grippers = "\n".join(
        _joint(name, child_link=f"{name}_Link")
        for name in gripper_names
    )
    moving_link_names = tuple(
        f"{name}_Link" if name in gripper_names else f"{name}_link"
        for name in (*REQUIRED_JOINT_NAMES, *gripper_names)
    )
    moving_links = "\n".join(
        f'<link name="{name}"/>'
        for name in moving_link_names
    )
    sensor_links = (
        "head_camera_link",
        "left_camera_link",
        "right_camera_link",
        "left_force_sensor_link",
        "right_force_sensor_link",
    )
    sensor_joints = "\n".join(
        f'<joint name="{link.removesuffix("_link")}_fixture_joint" type="fixed">'
        f'<parent link="base_link"/><child link="{link}"/></joint>'
        for link in sensor_links
    )
    return f'''<robot name="fixture">
      <link name="base_link">
        <visual><geometry><mesh filename="{mesh_reference}"/></geometry></visual>
        <collision><geometry><mesh filename="{mesh_reference}"/></geometry></collision>
      </link>
      {moving_links}
      <link name="head_camera_link"/>
      <link name="left_camera_link"/>
      <link name="right_camera_link"/>
      <link name="left_force_sensor_link"/>
      <link name="right_force_sensor_link"/>
      {sensor_joints}
      {joints}
      {grippers}
    </robot>'''


def test_mesh_references_resolve_inside_extraction_root(tmp_path: Path) -> None:
    urdf = tmp_path / "URDF_robot_1" / "robot.urdf"
    urdf.parent.mkdir(parents=True)
    urdf.write_text(_urdf(), encoding="utf-8")
    mesh = tmp_path / "pkg" / "meshes" / "part.stl"
    mesh.parent.mkdir(parents=True)
    mesh.write_bytes(b"mesh")

    inventory = inspect_urdf(urdf)
    assert inventory.root_link == "base_link"
    assert inventory.required_joint_names == REQUIRED_JOINT_NAMES
    assert len(inventory.meshes) == 2
    assert inventory.meshes[0].resolved_path == mesh
    assert inventory.meshes[0].sha256.startswith("sha256:")
    assert inventory.to_dict()["root_link"] == "base_link"


def test_file_reference_resolves_relative_to_extraction_root(tmp_path: Path) -> None:
    urdf = tmp_path / "URDF_robot_1" / "robot.urdf"
    urdf.parent.mkdir(parents=True)
    urdf.write_text(_urdf("file://realsense2_description/meshes/d405.stl"), encoding="utf-8")
    mesh = tmp_path / "realsense2_description" / "meshes" / "d405.stl"
    mesh.parent.mkdir(parents=True)
    mesh.write_bytes(b"camera")
    assert resolve_mesh_reference(urdf, "file://realsense2_description/meshes/d405.stl") == mesh


def test_missing_mesh_is_rejected(tmp_path: Path) -> None:
    urdf = tmp_path / "URDF_robot_1" / "robot.urdf"
    urdf.parent.mkdir(parents=True)
    urdf.write_text(_urdf(), encoding="utf-8")
    with pytest.raises(Exception, match="mesh"):
        inspect_urdf(urdf)
```

Also add table-driven tests for these failures, each by mutating one synthetic URDF:

- wrong root link;
- duplicate joint name;
- missing `connector_joint`;
- missing one `left_joint`;
- missing one `right_joint`;
- wrong required joint type;
- missing, non-finite, inverted, or zero effort limits;
- missing any required gripper joint/link;
- missing any required camera or force-sensor link.

- [ ] **Step 5: Run URDF tests and confirm failure**

Run:

```bash
.venv/bin/pytest -q tests/unit_tests/robots/lynsense/isaac/test_urdf_contract.py
```

Expected: FAIL because `robots.lynsense.isaac.urdf_contract` does not exist.

- [ ] **Step 6: Implement the offline URDF contract**

Create `robots/lynsense/isaac/urdf_contract.py`.

Use these exact inventory types:

```python
@dataclass(frozen=True)
class UrdfJoint:
    name: str
    joint_type: str
    parent_link: str
    child_link: str
    lower: float | None
    upper: float | None
    effort: float | None
    velocity: float | None
    damping: float | None
    friction: float | None


@dataclass(frozen=True)
class UrdfMesh:
    reference: str
    resolved_path: Path
    geometry_type: str
    sha256: str


@dataclass(frozen=True)
class UrdfInventory:
    robot_name: str
    root_link: str
    joints: tuple[UrdfJoint, ...]
    links: tuple[str, ...]
    meshes: tuple[UrdfMesh, ...]
    required_joint_names: tuple[str, ...]
```

Define:

```python
REQUIRED_JOINT_NAMES = (
    "connector_joint",
    *(f"left_joint{i}" for i in range(1, 7)),
    *(f"right_joint{i}" for i in range(1, 7)),
)
REQUIRED_LINK_NAMES = (
    "head_camera_link",
    "left_camera_link",
    "right_camera_link",
    "left_force_sensor_link",
    "right_force_sensor_link",
)
REQUIRED_GRIPPER_JOINT_NAMES = (
    "left_R1", "left_R2", "left_R3", "left_L1", "left_L2", "left_L3",
    "right_R1", "right_R2", "right_R3", "right_L1", "right_L2", "right_L3",
)
REQUIRED_GRIPPER_LINK_NAMES = tuple(
    f"{name}_Link" for name in REQUIRED_GRIPPER_JOINT_NAMES
)
```

Implement `inspect_urdf(path: Path) -> UrdfInventory` with standard-library XML parsing:

- Parse only direct `<robot>` children for `<link>` and `<joint>`; this avoids treating duplicate `ros2_control` joint declarations as kinematic joints.
- Require a single unique root link: the link never appearing as a joint child.
- Require the root to be `base_link`.
- Reject duplicate links, duplicate joints, unknown parent/child links, and invalid XML.
- Require `connector_joint` to be prismatic and all 12 arm joints to be revolute.
- Parse finite lower/upper/effort/velocity limits for these 13 joints; require `lower < upper`, positive effort, and positive velocity.
- Require the six named R/L joints and their corresponding named links for each gripper subtree.
- Require every required sensor link.
- Collect every `<mesh filename="...">` under direct visual and collision geometry nodes.
- Resolve `package://package_name/rest/path` as `urdf.parent / package_name / rest`.
- Resolve `file://rest/path` as `urdf.parent / rest`.
- Reject other schemes and any path that escapes `urdf.parent`.
- Require every resolved mesh to be an existing regular file and digest it with SHA-256.

Give `UrdfInventory` a `to_dict()` method that emits JSON-safe strings, numbers, and nested lists.

Create `tests/unit_tests/robots/lynsense/isaac/fixtures.py` with
`write_valid_probe_archive(path: Path) -> None`. It writes the same synthetic
root, required joints/links, gripper subtrees, sensor links, package mesh, and
`URDF_robot_1/robot.urdf` layout used by the URDF tests into a gzipped tar
archive. The helper is test-only and never imports company assets.

Create `tests/unit_tests/robots/lynsense/isaac/conftest.py`:

```python
from __future__ import annotations

from pathlib import Path

import pytest

from tests.unit_tests.robots.lynsense.isaac.fixtures import (
    write_valid_probe_archive,
)


@pytest.fixture
def valid_probe_archive(tmp_path: Path) -> Path:
    archive = tmp_path / "robot.tar.gz"
    write_valid_probe_archive(archive)
    return archive
```

- [ ] **Step 7: Run archive and URDF tests**

Run:

```bash
.venv/bin/pytest -q \
  tests/unit_tests/robots/lynsense/isaac/test_archive.py \
  tests/unit_tests/robots/lynsense/isaac/test_urdf_contract.py
```

Expected: PASS.

- [ ] **Step 8: Review the task diff**

Run `git diff --check` and inspect only the files named in this task. Do not commit.

### Task 3: Importer Warning Classification And Render Evidence Gate

**Files:**
- Create: `robots/lynsense/isaac/import_contract.py`
- Create: `robots/lynsense/isaac/render_evidence.py`
- Test: `tests/unit_tests/robots/lynsense/isaac/test_import_contract.py`
- Test: `tests/unit_tests/robots/lynsense/isaac/test_render_evidence.py`

**Interfaces:**
- Consumes: `IsaacProbeError`.
- Produces: `ImporterEvent`, `ImporterContractError`, `classify_importer_events`, `require_no_blocking_events`, `validate_render_evidence`.

- [ ] **Step 1: Write failing importer classification tests**

```python
from __future__ import annotations

import pytest

from robots.lynsense.isaac.import_contract import (
    ImporterEvent,
    classify_importer_events,
    require_no_blocking_events,
)


def event(category: str, severity: str = "warning") -> ImporterEvent:
    return ImporterEvent(
        category=category,
        severity=severity,
        source="URDF importer",
        message="fixture message",
        decision="recorded",
    )


@pytest.mark.parametrize(
    "category",
    [
        "material_change",
        "color_change",
        "texture_change",
        "skipped_link",
        "mimic_joint_decision",
        "fixed_link_reduction",
        "naming_change",
    ],
)
def test_non_destructive_changes_are_recorded(category: str) -> None:
    result = classify_importer_events([event(category)])
    assert result[0].decision == "recorded"


@pytest.mark.parametrize(
    "category",
    [
        "missing_required_joint",
        "invalid_joint_limit",
        "unreadable_mesh",
        "mesh_substitution",
        "dropped_gripper_link",
        "dropped_gripper_coupling",
        "dropped_camera",
        "dropped_sensor",
    ],
)
def test_destructive_changes_block(category: str) -> None:
    classified = classify_importer_events([event(category)])
    assert classified[0].decision == "blocking"
    with pytest.raises(Exception, match=category):
        require_no_blocking_events(classified)


def test_error_severity_and_unknown_category_block() -> None:
    classified = classify_importer_events(
        [event("material_change", severity="error")]
    )
    assert classified[0].decision == "blocking"
    with pytest.raises(Exception, match="error"):
        require_no_blocking_events(classified)

    with pytest.raises(Exception, match="uncategorized"):
        require_no_blocking_events(
            classify_importer_events([event("unknown_category")])
        )
```

- [ ] **Step 2: Write failing render evidence tests**

```python
from __future__ import annotations

from pathlib import Path

import imageio.v3 as iio
import numpy as np
import pytest

from robots.lynsense.isaac.render_evidence import validate_render_evidence


def _write_png(path: Path, pixels: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    iio.imwrite(path, pixels, extension=".png")


def test_three_nonblank_pngs_pass(tmp_path: Path) -> None:
    y, x = np.mgrid[:180, :320]
    pixels = (((x + y) % 256)).astype(np.uint8)
    pixels = np.stack((pixels, pixels, pixels), axis=-1)
    for name in (
        "scene-initial.png", "scene-action.png", "scene-final.png"
    ):
        _write_png(tmp_path / name, pixels)
    result = validate_render_evidence(tmp_path)
    assert result["minimum_pixels"] == 180
    assert result["all_nonblank"] is True


def test_blank_png_fails(tmp_path: Path) -> None:
    pixels = np.full((180, 320, 3), 17, dtype=np.uint8)
    for name in (
        "scene-initial.png", "scene-action.png", "scene-final.png"
    ):
        _write_png(tmp_path / name, pixels)
    with pytest.raises(Exception, match="blank"):
        validate_render_evidence(tmp_path)


def test_rgba_png_passes_with_color_statistics(tmp_path: Path) -> None:
    y, x = np.mgrid[:180, :320]
    gray = ((x + y) % 256).astype(np.uint8)
    pixels = np.stack(
        (gray, gray, gray, np.full_like(gray, 255)),
        axis=-1,
    )
    for name in (
        "scene-initial.png", "scene-action.png", "scene-final.png"
    ):
        _write_png(tmp_path / name, pixels)
    assert validate_render_evidence(tmp_path)["all_nonblank"] is True


def test_missing_or_undersized_image_fails(tmp_path: Path) -> None:
    pixels = np.zeros((160, 320, 3), dtype=np.uint8)
    pixels[0, 0] = 255
    _write_png(tmp_path / "scene-initial.png", pixels)
    _write_png(tmp_path / "scene-action.png", pixels)
    with pytest.raises(Exception, match="scene-final.png"):
        validate_render_evidence(tmp_path)
```

- [ ] **Step 3: Run tests and confirm failures**

Run:

```bash
.venv/bin/pytest -q \
  tests/unit_tests/robots/lynsense/isaac/test_import_contract.py \
  tests/unit_tests/robots/lynsense/isaac/test_render_evidence.py
```

Expected: FAIL because both modules do not exist.

- [ ] **Step 4: Implement classification and render checks**

Create `robots/lynsense/isaac/import_contract.py`:

```python
from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from robots.lynsense.isaac.contracts import IsaacProbeError


EventSeverity = Literal["info", "warning", "error"]
EventDecision = Literal["recorded", "blocking"]


@dataclass(frozen=True)
class ImporterEvent:
    category: str
    severity: EventSeverity
    source: str
    message: str
    decision: EventDecision = "recorded"


@dataclass(frozen=True)
class ImporterContractError(IsaacProbeError):
    events: tuple[ImporterEvent, ...]

    def to_dict(self) -> dict[str, object]:
        return {
            "reason": str(self),
            "events": [event.to_dict() for event in self.events],
        }


RECORDED_CATEGORIES = frozenset({
    "material_change",
    "color_change",
    "texture_change",
    "skipped_link",
    "mimic_joint_decision",
    "fixed_link_reduction",
    "naming_change",
})

BLOCKING_CATEGORIES = frozenset({
    "uncategorized",
    "missing_required_joint",
    "invalid_joint_limit",
    "unreadable_mesh",
    "mesh_substitution",
    "dropped_gripper_link",
    "dropped_gripper_coupling",
    "dropped_camera",
    "dropped_sensor",
})
```

Implement `classify_importer_events(events: tuple[ImporterEvent, ...]) -> tuple[ImporterEvent, ...]`:

- Reject empty category, source, or message.
- Reject severity outside `info`, `warning`, and `error`.
- Normalize categories outside both sets to blocking category `uncategorized`; retain the original category in the serialized message.
- Force every `error` and every blocking category to `decision="blocking"`.
- Preserve input order and return rebuilt immutable events.
- Give `ImporterEvent` a `to_dict()` method containing category, severity, source, message, and decision.

Implement `require_no_blocking_events(events: tuple[ImporterEvent, ...]) -> None` separately. It raises `ImporterContractError` whose serialized payload includes every event, not only the first blocking event. The runtime calls `classify_importer_events`, writes or retains those events, and calls `require_no_blocking_events` before constructing `World`; the host catches `ImporterContractError` and includes `events` in the failed technical summary.

Create `robots/lynsense/isaac/render_evidence.py`. Import `imageio.v3` and `numpy` at function scope so importing the module remains lightweight. Implement `validate_render_evidence(run_directory: Path) -> dict[str, object]`:

- Require all three exact screenshot names.
- Require valid, decoded RGB or RGBA images with width at least 320 and height at least 180.
- Reject non-finite or negative values.
- Compute statistics on the first three color channels only. Reject a single-value image, standard deviation below `1.0`, or fewer than `1,000` pixels whose RGB value differs from the upper-left RGB value.
- Return per-file width, height, standard deviation, changed-pixel count, plus `all_nonblank=True`.

- [ ] **Step 5: Run focused tests**

Run the corrected two-file pytest command. Expected: PASS.

- [ ] **Step 6: Review the task diff**

Run `git diff --check`. Do not commit.

### Task 4: Bounded Joint Action And Fixed-Base Evaluators

**Files:**
- Create: `robots/lynsense/isaac/action_contract.py`
- Test: `tests/unit_tests/robots/lynsense/isaac/test_action_contract.py`

**Interfaces:**
- Consumes: `IsaacProbeError`.
- Produces: `DriveGains`, `DriveLimits`, `DriveConfiguration`, `JointActionPlan`, `JointSample`, `TransformSample`, `plan_joint_action`, `validate_step_state`, `evaluate_joint_action`, `evaluate_base_motion`, `joint_trajectories_to_dict`, `base_motion_to_dict`.

- [ ] **Step 1: Write failing action tests**

```python
from __future__ import annotations

import math

import pytest

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


def test_drive_configuration_separates_gains_from_physical_limits() -> None:
    configuration = DriveConfiguration(
        gains=DriveGains(stiffness=200.0, damping=16.6),
        limits=DriveLimits(
            max_effort_nm=200.0,
            max_velocity_rad_s=0.314,
        ),
        verified=True,
    )
    assert configuration.to_dict() == {
        "gains": {"stiffness": 200.0, "damping": 16.6},
        "limits": {"max_effort_nm": 200.0, "max_velocity_rad_s": 0.314},
        "verified": True,
    }


def test_action_target_is_initial_plus_one_tenth() -> None:
    plan = plan_joint_action("left_joint1", 0.2, -1.0, 1.0)
    assert plan == JointActionPlan(
        joint_name="left_joint1",
        initial_position=0.2,
        target_position=0.30000000000000004,
        lower_limit=-1.0,
        upper_limit=1.0,
    )


@pytest.mark.parametrize(
    ("lower", "upper", "initial"),
    [(-1.0, 1.0, 0.95), (-1.0, 1.0, 1.0), (-1.0, 1.0, -2.0)],
)
def test_out_of_limit_or_invalid_target_fails(
    lower: float, upper: float, initial: float
) -> None:
    with pytest.raises(Exception, match="target"):
        plan_joint_action("left_joint1", initial, lower, upper)


def _trajectory(target: float, final_time: float = 1.0) -> tuple[JointSample, ...]:
    samples = [
        JointSample(time_s=0.0, position_rad=0.0, velocity_rad_s=0.0),
        JointSample(time_s=0.2, position_rad=0.05, velocity_rad_s=0.2),
        JointSample(time_s=0.6, position_rad=target, velocity_rad_s=0.01),
        JointSample(time_s=1.0, position_rad=target, velocity_rad_s=0.01),
    ]
    return tuple(samples)


def test_complete_settled_action_passes() -> None:
    plans = {
        name: plan_joint_action(name, 0.0, -1.0, 1.0)
        for name in ("left_joint1", "right_joint1")
    }
    trajectories = {name: _trajectory(0.1) for name in plans}
    result = evaluate_joint_action(trajectories, plans)
    assert result["passed"] is True
    assert result["final_time_s"] == 1.0


def test_insufficient_progress_fails() -> None:
    plans = {
        name: plan_joint_action(name, 0.0, -1.0, 1.0)
        for name in ("left_joint1", "right_joint1")
    }
    trajectories = {
        "left_joint1": _trajectory(0.1),
        "right_joint1": _trajectory(0.04),
    }
    with pytest.raises(Exception, match="right_joint1.*progress"):
        evaluate_joint_action(trajectories, plans)


def test_final_window_speed_and_overshoot_fail() -> None:
    plan = plan_joint_action("left_joint1", 0.0, -1.0, 1.0)
    moving = list(_trajectory(0.1))
    moving[-1] = JointSample(1.0, 0.1, 0.2)
    with pytest.raises(Exception, match="speed"):
        evaluate_joint_action({"left_joint1": tuple(moving)}, {"left_joint1": plan})

    overshoot = list(_trajectory(0.1))
    overshoot[-2] = JointSample(0.6, 0.13, 0.01)
    with pytest.raises(Exception, match="overshoot"):
        evaluate_joint_action(
            {"left_joint1": tuple(overshoot)}, {"left_joint1": plan}
        )


def test_base_motion_translation_and_rotation_limits() -> None:
    identity = TransformSample(
        time_s=0.0,
        translation=(0.0, 0.0, 0.0),
        rotation_xyzw=(0.0, 0.0, 0.0, 1.0),
    )
    within = TransformSample(
        time_s=1.0,
        translation=(0.0005, 0.0, 0.0),
        rotation_xyzw=(0.0, 0.0, 0.0, 1.0),
    )
    assert evaluate_base_motion((identity, within))["passed"] is True

    outside = TransformSample(
        time_s=1.0,
        translation=(0.0011, 0.0, 0.0),
        rotation_xyzw=(0.0, 0.0, 0.0, 1.0),
    )
    with pytest.raises(Exception, match="translation"):
        evaluate_base_motion((identity, outside))


def test_base_rotation_is_relative_to_initial_orientation() -> None:
    initial = TransformSample(
        time_s=0.0,
        translation=(0.0, 0.0, 0.0),
        rotation_xyzw=(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)),
    )
    within = TransformSample(
        time_s=1.0,
        translation=(0.0, 0.0, 0.0),
        rotation_xyzw=(
            0.0,
            0.0,
            math.sin((math.pi / 2 + 0.001) / 2),
            math.cos((math.pi / 2 + 0.001) / 2),
        ),
    )
    assert evaluate_base_motion((initial, within))["passed"] is True

    outside = TransformSample(
        time_s=1.0,
        translation=(0.0, 0.0, 0.0),
        rotation_xyzw=(
            0.0,
            0.0,
            math.sin((math.pi / 2 + 0.01) / 2),
            math.cos((math.pi / 2 + 0.01) / 2),
        ),
    )
    with pytest.raises(Exception, match="rotation"):
        evaluate_base_motion((initial, outside))


def test_nonfinite_action_and_base_state_fail() -> None:
    plan = plan_joint_action("left_joint1", 0.0, -1.0, 1.0)
    bad = list(_trajectory(0.1))
    bad[-1] = JointSample(1.0, math.nan, 0.0)
    with pytest.raises(Exception, match="finite"):
        evaluate_joint_action({"left_joint1": tuple(bad)}, {"left_joint1": plan})

    identity = TransformSample(0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    bad_base = TransformSample(1.0, (0.0, math.inf, 0.0), (0.0, 0.0, 0.0, 1.0))
    with pytest.raises(Exception, match="finite"):
        evaluate_base_motion((identity, bad_base))


def test_streaming_step_check_fails_immediately() -> None:
    plans = {
        name: plan_joint_action(name, 0.0, -1.0, 1.0)
        for name in ("left_joint1", "right_joint1")
    }
    samples = {
        "left_joint1": JointSample(0.1, 0.05, 0.1),
        "right_joint1": JointSample(0.1, math.nan, 0.1),
    }
    base = (
        TransformSample(0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        TransformSample(0.1, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
    )
    with pytest.raises(Exception, match="finite"):
        validate_step_state(plans, samples, base)

    samples["right_joint1"] = JointSample(0.1, 0.05, 0.1)
    outside_base = base + (
        TransformSample(0.2, (0.0011, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
    )
    with pytest.raises(Exception, match="translation"):
        validate_step_state(plans, samples, outside_base)
```

Add tests that also reject empty trajectories, unordered times, duration over 10 seconds, a missing final half-second window, and quaternion norm near zero.

- [ ] **Step 2: Run action tests and confirm failure**

Run:

```bash
.venv/bin/pytest -q tests/unit_tests/robots/lynsense/isaac/test_action_contract.py
```

Expected: FAIL because `action_contract.py` does not exist.

- [ ] **Step 3: Implement the deterministic evaluators**

Create `robots/lynsense/isaac/action_contract.py` with:

```python
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping

from robots.lynsense.isaac.contracts import IsaacProbeError

ACTION_JOINT_NAMES = ("left_joint1", "right_joint1")
ACTION_DELTA_RAD = 0.1
MAX_ACTION_EFFORT_NM = 200.0
MAX_ACTION_SPEED_RAD_S = 0.314
MIN_PROGRESS_RAD = 0.05
MAX_FINAL_ERROR_RAD = 0.02
MAX_FINAL_SPEED_RAD_S = 0.05
MAX_OVERSHOOT_RAD = 0.02
MAX_ACTION_DURATION_S = 10.0
FINAL_SETTLE_WINDOW_S = 0.5
MAX_BASE_TRANSLATION_M = 0.001
MAX_BASE_ROTATION_RAD = math.radians(0.5)


@dataclass(frozen=True)
class DriveGains:
    stiffness: float = 200.0
    damping: float = 16.6


@dataclass(frozen=True)
class DriveLimits:
    max_effort_nm: float = MAX_ACTION_EFFORT_NM
    max_velocity_rad_s: float = MAX_ACTION_SPEED_RAD_S


@dataclass(frozen=True)
class DriveConfiguration:
    gains: DriveGains
    limits: DriveLimits
    verified: bool = False


@dataclass(frozen=True)
class JointActionPlan:
    joint_name: str
    initial_position: float
    target_position: float
    lower_limit: float
    upper_limit: float


@dataclass(frozen=True)
class JointSample:
    time_s: float
    position_rad: float
    velocity_rad_s: float


@dataclass(frozen=True)
class TransformSample:
    time_s: float
    translation: tuple[float, float, float]
    rotation_xyzw: tuple[float, float, float, float]
```

Implement:

- `DriveConfiguration.to_dict()` emits the nested gain/limit dictionaries shown in its test and rejects construction with non-finite or negative values.
- `plan_joint_action` validates finite initial/limits, `lower < upper`, initial inside limits, and requires `initial + 0.1 <= upper`; reject before returning a plan.
- `evaluate_joint_action(trajectories, plans)` requires the exact two names, identical sample-time sequences, at least two samples, ordered finite times, duration no more than 10 seconds, and finite positions/speeds.
- `validate_step_state(plans, latest_samples, base_history)` is the streaming gate for the runtime. It requires exactly both action-joint names, a finite latest time/position/speed, no current overshoot over `0.02 rad`, and recomputes cumulative base translation/rotation from the first base sample against `0.001 m` and `0.5 deg`. It raises on the first bad physics sample.
- For each joint, compute progress as signed movement toward target, maximum target error, maximum overshoot beyond the target, and final-window samples starting at `final_time - 0.5`.
- Require progress at least `0.05 rad`, all final-window errors at most `0.02 rad`, all final-window absolute speeds at most `0.05 rad/s`, and maximum overshoot at most `0.02 rad`.
- Return a JSON-safe dictionary with `passed=True`, final time, and per-joint metrics.
- `evaluate_base_motion(samples)` checks finite translation/quaternion values, ordered time, translation displacement from the first sample against `0.001 m`, relative rotation from the first sample against `0.5 deg`, and returns maximum metrics.
- Normalize both quaternions, compute `q_current * inverse(q_first)`, and derive the relative angle as `2 * atan2(norm(x, y, z), abs(w))`; reject either norm below `1e-9`. Do not interpret each sample's own `w` as rotation from the first sample.
- Implement `joint_trajectories_to_dict` and `base_motion_to_dict` without tuples in output.

- [ ] **Step 4: Run focused tests**

Repeat Step 2 command. Expected: PASS.

- [ ] **Step 5: Review the task diff**

Run `git diff --check`. Do not commit.

### Task 5: Probe Runner, Explicit Skip, And Isaac 3 Runtime Boundary

**Files:**
- Create: `robots/lynsense/isaac/runtime.py`
- Create: `robots/lynsense/isaac/probe.py`
- Modify: `robots/lynsense/isaac/__init__.py`
- Test: `tests/unit_tests/robots/lynsense/isaac/test_probe_runner.py`
- Test: `tests/unit_tests/robots/lynsense/isaac/test_runtime_boundary.py`

**Interfaces:**
- Consumes: all Task 1-4 interfaces.
- Produces: `IsaacRuntimeResult`, `IsaacRuntime`, `IsaacRuntimeFailure`, `Isaac3Runtime`, `build_parser`, `execute_host_probe`, `execute_isaac_probe`, `record_visual_review`, `verify_milestone`, `main`.
- [ ] **Step 1: Write failing host runner tests**

```python
from __future__ import annotations

import json
from pathlib import Path

import pytest

from robots.lynsense.isaac import probe


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
```

```python
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
```

The fake-runtime test must import `DriveConfiguration`, `DriveGains`, `DriveLimits`, `JointActionPlan`, `JointSample`, and `TransformSample` from their producing modules.

Add a passing fake-runtime test that injects a runtime factory through:

```python
execute_isaac_probe(
    [
        "--mode",
        "isaac",
        "--run-directory",
        str(run_directory),
    ],
    runtime_factory=factory,
)
```

The fake returns:

```python
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
IsaacRuntimeResult(
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
```

The fake also writes three nonblank fixture PNGs. Assert:

- `environment.json` is written;
- `technical-summary.json` has `outcome="passed"`, action metrics, base metrics, and importer events;
- render validation runs;
- no `visual-review.json` is created automatically;
- return code is `0`.

Add failure tests for unsupported fake version, classified blocking event, failed action evaluator, blank render, runtime exception, and shutdown exception. Each must write a technical summary with `outcome="failed"` and return `1`.

Add a milestone-gate test with a fake passing technical summary, valid accepted
visual review, nonblank screenshots, and generated USD marker. Assert
`verify_milestone(run_directory) == 0`. Mutate exactly one input at a time to
rejected visual review, missing visual review, failed technical outcome, missing
USD, unsupported environment version, blank screenshot, run directory outside
`DEFAULT_ARTIFACT_ROOT`, and a blocking importer event; each must return `1`
without changing the stored review.

- [ ] **Step 2: Write failing runtime-boundary source tests**

```python
from __future__ import annotations

from pathlib import Path

from robots.lynsense.isaac import runtime


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
    assert 'omni.importer.urdf' in source
    assert 'config.fix_base = True' in source
    assert 'config.merge_fixed_joints = False' in source
    assert 'JOINT_DRIVE_POSITION' in source
    assert 'config.default_drive_strength = 200.0' in source
    assert 'config.default_position_drive_damping = 16.6' in source
    assert 'GetMaxForceAttr().Set(200.0)' in source
    assert 'GetMaxJointVelocityAttr().Set(math.degrees(0.314))' in source
    assert 'drive_gains=' in source
    assert 'capture_viewport_to_file(' in source
    assert 'wait_for_result()' in source
    assert 'capture_screenshot' not in source
    assert 'self._app.close()' in source
```

Add a focused runtime helper test:

```python
def test_viewport_capture_helper_waits_for_result_protocol(
    tmp_path: Path,
) -> None:
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
```

Do not make this the only runtime review; it guards the version-sensitive import boundary and key constants.

- [ ] **Step 3: Run runner tests and confirm failures**

Run:

```bash
.venv/bin/pytest -q \
  tests/unit_tests/robots/lynsense/isaac/test_probe_runner.py \
  tests/unit_tests/robots/lynsense/isaac/test_runtime_boundary.py
```

Expected: FAIL because the runner and runtime modules do not exist.

- [ ] **Step 4: Implement the runtime result and Isaac 3 adapter**

Create `robots/lynsense/isaac/runtime.py`.

Define:

```python
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
    """An Isaac failure that preserves partial evidence.

    ``partial_result`` contains the last complete environment, importer,
    inventory, drive, and trajectory state that was safe to serialize.
    ``importer_events`` is retained even when blocking classification stops
    the probe before simulation.
    """

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
```

Implement `Isaac3Runtime.run` as the only place Isaac modules are imported.

Give `Isaac3Runtime` the constructor
`__init__(self, drive_gains: DriveGains = DriveGains()) -> None`. The probe
creates it with `Isaac3Runtime(drive_gains=DriveGains())` and never mutates the
gains after construction.

At module scope, import `asyncio` and `inspect` before any Isaac API. Implement
the synchronous `_await_isaac_task(task, output_path)` helper there so viewport
capture can wait for a returned future-like object.

The helper must:

1. Prefer `task.wait_for_result()` when that method exists.
2. If `wait_for_result()` returns an awaitable, run it with `asyncio.run`.
3. As a fallback only, run `task` itself with `asyncio.run` when
   `inspect.isawaitable(task)` is true.
4. Reject an object that exposes neither contract.
5. Require `output_path` to exist and be nonempty before
   returning to the caller.

Start the application before Isaac imports:

```python
from omni.isaac.kit import SimulationApp

self._app = SimulationApp(headless=False, experience="")
```

Then import and configure the 2023.1-era API:

```python
from omni.importer.urdf import _urdf
import omni.kit.commands
import omni.usd
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.world import World
from omni.isaac.core.articulations import Articulation, ArticulationAction
import omni.kit.app
from omni.kit.viewport.utility import (
    capture_viewport_to_file,
    get_active_viewport,
)
import carb
import math
import platform
from pxr import PhysxSchema, UsdPhysics
```

After those imports succeed and `get_active_viewport` returns an active
viewport, construct `IsaacEnvironment` only from observed runtime values:

- semantic version from `kit_app.get_app_version()`;
- Kit version from `kit_app.get_kit_version()`, without substituting the semantic version or a constant;
- Python version from `platform.python_version()`;
- renderer from the non-empty `carb.settings.get_settings().get("/rtx/rendermode")` value;
- `run_mode="desktop"` only because this process launched `SimulationApp(headless=False)`;
- capability names added only after `omni.importer.urdf`, `Articulation`, viewport capture, and `PhysxSchema.PhysxJointAPI` have imported successfully.

Call `validate_environment(environment)` as well as the version check. If a metadata or capability source is unavailable, fail the environment stage; do not synthesize a plausible value.

Configure the importer exactly as follows:

```python
urdf_interface = _urdf.acquire_urdf_interface()
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
```

Run:

```python
parsed, robot_model = omni.kit.commands.execute(
    "URDFParseFile",
    urdf_path=str(paths.robot_urdf),
    import_config=import_config,
)
if not parsed:
    raise IsaacProbeError("Isaac URDF parser rejected robot.urdf")
```

Inspect `robot_model.joints` before import and create blocking `ImporterEvent` records for missing required joints or limits. Execute:

```python
imported, prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=str(paths.robot_urdf),
    import_config=import_config,
    dest_path=str(paths.generated_usd),
)
if not imported or not paths.generated_usd.is_file():
    raise IsaacProbeError("Isaac did not generate the expected USD file")
```

Immediately classify importer output events before opening the generated stage
or constructing `World`. Read the Isaac log available at this point and map only
these source patterns:

- required joint or limit absence;
- unreadable or substituted mesh;
- dropped gripper link/coupling;
- dropped camera/sensor;
- skipped non-required links and mimic-joint decisions;
- material/color/texture/fixed-link/naming warnings.

Call `classified = classify_importer_events(events)` and then
`require_no_blocking_events(classified)`. On `ImporterContractError`, wrap it in
`IsaacRuntimeFailure(stage="import", importer_events=classified)` so shutdown
runs and the failed summary retains every event. Unmapped warning/error lines
become category `uncategorized` and therefore block. Never silently classify an
Isaac `ERROR` as recorded.

`default_drive_strength` and `default_position_drive_damping` are the fixed
probe gains and are recorded as `drive_gains`; neither value is an effort or
velocity limit. The runtime must pass the exact gain pair to `Isaac3Runtime`
rather than tuning either value during the run.

After importer-event classification, open the generated USD stage and configure
both selected angular drives:

```python
drive = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
drive.GetMaxForceAttr().Set(200.0)
drive.GetStiffnessAttr().Set(drive_gains.stiffness)
drive.GetDampingAttr().Set(drive_gains.damping)
physx_joint = PhysxSchema.PhysxJointAPI.Apply(joint_prim)
physx_joint.GetMaxJointVelocityAttr().Set(math.degrees(0.314))
```

Read all four values back before simulation. Fail unless maximum force is
`200.0`, maximum joint velocity is `math.degrees(0.314)` within USD float
precision, and the recorded gain pair round-trips exactly. `ArticulationAction`
velocities remain radians per second, while USD angular attributes use degrees;
the conversion is mandatory rather than cosmetic. Save the USD stage after the
limits round-trip.

Build the fixed-base scene:

```python
world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0)
world.scene.add_default_ground_plane()
robot_prim_path = "/lynsense_robot_1"
add_reference_to_stage(str(paths.generated_usd), robot_prim_path)
robot = Articulation(prim_path=robot_prim_path)
world.scene.add(robot)
world.reset()
```

Before selecting those indices, traverse `world.stage` to collect loaded prim names. Require all `REQUIRED_GRIPPER_JOINT_NAMES`, `REQUIRED_GRIPPER_LINK_NAMES`, and `REQUIRED_LINK_NAMES`; then require every name in `REQUIRED_JOINT_NAMES` to appear in `robot.dof_properties["names"]`. A URDF input that passes offline inspection but loses one of these paths in USD must fail here.

Find `left_joint1` and `right_joint1` indices from `robot.dof_properties["names"]`; fail if either is absent. Read their positions and limits, construct action plans with `plan_joint_action`, capture the initial viewport screenshot, apply this action at every step, and step for at most 600 physics frames:

```python
action = ArticulationAction(
    joint_positions=[left_plan.target_position, right_plan.target_position],
    joint_velocities=[0.314, 0.314],
    joint_indices=[left_index, right_index],
)
robot.apply_action(action)
world.step(render=True)
```

At each step append:

- both joint positions and velocities from `robot.get_joint_positions()` and `robot.get_joint_velocities()`;
- `robot.get_local_pose()` translation/orientation as a `TransformSample`, converting Isaac quaternion order to XYZW if the API returns WXYZ;
- immediately call `validate_step_state(action_plans, latest_samples, base_motion)` with the appended history.

If the streaming check raises, stop applying actions at that physics step and wrap the exception in `IsaacRuntimeFailure(stage="action", partial_result=partial_result)`; never continue stepping to the 10-second budget. The partial result includes both trajectories and base motion accumulated through the failing sample.

Capture `scene-initial.png` before the action, `scene-action.png` when both joints first exceed `0.05 rad` progress or at the halfway budget, and `scene-final.png` after the final step. Use the documented viewport capture API:

```python
viewport = get_active_viewport()
capture_task = capture_viewport_to_file(
    viewport,
    file_path=str(path.resolve()),
)
_await_isaac_task(capture_task, path)
```

Implement `_await_isaac_task(task, output_path)` before app startup with `inspect.isawaitable`;
prefer `wait_for_result()` as described above and require the resulting
screenshot file to exist before returning.

In a `finally` block, call `world.stop()`, `world.clear_instance()` where available, and `self._app.close()`. Convert any shutdown exception into a failed runtime result; do not mask the original action/import failure if one already happened.

- [ ] **Step 5: Implement host and Isaac probe modes**

Create `robots/lynsense/isaac/probe.py`.

Parser contract:

```text
--mode {host,isaac}
--archive PATH                         default ../URDF/URDF_robot_1.tar.gz
--isaac-python PATH
--max-uncompressed-bytes INTEGER      default 2147483648
--visual-review RUN_DIRECTORY
--status {accepted,rejected}
--reviewer TEXT
--notes TEXT
--verify-milestone RUN_DIRECTORY
```

Use a custom argparse action or post-parse validation for
`--max-uncompressed-bytes`: reject non-integers, values below `1`, and values
above `MAX_UNCOMPRESSED_BYTES`. Do not clamp a larger request silently.

Use a timestamped `YYYYMMDDTHHMMSSZ-<uuid4-hex>` run ID. `execute_host_probe(argv)`:

1. Parses arguments and creates artifact paths.
2. Creates the run directory with mode `0o700`.
3. Calls `extract_archive(args.archive, paths.extracted_root, max_uncompressed_bytes=args.max_uncompressed_bytes)`; on failure writes a failed technical summary and returns `1`.
4. Calls `inspect_urdf`; writes `urdf-inventory.json` atomically.
5. If `--isaac-python` is absent, writes `technical-summary.json` with `outcome="skipped"`, `reason="No explicit Isaac Sim Python executable was supplied"`, and `is_pass=False`; returns `77`.
6. If supplied, launches exactly:

```python
[
    str(args.isaac_python),
    str(probe_path),
    "--mode",
    "isaac",
    "--run-directory",
    str(paths.run_directory),
    "--max-uncompressed-bytes",
    str(args.max_uncompressed_bytes),
]
```

Set `PYTHONPATH` to include the repository root.
7. Captures stdout/stderr to `isaac.log`.
8. Reads the Isaac-mode technical summary; returns its process exit status, or `1` if no valid summary was produced.

`execute_isaac_probe(argv, runtime_factory=Isaac3Runtime)`:

1. Requires `--run-directory` and reconstructs `ArtifactPaths`.
2. Requires the run directory to contain the offline `urdf-inventory.json`.
3. Requires the extracted URDF to remain present and never restages the company archive.
4. Instantiates the runtime, calls `runtime.run(paths)`, and validates the returned environment.
5. Classifies importer events and validates that every required articulation joint, gripper joint/link, and sensor link appears in the runtime-returned loaded-name inventories.
6. Requires `drive_configuration.verified=True`, `max_effort_nm=200.0`, and `max_velocity_rad_s=0.314`.
7. Uses the runtime-returned `action_plans` dictionary.
8. Evaluates joint trajectories and base motion.
9. Validates render evidence.
10. Writes `environment.json`, importer events, evaluator metrics, drive configuration, extraction limit, USD path, run directory, and `outcome="passed"` to `technical-summary.json`.
11. Catches `IsaacRuntimeFailure` before generic exceptions and writes `outcome="failed"`, `reason`, `stage`, `exc.importer_events`, and the partial environment/inventory/drive/trajectory/base data from `exc.partial_result` when present; generic exceptions use the same failed-summary shape with the evidence they can recover, then return `1`.

Implement `record_visual_review` as a host operation. It:

- validates all three screenshots first;
- requires `--status`, `--reviewer`, and `--notes`;
- constructs the exact `VisualReview` JSON contract;
- writes `visual-review.json` atomically;
- returns `0` for accepted or rejected review, because both are valid recorded outcomes.

`main()` returns the selected function's integer code. Do not auto-discover Isaac executables. Do not write `visual-review.json` from a technical probe.

Implement `verify_milestone(run_directory: Path) -> int` as the final gate. It:

1. Requires the resolved run directory to remain below `DEFAULT_ARTIFACT_ROOT`.
2. Loads `environment.json`, `technical-summary.json`, and `visual-review.json`; reconstructs the environment with `environment_from_dict` and validates both review documents.
3. Requires semantic version `2023.1.*`, complete environment capabilities, technical `outcome="passed"`, `is_pass=true`, verified `200 N*m` and `0.314 rad/s` drive limits, action/base evaluation pass flags, and no blocking importer event.
4. Requires `visual-review.json` status to be exactly `accepted`.
5. Requires the generated USD file and all three screenshots to exist and the render check to pass.
6. Returns `0` only when all checks pass and `1` otherwise; it never edits technical evidence or the human review.

Export only the public command entry point from `robots.lynsense/isaac/__init__.py`; preserve optional runtime imports at their use sites.

- [ ] **Step 6: Run focused runner and boundary tests**

Run:

```bash
.venv/bin/pytest -q \
  tests/unit_tests/robots/lynsense/isaac/test_probe_runner.py \
  tests/unit_tests/robots/lynsense/isaac/test_runtime_boundary.py
```

Expected: PASS.

- [ ] **Step 7: Run the real no-Isaac host command**

From the RPent repository root:

```bash
PYTHONPATH=. .venv/bin/python robots/lynsense/isaac/probe.py \
  --archive ../URDF/URDF_robot_1.tar.gz
```

Expected:

- process exits `77`;
- `technical-summary.json` has `outcome="skipped"` and `is_pass=false`;
- `urdf-inventory.json` contains all 13 required joints, both gripper subtrees, and resolved mesh paths;
- no USD or screenshots exist;
- generated files remain under `.artifacts/`.

Do not treat this as milestone acceptance.

- [ ] **Step 8: Review the task diff**

Run `git diff --check`; inspect the runtime imports, cleanup path, and every write destination. Do not commit.

### Task 6: Documentation, Regression Matrix, And Open Company Gate

**Files:**
- Create: `robots/lynsense/isaac/README.md`
- Modify: `tests/unit_tests/robots/lynsense/isaac/test_docs.py`
- Test: `tests/unit_tests/robots/lynsense/isaac/test_docs.py`

**Interfaces:**
- Consumes: final Task 1-5 command and artifact contracts.
- Produces: operator documentation, focused/full offline regression evidence, and an explicit record that the company Isaac gate remains open on this workstation.

- [ ] **Step 1: Write failing documentation tests**

```python
from __future__ import annotations

from pathlib import Path


def test_isaac_readme_documents_scope_commands_and_gates() -> None:
    readme = Path("robots/lynsense/isaac/README.md").read_text(encoding="utf-8")
    assert "Isaac Sim `2023.1.*`" in readme
    assert "--isaac-python" in readme
    assert "--visual-review" in readme
    assert "--verify-milestone" in readme
    assert "`77`" in readme
    assert "not a pass" in readme
    assert "`visual-review.json`" in readme
    assert "Webots baseline" in readme
    assert "URDF_robot_1.tar.gz" in readme
    assert "ROS Domain 3" in readme
    assert ".artifacts/lynsense-isaac" in readme
```

- [ ] **Step 2: Run documentation test and confirm failure**

Run:

```bash
.venv/bin/pytest -q tests/unit_tests/robots/lynsense/isaac/test_docs.py
```

Expected: FAIL because the README and test do not exist.

- [ ] **Step 3: Write the operator README**

Create `robots/lynsense/isaac/README.md`.

It must contain these sections:

1. `Scope`: company Isaac Sim `2023.1.*` only; Webots remains the verified baseline; no ROS, `py_trees`, RPent policy, real robot, ROS Domain 3, robot SSH state, robot workspace, or `.env.lynsense`.
2. `Offline inspection and explicit skip`: exact host command and explanation that exit `77` is an explicit skip, not a pass.
3. `Company Isaac run`: require an operator-supplied Isaac Python path and show:

```bash
PYTHONPATH=. .venv/bin/python robots/lynsense/isaac/probe.py \
  --archive ../URDF/URDF_robot_1.tar.gz \
  --isaac-python /company/path/to/isaac-sim/python.sh
```

4. `Evidence`: list `environment.json`, `robot_1.usd`, `technical-summary.json`, `urdf-inventory.json`, `isaac.log`, and all three screenshots.
5. `Human visual review`: explain that automated nonblank checks cannot prove company-model fidelity, then show:

```bash
PYTHONPATH=. .venv/bin/python robots/lynsense/isaac/probe.py \
  --visual-review .artifacts/lynsense-isaac/<run-id> \
  --status accepted \
  --reviewer "<company reviewer identity>" \
  --notes "<specific visual acceptance notes>"
```

6. `Failure meanings`: unsupported version, absent explicit executable, archive/URDF/mesh failure, USD/import failure, missing joint/subtree, non-finite state, action timeout, blank render, and shutdown exception.
7. `Final milestone gate`: after an accepted review, show `--verify-milestone .artifacts/lynsense-isaac/<run-id>` and state that exit `0` is the only machine-readable milestone acceptance.
8. `Limits`: no navigation, full box task, viewer/video parity, calibrated physics, or real-robot readiness; generated company assets must not be committed.

- [ ] **Step 4: Run the complete focused suite**

Run:

```bash
.venv/bin/pytest -q tests/unit_tests/robots/lynsense/isaac
```

Expected: all new tests pass.

- [ ] **Step 5: Run broader offline regressions**

Run:

```bash
.venv/bin/pytest -q tests/unit_tests/robots/lynsense
.venv/bin/python -m compileall -q robots/lynsense/isaac
git diff --check
```

Expected: all Lynsense tests pass; compilation and diff checks exit zero.

If shared registry or package behavior was touched, also run:

```bash
.venv/bin/pytest -q tests/unit_tests/rpent/robots/test_registry_contracts.py
```

The expected result is pass; do not run the Webots Docker matrix unless a shared file outside `robots/lynsense/isaac` changed.

- [ ] **Step 6: Preserve the company gate state**

On this workstation, record in the final implementation report:

- the passing offline suite count;
- the real host command's run directory;
- `outcome="skipped"` and exit `77`;
- `is_pass=false`;
- no compatible company Isaac runtime was available;
- no `visual-review.json` exists yet.

The implementation task can complete while the milestone remains open. Do not claim the approved Isaac minimal-loop milestone is accepted until a company `2023.1.*` run and human visual review both pass.

- [ ] **Step 7: Review the full task diff**

Inspect every new file and any modified shared file. Run:

```bash
git status --short
git diff --check
git diff -- robots/lynsense/isaac tests/unit_tests/robots/lynsense/isaac
rg -n '[ \t]+$' robots/lynsense/isaac \
  tests/unit_tests/robots/lynsense/isaac || true
git check-ignore -v .artifacts/lynsense-isaac
```

Because the Isaac package and tests are new and therefore untracked, also inspect their complete contents with `sed` or the editor; `git diff` alone does not show new files. The `rg` command must return no matches, and `git check-ignore` must show `.artifacts/` ownership.

Do not reset, clean, commit, or push.

## Plan Validation Checklist

- Every task has a failing-test step, implementation step, focused test command, and diff review.
- Tasks 1-5 produce the complete offline contract surface before any Isaac API is imported.
- Task 5 keeps Isaac imports inside `Isaac3Runtime.run`, after `SimulationApp` starts.
- The host runner cannot promote a skip to a pass and cannot create technical visual acceptance.
- The final task preserves the Webots baseline and leaves the company Isaac and human visual gates explicit and open.
