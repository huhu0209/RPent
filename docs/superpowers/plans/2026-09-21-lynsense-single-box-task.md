# Lynsense Single-Box Task Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a simulation-only, serial single-box pick-and-place Webots gate while preserving the existing navigation, stability, isolation, and viewer behavior.

**Architecture:** Extend the isolated Lynsense Webots package with a primitive upper-body model, four simulation-only ROS actions, a dedicated manipulation runtime, and an RPent-owned py_trees runner. Keep navigation and manipulation mutually exclusive in the first gate; use a validated attachment for the end-to-end task and a separate world for physical grasp testing.

**Tech Stack:** Python 3.10, ROS 2 Humble actions and py_trees/py_trees_ros, Webots R2025a Supervisor/Motor/TouchSensor APIs, pytest, Docker Compose, ffmpeg.

**Spec:** `docs/superpowers/specs/2026-09-21-lynsense-single-box-task-design.md`

Plan status: self-reviewed and ready for user approval. Independent plan
review is `NOT RUN` because the configured subagent runtime timed out; it must
not be reported as passed.

## Global Constraints

- Keep runtime isolation at `ROS_DOMAIN_ID=42` and `ROS_LOCALHOST_ONLY=1`.
- Do not connect to robot one, use `ROS_DOMAIN_ID=3`, publish DDS ports, or source a robot workspace.
- Do not mount or copy `.env.lynsense`, SSH configuration, home directories, robot workspaces, or `../URDF/URDF_robot_1.tar.gz`.
- Do not commit or copy company STL/URDF meshes; use primitive Webots geometry.
- Preserve `interface`, `smoke`, `blocked`, and `match` behavior and their existing `all` sequence.
- First implementation is strictly serial: no navigation/manipulation parallel branches.
- Main-task box carry uses an attachment only after configured geometric checks pass.
- Physical contact grasp is tested in a separate `grasp-physics` gate.
- Perception and RPent Planner/LLM integration are out of scope.
- No commit, push, PR, or real-robot operation unless separately authorized.
- Use `apply_patch` for manual edits and `.venv/bin/pytest` for host unit tests.
- Include all uncommitted and new files in review evidence.

---

### Task 1: Add Action Contracts And Configuration Model

**Files:**

- Create: `robots/lynsense/simulation/ros/lynsense_utils_compat/action/MoveWaist.action`
- Create: `robots/lynsense/simulation/ros/lynsense_utils_compat/action/MoveNamedConfig.action`
- Create: `robots/lynsense/simulation/ros/lynsense_utils_compat/action/SetGripper.action`
- Create: `robots/lynsense/simulation/ros/lynsense_utils_compat/action/BoxPhase.action`
- Modify: `robots/lynsense/simulation/ros/lynsense_utils_compat/CMakeLists.txt`
- Create: `robots/lynsense/simulation/ros/lynsense_webots_sim/config/box.yaml`
- Create: `robots/lynsense/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/box_config.py`
- Test: `tests/unit_tests/robots/lynsense/simulation/test_box_config.py`

**Interfaces:**

- Consumes: ROS 2 `rosidl_default_generators`.
- Produces:
  - `lynsense_utils/action/MoveWaist`
  - `lynsense_utils/action/MoveNamedConfig`
  - `lynsense_utils/action/SetGripper`
  - `lynsense_utils/action/BoxPhase`
  - `load_box_config(path: Path) -> BoxConfig`
  - `BoxConfig`, `JointConfig`, `BoxGeometry`, `NamedTarget`, and threshold dataclasses with exact field names used by later tasks.

- [ ] **Step 1: Write failing configuration and action contract tests**

Create focused tests that read the four `.action` files and `box.yaml`:

```python
from pathlib import Path

import pytest

from lynsense_webots_sim.box_config import (
    ConfigError,
    load_box_config,
)


ROOT = (
    Path("robots")
    / "lynsense"
    / "simulation"
    / "ros"
)
ACTION_ROOT = ROOT / "lynsense_utils_compat" / "action"


@pytest.mark.parametrize(
    "name,fields",
    [
        ("MoveWaist.action", ["float64 height_mm", "bool success", "string message", "string phase", "float64 position_mm", "float64 remaining_mm"]),
        ("MoveNamedConfig.action", ["string target", "bool initok", "bool success", "string message", "string phase", "float64 progress", "float64 max_joint_error_rad"]),
        ("SetGripper.action", ["float64 position", "bool success", "string message", "string phase", "float64 position", "float64 remaining"]),
        ("BoxPhase.action", ["string action", "string flow", "string config", "bool initok", "bool success", "string message", "string phase", "float64 progress", "float64 box_error_m"]),
    ],
)
def test_action_contracts_are_explicit(name, fields):
    text = (ACTION_ROOT / name).read_text(encoding="utf-8")
    sections = text.split("---")
    assert [line.strip() for line in sections[0].splitlines() if line.strip()] == ["# Goal", fields[0]]
    assert [line.strip() for line in sections[1].splitlines() if line.strip()] == ["# Result", *fields[1:3]]
    assert [line.strip() for line in sections[2].splitlines() if line.strip()] == ["# Feedback", *fields[3:]]


def test_box_config_loads_complete_model():
    config = load_box_config(ROOT / "lynsense_webots_sim" / "config" / "box.yaml")
    assert config.box.dimensions_m == (0.397, 0.295, 0.217)
    assert config.box.mass_kg == pytest.approx(1.0)
    assert config.waist.motor_name == "waist_motor"
    assert config.left_arm.joint_names == tuple(f"left_joint{i}" for i in range(1, 7))
    assert config.right_arm.joint_names == tuple(f"right_joint{i}" for i in range(1, 7))
    assert set(config.named_targets) == {
        "dualjo:joints_s",
        "dualjo:joints_br",
        "dualposi_armbase_abso:pt_1f1_ready",
        "dualposi_armbase_abso:pt_up",
    }
    assert config.thresholds.grasp_alignment_m == pytest.approx(0.03)
    assert config.thresholds.carry_drift_m == pytest.approx(0.02)


def test_box_config_rejects_duplicate_and_out_of_range_joints(tmp_path):
    source = (ROOT / "lynsense_webots_sim" / "config" / "box.yaml").read_text(encoding="utf-8")
    broken = source.replace("left_joint2: -1.120501", "left_joint1: -1.120501")
    path = tmp_path / "box.yaml"
    path.write_text(broken, encoding="utf-8")
    with pytest.raises(ConfigError, match="duplicate joint"):
        load_box_config(path)
```

- [ ] **Step 2: Run the failing tests**

Run:

```bash
.venv/bin/pytest -q tests/unit_tests/robots/lynsense/simulation/test_box_config.py
```

Expected: import failures for `lynsense_webots_sim.box_config`, followed by missing `.action` assertions after the import is added.

- [ ] **Step 3: Add action definitions**

Use these exact files:

```text
# Goal
float64 height_mm
---
# Result
bool success
string message
---
# Feedback
string phase
float64 position_mm
float64 remaining_mm
```

```text
# Goal
string target
bool initok
---
# Result
bool success
string message
---
# Feedback
string phase
float64 progress
float64 max_joint_error_rad
```

```text
# Goal
float64 position
---
# Result
bool success
string message
---
# Feedback
string phase
float64 position
float64 remaining
```

```text
# Goal
string action
string flow
string config
bool initok
---
# Result
bool success
string message
---
# Feedback
string phase
float64 progress
float64 box_error_m
```

Extend `rosidl_generate_interfaces` in `CMakeLists.txt` with all four files.

- [ ] **Step 4: Add the initial configuration model**

Create `box.yaml` with this structure and use these initial teach values:

```yaml
box:
  dimensions_m: [0.397, 0.295, 0.217]
  mass_kg: 1.0
  initial_pose_frame: world
  initial_pose_m: [1.674931, -2.493846, 0.1085]
  initial_yaw_rad: 0.0
  place_target_frame: world
  place_target_m: [2.810559, -2.734170, 0.1085]
  place_target_yaw_rad: 0.006126

waist:
  motor_name: waist_motor
  position_sensor_name: waist_sensor
  lower_m: -0.478
  upper_m: -0.018
  max_velocity_mps: 0.0248
  height_reference_m: -0.478
  height_positive_up: true
  arrival_tolerance_m: 0.003

left_arm:
  joint_names: [left_joint1, left_joint2, left_joint3, left_joint4, left_joint5, left_joint6]
  max_velocity_radps: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
  max_acceleration_radps2: [2.0, 2.0, 2.0, 2.0, 2.0, 2.0]
  limits_rad:
    left_joint1: [-6.28318530718, 6.28318530718]
    left_joint2: [-2.3038346, 2.3038346]
    left_joint3: [-4.2236968, 0.061087]
    left_joint4: [-6.28318530718, 6.28318530718]
    left_joint5: [-2.1642, 2.1642]
    left_joint6: [-6.28318530718, 6.28318530718]

right_arm:
  joint_names: [right_joint1, right_joint2, right_joint3, right_joint4, right_joint5, right_joint6]
  max_velocity_radps: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
  max_acceleration_radps2: [2.0, 2.0, 2.0, 2.0, 2.0, 2.0]
  limits_rad:
    right_joint1: [-6.28318530718, 6.28318530718]
    right_joint2: [-2.33175986832, 2.27590933197]
    right_joint3: [-3.44702528286, 0.837758517137]
    right_joint4: [-6.28318530718, 6.28318530718]
    right_joint5: [-2.1642, 2.1642]
    right_joint6: [-6.28318530718, 6.28318530718]

grippers:
  left:
    motor_name: left_gripper_motor
    position_sensor_name: left_gripper_sensor
    closed_position: 0.0
    open_position: 0.6
    arrival_tolerance: 0.05
    max_velocity: 1.0
  right:
    motor_name: right_gripper_motor
    position_sensor_name: right_gripper_sensor
    closed_position: 0.0
    open_position: 0.6
    arrival_tolerance: 0.05
    max_velocity: 1.0

named_targets:
  dualjo:joints_s:
    waist_height_mm: 200.0
    gripper_position: 0.0
    left_joints_rad: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    right_joints_rad: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  dualjo:joints_br:
    waist_height_mm: 200.0
    gripper_position: 0.0
    left_joints_rad: [1.589995, -1.120501, -2.979277, -1.382301, -0.141372, 1.32645]
    right_joints_rad: [-1.589995, -1.120501, -2.979277, -1.382301, -0.141372, 1.32645]
  dualposi_armbase_abso:pt_1f1_ready:
    waist_height_mm: 200.0
    gripper_position: 0.0
    left_joints_rad: [1.589995, -1.295034, -2.979277, -1.382301, -0.141372, 1.32645]
    right_joints_rad: [-1.589995, -1.295034, -2.979277, -1.382301, -0.141372, 1.32645]
  dualposi_armbase_abso:pt_up:
    waist_height_mm: 200.0
    gripper_position: 0.0
    left_joints_rad: [-0.458970, 0.050475, -0.521966, -0.202441, 0.193761, 0.076757]
    right_joints_rad: [0.458970, 0.050475, -0.521966, -0.202441, 0.193761, 0.076757]

thresholds:
  joint_arrival_rad: 0.03
  waist_arrival_m: 0.003
  gripper_arrival: 0.05
  grasp_alignment_m: 0.03
  grasp_yaw_rad: 0.174533
  carry_drift_m: 0.02
  place_alignment_m: 0.05
  place_yaw_rad: 0.174533
  release_roll_pitch_rad: 0.087266
  release_settle_s: 0.5

timeouts:
  waist_s: 30.0
  named_config_s: 60.0
  gripper_s: 15.0
  box_phase_s: 45.0
  stall_window_s: 1.0
  joint_stall_threshold_rad: 0.003
  total_budget_s: 180.0
```

Implement `box_config.py` as a strict dataclass loader. Reject missing keys, duplicate joint names, non-finite values, wrong cardinality, targets outside limits, unknown named targets, non-positive timeouts, and thresholds outside `$(0, pi]` where applicable. Normalize all distance units to metres internally except the public `height_mm` action value.

- [ ] **Step 5: Run focused tests and diff check**

Run:

```bash
.venv/bin/pytest -q tests/unit_tests/robots/lynsense/simulation/test_box_config.py
git diff --check
```

Expected: all focused tests pass and diff check is clean.

---

### Task 2: Build Primitive Upper-Body And Box Worlds

**Files:**

- Create: `robots/lynsense/simulation/ros/lynsense_webots_sim/worlds/lynsense_box.wbt`
- Create: `robots/lynsense/simulation/ros/lynsense_webots_sim/worlds/lynsense_grasp_physics.wbt`
- Test: `tests/unit_tests/robots/lynsense/simulation/test_box_worlds.py`
- Test: modify `tests/unit_tests/robots/lynsense/simulation/test_viewer_worlds.py`

**Interfaces:**

- Consumes: `box.yaml` joint/motor names and geometry.
- Produces Webots devices and Supervisor names:
  - `waist_motor`, `waist_sensor`
  - `left_joint1..6_motor`, `left_joint1..6_sensor`
  - `right_joint1..6_motor`, `right_joint1..6_sensor`
  - `left_gripper_motor`, `right_gripper_motor`
  - `BOX_1`
  - `LEFT_GRIPPER_CONTACT`, `RIGHT_GRIPPER_CONTACT`

- [ ] **Step 1: Write failing world-contract tests**

```python
from pathlib import Path


ROOT = (
    Path("robots")
    / "lynsense"
    / "simulation"
    / "ros"
    / "lynsense_webots_sim"
    / "worlds"
)


def _world(name):
    return (ROOT / name).read_text(encoding="utf-8")


def test_box_world_declares_all_actuators_and_box():
    source = _world("lynsense_box.wbt")
    for name in ("waist_motor", "left_gripper_motor", "right_gripper_motor"):
        assert f'name "{name}"' in source
    for side in ("left", "right"):
        for index in range(1, 7):
            assert f'name "{side}_joint{index}_motor"' in source
            assert f'name "{side}_joint{index}_sensor"' in source
    assert source.count("LinearMotor {") == 1
    assert source.count("RotationalMotor {") == 16
    assert "DEF BOX_1 Solid" in source
    assert "size 0.397 0.295 0.217" in source
    assert source.count("physics Physics {") >= 3


def test_grasp_world_fixes_base_and_exposes_contact_sensors():
    source = _world("lynsense_grasp_physics.wbt")
    assert 'name "EA200_BOX"' in source
    assert 'name "LEFT_GRIPPER_CONTACT"' in source
    assert 'name "RIGHT_GRIPPER_CONTACT"' in source
    assert "TouchSensor {" in source
    assert "DEF BOX_1 Solid" in source
```

- [ ] **Step 2: Run tests and verify failure**

Run:

```bash
.venv/bin/pytest -q tests/unit_tests/robots/lynsense/simulation/test_box_worlds.py
```

Expected: both tests fail because the worlds do not exist.

- [ ] **Step 3: Create `lynsense_box.wbt`**

Start from `lynsense_match.wbt` so the corrected wheel geometry, contact settings, floor, goals, and viewer grid remain identical. Then:

1. rename `EA200_SMOKE` to `EA200_BOX`;
2. retain both wheel `HingeJoint` trees unchanged;
3. replace the two fixed body boxes with the same visual proportions plus actuated children;
4. add a `SlidingJoint` with `axis 0 0 1`, anchor matching `connector_joint`, `minPosition -0.478`, `maxPosition -0.018`, and `maxVelocity 0.0248`;
5. add left and right six-joint chains using primitive `Cylinder`, `Box`, or `Capsule` visual/collision geometry;
6. mirror every motor with a `PositionSensor`;
7. add one master gripper motor per side and two opposing pad solids;
8. add `DEF BOX_1 Solid` at the configured initial pose, with a `Box` bounding object, contact material, mass `1.0`, and finite inertia;
9. keep all remote URLs, `EXTERNPROTO`, and company mesh references absent.

Use bounded motor torque values in the world; do not rely on Webots default torque. Set initial waist/arm/gripper positions explicitly to the `dualjo:joints_s` target.

- [ ] **Step 4: Create `lynsense_grasp_physics.wbt`**

Copy `lynsense_box.wbt`, then:

1. remove the two wheel `HingeJoint` devices and replace wheel solids with fixed-radius support solids;
2. fix the robot root with a non-physics support pose or fixed base equivalent suitable for Webots;
3. retain waist, arms, grippers, and box physics;
4. add two `TouchSensor` nodes named `LEFT_GRIPPER_CONTACT` and `RIGHT_GRIPPER_CONTACT` on opposing finger pads;
5. place the box at the configured grasp pose.

- [ ] **Step 5: Update viewer world tests**

Extend `WORLD_CASES` and any grid assertions so `lynsense_box.wbt` shares the visual-only grid and corrected wheel contract. Do not add the fixed-base physics world to navigation viewer cases.

- [ ] **Step 6: Run focused tests**

Run:

```bash
.venv/bin/pytest -q tests/unit_tests/robots/lynsense/simulation/test_box_worlds.py tests/unit_tests/robots/lynsense/simulation/test_viewer_worlds.py tests/unit_tests/robots/lynsense/simulation/test_wheel_geometry.py
```

Expected: all pass.

---

### Task 3: Implement Manipulation Telemetry And State Machine

**Files:**

- Create: `robots/lynsense/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/manipulation_runtime.py`
- Test: `tests/unit_tests/robots/lynsense/simulation/test_manipulation_runtime.py`

**Interfaces:**

- Consumes: `BoxConfig` from Task 1.
- Produces:

```python
class ManipulationKind(enum.Enum):
    WAIST = "waist"
    NAMED_CONFIG = "named_config"
    GRIPPER = "gripper"
    BOX_PHASE = "box_phase"

@dataclass(frozen=True)
class ManipulationGoal:
    kind: ManipulationKind
    height_mm: float | None = None
    target: str | None = None
    gripper_position: float | None = None
    box_action: str | None = None
    flow: str | None = None
    config_name: str | None = None

@dataclass(frozen=True)
class ManipulationState:
    sim_time_s: float
    waist_position_m: float
    arm_positions_rad: tuple[float, ...]
    gripper_positions: tuple[float, float]
    box_position_m: tuple[float, float, float]
    box_orientation_rad: tuple[float, float, float]
    robot_pose: Pose2
    box_attached: bool

@dataclass(frozen=True)
class ManipulationObservation:
    phase: str
    outcome: Outcome
    reason: str
    terminal: bool
    success: bool
    message: str
    feedback_kind: str
    position_mm: float = 0.0
    remaining_mm: float = 0.0
    progress: float = 0.0
    max_joint_error_rad: float = 0.0
    gripper_position: float = 0.0
    gripper_remaining: float = 0.0
    box_error_m: float = 0.0
```

- `ManipulationRuntime(config, initial_time_s)` exposes:
  - `submit(goal: ManipulationGoal, state: ManipulationState) -> Acceptance`
  - `request_cancel() -> bool`
  - `observe(state: ManipulationState) -> ManipulationObservation`
  - `commands() -> ManipulationCommands`
- `ManipulationCommands` contains waist velocity, 12 arm velocities, two gripper velocities, and `attach_box`/`release_box` booleans.

- [ ] **Step 1: Write normal-path and failure-path tests**

Cover at least:

```python
def test_waist_moves_and_settles(runtime, state_factory):
    assert runtime.submit(goal_waist(200.0), state_factory(waist=-0.478)).accepted
    first = runtime.observe(state_factory(waist=-0.478))
    assert first.outcome is Outcome.RUNNING
    assert first.phase == "moving"
    done = runtime.observe(state_factory(waist=-0.2781))
    assert done.terminal and done.success and done.phase == "settling"


def test_named_config_rejects_unknown_target(runtime, state_factory):
    result = runtime.submit(goal_named("dualjo:not_allowed"), state_factory())
    assert not result.accepted
    assert result.reason == "unknown_target"


def test_pick_requires_alignment_and_closed_grippers(runtime, aligned_state):
    runtime.submit(goal_pick(), aligned_state)
    runtime.observe(aligned_state)
    assert runtime.commands().attach_box is False
    attached = runtime.observe(replace(aligned_state, box_attached=True))
    assert attached.success


def test_pick_fails_when_box_is_misaligned(runtime, misaligned_state):
    runtime.submit(goal_pick(), misaligned_state)
    result = runtime.observe(misaligned_state)
    assert result.terminal and not result.success
    assert result.reason == "box_alignment_failed"
```

Add table-driven failure cases for every reason in the spec: `joint_stall`, `motion_timeout`, `grasp_failed`, `place_alignment_failed`, `carry_lost`, and `release_settle_failed`.

- [ ] **Step 2: Run tests to verify failure**

Run:

```bash
.venv/bin/pytest -q tests/unit_tests/robots/lynsense/simulation/test_manipulation_runtime.py
```

Expected: module import fails.

- [ ] **Step 3: Implement pure state transitions**

Keep this module free of `rclpy`, Webots, ROS message, and file I/O imports. Use only `BoxConfig`, dataclasses, `math`, `enum`, and threading. Enforce:

- one active goal;
- finite state values;
- serial terminal lockout;
- bounded monotonic progress;
- non-negative feedback remainders;
- cancellation before terminal state;
- attached retained on failure;
- zero command velocities on terminal and failed states;
- release only after all place preconditions.

Do not mutate Webots objects here.

- [ ] **Step 4: Run focused tests**

Run:

```bash
.venv/bin/pytest -q tests/unit_tests/robots/lynsense/simulation/test_manipulation_runtime.py
```

Expected: all pass.

---

### Task 4: Add Supervisor Device Adapter And Attachment Control

**Files:**

- Create: `robots/lynsense/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/webots_manipulation_adapter.py`
- Test: `tests/unit_tests/robots/lynsense/simulation/test_webots_manipulation_adapter.py`

**Interfaces:**

- Produces:

```python
class WebotsManipulationAdapter:
    def __init__(self, robot: object, config: BoxConfig) -> None: ...
    def read(self, robot_pose: Pose2, sim_time_s: float) -> ManipulationState: ...
    def write(self, commands: ManipulationCommands) -> None: ...
    def attach(self) -> None: ...
    def release(self) -> None: ...
    def stop_all(self) -> None: ...
```

- Uses Supervisor node `BOX_1`, all listed motors and sensors, and fixed forward kinematics from the robot root for carrying-frame checks.

- [ ] **Step 1: Write fake-device tests**

Create fake `Motor`, `PositionSensor`, and `Node` classes. Assert:

1. all 17 motors and 15 sensors are acquired exactly once;
2. every wheel and upper-body motor is set to velocity mode on initialization;
3. sensor readings become a `ManipulationState`;
4. finite device values are enforced;
5. `write()` maps command values to matching motors without cross-device writes;
6. `stop_all()` writes zero to every motor;
7. `attach()` saves the box-to-root transform;
8. attached `read()` computes the box from that transform;
9. `release()` clears the transform and does not teleport the box.

Use a recorded-call fake, not a mock that asserts only interactions.

- [ ] **Step 2: Run failing adapter tests**

Run:

```bash
.venv/bin/pytest -q tests/unit_tests/robots/lynsense/simulation/test_webots_manipulation_adapter.py
```

- [ ] **Step 3: Implement the adapter**

Acquire devices lazily but cache them. Read sensor values with `getValue()`. For attached mode, compute the world box pose from the saved robot-relative transform on every read. Preserve current box roll/pitch unless the runtime requests release. All exceptions become Python exceptions; ROS fault translation belongs to the controller integration.

- [ ] **Step 4: Run focused tests**

Run:

```bash
.venv/bin/pytest -q tests/unit_tests/robots/lynsense/simulation/test_webots_manipulation_adapter.py
```

Expected: all pass.

---

### Task 5: Integrate Six Actions Into The Controller

**Files:**

- Modify: `robots/lynsense/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/webots_controller.py`
- Test: `tests/unit_tests/robots/lynsense/simulation/test_controller_integration.py`
- Test: modify `tests/unit_tests/robots/lynsense/simulation/test_action_runtime.py`

**Interfaces:**

- Consumes `ManipulationRuntime` and `WebotsManipulationAdapter`.
- Produces six action endpoints:
  - existing `/lynsense/nav_to_pose`
  - existing `/lynsense/move_distance`
  - `/lynsense/move_waist`
  - `/lynsense/move_named_config`
  - `/lynsense/set_gripper`
  - `/lynsense/box_phase`
- Produces controller-ready telemetry containing wheel rates plus waist, 12 arm joints, two grippers, and box state.

- [ ] **Step 1: Write controller tests**

Use fake action bindings and a fake Webots robot to verify:

```python
def test_controller_ready_lists_six_actions(controller):
    event = controller.ready_event()
    assert event["action_names"] == [
        "nav_to_pose", "move_distance", "move_waist",
        "move_named_config", "set_gripper", "box_phase",
    ]
    assert event["box"]["attached"] is False


def test_navigation_rejects_manipulation_goal_while_active(controller):
    controller.submit_nav("搬箱子1")
    result = controller.submit_manipulation(goal_pick())
    assert not result.accepted
    assert result.reason == "navigation_active"


def test_manipulation_rejects_navigation_goal_while_active(controller):
    controller.submit_manipulation(goal_pick())
    result = controller.submit_nav("放箱子1_1")
    assert not result.accepted
    assert result.reason == "manipulation_active"


def test_fault_stops_wheels_waist_arms_and_grippers(controller):
    controller.force_fault("simulation_fault:test")
    assert controller.zero_velocity_motor_names == {
        "left_wheel_motor", "right_wheel_motor", "waist_motor",
        *(f"{side}_joint{i}_motor" for side in ("left", "right") for i in range(1, 7)),
        "left_gripper_motor", "right_gripper_motor",
    }
```

Also test all six goal descriptors, feedback/result field mapping, terminal lockout, cancellation, and cleanup idempotence.

- [ ] **Step 2: Run failing controller tests**

Run:

```bash
.venv/bin/pytest -q tests/unit_tests/robots/lynsense/simulation/test_controller_integration.py
```

- [ ] **Step 3: Generalize active-goal ownership**

Refactor the controller without changing navigation semantics:

1. replace the exact-two-server check with a required-endpoint map;
2. introduce one active resource mode: `IDLE`, `NAVIGATION`, or `MANIPULATION`;
3. reject cross-mode and second-goal submissions while non-idle;
4. keep navigation event names and existing field compatibility;
5. add manipulation event fields without removing existing fields;
6. stop all motors on shutdown/fault;
7. publish one feedback event per controller step and one terminal result event;
8. retain attachment on manipulation failure.

Build action servers only when the selected script is `box`; `interface`, `smoke`, `blocked`, and `match` continue to advertise only the two navigation actions so existing interface isolation remains comparable.

- [ ] **Step 4: Run controller and regression tests**

Run:

```bash
.venv/bin/pytest -q tests/unit_tests/robots/lynsense/simulation/test_controller_integration.py tests/unit_tests/robots/lynsense/simulation/test_action_runtime.py tests/unit_tests/robots/lynsense/simulation/test_events.py
```

Expected: all pass.

---

### Task 6: Add RPent-Owned Tree Behaviours And Runner

**Files:**

- Create: `robots/lynsense/simulation/ros/lynsense_webots_sim/trees/lynsense_single_box_tree.xml`
- Create: `robots/lynsense/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/tree_behaviours.py`
- Create: `robots/lynsense/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/scripts/run_single_box_tree.py`
- Modify: `robots/lynsense/simulation/ros/lynsense_webots_sim/setup.py`
- Test: `tests/unit_tests/robots/lynsense/simulation/test_tree_behaviours.py`

**Interfaces:**

- Produces `lynsense_single_box_tree` console entry point.
- Produces behaviour classes:
  - `EnsureSimulationManipulationServices`
  - `SimulationNavToPose`
  - `SimulationMoveDistance`
  - `SimulationPlanMoveWaist`
  - `SimulationPlanMoveNamedConfig`
  - `SimulationPlanSetGripper`
  - `SimulationPlanBoxPhase`

- [ ] **Step 1: Write failing XML and adapter tests**

Parse the XML with `xml.etree.ElementTree` and assert:

1. root is a serial `Sequence`;
2. no `Parallel` element exists;
3. node order exactly matches the spec;
4. every manipulation node maps to the expected endpoint;
5. no node text contains `plan_arm_simple`, `subprocess`, or `ros2 run`.

Use fake action clients to test that each behaviour:

- waits for its server;
- sends one goal;
- logs accepted goal and feedback;
- returns `SUCCESS` for succeeded results;
- returns `FAILURE` for rejected/failed results;
- remains `RUNNING` while waiting;
- cancels on `halt()` and remains running until cancellation completes.

- [ ] **Step 2: Run tests to verify failure**

Run:

```bash
.venv/bin/pytest -q tests/unit_tests/robots/lynsense/simulation/test_tree_behaviours.py
```

- [ ] **Step 3: Create the tree**

Use the exact serial sequence:

```xml
<py_trees.composites.Sequence name="SingleBoxSerialTask" memory="True">
  <EnsureSimulationManipulationServices name="EnsureSimulationServices" />
  <SimulationNavToPose goal_name="搬箱子1" name="NavigateToBox1" />
  <SimulationPlanMoveWaist height_mm="200" name="PrepareWaist" />
  <SimulationPlanSetGripper position="0.0" name="CloseGrippersForPrepare" />
  <SimulationPlanMoveNamedConfig target="dualjo:joints_br" name="PrepareDualArms" />
  <SimulationPlanBoxPhase action="pick" flow="flow" config="box1" name="PickBox1" />
  <SimulationMoveDistance distance="-0.6" angle="0" name="BackOffFromBox1" />
  <SimulationMoveDistance distance="0.0" angle="90" name="TurnAfterBox1" />
  <SimulationPlanMoveNamedConfig target="dualposi_armbase_abso:pt_1f1_ready" name="PreparePlacement" />
  <SimulationNavToPose goal_name="放箱子1_1" name="NavigateToBox1Drop" />
  <SimulationPlanBoxPhase action="place" flow="flow" config="box1" name="PlaceBox1" />
  <SimulationMoveDistance distance="-0.6" angle="0" name="RetreatAfterPlacement" />
  <SimulationPlanMoveNamedConfig target="dualposi_armbase_abso:pt_up" name="RaiseArmsAfterPlacement" />
  <SimulationPlanMoveWaist height_mm="200" name="RestoreWaist" />
  <SimulationPlanSetGripper position="0.0" name="RestoreGrippers" />
  <SimulationPlanMoveNamedConfig target="dualjo:joints_s" name="RestoreTravelPosture" />
</py_trees.composites.Sequence>
```

- [ ] **Step 4: Implement Action-client behaviours**

Create one reusable client helper rather than six unrelated clients. Keep all ROS imports inside `main()` or behaviour setup so offline tests can import the module without ROS installed. Resolve actions from `lynsense_utils.action`. Shutdown must cancel active goals before destroying clients and node.

- [ ] **Step 5: Run focused tests**

Run:

```bash
.venv/bin/pytest -q tests/unit_tests/robots/lynsense/simulation/test_tree_behaviours.py
```

Expected: all pass.

---

### Task 7: Add Box Orchestrator Phase And Evidence Gate

**Files:**

- Modify: `robots/lynsense/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/scripts/run_smoke.py`
- Modify: `robots/lynsense/simulation/docker/bootstrap.py`
- Modify: `robots/lynsense/simulation/docker/bootstrap_viewer.py`
- Test: `tests/unit_tests/robots/lynsense/simulation/test_box_orchestrator.py`
- Test: modify existing orchestrator/static tests that assert phase maps.

**Interfaces:**

- Produces `--phase box` and `--phase box-all`.
- `all` remains exactly `interface -> smoke -> blocked -> match`.
- `box-all` remains exactly `interface -> smoke -> blocked -> match -> box`.
- Produces `events-box.jsonl` and unified phase summaries.

- [ ] **Step 1: Write failing phase-map tests**

Assert:

```python
def test_box_phase_uses_dedicated_world_config_tree_and_events():
    assert run_smoke.CONFIG_PATHS["box"].name == "box.yaml"
    assert run_smoke.WORLD_PATHS["box"].name == "lynsense_box.wbt"
    assert run_smoke.TREE_NAMES["box"] == "lynsense_single_box_tree.xml"
    assert run_smoke.EVENT_NAMES["box"] == "events-box.jsonl"


def test_all_does_not_include_box_and_box_all_does():
    assert run_smoke.PHASE_SEQUENCE["all"] == ("interface", "smoke", "blocked", "match")
    assert run_smoke.PHASE_SEQUENCE["box-all"] == (*run_smoke.PHASE_SEQUENCE["all"], "box")
```

Add event-validator tests for the exact 15-action serial script and final box assertions.

- [ ] **Step 2: Run failing orchestrator tests**

Run:

```bash
.venv/bin/pytest -q tests/unit_tests/robots/lynsense/simulation/test_box_orchestrator.py
```

- [ ] **Step 3: Extend readiness and validation**

For `box`, require all six action endpoints. Generalize `EXPECTED_ACTION_TYPES` per phase. Generalize expected action matching to include:

```python
("move_waist", {"height_mm": 200.0})
("move_named_config", {"target": "dualjo:joints_br"})
("set_gripper", {"position": 0.0})
("box_phase", {"action": "pick", "flow": "flow", "config": "box1"})
```

Extend `TreeEventValidator` so terminal success also checks:

- final robot pose near the retreat pose computed from the configured script;
- all recorded wheel, waist, arm, and gripper rates are zero;
- pick result records `box.attached is True`;
- place result records `box.attached is False`;
- final box pose is within the configured place tolerances;
- post-release samples span at least `0.5 s`.

Use a 180-second tree deadline for `box`.

- [ ] **Step 4: Start the RPent-owned runner for `box`**

Add `SINGLE_BOX_TREE_PATH` from the ephemeral `lynsense_webots_sim` install. `_start_tree(phase=...)` must select that executable for `box` and continue selecting the existing pytrees executable for prior phases.

- [ ] **Step 5: Extend bootstrap phases**

Add `box` and `box-all` to delegated phases. Keep `all` unchanged. Copy/install the RPent-owned tree through the normal package data mechanism rather than writing into the read-only sibling checkout.

- [ ] **Step 6: Run focused and full unit suites**

Run:

```bash
.venv/bin/pytest -q tests/unit_tests/robots/lynsense/simulation
git diff --check
```

Expected: all pass.

---

### Task 8: Add Isolated Mechanical And Physical Grasp Regressions

**Files:**

- Create: `robots/lynsense/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/scripts/run_mechanical.py`
- Create: `robots/lynsense/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/scripts/run_grasp_physics.py`
- Test: `tests/unit_tests/robots/lynsense/simulation/test_isolated_regressions.py`
- Modify: `robots/lynsense/simulation/README.md`

**Interfaces:**

- Produces two direct Supervisor probes with no ROS graph and no real-robot connection.
- Mechanical probe exits nonzero on missing devices, non-arrival, stall, or non-zero terminal rates.
- Physical probe exits nonzero if either contact sensor loses contact during the lift hold or the box does not fall after release.

- [ ] **Step 1: Write pure summarizer tests**

Both scripts should expose `summarize(samples)` for offline tests. Test:

- complete sample cadence;
- finite values;
- every named target reached;
- terminal rates zero;
- both contacts present through the required hold;
- contact loss failure;
- box height increases after grasp;
- box falls or becomes dynamic after release.

Reject sparse recordings and sampling gaps using the same completeness pattern as `run_stability.py`.

- [ ] **Step 2: Implement direct probes**

Use `xvfb-run`, Webots fast mode, and the `webots-controller` launcher pattern from `run_stability.py`. Do not introduce ROS actions or a tree process. Start from settled states, command bounded trajectories, and write `samples.json` plus `summary.json`.

- [ ] **Step 3: Run container probes**

From `robots/lynsense/simulation/docker`:

```bash
docker compose run --rm --no-deps --entrypoint python3 lynsense-webots-smoke \
  /workspace/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/scripts/run_mechanical.py \
  --world /workspace/simulation/ros/lynsense_webots_sim/worlds/lynsense_box.wbt \
  --output /workspace/artifacts/mechanical-check

docker compose run --rm --no-deps --entrypoint python3 lynsense-webots-smoke \
  /workspace/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/scripts/run_grasp_physics.py \
  --world /workspace/simulation/ros/lynsense_webots_sim/worlds/lynsense_grasp_physics.wbt \
  --output /workspace/artifacts/grasp-physics-check
```

Expected: both summaries have `"passed": true`.

- [ ] **Step 4: Tune only declared simulation parameters**

If a probe fails, adjust only primitive geometry, configured joint targets, motor limits, contact materials, or thresholds in `box.yaml`. Record the changed value and reason in the artifact summary. Do not weaken the sample-completeness or terminal-rate checks.

---

### Task 9: Run The End-To-End Box Gate

**Files:**

- Modify only if a concrete failure requires it: controller, runtime, world, config, tree, or orchestrator files from Tasks 1-7.
- Evidence: `.artifacts/lynsense-webots/events-box.jsonl`
- Evidence: `.artifacts/lynsense-webots/summary.json`

**Interfaces:**

- Consumes all previous tasks.
- Produces a passing `--phase box` run.

- [ ] **Step 1: Build the overlay**

Run:

```bash
docker compose run --rm lynsense-webots-smoke --phase build
```

- [ ] **Step 2: Run the box gate**

Run:

```bash
docker compose run --rm lynsense-webots-smoke --phase box
```

Expected exit code `0`, with every action result successful and terminal motor rates zero.

- [ ] **Step 3: Validate artifact content**

Use `jq` or equivalent read-only checks to confirm:

- exact 15-action sequence;
- pick attached state;
- place detached state;
- final box position and yaw tolerances;
- at least `0.5 s` post-release samples;
- no NaN or infinity in JSON;
- failure reason empty at terminal success.

- [ ] **Step 4: Run original and extended matrices**

Run:

```bash
docker compose run --rm lynsense-webots-smoke --phase all
docker compose run --rm lynsense-webots-smoke --phase box-all
```

Expected: `all` and `box-all` both pass.

---

### Task 10: Extend Viewer For The Box Phase

**Files:**

- Modify: `robots/lynsense/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/scripts/run_viewer.py`
- Modify: `robots/lynsense/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/viewer_state.py`
- Modify: `robots/lynsense/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/viewer_server.py`
- Modify: `robots/lynsense/simulation/viewer/static/app.js`
- Modify: `robots/lynsense/simulation/viewer/static/app.css`
- Modify: `robots/lynsense/simulation/viewer/index.html`
- Test: `tests/unit_tests/robots/lynsense/simulation/test_viewer_box.py`
- Test: modify viewer runtime/state/server/asset tests.

**Interfaces:**

- Produces `LYNSENSE_VIEWER_PHASE=box`.
- Produces replay routes unchanged and additional telemetry fields for waist, grippers, arms, box pose, and attached state.

- [ ] **Step 1: Write failing viewer tests**

Assert:

- phase allowlist includes `box`;
- map uses `box.yaml`;
- event state exposes box and manipulation telemetry;
- API remains read-only;
- no new mutation endpoint exists;
- page assets contain box state controls or labels;
- static vendor files remain local.

- [ ] **Step 2: Extend state parsing**

Normalize manipulation events into existing viewer state without breaking navigation-only phases. Missing upper-body fields must render as `-` for old replays, not as `undefined`.

- [ ] **Step 3: Extend UI**

Add compact telemetry to the existing operational layout:

- task phase;
- waist mm;
- left/right gripper positions;
- max arm joint error;
- box `x/y/yaw`;
- attached state.

Do not add cards inside cards, mutation controls, feature explanations, or marketing copy. Keep existing view switching and replay time behavior.

- [ ] **Step 4: Run browser verification**

Start the viewer for `box`, then use Playwright to verify:

- page loads without horizontal overflow at 1440x900 and 390x844;
- scene and robot streams are nonblank;
- both 960x540 replays play;
- view switching preserves playback position;
- box state changes during replay;
- no console error other than the known optional favicon 404;
- replay files are playable with `ffprobe`.

- [ ] **Step 5: Run viewer regression**

Run viewer tests for `smoke`, `blocked`, `match`, and `box`. Confirm prior phase behavior and replay availability remain intact.

---

### Task 11: Documentation, Final Regression, And Review

**Files:**

- Modify: `robots/lynsense/simulation/README.md`
- Create: `docs/superpowers/plans/2026-09-21-lynsense-single-box-acceptance.md`

**Interfaces:**

- Produces reproducible commands, artifact locations, limitations, and final evidence.

- [ ] **Step 1: Document operation and limits**

Add sections covering:

- `--phase box` and `--phase box-all`;
- mechanical and physical grasp probes;
- serial-versus-competition-tree difference;
- attachment versus physical contact distinction;
- box tolerances;
- viewer phase;
- no real-robot or perception claim.

- [ ] **Step 2: Run the complete host suite**

Run:

```bash
.venv/bin/pytest -q tests/unit_tests/robots/lynsense/simulation
git diff --check
```

Expected: all tests pass.

- [ ] **Step 3: Run complete container matrix**

Run:

1. `--phase all`
2. chassis stability on `lynsense_smoke.wbt`
3. mechanical regression
4. physical grasp regression
5. `--phase box`
6. `--phase box-all`
7. viewer `box`

Record run IDs, durations, summaries, and any tuned parameters in the acceptance document.

- [ ] **Step 4: Inspect worktree**

Run:

```bash
git status --short
git diff --check
```

Preserve unrelated pre-existing changes. Do not revert user work.

- [ ] **Step 5: Independent implementation review**

Request a read-only review of all uncommitted/new simulation files and tests. Require the reviewer to check:

- real-robot isolation;
- no company asset copy;
- serial resource exclusivity;
- attachment preconditions;
- physical gate independence;
- event and summary completeness;
- viewer regression risk;
- test honesty.

Fix every blocker and important finding, rerun the affected gates, and obtain reviewer approval before user handoff.

---

## Execution Notes

- Tasks 1-4 can be implemented mostly in parallel only if writers keep to their listed files; Task 5 is the integration point and should have one writer.
- Task 2 world geometry and Task 1 configured target values may require joint tuning during Task 8. Treat a tuning change as a normal reviewed diff with probe evidence, not as a local uncommitted hand edit.
- If Webots primitive geometry cannot represent a required collision shape, use multiple primitive solids rather than importing company meshes.
- If the first physical grasp gate cannot pass, finish the hybrid end-to-end gate and report the physical gap explicitly; do not silently disable the physical gate.
- If a full runtime matrix cannot run in the available session, record the exact missing gate and do not mark the stage complete.
