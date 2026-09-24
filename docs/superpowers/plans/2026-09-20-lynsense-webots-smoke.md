# Lynsense Webots Smoke Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在本工作站容器中建立 Webots 最小导航仿真，并用既有 `lynsense_webots_tree.xml` 完整验证 Lynsense 导航 Action 链路。

**Architecture:** 以 `robots/lynsense/simulation/` 为边界新增仿真资产，不修改现有真机只读后端。纯 Python 模块先实现差速运动、阶段状态机、事件跟踪和输入校验；Webots Supervisor controller 只做 ROS 2 Action Adapter 和 Motor I/O。Ubuntu 22.04 + ROS 2 Humble + Webots R2025a 容器与外部网络隔离，现有 `lynsense_pytrees` 只读挂载。

**Tech Stack:** Python 3.10/pytest、ROS 2 Humble `rclpy`/rosidl actions、Webots R2025a Python Supervisor/Motor API、Docker Compose。

**Spec:** [Lynsense Webots 最小导航仿真设计](../../superpowers/specs/2026-09-20-lynsense-webots-smoke-design.md)

## Global Constraints

- 只在本工作站运行仿真；不连接机器人一号，不使用 `ROS_DOMAIN_ID=3`，不 source 机器人工作空间。
- 运行容器 `network_mode: none`，显式设置 `ROS_DOMAIN_ID=42`、`ROS_LOCALHOST_ONLY=1`。
- 不调用真实底盘、机械臂、夹爪、升降、使能、清错或状态修改接口。
- Webots 模型只包含 `left_wheel_motor` 和 `right_wheel_motor`；不得出现机械臂、夹爪或升降 Motor。
- 不挂载 `.env.lynsense`、SSH 配置、机器人工作空间或任何 API key。
- `lynsense_pytrees` 和 RPent 的 `robots/lynsense/simulation/` 子树以 read-only 挂载；不挂载 RPent 仓库根目录，build/install/log/artifacts 写入容器临时目录或本地 `.artifacts/`。
- 基础镜像固定为 `cyberbotics/webots@sha256:f0023e30daf38b172e4e6ad24ed345909bcd9551df34d63d824e121a7cebf099`。
- 不提交 `../URDF/URDF_robot_1.tar.gz` 或其中网格资产；模型用基础几何体重建。
- 容器内仿真使用 CR100 参数：轮距 `0.461 m`、轮半径 `0.08 m`、最大线速度 `2.0 m/s`、最大角速度 `1.5 rad/s`。
- `MoveDistance.angle` 是相对当前朝向的转角；正角逆时针，负角顺时针；先转向，后按 `distance` 前进/后退。
- 运动超时和停滞基于 Webots `Supervisor.getTime()` 仿真时间，不基于 GUI 壁钟。
- 自动提交、推送、上游 PR 均不允许；每个任务只保留工作区变更并记录验证结果。

## Baseline

- 分支：`feat/lynsense-readonly-state`，跟踪 `fork/feat/lynsense-readonly-state`。
- 当前新增文件：`docs/superpowers/specs/2026-09-20-lynsense-webots-smoke-design.md`。
- 本机宿主 Ubuntu 26.04 没有 `webots`、`ros2`、`xacro`；所有 ROS/Webots 验证必须在容器内执行。
- 项目本地 `.venv` 可运行 RPent 离线 pytest。
- `docs/codex-workflow-guide.md` 在当前仓库不存在；计划遵循 `AGENTS.md` 和已批准 spec。

## File Structure

| 文件（相对 RPent） | 责任 |
| --- | --- |
| `robots/lynsense/simulation/README.md` | 构建、运行、隔离边界和证据说明 |
| `robots/lynsense/simulation/ros/lynsense_utils_compat/package.xml` | 仿真专用 `lynsense_utils` overlay 包声明 |
| `robots/lynsense/simulation/ros/lynsense_utils_compat/CMakeLists.txt` | 生成两个 Action 接口 |
| `robots/lynsense/simulation/ros/lynsense_utils_compat/action/NavToPose.action` | NavToPose 契约 |
| `robots/lynsense/simulation/ros/lynsense_utils_compat/action/MoveDistance.action` | MoveDistance 契约 |
| `robots/lynsense/simulation/ros/lynsense_utils_compat/README.md` | 说明兼容包不是公司原始接口 |
| `robots/lynsense/simulation/ros/lynsense_webots_sim/package.xml` | Webots ROS Python 包声明 |
| `robots/lynsense/simulation/ros/lynsense_webots_sim/setup.py` | 安装 Python 模块、world、launch 和脚本 |
| `robots/lynsense/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/scripts/__init__.py` | console entry point 使用的 scripts 子包 |
| `robots/lynsense/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/geometry.py` | 2D pose、角度、差速正逆解 |
| `robots/lynsense/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/state_machine.py` | 阶段状态机、停滞、超时和命令限制 |
| `robots/lynsense/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/action_runtime.py` | 单 active goal、输入校验、cancel 和结果状态 |
| `robots/lynsense/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/events.py` | JSONL 事件格式与逐条 flush |
| `robots/lynsense/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/webots_controller.py` | Supervisor、rclpy Action Server 和 Motor I/O |
| `robots/lynsense/simulation/ros/lynsense_webots_sim/worlds/lynsense_smoke.wbt` | 成功 world |
| `robots/lynsense/simulation/ros/lynsense_webots_sim/worlds/lynsense_blocked.wbt` | 受阻失败 world |
| `robots/lynsense/simulation/ros/lynsense_webots_sim/config/smoke.yaml` | 点位、控制参数、物理与验收阈值 |
| `robots/lynsense/simulation/ros/lynsense_webots_sim/launch/minimal_smoke.launch.py` | 容器内统一 launch 入口 |
| `robots/lynsense/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/scripts/action_probe.py` | reject、cancel、feedback 和 overlay 检查 |
| `robots/lynsense/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/scripts/run_smoke.py` | readiness、构建、行为树、watchdog 和清理 |
| `robots/lynsense/simulation/docker/Dockerfile` | ROS Humble + Webots 运行镜像 |
| `robots/lynsense/simulation/docker/bootstrap.py` | 首次构建 overlay、验证外部 controller 入口并转发 phase |
| `robots/lynsense/simulation/docker/compose.yaml` | 无网络自动测试服务 |
| `robots/lynsense/simulation/docker/compose.gui.yml` | 仅手动查看时附加 X11 socket |
| `robots/lynsense/simulation/docker/.dockerignore` | 限制 Docker build context 只暴露 Dockerfile |
| `tests/unit_tests/robots/lynsense/simulation/conftest.py` | 将仿真 Python 包加入本机 pytest 路径 |
| `tests/unit_tests/robots/lynsense/simulation/test_geometry.py` | 差速和角度契约 |
| `tests/unit_tests/robots/lynsense/simulation/test_state_machine.py` | 阶段、停滞、超时、取消 |
| `tests/unit_tests/robots/lynsense/simulation/test_action_runtime.py` | 输入校验和单 goal 策率 |
| `tests/unit_tests/robots/lynsense/simulation/test_events.py` | JSONL 事件契约 |
| `tests/unit_tests/robots/lynsense/simulation/test_static_assets.py` | Action、world、Docker 和 Compose 安全契约 |
| `.gitignore` | 忽略 `.artifacts/` |

---

### Task 1: Pure Navigation Core

**Files:**
- Create: `robots/lynsense/simulation/ros/lynsense_webots_sim/package.xml`
- Create: `robots/lynsense/simulation/ros/lynsense_webots_sim/setup.py`
- Create: `robots/lynsense/simulation/ros/lynsense_webots_sim/resource/lynsense_webots_sim`
- Create: `robots/lynsense/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/__init__.py`
- Create: `robots/lynsense/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/geometry.py`
- Create: `robots/lynsense/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/state_machine.py`
- Create: `tests/unit_tests/robots/lynsense/simulation/conftest.py`
- Test: `tests/unit_tests/robots/lynsense/simulation/test_geometry.py`
- Test: `tests/unit_tests/robots/lynsense/simulation/test_state_machine.py`

**Interfaces:**
- Produces: `Pose2(x: float, y: float, yaw: float)`.
- Produces: `normalize_angle(angle_rad: float) -> float`.
- Produces: `angular_error(current_rad: float, target_rad: float) -> float`.
- Produces: `bearing_to_goal(pose: Pose2, target: Pose2) -> float`.
- Produces: `wheel_rates(linear_velocity: float, angular_velocity: float, profile: MotionProfile) -> tuple[float, float]`.
- Produces: `webots_motor_rate(logical_wheel_rate_radps: float) -> float`.
- Produces: `MotionProfile(wheel_separation_m=0.461, wheel_radius_m=0.08, max_linear_velocity_mps=2.0, max_angular_velocity_radps=1.5, max_linear_acceleration_mps2=1.0, max_angular_acceleration_radps2=1.0, smoke_linear_velocity_mps=0.3, smoke_angular_velocity_radps=0.5, smoke_linear_acceleration_mps2=0.5, smoke_angular_acceleration_radps2=1.0, position_tolerance_m=0.05, yaw_tolerance_rad=math.radians(2), stall_window_s=1.0, yaw_stall_threshold_rad=math.radians(0.5), distance_stall_threshold_m=0.005, total_budget_s=60.0, timeout_factor=3.0, timeout_margin_s=2.0, minimum_stage_timeout_s=5.0)`.
- Produces: `NavigationStateMachine.start_nav(start: Pose2, goal: Pose2, sim_time_s: float) -> None`.
- Produces: `NavigationStateMachine.start_move(start: Pose2, distance_m: float, angle_deg: float, sim_time_s: float) -> None`.
- Produces: `NavigationStateMachine.update(pose: Pose2, sim_time_s: float) -> StateMachineUpdate`.

- [x] **Step 1: Add pytest import path and failing geometry tests**

Create `tests/unit_tests/robots/lynsense/simulation/conftest.py`:

```python
from __future__ import annotations

import sys
from pathlib import Path


PACKAGE_PARENT = (
    Path(__file__).resolve().parents[5]
    / "robots"
    / "lynsense"
    / "simulation"
    / "ros"
    / "lynsense_webots_sim"
)
sys.path.insert(0, str(PACKAGE_PARENT))
```

Create `test_geometry.py` with tests for yaw wrapping, shortest angular error, bearing, positive/negative turn direction, wheel-rate limits, and forward/left differential geometry:

```python
import math

import pytest

from lynsense_webots_sim.geometry import (
    Pose2,
    angular_error,
    bearing_to_goal,
    normalize_angle,
    webots_motor_rate,
    wheel_rates,
)
from lynsense_webots_sim.state_machine import MotionProfile


def test_angle_normalization_and_shortest_error():
    assert normalize_angle(math.pi + 0.1) == pytest.approx(-math.pi + 0.1)
    assert angular_error(0.1, -0.1) == pytest.approx(-0.2)
    assert angular_error(math.pi - 0.1, -math.pi + 0.1) == pytest.approx(0.2)


def test_bearing_uses_map_coordinates():
    pose = Pose2(0, 0, 0)
    assert bearing_to_goal(pose, Pose2(1, 1, 0)) == pytest.approx(math.pi / 4)


def test_wheel_rates_are_bounded_and_ordered_for_left_turn():
    profile = MotionProfile()
    left, right = wheel_rates(2.0, 1.5, profile)
    assert abs(left) <= profile.max_wheel_rate_radps + 1e-9
    assert abs(right) <= profile.max_wheel_rate_radps + 1e-9
    # Positive yaw is CCW; the right wheel must spin faster for the same axis sign.
    assert abs(right) > abs(left)


def test_webots_motor_axis_preserves_logical_rate_sign():
    assert webots_motor_rate(1.25) == pytest.approx(1.25)
    assert webots_motor_rate(0.0) == 0.0
```

- [x] **Step 2: Run geometry red**

Run: `.venv/bin/python -m pytest tests/unit_tests/robots/lynsense/simulation/test_geometry.py -q`

Expected: import failure for `lynsense_webots_sim.geometry`.

- [x] **Step 3: Implement package metadata and geometry**

Create the ROS Python package metadata with package name `lynsense_webots_sim`, dependencies `rclpy`, `rosidl_default_runtime`, and `lynsense_utils`. Create `setup.py` with `data_files` for `config`, `worlds`, and `launch`, plus `entry_points`:

```python
packages=["lynsense_webots_sim", "lynsense_webots_sim.scripts"]
```

```python
entry_points={
    "console_scripts": [
        "lynsense_webots_controller = lynsense_webots_sim.webots_controller:main",
        "lynsense_action_probe = lynsense_webots_sim.scripts.action_probe:main",
        "lynsense_run_smoke = lynsense_webots_sim.scripts.run_smoke:main",
    ]
}
```

Implement `geometry.py` without NumPy or ROS imports:

```python
def wheel_rates(linear_velocity, angular_velocity, profile):
    v = clamp(linear_velocity, -profile.max_linear_velocity_mps, profile.max_linear_velocity_mps)
    omega = clamp(
        angular_velocity,
        -profile.max_angular_velocity_radps,
        profile.max_angular_velocity_radps,
    )
    left = (v - omega * profile.wheel_separation_m / 2) / profile.wheel_radius_m
    right = (v + omega * profile.wheel_separation_m / 2) / profile.wheel_radius_m
    limit = profile.max_wheel_rate_radps
    scale = 1.0
    if max(abs(left), abs(right)) > limit:
        scale = limit / max(abs(left), abs(right))
    return left * scale, right * scale
```

The public `MotionProfile.max_wheel_rate_radps` is `max_linear_velocity_mps / wheel_radius_m`. `clamp` rejects NaN/Inf rather than silently coercing.

`MotionProfile.__post_init__` validates that every length, speed, acceleration, tolerance, budget, and threshold is finite and positive; yaw tolerance must be below `pi`, and smoke limits must not exceed hardware limits.

- [x] **Step 4: Write failing phase-state tests**

Create `test_state_machine.py`. Cover:

```python
def test_move_distance_turns_before_negative_translation():
    machine = NavigationStateMachine(MotionProfile())
    machine.start_move(Pose2(0, 0, 0), distance_m=-1.2, angle_deg=90, sim_time_s=0)
    turning = machine.update(Pose2(0, 0, 0), 0.032)
    assert turning.phase is MotionPhase.MOVE_TURN
    assert turning.feedback.distance_remaining == pytest.approx(1.2)
    assert turning.feedback.angle_remaining > 0

    after_turn = machine.update(Pose2(0, 0, math.pi / 2), 2.0)
    assert machine.phase is MotionPhase.MOVE_TRANSLATE
    assert after_turn.feedback.angle_remaining == pytest.approx(0.0)


def test_turn_stall_ignores_translation_progress():
    machine = NavigationStateMachine(MotionProfile())
    machine.start_move(Pose2(0, 0, 0), distance_m=1.0, angle_deg=90, sim_time_s=0)
    machine.update(Pose2(0, 0, 0), 0.032)
    result = machine.update(Pose2(0, 0, 0), 1.1)
    assert result.outcome is Outcome.FAILED
    assert result.reason == "turn_stalled"


def test_translate_stall_ignores_yaw_progress():
    machine = NavigationStateMachine(MotionProfile())
    machine.start_move(Pose2(0, 0, 0), distance_m=1.0, angle_deg=0, sim_time_s=0)
    machine.update(Pose2(0, 0, 0), 0.032)
    result = machine.update(Pose2(0.001, 0, 0), 1.1)
    assert result.outcome is Outcome.FAILED
    assert result.reason == "translate_stalled"
```

Also cover `NavToPose` turn/approach/final-yaw phases, phase reset on transition, stage timeout, total budget, success tolerances, zero-distance turn-only, zero-angle translate-only, and NaN pose rejection.

- [x] **Step 5: Run state-machine red**

Run: `.venv/bin/python -m pytest tests/unit_tests/robots/lynsense/simulation/test_state_machine.py -q`

Expected: import failure for `state_machine`.

- [x] **Step 6: Implement state machine**

Use immutable dataclasses for `MotionCommand(left_wheel_rate_radps: float, right_wheel_rate_radps: float)`, `ActionFeedback(distance_remaining_m: float, angle_remaining_deg: float)`, `StateMachineUpdate`, and `Outcome` enum. Use `NavigationStateMachine` with phases:

```python
class MotionPhase(enum.Enum):
    IDLE = "idle"
    NAV_TURN = "nav_turn"
    NAV_APPROACH = "nav_approach"
    NAV_FINAL_YAW = "nav_final_yaw"
    MOVE_TURN = "move_turn"
    MOVE_TRANSLATE = "move_translate"
    TERMINAL = "terminal"
```

State transitions:

- `NavToPose`: turn to bearing, approach target, converge final yaw.
- `MoveDistance`: turn `angle_deg` relative to start yaw, then translate signed distance along the new body `+x`.
- Success only after both position and yaw tolerances are satisfied.
- On NaN/Inf pose, emit `failed("invalid_pose")` and zero command.
- On stage timeout, emit the exact stage reason such as `nav_approach_timeout`.
- On total timeout, emit `total_timeout`.
- Each transition stores the current error and resets the stall baseline and phase timeout.

Use a proportional controller with acceleration-limited previous `v`/`omega`, clamped by smoke limits and hardware limits. `StateMachineUpdate` always contains `command`, `feedback`, `phase`, `outcome`, and `reason`.

- [x] **Step 7: Run Task 1 green**

Run: `.venv/bin/python -m pytest tests/unit_tests/robots/lynsense/simulation -q`

Expected: all new tests pass. Then run:

```bash
.venv/bin/python -m pytest tests/unit_tests/robots/lynsense -q
```

Expected: existing read-only tests and new simulation tests all pass.

### Task 2: Action Compatibility Package and Deterministic Assets

**Files:**
- Create: `robots/lynsense/simulation/ros/lynsense_utils_compat/package.xml`
- Create: `robots/lynsense/simulation/ros/lynsense_utils_compat/CMakeLists.txt`
- Create: `robots/lynsense/simulation/ros/lynsense_utils_compat/action/NavToPose.action`
- Create: `robots/lynsense/simulation/ros/lynsense_utils_compat/action/MoveDistance.action`
- Create: `robots/lynsense/simulation/ros/lynsense_utils_compat/README.md`
- Create: `robots/lynsense/simulation/ros/lynsense_webots_sim/config/smoke.yaml`
- Create: `robots/lynsense/simulation/ros/lynsense_webots_sim/worlds/lynsense_smoke.wbt`
- Create: `robots/lynsense/simulation/ros/lynsense_webots_sim/worlds/lynsense_blocked.wbt`
- Test: `tests/unit_tests/robots/lynsense/simulation/test_static_assets.py`

**Interfaces:**
- Produces ROS package `lynsense_utils`.
- Produces action types `lynsense_utils/action/NavToPose` and `lynsense_utils/action/MoveDistance`.
- Produces YAML keys consumed by controller: `goals`, `motion`, `physics`, `world`, `timeouts`, `actuators`, and `scripts`.

- [x] **Step 1: Write failing static asset tests**

Create tests that read exact files and assert:

```python
def test_action_definitions_match_simulation_contract():
    nav = ACTION_ROOT.joinpath("NavToPose.action").read_text()
    move = ACTION_ROOT.joinpath("MoveDistance.action").read_text()
    assert nav.splitlines()[:5] == [
        "string goal_name",
        "---",
        "bool success",
        "string message",
        "---",
    ]
    assert "float64 distance_remaining" in move
    assert "float64 distance" in move
    assert "float64 angle" in move
    assert "float64 angle_remaining" in move


def test_goals_and_world_are_deterministic():
    config = yaml.safe_load(CONFIG_PATH.read_text())
    assert config["goals"] == {
        "home": {"x": 0.0, "y": 0.0, "yaw_deg": 0.0},
        "sim_goal1": {"x": 2.0, "y": 0.0, "yaw_deg": 0.0},
        "sim_goal2": {"x": 0.5, "y": 0.0, "yaw_deg": 0.0},
    }
    assert config["physics"]["wheel_separation_m"] == 0.461
    assert config["physics"]["wheel_radius_m"] == 0.08
    assert config["actuators"] == {
        "left_wheel_motor": {"max_velocity_radps": 25.0, "max_torque_nm": 50.0},
        "right_wheel_motor": {"max_velocity_radps": 25.0, "max_torque_nm": 50.0},
    }
    assert config["scripts"]["smoke"] == [
        {"action": "nav_to_pose", "goal_name": "sim_goal1"},
        {"action": "move_distance", "distance": -1.5, "angle": 0.0},
        {"action": "move_distance", "distance": 1.2, "angle": 0.0},
        {"action": "move_distance", "distance": -1.2, "angle": 0.0},
        {"action": "nav_to_pose", "goal_name": "sim_goal2"},
    ]
    assert config["scripts"]["blocked"] == config["scripts"]["smoke"]
```

Add assertions that both worlds contain exactly two `RotationalMotor` names, each has `maxVelocity 25` and `maxTorque 50`, no arm/gripper/lift motor names, correct time step, root translation, caster anchors, obstacle presence only in the blocked world, and no `ROS_DOMAIN_ID=3`. Record in the README that torque is a smoke-model parameter because the supplied URDF does not provide a reviewed wheel torque limit.

- [x] **Step 2: Run static tests red**

Run: `.venv/bin/python -m pytest tests/unit_tests/robots/lynsense/simulation/test_static_assets.py -q`

Expected: missing files fail.

- [x] **Step 3: Implement action package**

`NavToPose.action`:

```text
string goal_name
---
bool success
string message
---
float64 distance_remaining
```

`MoveDistance.action`:

```text
float64 distance
float64 angle
---
bool success
string message
---
float64 distance_remaining
float64 angle_remaining
```

Use `ament_cmake`, `rosidl_default_generators`, `rosidl_default_runtime`, and membership in `rosidl_interface_packages`. Generate both actions in `CMakeLists.txt`.

The README states that this package is a simulation-only reconstruction from caller-side fields and must be replaced or reconciled after obtaining the company's original `lynsense_utils`.

- [x] **Step 4: Implement config and worlds**

`smoke.yaml` contains exact spec values for goals, 8 x 5 arena, motion profile, actuator limits, the smoke/blocked action scripts, 16 ms basic time step, 32 ms controller step, 60 s action budget, and readiness budget. The world uses:

```text
DEF EA200_SMOKE Robot:
  supervisor TRUE
  controller "<extern>"
  synchronization TRUE
  translation [0, 0, 0.08]
  rotation [0, 0, 1, 0]
  left wheel anchor [0, 0.2305, 0], axis [0, 1, 0]
  right wheel anchor [0, -0.2305, 0], axis [0, 1, 0]
  left caster anchor [-0.436, 0.18, -0.0425]
  right caster anchor [-0.436, -0.18, -0.0425]
```

The root robot has two fixed collision boxes, two wheel `HingeJoint` devices, and two passive `BallJoint` spheres. The blocked world is identical except for the fixed obstacle at `(1.0, 0.0, 0.25)` with size `0.10 x 1.00 x 0.50`.

- [x] **Step 5: Run static tests green**

Run:

```bash
.venv/bin/python -m pytest tests/unit_tests/robots/lynsense/simulation/test_static_assets.py -q
.venv/bin/python -m pytest tests/unit_tests/robots/lynsense/simulation -q
```

Expected: all local asset and pure-core tests pass.

### Task 3: Action Runtime and JSONL Events

**Files:**
- Create: `robots/lynsense/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/action_runtime.py`
- Create: `robots/lynsense/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/events.py`
- Test: `tests/unit_tests/robots/lynsense/simulation/test_action_runtime.py`
- Test: `tests/unit_tests/robots/lynsense/simulation/test_events.py`

**Interfaces:**
- Consumes: `NavigationStateMachine.update(...)`.
- Produces: `ActionRuntime(profile: MotionProfile, goals: dict[str, Pose2], allowed_actions: Sequence[ExpectedAction] | None = None)`.
- Produces: `ActionRuntime.submit_nav(goal_name: str) -> Acceptance`.
- Produces: `ActionRuntime.submit_move(distance_m: float, angle_deg: float) -> Acceptance`.
- Produces: `ActionRuntime.request_cancel() -> bool`.
- Produces: `ActionRuntime.observe(pose: Pose2, sim_time_s: float) -> RuntimeObservation`.
- Produces: `EventRecorder(path: Path, run_id: str)` with `record(event: dict) -> None`; it appends for an existing same-run file, starts a fresh generation for a different run ID, and injects/validates `run_id`.
- Produces: `FaultSignal.request(reason: str) -> None` and `FaultSignal.take() -> str | None`.

- [x] **Step 1: Write failing runtime tests**

Cover:

```python
def test_unknown_and_invalid_goals_are_rejected_without_state_change():
    runtime = ActionRuntime(MotionProfile(), {"home": Pose2(0, 0, 0)})
    assert runtime.submit_nav("missing").accepted is False
    assert runtime.submit_move(float("nan"), 0).accepted is False
    assert runtime.submit_move(1.0, float("inf")).accepted is False
    assert runtime.submit_nav("").accepted is False
    assert runtime.submit_nav("x" * 129).accepted is False
    assert runtime.active is False


def test_second_goal_is_rejected_and_cancel_stops():
    runtime = ActionRuntime(MotionProfile(), {"home": Pose2(0, 0, 0)})
    assert runtime.submit_move(1, 360).accepted is True
    assert runtime.submit_nav("home").accepted is False
    assert runtime.request_cancel() is True
    observation = runtime.observe(Pose2(0, 0, 0), 0.1)
    assert observation.outcome is Outcome.CANCELED
    assert observation.command.left_wheel_rate_radps == 0
    assert observation.command.right_wheel_rate_radps == 0
```

Add tests for control-character goal names, all valid boundary values, preserving terminal state after cancel, and terminal lockout: once final success, failure, or cancellation locks the runtime, every new goal is rejected with `terminal_lockout`, even when its fields otherwise match the expected script.

Add script-mode tests:

```python
def test_script_accepts_only_strict_next_action_then_locks():
    script = [
        ExpectedAction.nav("sim_goal1"),
        ExpectedAction.move(-1.5, 0.0),
    ]
    runtime = ActionRuntime(MotionProfile(), GOALS, allowed_actions=script)
    assert runtime.submit_move(-1.5, 0).accepted is False
    assert runtime.submit_nav("sim_goal1").accepted is True
    assert runtime.submit_nav("sim_goal1").accepted is False
    assert runtime.request_cancel() is True
    assert runtime.observe(Pose2(0, 0, 0), 0.1).outcome is Outcome.CANCELED
    assert runtime.submit_move(-1.5, 0).accepted is False
    assert runtime.submit_move(-1.5, 0).reason == "terminal_lockout"
```

The same test covers final success: run the second action to completion and verify all later submissions return `terminal_lockout`. A blocked first action also enters the same lockout.

Add an intermediate-success test for a three-entry script: after the first action succeeds, only the exact second entry is accepted; a repeat of the first entry returns `script_out_of_order`, and the final successful action enters `terminal_lockout`.

- [x] **Step 2: Run runtime tests red**

Run: `.venv/bin/python -m pytest tests/unit_tests/robots/lynsense/simulation/test_action_runtime.py -q`

Expected: import failure.

- [x] **Step 3: Implement runtime**

`Acceptance` contains `accepted: bool` and `reason: str`. `ExpectedAction` has discriminators `nav(goal_name)` and `move(distance_m, angle_deg)`. Runtime owns at most one active state machine. Valid bounds are:

```text
goal_name: non-empty, <=128 UTF-8 bytes, no Unicode control characters
distance_m: finite and [-10, 10]
angle_deg: finite and [-360, 360]
```

`observe` runs the state machine once, converts feedback fields to non-negative finite values, and returns zero wheel rates on every terminal outcome. Terminal result dataclass carries `success`, `message`, and outcome enum.

When `allowed_actions` is set, submissions must match the next unmatched script entry exactly; mismatched action, goal name, distance, or angle returns `script_out_of_order`. A successful non-final scripted action advances the index and permits exactly the next entry. Final scripted success, failure, or cancellation enters `terminal_lockout`. Unconstrained interface mode also locks after its one terminal probe result. This lockout is required because the existing `lynsense_pytrees_node` continuously ticks and may restart the root after a terminal state before the external watchdog can signal it.

The controller records the first `terminal_lockout` rejection but suppresses further duplicate lockout events from the continuously ticking tree. It counts suppressed duplicates for the summary, preventing JSONL/fsync spam while preserving evidence of the restart attempt.

- [x] **Step 4: Write failing event tests**

Test JSON schema, UTF-8, single-generation file semantics, multi-record append within one recorder instance, invalid event rejection, and one `flush`/`os.fsync` per record:

```python
def test_event_recorder_writes_one_json_object_per_line(tmp_path):
    path = tmp_path / "events.jsonl"
    recorder = EventRecorder(path, "run-1")
    recorder.record({"kind": "run_started", "action": "controller", "sim_time_s": 0.0})
    lines = path.read_text(encoding="utf-8").splitlines()
    assert len(lines) == 1
    assert json.loads(lines[0])["run_id"] == "run-1"
```

Require `kind`, `sim_time_s`, `action`, and `run_id`; reject a mismatched supplied `run_id` and NaN/Inf recursively.

Add a thread-safe `FaultSignal` test: a request from another thread becomes visible exactly once; an empty signal returns `None`; a second request cannot overwrite an earlier unread fault.
Add an EventRecorder failure test by injecting a path or file object whose write/flush/fsync raises `OSError`. The recorder propagates the failure; it does not partially emit an unnewline-terminated JSON object or claim success.

- [x] **Step 5: Implement recorder and run green**

Implement deterministic JSON serialization with `sort_keys=True`, `allow_nan=False`, UTF-8, line flush, and `os.fsync`. Run:

```bash
.venv/bin/python -m pytest tests/unit_tests/robots/lynsense/simulation -q
```

Expected: Task 1-3 tests pass.

### Task 4: Webots Supervisor Controller

**Files:**
- Create: `robots/lynsense/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/webots_controller.py`
- Modify: `robots/lynsense/simulation/ros/lynsense_webots_sim/setup.py`

**Interfaces:**
- Consumes: Webots `Robot`, `Supervisor`, `RotationalMotor`, `rclpy`, generated action types, `ActionRuntime`, and `EventRecorder`.
- Produces: console entry point `lynsense_webots_controller`.
- Produces: controller environment variables `LYNSENSE_SIM_CONFIG`, `LYNSENSE_SIM_EVENT_FILE`, `LYNSENSE_SIM_SCRIPT`, and `LYNSENSE_SIM_RUN_ID`.

- [x] **Step 1: Add controller integration tests without Webots**

Add tests to `test_static_assets.py` that inspect `webots_controller.py` for:

- lazy ROS imports inside the process entry point;
- exactly two motor names;
- `Supervisor.getTime()` used for observation time;
- explicit executor thread;
- signal handlers for SIGINT/SIGTERM;
- finally-ordered stop, action server destruction, node destruction, executor shutdown, and `rclpy.shutdown()`;
- no publisher/client/service to xArm, gripper, chassis, or any real robot topic.

These are structural tests because host pytest cannot import `controller` or `rclpy`.

- [x] **Step 2: Run controller static tests red**

Run: `.venv/bin/python -m pytest tests/unit_tests/robots/lynsense/simulation/test_static_assets.py -q`

Expected: controller file missing or checks fail.

- [x] **Step 3: Implement controller**

Controller structure:

```python
class LynsenseWebotsController:
    def __init__(self, robot, node, action_servers, event_recorder, runtime, config):
        ...

    def run(self) -> int:
        self._record_controller_ready()
        while self._robot.step(32) != -1:
            fault = self._faults.take()
            if fault is not None:
                return self._abort_for_fault(fault)
            pose = self._read_pose()
            observation = self._runtime.observe(pose, self._robot.getTime())
            self._write_motors(observation.command)
            self._publish_feedback_and_result(observation)
            if self._shutdown_requested:
                break
        return self._shutdown()
```

`_record_controller_ready` constructs both action servers, reads a finite Supervisor pose, writes zero velocity to both motors, and records a `controller_ready` event containing action names, motor names, pose, and zero rates. Any failure exits non-zero before the orchestrator starts the behavior tree.

ROS callbacks and the simulation loop share one controller lock around `ActionRuntime`; callbacks never touch Webots Motor objects, and the simulation step thread is the only Webots Motor writer. The executor uses `MultiThreadedExecutor(num_threads=2)` in daemon threads with reentrant action callback groups, allowing goal, execute, and cancel callbacks to run while one execute callback is reserved for a running goal.

Read pose from `robot.getSelf().getPosition()` and `getOrientation()`. Webots returns a row-major 3x3 matrix; map yaw is `atan2(orientation[3], orientation[0])`, the direction of the robot body `+x` axis in the world plane. Task 1 exposes logical wheel rates where positive drives the robot along body `+x`; with both hinge axes set to `+y`, `motor.setVelocity(logical_wheel_rate)` is the identity conversion. Zero is passed unchanged. This helper is unit-tested and must not use different signs for left and right wheels.

Action goal callbacks:

- reject invalid or concurrent goals synchronously;
- record `goal_accepted` or `goal_rejected`;
- send accepted goals into `ActionRuntime`;
- honor cancel callbacks by setting the runtime cancel flag.

Every feedback/result event includes action, goal fields, remaining values, phase, outcome, reason, pose, sim time, and motor rates. JSONL events are flushed one by one.

Executor and Webots fault handling:

- The executor thread catches `BaseException`, requests a fault with `executor_failed:<detail>`, and exits.
- The simulation loop checks `FaultSignal` before and after each pose observation.
- Any `robot.step()` exception, pose read exception, or motor write exception requests `webots_failed:<detail>`.
- A fault handler writes zero to both motors, aborts the active goal handle when present, records `controller_failed`, shuts down the executor/node, and returns non-zero.
- Normal Webots termination with an active goal records `webots_terminated`, aborts the goal, and returns non-zero; normal termination after a completed result exits zero.
- `finally` always writes zero motor velocity, stops the executor, destroys action servers and node, and shuts down only an `rclpy` context successfully initialized by this controller.
- Every controller event write goes through a fault-aware wrapper. JSON serialization, file write, flush, or fsync failure requests `event_recorder_failed`, writes zero motor velocity, aborts the active goal, and returns non-zero. If the recorder is already broken, the controller still performs the safe stop even though it cannot append another event.

Host tests inspect these paths and exercise `FaultSignal` directly. Although the final ROS/Webots branch is covered by the container gate, missing fault handling is a host test failure.

- [x] **Step 4: Run host tests and package import checks**

Run:

```bash
.venv/bin/python -m pytest tests/unit_tests/robots/lynsense/simulation -q
.venv/bin/python -m pytest tests/unit_tests/robots/lynsense -q
```

Expected: all local tests pass without importing Webots or ROS at module import time.

### Task 5: Container, Readiness, Probe, and Orchestrator

**Files:**
- Create: `robots/lynsense/simulation/docker/Dockerfile`
- Create: `robots/lynsense/simulation/docker/bootstrap.py`
- Create: `robots/lynsense/simulation/docker/compose.yaml`
- Create: `robots/lynsense/simulation/docker/compose.gui.yml`
- Create: `robots/lynsense/simulation/docker/.dockerignore`
- Create: `robots/lynsense/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/scripts/__init__.py`
- Create: `robots/lynsense/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/scripts/action_probe.py`
- Create: `robots/lynsense/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/scripts/run_smoke.py`
- Create: `robots/lynsense/simulation/ros/lynsense_webots_sim/launch/minimal_smoke.launch.py`
- Create: `robots/lynsense/simulation/README.md`
- Modify: `.gitignore`
- Test: `tests/unit_tests/robots/lynsense/simulation/test_static_assets.py`

**Interfaces:**
- Produces image `rpent-lynsense-webots:r2025a-humble`.
- Produces compose service `lynsense-webots-smoke`.
- Produces `lynsense_action_probe --mode interface|cancel`.
- Produces `bootstrap.py --phase build|all`, where `all` builds then delegates to `lynsense_run_smoke`.
- Produces installed `lynsense_run_smoke --phase interface|smoke|blocked|all` for post-build gates; its `all` phase runs the three gates sequentially without rebuilding.

- [x] **Step 1: Write failing container and orchestration tests**

Static tests must assert:

```python
def test_compose_is_isolated_and_read_only():
    compose = yaml.safe_load(COMPOSE_PATH.read_text())
    service = compose["services"]["lynsense-webots-smoke"]
    assert service["network_mode"] == "none"
    assert service["environment"]["ROS_DOMAIN_ID"] == "42"
    assert service["environment"]["ROS_LOCALHOST_ONLY"] == "1"
    volumes = service["volumes"]
    source_mounts = [
        volume for volume in volumes
        if volume["target"] in {"/workspace/simulation", "/workspace/ws/src/lynsense_pytrees"}
    ]
    assert len(source_mounts) == 2
    assert all(volume["read_only"] is True for volume in source_mounts)
    assert "ports" not in service
```

Use long-form Compose volumes. Require `/workspace/simulation` and `/workspace/ws/src/lynsense_pytrees` read-only; require `/workspace/artifacts` writable. Do not mount the RPent repository root, because it contains the ignored local LLM env file. Do not create persistent build, install, or log volumes. Reject mounts containing `.env.lynsense`, `.ssh`, `/home/rpp`, or the robot SSH alias. Assert GUI override only adds `/tmp/.X11-unix` read-only and never changes `network_mode`.

Also assert Dockerfile digest, Jammy ROS repository, Humble packages, no `apt-get upgrade`, no secret `COPY`, artifact/build paths, non-root `USER lynsense`, UID `1000`, and writable `/workspace/ws/src` plus `/workspace/artifacts`.

Assert `.dockerignore` allows only `Dockerfile`. Assert compose entrypoint points to mounted `bootstrap.py`, build context is the Docker directory, and bootstrap verifies the installed controller entry point after colcon build. Assert both worlds use `controller "<extern>"` and `synchronization TRUE`; the orchestrator starts that entry point with `$WEBOTS_HOME/webots-controller`. Assert no command invokes a colcon console script before bootstrap has sourced the newly built overlay.

- [x] **Step 2: Run container tests red**

Run: `.venv/bin/python -m pytest tests/unit_tests/robots/lynsense/simulation/test_static_assets.py -q`

Expected: Docker/compose/scripts missing.

- [x] **Step 3: Implement Dockerfile and compose**

Dockerfile:

```dockerfile
FROM cyberbotics/webots@sha256:f0023e30daf38b172e4e6ad24ed345909bcd9551df34d63d824e121a7cebf099

ENV DEBIAN_FRONTEND=noninteractive ROS_DISTRO=humble
RUN apt-get update && apt-get install -y --no-install-recommends \
      curl gnupg lsb-release software-properties-common ca-certificates \
    && install -d /usr/share/keyrings \
    && curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
       -o /usr/share/keyrings/ros-archive-keyring.asc \
    && echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.asc] http://packages.ros.org/ros2/ubuntu jammy main" \
       > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-ros-base \
      ros-humble-rosidl-default-generators \
      ros-humble-rosidl-default-runtime \
      ros-humble-py-trees \
      ros-humble-py-trees-ros \
      python3-colcon-common-extensions \
      python3-yaml \
      xvfb \
    && rm -rf /var/lib/apt/lists/* \
    && useradd --uid 1000 --create-home --shell /bin/bash lynsense \
    && mkdir -p /workspace/ws/src /workspace/artifacts \
    && chown -R lynsense:lynsense /workspace

USER lynsense
WORKDIR /home/lynsense
ENV HOME=/home/lynsense
```

Compose build context is `.` inside `robots/lynsense/simulation/docker/`; `.dockerignore` excludes everything except `Dockerfile`, so the local `.env.lynsense` and Git data never enter the Docker build context. At runtime it mounts the simulation subtree from `..` at `/workspace/simulation:ro`, mounts sibling `../../../../../lynsense_pytrees` at `/workspace/ws/src/lynsense_pytrees:ro`, and writes evidence to `../../../../.artifacts/lynsense-webots:/workspace/artifacts`. Build/install/log state stays in the ephemeral container filesystem. The service entrypoint is `python3 /workspace/simulation/docker/bootstrap.py`, with default command `--phase all`. It sets `WEBOTS_HOME` and `LYNSENSE_SIM_CONFIG`; the orchestrator supplies phase-specific `LYNSENSE_SIM_EVENT_FILE` and `LYNSENSE_SIM_SCRIPT`.

`bootstrap.py` is required because the first `build` phase runs before any colcon console script exists. It is a Python entrypoint but does not try to parse `setup.bash` in Python; every shell-dependent operation runs through one bounded `subprocess.run(["/bin/bash", "-lc", command])` helper with explicit environment, captured output, and propagated return code.

Each Bash command that needs ROS or the overlay includes its required `source ... && ...` chain inside that same command; separate subprocess invocations never assume inherited shell state.

1. sources `/opt/ros/humble/setup.bash`;
2. removes and recreates `/workspace/ws/build`, `/workspace/ws/install`, `/workspace/ws/log`, and the two copied simulation package directories under `/workspace/ws/src`; it never modifies the read-only `lynsense_pytrees` mount;
3. copies the two read-only simulation ROS packages into `/workspace/ws/src`;
4. runs `colcon build --packages-select lynsense_utils lynsense_webots_sim lynsense_pytrees`;
5. verifies imports and the local `lynsense_utils` overlay prefix;
6. verifies `/workspace/ws/install/lynsense_webots_sim/bin/lynsense_webots_controller` exists and is executable;
7. writes build evidence;
8. for non-build phases, runs `source /opt/ros/humble/setup.bash && source /workspace/ws/install/setup.bash && exec lynsense_run_smoke "$@"` through the same Bash helper and propagates its exit code.

- [x] **Step 4: Implement action probe**

`action_probe.py`:

1. Uses a bounded 45-second budget for both action types and endpoints.
2. Sends unknown `NavToPose` and verifies reject.
3. Sends NaN/Inf/out-of-range `MoveDistance` goals and verifies reject.
4. For concurrent-goal checking, submits `angle=360,distance=0`, then a second valid goal and verifies reject.
5. For cancel, waits until at least two feedback messages and at least half of the requested rotation remains, cancels, verifies canceled status, `success=false`, zero motor rates in events, and no further progress.

Exit non-zero if any assertion fails.

- [x] **Step 5: Implement run orchestrator**

`run_smoke.py` phases:

- `build`: performed by bootstrap before delegation.
- `interface`: start Webots, start the installed `/workspace/ws/install/lynsense_webots_sim/bin/lynsense_webots_controller` console script, wait for `controller_ready` plus both action endpoints, run interface and cancellation probes, stop cleanly.
- `smoke`: start success world, start the same external controller, wait for `controller_ready` plus both action endpoints, run `lynsense_pytrees_node` with `xml=lynsense_webots_tree.xml`, tail JSONL, wait for expected final action and pose, SIGINT tree, then terminate controller and Webots in order.
- `blocked`: start blocked world, run the same tree, require a `translate_stalled` failure with zero wheel rates, and normalize only that expected blocked failure to exit `0`.
- `all`: build, interface, smoke, then blocked; write `summary.json`.

Readiness requires both action names/types, a valid `controller_ready` JSONL event, finite initial pose, and evidence that both motors accepted zero velocity. It then waits 1 second. The budget is 15 seconds; if Webots exits first or the event file is missing, readiness fails immediately.

The orchestrator passes `/workspace/simulation/ros/lynsense_webots_sim/config/smoke.yaml` as `LYNSENSE_SIM_CONFIG`. It overrides `LYNSENSE_SIM_EVENT_FILE` per phase to `events-interface.jsonl`, `events-smoke.jsonl`, or `events-blocked.jsonl`, so repeated gates cannot overwrite one another.

The orchestrator passes `LYNSENSE_SIM_SCRIPT=interface`, `smoke`, or `blocked`. `interface` leaves runtime unconstrained for probe goals; `smoke` and `blocked` install the exact action script and terminal lockout described in Task 3.

Every phase invocation also creates a run generation boundary:

1. Generate `run_id = <UTC timestamp>-<uuid4>`.
2. Move an existing same-phase event file to `archive/<phase>-<old mtime>-<uuid>.jsonl`.
3. Create the new phase event file empty before spawning Webots or controller.
4. Pass `LYNSENSE_SIM_RUN_ID` to controller and watchdog.
5. Require the first controller event to be `run_started` with that exact ID, followed by `controller_ready` with the same ID.
6. Reject any JSONL line with a missing or different `run_id`, even if it is valid JSON.

`EventRecorder` appends when reopening an existing file whose first event has the same run ID and rewrites the file when that run ID differs. This lets the interface phase preserve both controller lifecycles while preventing a failed rerun from consuming a previous complete sequence and producing a false pass.

The watchdog accepts only exact expected event sequence:

```text
nav_to_pose sim_goal1 success
move_distance distance=-1.5 success
move_distance distance=1.2 success
move_distance distance=-1.2 success
nav_to_pose sim_goal2 success
```

It verifies final pose against `(0.5, 0, 0°)`, sends SIGINT to the tree, and invokes the same cleanup path for success and failure. The watchdog uses a monotonic 120-second deadline for the normal tree and a separate 30-second deadline for the blocked case. Invalid JSON, a missing newline, an out-of-order event, tree exit before terminal event, Webots exit before terminal event, or deadline expiry fails the phase. The watchdog polls both JSONL growth and child process liveness at least every 100 ms.

The exact sequence applies only to `goal_accepted` and `action_result` events after filtering:

```text
goal_accepted(nav_to_pose, sim_goal1)
action_result(nav_to_pose, sim_goal1, success)
goal_accepted(move_distance, -1.5, 0)
action_result(move_distance, -1.5, 0, success)
goal_accepted(move_distance, 1.2, 0)
action_result(move_distance, 1.2, 0, success)
goal_accepted(move_distance, -1.2, 0)
action_result(move_distance, -1.2, 0, success)
goal_accepted(nav_to_pose, sim_goal2)
action_result(nav_to_pose, sim_goal2, success)
```

Feedback, `run_started`, and `controller_ready` events are validated separately and do not consume sequence positions. Before the terminal result, any rejected goal is a smoke/blocked failure. After the terminal result, zero or more `goal_rejected(reason=terminal_lockout)` events are allowed because the tree may tick again before SIGINT, but any post-terminal `goal_accepted`, any other rejection reason, or a new motor command is a failure.

Automated Webots startup uses:

```bash
xvfb-run -a --server-args="-screen 0 1280x1024x24 +extension GLX +render" \
  webots --batch --no-rendering --mode=run --stdout --stderr \
    "${WEBOTS_WORLD}"
```

 stdout/stderr are redirected to separate artifact logs. The orchestrator checks the process immediately after spawn and treats early exit, missing world, or missing executable as a startup failure.

- [x] **Step 6: Write README and ignore artifacts**

README documents build, interface, smoke, blocked, display override, expected evidence, isolation guarantees, the bootstrap/installed-command boundary, and the explicit statement that simulation success does not authorize real-robot motion. Explain that Webots uses official `--no-rendering` for the headless run, records that `--mode=run` is deprecated and falls back to `fast`, and notes that action deadlines still use Webots simulation time. Add `.artifacts/` to `.gitignore`.

- [x] **Step 7: Run all host tests**

Run:

```bash
.venv/bin/python -m pytest tests/unit_tests/robots/lynsense/simulation -q
.venv/bin/python -m pytest tests/unit_tests/robots/lynsense -q
```

Expected: all local tests pass. No container command has run yet.

### Task 6: Container Build and Interface Gate

**Files:**
- No new product files.
- Evidence: `.artifacts/lynsense-webots/build-summary.json`
- Evidence: `.artifacts/lynsense-webots/interface-summary.json`

**Interfaces:**
- Consumes Task 5 image and scripts.
- Produces a reproducible image digest and ROS action interface evidence.

- [x] **Step 1: Build image**

First create the host artifact directory with the workstation user:

```bash
mkdir -p .artifacts/lynsense-webots
```

Run from `RPent/`:

```bash
docker compose \
  -f robots/lynsense/simulation/docker/compose.yaml \
  build --pull lynsense-webots-smoke
```

Expected: image builds successfully. Record base digest, output image ID/digest, and package versions in `build-summary.json`.

- [x] **Step 2: Run build phase**

```bash
docker compose \
  -f robots/lynsense/simulation/docker/compose.yaml \
  run --rm lynsense-webots-smoke \
  --phase build
```

Expected exit `0`; `colcon build` succeeds for `lynsense_utils`, `lynsense_webots_sim`, and `lynsense_pytrees`; imports and overlay prefix pass.

- [x] **Step 3: Run interface gate**

```bash
docker compose \
  -f robots/lynsense/simulation/docker/compose.yaml \
  run --rm lynsense-webots-smoke \
  --phase interface
```

Expected exit `0`; both actions are discoverable, invalid goals reject, concurrent goal rejects, and cancel stops wheels.

- [x] **Step 4: Inspect container isolation**

Inside the interface phase, record:

```bash
env | grep '^ROS_'
printf 'AMENT_PREFIX_PATH=%s\n' "${AMENT_PREFIX_PATH-}"
printf 'ROS_PACKAGE_PATH=%s\n' "${ROS_PACKAGE_PATH-}"
ros2 pkg prefix lynsense_utils
python3 - <<'PY'
import rclpy, py_trees, py_trees_ros, yaml
print(rclpy.__name__, py_trees.__name__, py_trees_ros.__name__, yaml.__name__)
PY
```

Expected: Domain `42`, localhost only, local overlay prefix, all imports successful, and no network mapping. `AMENT_PREFIX_PATH` and `ROS_PACKAGE_PATH` must not contain `/home/rpp`, `rpp_ws`, or any robot workspace path. Bootstrap also records these two variables before and after sourcing in `build-summary.json`; the pre-source values may be empty, and the post-source values may contain only `/opt/ros/humble` and `/workspace/ws/install`.

### Task 7: Webots Behavior Tree Success and Blocked Gates

**Files:**
- No new product files.
- Evidence: `.artifacts/lynsense-webots/summary.json`
- Evidence: `.artifacts/lynsense-webots/events-smoke.jsonl`
- Evidence: `.artifacts/lynsense-webots/events-blocked.jsonl`

**Interfaces:**
- Consumes Task 6 image and action servers.
- Produces final simulation evidence for user review.

- [x] **Step 1: Run success smoke**

```bash
docker compose \
  -f robots/lynsense/simulation/docker/compose.yaml \
  run --rm lynsense-webots-smoke \
  --phase smoke
```

Expected exit `0`, exact five-action sequence, all success results, final pose within tolerance, wheel rates zero, and clean process exit.

- [x] **Step 2: Run blocked gate**

```bash
docker compose \
  -f robots/lynsense/simulation/docker/compose.yaml \
  run --rm lynsense-webots-smoke \
  --phase blocked
```

The orchestrator treats expected blocked failure as gate success: exit `0` only when the first `NavToPose` aborts by stall/timeout, wheels stop, tree fails, and no later action is accepted.

- [x] **Step 3: Verify no host residue**

Run:

```bash
docker compose -f robots/lynsense/simulation/docker/compose.yaml ps
pgrep -af 'webots|lynsense_pytrees_node|lynsense_webots_controller' || true
```

Expected no running compose service and no relevant host process.

- [x] **Step 4: Run complete local regression**

```bash
.venv/bin/python -m pytest tests/unit_tests/robots/lynsense -q
```

Expected all read-only and simulation host tests pass.

### Task 8: Final Review and Delivery Package

**Files:**
- Modify: `robots/lynsense/simulation/README.md`
- Modify: `docs/superpowers/plans/2026-09-20-lynsense-webots-smoke.md`

**Interfaces:**
- Consumes all previous evidence.
- Produces final review result and user-facing run instructions.

- [x] **Step 1: Collect full worktree evidence**

Run:

```bash
git status --short
git diff --check
git diff --stat
find robots/lynsense/simulation tests/unit_tests/robots/lynsense/simulation .artifacts/lynsense-webots -type f -print 2>/dev/null | sort
```

Review untracked and modified files; do not rely on commit-only diff.

- [x] **Step 2: Independent read-only review**

Dispatch one independent reviewer for:

- spec/plan coverage;
- real-robot isolation;
- ROS/Webots lifecycle;
- action semantics and feedback;
- JSONL watchdog races;
- Docker/compose security;
- test evidence completeness.

Fix every blocker/important finding and rerun the focused gate plus the same review until approved.

- [x] **Step 3: Update plan evidence**

Record actual image digest, test counts, exit codes, final pose, blocked reason, cleanup check, and any `NOT RUN` command with its missing prerequisite. Mark all completed checkboxes.

- [x] **Step 4: Final user report**

Report:

- files changed;
- host test result;
- image digest;
- build/interface/smoke/blocked results;
- final pose and action sequence;
- isolation evidence;
- no real robot connection or motion;
- remaining limitations and next milestone.

Do not claim real-robot safety or authorize the next hardware phase.

## Completed Evidence

- Output image: `rpent-lynsense-webots:r2025a-humble`, digest `sha256:cf91e1d0c108c2c585033fcab8df56db7d1338738a5f4c192b161cd11fc91927`; pinned base digest `sha256:f0023e30daf38b172e4e6ad24ed345909bcd9551df34d63d824e121a7cebf099`.
- Final host regression: simulation suite `139 passed`; full Lynsense suite `249 passed`; syntax compilation passed; `git diff --check` passed.
- Final unified `all` gate exited `0` at `2026-09-21T06:31:33Z`; `summary.json` records all three phases as `passed`.
- Final interface gate: run `20260921T063118Z-7ed4baae-3177-4952-ab92-a5c0eed961f2`; the gate ran interface and cancellation probes in separate controller lifecycles while appending both evidence generations to one JSONL file. Cancel ended `STATUS_CANCELED`, `success=false`, retained `176.32042887324187` degrees and `0.0 m`, and both wheel rates were `0.0`.
- Final smoke gate: run `20260921T063123Z-4498d7ef-40fa-42a0-ba2d-3be881d45057`; all five scripted actions succeeded; final pose was approximately `(0.4962822101, -5.694695836e-13, 3.363966579e-13)` with both terminal wheel rates `0.0`.
- Final blocked gate: run `20260921T063130Z-ff81af8d-5104-4415-a2e2-452e14030ac9`; expected failure reason was `translate_stalled`, terminal remaining distance was `1.1376252721492885 m`, both terminal wheel rates were `0.0`, and no later action was accepted.
- Runtime isolation: `network_mode: none`, `ROS_DOMAIN_ID=42`, `ROS_LOCALHOST_ONLY=1`, no published ports, no forbidden robot workspace path, no robot-one connection, and no real-robot motion.
- Cleanup: final Compose service list was empty and no Webots, Lynsense tree, or Lynsense controller process remained on the host.
- Known limitation: the read-only upstream tree can log an already-shutdown `rclpy` context during ordered exit. Both final tree gates still passed and stopped both wheels; this package does not patch `lynsense_pytrees`.
- Final evidence archive: `.artifacts/lynsense-webots/archive/final-pass-20260921-143133/`.

## Plan Self-Review

- Spec coverage: Tasks 1-3 cover geometry, semantics, feedback, validation and state machine; Task 2 covers action definitions and deterministic world; Tasks 4-5 cover Supervisor/controller and readiness; Tasks 6-7 cover container, cancel, behavior tree and blocked gates; Task 8 covers review and evidence.
- Type consistency: `Pose2`, `MotionProfile`, `NavigationStateMachine.update`, `ActionRuntime`, and `EventRecorder` signatures are defined before consumers.
- Scope control: No RPent Planner/LLM integration, no real ROS domain, no arm/gripper control, no upstream PR, no auto-commit.
- Environment split: Host pytest runs before Docker; ROS and Webots gates run only in the isolated container.
