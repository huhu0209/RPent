# Lynsense Webots 最小导航仿真设计

日期：2026-09-20

状态：已获用户批准；实施、 scoped 复审和容器 gate 已完成。实施计划见
`docs/superpowers/plans/2026-09-20-lynsense-webots-smoke.md`。最终自建镜像
digest 为 `sha256:cf91e1d0c108c2c585033fcab8df56db7d1338738a5f4c192b161cd11fc91927`。
最终统一 gate run 覆盖 interface、smoke 和 blocked，三者在
`summary.json` 中均为 `passed`。

2026-09-21 扩展：已把 `origin/WHRG2026`（commit `a70e301`）中
`lynsense_plan_move2box_tree.xml` 的第一个导航片段迁移为 `match` gate：
`搬箱子1 -> MoveDistance(-0.6, 0) -> MoveDistance(0, 90) -> 放箱子1_1`。
两个点位坐标和朝向直接来自 `config/move_points.yaml`，不做缩放或平移；
比赛 world 仅把地板宽度扩为 `8 x 6`，不添加障碍。最终统一 gate 覆盖
interface、smoke、blocked 和 match，四者均为 `passed`。该结果仍不构成
真机运动授权。

## 目标

在公司机器人一号执行任何真机运动之前，先在本工作站建立 Webots 最小导航仿真，验证公司机器人底盘模型和既有比赛行为树的导航调用链。第一阶段复用 `lynsense_webots_tree.xml` 的流程，确认 `NavToPose` 与 `MoveDistance` 两个接口能在仿真中可靠执行，为后续比赛场景迁移和真机验证建立安全前置条件。

本阶段只证明仿真链路。测试通过不代表机器人一号健康、安全或获准运动，也不代表 RPent 已具备真机运动能力。

## 现状与输入

- 真实机器人一号已完成 RPent 只读状态接入验证；本设计不修改该只读后端。
- 既有行为树位于同级 `lynsense_pytrees` 仓库，`lynsense_webots_tree.xml` 执行：
  - `NavToPose(sim_goal1)`
  - `MoveDistance(-1.5)`
  - `MoveDistance(1.2)`
  - `MoveDistance(-1.2)`
  - `NavToPose(sim_goal2)`
- 本地存在完整机器人描述包压缩文件 `../URDF/URDF_robot_1.tar.gz`。其中 `ea200_description` 包含 CR100 底盘、上身、双 UF850、相机和夹爪；`cr100_description` 包含底盘几何、轮关节和控制配置。
- 该压缩文件和其中的网格资产不提交到 RPent。第一版模型使用 Webots 基础几何体重建，URDF 仅作为参数和尺寸来源；除非另行确认授权，不复制或改包公司网格资产。
- CR100 关键参数：
  - 差速主动轮：`wheel_LF_joint`、`wheel_RF_joint`
  - 轮距：`0.461 m`
  - 轮半径：`0.08 m`
  - 最大线速度：`2.0 m/s`
  - 最大角速度：`1.5 rad/s`
  - 最大线/角加速度：`1.0`
- 本地未找到原始 `lynsense_utils` 包或 `.action` 定义。不能假设拿到公司原始接口；需要建立显式标记的仿真兼容接口。
- 本工作站为 Ubuntu 26.04，未发现可用的宿主机 `webots`、`ros2` 或 `xacro` 命令。机器人一号生态为 ROS 2 Humble / Ubuntu 22.04。

## 不在本轮范围内

- 不连接机器人一号，不使用 `ROS_DOMAIN_ID=3`，不 source 机器人工作空间。
- 不调用真实底盘、机械臂、夹爪、升降、使能、清错或状态修改接口。
- 不把机械臂或夹爪做成可动仿真关节。
- 不接入 RPent Planner、LLM、视觉系统、Dashboard 或真实传感器。
- 不实现 SLAM、AMCL、Nav2、全局路径规划或动态避障。
- 不保存或处理比赛点位到真实机器人。
- 不把 `.env.lynsense`、API key、SSH 配置或机器人工作空间挂载进仿真容器。
- 不在本轮把仿真结果解释为真机运动安全验收。

## 总体架构

第一版仿真运行在本工作站容器中。容器使用：

```text
Base: cyberbotics/webots:R2025a-ubuntu22.04
OS:   Ubuntu 22.04
ROS:  Humble
```

官方 Webots R2025a Docker 基线使用 Ubuntu 22.04，与 ROS 2 Humble 匹配。基础镜像使用实际存在的 `cyberbotics/webots:R2025a-ubuntu22.04` 标签；2026-09-20 验证的 manifest digest 为 `sha256:f0023e30daf38b172e4e6ad24ed345909bcd9551df34d63d824e121a7cebf099`，实施时按该 digest 固定基础镜像，并把自建镜像的 digest 写入测试证据。

容器内固定：

```text
ROS_DOMAIN_ID=42
ROS_LOCALHOST_ONLY=1
```

Webots、Action Server 和行为树位于同一个本地 ROS 图内，与真实机器人 DDS 隔离。宿主机只负责文件编辑、GUI 显示和结果保存。

系统分为四层：

1. Webots 世界：内置 `Plane` 无限碰撞平面、可选受阻用例障碍和三个命名点位。
2. 机器人模型：使用 CR100 真实底盘参数和简化固定 EA200 外形。
3. 仿真 Action Server：提供两个 Lynsense 导航 Action。
4. 既有行为树入口：运行 `lynsense_webots_tree.xml`，调用仿真 Action。

运行容器使用 `network_mode: none`，只保留容器 loopback。Webots、Action Server 和行为树都在同一容器内，通过本地 ROS 图通信；不映射 DDS 端口到宿主机或局域网。GUI 仅按需挂载 X11 socket，构建阶段需要的网络与运行阶段隔离。

启动顺序使用 readiness barrier：

```text
启动 Webots 和 Supervisor controller
-> 等待两个 Action 的类型和 server endpoint 均可发现
-> 等待 Supervisor pose 有效且左右轮 Motor 可写
-> 等待短暂 discovery settle 窗口
-> 才启动 lynsense_pytrees_node
```

现有 `NavToPose` 和 `MoveDistance` 客户端只等待 server 1 秒，因此行为树不能作为启动顺序保障。bring-up 脚本必须在树进程启动前完成上述检查；如果 readiness 超时，直接停止 Webots 并返回失败。

readiness 默认总预算为 `15 s`，成功后额外等待 `1 s` discovery settle 窗口，再启动行为树。两个时间均基于容器 monotonic clock，仅用于进程启动，不用于运动超时。

## 机器人模型

第一版使用 CR100 导航模型加简化 EA200 外形，不整体转换 121 个 joint 的完整 URDF。

必须保留：

- `base_link` 位置和朝向。
- 两个主动轮的安装位置、轮距、半径和差速轴。
- 后侧 caster 支撑。
- 底盘主体、上身简化碰撞体和整体 footprint。
- 最大速度与加速度限制。
- 坐标系约定：`x` 前方，`y` 左侧，`z` 上方；`base_link` 位于主动轮轴线中心，Webots 根节点 `translation` 对应 `base_link` 位置，yaw 是绕 `z` 轴从 `+x` 到 `+y` 的旋转角。

必须固定：

- 双 UF850 机械臂关节。
- 左右夹爪关节。
- 上身升降机构。
- 头部和腕部相机外形。

仿真机器人节点只暴露两个主动轮 Motor。模型中不存在机械臂 Motor、夹爪 Motor 或升降 Motor，从模型层面保证行为树无法驱动手臂。

第一版物理与几何参数：

- 世界 basic time step 为 `16 ms`，Supervisor controller 以 `32 ms` 步进，控制频率约 `31.25 Hz`。
- 初始 Webots 根节点 translation 为 `(0, 0, 0.08)`、rotation 为 identity；`z=0.08` 是主动轮半径，使轮子和 caster 与 `z=0` 地板接触。Supervisor 读取 root pose 后仅取平面 `x/y` 和 yaw 与点位比较。
- 两个主动轮中心相对 `base_link` 为 `x=0`、`y=±0.2305`、`z=0`；半径 `0.08 m`，宽度 `0.078 m`，每个轮质量 `5 kg`，轮胎摩擦系数按 URDF Gazebo 配置采用 `100.0`。
- 两个后支撑不用 URDF 的绕 Y continuous joint 直接转换，而用两个自由 `BallJoint` 被动球体 caster。球心相对 `base_link` 为 `x=-0.436`、`y=±0.18`、`z=-0.0425`；半径 `0.0375 m`，每个质量 `5 kg`，旋转阻尼初值 `0.1`，接触摩擦初值 `0.05`。
- 简化主体使用两个固定碰撞箱：下部箱中心 `(-0.227, 0, 0.163)`、尺寸 `0.70 x 0.60 x 0.35`；上部固定外形箱中心 `(-0.282, 0, 0.818)`、尺寸 `0.50 x 0.34 x 0.97`。
- 简化固定主体合并质量初值 `120 kg`，质心初值 `(-0.18, 0, 0.75)`；惯量按两个碰撞箱尺寸和质量配置计算，不直接使用完整 EA200 的 121 个 joint。
- 每个碰撞箱的惯量按长方体公式计算并写入 Webots `inertiaMatrix`；不得让 Webots 使用未审查的默认物理值。
- 所有质量、质心、惯量、摩擦、阻尼和几何值集中在模型配置中，测试报告记录实际值；这些是烟测模型参数，不宣称等同真实机器人动力学。

第一版使用 Webots Supervisor 读取 ground-truth pose，不模拟定位误差。世界坐标作为 map 坐标系使用。

## 接口设计

为避免复制授权不明的公司代码，`lynsense_pytrees` 仍作为同级目录挂载进容器工作区，不提交到 RPent。

新增一个仿真专用 ROS 接口包，包名声明为 `lynsense_utils`，仅用于本地仿真 overlay。README 必须明确：该包根据行为树调用侧字段重建，不代表拿到公司原始接口定义；如后续拿到真实 `lynsense_utils`，必须先做字段和语义 diff，再决定替换或调整。

接口契约：

```text
NavToPose
  Goal:     string goal_name
  Result:   bool success, string message
  Feedback: float64 distance_remaining

MoveDistance
  Goal:     float64 distance, float64 angle
  Result:   bool success, string message
  Feedback: float64 distance_remaining, float64 angle_remaining
```

类型和单位：

- `goal_name` 是非空字符串，字节长度不超过 `128`，不包含控制字符；未知名称拒绝。无效 UTF-8 若在客户端序列化阶段失败，按客户端错误记录，不宣称 Action Server 能收到后拒绝。
- `distance` 单位为米；正数前进，负数后退。
- `angle` 单位为度；正数逆时针，负数顺时针。
- `angle` 是相对当前朝向的转角，不是 map 下的绝对目标朝向。
- `MoveDistance` 先完成原地转向，再沿机器人本体前后方向执行直线距离。
- `distance=0` 时只转向；`angle=0` 时只直线移动。

Feedback 语义：

- 所有 remaining 字段都是非负数值，方向由 goal 的符号和机器人当前控制阶段决定。
- `NavToPose.distance_remaining` 始终是当前机器人位置到目标点的平面欧氏距离。
- `MoveDistance` 转向阶段：`distance_remaining` 保持为请求距离的绝对值，`angle_remaining` 是剩余转向角绝对值。
- `MoveDistance` 直线阶段：`angle_remaining=0`，`distance_remaining` 是剩余直线距离绝对值。
- 成功、取消或失败终止时，`MoveDistance` 发布当前阶段对应的剩余距离和剩余角度；终止后不再发布增长的重估值。
- NaN、Inf 或负 remaining 值视为服务端内部错误，触发停车和失败。

内置点位：

最小成功世界为 `8 m x 5 m` 的内置 `Plane` collision 地板，不使用远程
`EXTERNPROTO`、`RectangleArena` 或边界墙。点位使用 `(x, y, yaw)`：

```text
home:       (0.0, 0.0, 0.0°)
sim_goal1:  (2.0, 0.0, 0.0°)
sim_goal2:  (0.5, 0.0, 0.0°)
```

该顺序与行为树匹配：从 `home` 到 `sim_goal1` 前进 2 m；随后 `-1.5/+1.2/-1.2 m` 的最终位置为 `x=0.5 m`，即 `sim_goal2`。初始机器人在 `home`，yaw 为 `0°`。

受阻失败用例使用同一地板和机器人，但在 `(1.0, 0.0)` 放置尺寸
`0.10 x 1.00 x 0.50 m` 的固定障碍，遮挡从 `home` 直线到 `sim_goal1` 的路径。
轮与地板的 Webots `ContactProperties` 摩擦为 `100.0`，caster 与地板为
`0.05`；该配置由后续离线运行验证确定，取代早期 `1.0` 摩擦和外部 arena
假设。

`NavToPose` 收到未知点位时 reject，不猜默认位置。

## 数据流

```text
Docker: Ubuntu 22.04 + ROS 2 Humble + Webots
  -> 启动最小 Webots world
  -> 启动 EA200/CR100 简化机器人 controller
  -> controller 创建 /lynsense/nav_to_pose Action Server
  -> controller 创建 /lynsense/move_distance Action Server
  -> 启动现有 lynsense_webots_tree 行为树
  -> Action Client 发送 goal
  -> Webots Supervisor 读取 ground-truth pose
  -> 差速控制器计算左右轮速度
-> Webots Motor 驱动两个主动轮
-> Action Server 发布 feedback
-> 到达目标后返回 success/message
-> 行为树继续下一个节点
-> 测试编排器观察 action 事件跟踪文件
-> 预期最终 SUCCESS/FAILURE 后触发有序退出
```

第一版 `NavToPose` 是直线/goal-tracking 烟测。世界中的点位布置保证最小路径可达。另设带障碍用例验证受阻时能停止并失败，不将该用例称为避障能力。

代码归属：

```text
robots/lynsense/simulation/
```

包含 Webots world、机器人模型、接口兼容包、Supervisor controller、bring-up、测试脚本、Dockerfile/Compose 配置和运行说明。

`lynsense_pytrees` 以只读方式挂载到容器工作区 source 目录，build/install 目录位于容器内独立路径，不写回外部仓库。该包的 `package.xml` 未完整声明 Python 运行依赖；容器镜像必须显式安装并预检 `rclpy`、`py_trees`、`py_trees_ros` 和 `yaml`，不能依赖其包声明推导环境。

Webots controller 在自身同步 step 循环中读取机器人状态并写 Motor 命令；rclpy Action 回调由该 controller 拥有的独立 executor 线程处理。共享目标状态、取消状态和控制命令必须有明确锁或不可变快照，避免 Action 回调与仿真 step 并发读写。executor 线程异常必须让 active goal 停车并失败。

Action Server 在容器内写 JSONL 事件跟踪文件，记录 goal 接受/拒绝、feedback 采样、取消、结果和最终 pose。测试编排器 tail 该文件；观察到预期最后一个 action 结果并确认最终位姿后，向行为树进程发送 SIGINT，再执行 Webots/controller 的统一清理。现有 `lynsense_pytrees_node` 不会在树根节点进入 terminal 状态后自动退出，因此不能把树进程自然退出作为默认预期。

## 控制策略

- `NavToPose` 分三个阶段执行：转向目标点、接近目标点、收敛最终 yaw。前两个阶段使用目标方位角，第三阶段使用点位 yaw。
- `MoveDistance` 分两个阶段执行：先完成相对转向，再沿本体 `+x`/`-x` 直线移动。
- 控制输出经过 CR100 速度与加速度限制。
- 差速正逆解使用轮距 `0.461 m` 和轮半径 `0.08 m`。
- 烟测默认使用低于最大限制的速度，减少打滑和数值抖动。

初始完成容差：

- 位置误差：`0.05 m`
- 朝向误差：`2°`

容差、最大速度、加速度和停滞阈值必须可配置。

烟测默认速度低于硬件上限：线速度 `0.3 m/s`，角速度 `0.5 rad/s`，线加速度 `0.5 m/s^2`，角加速度 `1.0 rad/s^2`。每个阶段的超时按该阶段初始误差、默认速度和加速度计算，使用 `max(5 s, 3 × 理想耗时 + 2 s)`；单个 Action 总预算不超过 `60 s`。

停滞、反馈和超时都基于 Webots `Supervisor.getTime()` 的仿真时间计算，不使用 GUI 渲染壁钟时间。暂停仿真即暂停目标超时。

## 输入校验

- `goal_name` 必须存在且在点位表中；未知点位 reject。
- `distance` 和 `angle` 必须是有限数字。
- `distance` 限制在 `[-10 m, 10 m]`。
- `angle` 限制在 `[-360°, 360°]`。
- NaN、Inf、超限值、空字符串、超长字符串和控制字符均拒绝。
- 点位表启动时逐项校验，任何非有限 pose 值导致启动失败。
- 同一时刻仅允许一个 active goal；已有目标未完成时，新目标 reject。

## 失败、取消与清理

所有失败路径统一执行：

```text
左右轮速度置零
发布最终 feedback
goal handle abort
记录原因
```

必须覆盖：

- 未知点位。
- 参数非法。
- 超时。
- 底盘停滞。停滞判定必须感知当前动作阶段，不得把“位置不变”套用到原地转向，也不得把“角度不变”套用到直线运动。
- 受阻：障碍、接触或机械卡滞导致持续无有效进展。第一版通过当前阶段的目标误差趋势判定，不宣称实现传感器级碰撞检测或通用碰撞恢复。
- 取消请求。
- Webots step 异常。
- rclpy executor 或 Action Server 异常。
- SIGINT/SIGTERM。

阶段感知停滞窗口为 `1.0 s`：

- 转向阶段观察 yaw 误差绝对值是否减少；窗口内减少少于 `0.5°` 判定停滞。
- 直线阶段观察平面目标距离是否减少；窗口内减少少于 `0.005 m` 判定停滞。
- `NavToPose` 最终朝向阶段按转向阶段判定。
- 阶段切换时重置停滞窗口，并保留上一阶段完成误差作为进入条件。

停滞阈值可配置。若控制命令非零但预期误差在阈值内未按方向减少，立即停车并失败；不因无关自由度的进展掩盖停滞。

取消请求先停止轮子，再返回 canceled。行为树节点进入 invalid 时通过 ROS Action cancel 传递取消意图，仿真服务端不猜测行为树内部状态。

进程退出顺序必须是：停车、销毁 Action Server、销毁 Node 和 executor、退出 Webots controller。测试服务和 Compose 配置必须保证失败返回非零退出码，且不遗留 ROS 或 Webots 进程。

## 测试与验收

### 第一层：纯离线单元测试

- 差速正逆解。
- 位置误差和最短 yaw 误差。
- `MoveDistance` 正负距离与正负角度语义。
- `distance=0` 只转向。
- `angle=0` 只平移。
- 参数校验。
- 单 active goal 策略。
- 超时和阶段感知停滞判定，分别覆盖原地转向、直线移动和最终朝向收敛。

### 第二层：容器内 ROS 接口测试

- `colcon build` 成功构建仿真兼容接口、Lynsense Webots bring-up 和同级挂载的 `lynsense_pytrees`。
- 容器镜像预检导入 `rclpy`、`py_trees`、`py_trees_ros` 和 `yaml`。
- source 仿真 overlay 后，`ros2 pkg prefix lynsense_utils` 解析到本地 install 目录，而非任何真实机器人工作空间。
- `ros2 interface show` 显示两个 Action 定义。
- readiness barrier 在启动行为树前确认两个 Action 的类型、server endpoint、Supervisor pose 和左右轮 Motor 可写。
- `ros2 action list` 显示：
  - `/lynsense/nav_to_pose`
  - `/lynsense/move_distance`
- 独立测试客户端验证 unknown goal reject 和非法参数 reject。
- 独立测试客户端发送长时合法目标 `MoveDistance(angle=180, distance=0)`，收到至少两次 feedback 且目标尚未接近完成后再 cancel，确认轮速归零、goal 状态为 canceled、Result `success=false`。不使用 `angle=360`：目标 yaw 会归一化回当前朝向，该 goal 会立即成功而不是长目标。
- feedback 持续更新且数值有限，并按接口设计章节验证分阶段语义。

### 第三层：Webots 行为树烟测

- 无人工干预完整运行 `lynsense_webots_tree.xml`。
- Action 顺序与 XML 一致。
- 所有 Result 为 `success=true`。
- 最终位姿在容差内到达 `sim_goal2` 的 `(0.5, 0, 0°)`。
- 左右轮最终速度为 0。
- 行为树、Action Server 和 Webots 正常退出。
- 测试编排器的 root-terminal watchdog 在预期最终 action 结果和最终位姿确认后触发有序退出，并记录树根终止前不再有新运动命令。
- 保存 action 调用顺序、每步最终 pose、feedback 采样和失败原因。
- 带障碍用例验证受阻/停滞后轮子停止，行为树失败退出。

## 安全与隔离验收

- 容器运行环境显示 `ROS_DOMAIN_ID=42` 和 `ROS_LOCALHOST_ONLY=1`。
- 运行服务使用 `network_mode: none`，不映射 DDS 或 Webots 端口。
- Compose 显式设置 Domain 和 localhost 参数，宿主机 `ROS_DOMAIN_ID=3` 不会被继承；启动前 `AMENT_PREFIX_PATH` 和 `ROS_PACKAGE_PATH` 不包含真实机器人工作空间。
- 不挂载机器人一号 SSH 配置、工作空间或 RPent LLM env。
- `lynsense_pytrees` source 挂载为 read-only，build/install 输出不在该仓库内。
- 仿真模型只包含两个主动轮 Motor。
- 不注册任何真实 xArm、夹爪或底盘控制接口。
- 输出报告明确区分仿真链路成功和真机运动安全。
- 测试结束后容器退出，宿主机无残留 Webots、controller、行为树或 ROS 测试进程；轮子停止，无凭据进入容器。

## 后续阶段

本设计完成并验收后，才进入下一阶段规划：

1. 将一个比赛场景的最小导航片段迁移到仿真。
2. 在仿真中验证该场景的完整行为树导航段。
3. 设计真实底盘只读状态与仿真结果的对照。
4. 在单独设计、现场监督和明确授权下，才考虑任何真机运动。

任何真机运动都必须是新的设计、新的测试门禁和新的用户授权，不能由本仿真通过自动推导。
