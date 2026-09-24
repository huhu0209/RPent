# Lynsense Webots Single-Box Task Design

Date: 2026-09-21

Status: design approved by the user; implementation has not started. The
independent document review is currently `NOT RUN` because both configured
subagent attempts timed out; the local consistency contradiction found during
self-review has been corrected.

## Goal

Extend the verified Lynsense Webots navigation baseline into an end-to-end,
simulation-only task that moves one industrial box:

1. navigate to `搬箱子1`;
2. prepare the waist, dual arms, and grippers;
3. pick `box1`;
4. back away and turn;
5. navigate to `放箱子1_1`;
6. place the box;
7. retreat and return to a safe posture.

The stage deliberately implements one complete box cycle. The second box and
full WHRG2026 two-box flow are follow-up work after this gate passes.

## Current State And Inputs

- The existing `interface`, `smoke`, `blocked`, and `match` gates pass in an
  isolated Webots/ROS 2 Humble container.
- The current `match` gate is navigation-only. It does not include waist,
  arms, grippers, box physics, perception, collision avoidance, or the full
  competition task.
- The corrected chassis model is stable under the isolated straight/turn
  regression. Its wheel geometry, contact settings, and stability evidence
  must not be regressed.
- The sibling `lynsense_pytrees` checkout does not contain
  `lynsense_plan_move2box_tree.xml` on its checked-out branch. The complete
  source is available from `origin/WHRG2026`, commit `a70e301`.
- The WHRG2026 tree uses `NavToPose`, `MoveDistance`, `PlanMoveWaist`,
  `PlanMoveNamedConfig`, `PlanSetGripper`, `PlanBoxPhase`, and parallel
  synchronization. Its original manipulation behaviours execute company
  `plan_arm_simple` programs and therefore must not be used directly in this
  simulation.
- The local company robot archive contains an EA200/CR100 URDF with one waist
  prismatic joint, two six-degree-of-freedom UF850 arms, and two grippers.
  The archive and its company meshes remain outside the repository and are
  only parameter references.
- The perception checkout defines the industrial box as
  `0.397 x 0.295 x 0.217 m`. Perception itself is not used for decisions in
  this stage.

## Non-Goals

- Do not connect to robot one or use `ROS_DOMAIN_ID=3`.
- Do not publish DDS ports, source a robot workspace, or call a real chassis,
  arm, waist, gripper, enable, clear-error, or emergency interface.
- Do not mount `.env.lynsense`, SSH configuration, home directories, robot
  workspaces, or the company URDF archive into the simulation container.
- Do not copy, commit, or redistribute company STL/URDF assets.
- Do not integrate the RPent Planner, LLM, Dashboard, or a Lynsense motion
  toolkit in this stage.
- Do not put perception into the decision chain.
- Do not implement SLAM, AMCL, Nav2, dynamic obstacle avoidance, or the full
  competition map.
- Do not claim that simulation success calibrates real dynamics or authorizes
  real-robot motion.
- Do not weaken or replace the existing passing navigation and chassis gates.

## Scope Decision

The user selected three material options:

1. **One-box end-to-end scope**: complete one pick-and-place cycle before
   attempting the two-box competition flow.
2. **Hybrid grasp fidelity**: the main task uses an attachment after geometric
   and state validation; a separate gate tests physical contact.
3. **Competition-tree-compatible slice**: preserve the semantic node types and
   task ordering of the WHRG2026 tree, but implement simulation-owned adapters
   beneath them.

This balance lets task orchestration, navigation, mechanical motion, video,
and event evidence stabilize before friction/contact tuning and RPent policy
integration multiply the debugging surface.

The first implementation serializes the WHRG2026 transitions that originally
run navigation and manipulation in parallel. This is an explicit first-gate
trade-off: it avoids concurrent wheel and upper-body actuation while the
controller, event contract, and mechanical model are still being established.
The tree keeps the same semantic node types and task order, but every step
waits for the preceding step to terminate. Restoring the competition tree's
parallel transitions is follow-up work after the serial gate and chassis
stability regression pass.

## Architecture

Add a new isolated `box` phase alongside the existing phases:

```text
robots/lynsense/simulation/
  docker compose gate
    -> Webots box world
    -> simulation controller
    -> navigation Actions
    -> manipulation Actions
    -> RPent-owned single-box py_trees XML
    -> events-box.jsonl and summary.json
```

The runtime remains on the existing container baseline:

```text
Webots R2025a + ROS 2 Humble + ROS_DOMAIN_ID=42 + ROS_LOCALHOST_ONLY=1
```

The no-network regression container continues to run without published DDS
ports. The later viewer service may publish only its existing HTTP port.

### Task Sequence

The first single-box tree is:

```text
EnsureSimulationManipulationServices
NavToPose(搬箱子1)
PlanMoveWaist(height_mm=200)
PlanSetGripper(position=0.0)
PlanMoveNamedConfig(target="dualjo:joints_br")
PlanBoxPhase(action="pick", flow="flow", config="box1")
MoveDistance(distance=-0.6, angle=0)
MoveDistance(distance=0, angle=90)
PlanMoveNamedConfig(target="dualposi_armbase_abso:pt_1f1_ready")
NavToPose(放箱子1_1)
PlanBoxPhase(action="place", flow="flow", config="box1")
MoveDistance(distance=-0.6, angle=0)
PlanMoveNamedConfig(target="dualposi_armbase_abso:pt_up")
PlanMoveWaist(height_mm=200)
PlanSetGripper(position=0.0)
PlanMoveNamedConfig(target="dualjo:joints_s")
```

This intentionally retains the first box cycle and recovery posture from the
WHRG2026 flow while omitting `box4`, the second cycle, joint-limit profile
changes, manual-check fallbacks, and concurrent navigation/manipulation
transitions. The omitted parallelism must be documented in the run evidence;
it is not silently presented as full competition-tree equivalence.

The tree must be an RPent-owned XML file. It may use the visual vocabulary of
the company tree, but manipulation nodes must instantiate simulation-owned
behaviours; they must not spawn `plan_arm_simple` executables.

## Webots Robot Model

The current CR100 chassis and corrected wheel/contact model remain the base.
Add the minimum actuated upper body rather than converting all 121 URDF
joints.

### Waist

- One Webots sliding joint models `connector_joint`.
- Its parent transform, child transform, axis, limits, speed, mass, and center
  of mass are taken from the company URDF as simulation parameters.
- The URDF travel is `[-0.478, -0.018] m` with velocity `0.0248 m/s`.
- `PlanMoveWaist(height_mm=200)` maps to a configured position in that range.
  The mapping and its zero reference are explicit in `box.yaml`.

### Arms

- Two six-degree-of-freedom arms use names aligned with the URDF:
  `left_joint1..left_joint6` and `right_joint1..right_joint6`.
- Joint parent/child transforms, axes, limits, masses, centers of mass, and
  inertia tensors are sourced from the URDF and stored in the simulation
  configuration.
- Visual and collision geometry use Webots primitive shapes. Company meshes
  are not copied.
- Each arm has a deterministic `ready`, `grasp`, `carry`, and `place` set of
  joint targets stored in `box.yaml`.
- Motion uses bounded interpolation toward position targets. It does not use
  Webots infinite-velocity position teleporting.

### Grippers

- Each side has a simplified base, two opposing finger pads, and one master
  open/close motor.
- `PlanSetGripper(position)` maps `0.0` to closed and `0.6` to the simulation's
  open command; intermediate values are interpolated.
- The CTAG mimic linkage is not modelled in the main task. Finger geometry is
  sufficient for contact checks in the separate physical grasp gate.

### Box

- Initial dimensions are `0.397 x 0.295 x 0.217 m`.
- The first simulation mass baseline is `1.0 kg`, explicitly recorded in
  `box.yaml`.
- The box has both visual and bounding geometry, finite mass and inertia, and
  contact material settings.
- The initial box pose is defined relative to the `搬箱子1` navigation frame.
- The place target is defined relative to the `放箱子1_1` navigation frame.

These are task-regression parameters, not calibrated company-box material
properties.

## Hybrid Grasp Model

The main `box` gate uses a validated attachment so end-to-end orchestration is
not blocked by contact solver tuning.

### Pick

`PlanBoxPhase(action="pick")` succeeds only when all of the following hold:

1. waist position is within its configured tolerance;
2. all twelve arm joints are within their configured joint tolerance;
3. both grippers are in the closed command state;
4. the box's relative position error is below `0.03 m`;
5. the box's yaw error is below `10 degrees`;
6. the controller has observed every required state for at least one complete
   controller step.

Once these conditions hold, the controller switches the box to attached mode.
At each simulation step it sets the box pose from a fixed transform relative to
the robot's carrying frame. The box is excluded from ordinary dynamic response
while attached, but the controller records its relative pose and detects drift.

If any precondition fails, the action fails, all motors stop, and the box
remains in its pre-attachment physical state.

### Carry

Attached mode continues through navigation and arm motion. The controller
computes the box pose from the carrying frame after each Webots step and
records:

- world box pose;
- box pose relative to the carrying frame;
- attachment state;
- maximum relative translation and rotation error.

If translation error exceeds `0.02 m`, the action state becomes
`carry_lost`, all motors stop, and the tree terminates without automatic
retry. The box remains attached on failure so the evidence does not introduce
an artificial drop.

### Place

`PlanBoxPhase(action="place")` may release only after:

1. the robot navigation pose is inside the configured place tolerance;
2. all twelve arm joints are within their place tolerance;
3. the attached box's computed world pose is within `0.05 m` of the configured
   place target in `x/y`;
4. yaw error is below `10 degrees`;
5. roll and pitch are below `5 degrees`;
6. both grippers have reached the open command state.

Release restores the box's dynamic body, contact response, and bounding
collision. The gate records at least `0.5 s` of post-release samples and
requires the box to settle without excessive roll, pitch, or lateral motion.

## Simulation-Only ROS Interfaces

Keep the existing action endpoints and semantics:

```text
/lynsense/nav_to_pose
/lynsense/move_distance
```

Add four simulation-only endpoints:

```text
/lynsense/move_waist
  Goal:     float64 height_mm
  Result:   bool success, string message
  Feedback: string phase, float64 position_mm, float64 remaining_mm

/lynsense/move_named_config
  Goal:     string target, bool initok
  Result:   bool success, string message
  Feedback: string phase, float64 progress, float64 max_joint_error_rad

/lynsense/set_gripper
  Goal:     float64 position
  Result:   bool success, string message
  Feedback: string phase, float64 position, float64 remaining

/lynsense/box_phase
  Goal:     string action, string flow, string config, bool initok
  Result:   bool success, string message
  Feedback: string phase, float64 progress, float64 box_error_m
```

`height_mm`, `position`, and all feedback values must be finite. Remaining
values are non-negative. Empty strings are rejected. The first implementation
supports only `flow="flow"` and `config="box1"`.

### Named Config Whitelist

`/lynsense/move_named_config` accepts only:

```text
dualjo:joints_s
dualjo:joints_br
dualposi_armbase_abso:pt_1f1_ready
dualposi_armbase_abso:pt_up
```

Each target resolves to:

- twelve arm joint targets;
- a required waist position;
- required gripper states;
- interpolation speed and acceleration limits;
- arrival and timeout tolerances.

Targets are simulation-taught values in RPent-owned `box.yaml`; they are not
represented as values imported from the company simpleplan database.

### Tree Adapter Ownership

The single-box tree uses RPent-owned behaviour classes with the same external
semantics as the WHRG2026 node names. The adapters are Action clients, not
subprocess wrappers:

```text
PlanMoveWaist       -> /lynsense/move_waist
PlanMoveNamedConfig -> /lynsense/move_named_config
PlanSetGripper      -> /lynsense/set_gripper
PlanBoxPhase        -> /lynsense/box_phase
```

`EnsureSimulationManipulationServices` waits for all six action endpoints and
uses the existing short client discovery budget pattern. It does not wait for
company MoveIt or `plan_arm_simple` services.

The adapter must distinguish terminal rejection, execution failure, process
shutdown, and timeout. Tree cancellation must cancel an active Action goal and
stop ticking only after the result or cancellation result is observed.

## Controller State Model

Navigation and manipulation share one controller-level resource lock:

- a navigation goal is rejected while any manipulation action is active;
- a manipulation goal is rejected while navigation is active;
- only one manipulation action may be active at a time;
- a second manipulation goal is rejected, not queued;
- cancellation and terminal state are serialized through the controller lock;
- terminal state remains locked until the tree process has exited.

The existing two navigation action servers remain unchanged. The manipulation
state machine is separate from `ActionRuntime` and does not rewrite navigation
acceptance semantics.

### Manipulation Phases

`move_waist`:

```text
accepted -> accelerating -> moving -> settling -> terminal
```

`move_named_config`:

```text
accepted -> interpolating -> settling -> terminal
```

`set_gripper`:

```text
accepted -> closing_or_opening -> settling -> terminal
```

`box_phase(pick)`:

```text
accepted -> aligning -> closing -> verifying -> attached -> terminal
```

`box_phase(place)`:

```text
accepted -> aligning -> opening -> releasing -> settling -> terminal
```

Every phase transition records the reason and the measured error that caused
it.

## Error Handling

Reject malformed goals before motion starts:

- non-finite numbers;
- waist target outside its configured range;
- gripper target outside `[0.0, 0.6]`;
- empty or unknown target/config/action;
- unsupported `flow`;
- active navigation or manipulation action;
- terminal-locked controller state.

Runtime failures:

- `joint_stall`;
- `motion_timeout`;
- `box_alignment_failed`;
- `grasp_failed`;
- `place_alignment_failed`;
- `carry_lost`;
- `release_settle_failed`;
- `simulation_fault`.

On any failure:

1. command zero velocity for both wheels, waist, all twelve arm joints, and
   both gripper master joints;
2. publish a terminal result with the reason;
3. record the complete event;
4. terminate the tree without automatic retry;
5. preserve logs and artifacts;
6. retain an attached box in attached mode.

Cleanup must be idempotent. Losing Webots or the controller still requires the
orchestrator to terminate all child process groups and write a failed summary.

## Configuration

Add an RPent-owned `box.yaml` with at least:

- box dimensions, mass, inertia, initial and target poses;
- waist joint, direction convention, travel mapping, and tolerances;
- left/right arm joint names, limits, velocity and acceleration limits;
- gripper open/closed mapping and tolerances;
- every named config from the whitelist;
- grasp/carry/place thresholds from this design;
- action, settling, stall, and total-budget timeouts;
- tree node and action sequence expected by the gate.

Configuration loading must fail closed on missing keys, duplicate joint names,
wrong arm cardinality, non-finite values, out-of-range targets, or unknown
named configs.

The configuration must not import or parse the company archive at runtime.

## Evidence

Each `box` run creates a unique artifact directory containing:

```text
events-box.jsonl
summary.json
webots.log
controller.log
tree.log
```

The unified `summary.json` records phase, run ID, outcome, action evidence
files, final robot pose, final box pose, attachment state, and terminal wheel,
waist, arm, and gripper rates.

Every action feedback/result records:

- action name and phase;
- robot planar pose;
- waist position;
- twelve arm positions and errors;
- both gripper positions;
- box pose;
- box error;
- attachment state;
- motor rates;
- simulation time;
- run ID;
- terminal reason.

No event contains credentials, host paths outside the container, or private
company asset contents.

## Viewer Extension

Add `LYNSENSE_VIEWER_PHASE=box` after the headless gate is stable.

The viewer must:

- retain the third-person scene and robot camera;
- show task phase, waist position, gripper states, box pose, and attached
  state;
- show the complete pick, carry, place, and retreat sequence;
- preserve live HLS and replay MP4 behavior;
- label the run as simulation-only and `real_robot_connected: false`.

The viewer must not add a start/stop control endpoint or expose ROS DDS.

## Testing

### Offline Unit And Static Tests

- Validate every `box.yaml` section, joint name, cardinality, range, finite
  value, named config, and expected tree sequence.
- Validate the XML structure, node parameters, strictly serial transition
  order, absence of navigation/manipulation parallel branches, and whitelist.
- Validate manipulation state transitions for every normal and failure path.
- Validate navigation/manipulation mutual exclusion and terminal lockout.
- Validate attached state transitions, release preconditions, and failure
  retention.
- Validate event schema and finite values.
- Validate the world contains the expected motors, bounding objects, box
  dimensions, and no remote assets.
- Validate Compose still mounts only the approved simulation, pytrees, and
  artifact paths.

### Mechanical Regression

Run an isolated world/controller check that:

1. moves the waist to its configured target;
2. moves both arms to each named config;
3. opens and closes both grippers;
4. returns to a safe posture;
5. verifies joint arrival, bounded duration, and zero terminal rates;
6. repeats the current chassis stability probe.

### Physical Grasp Regression

Add a separate `grasp-physics` scenario with the chassis fixed:

1. arms move to grasp posture;
2. finger pads close against the box;
3. contact is observed on both sides;
4. the waist or arms lift the box;
5. contact remains present during a short hold;
6. grippers open and the box falls under physics;
7. terminal motor rates are zero.

This gate may be tuned independently and does not use the main-task
attachment. A failure here is explicitly reported as a physical-grasp gap even
when the hybrid end-to-end gate passes.

### End-To-End Box Gate

The gate requires the exact accepted/result sequence and terminal state:

```text
nav_to_pose(搬箱子1) succeeded
move_waist(200) succeeded
set_gripper(0.0) succeeded
move_named_config(dualjo:joints_br) succeeded
box_phase(pick, box1) succeeded and box attached
move_distance(-0.6, 0) succeeded
move_distance(0, 90) succeeded
move_named_config(pt_1f1_ready) succeeded
nav_to_pose(放箱子1_1) succeeded
box_phase(place, box1) succeeded and box detached
move_distance(-0.6, 0) succeeded
move_named_config(pt_up) succeeded
move_waist(200) succeeded
set_gripper(0.0) succeeded
move_named_config(dualjo:joints_s) succeeded
```

Initial numerical limits:

- navigation pose tolerance: `0.05 m`, yaw `0.0349 rad`;
- arm joint arrival error: `< 0.03 rad`;
- grasp box alignment error: `< 0.03 m`;
- grasp yaw error: `< 10 degrees`;
- attached relative translation drift: `< 0.02 m`;
- place planar error: `< 0.05 m`;
- place yaw error: `< 10 degrees`;
- post-release roll/pitch: `< 5 degrees`;
- post-release settle observation: at least `0.5 s`;
- all wheel, waist, arm, and gripper terminal rates: `0`;
- total simulation budget: `180 s`.

These are regression thresholds, not real-robot accuracy or safety limits.

### Regression Matrix

Before delivery, all of the following must pass:

1. focused Lynsense unit/static tests;
2. the full existing Lynsense simulation test suite;
3. the original `--phase all` navigation gate;
4. chassis stability;
5. mechanical regression;
6. physical grasp regression;
7. end-to-end `box`;
8. viewer replay checks for the existing phases and `box`.

If runtime constraints prevent a complete matrix run, the delivery must report
the exact missing gate rather than implying success.

## Acceptance

This stage is complete when:

- one box is visibly and event-verified as picked, carried, placed, and
  released in the simulation;
- the serial competition-compatible single-box action sequence passes;
- the simplified actuated waist/arms/grippers reach configured targets and
  stop at terminal state;
- the separate physical grasp gate reports measurable contact behavior;
- chassis navigation and stability regressions still pass;
- the viewer preserves a replayable dual-view record;
- evidence clearly distinguishes hybrid attachment from physical contact;
- no real-robot connection, perception decision input, RPent policy control,
  or real-robot authorization is introduced.

## Follow-Up Work

After this gate passes:

1. extend the same sequence to the WHRG2026 two-box flow;
2. add simulated RGB/depth sensor publication with ground-truth comparison;
3. integrate the existing perception package as an advisory input;
4. introduce the Lynsense simulation/RPent toolkit boundary;
5. only then design separately authorized real-robot validation.
