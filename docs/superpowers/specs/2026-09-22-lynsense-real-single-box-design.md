# Lynsense RPent Real Single-Box Design

日期：2026-09-22

状态：已自审并通过独立只读复核（2026-09-22，APPROVED）；等待用户确认。
本文只定义设计和验证门禁；不表示已获准连接、部署或运动机器人一号。

## Goal

Build a direct Robot One path in which the RPent API Planner performs the
single-box task through native stepwise tool decisions, while the physical task
uses chassis motion plus coordinated dual arms and dual grippers. The box cannot
be carried safely by one arm, so a right-arm-only demo is not an acceptable
substitute for this milestone.

The accepted Webots task supplies task semantics and an auditable sequence, not
physical trajectories. Real goal names, joint configurations, waist positions,
gripper commands, speeds, tolerances, and calibration values must come from the
robot-side interfaces and supervised site setup. Webots attachment and Isaac
teach values must not be sent to Robot One.

`lynsense_pytrees` is reference material for ROS action names, message shapes,
and company-side calling conventions. RPent must not run that repository's
behavior tree. This design follows RPent's native API Planner model: the model
observes structured state and chooses one tool call at a time.

## Current Evidence And Gap

- The existing real `lynsense` backend has passed one controlled read-only
  `read_robot_state` acceptance. It exposes no motion, perception, navigation,
  gripper, file, image, or finish tool.
- The isolated Webots `rpent-box` gate has completed a 15-action single-box
  scenario, but its action servers and hybrid box attachment are simulation-only.
- Company references show `/lynsense/nav_to_pose` and `/lynsense/move_distance`
  action clients. They do not by themselves prove that those servers, the full
  dual-arm controller, waist, dual grippers, or localization are available in the
  target Robot One session.
- Separately authorized read-only checks confirmed live
  `/industrial_box_pose_node_v3` interfaces: `/industrial_box/pose_base`
  (`geometry_msgs/msg/PoseStamped`), `/industrial_box/status`
  (`std_msgs/msg/String`), and `/industrial_box/trigger`
  (`std_srvs/srv/Trigger`). One approved retry returned `success=true` and a
  fresh selected box pose in `base_link`; the perception service performs no
  robot motion.
- `object_approach` and `ea200_motion_tools` concern the right EA200 arm. They
  are useful safety patterns, but they are not a dual-arm box controller and do
  not satisfy this milestone.
- The live graph also exposes `/plan_arm_simple/compute_trajectory`,
  `/plan_arm_simple/check_dual_collision`, and
  `/plan_arm_simple/get_ik_candidates`, but `plan_arm_simple_msgs` definitions
  were unavailable through the sourced workspace. These services are unresolved
  planning candidates, not reviewed execution or grasp/place controllers.
- No RPent-driven real chassis plus dual-arm plus dual-gripper box-moving path
  exists yet.

Consequently, the first implementation milestone is a read-only Robot One ROS
graph inventory. Motion implementation begins only after that inventory confirms
the interfaces and the user separately approves the next implementation phase.

## Architecture

Add a separate backend named `lynsense_real_box` under
`robots/lynsense_real_box/`. Keep the three existing capabilities isolated:

- `lynsense`: remains the read-only state backend.
- `lynsense_simulation`: remains the offline Webots regression backend.
- `lynsense_real_box`: is the only candidate backend for this supervised real
  task, and starts with no hardware capability until its explicit configuration
  and live-motion gate are enabled.

The original Phase 1 task-flow path is:

```text
RPent API Planner
  -> LynsenseRealBoxToolkit
  -> task state machine and tool guards
  -> LynsenseRealBoxAdapter
  -> reviewed ROS action/service clients
  -> Robot One chassis, waist, dual arms, dual grippers, and perception stack
```

The model does not receive left-arm, right-arm, left-gripper, right-gripper, or
free-form trajectory tools. Coordinated operations are exposed as one tool and
implemented as synchronized commands inside the adapter.

2026-09-23 direction update: the user selected an atomic-capability architecture
rather than opaque `pick_box`/`place_box` task-flow methods. The follow-up
design in `2026-09-23-lynrotcontrol-atomic-capabilities.md` supersedes this
tool boundary for the next implementation: RPent composes reviewed chassis,
waist, paired dual-arm, paired gripper, perception, and stop capabilities over
`lynrotcontrol`; it still receives no single-arm or free numeric motion tool.

## Native RPent Decision Model

The API Planner sees a small task-level tool table and decides one call at a
time. Tool visibility alone is not the safety boundary: all tools can remain
visible in the model tool schema, while execution is denied unless the current
task state, arguments, hardware state, and site mode allow it.

The initial tool table is exactly:

- `read_task_state`
- `read_robot_state`
- `detect_box`
- `nav_to_pose(goal_name)`
- `move_distance(profile_name)`
- `move_waist(profile_name)`
- `move_dual_arms(config_name)`
- `set_dual_grippers(command)`
- `pick_box()`
- `place_box()`
- `stop_task()`
- `finish(status)`

No image reader, file tool, shell tool, arbitrary joint command, Cartesian
motion tool, or separate left/right actuator tool is registered. If the API
Planner injects a common image reader, this backend must disable it through the
existing image-reader capability boundary.

Invalid calls return a structured error to the model and do not create a ROS
goal or service request. A rejected call is recorded as decision evidence. It
does not automatically fail the session unless it violates a live-motion
constraint after hardware dispatch.

`finish` accepts exactly `success` and `failure`. `success` is legal only from
`safe_complete`. `failure` is legal only from
`operator_acknowledged_stopped`. Invalid status, extra fields, an unsafe task
state, or an attempt to use `finish` as a stop command is rejected. Terminal
success must not be inferred from a completed ROS action alone; it also requires
the placement and safe-posture checks below.

## Task State Machine

The first real task is one fixed-layout box cycle:

```text
initialized
  -> box_localized
  -> at_pick_approach
  -> dual_pick_prepared
  -> box_grasped
  -> carrying
  -> at_place_approach
  -> dual_place_prepared
  -> box_released
  -> withdrawn
  -> safe_complete
```

Failure handling is a separate branch from every nonterminal live state:

```text
any nonterminal live task state
  -> failed_stopping
  -> failed_stopped -> operator_acknowledged_stopped
  -> operator_review -> operator_acknowledged_stopped

known external E-stop -> operator_review
```

The state machine constrains execution, not the model's right to reason:

- `detect_box` is allowed before pickup and again before placement when the
  site configuration requires fresh pose evidence.
- `nav_to_pose` is allowed only for the configured pickup and placement goal
  names, and only when the required localization and chassis state are healthy.
- `move_distance(profile_name)` is allowed only in the configured carry/retreat
  transitions. A profile maps to one exact `(distance_m, angle_deg)` pair; the
  model chooses a named transition, never a free numeric distance or turn.
- `move_waist` and `move_dual_arms` accept only named site profiles. The model
  cannot invent numeric joint, Cartesian, speed, acceleration, or payload values.
- `set_dual_grippers` accepts only `open` and `close` in the first version.
- `pick_box` requires fresh box pose, configured dual-arm pose, synchronized
  gripper readiness, healthy dual-arm state, and all configured grasp gates.
- `place_box` requires the box to be marked carried, fresh placement evidence
  when configured, healthy dual-arm state, and synchronized placement readiness.
- `stop_task` and read-only tools remain available in every non-closed state.
- Any dispatched hardware action that fails, times out, cancels, or detects
  disagreement between the two arms moves the task to `failed_stopping`. The
  adapter must request bounded cancellation/stop and attempt to confirm that
  chassis, waist, both arms, and both grippers are stopped.
- `failed_stopping` becomes `failed_stopped` only when software can confirm the
  reviewed stop state and there is no unresolved held-box hazard. It becomes
  `operator_review` when stop confirmation fails, state remains unknown, the box
  may remain partially held, localization is lost while carrying, or an
  asynchronous driver fault is observed.
- A known external E-stop event also enters `operator_review`; software must not
  interpret an E-stop as a completed software stop or automatically reset it.
- Successful cancellation between operations enters `failed_stopped`, not the
  previous or next task state. Unknown cancellation results enter
  `operator_review`.
- No automatic retry is permitted. Live motion tools remain unavailable in every
  failure state. The model may use read-only tools and `stop_task`, but cannot
  restore a motion state.
- `operator_acknowledged_stopped` is entered only through a separate operator
  control surface after the operator has resolved the physical condition. It is
  never set by a model tool call. Starting another physical attempt requires a
  new session and fresh user authorization.

The accepted Webots action order is the reference sequence for the first site
profile. Unlike the simulation backend, the real state machine is not required
to accept only one hard-coded 15-action JSON plan. It enforces physical
preconditions while allowing the API Planner to make successive tool decisions.

## Real Adapter Contract

The adapter owns ROS lifecycle and hides transport details from the Toolkit.
Its interface remains independent of `rclpy` imports so robot discovery, CLI
help, and host tests work without a ROS installation.

It must provide at least:

```text
connect() -> dict
read_state() -> dict
detect_box() -> dict
navigate(goal_name) -> dict
move_distance(profile_name) -> dict
move_waist(profile_name) -> dict
move_dual_arms(profile_name) -> dict
set_dual_grippers(command) -> dict
pick_box() -> dict
place_box() -> dict
stop_motion() -> dict
close() -> None
```

Implementation details are established by the read-only ROS inventory. The
adapter may create only the clients and subscriptions that map to confirmed
interfaces in the site profile. It must not search the graph at tool-call time,
shell out to `ros2`, import `lynsense_pytrees`, or create an unlisted publisher,
service client, or action client.

All potentially physical operations pass through the same guard sequence:

1. site configuration and live-motion mode;
2. task-state legality;
3. argument normalization and whitelist;
4. freshness and validity of required state/perception;
5. reviewed client readiness;
6. bounded goal dispatch, feedback, result, cancellation, and cleanup.

In dry-run mode, the adapter executes the complete guard sequence and records
the exact intended ROS interface and normalized request, but does not send the
goal or service request.

## Dual-Arm And Chassis Semantics

`move_dual_arms(config_name)` is one synchronized operation:

- both left and right commands are normalized from the same site profile;
- both target controllers must be ready, idle where required, error-free, and
  inside configured workspace/joint limits;
- dispatch is bounded and both results are awaited;
- if either side rejects, times out, cancels, or reports an error, the adapter
  stops or cancels the counterpart through the reviewed controller interface and
  initiates `failed_stopping`;
- the tool does not report success on a single-arm result.

`set_dual_grippers(command)` has the same two-sided semantics. `pick_box()` and
`place_box()` are composite guarded operations, not aliases for arbitrary arm
motion. They may call the company's dual-arm box controller if the inventory
finds one; otherwise implementing that controller is explicit follow-up work.
The absence of a reviewed company-side dual-arm grasp/place controller is a
live-motion blocker. If the inventory cannot identify that controller and its
safety evidence, all pick, carry, place, and full-cycle live gates remain not
run. Continuing then requires a new user-approved controller design; this spec
must not be reinterpreted as authorization to synthesize trajectories.

`nav_to_pose` and `move_distance` require the chassis localization and action
interface to be available and healthy. They must honor action cancellation and
zero-motion terminal checks. If only one of these interfaces exists, the full
chassis-plus-dual-arm milestone remains incomplete.

Gripper success is feedback-based, not command-acceptance-based. Both the left
and right grippers must return fresh actual state or position within configured
tolerances of the commanded profile before the synchronized tool succeeds. For
`pick_box`, the box is marked carried only when both grippers pass their grasp
evidence gate in addition to dual-arm arrival. Approved evidence may be contact,
force/current, or a reviewed controller signal; if the target hardware exposes
none of these, physical grasping remains blocked. If the two grippers disagree,
arms stop, the box is not marked carried, and the session follows the
`failed_stopping` / `operator_review` path. The site profile determines whether
the safest failure posture keeps grippers closed or opens them; that policy is
operator-reviewed configuration and is never chosen by the model.

## Perception Contract

`detect_box` triggers one configured inference, waits within a bounded interval
for a fresh selected box pose and successful status, and returns structured
pose, frame, timestamp, confidence/error information, and freshness. It does not
return raw camera images to the model.

The confirmed contract is the `/industrial_box/trigger` service plus the
already-publishing `/industrial_box/pose_base` and `/industrial_box/status`
topics from `industrial_box_perception`. The client must install both topic
subscriptions before starting exactly one trigger request. A trigger response
timeout is not by itself a perception failure: bounded topic observation and
validated `success`, frame, transform, timestamp, and freshness evidence decide
the result. No automatic trigger retry is permitted.

A pose older than the configured threshold is unavailable. `pick_box` and
`place_box` do not accept stale perception. Missing calibration, an invalid
frame, or an unverifiable transform is a failed precondition and must not be
converted into guessed offsets.

Placement has its own finite acceptance profile: target pose and yaw tolerances,
supported placement region, required release evidence, post-release settle time,
and stability criteria. Successful placement requires verified dual-gripper
open state, release evidence from the reviewed controller, the box remaining in
the supported region within the configured pose and yaw tolerances, and no
configured stability violation during the settle interval. Fresh perception may
supply the pose check when its frame and calibration are valid; otherwise an
operator confirmation outside the model tool boundary may be recorded as site
evidence. A box dropped outside the region, a near miss, or unverified release
is not successful placement.

## Configuration And Authorization

The backend is disabled by default. A live session requires separate operator
configuration; a model call can never enable it.

The first version supports only API Planner, local memory, and an ordinary
TTY. Dashboard, interactive mode, exploration, remote execution helpers, and
planners that inject general-purpose file or shell tools are rejected.

The site profile is local, reviewed configuration and contains:

- ROS Domain expectation, currently `3`;
- confirmed topic, service, and action names and types;
- state freshness thresholds and action budgets;
- pickup and placement navigation goal names;
- exact distance and turn profiles;
- waist profiles;
- dual-arm named configurations;
- gripper commands and timing;
- perception frame, timeout, and freshness threshold;
- synchronized grasp/place gates;
- approved grasp and release evidence;
- placement pose, yaw, containment, settle, and stability criteria;
- failure posture for a partially held box;
- dry-run/live mode.

The reviewed profile has an explicit identifier and content hash. Every offline,
dry-run, and live evidence bundle records that identifier and hash so a result
cannot be attributed to a different site configuration.

`ROS_DOMAIN_ID=3` must be present before ROS initialization; RPent does not
mutate the process environment or source a robot workspace. The adapter keeps
exclusive ROS context ownership and performs bounded, ordered cleanup.

Software `stop_task()` is a best-effort cancellation/stop request. It is not an
E-stop and must never be described as one. Every live run requires a person at
the physical E-stop, a cleared envelope, site-approved speed/payload limits, and
explicit user authorization for that run.

## Error Handling

Every tool returns a structured result with status, reason, task state, and
evidence identifiers rather than raising across the model boundary. At minimum:

- unavailable/stale/invalid robot or perception state;
- illegal task transition;
- unknown or extra argument;
- interface missing or not ready;
- rejected, failed, timed-out, or canceled action;
- dual-arm disagreement or single-sided completion;
- external E-stop, asynchronous driver fault, or stop confirmation failure;
- localization loss or an unverifiable transform while carrying;
- gripper feedback disagreement or absent grasp/release evidence;
- live mode requested while configuration remains dry-run;
- adapter cleanup failure.

Cancellation and interruption must preserve the pending ROS request, request
cancellation, wait within bounded budgets, and then destroy clients and node.
Cleanup failure is reported and retained for retry; it is not converted into a
successful close. A failed hardware action does not automatically retry.
Unknown stop state, a possible partially held box, external E-stop, asynchronous
driver fault, or localization loss requires human review; software must not
continue, reopen grippers under load unless the reviewed failure profile says
that is safe, or report terminal failure before the operator acknowledges the
stopped physical state.

## Evidence

Each session writes:

- model transcript and tool-call/result timeline;
- site-profile identifier and content hash;
- normalized intended interface and request in dry-run;
- dispatched hardware events in live-run;
- task-state transitions;
- state/perception freshness evidence;
- cancellation/stop behavior;
- final task result and operator context.

No credentials, API keys, SSH configuration, or robot-workspace secrets may be
copied into artifacts or project memory.

## Validation Gates

### Offline implementation gate

Use fake ROS clients and fake perception; do not connect to Robot One:

- exact final API tool table, including no injected image/file/shell tool;
- argument rejection and normalization;
- legal and illegal state transitions;
- dry-run produces intended requests but no client dispatch;
- successful chassis, waist, dual-arm, dual-gripper, pick, and place paths;
- action reject, failure, timeout, feedback, cancellation, and cleanup paths;
- stale/unavailable right or left arm state and perception;
- dual-arm dispatch disagreement and single-sided result handling;
- left/right gripper feedback disagreement and missing grasp evidence;
- successful placement containment, release, settle, and stability checks;
- stop/final state after each dispatched failure;
- external E-stop, asynchronous fault, stop failure, and localization loss;
- operator review and operator-acknowledged stop transitions;
- legal and illegal `finish` states and status values;
- adapter context ownership and bounded cleanup;
- FunctionModel transcript showing multiple successive RPent decisions, not a
  one-call static plan submission;
- invalid model choice denied before hardware dispatch.

### Robot read-only inventory gate

With separate user approval, record the target ROS graph using only observation
commands. Confirm names, types, publishers, action servers, service servers,
state availability, permissions, and runtime owners for chassis, waist, both
arms, both grippers, localization, and box perception. No motion, enable,
clear-error, parameter write, deploy, or process start is allowed in this gate.
The inventory must also identify a reviewed dual-arm grasp/place controller and
its feedback signals. If that controller or its safety evidence is absent, the
pick/carry/place/full-cycle live gates are blocked and a new controller design
must be approved before those gates can be scheduled.

### Dry-run gate

Run the real RPent Planner and real configuration with adapter motion disabled.
Accept only if the model's stepwise decisions, state transitions, normalized
requests, tool results, and cleanup satisfy the expected site profile and no
physical request is dispatched.

### Component live gates

Each live gate requires a new user authorization, physical E-stop supervision,
cleared envelope, and site-approved limits:

1. chassis navigation or bounded move;
2. waist profile;
3. dual-arm safe named configuration;
4. dual-gripper open/close;
5. box perception at pickup and placement;
6. supervised dual-arm pick;
7. carry and place;
8. full fixed-layout single-box cycle.

A component gate may be skipped only when the user explicitly changes scope.
The full milestone cannot be claimed from a subset because this task requires
coordinated chassis and dual-arm behavior.

### Final acceptance

The final acceptance requires:

- the physical box is moved from the configured pickup location to placement;
- RPent made the decisions through successive tool calls;
- every dispatched action has evidence;
- no unlisted hardware interface was called;
- dual-arm and dual-gripper operations completed synchronously or stopped;
- final posture reaches the configured safe state;
- failure paths stop without automatic retry;
- logs distinguish planner decisions from hardware results;
- the operator confirms the supervised run.

## Out Of Scope

- Two-box or full competition flow.
- General obstacle avoidance, dynamic replanning, or free-form exploration.
- Model-generated joint trajectories, Cartesian paths, speeds, or payloads.
- Automatic recovery or retry after hardware failure.
- Using Webots attachment or Isaac teach values as real motion.
- Installing, launching, writing files, or starting RPent processes on the Robot
  One host during implementation; any direct ROS connection from an operator
  workstation is itself separately authorized.
- Replacing, bypassing, or emulating the physical E-stop.
- Modifying the existing read-only `lynsense` backend into a motion backend.

## Open Site Questions

These are inventory questions that must be resolved in the approved Robot One
inventory record before live-motion implementation:

- Which chassis `nav_to_pose` and `move_distance` servers are active, and what
  are their exact action types?
- Which waist and dual-gripper interfaces are available?
- Which left- and right-arm state and control interfaces are approved?
- Does a reviewed company dual-arm box controller already exist?
- Which localization and map frames are authoritative?
- Is the industrial-box perception service active with calibrated cameras and a
  valid target frame?
- What are the site-approved speed, payload, clearance, and E-stop procedures?

The design remains blocked for live motion until these questions are answered
from Robot One and accepted by the user.
