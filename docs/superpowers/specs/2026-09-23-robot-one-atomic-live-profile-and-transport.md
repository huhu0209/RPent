# Robot One atomic live profile worksheet and transport design

Date: 2026-09-23

Status: offline design only. This document does not authorize a Robot One
connection, transport construction, calibration, recovery, or motion.
The offline coordinator/Toolkit has passed local regressions. A later independent
read-only review found a close/stop/dispatch race, which was fixed and covered
offline. A blocked synchronous submission now makes `stop_all()` fail within its
lock-acquisition budget, but the stop request may not have been sent. This is
not a physical stop guarantee. Live transport remains blocked until every live
gate below passes under fresh explicit authorization.

## Decision

The live path remains:

```text
RPent API Planner
-> LynsenseAtomicToolkit
-> LynrotControlAtomicAdapter
-> LynrotControl robot object
-> separately reviewed chassis runtime
-> separately reviewed perception transport
```

The company pytree task flow is not imported or executed. `plan_arm_simple` is
optional assistance for IK, trajectory, or collision checking only; it is not a
required task-flow or grasp/place orchestrator. Its service definitions remain
unavailable in the reviewed environment, so no live dependency is allowed.

## Review Status

Two independent read-only reviews found that the first offline skeleton was not
safe for live transport. The following Critical/High fixes are now implemented
and covered by the focused offline suite:

- fresh current joints must match each paired profile's first waypoint;
- unknown LynrotControl return objects fail closed;
- stop requests are dispatched in parallel and stop means confirmed request
  plus fresh physical stability within one monotonic budget;
- reconnect cannot clear review, and active motion blocks close;
- review acknowledgement is explicit and non-model;
- a rejected `finish` tool result cannot terminate `ApiAgentLoop`;
- finish authorization is bound to both the serialized `finish` result name and
  its `_finish` payload;
- force hold timing is monotonic and an already-high force blocks dispatch;
- adapter, evidence recorder, and Toolkit must agree on the profile SHA-256.
- effectful calls persist a `tool_intent` event before dispatch and link the
  completion event to that intent;
- arm joint feedback uses a monotonic sequence high-water mark across
  preflight, start, and final arrival checks;
- close is blocked until fresh subsystem stability is confirmed, and a partial
  Toolkit close failure can be retried.
- if normal completion evidence fails after dispatch, the Toolkit requests a
  confirmed software stop and writes a linked minimal
  `tool_completion_failure`; if that fallback write also fails, the run fails
  rather than reporting success.

The latest independent read-only review found a Toolkit guard rejection bypass
and a cancellation `NameError`. Both were repaired locally; v2 `stop_all()` now
requests a confirmed stop from both grippers as well as the arms, waist, and
chassis. The API finish result is bound to tool name, call ID, and `_finish`
payload. On 2026-09-23 the offline suite passed (`1050 passed, 3 skipped`,
excluding the existing pi05 contract file), including API Planner tests. Earlier
API test timeouts were caused by sandbox stream-FD restrictions, not established
Pydantic AI incompatibility. The post-fix independent read-only review found
one High: a timed-out synchronous stop request may complete after acknowledgement
and resume. An offline one-way lockout now blocks acknowledgement, resume, new
actions and close while a timed-out cancel is running; late completion cannot
clear it. Both general and gripper-specific stop paths have regression tests.
Stop now serializes its state transition against final transport submission;
four barrier tests confirm no command is submitted after a confirmed stop,
and a late completion cannot overwrite review with idle. Toolkit cancellation
also requests stop when a tool is active but still in an idle-mode preflight.
Stop submission and confirmation exceptions, including a partially submitted
stop batch, now latch the instance against acknowledgement, resume, and motion;
in-flight stop futures must settle before close. These later exception-path
changes have local regressions but have not yet been independently re-reviewed.
Its Important finding (an unrelated tool result consumed a pending finish) was
reproduced and fixed with a regression test. A subsequent independent read-only
review found that `close()` could race a new stop or motion dispatch. The close
path now rechecks stop/dispatch generations before committing `closed`, and a
stop rechecks `closed` when registering intent. New offline barriers cover a
blocked synchronous submit, paired second-side suppression, resume racing a
stop, and close racing a stop or waist submit. After these fixes the offline
suite passed (`1057 passed, 3 skipped`, excluding the existing pi05 contract
file). The final close-race patch has local tests but no independent follow-up
review. No live run occurred.

Remaining live blockers include bounded/interruptible synchronous transport
submission with an independently available stop path (a blocked submit can
prevent any software stop request from being sent), cumulative session budgets,
independent review of the final close-race patch, live ROS implementations of runtime and
perception identity handshakes, orphaned intent reconciliation when both
completion writes fail, tamper-evident evidence replay, actual reviewed gripper
stop/cancel transport, physical waist/gripper limits, and the real
chassis/perception composition root. The v2 validator and offline fake-runtime
failure matrix do not authorize live motion.

The on-site hardware E-stop is an independent contingency, not a substitute for
transport cancellation, bounded stop requests, fresh stability feedback, or
the operator's ability to abort an ordinary test without an E-stop.

## Next Read-Only Gate

The recorded ROS graph and box-perception sample below are historical evidence,
not a current runtime handshake. The next gate is **evidence collection only**:

1. Before any new SSH/ROS session, obtain authorization naming Robot One, the
   operator, `ROS_DOMAIN_ID=3`, the exact approved read-only runbook and its
   digest, and the allowed command families. The prior inventory approvals do
   not authorize a new session. Check Asia/Shanghai time before connecting.
2. Under that runbook, record fresh left/right joint state, force state, waist,
   gripper and chassis feedback names, types, timestamps, and sample sequences
   where exposed by the messages; record missing fields explicitly.
   Compare them with the historical graph; an absent or inconsistent interface
   is a recorded blocker, not a reason to try a guessed substitute.
3. Separately review the selected LynrotControl instance's configuration and
   construction side effects **offline**. In the local reference source,
   `lynrotcontrol/implementation/initialization.py::initialize()` initializes
   both arms, and `lynrotcontrol/lynarmcontrol/interfaces/initialization.py`
   calls `request(op="initialize", start=True)`. It is not a read-only probe.
   `get_model/get_ip/get_axes`, bindings, limits, and force-frame metadata must
   be tied to the actual selected instance. Do not instantiate it on Robot One
   merely to read values; this requires separate transport review and approval.
4. Record each site-profile number with units, source, timestamp, and reviewer.
   Do not import a value from the simulation, pytree scripts, or dry-run tests.
   Missing numeric evidence stays missing; it does not become a motion profile.

The output of this gate is a reviewed identity/interface evidence record and a
list of missing numeric measurements. It does not create a live composition
root, enable the driver, publish a command, call a service, trigger perception,
or authorize any single-component or box-handling motion.

## Confirmed Robot One Evidence

The following values come only from recorded read-only artifacts:

- Selected ROS environment: `ROS_DOMAIN_ID=3`,
  `ROS_LOCALHOST_ONLY=0`.
- Workspace review succeeded for `/opt/ros/humble/setup.bash`
  (`root:root`, mode `644`) and `/home/rpp/rpp_ws/install/setup.bash`
  (`rpp:rpp`, mode `664`).
- Live robot driver nodes have included `/left_ufactory_driver`,
  `/right_ufactory_driver`, `/left/crt_gripper_driver`,
  `/right/crt_gripper_driver`, `/left/force_sense_driver`,
  `/right/force_sense_driver`, `/motor_lift_ros_node`, and `/rpp_ros_driver`.
- State candidates observed in the live graph include:
  - `/left_xarm/joint_states` and `/left_xarm/robot_states`;
  - `/right_xarm/joint_states` and `/right_xarm/robot_states`;
  - `/left/joint_states` and `/right/joint_states`;
  - `/left/wrench` and `/right/wrench`;
  - `/motor_lift/joint_states` and `/motor_lift/position`;
  - `/odom`;
  - `/device_state`, `/error_state`, `/motor_state`, and `/turn_motor_state`.
- The graph has `/cmd_vel` with one subscriber. No reviewed chassis action
  server was present in the latest graph.
- Box perception is confirmed at:
  - `/industrial_box/pose_base`, `geometry_msgs/msg/PoseStamped`;
  - `/industrial_box/status`, `std_msgs/msg/String`;
  - `/industrial_box/trigger`, `std_srvs/srv/Trigger`.
- One authorized retry produced a valid `base_link` pose:
  - stamp `2026-09-23T01:50:40.978168037Z`;
  - position `(0.8873708746631405, -0.035999283681093355,
    0.42800562311025236)` meters;
  - quaternion `(-0.4957546935576571, -0.47986237790359615,
    0.5034351180714871, 0.520108127201073)`.
- The trigger service response can time out even when topic inference succeeds.
  Topic pose/status evidence is authoritative; the service receipt is secondary.

Reference artifact directories:

- `inventory-artifacts/20260923T004611Z/`
- `inventory-artifacts/20260923T012141Z/`
- `inventory-artifacts/20260923T013318Z-box-sample/`
- `inventory-artifacts/20260923T013918Z-box-trigger/`
- `inventory-artifacts/20260923T015005Z-box-trigger-retry/`

No pytree numeric configuration is site evidence. Values such as historical arm
points, gripper openings, waist commands, or navigation distances may suggest
fields to inspect, but must not be copied into the live atomic profile without a
fresh Robot One measurement and review.

## Blocking Unknowns

### Runtime identity

The correct `lynrotcontrol` instance is not yet confirmed. Local `ea200.yaml`
and `lyn2.yaml` are candidates, not evidence. The pure v2 adapter now requires
a fresh hashed-profile runtime receipt and the Toolkit requires an exact
perception-interface receipt, but no live composition root yet produces those
receipts. A future read-only component gate must record all of:

- selected instance name and configuration SHA-256;
- `get_model()`, `get_ip()`, `get_side()`, and `get_axes()` for both arms;
- `get_modules()` and `get_bindings()`;
- left/right force source and frame;
- gripper and waist adapter identities;
- process host and ROS environment.

The local `ea200` candidate uses xArm/uf850 arms, external force, CTag
grippers, and `waist01`; the local `lyn2` candidate uses Marvin arms, arm joint
estimate force, CTPM grippers, and Lyn2 waist adapters. The observed xArm and
CRT graph alone does not prove either instance is correct.

### Motion numeric profile

The live atomic profile is currently empty. Before validation it must contain:

- both arms' axis count and reviewed joint limits;
- exact paired left/right joint paths for every named capability;
- per-path joint tolerance, maximum step, force gate, and timeout;
- gripper open/close positions, tolerances, and timeouts;
- waist lift targets, tolerances, limits, and timeouts;
- bounded chassis displacement/turn profiles and stop criteria;
- all action and stop budgets;
- robot-state and perception freshness bounds;
- reviewer, review identifier, and safety evidence references.

The first live profile should begin with current-position/no-displacement or
equivalent explicitly reviewed low-risk component checks. It must not start
with a box grasp.

### Chassis control semantics

No `/lynsense/move_distance` or `/lynsense/nav_to_pose` action server appeared
in the latest live action list. A direct `/cmd_vel` publisher is observed
transport, not an acceptable model-facing capability.

Live chassis integration is therefore blocked until one of these is reviewed:

1. a real chassis action server with typed goal/result/feedback and cancel
   semantics;
2. a new reviewed bounded chassis bridge that internally uses `/cmd_vel` and
   `/odom`, but exposes only profile-named `move_distance` capabilities.

Option 2 must be separately implemented and reviewed. It may never expose a
free velocity, distance, angle, acceleration, topic name, or retry count to the
model.

## Site Profile Worksheet

Every row needs a source artifact or reviewed operator record before it can be
placed in `mode: live`. The final JSON is rejected if any value remains `null`,
unfinite, outside bounds, or unreferenced.

| Field group | Required values | Approved source | Acceptance rule |
| --- | --- | --- | --- |
| Identity | `profile_id`, `version=2`, `mode=live` | operator-authored live profile | stable SHA-256; one file per review |
| Runtime | LynrotControl instance/config SHA, host, ROS domain | read-only runtime gate | graph and API identities agree |
| Arm axes | left/right axes, model, IP | `get_model/get_ip/get_axes` | both sides match profile and each other where required |
| Arm limits | per-axis min/max deg | reviewed API/vendor evidence plus live readback | all path points strictly inside limits |
| Initial pose | left/right current joints | `get_joints` under reviewed idle state | fresh, finite, no arm error |
| Paired paths | named left/right MxN joint arrays | taught/reviewed site procedure | equal path lengths; exact axis count; bounded steps |
| Arm arrival | tolerance deg, timeout/budget | site review + low-speed gate | both tasks and both joint readbacks agree |
| Force baseline | static and held-load force/torque samples | repeated force reads | finite, fresh, sequence advances |
| Force gates | N and Nm thresholds plus hold time | safety review | above measured noise, below reviewed harm limits |
| Grippers | open/close rad, tolerance, timeout | LynrotControl read/move calibration | paired feedback; non-overlapping open/close |
| Waist | current mm, limits/reference, named targets | waist read and reviewed limits | target in range; tolerance and timeout confirmed |
| Chassis state | `/odom` frame, freshness, error inputs | live graph and topic samples | finite pose/twist; stale state blocks motion |
| Chassis move | distance/angle, max speed, max accel, watchdog, stop settle | reviewed bounded bridge config | no free model values; profile hash includes all numbers |
| Chassis navigation | goal names and target poses | action server/map review | disabled until typed action and goals are proven |
| Perception | frame, transform validity, freshness, trigger/result timeout | recorded live pose/status | subscribe before one trigger; topics authoritative |
| Review | status, review id, reviewer, evidence list | independent live review | incomplete review rejects profile |

### Required profile names

The final box strategy may name profiles such as:

- `arms_home`;
- `arms_box_approach`;
- `arms_grasp`;
- `arms_lift_carry`;
- `arms_place_approach`;
- `arms_place_release`;
- `arms_post_place_safe`;
- `waist_home`;
- `waist_box_approach`;
- `waist_lift_carry`;
- `chassis_retreat_small`;
- `chassis_advance_small`.

Exact names are site decisions. Every name must resolve to one complete paired
capability; no generic free-form motion profile is allowed.

### Numeric capture order

1. Review the independent coordinator/Toolkit findings and close all
   Critical/Important issues.
2. Under fresh read-only authorization, identify the LynrotControl instance and
   collect both arms' model, IP, axes, state, joints, limits, TCP selection,
   force metadata, gripper metadata, and waist metadata.
3. Record static force samples on both sides. Do not zero, compensate, enable,
   recover, or command a device during this gate.
4. Teach or select reviewed dual-arm points using the company's approved
   teaching/calibration procedure, not LLM-generated targets.
5. Convert reviewed points into paired paths with small joint steps. Validate
   offline against limits, path length, symmetry requirements, force gates, and
   budgets.
6. Calibrate gripper open/close values only through the reviewed LynrotControl
   gripper adapter and confirm fresh paired feedback.
7. Map waist command units to the selected adapter's `limits/reference`; do not
   mix `/motor_lift/position` scalar units with LynrotControl mm semantics.
8. Design and independently review the bounded chassis bridge before any
   chassis motion.
9. Publish a candidate live profile and compute its SHA-256. Independent review
   must reject any field without source evidence.

## Live Transport Composition

### Module boundary

The current pure modules remain transport-free. Live imports belong only in a
future composition root, for example:

```text
robots/lynsense_real_box/live/composition_root.py
robots/lynsense_real_box/live/perception_transport.py
robots/lynsense_real_box/live/chassis_runtime.py
```

These modules may import `rclpy` and `lynrotcontrol`, but:

- `atomic_adapter.py`, `atomic_toolkit.py`, `atomic_contract.py`, and
  `atomic_profile.py` must not import them;
- no live module may import `lynsense_pytrees`;
- no live module may read arbitrary model numeric arguments;
- no live module may construct a robot before profile and review checks pass.

### Construction order

1. Load and validate `AtomicCapabilityProfile`.
2. Require `mode=live`, `review.status=site_confirmed`, reviewer, and safety
   evidence.
3. Compute and log the profile SHA-256.
4. Initialize the selected LynrotControl instance and confirm both arms,
   grippers, waist, and force bindings.
5. Create the perception transport with its own rclpy context.
6. Create the reviewed chassis runtime with its own rclpy context.
7. Construct `LynrotControlAtomicAdapter` with `live_dispatch_enabled` only
   after all preflight state is fresh and healthy.
8. Construct exactly one `LynsenseAtomicToolkit` and evidence recorder.
9. Expose the Toolkit to `ApiAgentLoop`; never expose the composition root or
   transport objects to the model.

Any construction failure closes only resources already owned by that root. It
must not call recovery, clear errors, enable devices, or retry automatically.

### rclpy context ownership

LynrotControl creates private contexts for ROS-backed peripherals. Perception
and chassis should therefore use explicitly owned private contexts as well:

- no global default-context initialization;
- one executor and close owner per context;
- no context sharing with `lynsense_pytrees`;
- subscriptions installed before perception trigger;
- no publisher is created by the perception transport;
- chassis `/cmd_vel` publisher exists only inside the reviewed bounded chassis
  runtime and only after its state preflight passes;
- all live callbacks copy scalar data into validated snapshots and never call
  Toolkit methods reentrantly.

Close order is perception/chassis transports, LynrotControl robot, adapter
ownership state, then evidence. Close is not recovery and is not an E-stop. A
close error is retained and does not suppress attempts to close the remaining
owners.

### Perception transport

The live perception transport implements the existing `BoxPerceptionClient`
transport protocol:

1. subscribe to `/industrial_box/pose_base` and `/industrial_box/status`;
2. arm callbacks around the trigger request;
3. call `/industrial_box/trigger` exactly once;
4. pump the executor until result deadline;
5. validate `success`, frame, transform, finite pose, quaternion, timestamp,
   and freshness;
6. return the topic result even if the service response timed out.

No automatic retry is allowed. A failed or stale result is returned as a
structured perception failure; site policy must prevent task progression until
a new explicitly requested and reviewed perception capability is allowed.

### Bounded chassis bridge

Until a real chassis action is proven, the only permissible bridge design is a
profile-driven `/odom` to `/cmd_vel` controller:

- input to the bridge is a profile name already resolved by
  `LynrotControlAtomicAdapter`;
- fixed linear and angular maxima, acceleration/deceleration, command rate,
  watchdog timeout, stop-settle timeout, and odometry freshness live in the
  hashed live profile;
- `/cmd_vel` is published only while `/odom` is fresh and the goal remains
  active;
- distance and angle are integrated from a reviewed initial `/odom` sample;
- stale `/odom`, device/error state, timeout, overshoot, or cancel immediately
  publishes zero velocity and enters stop confirmation;
- stop is confirmed only after repeated fresh zero-twist observations and
  bounded remaining motion;
- no retry is automatic;
- navigation goals remain disabled.

This bridge requires a schema extension because the current
`AtomicCapabilityProfile` hashes distance and angle but not chassis speed,
acceleration, command rate, watchdog, or settle criteria. Those values must not
remain hidden implementation constants.

The live schema also needs a runtime-binding section for the LynrotControl
instance/configuration identity, execution host, ROS domain, selected interface
names, and source-artifact references. These values must be hashed with the
numeric profile rather than stored as unreviewed launcher constants.

### Concrete profile v2 extension

The first implementation is `version: 1`; no live file may add these fields to
that version. A future offline validator must implement `version: 2` with the
exact sections below before a composition root is allowed to load it:

```json
{
  "version": 2,
  "runtime_binding": {
    "lynrotcontrol_instance": "selected instance name",
    "lynrotcontrol_config_sha256": "64 lowercase hex characters",
    "execution_host": "recorded host identity",
    "ros_domain_id": 3,
    "source_artifacts": ["artifact-directory-or-reviewed-record"]
  },
  "chassis_runtime": {
    "kind": "bounded_odom_cmd_vel",
    "odom_topic": "/odom",
    "cmd_vel_topic": "/cmd_vel",
    "max_linear_speed_m_s": 0,
    "max_angular_speed_rad_s": 0,
    "max_linear_accel_m_s2": 0,
    "max_angular_accel_rad_s2": 0,
    "command_rate_hz": 0,
    "watchdog_timeout_s": 0,
    "stop_settle_timeout_s": 0,
    "zero_twist_confirm_samples": 0,
    "odom_max_age_s": 0,
    "source_artifacts": []
  },
  "gripper_runtime": {
    "failure_policy": "confirmed_stop_then_operator_review",
    "requires_stop_api": true,
    "stop_timeout_s": 0,
    "zero_motion_confirm_samples": 0,
    "source_artifacts": []
  },
  "carry_guards": {
    "chassis_retreat_small": {
      "arm_profile": "arms_lift_carry",
      "gripper_command": "close",
      "box_control_status": "held",
      "force_monitor": "both_arms",
      "source_artifacts": []
    }
  },
  "perception_binding": {
    "pose_topic": "/industrial_box/pose_base",
    "status_topic": "/industrial_box/status",
    "trigger_service": "/industrial_box/trigger",
    "expected_frame": "base_link",
    "pose_max_age_s": 0,
    "trigger_timeout_s": 0,
    "result_deadline_s": 0,
    "source_artifacts": []
  },
  "subsystem_freshness": {
    "left_arm_state_s": 0,
    "right_arm_state_s": 0,
    "left_arm_force_s": 0,
    "right_arm_force_s": 0,
    "left_gripper_s": 0,
    "right_gripper_s": 0,
    "waist_s": 0,
    "chassis_s": 0
  }
}
```

Every zero above is a placeholder for the shape only, not a valid value. The
validator must require positive finite timing/rate values, nonnegative
speed and acceleration maxima with at least one positive bound where motion is
enabled, exact interface strings, a valid runtime config hash, nonempty source
evidence, and stop-settle/odometry criteria that cannot exceed the shared stop
budget. The chassis bridge must reject `kind: bounded_odom_cmd_vel` unless all
of its fields are present and hashed. A typed action-server mode may omit only
the `/cmd_vel` fields after its goal/result/feedback/cancel contract is
independently reviewed.

A chassis profile used while a box may be held must have an entry in
`carry_guards` with the exact field names shown above. The referenced arm
profile, closed gripper command, `held` box-control state, and `both_arms`
force monitoring must be current immediately before and throughout chassis
motion. The validator rejects a carrying chassis profile without a complete
guard and does not let a non-carrying profile inherit one. The offline adapter
also enforces the guard immediately before dispatch, after dispatch, and during
every active-task poll; requires an advancing box-control sequence; and monitors
both force sources while the reviewed chassis task is active.

No live gripper dispatch is allowed from a protocol that exposes only `move`
and `read`. The selected adapter must provide a reviewed stop/cancel API, or
live gripper capability remains blocked. A gripper timeout, disagreement, or
invalid feedback must confirm physical non-motion and enter operator review;
issuing an opposite open/close command is not an acceptable implicit stop.

Numeric intake must remain separate from executable profiles. Capture records
may contain missing values and provenance, but they are never passed to
`AtomicCapabilityProfile`; only a complete reviewed file can validate.

## Low-Speed Component Gate Order

Each gate needs fresh authorization, a physical operator, E-stop ownership, and
a written stop condition. Proposed order:

1. live read-only state identity and freshness;
2. perception trigger/topic result;
3. waist current-position or minimal reviewed in-range target;
4. paired gripper open/close at reviewed clearance;
5. paired arms current-position or minimal taught joint delta;
6. bounded chassis zero/small displacement with watchdog and stop confirmation;
7. paired arm plus waist composition without box;
8. paired arms plus gripper composition without box;
9. carry an approved surrogate payload, not the production box;
10. only after all prior gates pass, supervised box approach/grasp trial.

The production box must not be grasped in the first live motion gate. A failed
or unconfirmed stop at any level returns to `operator_review`; it is not
cleared automatically.

## Acceptance

The transport design is complete only when:

1. independent review reports no open Critical or Important finding;
2. every worksheet field has source evidence;
3. the candidate live profile validates and its SHA-256 is recorded;
4. a dry-run composition root proves zero live import/dispatch in the default
   backend;
5. live-only import tests use fake `rclpy` and fake `lynrotcontrol`;
6. independent review approves the bounded chassis bridge;
7. the operator has separately authorized each low-speed component gate.

Until then, `LynsenseAtomicToolkit` remains offline-only and live construction
must continue to fail closed.
