# Robot One read-only inventory runbook

## Authorization gate

> **Do not run this inventory without new explicit user authorization.**

Task 8 does not execute this procedure. This document only prepares a separately
authorized future run against Robot One. The authorization must name Robot One,
this runbook, the read-only scope, and the operator. A past review of this file
is not authorization to connect.

## Stop conditions

**STOP before connecting** if:

- fresh authorization is absent or expired;
- the workspace path, ownership, mode, or review decision is unexpected;
- the operator cannot resolve an interface ambiguity from recorded evidence.

Missing coordination or perception evidence stops a future live-adapter plan;
it does not by itself stop this read-only inventory. Record the absence and
continue inventorying other interfaces. On any stop, keep the evidence already
written, do not source the workspace, do not issue further ROS commands, and
return the decision to the authorizing user. Do not substitute a guessed
alternate interface.

## Pre-flight, before SSH

1. Verify the current local Asia/Shanghai time and record it in the run notes:

   ```bash
   TZ=Asia/Shanghai date -Is
   ```

2. Confirm that the authorization is current and that the Robot One SSH session
   will use only the read-only allowlist below.

## Selected ROS domain

Selected ROS domain: `3`.

Apply it only as an inline `ROS_DOMAIN_ID=3` environment assignment to each
ROS observation command. Record both the workstation-selected value and the
command output. In prose, the operator must record that they do not modify
remote shell configuration, launch files, ROS parameters, or persistent
environment files.

The selected domain must be named by the fresh authorization. If the user
selects a different domain in a future authorization, replace every `3` in the
operational command sequence with that reviewed value.

## Source-reference interpretation

The local company source trees are references, not substitutes for live graph
evidence:

- `/home/huhu/work/pytree/lynsense_pytrees/trees/lynsense_movebox_tree.xml`
  shows a py_trees synchronized parallel composition of separate left- and
  right-arm controllers. This is independent synchronized left/right control,
  not an atomic composite dual-arm grasp/place controller.
- `/home/huhu/work/pytree/visual-perception/src/industrial_box_perception`
  defines `/industrial_box/pose`, `/industrial_box/pose_base`, and
  `/industrial_box/status` as box-perception candidates.
- `/home/huhu/work/pytree/visual-perception/src/object_approach` defines
  arm-namespaced candidates such as `/object_approach_right/object_pose_base`
  and `/object_approach_right/status`.

These are source-reference candidates, not proof of live presence. Record one
only when the live node, topic, service, or action list actually contains it.

## Workstation-only artifact preparation

Before connecting, prepare the artifact directory on the operator workstation.
Use exactly these workstation-only choices:

```bash
utc_stamp="$(date -u +%Y%m%dT%H%M%SZ)"
artifact_dir="inventory-artifacts/${utc_stamp}"
mkdir -p -- "$artifact_dir"
find "$artifact_dir" -mindepth 1 -print -quit
```

`find` must print nothing. If it prints a path, stop and generate a new UTC
directory instead of reusing or cleaning it. These commands never target Robot One
and must not be used inside the Robot One SSH session. After that session ends,
use only `sha256sum -- "$artifact_dir/<relative-artifact-path>"` on the
workstation to compute manifest hashes.

## Robot One SSH read-only allowlist

Inside the Robot One SSH session, only these observation-only command families
are allowed, with arguments bounded to this runbook:

- `date -Is`
- `readlink -f /home/rpp/rpp_ws`
- `stat -c '%U:%G %a %n' <reviewed file>`
- `source /opt/ros/humble/setup.bash`
- `source /home/rpp/rpp_ws/install/setup.bash`
- `printenv ROS_DOMAIN_ID ROS_LOCALHOST_ONLY`
- `timeout 10s ros2 node list`
- `timeout 10s ros2 topic list -v`
- `timeout 10s ros2 service list -v`
- `timeout 10s ros2 action list -v`
- `timeout 10s ros2 service list`
- `timeout 10s ros2 action list`
- `timeout 10s ros2 service type "$service"`
- `timeout 10s ros2 action info "$action"`
- `timeout 10s ros2 topic info -v "$topic"`
- `timeout 10s ros2 topic echo --once "$topic"`
- `timeout 10s ros2 interface show "$type"`

Every ROS observation command is wrapped in `timeout 10s`. If a timeout occurs,
record the command, exit status, and captured output, then mark the interface
`present: false` unless the output itself proves presence. Do not retry with a
mutating command.

The installed Robot One CLI has already reported `unrecognized arguments: -v`
for service and action listing. If that happens again, record the failure and
run the corresponding plain list command once. Do not retry in a loop. Resolve
a relevant service or action name only from successful list output, then use
the bounded `service type` or `action info` command above.

## Robot One SSH forbidden actions

The following are prohibited in the Robot One SSH session:

`ros2 topic pub`, `ros2 service call`, `ros2 action send_goal`, `ros2 run`,
`ros2 launch`, `ros2 param set`, `sudo`, `chmod`, `chown`, `rm -`, `mv`,
`cp`, `touch`, `mkdir`, `tee`, `systemctl start`, `systemctl stop`,
`systemctl restart`, and any other file, permission, service, controller,
trajectory, enable, disable, reset, or configuration change.

The inventory must not enable or command any actuator, alter ROS parameters,
trigger perception, create files on the robot, or modify ownership or modes.
Do not run `lynrotcontrol.initialize(instance)` as an identity probe: the local
reference implementation initializes both arms and starts a client process.
The workstation-only directory and hashing commands above are not Robot One
commands and cannot be used to change, copy data to, or collect data from the
robot.

## Shared workspace review and graph observations

After connecting with `ssh robot1` under the fresh authorization, run only:

```bash
date -Is
readlink -f /home/rpp/rpp_ws
stat -c '%U:%G %a %n' /opt/ros/humble/setup.bash /home/rpp/rpp_ws/install/setup.bash
# Review decision: reviewed expected owner and mode; otherwise STOP.
source /opt/ros/humble/setup.bash
source /home/rpp/rpp_ws/install/setup.bash
printenv ROS_DOMAIN_ID ROS_LOCALHOST_ONLY
ROS_DOMAIN_ID=3 printenv ROS_DOMAIN_ID ROS_LOCALHOST_ONLY
ROS_DOMAIN_ID=3 timeout 10s ros2 node list
ROS_DOMAIN_ID=3 timeout 10s ros2 topic list -v
ROS_DOMAIN_ID=3 timeout 10s ros2 service list -v
ROS_DOMAIN_ID=3 timeout 10s ros2 action list -v
# If service/action -v reports "unrecognized arguments: -v", run each once:
ROS_DOMAIN_ID=3 timeout 10s ros2 service list
ROS_DOMAIN_ID=3 timeout 10s ros2 action list
```

Record the command output, timestamp, both selected environment flags, interface
names/types, and operator notes. Store only those fields and the artifact record
metadata; do not store a complete environment dump or credentials.

The resolved workspace path, file ownership, and mode come from `readlink` and
`stat`. The expected owner is
`rpp:rpp` unless the authorizing user records a different reviewed owner for
this run. The operator records the review decision only after the resolved path,
ownership, and mode match that reviewed expectation. If either setup file is
unreviewed or its ownership is unexpected, stop; do not source it and do not
change permissions.

The `readlink` and `stat` evidence must precede both `source` commands and be
included in the artifacts. If either checked file is absent, record the command
failure as evidence and stop.

## Per-subsystem topic/type observations

For each subsystem, use the topic and type actually observed in the shared graph
output. Replace each `<OBSERVED ...>` value only from evidence. Repeat the
complete four-line observation for every subsystem; do not infer the right-arm
type or topic onto another subsystem. If an interface is absent, write a record
with `present: false` and evidence (for example, the exact absent topic result
or graph-list excerpt), rather than substituting a guessed alternate.
Continue recording the remaining subsystems after an absent interface.
For each received sample, record its timestamp and sequence only if those
fields actually exist in the observed message; otherwise record them as
unavailable. An echo result is a sample, not proof of continuous freshness.

### Chassis

```bash
topic=<OBSERVED CHASSIS STATE TOPIC>
ROS_DOMAIN_ID=3 timeout 10s ros2 topic info -v "$topic"
ROS_DOMAIN_ID=3 timeout 10s ros2 topic echo --once "$topic"
type=<OBSERVED CHASSIS STATE TYPE>
ROS_DOMAIN_ID=3 timeout 10s ros2 interface show "$type"
```

### Localization

```bash
topic=<OBSERVED LOCALIZATION STATE TOPIC>
ROS_DOMAIN_ID=3 timeout 10s ros2 topic info -v "$topic"
ROS_DOMAIN_ID=3 timeout 10s ros2 topic echo --once "$topic"
type=<OBSERVED LOCALIZATION STATE TYPE>
ROS_DOMAIN_ID=3 timeout 10s ros2 interface show "$type"
```

### Waist

```bash
topic=<OBSERVED WAIST STATE TOPIC>
ROS_DOMAIN_ID=3 timeout 10s ros2 topic info -v "$topic"
ROS_DOMAIN_ID=3 timeout 10s ros2 topic echo --once "$topic"
type=<OBSERVED WAIST STATE TYPE>
ROS_DOMAIN_ID=3 timeout 10s ros2 interface show "$type"
```

### Left arm

```bash
topic=<OBSERVED LEFT-ARM STATE TOPIC>
ROS_DOMAIN_ID=3 timeout 10s ros2 topic info -v "$topic"
ROS_DOMAIN_ID=3 timeout 10s ros2 topic echo --once "$topic"
type=<OBSERVED LEFT-ARM STATE TYPE>
ROS_DOMAIN_ID=3 timeout 10s ros2 interface show "$type"
```

### Right arm

The right-arm example in the task brief is not a default for other subsystems.

```bash
topic=<OBSERVED RIGHT-ARM STATE TOPIC>
ROS_DOMAIN_ID=3 timeout 10s ros2 topic info -v "$topic"
ROS_DOMAIN_ID=3 timeout 10s ros2 topic echo --once "$topic"
type=<OBSERVED RIGHT-ARM STATE TYPE>
ROS_DOMAIN_ID=3 timeout 10s ros2 interface show "$type"
```

The brief's known candidate values are `/right_xarm/joint_states` and
`sensor_msgs/msg/JointState`; confirm both from the live graph before recording
them. If they do not match, record the observed values or absence evidence
separately and do not apply them to another arm.

### Dual-arm controller

```bash
topic=<OBSERVED DUAL-ARM CONTROLLER FEEDBACK TOPIC>
ROS_DOMAIN_ID=3 timeout 10s ros2 topic info -v "$topic"
ROS_DOMAIN_ID=3 timeout 10s ros2 topic echo --once "$topic"
type=<OBSERVED DUAL-ARM CONTROLLER FEEDBACK TYPE>
ROS_DOMAIN_ID=3 timeout 10s ros2 interface show "$type"
```

Also inventory relevant services and actions with:

```bash
service=<OBSERVED DUAL-ARM RELEVANT SERVICE>
ROS_DOMAIN_ID=3 timeout 10s ros2 service type "$service"
action=<OBSERVED DUAL-ARM RELEVANT ACTION>
ROS_DOMAIN_ID=3 timeout 10s ros2 action info "$action"
```

Record exactly one coordination decision:

- `atomic_composite`: one live interface accepts a coordinated dual-arm
  grasp/place goal and returns coordinated feedback.
- `independent_synchronized`: separate left/right interfaces are present and a
  reviewed source/config explicitly defines synchronized composition and shared
  failure handling.
- `unresolved`: only separate left/right interfaces, or ambiguous evidence.

Do not infer coordination safety from the existence of left and right drivers alone.
Missing atomic coordination does not stop the read-only inventory, but the later
live adapter plan must stop for a fresh approval unless it presents a reviewed
coordination implementation and safety evidence.

### Left force feedback

```bash
topic=<OBSERVED LEFT-FORCE FEEDBACK TOPIC>
ROS_DOMAIN_ID=3 timeout 10s ros2 topic info -v "$topic"
ROS_DOMAIN_ID=3 timeout 10s ros2 topic echo --once "$topic"
type=<OBSERVED LEFT-FORCE FEEDBACK TYPE>
ROS_DOMAIN_ID=3 timeout 10s ros2 interface show "$type"
```

### Right force feedback

```bash
topic=<OBSERVED RIGHT-FORCE FEEDBACK TOPIC>
ROS_DOMAIN_ID=3 timeout 10s ros2 topic info -v "$topic"
ROS_DOMAIN_ID=3 timeout 10s ros2 topic echo --once "$topic"
type=<OBSERVED RIGHT-FORCE FEEDBACK TYPE>
ROS_DOMAIN_ID=3 timeout 10s ros2 interface show "$type"
```

The historical `/left/wrench` and `/right/wrench` names are only candidates;
use them only if they appear in the fresh graph. Note the message frame and
units where present; neither a topic name nor one sample establishes the
LynrotControl force binding or its calibration.

### Left gripper

```bash
topic=<OBSERVED LEFT-GRIPPER FEEDBACK TOPIC>
ROS_DOMAIN_ID=3 timeout 10s ros2 topic info -v "$topic"
ROS_DOMAIN_ID=3 timeout 10s ros2 topic echo --once "$topic"
type=<OBSERVED LEFT-GRIPPER FEEDBACK TYPE>
ROS_DOMAIN_ID=3 timeout 10s ros2 interface show "$type"
```

### Right gripper

```bash
topic=<OBSERVED RIGHT-GRIPPER FEEDBACK TOPIC>
ROS_DOMAIN_ID=3 timeout 10s ros2 topic info -v "$topic"
ROS_DOMAIN_ID=3 timeout 10s ros2 topic echo --once "$topic"
type=<OBSERVED RIGHT-GRIPPER FEEDBACK TYPE>
ROS_DOMAIN_ID=3 timeout 10s ros2 interface show "$type"
```

### Box perception

Do not invoke a perception trigger. Observe only an already publishing pose
interface and any status interface that is independently present.

```bash
topic=<OBSERVED BOX PERCEPTION POSE TOPIC>
ROS_DOMAIN_ID=3 timeout 10s ros2 topic info -v "$topic"
ROS_DOMAIN_ID=3 timeout 10s ros2 topic echo --once "$topic"
type=<OBSERVED BOX PERCEPTION POSE TYPE>
ROS_DOMAIN_ID=3 timeout 10s ros2 interface show "$type"
```

Observe the independently observed status interface with its own four lines:

```bash
topic=<OBSERVED BOX PERCEPTION STATUS TOPIC>
ROS_DOMAIN_ID=3 timeout 10s ros2 topic info -v "$topic"
ROS_DOMAIN_ID=3 timeout 10s ros2 topic echo --once "$topic"
type=<OBSERVED BOX PERCEPTION STATUS TYPE>
ROS_DOMAIN_ID=3 timeout 10s ros2 interface show "$type"
```

If pose, status, or both are absent, record each result with `present: false`
and its evidence. Do not trigger or configure perception. Do not substitute a
guessed alternate interface.

## Artifact directory and manifest

Write raw output files under `inventory-artifacts/<UTC timestamp>/`. The
directory name is UTC RFC 3339 with colons replaced for filesystem safety. Add
one raw output file per command and one `manifest.json`. Compute SHA-256 only
after command capture is complete.

```json
{
  "manifest_version": 1,
  "created_at_utc": "1970-01-01T00:00:00Z",
  "robot": "robot1",
  "runbook_sha256": "<runbook SHA-256>",
  "authorization_reference": "<user-approved authorization identifier>",
  "operator_notes": "<operator notes>",
  "artifacts": [
    {
      "path": "commands/0001-date.txt",
      "sha256": "0000000000000000000000000000000000000000000000000000000000000000",
      "bytes": 0,
      "command": "date -Is",
      "captured_at_utc": "1970-01-01T00:00:00Z"
    }
  ],
  "sha256_algorithm": "sha256"
}
```

Store only command output, timestamps, selected environment flags, interface names/types, and operator notes.
Artifact contents are limited to that set plus the required manifest metadata.
If an interface is absent, store an interface record with `present: false`, the
command or graph evidence, and no alternate guess.

## Completion

After the authorized run, stop all observation, verify every manifest hash, and
give the user the artifact directory plus explicit per-subsystem present/absent
results. Do not use the inventory output to justify a new execution without a
further authorization.
