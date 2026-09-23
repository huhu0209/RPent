# Robot One read-only identity and stop-interface inventory

- Operator: huhu (workstation); SSH account: zxh.
- Local preflight: 2026-09-23T18:04:36+08:00; remote time: 2026-09-23T18:06:51+08:00.
- Authorization: user approved Robot One, ROS_DOMAIN_ID=3, read-only
  list/type/info/single echo under runbook SHA-256
  `9c3cbb4c001188a0e4f6487925ab03def1653618a1451efbe01ce42cd2e4e304`.
- Preflight matched: /home/rpp/rpp_ws; /opt/ros/humble/setup.bash root:root
  644; /home/rpp/rpp_ws/install/setup.bash rpp:rpp 664.
- Default ROS_DOMAIN_ID was unset; ROS_LOCALHOST_ONLY=0. All ROS commands
  used inline ROS_DOMAIN_ID=3. service/action list -v returned exit 2, so
  each was retried once as plain list per the runbook.

## Read-only results

- Left and right xArm joint topics: present, each with one driver publisher.
  One JointState sample from each was received; the left reports six joints
  and a timestamp. This is not evidence of continuous freshness.
- Left and right force topics: /left/wrench and /right/wrench present,
  WrenchStamped, each with a driver publisher and one stamped sample.
  The left sample frame is left_force_sensor_link. No zeroing, calibration,
  or force-gate threshold was verified.
- Left and right gripper feedback: /left/joint_states and
  /right/joint_states present, each published by crt_gripper_driver with
  one stamped sample. No gripper stop/cancel behavior was verified.
- Waist /motor_lift/joint_states, chassis /odom, and /error_state: present
  with one sample each. Single samples do not establish sustained health.
- Left xArm /left_xarm/robot_states: present, one sample with state=2,
  mode=1, cmdnum=0, err=0, warn=0. The installed RobotMsg definition labels
  state 2 SLEEPING and mode 1 SERVOJ. Do not infer motion readiness from this
  sample or assume it matches a reviewed motion profile.
- No left-arm motion action was present in the action list. The observed
  /left_xarm_gripper/gripper_action has one server and zero clients.
- /left_xarm/set_state is xarm_msgs/srv/SetInt16 and
  /left_xarm/motion_enable is xarm_msgs/srv/SetInt16ById. Their presence
  does not establish a safe, bounded cancellation pathway for this runtime.
  Source review finds lynrotcontrol Arm.motion.cancel(), but selecting and
  initializing the actual LynrotControl instance was outside this read-only
  authorization. No cancel or set_state service was called.

## Decision

Read-only graph and one-shot feedback are available. Actual LynrotControl
instance/configuration and runtime identity, fresh repeated feedback, arm
motion-mode compatibility, and an independently usable, tested software
cancel/stop pathway remain unresolved. Do not promote these observations to
a live profile or authorize motion on their basis.

No service call, action goal, topic publish, perception trigger, runtime
initialization, controller change, stop/cancel request, or motion occurred.
An earlier UTC-stamped artifact directory (20260923T100542Z) was unused
because it acquired a subdirectory before the runbook's empty-directory
check; it was not cleaned or used as evidence.
