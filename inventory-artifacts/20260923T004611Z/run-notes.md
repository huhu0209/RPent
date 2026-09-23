# Robot One read-only inventory run notes, ROS domain 3

- Local Asia/Shanghai preflight time: `2026-09-23T08:46:11+08:00`.
- Remote Asia/Shanghai time at connection: `2026-09-23T08:46:22+08:00`.
- Authorization reference: user reported that the robot driver had been started
  and explicitly directed the read-only retry to use `ROS_DOMAIN_ID=3`.
- Operator: Codex agent on the `huhu` workstation, acting only within the
  read-only authorization and the domain-3 environment selection.
- Artifact directory created at UTC `2026-09-23T00:46:11Z`.
- Runbook SHA-256:
  `10a1788be500026e58500c380cdaa4faa112f57295137d365effc21e741e5c91`.

Workspace review decision:

- `/home/rpp/rpp_ws` resolved to `/home/rpp/rpp_ws`.
- `/opt/ros/humble/setup.bash`: `root:root`, mode `644`.
- `/home/rpp/rpp_ws/install/setup.bash`: `rpp:rpp`, mode `664`.
- Both matched the previously reviewed state. No ownership or permission change
  was made.

Selected environment for every graph command:

- `ROS_DOMAIN_ID=3`
- `ROS_LOCALHOST_ONLY=0`

The domain was supplied as an inline environment assignment for each observation
command. No remote shell configuration or file was changed.

Graph observations:

- `ros2 node list` succeeded and returned 10 nodes:
  `/joint_state_publisher`, `/left/crt_gripper_driver`,
  `/left/force_sense_driver`, `/left_ufactory_driver`,
  `/motor_lift_ros_node`, `/right/crt_gripper_driver`,
  `/right/force_sense_driver`, `/right_ufactory_driver`,
  `/robot_state_publisher`, and `/rpp_ros_driver`.
- `ros2 topic list -v` succeeded. It exposed chassis/localization, waist,
  left-arm, right-arm, and left/right gripper feedback candidates, but no
  composite dual-arm grasp/place controller feedback interface and no box
  perception pose/status interface.
- `ros2 service list -v` and `ros2 action list -v` failed with status 2 because
  the installed ROS CLI reported `unrecognized arguments: -v`.

Stop decision:

- The successful node/topic graph did not contain a reviewed dual-arm
  grasp/place controller or safety evidence.
- Verbose service/action inventory was unavailable, and the runbook forbids
  substituting an unlisted fallback command.
- Therefore the inventory stopped before per-subsystem `topic info`,
  `topic echo`, or `interface show` observations.
- No perception trigger, service call, action goal, parameter change, file
  change, permission change, or actuator command was attempted.
