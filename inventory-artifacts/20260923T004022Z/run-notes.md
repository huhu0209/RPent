# Robot One read-only inventory run notes

- Local Asia/Shanghai preflight time: `2026-09-23T08:40:02+08:00`.
- Remote Asia/Shanghai time at connection: `2026-09-23T08:40:36+08:00`.
- Authorization reference: user replied `再试一次 现在应该可以脸上来了`,
  authorizing a retry of this Robot One read-only inventory run.
- Operator: Codex agent on the `huhu` workstation, acting only within the
  read-only authorization.
- Artifact directory created at UTC `2026-09-23T00:40:22Z`.
- Runbook SHA-256:
  `10a1788be500026e58500c380cdaa4faa112f57295137d365effc21e741e5c91`.

Workspace review decision:

- `/home/rpp/rpp_ws` resolved to `/home/rpp/rpp_ws`.
- `/opt/ros/humble/setup.bash`: `root:root`, mode `644`, reviewed as the
  system ROS installation file.
- `/home/rpp/rpp_ws/install/setup.bash`: `rpp:rpp`, mode `664`, reviewed as the
  user workspace setup file for this run.
- No ownership or permission change was made.

Selected environment after sourcing both reviewed setup files:

- `ROS_DOMAIN_ID=0`
- `ROS_LOCALHOST_ONLY` was unset; `printenv` consequently returned status 1.

Graph observations:

- `timeout 10s ros2 node list` succeeded with no nodes.
- `timeout 10s ros2 topic list -v` succeeded and listed only:
  `/parameter_events` and `/rosout`.
- `timeout 10s ros2 service list -v` failed with status 2 because this ROS CLI
  reported `unrecognized arguments: -v`.
- `timeout 10s ros2 action list -v` failed with the same status and reason.

No robot subsystem state topic, controller feedback topic, gripper feedback
topic, or box-perception interface appeared in the successful topic graph. The
runbook's reviewed dual-arm grasp/place controller and safety evidence were
therefore absent before subsystem observation. The inventory stopped at that
stop condition. No subsystem `topic info`, `topic echo`, or `interface show`
command was attempted; no perception trigger, service call, action goal,
parameter change, file change, permission change, or actuator command was
attempted.
