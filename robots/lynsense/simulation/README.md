# Lynsense Webots Smoke

This directory contains the containerized, simulation-only Lynsense Webots
smoke gates. They exercise the reconstructed ROS action interfaces, the Webots
extern controller, and the deterministic behavior-tree script. A passing
simulation **must not be interpreted as authorization to move the real robot**.

## Isolation

The default Compose service uses `network_mode: none`, `ROS_DOMAIN_ID=42`, and
`ROS_LOCALHOST_ONLY=1`; it publishes no ports. Its bind mounts are limited to:

- the simulation subtree at `/workspace/simulation`, read-only;
- the sibling `lynsense_pytrees` checkout at
  `/workspace/ws/src/lynsense_pytrees`, read-only;
- `RPent/.artifacts/lynsense-webots` at `/workspace/artifacts`, writable.

Build, install, and log directories live only in the ephemeral container
filesystem. The Compose context is the `docker` directory and `.dockerignore`
admits only `Dockerfile`; repository root, `.env.lynsense`, Git metadata, SSH
configuration, home directories, and robot paths are not included.

The image is pinned by digest to
`cyberbotics/webots@sha256:f0023e30daf38b172e4e6ad24ed345909bcd9551df34d63d824e121a7cebf099`
and installs ROS 2 Humble from the Ubuntu Jammy ROS repository. It runs as
UID/GID 1000 user `lynsense`. The ephemeral workspace copy and
`/workspace/artifacts` are writable for simulation work; the nested read-only
`lynsense_pytrees` mount remains non-writable.

## Build And Gates

From `robots/lynsense/simulation/docker`:

```bash
docker compose build
docker compose run --rm lynsense-webots-smoke --phase build
docker compose run --rm lynsense-webots-smoke --phase interface
docker compose run --rm lynsense-webots-smoke --phase smoke
docker compose run --rm lynsense-webots-smoke --phase blocked
docker compose run --rm lynsense-webots-smoke --phase match
docker compose run --rm lynsense-webots-smoke --phase box
docker compose run --rm lynsense-webots-smoke --phase all
docker compose run --rm lynsense-webots-smoke --phase box-all
```

`bootstrap.py --phase build` builds the three ROS packages and exits. It sources
ROS/overlay setup separately in every Bash operation, copies only
`lynsense_utils_compat` as `lynsense_utils` and `lynsense_webots_sim` into the
ephemeral workspace, never writes to the read-only `lynsense_pytrees` checkout,
copies the RPent-owned `lynsense_match_min_tree.xml` into the ephemeral
`lynsense_pytrees` share after the build, and verifies that the installed console script at
`/workspace/ws/install/lynsense_webots_sim/bin/lynsense_webots_controller` is
present and executable before a run. Only after the newly built overlay is
sourced does bootstrap delegate to `lynsense_run_smoke`. Every
`compose run --rm` receives a fresh ephemeral filesystem, so `interface`,
`smoke`, `blocked`, `match`, and `all` also rebuild that overlay before delegating to
`lynsense_run_smoke --phase` with the exact requested phase.

The post-build orchestrator command is `lynsense_run_smoke --phase
interface|smoke|blocked|match|box|all|box-all`. Bootstrap’s `all` phase
rebuilds once and delegates once; the orchestrator then runs interface, smoke,
blocked, and match sequentially in that overlay and writes `summary.json`.
`box-all` keeps that original four-phase sequence unchanged and appends the
single-box task.

The interface gate waits for both action endpoints, checks unknown and invalid
goals, checks concurrent-goal rejection, and uses `lynsense_action_probe --mode
interface`. Because that boundary goal leaves the unconstrained interface
runtime in terminal lockout, the orchestrator restarts Webots and the controller
before `lynsense_action_probe --mode cancel`. The cancel probe uses a long
`MoveDistance(angle=180, distance=0)` goal, verifies `STATUS_CANCELED`,
`success=false`, retained terminal remaining values, zero post-cancel wheel
rates, and no further progress. Before starting the tree, smoke, blocked, and match also
poll `ros2 action list` until both `/lynsense/nav_to_pose` and
`/lynsense/move_distance` are discoverable; this compensates for the tree's
one-second Action Client wait without modifying the read-only
`lynsense_pytrees` checkout. Smoke, blocked, and match invoke the installed
`install/lynsense_pytrees/lib/lynsense_pytrees/lynsense_pytrees_node.py`
executable directly while preserving its ROS arguments and XML selection. The
smoke gate requires the exact accepted/result sequence, final pose `(0.5, 0, 0
degrees)`, and zero terminal wheel rates. The blocked gate requires the expected
failed navigation outcome and zero terminal wheel rates. The orchestrator
normalizes that expected blocked failure to a passed gate and exits `0`; any
unexpected action sequence, non-stalled failure, or nonzero terminal wheel rate
still exits nonzero.

The match gate is the first navigation-only excerpt from the read-only
`lynsense_pytrees` branch `origin/WHRG2026`, commit `a70e301`, and
`trees/lynsense_plan_move2box_tree.xml`. It executes `NavToPose(搬箱子1)`,
`MoveDistance(distance=-0.6, angle=0)`,
`MoveDistance(distance=0, angle=90)`, then `NavToPose(放箱子1_1)`. The goal
poses are copied from `config/move_points.yaml` on that branch and are not
translated or scaled; their quaternions are converted to yaw in the map frame.
The match world keeps the verified robot model but enlarges the floor from
`8 x 5` to `8 x 6` so the second goal remains inside the plane. It contains no
competition obstacle geometry. This validates only the navigation action
sequence and geometry propagation, not collision avoidance, manipulation,
localization, or the complete competition task.

## Single-Box End-To-End Gate

Run the simulation-only box gate with:

```bash
docker compose run --rm lynsense-webots-smoke --phase box
docker compose run --rm lynsense-webots-smoke --phase box-all
```

The `box` world uses the company robot's configured wheel geometry, motion
limits, primitive upper body, competition box dimensions, and the copied
box-navigation goals. It executes an exact 15-action serial sequence through an
RPent-owned `py_trees` runner: approach, waist, gripper, grasp pose, pick,
backoff, turn, placement prep, navigation, place, and retreat. This is a
minimum task implementation, not the original competition tree and not a
perception or planner replacement.

The main task uses a validated hybrid attachment. Pick is accepted only when
the box, arms, and closed grippers are aligned; after attachment, relative
translation and yaw are monitored. Place is accepted only after a successful
pick has recorded that attachment in the same manipulation runtime and while
the box remains attached. It opens the grippers, releases only at the
configured target, rejects target drift or bad attitude during settling, and
requires at least 0.5 seconds of post-release evidence. The final box must be
within 0.05 m and about 10 degrees of
`(2.810559, -2.734170, yaw 0.006126 rad)` in the map frame. The final robot
must stop at the configured retreat pose with all wheel, waist, arm, and
gripper rates zero. Evidence is in `events-box.jsonl` and `summary.json`.

The hybrid attachment does not claim that the main-world grippers generate the
forces needed to carry the box. The separate `lynsense_grasp_physics.wbt`
probe verifies sustained side contact, lift, and release-to-fall with Webots
contacts. That world and its pedestal/finger proof parameters are intentionally
separate from the end-to-end gate.

## RPent Single-Box Plan Gate

The RPent integration is plan-level and uses a separate
`lynsense_simulation` backend. The real `lynsense` backend remains read-only;
no motion tools are added to it. The simulation model surface is exactly:

```text
read_simulation_task
submit_simulation_plan
finish
```

From the RPent repository root, generate a plan with the native API Planner:

```bash
.venv/bin/rpent \
  --robot lynsense_simulation \
  --planner api \
  --memory-profile local \
  --output-dir .artifacts/lynsense-webots/rpent-plan-run \
  --model "$LYNSENSE_MODEL_ID"
```

The toolkit reads the static isolated-task state, goal, required action order,
and permitted action values. It atomically writes `plan.json` only after the
complete document passes the strict 15-action contract. Malformed metadata,
extra fields, non-finite numbers, partial plans, duplicates, and out-of-order
actions are rejected before Webots starts. A Toolkit session accepts only one
successful plan submission, and `finish(status="success")` is rejected until
that submission has been written.

Execute the accepted plan from `robots/lynsense/simulation/docker`:

```bash
ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 docker compose run --rm \
  lynsense-webots-smoke \
  --phase rpent-box \
  --plan /workspace/artifacts/rpent-plan-run/plan.json
```

`rpent-box` uses the same box world, controller, manipulation preconditions,
event validator, final-pose checks, and 180-second budget as `box`, but writes
`events-rpent-box.jsonl` and its own process logs. The original `all` and
`box-all` sequences are unchanged. The runner validates the plan before ROS
startup and executes actions serially; interruption or an action exception
waits for a pending goal request and bounded cancellation before client and
node teardown. Offline `FunctionModel` tests exercise the
real API Planner, Toolkit, and CLI paths without contacting an external model;
they do not prove autonomous perception, collision avoidance, closed-loop
control, real-robot readiness, or authorization to move robot one.

Each invocation gets a unique `run_id`, archives an existing same-phase JSONL
file under `archive/`, and creates a fresh empty event file before any process
starts. Evidence is split into `events-interface.jsonl`,
`events-smoke.jsonl`, `events-blocked.jsonl`, and `events-match.jsonl`, plus build, Webots,
controller, tree, and probe logs. `summary.json` records the gate outcomes.
`summary.json` is the unified gate summary and records phase, run ID, outcome,
and interface isolation evidence. Detailed action goals, feedback, results,
poses, wheel rates, and failure reasons are in the phase JSONL files. Build
evidence in `build-summary.json` records the ROS environment and the
pre/post-source `AMENT_PREFIX_PATH` and `ROS_PACKAGE_PATH`; the output image
digest recorded by the final delivery review is
`sha256:cf91e1d0c108c2c585033fcab8df56db7d1338738a5f4c192b161cd11fc91927`.
Evidence in `interface-summary.json` records Domain 42, localhost-only mode,
the ephemeral overlay prefix, `ros2 pkg prefix lynsense_utils`, import evidence
for `rclpy`, `py_trees`, `py_trees_ros`, and `yaml`, and rejects `/home/rpp`,
`rpp_ws`, or other robot workspace paths.
Within the interface phase, the second controller lifecycle appends to the same
run ID's JSONL and process logs, preserving both the rejection/concurrency
checks and the cancellation check.

## Chassis Stability Regression

The wheel cylinders and their collision shapes share a 90-degree X rotation
inside each wheel Solid, aligning the cylinder axis with the Y-axis hinge.
The Solid frame and its Y-axis inertia remain unchanged. Both wheel/floor and
caster/floor contacts explicitly use `bounce 0` and `softCFM 0.00001` to avoid
excessive settling and rocking with the simplified 120 kg body. These are
simulation baseline parameters, not calibrated real-robot material properties.

From `robots/lynsense/simulation/docker`, run the isolated physics regression
with a fresh output directory:

```bash
docker compose run --rm --no-deps --entrypoint python3 lynsense-webots-smoke \
  /workspace/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/scripts/run_stability.py \
  --world /workspace/simulation/ros/lynsense_webots_sim/worlds/lynsense_smoke.wbt \
  --output /workspace/artifacts/stability-check
```

The probe uses Supervisor ground truth and direct, acceleration-limited motor
commands, with no ROS connection. After five seconds of settling it measures
stationary, straight, left/right turn and stopping phases at 32 ms intervals.
`samples.json` contains the raw poses; `summary.json` requires roll/pitch below
1 degree and each phase's height range below 3 mm, plus actual translation and
both signed turns. These limits detect gross rocking; they are not a hardware
acceptance specification. Run the regular `--phase all` gate separately to
verify the ROS actions and navigation sequence.

## Single-Box Mechanical And Grasp Probes

Two additional Supervisor-only regressions exercise the primitive upper body
without a ROS graph, tree process, or real-robot connection. Run
`run_mechanical.py` with `worlds/lynsense_box.wbt` to verify waist and named
arm/gripper arrival, motion without stall, and zero terminal command rates.
Run `run_grasp_physics.py` with `worlds/lynsense_grasp_physics.wbt` to verify
that both gripper contact sensors remain active through a lift hold and that
the box drops after release. Both probes use Webots fast mode, write
`samples.json` and `summary.json`, and require a fresh output directory.
The physical world starts the waist at -0.478 m, places the box on a narrow
0.6415 m pedestal, and uses long low contact fingers so the -0.278 m lift
command raises the box through actual side contact. These are isolated proof
parameters, not calibrated real-robot grasping limits.
Physical-contact success in this isolated world is separate from the main box
task's validated hybrid attachment.

## Startup And GUI

Headless Webots starts through `xvfb-run` with `--batch --no-rendering
--mode=run`. Webots logs that `--mode=run` is deprecated and falls back to
`fast`; motion deadlines and watchdog deadlines still use simulation time and
monotonic process deadlines respectively. The display override adds only a
read-only X11 socket:

```bash
docker compose -f compose.yaml -f compose.gui.yml run --rm \
  lynsense-webots-smoke --phase smoke
```

The override does not alter the network isolation or source mounts. It also does
not remove `--no-rendering`, so it is useful for display-environment diagnostics
rather than a rendered GUI run.

## Simulation Viewer

The separate LAN viewer renders the `smoke`, `blocked`, `match`, and `box`
gates in realtime. It is an observer only: it does not add perception,
collision avoidance, a competition planner, or real-robot control, and it does
not authorize real-robot motion.

From the RPent repository root, choose one phase and start the viewer service:

```bash
cd robots/lynsense/simulation/docker
LYNSENSE_VIEWER_PHASE=match docker compose -f compose.yaml -f compose.viewer.yaml up lynsense-webots-viewer
LYNSENSE_VIEWER_PHASE=box docker compose -f compose.yaml -f compose.viewer.yaml up lynsense-webots-viewer
```

Open `http://<workstation-ip>:7047/` from a browser on the local network. The
page shows delayed live video with a typical 2-5 seconds of stream delay,
JSONL-driven pose and wheel telemetry, the configured map, action timeline, and
the terminal result, waist/gripper/arm telemetry, and box pose/attachment
state. Use `LYNSENSE_VIEWER_PHASE=smoke`, `blocked`, or `box` to inspect the
other gates.

The **Scene** / **Robot view** selector chooses between two rendered sources:

- Scene is exported directly from the Webots 3D view, with application chrome
  and camera preview overlays excluded. It is a fixed external viewpoint, not
  an interactive browser orbit camera.
- Robot view comes from `viewer_front_camera`, attached to the robot at
  `(0.16, 0, 1.05)` meters in its local frame and tilted down `0.45` radians.
  Its horizontal field of view is `1.22` radians. These are visualization
  defaults, not calibrated real-robot camera intrinsics or extrinsics.

Both streams are silent H.264 at 960x540 and 10 fps. The controller exports
complete scene/camera images, which replace the latest snapshots atomically;
ffmpeg samples these files instead of capturing an X11 desktop. Camera APIs
are used only on the controller's stepping thread, and the original smoke
runner explicitly removes the viewer capture activation environment variable.
Webots overlay settings are isolated under each run's `webots-config/`.

Switching views during replay preserves the playback position and pause state.
The two streams sample the same simulation, but are not frame-locked sensor
recordings. A visual-only ground grid provides motion references and does not
change collision geometry. The finite arena remains a navigation baseline,
not the competition environment; a camera facing outside it may see only the
surrounding ground.

Each run writes evidence under
`.artifacts/lynsense-webots/viewer/<run-id>/`, including `events.jsonl`,
`viewer-summary.json`, HLS segments and playlist, `replay.mp4` when conversion
succeeds, and per-process logs. The blocked phase's expected
`translate_stalled` result is a completed visualization outcome, not an HTTP
failure. Failed MP4 conversion leaves the HLS files on disk. A conversion
timeout or finalization exception fails the viewer and stops its processes;
the failure reason is recorded in the summary when the artifact directory is
writable. Successful runs keep HTTP and replay available until stopped.

Scene HLS is under `video/scene/`, robot HLS under `video/robot/`, and the
first-person replay is `replay-robot.mp4`. Fixed HTTP routes are
`/video/scene/index.m3u8`, `/video/robot/index.m3u8`, `/replay/scene.mp4`, and
`/replay/robot.mp4`; the original video/replay routes alias the scene stream.
The run's video/replay availability flag becomes true only when both views
are available. The latest JPEGs remain in `frames/`, and capture stops when
the run reaches a terminal outcome.

The four worlds use local lighting and an arena-facing camera. Collision and
motor sounds are disabled explicitly, so starting the rendered viewer does not
depend on Webots downloading its default online sound assets.

Stop the viewer with Ctrl-C, then remove the Compose service:

```bash
docker compose -f compose.yaml -f compose.viewer.yaml down
```

The viewer publishes only HTTP TCP port `7047`; ROS remains inside the
container with Domain 42 and localhost-only DDS. It has no start, stop, retry,
motion, or other control endpoint. The original `compose.yaml` smoke gate
remains `network_mode: none` and is not changed by this LAN viewer.

## Known Log Limit

After terminal results, `lynsense_pytrees_node` can log:

```text
rclpy._rclpy_pybind11.RCLError: failed to shutdown:
rcl_shutdown already called on the given context
```

The error appears in successful tree gates during the read-only
tree process's ordered shutdown. These gates still observe their terminal action
results, stop the wheels, exit the tree/controller/Webots processes, and report
`passed`. The upstream shutdown path is in the read-only `lynsense_pytrees`
repository, so this smoke package records the log as a known limitation instead
of patching that repository.
