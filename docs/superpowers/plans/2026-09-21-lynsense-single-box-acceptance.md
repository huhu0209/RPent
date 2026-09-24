# Lynsense Single-Box Acceptance

Date: 2026-09-22 Asia/Shanghai

Status: implementation, regression matrix, and independent implementation
review complete. The review returned PASS with no blockers. Its Important
place-precondition finding was fixed and retested; release-time target drift
received an explicit regression. Two non-blocking review observations remain:
unreachable controller helper returns and an unbounded viewer event `limit`.

## Scope And Limits

This accepts a simulation-only minimum single-box task on the workstation. It
does not connect to robot one, use `ROS_DOMAIN_ID=3`, source a robot workspace,
mount company URDF/STL assets, or claim perception, localization, collision
avoidance, calibrated contact physics, or real-robot authorization.

The end-to-end task uses a hybrid attachment with alignment and carry-drift
checks. Physical side-contact grasp, lift, and release-to-fall are accepted
only in the separate primitive `lynsense_grasp_physics.wbt` proof world.

## Commands

All container commands were run from `robots/lynsense/simulation/docker` with
Domain 42, localhost-only DDS, and the isolated ephemeral overlay:

```bash
docker compose run --rm lynsense-webots-smoke --phase all
docker compose run --rm lynsense-webots-smoke --phase box
docker compose run --rm lynsense-webots-smoke --phase box-all
docker compose run --rm --no-deps --entrypoint python3 lynsense-webots-smoke \
  /workspace/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/scripts/run_stability.py \
  --world /workspace/simulation/ros/lynsense_webots_sim/worlds/lynsense_smoke.wbt \
  --output /workspace/artifacts/stability-box-acceptance-final
docker compose run --rm --no-deps --entrypoint python3 lynsense-webots-smoke \
  /workspace/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/scripts/run_mechanical.py \
  --world /workspace/simulation/ros/lynsense_webots_sim/worlds/lynsense_box.wbt \
  --output /workspace/artifacts/mechanical-box-acceptance-final
docker compose run --rm --no-deps --entrypoint python3 lynsense-webots-smoke \
  /workspace/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/scripts/run_grasp_physics.py \
  --world /workspace/simulation/ros/lynsense_webots_sim/worlds/lynsense_grasp_physics.wbt \
  --output /workspace/artifacts/grasp-physics-box-acceptance-final
```

The original `all` sequence remains interface, smoke, blocked, and match.
`box-all` repeats those phases and appends box.

## Evidence

### Host Suite

Command:

```bash
.venv/bin/pytest -q tests/unit_tests/robots/lynsense/simulation
find robots/lynsense/simulation tests/unit_tests/robots/lynsense/simulation \
  -type f -name '*.py' -print0 |
  xargs -0 .venv/bin/python -m py_compile
git diff --check
```

Result: 353 tests passed in 23.51 seconds; compilation and diff checks exited
zero.

### End-To-End Box Matrix

Unified evidence: `.artifacts/lynsense-webots/summary.json`.

The final `box-all` run passed all five phases in about 28 seconds:

- interface: `20260922T002001Z-11eda99a-c903-4b38-a932-74b923f6ebc0`
- smoke: `20260922T002007Z-e573ad8d-79cf-48b0-b120-34f5afc7927d`
- blocked: `20260922T002015Z-b8b15e73-543d-4232-a8c4-8397fec0d7d7`
- match: `20260922T002018Z-66c787c2-20b3-48e6-a006-2b25726a48d2`
- box: `20260922T002023Z-20317507-8e5d-4e51-bfbf-71c8329eb8cb`

`events-box.jsonl` records exactly 15 successful action results. Pick ended
attached and place ended detached. Place submission now requires a successful
pick and retained attachment evidence in the same manipulation runtime. The
final box was at `(2.81647796301349, -2.728854184259788,
0.10849803799999998)` with yaw `-0.0034015624187844406 rad`; position error
was 0.007955628205173523 m and yaw error 0.009527562418784441 rad.
Post-release evidence spanned 12.288 seconds.
The terminal result succeeded with an empty reason and zero wheel, waist, arm,
and gripper rates.

### Isolated Probes

Stability evidence:
`.artifacts/lynsense-webots/stability-box-acceptance-final/summary.json`.
The probe passed in about three seconds. Maximum absolute roll was below
0.008 degrees, maximum absolute pitch below 0.022 degrees, and every phase
height range was below 0.00007 m. Straight travel was 1.411 m; left turn was
+2.607 rad and right turn was -2.691 rad.

Mechanical evidence:
`.artifacts/lynsense-webots/mechanical-box-acceptance-final/summary.json`.
The probe passed in about three seconds with targets reached, no stall, and
zero terminal rates.

Physical grasp evidence:
`.artifacts/lynsense-webots/grasp-physics-box-acceptance-final/summary.json`.
The probe passed in about three seconds. Both contacts held through lift, the
box rose from 0.747479 m to 0.944855 m, and it fell after release to 0.749794 m.

### Viewer

Run:
`.artifacts/lynsense-webots/viewer/20260922T000308Z-cd3cea6c-1d3e-447b-81b1-34fe9ca7b427`.

The box viewer completed in 60.93 seconds with 1316 events, live video, both
replays, final detached box state, and an empty failure reason. Browser checks
passed at 1440x900 and 390x844 with no horizontal overflow. Scene and robot
replays were both H.264/YUV420p, 960x540, 57.9 seconds, and 579 frames. Both
were nonblank; switching views preserved playback position. Box telemetry
changed from attached to released. The only browser console error was the
known optional favicon 404.

## Tuned Simulation Values

- Box world robot start: `(2.024931, -2.493846)`, yaw `3.124139 rad`.
- Box place target: `(2.810559, -2.734170, 0.1085)`, yaw `0.006126 rad`.
- Carry-aligned place station: `(2.4606561495236394, -2.742422444761675)`,
  yaw `0.35089784218538095 degrees`.
- Box navigation position/yaw tolerances: `0.01 m` and `0.01 rad`.
- Place alignment: `0.05 m`; place yaw: approximately 10 degrees;
  post-release settle: at least `0.5 s`.
- Physical proof box start: `z=0.75 m`; pedestal top: `z=0.6415 m`;
  fingers: `0.08 x 0.1 x 0.4 m`, centered `0.065 m` from each gripper side;
  waist proof moves from `-0.478 m` to `-0.278 m`.

These are simulation gate values, not calibrated robot or competition
operational parameters.
