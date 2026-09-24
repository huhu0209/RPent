# Lynsense Navigation Viewer Acceptance

## Dual-Camera Correction (2026-09-21)

The user clarified that the desired video sources are a clean 3D scene and a
robot first-person camera, not a captured Webots window. This correction
supersedes the single-window media implementation described below.

- Scene images now use `Supervisor.exportImage`; per-run Webots preferences
  hide device overlays. First-person images use an on-robot `Camera` device.
- Front camera visualization defaults: local position `(0.16, 0, 1.05)` m,
  downward tilt `0.45` rad, horizontal FOV `1.22` rad. Not calibrated hardware.
- Both image exports must succeed before individual atomic file replacements.
  A single ffmpeg process produces independent scene/robot HLS streams.
- Scene/robot replay paths support byte ranges. View switching retains replay
  position and pause state; live switching retains its playback position.
- Visual-only grid/ground shapes add no physics or collision objects. Motion
  primitives, action sequence, and real-robot configuration remain unchanged.

Final rendered match run:
`20260921T130840Z-1caba44e-a752-49b7-963a-652cf900b66e`, under
`.artifacts/lynsense-webots/viewer/`. Both MP4s are H.264, 960x540, 10 fps,
63.2 seconds, without audio. `status=completed` and both availability flags
are true.

Browser acceptance includes both HLS streams advancing while running, scene
and robot replay seeking, paused view-switch at 5 seconds retaining that time,
and desktop/mobile screenshots with no horizontal overflow. Pixel comparison
at 5 and 25 seconds on the previous visually identical run found 779 changed
scene pixels and 96,410 changed robot-view pixels in 480x270 samples. This
checks actual moving imagery, not just codec readiness. Screenshots:
`.playwright-cli/dual-scene-desktop.png`, `dual-robot-desktop.png`, and
`dual-robot-mobile.png` in the same directory.

Regression: 368 Lynsense tests passed. The original isolated four-phase
`--phase all` gate passed after adding the passive camera/grid. The final
environment hardening also has a regression proving smoke does not inherit
the viewer capture switch. Independent Luna review found no remaining P1.
`git diff --check` passed. No new dependency, commit, push, or robot connection.

Limits: the external scene camera is fixed, not mouse-orbitable; the two
streams are not hard-synchronized sensor recordings. The first-person camera
can face outside the small arena and see only the surrounding ground. This
remains the simplified navigation scene, not a complete competition task.
Only match received dual-camera live/browser acceptance; the other phases
share the capture path but were verified through the original simulation gate
and structural tests. The viewer remains available at the workstation LAN
address on port 7047.

## Original Single-Window Acceptance

Date: 2026-09-21. Branch: `feat/lynsense-readonly-state`.

This delivers stage 1, a workstation-only navigation viewer. It is not the
complete box-moving simulation or RPent strategy integration. The real-robot
backend and its Dashboard restrictions are unchanged. No robot connection,
commit, push, merge, or PR was performed during this acceptance.

## Fixes Found During Acceptance

- Restricted launch-file packaging so generated `__pycache__` directories do
  not break the container's colcon build.
- Reused the smoke runner's software-rendering environment and waited for the
  Webots ready marker before starting its external controller.
- Disabled default online contact/motor sound assets. Bridge-mode startup
  failed before readiness with exit 139 while the same image completed with
  no networking; bridge runs completed after removing the sound downloads.
  This is an observed mitigation, not a native crash-stack diagnosis.
- Added local lighting and aimed the camera at the arena. Codec validation
  alone had missed an entirely black 3D view.
- Made HTTP event pagination read cached events only. Runtime now exclusively
  consumes JSONL and validates every batch, including the first tree batch.
- Bounded MP4 conversion to 30 seconds and guaranteed cleanup attempts after
  finalization, summary, and individual child-termination failures.
- Prevented initial HLS setup from overwriting an already selected MP4 replay.
- Added bounded single-byte-range MP4 responses for actual browser seeking.
- Aligned new artifact directories with the documented host mount path.

## Verification

| Check | Result |
| --- | --- |
| `.venv/bin/pytest tests/unit_tests/robots/lynsense/simulation -q` | 229 passed |
| `.venv/bin/pytest tests/unit_tests/robots/lynsense -q` | 339 passed, includes simulation tests |
| Python compileall | Passed |
| `git diff --check` | Passed |
| Original isolated Compose `--phase all` | interface, smoke, blocked, match all passed |
| Independent Luna review | Both P1 findings closed; no remaining P0/P1 found before the separate MP4 range patch |
| MP4 range patch focused tests | 33 passed; primary reviewed changes and verified browser seek |

The original gate summary is `.artifacts/lynsense-webots/summary.json`, generated
at `2026-09-21T12:18:54Z`. It records Domain 42, localhost-only DDS, and no
robot-workspace imports.

## Rendered Runs

| Phase | Completed run ID | Video duration |
| --- | --- | --- |
| smoke | `20260921T122341Z-ba69072a-0b1b-42a3-ad6f-7daae572e850` | 37.35 s |
| blocked | `20260921T122602Z-9bc16a05-d523-4d69-8c5d-954af1b2048b` | 8.10 s |
| match | `20260921T122903Z-d134a6e3-b43e-41e9-b8ef-f3b5a4b5a41a` | 56.90 s |

All three recordings have one H.264 1280x1024 video stream, no audio, and an HLS
end marker. The blocked run is completed with the expected action failure
`translate_stalled`. New evidence is in
`.artifacts/lynsense-webots/viewer/<run-id>/`; the smoke evidence predates the
path correction and remains in
`.artifacts/lynsense-webots/lynsense-webots/viewer/<run-id>/`.

Playwright accessed the workstation LAN address `http://192.168.77.14:7047/`.
Final match HLS playback advanced while status was `running`, with 99,775
non-dark pixels in a 600x650 crop of the 3D view. Replay seeking to 15 seconds
advanced to 16.003 seconds; sampling the scene at 10 and 35 seconds found 4,388
changed pixels. The MP4 range request `bytes=0-31` returned 206 and 32 bytes.
No playback error or horizontal overflow was observed.

Desktop 1440x900 and mobile 390x844 screenshots are in `.playwright-cli/`:
`viewer-final-desktop.png`, `viewer-final-mobile.png`,
`viewer-smoke-desktop.png`, `viewer-smoke-mobile.png`,
`viewer-blocked-desktop.png`, and `viewer-blocked-mobile.png`.
Live playback was explicitly measured for match; smoke/blocked were checked
through completed-run browser playback and their media artifacts.

## Service And Limits

Temporary validation containers were removed. The documented Compose service
`docker-lynsense-webots-viewer-1` is intentionally left serving the completed
match and replay on TCP 7047. From `robots/lynsense/simulation/docker`, stop it
with `docker compose -f compose.yaml -f compose.viewer.yaml down`.

Only the workstation's own browser was used to access its LAN address; access
from a second physical LAN device and firewall policy were not tested.
The viewer is unauthenticated HTTP for a trusted LAN, not an Internet service.
The review noted a non-blocking resource risk: `/api/events` accepts a client
limit without a server maximum. The upstream tree shutdown warning documented
in the simulation README remains. Full RPent tests and the full platform matrix
were not run; validation was scoped to Lynsense and the simulation gate.

## Chassis Rocking Correction (2026-09-21)

Earlier navigation/video gates did not measure chassis roll, pitch, or height.
The wheel cylinders were Z-aligned while their hinges and axial inertia were
Y-aligned. All three worlds now rotate the shared visual/collision Pose about X
by pi/2, retaining the wheel Solid frame and inertia. Wheel/floor and caster/floor
contacts also explicitly use `bounce 0` and `softCFM 0.00001`; friction, masses,
motor limits, and navigation control parameters are unchanged.

An isolated Supervisor probe applies the same acceleration-limited commands to
the original and corrected worlds. Measurements exclude initial settling:

| Variant | Straight max pitch | Straight height range | Maximum turn roll |
| --- | --- | --- | --- |
| Original model | 5.743 degrees | 47.196 mm | 4.869 degrees |
| Wheel orientation only | 3.187 degrees | 6.665 mm | 0.551 degrees |
| Wheel and contact correction | 0.01985 degrees | 0.06745 mm | 0.00724 degrees |

Raw samples and summaries are under `.artifacts/lynsense-webots/` in
`stability-before/result`, `stability-after-smoke`, `stability-after-contact`,
and `stability-after-match`. The orientation-only model still settled about
15 mm below its initial root height. The final smoke and match probes pass the
1-degree attitude and 3-mm phase-height-range checks and actual-motion checks.
These are numerical regression limits, not real-robot calibration evidence.

The interface, smoke, blocked, and match ROS gates all passed after correction.
The updated viewer run is `20260921T134200Z-b429d3aa-4ad4-4749-9631-cd99f40bb798`;
it completed successfully, and Playwright verified both 960x540 replays play
and switch views. Screenshot: `.playwright-cli/stability-fixed-robot.png`.
Recreating the viewer container was needed because a plain restart retained
an old Xvfb display lock; the completed-run service remains on port 7047.
