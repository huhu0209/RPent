# Lynsense Webots Navigation Viewer Design

Date: 2026-09-21

Status: design approved by the user; implementation has not started.

## Goal

Add a simulation-only, read-only viewer for the three navigation gates that are
already supported by the Lynsense Webots framework:

- `smoke`: the deterministic successful navigation sequence.
- `blocked`: the expected translation-stall failure sequence.
- `match`: the navigation-only WHRG2026 excerpt using `搬箱子1`,
  `MoveDistance(-0.6, 0)`, `MoveDistance(0, 90)`, and `放箱子1_1`.

The viewer must show a run while it is happening and preserve a replayable
recording after it finishes. The user explicitly accepted a 2-5 second video
delay and allowed direct access from the local-area network.

This is a viewer for navigation baselines. It is not a complete box-moving
task, does not include manipulation, perception, localization, collision
avoidance, or a real robot, and must not be presented as authorization to move
robot one.

## Non-Goals

- Do not connect to robot one or use `ROS_DOMAIN_ID=3`.
- Do not expose ROS DDS, the Webots controller, or the behavior-tree process
  outside the container.
- Do not add start, stop, retry, parameter-change, or other control endpoints.
- Do not enable Dashboard for the real `robots/lynsense` backend.
- Do not integrate the RPent Planner or add a Lynsense motion Toolkit in this
  stage.
- Do not modify the existing `interface`, `smoke`, `blocked`, `match`, or `all`
  regression gates.
- Do not render the full competition scene or implement box manipulation.

## Architecture

The existing gate remains unchanged:

```text
compose.yaml
-> lynsense-webots-smoke
-> network_mode: none
-> headless Webots
-> automatic pass/fail gate
```

Visualization uses a separate Compose service:

```text
compose.viewer.yaml
-> lynsense-webots-viewer
-> Docker bridge network
-> read-only HTTP viewer on 0.0.0.0:7047
-> Xvfb display
-> Webots realtime rendering
-> Supervisor controller
-> lynsense_pytrees node
-> ffmpeg x11grab
-> HLS live stream
-> replay.mp4 and JSONL evidence
```

The viewer service uses the same isolated source mounts as the gate:

- the simulation subtree, read-only;
- the sibling `lynsense_pytrees` checkout, read-only;
- `RPent/.artifacts/lynsense-webots`, writable.

It does not mount the repository root, `.env.lynsense`, SSH configuration,
home directories, robot workspaces, or company URDF assets.

The only published port is HTTP `7047`. Docker bridge networking is required
for LAN access, but ROS remains scoped to the container with
`ROS_DOMAIN_ID=42` and `ROS_LOCALHOST_ONLY=1`. DDS ports are not published.
Unlike the no-network gate, a Docker bridge network can also permit outbound
container traffic. This is part of the user-approved LAN trust model: the
viewer runtime must not fetch remote assets or read credentials, all frontend
resources are served from the mounted simulation subtree, and source mounts stay
limited to the approved read-only paths.

## Runtime Flow

The viewer is launched from the Docker directory with one selected phase:

```bash
LYNSENSE_VIEWER_PHASE=match docker compose -f compose.yaml -f compose.viewer.yaml up \
  lynsense-webots-viewer
```

`LYNSENSE_VIEWER_PHASE` accepts `smoke`, `blocked`, and `match`; `match` is the
default. The service is started with `compose up`, rather than
compose `run`, because `run` does not publish the service's LAN port. The
foreground command keeps the replay server alive until the operator interrupts
it; `docker compose down` after interruption removes the container.

Startup order:

1. Build the ephemeral ROS overlay without writing to the read-only source
   checkouts.
2. Start the HTTP viewer and health endpoint.
3. Start a fixed Xvfb display.
4. Start rendered Webots in realtime mode without `--no-rendering`.
5. Start the Supervisor controller and wait for readiness.
6. Start `ffmpeg`, verify that it remains alive, and wait until its initial HLS
   playlist exists.
7. Wait for both ROS action endpoints, then start `lynsense_pytrees_node` for
   the selected phase.
8. Continue serving the page and HLS stream until the run reaches its terminal
   action result.
9. Finalize the HLS playlist, convert it to MP4, write the viewer summary, and
   keep serving the replay until the operator interrupts the container.

The HTTP server starts before simulation so a browser can show startup,
discovery, rendering, and failure states rather than appearing only after a
successful launch.

## Video Pipeline

Webots renders to a fixed Xvfb display. `ffmpeg` captures that display with
`x11grab`.

The first implementation uses 20 fps, 1280x1024 capture, H.264, and HLS with
one-second segments. The live playlist may retain all segments so the completed
playlist can be converted to a single MP4:

```text
viewer/<run_id>/video/index.m3u8
viewer/<run_id>/video/segment-*.ts
viewer/<run_id>/replay.mp4
```

The exact encoder flags may be tuned during implementation, but the observable
contract is:

- browser-compatible live video over HTTP;
- 2-5 seconds typical end-to-end delay;
- no audio track;
- finite disk use bounded by the run duration;
- playlist receives `#EXT-X-ENDLIST` only after the terminal event;
- MP4 generation is attempted only after the playlist is finalized;
- partial video remains available if simulation or MP4 conversion fails.

The viewer page labels the video as delayed live playback. Event state and the
trajectory update directly from JSONL and are therefore expected to lead the
video by the streaming delay.

The page uses the browser's native HLS support when available, such as Safari,
and otherwise uses a version-pinned local copy of `hls.js` served from the
simulation subtree. It must not load media JavaScript from a CDN or require an
internet connection. If neither native HLS nor the local polyfill can initialize
playback, the page shows an explicit video playback failure while continuing to
display events and trajectory.

## Viewer API And UI

The viewer is a small Python HTTP service backed by static HTML, CSS, and
JavaScript. It does not expose generic file serving. Allowed routes are:

```text
GET /
GET /healthz
GET /api/run
GET /api/events?after=<event-index>
GET /static/app.css
GET /static/app.js
GET /static/vendor/hls.min.js
GET /video/index.m3u8
GET /video/segment-*.ts
GET /replay.mp4
```

The page contains:

- live HLS video;
- top-level run state: phase, run ID, outcome, simulation time;
- current pose `x`, `y`, yaw;
- left and right wheel rates;
- action timeline for accepted goals, feedback, results, rejection, and
  terminal lockout;
- 2D map with goal points and the robot trajectory;
- active action, phase, reason, and remaining distance/angle;
- link to the finalized MP4 replay after the run ends.

The event API can be polled rather than using WebSocket or SSE. A 250-500 ms
poll interval is sufficient because the video itself has a 2-5 second delay.
The frontend must tolerate polling errors, an empty event stream while Webots
starts, and stale HLS output.

The map is scaled from each phase's configured arena and goals. The `match`
coordinates remain unscaled and untranslated in the simulation world; the map
view may scale only its pixels to fit the page.

## Event And Replay Data

Each viewer run gets a unique directory:

```text
.artifacts/lynsense-webots/viewer/<run_id>/
```

It contains:

```text
events.jsonl
viewer-summary.json
video/index.m3u8
video/segment-*.ts
replay.mp4
webots.log
controller.log
tree.log
ffmpeg.log
```

`events.jsonl` records the same logical event objects produced by the existing
controller. The viewer records an ingest offset relative to video capture start
when it observes each event. This does not alter controller behavior; it gives
the frontend a best-effort wall-clock alignment for replay. Exact frame-level
event-video synchronization is not promised in this stage.

`viewer-summary.json` records:

- selected phase;
- run ID;
- start and end wall-clock times;
- event count;
- terminal action and result;
- final pose and wheel rates;
- HLS and MP4 availability;
- process exit statuses;
- explicit `real_robot_connected: false`.

## Failure Handling

Failure must preserve evidence and clean up processes.

- If Xvfb, Webots, the controller, the tree, `ffmpeg`, or the HTTP server exits
  unexpectedly, the viewer marks the run failed and terminates the remaining
  child process groups.
- If the selected gate fails, the page shows the action failure and reason;
  this is a valid visualization outcome, especially for `blocked`.
- If video capture fails but simulation succeeds, the run remains available as
  an event and trajectory replay, and the summary marks video unavailable.
- If simulation fails before video starts, the page shows the startup failure
  and retains logs.
- If MP4 conversion fails, finalized HLS remains the replay source.
- HTTP route handling must reject path traversal and unknown paths.
- Interrupting the container must stop Webots, the controller, the tree,
  `ffmpeg`, Xvfb, and the HTTP server.

## Testing

Focused unit and static tests:

- original gate Compose remains `network_mode: none` and publishes no ports;
- viewer Compose uses bridge networking and publishes only TCP `7047`;
- viewer mounts only the approved simulation, pytrees, and artifact paths;
- Dockerfile adds `ffmpeg` without weakening the digest-pinned base image;
- HTTP route table is read-only and contains no mutation endpoint;
- static file handler rejects traversal and serves expected media types;
- event pagination returns only complete JSONL records after the requested
  index;
- viewer summary serializes finite, deterministic fields;
- process cleanup is invoked for every launch and terminal failure path.

Container validation:

1. Run the viewer for `match`.
2. Verify `/healthz`, page assets, event pagination, and HLS playlist over the
   published port.
3. Verify the terminal action result, final pose, and zero wheel rates in the
   viewer summary.
4. Verify `replay.mp4` exists, is non-empty, and is playable by ffprobe.
5. Repeat focused viewer runs for `smoke` and `blocked`.
6. Run the original `--phase all` gate and confirm `interface`, `smoke`,
   `blocked`, and `match` still pass.
7. Confirm no Webots, pytree, controller, ffmpeg, or viewer process remains.

Browser-level visual inspection is required before delivery. It must confirm
that the video, map, timeline, and terminal states are visible without overlap
at desktop and narrow viewport widths. The same browser validation must also
confirm actual media behavior: during a live run the HLS player reaches a ready
state and its playback position advances; after the run, the MP4 element can
start playback and seek. Passing only ffprobe or a screenshot is insufficient.

## Acceptance

Stage 1 is complete when:

- the three navigation behaviors can be watched live from the LAN;
- events and trajectory update during the run;
- completed runs produce replayable video and a viewer summary;
- the original no-network gate still passes unchanged;
- the page clearly identifies itself as a navigation-baseline viewer;
- no real robot connection or motion authorization is introduced.
