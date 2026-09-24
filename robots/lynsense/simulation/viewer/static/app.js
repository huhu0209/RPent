const state = {
  afterIndex: -1,
  consecutiveFailures: 0,
  events: [],
  latestRun: null,
  hls: null,
  mediaMode: null,
  mediaView: null,
  replayLoaded: false,
  videoInitialized: false,
  view: "scene",
};

const VIEWS = {
  scene: {
    label: "Scene",
    live: "/video/scene/index.m3u8",
    replay: "/replay/scene.mp4",
  },
  robot: {
    label: "Robot view",
    live: "/video/robot/index.m3u8",
    replay: "/replay/robot.mp4",
  },
};

async function fetchJSON(path) {
  const response = await fetch(path, {cache: "no-store"});
  if (!response.ok) throw new Error(`HTTP ${response.status}`);
  return response.json();
}

async function fetchRun() { return fetchJSON("/api/run"); }
async function fetchEvents(afterIndex) { return fetchJSON(`/api/events?after=${afterIndex}`); }

function formatNumber(value) {
  return Number.isFinite(Number(value)) ? Number(value).toFixed(3) : "-";
}

function renderTelemetry(run) {
  const pose = run.pose;
  const rates = run.motor_rates;
  const manipulation = run.manipulation;
  const box = run.box;
  document.querySelector("#run-id").textContent = run.run_id || "-";
  document.querySelector("#phase").textContent = run.phase || "-";
  document.querySelector("#pose").textContent = pose ? `x ${formatNumber(pose.x)} · y ${formatNumber(pose.y)} · yaw ${formatNumber(pose.yaw)}` : "-";
  document.querySelector("#wheel-rates").textContent = rates ? `L ${formatNumber(rates.left_wheel_rate_radps)} · R ${formatNumber(rates.right_wheel_rate_radps)}` : "-";
  document.querySelector("#waist-position").textContent = manipulation ? `${formatNumber(manipulation.waist_position_mm)} mm` : "-";
  document.querySelector("#gripper-positions").textContent = manipulation ? `L ${formatNumber(manipulation.gripper_positions?.[0])} · R ${formatNumber(manipulation.gripper_positions?.[1])}` : "-";
  document.querySelector("#arm-error").textContent = manipulation ? `${formatNumber(manipulation.max_arm_joint_error_rad)} rad` : "-";
  document.querySelector("#box-pose").textContent = box ? `x ${formatNumber(box.position_m?.[0])} · y ${formatNumber(box.position_m?.[1])} · yaw ${formatNumber(box.orientation_rad?.[2])}` : "-";
  document.querySelector("#box-state").textContent = box ? (box.attached ? "Attached" : "Released") : "-";
  document.querySelector("#active-action").textContent = run.current_event?.action || "-";
  document.querySelector("#run-reason").textContent = run.reason || run.current_event?.reason || "-";
  document.querySelector("#simulation-time").textContent = `Simulation time ${formatNumber(run.current_event?.sim_time_s)} s`;
}

function renderRun(run) {
  state.latestRun = run;
  document.querySelector("#run-state").textContent = run.status || "-";
  renderTelemetry(run);
  updateReplayLink(run);
  const terminal = document.querySelector("#terminal-summary");
  terminal.hidden = !run.terminal_event;
  if (run.terminal_event) {
    const event = run.terminal_event.event;
    document.querySelector("#terminal-text").textContent = `${event.kind}: ${event.success === true ? "success" : event.reason || "failed"} · pose ${event.pose ? `${formatNumber(event.pose.x)}, ${formatNumber(event.pose.y)}` : "-"}`;
  }
  drawMap(run.map, state.events);
}

function appendTimelineEntries(events) {
  const timeline = document.querySelector("#action-timeline");
  for (const record of events.slice(-200)) {
    const event = record.event || {};
    const item = document.createElement("li");
    item.dataset.kind = event.kind || "unknown";
    item.textContent = `${event.kind || "event"}${event.action ? ` · ${event.action}` : ""}${event.goal_name ? ` · ${event.goal_name}` : ""}${event.reason ? ` · ${event.reason}` : ""}`;
    timeline.appendChild(item);
  }
  while (timeline.children.length > 200) timeline.removeChild(timeline.firstChild);
}

function renderEvents(events) {
  state.events.push(...events.filter((record) => record.event && record.event.pose));
  if (state.events.length > 2000) state.events.splice(0, state.events.length - 2000);
  appendTimelineEntries(events);
  if (state.latestRun) drawMap(state.latestRun.map, state.events);
}

function drawMap(map, events) {
  const canvas = document.querySelector("#map");
  const context = canvas.getContext("2d");
  const arena = map?.arena || {};
  const boundaryX = Number(arena.boundary_x_m) || 1;
  const boundaryY = Number(arena.boundary_y_m) || 1;
  const goals = Object.values(map?.goals || {});
  const values = goals.flatMap((goal) => [Number(goal.x), Number(goal.y)]).concat(events.flatMap((record) => [Number(record.event.pose.x), Number(record.event.pose.y)]));
  const minX = Math.min(-boundaryX, ...values.filter(Number.isFinite));
  const maxX = Math.max(boundaryX, ...values.filter(Number.isFinite));
  const minY = Math.min(-boundaryY, ...values.filter(Number.isFinite));
  const maxY = Math.max(boundaryY, ...values.filter(Number.isFinite));
  const scale = Math.min(canvas.width / Math.max(maxX - minX, 1), canvas.height / Math.max(maxY - minY, 1));
  const toX = (x) => (x - minX) * scale;
  const toY = (y) => canvas.height - (y - minY) * scale;
  context.clearRect(0, 0, canvas.width, canvas.height);
  context.strokeStyle = "#6b7d84";
  context.strokeRect(toX(-boundaryX), toY(boundaryY), 2 * boundaryX * scale, 2 * boundaryY * scale);
  context.fillStyle = "#e7b56d";
  context.font = "14px system-ui";
  for (const [name, goal] of Object.entries(map?.goals || {})) context.fillText(name, toX(Number(goal.x)), toY(Number(goal.y)));
  context.strokeStyle = "#6ed0bd";
  context.lineWidth = 3;
  context.beginPath();
  events.forEach((record, index) => {
    const x = toX(Number(record.event.pose.x));
    const y = toY(Number(record.event.pose.y));
    if (index === 0) context.moveTo(x, y); else context.lineTo(x, y);
  });
  context.stroke();
}

function showVideoError(message) {
  const element = document.querySelector("#video-error");
  element.textContent = message;
  element.hidden = false;
}

function clearVideoError() {
  const element = document.querySelector("#video-error");
  element.textContent = "";
  element.hidden = true;
}

function updateReplayLink(run) {
  const link = document.querySelector("#replay-link");
  const view = VIEWS[state.view];
  link.href = view.replay;
  link.textContent = `Open ${view.label.toLowerCase()} replay`;
  link.hidden = !run?.replay_available;
}

function updateViewControls() {
  for (const viewName of Object.keys(VIEWS)) {
    const button = document.querySelector(`#view-${viewName}`);
    const selected = viewName === state.view;
    button.classList.toggle("is-active", selected);
    button.setAttribute("aria-pressed", String(selected));
  }
  document.querySelector("#video").setAttribute("aria-label", `${VIEWS[state.view].label} simulation video`);
}

function destroyHls() {
  if (state.hls) {
    state.hls.destroy();
    state.hls = null;
  }
}

function resetVideoSource(video) {
  if (typeof video.removeAttribute === "function") video.removeAttribute("src");
  else video.src = "";
  if (typeof video.load === "function") video.load();
}

function stopLiveVideo(video) {
  destroyHls();
  if (typeof video.pause === "function") video.pause();
  resetVideoSource(video);
}

function initializeVideo() {
  const video = document.querySelector("#video");
  const view = VIEWS[state.view];
  clearVideoError();
  if (state.mediaMode === "live" && state.mediaView === state.view) return;
  const wasPaused = state.mediaMode !== null && video.paused !== false;
  const currentTime = state.mediaMode === "live" ? Number(video.currentTime) : 0;
  stopLiveVideo(video);
  state.mediaMode = "live";
  state.mediaView = state.view;
  state.replayLoaded = false;
  state.videoInitialized = true;
  if (video.canPlayType("application/vnd.apple.mpegurl")) {
    video.src = view.live;
    if (typeof video.load === "function") video.load();
  } else if (window.Hls && window.Hls.isSupported()) {
    const HlsConstructor = window.Hls;
    state.hls = new HlsConstructor();
    state.hls.on(HlsConstructor.Events.ERROR, (_event, data) => {
      if (data.fatal) showVideoError(`${view.label} live video stream failed`);
    });
    state.hls.loadSource(view.live);
    state.hls.attachMedia(video);
  } else {
    showVideoError("HLS playback is unavailable in this browser");
  }
  restorePlayback(video, currentTime, wasPaused);
}

function restorePlayback(video, currentTime, wasPaused) {
  const expectedView = state.mediaView;
  const expectedMode = state.mediaMode;
  const restore = () => {
    if (state.mediaView !== expectedView || state.mediaMode !== expectedMode) return;
    if (Number.isFinite(currentTime) && currentTime > 0) {
      const target = Number.isFinite(video.duration) ? Math.min(currentTime, Math.max(0, video.duration - 0.05)) : currentTime;
      try { video.currentTime = target; } catch (_error) { /* wait for metadata */ }
    }
    if (!wasPaused && typeof video.play === "function") {
      const result = video.play();
      if (result && typeof result.catch === "function") result.catch(() => {});
    }
  };
  restore();
  if (typeof video.addEventListener === "function") video.addEventListener("loadedmetadata", restore, {once: true});
}

function switchToReplay() {
  if (state.mediaMode === "replay" && state.mediaView === state.view) return;
  const view = VIEWS[state.view];
  const video = document.querySelector("#video");
  const currentTime = Number(video.currentTime);
  const wasPaused = video.paused !== false;
  state.replayLoaded = true;
  state.mediaMode = "replay";
  state.mediaView = state.view;
  state.videoInitialized = false;
  if (typeof video.pause === "function") video.pause();
  destroyHls();
  video.src = view.replay;
  if (typeof video.load === "function") video.load();
  restorePlayback(video, currentTime, wasPaused);
  clearVideoError();
}

function syncMedia(run) {
  if (run.replay_available) {
    switchToReplay();
  } else if (run.video_available && !state.replayLoaded) {
    initializeVideo();
  }
}

function setView(viewName) {
  if (!VIEWS[viewName] || state.view === viewName) return;
  state.view = viewName;
  updateViewControls();
  if (state.latestRun) {
    updateReplayLink(state.latestRun);
    syncMedia(state.latestRun);
  }
}

async function poll() {
  try {
    const [run, eventPage] = await Promise.all([fetchRun(), fetchEvents(state.afterIndex)]);
    state.consecutiveFailures = 0;
    renderRun(run);
    syncMedia(run);
    if (eventPage.events.length) {
      state.afterIndex = eventPage.events[eventPage.events.length - 1].index;
      renderEvents(eventPage.events);
    }
  } catch (error) {
    state.consecutiveFailures += 1;
    document.querySelector("#run-state").textContent = state.consecutiveFailures > 5 ? "connection unavailable" : "connection retrying";
  }
}

for (const viewName of Object.keys(VIEWS)) {
  document.querySelector(`#view-${viewName}`).addEventListener("click", () => setView(viewName));
}
updateViewControls();
window.viewerState = state;
window.viewerPoll = poll;
poll();
window.setInterval(poll, 400);
