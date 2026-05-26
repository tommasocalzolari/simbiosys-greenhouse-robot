import copy
import json
import math
import os
import socket
import threading
import time
from datetime import datetime, timezone
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse, urlunparse

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, CompressedImage
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    from simbiosys_interfaces.action import ExecuteBehavior
    from simbiosys_interfaces.msg import BedObservation, BehaviorType, MappingStatus, PlantHealth, TaskStatus
    from simbiosys_interfaces.srv import SendNamedArmPose, SetRobotMode
except ImportError:
    ExecuteBehavior = None
    BedObservation = None
    BehaviorType = None
    MappingStatus = None
    PlantHealth = None
    TaskStatus = None
    SendNamedArmPose = None
    SetRobotMode = None

CONFIG_PATH = Path(__file__).resolve().parent / "config" / "rosTopics.json"
DEFAULT_WEB_HOST = "0.0.0.0"
DEFAULT_WEB_PORT = 8080
DEFAULT_CAMERA_MAX_FPS = 1.0
DEFAULT_TELEOP_QUEUE_DEPTH = 1
DEFAULT_CONFIG = {
    "rosbridgeUrl": "ws://localhost:9090",
    "topics": {
        "cmdVel": "/mirte_base_controller/cmd_vel",
        "baseCameraCompressed": "/camera/color/image_raw/compressed",
        "baseCameraRaw": None,
        "armCameraCompressed": "/gripper_camera/image_raw/compressed",
        "armCameraRaw": None,
        "liveMap": "/map",
        "mapUpdatePeriodSec": 10,
        "mappingStatus": "simbiosys/mapping_status",
        "taskStatus": "simbiosys/task_status",
        "setTaskMode": "simbiosys/set_robot_mode",
        "executeBehavior": "simbiosys/execute_behavior",
        "sendNamedArmPose": "simbiosys/send_named_arm_pose",
        "startMapping": "/mapping/start",
        "doneMapping": "/mapping/done",
        "artifactCandidates": "/mapping/artifact_candidates",
        "classifyArtifact": None,
        "saveSafeMap": "/mapping/save_safe_map",
        "safeMapOutput": None,
        "takeControl": None,
        "odom": "/mirte_base_controller/odom",
        "amclPose": "/amcl_pose",
        "plantHealth": "/plant_health",
        "plantHealthReport": "/plant_health_report",
        "typedPlantHealth": "simbiosys/plant_health",
        "flowerCounts": "/simbiosys/flower_counts",
        "bedObservation": "simbiosys/bed_observation",
        "battery": "/io/power/power_watcher",
    },
}

SPEED_MODES = {
    "slow": {"linear": 0.50, "angular": 0.8},
    "normal": {"linear": 0.75, "angular": 1.4},
    "fast": {"linear": 1.00, "angular": 2.0},
}
MAX_LINEAR_SPEED = 1.0
MAX_ANGULAR_SPEED = 2.0
ARM_POSES = ("home", "camera_forward", "camera_down", "inspect", "stow")
TASK_MODE_TO_BACKEND = {"harvest": "HARVESTING", "scanning": "SCANNING"}


class UiHttpServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True
    block_on_close = False


INDEX_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>SimBioSys UI</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #101312;
      --panel: #18211e;
      --panel-2: #202b27;
      --line: #34433d;
      --text: #edf5ef;
      --muted: #a9b8af;
      --green: #56c271;
      --yellow: #e1b94b;
      --red: #e2685d;
      --blue: #66a8d9;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      background: var(--bg);
      color: var(--text);
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }
    main {
      width: min(1280px, 100%);
      margin: 0 auto;
      padding: 18px;
      display: grid;
      gap: 14px;
    }
    header, .topline, .control-row {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 10px;
      flex-wrap: wrap;
    }
    h1 { margin: 0; font-size: 1.3rem; letter-spacing: 0; }
    h2 { margin: 0; font-size: 1.02rem; letter-spacing: 0; }
    h3 { margin: 0; font-size: 0.95rem; letter-spacing: 0; }
    p { margin: 0; color: var(--muted); }
    button, select {
      border: 1px solid var(--line);
      border-radius: 8px;
      background: var(--panel-2);
      color: var(--text);
      font: inherit;
    }
    button {
      min-height: 42px;
      padding: 0 14px;
      font-weight: 700;
      cursor: pointer;
      touch-action: manipulation;
    }
    button:disabled, select:disabled {
      cursor: not-allowed;
      color: #78847e;
      background: #141a18;
    }
    button:active:not(:disabled), button.active {
      background: var(--green);
      border-color: var(--green);
      color: #07110b;
    }
    select { min-height: 38px; padding: 0 10px; }
    canvas {
      width: 100%;
      height: 100%;
      min-height: 300px;
      display: block;
      background: #0a0d0c;
    }
    .page { display: none; }
    .page.active { display: grid; gap: 14px; }
    .panel {
      border: 1px solid var(--line);
      border-radius: 8px;
      background: var(--panel);
      overflow: hidden;
    }
    .panel-body { padding: 14px; display: grid; gap: 12px; }
    .status-pill {
      border: 1px solid var(--line);
      border-radius: 999px;
      padding: 6px 10px;
      color: var(--muted);
      background: #121917;
      font-size: 0.9rem;
      white-space: nowrap;
    }
    .safety {
      min-width: 112px;
      border-color: #ffaaa3;
      background: var(--red);
      color: #210504;
    }
    .safety.paused {
      border-color: #a7efb7;
      background: var(--green);
      color: #07110b;
    }
    .dashboard-grid {
      display: grid;
      grid-template-columns: minmax(0, 1.2fr) minmax(300px, 0.8fr);
      gap: 14px;
    }
    .metric-grid, .bed-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px;
    }
    .metric, .bed-card {
      border-top: 1px solid var(--line);
      padding-top: 10px;
      color: var(--muted);
    }
    .metric strong, .bed-card strong {
      display: block;
      color: var(--text);
      font-size: 1.1rem;
    }
    .bed-card {
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 12px;
      background: #17201d;
      display: grid;
      gap: 8px;
    }
    .bed-card.available { border-color: #4d7258; }
    .bed-card.unavailable { border-color: #59615d; }
    .teleop-main {
      display: grid;
      grid-template-columns: minmax(280px, 0.9fr) minmax(340px, 1.1fr);
      gap: 14px;
    }
    .camera {
      min-height: 320px;
      display: grid;
      place-items: center;
      background: #040605;
    }
    .camera img {
      width: 100%;
      height: 100%;
      max-height: 46vh;
      object-fit: contain;
      display: block;
    }
    .placeholder {
      min-height: 140px;
      display: grid;
      place-items: center;
      padding: 20px;
      text-align: center;
      color: var(--muted);
    }
    .map-panel {
      min-height: 330px;
      position: relative;
    }
    .workflow-grid {
      display: grid;
      grid-template-columns: minmax(300px, 0.8fr) minmax(320px, 1fr);
      gap: 14px;
    }
    .pad, .arm-pose-grid {
      display: grid;
      grid-template-columns: repeat(3, minmax(70px, 1fr));
      gap: 10px;
    }
    .pad { grid-template-rows: repeat(3, 74px); }
    .empty { visibility: hidden; }
    .candidate-list {
      display: grid;
      gap: 8px;
      max-height: 220px;
      overflow: auto;
    }
    .candidate-list button { min-height: 46px; text-align: left; }
    .candidate-list small { display: block; color: var(--muted); margin-top: 4px; }
    .class-controls {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 8px;
    }
    .warning { color: #e1b94b; }
    @media (max-width: 900px) {
      main { padding: 12px; }
      header { align-items: flex-start; flex-direction: column; }
      .dashboard-grid, .teleop-main, .workflow-grid { grid-template-columns: 1fr; }
      .camera, .map-panel { min-height: 280px; }
      .metric-grid, .bed-grid { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>
  <main>
    <header>
      <div>
        <h1>SimBioSys Greenhouse Robot</h1>
        <p>MIRTE Master dashboard</p>
      </div>
      <div class="topline">
        <button id="safety-toggle" class="safety">STOP</button>
        <span class="status-pill" id="connection">Starting</span>
        <span class="status-pill" id="battery">Battery --</span>
        <button id="to-teleop">Teleop / Camera</button>
        <button id="to-dashboard" style="display:none">Dashboard</button>
      </div>
    </header>

    <section id="dashboard-page" class="page active">
      <div class="dashboard-grid">
        <section class="panel">
          <div class="panel-body">
            <div class="topline">
              <h2>Map Position</h2>
              <span id="dashboard-map-time" class="status-pill">Last SLAM map: not received yet</span>
            </div>
            <div id="dashboard-map-placeholder" class="placeholder">Waiting for SLAM map</div>
            <canvas id="dashboard-map-canvas" width="900" height="520" style="display:none"></canvas>
            <div class="control-row"><span id="selected-target">No map position selected</span><button id="navigate-target" disabled>Navigate</button></div>
            <p id="navigation-message">Navigation backend unavailable</p>
          </div>
        </section>
        <aside class="panel">
          <div class="panel-body">
            <h2>Task Mode</h2>
            <div class="control-row">
              <span>Current task</span>
              <select id="task-mode" disabled>
                <option value="">Unavailable</option>
                <option value="harvest">Harvest</option>
                <option value="scanning">Scanning</option>
              </select>
            </div>
            <p id="task-message">Task mode backend unavailable</p>
            <h2>Scan Summary</h2>
            <div class="metric-grid">
              <div class="metric"><strong id="bed-count">0</strong>real beds observed</div>
              <div class="metric"><strong id="flower-count">0</strong>real flower records</div>
              <div class="metric"><strong id="last-scan">not available</strong>last scan</div>
              <div class="metric"><strong id="next-action">Waiting for real data</strong>status</div>
            </div>
          </div>
        </aside>
      </div>
      <section class="panel">
        <div class="panel-body">
          <h2>Bed Overview</h2>
          <div class="bed-grid" id="bed-overview"></div>
        </div>
      </section>
      <section class="panel">
        <div class="panel-body" id="plant-records">
          <h2>Plant Records</h2>
          <p>No plant records received</p>
        </div>
      </section>
    </section>

    <section id="teleop-page" class="page">
      <div class="teleop-main">
        <section class="panel">
          <div class="panel-body">
            <div class="topline">
              <h2>Camera</h2>
              <select id="camera-select">
                <option value="base">Base/front camera</option>
                <option value="arm">Arm camera</option>
              </select>
              <span class="status-pill" id="camera-state">waiting</span>
            </div>
            <div class="camera">
              <img id="camera-feed" alt="Live camera feed">
              <div id="camera-placeholder" class="placeholder" style="display:none">Waiting for camera frames</div>
            </div>
          </div>
        </section>
        <section class="panel">
          <div class="panel-body">
            <div class="topline">
              <h2>Live Mapping</h2>
              <span class="status-pill" id="map-state">Waiting for SLAM map</span>
            </div>
            <div class="map-panel">
              <div id="teleop-map-placeholder" class="placeholder">Waiting for SLAM map</div>
              <canvas id="teleop-map-canvas" width="900" height="520" style="display:none"></canvas>
            </div>
            <div class="control-row"><span id="map-meta">Last SLAM map: not received yet</span><strong id="pose-state">robot pose unavailable</strong></div>
          </div>
        </section>
      </div>

      <div class="workflow-grid">
        <section class="panel">
          <div class="panel-body">
            <div class="topline">
              <h2>Operations</h2>
              <button id="take-control">Take Control</button>
            </div>
            <p id="take-control-message">Manual-control backend unavailable</p>
            <div class="control-row">
              <span>Mode</span>
              <select id="operation-mode">
                <option value="robot">Robot Operations</option>
                <option value="arm">Arm Operations</option>
              </select>
            </div>
            <div id="robot-controls">
              <div class="control-row">
                <span>Speed</span>
                <select id="speed-mode">
                  <option value="slow">Slow 0.50 m/s</option>
                  <option value="normal" selected>Normal 0.75 m/s</option>
                  <option value="fast">Fast 1.00 m/s</option>
                </select>
              </div>
              <div class="pad">
                <button data-control="rotate-left" disabled>Rotate Left</button>
                <span class="empty"></span>
                <button data-control="rotate-right" disabled>Rotate Right</button>
                <button data-control="strafe-left" disabled>Strafe Left</button>
                <button data-control="forward" disabled>Forward</button>
                <button data-control="strafe-right" disabled>Strafe Right</button>
                <span class="empty"></span>
                <button data-control="backward" disabled>Back</button>
                <span class="empty"></span>
              </div>
              <div class="control-row"><span>Command</span><strong id="teleop-state">take control required</strong></div>
              <div class="control-row"><span>Keyboard</span><strong>Press Take Control first</strong></div>
            </div>
            <div id="arm-controls" style="display:none">
              <p id="arm-message">Arm pose backend unavailable</p>
              <div class="arm-pose-grid" id="arm-pose-buttons"></div>
              <button id="robot-ops-return">Return to Robot Operations</button>
            </div>
          </div>
        </section>

        <section class="panel">
          <div class="panel-body">
            <h2>Mapping Workflow</h2>
            <div class="topline">
              <button id="start-mapping" disabled>Start Mapping</button>
              <button id="done-mapping" disabled>Stop Mapping</button>
              <button id="retry-mapping" disabled>Retry</button>
              <button id="save-safe-map" disabled>Save Safe Map</button>
            </div>
            <p id="workflow-message">Mapping workflow unavailable</p>
            <div class="control-row"><span>Artifact candidates</span><strong id="candidate-state">No artifact candidates received</strong></div>
            <div class="control-row"><span>Map</span><strong id="mapping-map-time">Last SLAM map: not received yet</strong></div>
            <p id="candidate-parse-error" class="warning" style="display:none"></p>
            <div class="candidate-list" id="candidate-list"></div>
            <div id="candidate-detail" class="panel-body" style="padding:0">
              <h3>Artifact</h3>
              <p>No artifact candidates received</p>
            </div>
            <div class="class-controls">
              <button data-classification="wall" disabled>Wall</button>
              <button data-classification="bed" disabled>Bed</button>
              <button data-classification="obstacle" disabled>Obstacle</button>
              <button data-classification="false_scan" disabled>False Scan</button>
            </div>
            <p id="save-safe-map-warning" class="warning" style="display:none"></p>
            <div id="safe-map-result" class="panel-body" style="padding:0;display:none"></div>
          </div>
        </section>
      </div>
    </section>
  </main>

  <script>
    const state = {
      page: "dashboard",
      operationMode: "robot",
      activeControls: new Set(),
      activeSources: new Map(),
      pressedKeys: new Set(),
      latestMap: null,
      frozenMap: null,
      mappingActive: false,
      reviewMode: false,
      candidates: [],
      frozenCandidates: null,
      selectedCandidateId: null,
      classifications: {},
      safeMapResult: null,
      artifactCandidatesReceivedAt: null,
      artifactCandidatesParseError: "",
      lastMapRenderMs: 0,
      lastMapStamp: null,
      lastCandidateStamp: null,
      selectedMapTarget: null,
      manualControlActive: false,
      safetyPaused: false,
      topics: {}
    };
    const pages = {
      dashboard: document.getElementById("dashboard-page"),
      teleop: document.getElementById("teleop-page"),
    };

    function updateCameraStream() {
      const feed = document.getElementById("camera-feed");
      const camera = document.getElementById("camera-select").value;
      if (state.page !== "teleop") {
        feed.removeAttribute("src");
        return;
      }
      feed.src = `/stream.mjpg?camera=${camera}&t=${Date.now()}`;
    }

    function showPage(page) {
      state.page = page;
      pages.dashboard.classList.toggle("active", page === "dashboard");
      pages.teleop.classList.toggle("active", page === "teleop");
      document.getElementById("to-teleop").style.display = page === "dashboard" ? "" : "none";
      document.getElementById("to-dashboard").style.display = page === "teleop" ? "" : "none";
      if (page === "dashboard") stopTeleop();
      updateCameraStream();
      postJson("/api/view", {page}).catch(() => {});
    }
    document.getElementById("to-teleop").addEventListener("click", () => showPage("teleop"));
    document.getElementById("to-dashboard").addEventListener("click", () => showPage("dashboard"));

    function describeActiveControls() {
      if (state.safetyPaused) return "paused";
      if (!state.manualControlActive) return "take control required";
      if (!state.activeControls.size) return "idle";
      return Array.from(state.activeControls).sort().join(" + ");
    }
    function refreshActiveControls() {
      state.activeControls = new Set(state.activeSources.values());
    }
    async function sendTeleop() {
      const controls = state.safetyPaused || !state.manualControlActive || state.operationMode !== "robot" ? [] : Array.from(state.activeControls);
      document.getElementById("teleop-state").textContent = describeActiveControls();
      try {
        await fetch("/api/teleop", {
          method: "POST",
          headers: {"Content-Type": "application/json"},
          body: JSON.stringify({controls, speedMode: document.getElementById("speed-mode").value})
        });
      } catch (error) {
        document.getElementById("teleop-state").textContent = "offline";
      }
    }

    let repeatTimer = null;
    function clearRepeat() {
      if (repeatTimer) clearInterval(repeatTimer);
      repeatTimer = null;
    }
    function ensureRepeat() {
      if (!repeatTimer) repeatTimer = setInterval(sendTeleop, 100);
    }
    function stopTeleop() {
      clearRepeat();
      state.activeSources.clear();
      refreshActiveControls();
      state.pressedKeys.clear();
      document.querySelectorAll("button[data-control]").forEach((button) => button.classList.remove("active"));
      sendTeleop();
    }
    function startControl(event) {
      event.preventDefault();
      if (state.safetyPaused || !state.manualControlActive || state.operationMode !== "robot") return;
      const control = event.currentTarget.dataset.control;
      state.activeSources.set(`button:${control}`, control);
      refreshActiveControls();
      event.currentTarget.classList.add("active");
      sendTeleop();
      ensureRepeat();
    }
    function stopControl(event) {
      if (event) event.preventDefault();
      if (event && event.currentTarget) {
        const control = event.currentTarget.dataset.control;
        state.activeSources.delete(`button:${control}`);
        event.currentTarget.classList.remove("active");
      }
      refreshActiveControls();
      if (!state.activeControls.size) clearRepeat();
      sendTeleop();
    }
    document.querySelectorAll("button[data-control]").forEach((button) => {
      button.addEventListener("pointerdown", startControl);
      button.addEventListener("pointerup", stopControl);
      button.addEventListener("pointerleave", stopControl);
      button.addEventListener("pointercancel", stopControl);
    });
    document.getElementById("speed-mode").addEventListener("change", () => {
      if (state.page === "teleop") sendTeleop();
    });
    document.addEventListener("visibilitychange", () => {
      if (document.hidden) stopTeleop();
    });
    window.addEventListener("pagehide", () => {
      state.activeSources.clear();
      refreshActiveControls();
      navigator.sendBeacon("/api/view", JSON.stringify({page: "dashboard"}));
      navigator.sendBeacon("/api/teleop", JSON.stringify({controls: [], speedMode: "slow"}));
    });

    function isTypingTarget(target) {
      return target && (
        target.tagName === "INPUT" ||
        target.tagName === "TEXTAREA" ||
        target.tagName === "SELECT" ||
        target.isContentEditable
      );
    }
    const keyCommands = {
      KeyW: "forward",
      KeyS: "backward",
      KeyA: "strafe-left",
      KeyD: "strafe-right",
      KeyQ: "rotate-left",
      KeyE: "rotate-right",
    };
    const stopKeys = new Set(["Space", "Escape"]);
    document.addEventListener("keydown", (event) => {
      if (state.page !== "teleop" || state.operationMode !== "robot" || state.safetyPaused || !state.manualControlActive || isTypingTarget(event.target)) return;
      if (stopKeys.has(event.code)) {
        event.preventDefault();
        stopTeleop();
        return;
      }
      const command = keyCommands[event.code];
      if (!command) return;
      event.preventDefault();
      if (event.repeat || state.pressedKeys.has(event.code)) return;
      state.pressedKeys.add(event.code);
      state.activeSources.set(`key:${event.code}`, command);
      refreshActiveControls();
      sendTeleop();
      ensureRepeat();
    });
    document.addEventListener("keyup", (event) => {
      if (state.page !== "teleop") return;
      const command = keyCommands[event.code];
      if (!command) return;
      event.preventDefault();
      state.pressedKeys.delete(event.code);
      state.activeSources.delete(`key:${event.code}`);
      refreshActiveControls();
      if (!state.activeControls.size) clearRepeat();
      sendTeleop();
    });

    async function postJson(url, payload = {}) {
      const response = await fetch(url, {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify(payload)
      });
      return response.json();
    }

    document.getElementById("safety-toggle").addEventListener("click", async () => {
      const data = await postJson("/api/safety/toggle");
      state.safetyPaused = data.paused;
      state.manualControlActive = data.manualControlActive;
      if (state.safetyPaused || !state.manualControlActive) stopTeleop();
      updateSafetyButton();
    });
    function updateSafetyButton() {
      const button = document.getElementById("safety-toggle");
      button.textContent = state.safetyPaused ? "START" : "STOP";
      button.classList.toggle("paused", state.safetyPaused);
      const movementDisabled = state.safetyPaused || !state.manualControlActive || state.operationMode !== "robot";
      document.querySelectorAll("button[data-control]").forEach((control) => control.disabled = movementDisabled);
      document.getElementById("navigate-target").disabled = state.safetyPaused || !state.selectedMapTarget || !state.navigationAvailable;
      const takeButton = document.getElementById("take-control");
      takeButton.textContent = state.manualControlActive ? "Release Control" : "Take Control";
      takeButton.disabled = state.safetyPaused;
      const keyboardText = document.querySelector("#robot-controls .control-row:last-child strong");
      if (keyboardText) keyboardText.textContent = state.manualControlActive ? "W/S A/D Q/E" : "Press Take Control first";
    }

    function mapToCanvas(map, canvas) {
      const cell = Math.max(1, Math.floor(Math.min(canvas.width / map.width, canvas.height / map.height)));
      return {
        cell,
        x0: Math.floor((canvas.width - map.width * cell) / 2),
        y0: Math.floor((canvas.height - map.height * cell) / 2),
      };
    }
    function worldToCanvas(map, canvas, x, y) {
      const geom = mapToCanvas(map, canvas);
      const mx = (x - map.origin.x) / map.resolution;
      const my = (y - map.origin.y) / map.resolution;
      return {
        x: geom.x0 + mx * geom.cell,
        y: geom.y0 + (map.height - 1 - my) * geom.cell,
      };
    }
    function canvasToWorld(map, canvas, px, py) {
      const geom = mapToCanvas(map, canvas);
      const mx = (px - geom.x0) / geom.cell;
      const my = map.height - 1 - (py - geom.y0) / geom.cell;
      if (mx < 0 || my < 0 || mx >= map.width || my >= map.height) return null;
      return {
        x: map.origin.x + mx * map.resolution,
        y: map.origin.y + my * map.resolution,
      };
    }
    function pointToXY(point) {
      if (!point) return null;
      if (Array.isArray(point) && point.length >= 2) {
        const x = Number(point[0]);
        const y = Number(point[1]);
        return Number.isFinite(x) && Number.isFinite(y) ? {x, y} : null;
      }
      const source = point.position && typeof point.position === "object" ? point.position : point;
      if (source.x == null || source.y == null) return null;
      const x = Number(source.x);
      const y = Number(source.y);
      return Number.isFinite(x) && Number.isFinite(y) ? {x, y} : null;
    }
    function pointsToXY(points) {
      if (!Array.isArray(points)) return [];
      return points.map(pointToXY).filter(Boolean);
    }
    function sizeToWH(size) {
      if (!size) return null;
      if (Array.isArray(size) && size.length >= 2) {
        const width = Number(size[0]);
        const height = Number(size[1]);
        return Number.isFinite(width) && Number.isFinite(height) ? {width, height} : null;
      }
      if ((size.width ?? size.w ?? size.x) == null || (size.height ?? size.h ?? size.y) == null) return null;
      const width = Number(size.width ?? size.w ?? size.x);
      const height = Number(size.height ?? size.h ?? size.y);
      return Number.isFinite(width) && Number.isFinite(height) ? {width, height} : null;
    }
    function candidateSet() {
      return state.reviewMode && state.frozenCandidates ? state.frozenCandidates : state.candidates;
    }
    function candidateLabel(candidate) {
      return state.classifications[candidate.id] || "unclassified";
    }
    function candidateColor(candidate) {
      const label = candidateLabel(candidate);
      if (label === "wall") return "#d8e0dc";
      if (label === "bed") return "#69c174";
      if (label === "obstacle") return "#d96b5f";
      if (label === "false_scan") return "#e1b94b";
      return "#8fa19a";
    }
    function safeMapReviewResult() {
      const objects = [];
      const removed = [];
      (state.frozenCandidates || []).forEach((candidate) => {
        const classification = state.classifications[candidate.id] || "unclassified";
        if (classification === "unclassified" || classification === "false_scan") {
          removed.push(candidate.id);
          return;
        }
        objects.push({
          id: candidate.id,
          class: classification,
          geometryType: candidate.geometryType || "unknown",
          geometry: candidate.raw && candidate.raw.geometry ? candidate.raw.geometry : {},
          source: candidate.source || ""
        });
      });
      return {objects, removed_false_scans: removed};
    }
    function renderSafeMapResult() {
      const container = document.getElementById("safe-map-result");
      if (!state.safeMapResult) {
        container.style.display = "none";
        container.innerHTML = "";
        return;
      }
      container.style.display = "";
      container.innerHTML = `
        <h3>Safe Map Save Review</h3>
        <p>Objects kept: ${state.safeMapResult.objects.length}</p>
        <p>Removed false scans: ${state.safeMapResult.removed_false_scans.join(", ") || "none"}</p>
      `;
    }
    function candidateBounds(points) {
      if (!points.length) return null;
      const xs = points.map((point) => point.x);
      const ys = points.map((point) => point.y);
      const minX = Math.min(...xs);
      const maxX = Math.max(...xs);
      const minY = Math.min(...ys);
      const maxY = Math.max(...ys);
      return {
        pose: {x: minX, y: minY},
        size: {width: maxX - minX, height: maxY - minY},
      };
    }
    function drawWorldPolyline(ctx, map, canvas, points, closePath, fill) {
      const canvasPoints = points.map((point) => worldToCanvas(map, canvas, point.x, point.y));
      if (!canvasPoints.length) return false;
      ctx.beginPath();
      ctx.moveTo(canvasPoints[0].x, canvasPoints[0].y);
      canvasPoints.slice(1).forEach((point) => ctx.lineTo(point.x, point.y));
      if (closePath && canvasPoints.length > 2) ctx.closePath();
      if (fill && canvasPoints.length > 2) ctx.fill();
      ctx.stroke();
      return true;
    }
    function drawCandidateOverlay(ctx, map, canvas, candidate, selected) {
      const classification = state.classifications[candidate.id];
      if (classification === "false_scan") return "";
      const color = candidateColor(candidate);
      const type = String(candidate.geometryType || "unknown").toLowerCase();
      const rawGeometry = candidate.raw && candidate.raw.geometry && typeof candidate.raw.geometry === "object" ? candidate.raw.geometry : {};
      const points = pointsToXY(candidate.points && candidate.points.length ? candidate.points : rawGeometry.points);
      const pose = pointToXY(candidate.pose || rawGeometry.pose);
      const size = sizeToWH(candidate.size || rawGeometry.size);
      const center = pointToXY(candidate.center || rawGeometry.center);
      const radius = Number(candidate.radius ?? rawGeometry.radius);
      const start = pointToXY(candidate.start || rawGeometry.start);
      const end = pointToXY(candidate.end || rawGeometry.end);

      ctx.save();
      ctx.strokeStyle = color;
      ctx.fillStyle = `${color}33`;
      ctx.lineWidth = selected ? 4 : 2;
      ctx.setLineDash(selected ? [] : [8, 5]);

      let rendered = false;
      if ((classification === "bed" || type.includes("rect")) && points.length >= 2 && (!pose || !size)) {
        const bounds = candidateBounds(points);
        if (bounds) {
          const bottomLeft = bounds.pose;
          const topRight = {x: bounds.pose.x + bounds.size.width, y: bounds.pose.y + bounds.size.height};
          rendered = drawWorldPolyline(ctx, map, canvas, [
            bottomLeft,
            {x: topRight.x, y: bottomLeft.y},
            topRight,
            {x: bottomLeft.x, y: topRight.y},
          ], true, true);
        }
      } else if ((type.includes("rect") || classification === "bed") && pose && size) {
        const corners = [
          pose,
          {x: pose.x + size.width, y: pose.y},
          {x: pose.x + size.width, y: pose.y + size.height},
          {x: pose.x, y: pose.y + size.height},
        ];
        rendered = drawWorldPolyline(ctx, map, canvas, corners, true, true);
      } else if ((type.includes("polygon") || type === "walls" || type === "wall") && points.length >= 3) {
        rendered = drawWorldPolyline(ctx, map, canvas, points, true, true);
      } else if ((type.includes("line") || type === "walls" || type === "wall") && points.length >= 2) {
        rendered = drawWorldPolyline(ctx, map, canvas, points, false, false);
      } else if ((type.includes("line") || type === "segment") && start && end) {
        rendered = drawWorldPolyline(ctx, map, canvas, [start, end], false, false);
      } else if (type.includes("circle") && center && Number.isFinite(radius)) {
        const canvasCenter = worldToCanvas(map, canvas, center.x, center.y);
        const canvasRadius = Math.max(2, radius / map.resolution * mapToCanvas(map, canvas).cell);
        ctx.beginPath();
        ctx.arc(canvasCenter.x, canvasCenter.y, canvasRadius, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
        rendered = true;
      }

      const segments = rawGeometry.segments || candidate.segments || [];
      if (!rendered && Array.isArray(segments) && segments.length) {
        rendered = segments.some((segment) => {
          const segmentStart = pointToXY(segment.start || segment[0]);
          const segmentEnd = pointToXY(segment.end || segment[1]);
          return segmentStart && segmentEnd && drawWorldPolyline(ctx, map, canvas, [segmentStart, segmentEnd], false, false);
        });
      }

      if (rendered) {
        const labelPoint = points[0] || pose || center || start;
        if (labelPoint) {
          const label = worldToCanvas(map, canvas, labelPoint.x, labelPoint.y);
          ctx.setLineDash([]);
          ctx.fillStyle = color;
          ctx.font = "13px sans-serif";
          ctx.fillText(candidate.id || "candidate", label.x + 6, label.y - 6);
        }
      }
      ctx.restore();
      return rendered ? "" : "Artifact could not be rendered";
    }
    function drawCandidateOverlays(ctx, map, canvas) {
      if (!map) return;
      const warnings = [];
      candidateSet().forEach((candidate) => {
        const warning = drawCandidateOverlay(ctx, map, canvas, candidate, candidate.id === state.selectedCandidateId);
        if (warning) warnings.push(`${candidate.id || "candidate"}: ${warning}`);
      });
      state.renderWarnings = warnings;
    }
    function drawOccupancyGrid(canvasId, placeholderId, map, robotPose, showRobot, selectedTarget) {
      const canvas = document.getElementById(canvasId);
      const placeholder = document.getElementById(placeholderId);
      if (!map || !map.width || !map.height || !Array.isArray(map.data)) {
        canvas.style.display = "none";
        placeholder.style.display = "grid";
        return;
      }
      placeholder.style.display = "none";
      canvas.style.display = "block";
      const ctx = canvas.getContext("2d");
      const {cell, x0, y0} = mapToCanvas(map, canvas);
      ctx.fillStyle = "#0a0d0c";
      ctx.fillRect(0, 0, canvas.width, canvas.height);
      for (let y = 0; y < map.height; y++) {
        for (let x = 0; x < map.width; x++) {
          const value = map.data[y * map.width + x];
          ctx.fillStyle = value < 0 ? "#26312d" : value > 50 ? "#d8e0dc" : "#111815";
          ctx.fillRect(x0 + x * cell, y0 + (map.height - 1 - y) * cell, cell, cell);
        }
      }
      if (selectedTarget) {
        const target = worldToCanvas(map, canvas, selectedTarget.x, selectedTarget.y);
        ctx.strokeStyle = "#e1b94b";
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.arc(target.x, target.y, 10, 0, Math.PI * 2);
        ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(target.x - 14, target.y);
        ctx.lineTo(target.x + 14, target.y);
        ctx.moveTo(target.x, target.y - 14);
        ctx.lineTo(target.x, target.y + 14);
        ctx.stroke();
      }
      if (showRobot && robotPose && Number.isFinite(robotPose.x) && map.resolution) {
        const robot = worldToCanvas(map, canvas, robotPose.x, robotPose.y);
        ctx.fillStyle = "#66a8d9";
        ctx.beginPath();
        ctx.arc(robot.x, robot.y, 8, 0, Math.PI * 2);
        ctx.fill();
      }
      drawCandidateOverlays(ctx, map, canvas);
    }

    document.getElementById("dashboard-map-canvas").addEventListener("click", (event) => {
      if (!state.latestMap) return;
      const canvas = event.currentTarget;
      const rect = canvas.getBoundingClientRect();
      const target = canvasToWorld(
        state.latestMap,
        canvas,
        (event.clientX - rect.left) * (canvas.width / rect.width),
        (event.clientY - rect.top) * (canvas.height / rect.height)
      );
      if (!target) return;
      state.selectedMapTarget = target;
      document.getElementById("selected-target").textContent = `Selected ${target.x.toFixed(2)}, ${target.y.toFixed(2)}`;
      updateSafetyButton();
      renderMaps({map: state.latestMap, robotPose: null});
    });
    document.getElementById("navigate-target").addEventListener("click", async () => {
      if (!state.selectedMapTarget) return;
      const data = await postJson("/api/navigation/goal", state.selectedMapTarget);
      document.getElementById("navigation-message").textContent = data.message;
    });

    function renderMaps(data) {
      const map = state.reviewMode && state.frozenMap ? state.frozenMap : data.map;
      drawOccupancyGrid("dashboard-map-canvas", "dashboard-map-placeholder", map, null, false, state.selectedMapTarget);
      drawOccupancyGrid("teleop-map-canvas", "teleop-map-placeholder", map, data.robotPose, true, null);
    }
    function maybeRenderMaps(data) {
      const map = state.reviewMode && state.frozenMap ? state.frozenMap : (data.map || state.latestMap);
      const stamp = map ? map.receivedAt : null;
      const candidateStamp = state.reviewMode ? "review" : state.artifactCandidatesReceivedAt;
      const periodMs = Math.max(1, Number(data.topics.mapUpdatePeriodSec || 10)) * 1000;
      const now = Date.now();
      if (!map) {
        renderMaps(data);
        return;
      }
      if (
        candidateStamp !== state.lastCandidateStamp ||
        (stamp !== state.lastMapStamp && (now - state.lastMapRenderMs >= periodMs || !state.lastMapStamp))
      ) {
        state.lastMapStamp = stamp;
        state.lastCandidateStamp = candidateStamp;
        state.lastMapRenderMs = now;
        renderMaps(data);
      }
    }

    function timeSince(timestamp) {
      if (!timestamp) return "not available";
      const parsed = Date.parse(timestamp);
      if (!Number.isFinite(parsed)) return "not available";
      const seconds = Math.max(0, Math.round((Date.now() - parsed) / 1000));
      const minutes = Math.round(seconds / 60);
      if (minutes < 1) return "less than 1 min ago";
      if (minutes < 60) return `${minutes} min ago`;
      const hours = Math.round(minutes / 60);
      if (hours < 24) return `${hours} ${hours === 1 ? "hour" : "hours"} ago`;
      const days = Math.round(hours / 24);
      return `${days} ${days === 1 ? "day" : "days"} ago`;
    }
    function mapAgeText(map) {
      if (!map || !map.receivedAt) return "Last SLAM map: not received yet";
      const parsed = Date.parse(map.receivedAt);
      if (!Number.isFinite(parsed)) return "Last SLAM map: not received yet";
      const seconds = Math.max(0, Math.round((Date.now() - parsed) / 1000));
      return `Last SLAM map: ${seconds} sec ago`;
    }
    function renderReport(report, plants) {
      document.getElementById("bed-count").textContent = report.totalBeds;
      document.getElementById("flower-count").textContent = report.totalFlowers;
      document.getElementById("last-scan").textContent = timeSince(report.lastScanTime);
      document.getElementById("next-action").textContent = report.nextAction || "Waiting for real data";
      const records = document.getElementById("plant-records");
      records.innerHTML = "<h2>Plant Records</h2>";
      if (!plants.length) {
        const empty = document.createElement("p");
        empty.textContent = "No plant records received";
        records.appendChild(empty);
        return;
      }
      plants.forEach((plant) => {
        const row = document.createElement("div");
        row.className = "control-row";
        const name = plant.flower_id || "unidentified plant";
        const parts = [];
        if (plant.bed_id) parts.push(`bed ${plant.bed_id}`);
        if (plant.health) parts.push(plant.health);
        if (plant.color) parts.push(plant.color);
        if (plant.growth_stage) parts.push(plant.growth_stage);
        if (plant.bug_detected) parts.push("bugs detected");
        if (plant.ready_for_harvest) parts.push("ready");
        if (plant.confidence != null) parts.push(`${Math.round(Number(plant.confidence) * 100)}% confidence`);
        if (plant.last_scan_time) parts.push(`last scan ${timeSince(plant.last_scan_time)}`);
        row.innerHTML = `<span>${name}</span><strong>${parts.join(", ") || "real record received"}</strong>`;
        records.appendChild(row);
      });
    }
    function renderBeds(data) {
      const container = document.getElementById("bed-overview");
      container.innerHTML = "";
      const beds = data.beds || [];
      if (!beds.length) {
        const empty = document.createElement("p");
        empty.textContent = "Waiting for bed telemetry";
        container.appendChild(empty);
        return;
      }
      beds.forEach((bed) => {
        const card = document.createElement("article");
        card.className = `bed-card ${bed.available ? "available" : "unavailable"}`;
        card.innerHTML = `
          <h3>Bed ${bed.bed_id}</h3>
          <p><strong>${bed.co2 == null ? "unavailable" : bed.co2}</strong>CO2</p>
          <p><strong>${bed.humidity == null ? "unavailable" : bed.humidity}</strong>humidity</p>
          <p><strong>${bed.bugs_detected == null ? "unavailable" : (bed.bugs_detected ? "yes" : "no")}</strong>bugs detected</p>
        `;
        container.appendChild(card);
      });
    }
    function renderCandidates() {
      const list = document.getElementById("candidate-list");
      list.innerHTML = "";
      const detail = document.getElementById("candidate-detail");
      const candidates = candidateSet();
      const parseError = document.getElementById("candidate-parse-error");
      const renderWarnings = state.renderWarnings || [];
      document.getElementById("mapping-map-time").textContent = mapAgeText(state.latestMap);
      parseError.style.display = state.artifactCandidatesParseError || renderWarnings.length ? "" : "none";
      parseError.textContent = [state.artifactCandidatesParseError, ...renderWarnings].filter(Boolean).join(" | ");
      if (!candidates.length) {
        document.getElementById("candidate-state").textContent = "No artifact candidates received";
        detail.innerHTML = "<h3>Artifact</h3><p>No artifact candidates received</p>";
        document.querySelectorAll("button[data-classification]").forEach((button) => button.disabled = true);
        return;
      }
      document.getElementById("candidate-state").textContent = `${candidates.length} artifacts received`;
      candidates.forEach((candidate) => {
        const button = document.createElement("button");
        const label = document.createElement("span");
        label.textContent = candidate.id || "candidate";
        const meta = document.createElement("small");
        meta.textContent = `classification: ${candidateLabel(candidate)}`;
        button.appendChild(label);
        button.appendChild(meta);
        button.classList.toggle("active", candidate.id === state.selectedCandidateId);
        button.addEventListener("click", () => {
          state.selectedCandidateId = candidate.id;
          renderMaps({map: state.reviewMode && state.frozenMap ? state.frozenMap : state.latestMap, robotPose: null});
          renderCandidates();
        });
        list.appendChild(button);
      });
      const selected = candidates.find((candidate) => candidate.id === state.selectedCandidateId);
      if (!selected) {
        detail.innerHTML = "<h3>Artifact</h3><p>Select an artifact to classify.</p>";
        document.querySelectorAll("button[data-classification]").forEach((button) => button.disabled = true);
        return;
      }
      detail.innerHTML = `
        <h3>Candidate ${selected.id}</h3>
        <p>Classification: ${candidateLabel(selected)}</p>
      `;
      document.querySelectorAll("button[data-classification]").forEach((button) => button.disabled = false);
      renderSafeMapResult();
    }

    document.querySelectorAll("button[data-classification]").forEach((button) => {
      button.addEventListener("click", () => {
        if (!state.selectedCandidateId) return;
        if (button.dataset.classification === "unclassified") delete state.classifications[state.selectedCandidateId];
        else state.classifications[state.selectedCandidateId] = button.dataset.classification;
        state.safeMapResult = null;
        document.getElementById("save-safe-map-warning").style.display = "none";
        renderMaps({map: state.reviewMode && state.frozenMap ? state.frozenMap : state.latestMap, robotPose: null});
        renderCandidates();
      });
    });
    document.getElementById("start-mapping").addEventListener("click", async () => {
      const data = await postJson("/api/mapping/start");
      document.getElementById("workflow-message").textContent = data.message;
    });
    document.getElementById("done-mapping").addEventListener("click", async () => {
      if (!state.latestMap) return;
      const data = await postJson("/api/mapping/done");
      state.frozenMap = JSON.parse(JSON.stringify(state.latestMap));
      state.frozenCandidates = JSON.parse(JSON.stringify(state.candidates || []));
      state.reviewMode = true;
      state.classifications = {};
      state.safeMapResult = null;
      document.getElementById("save-safe-map-warning").style.display = "none";
      if (!state.frozenCandidates.some((candidate) => candidate.id === state.selectedCandidateId)) state.selectedCandidateId = null;
      document.getElementById("workflow-message").textContent = data.message || (state.frozenCandidates.length ? "Reviewing frozen real artifact candidates" : "No artifact candidates received");
      renderMaps({map: state.frozenMap, robotPose: null});
      renderCandidates();
    });
    document.getElementById("retry-mapping").addEventListener("click", () => {
      if (!state.frozenMap) return;
      state.reviewMode = false;
      state.frozenCandidates = null;
      state.classifications = {};
      state.safeMapResult = null;
      state.selectedCandidateId = null;
      document.getElementById("save-safe-map-warning").style.display = "none";
      renderMaps({map: state.latestMap, robotPose: null});
      renderCandidates();
    });
    document.getElementById("save-safe-map").addEventListener("click", async () => {
      const reviewResult = safeMapReviewResult();
      const unclassifiedCount = (state.frozenCandidates || []).filter((candidate) => !state.classifications[candidate.id]).length;
      const data = await postJson("/api/mapping/save_safe_map", {
        classifications: state.classifications,
        candidates: state.frozenCandidates || [],
        safeMapReview: reviewResult
      });
      state.safeMapResult = data.safeMapReview || reviewResult;
      const warning = document.getElementById("save-safe-map-warning");
      warning.style.display = unclassifiedCount ? "" : "none";
      warning.textContent = unclassifiedCount ? "Unclassified artifacts will be saved as false_scan." : "";
      document.getElementById("workflow-message").textContent = unclassifiedCount ? `${data.message}. Unclassified artifacts will be saved as false_scan.` : data.message;
      renderSafeMapResult();
    });

    function saveDisabledReason(data) {
      if (state.safetyPaused) return "START required before safe-map save";
      if (!data.mapping.saveSafeMapAvailable) return "Save safe map unavailable";
      if (!state.reviewMode || !state.frozenMap) return "No reviewed map yet";
      if (!state.frozenCandidates || !state.frozenCandidates.length) return "No artifact candidates received";
      return "";
    }

    document.getElementById("task-mode").addEventListener("change", async (event) => {
      if (!event.target.value) return;
      const data = await postJson("/api/task_mode", {mode: event.target.value});
      document.getElementById("task-message").textContent = data.message;
    });
    document.getElementById("operation-mode").addEventListener("change", (event) => {
      state.operationMode = event.target.value;
      stopTeleop();
      document.getElementById("robot-controls").style.display = state.operationMode === "robot" ? "" : "none";
      document.getElementById("arm-controls").style.display = state.operationMode === "arm" ? "" : "none";
      updateSafetyButton();
    });
    document.getElementById("robot-ops-return").addEventListener("click", () => {
      document.getElementById("operation-mode").value = "robot";
      document.getElementById("operation-mode").dispatchEvent(new Event("change"));
    });
    document.getElementById("take-control").addEventListener("click", async () => {
      const data = await postJson("/api/take_control");
      state.manualControlActive = data.manualControlActive;
      document.getElementById("take-control-message").textContent = data.message;
      if (!state.manualControlActive) stopTeleop();
      updateSafetyButton();
    });
    document.getElementById("camera-select").addEventListener("change", async (event) => {
      const data = await postJson("/api/camera/select", {camera: event.target.value});
      document.getElementById("camera-state").textContent = data.message;
      updateCameraStream();
    });
    function renderArmButtons(data) {
      const grid = document.getElementById("arm-pose-buttons");
      grid.innerHTML = "";
      (data.arm.availablePoses || []).forEach((pose) => {
        const button = document.createElement("button");
        button.textContent = pose.replaceAll("_", " ");
        button.disabled = !data.arm.poseBackendAvailable || state.safetyPaused;
        button.addEventListener("click", async () => {
          const result = await postJson("/api/arm/pose", {pose});
          document.getElementById("arm-message").textContent = result.message;
        });
        grid.appendChild(button);
      });
      document.getElementById("arm-message").textContent = data.arm.poseBackendAvailable ? "Arm pose backend available" : "Arm pose backend unavailable";
    }

    async function refreshStatus() {
      try {
        const response = await fetch("/api/status");
        const data = await response.json();
        state.topics = data.topics;
        state.candidates = data.artifactCandidates || [];
        state.artifactCandidatesReceivedAt = data.artifactCandidatesReceivedAt;
        state.artifactCandidatesParseError = data.artifactCandidatesParseError || "";
        if (!candidateSet().some((candidate) => candidate.id === state.selectedCandidateId)) state.selectedCandidateId = null;
        state.safetyPaused = data.safetyPaused;
        state.manualControlActive = data.manualControlActive;
        state.navigationAvailable = data.navigation.available;
        if (data.map) state.latestMap = data.map;
        document.getElementById("connection").textContent = data.rosConnected ? "ROS node running" : "ROS waiting";
        document.getElementById("battery").textContent = data.batteryPercent == null ? "Battery --" : `Battery ${Math.round(data.batteryPercent)}%`;
        document.getElementById("camera-state").textContent = data.cameraReady ? `${data.camera.current} online` : `${data.camera.current} waiting`;
        document.getElementById("camera-feed").style.display = data.cameraReady ? "block" : "none";
        document.getElementById("camera-placeholder").style.display = data.cameraReady ? "none" : "grid";
        document.getElementById("camera-select").value = data.camera.current;
        document.querySelector('#camera-select option[value="arm"]').disabled = !data.camera.armAvailable;
        const currentMap = data.map || state.latestMap;
        const mapText = currentMap ? `Map ${currentMap.width}x${currentMap.height} at ${currentMap.resolution} m/cell` : "Waiting for SLAM map";
        document.getElementById("map-state").textContent = mapText;
        document.getElementById("map-meta").textContent = mapAgeText(state.latestMap);
        document.getElementById("dashboard-map-time").textContent = mapAgeText(state.latestMap);
        document.getElementById("pose-state").textContent = data.robotPose ? "robot pose live" : "robot pose unavailable";
        maybeRenderMaps(data);
        renderReport(data.report, data.plants);
        renderBeds(data);
        document.getElementById("task-mode").disabled = state.safetyPaused || !data.taskMode.available;
        if (data.taskMode.available) {
          document.getElementById("task-message").textContent = data.taskMode.current ? `Current backend state: ${data.taskMode.current}` : "Task mode backend available";
        }
        document.getElementById("navigate-target").disabled = state.safetyPaused || !state.selectedMapTarget || !data.navigation.available;
        document.getElementById("navigation-message").textContent = data.navigation.available ? "Select a map position to navigate" : "Navigation backend unavailable";
        document.getElementById("take-control-message").textContent = state.manualControlActive ? "Manual teleop control active" : data.takeControl.message;
        const startReason = state.safetyPaused ? "START required before mapping commands" : (!data.mapping.startAvailable ? "Start mapping service unavailable" : "");
        const doneReason = !data.map ? "No map received yet" : (!data.mapping.doneBackendAvailable ? "No finalize backend; UI will freeze latest received map" : "");
        const saveReason = saveDisabledReason(data);
        document.getElementById("start-mapping").disabled = Boolean(startReason);
        document.getElementById("start-mapping").title = startReason;
        document.getElementById("done-mapping").disabled = !data.map;
        document.getElementById("done-mapping").title = doneReason;
        document.getElementById("retry-mapping").disabled = !state.frozenMap;
        document.getElementById("save-safe-map").disabled = Boolean(saveReason);
        document.getElementById("save-safe-map").title = saveReason;
        if (startReason && !state.reviewMode) document.getElementById("workflow-message").textContent = startReason;
        else if (saveReason && state.reviewMode) document.getElementById("workflow-message").textContent = saveReason;
        renderCandidates();
        renderArmButtons(data);
        updateSafetyButton();
      } catch (error) {
        document.getElementById("connection").textContent = "UI server offline";
      }
    }

    setInterval(refreshStatus, 1000);
    refreshStatus();
  </script>
</body>
</html>
"""


def load_config() -> dict:
    config = copy.deepcopy(DEFAULT_CONFIG)
    if CONFIG_PATH.exists():
        try:
            loaded = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
            config.update({key: value for key, value in loaded.items() if key != "topics"})
            config["topics"].update(loaded.get("topics", {}))
        except (OSError, json.JSONDecodeError):
            pass
    return config


def env_web_port() -> int:
    value = os.getenv("SIMBIOSYS_UI_PORT")
    if not value:
        return DEFAULT_WEB_PORT
    try:
        port = int(value)
    except ValueError:
        return DEFAULT_WEB_PORT
    return port if 1 <= port <= 65535 else DEFAULT_WEB_PORT


def request_hostname(host_header: str) -> str:
    if not host_header:
        return "localhost"
    host = host_header.rsplit("@", 1)[-1].strip()
    if host.startswith("[") and "]" in host:
        return host[1 : host.index("]")] or "localhost"
    return host.rsplit(":", 1)[0] or "localhost"


def dynamic_rosbridge_url(configured_url: str, host_header: str) -> str:
    parsed = urlparse(configured_url or "")
    if parsed.scheme not in {"ws", "wss"} or not parsed.hostname:
        return configured_url
    if parsed.hostname not in {"localhost", "127.0.0.1", "::1"}:
        return configured_url
    hostname = request_hostname(host_header)
    port = parsed.port or 9090
    netloc = f"[{hostname}]:{port}" if ":" in hostname else f"{hostname}:{port}"
    return urlunparse((parsed.scheme, netloc, parsed.path or "", parsed.params, parsed.query, parsed.fragment))


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _clean_text(value, default: str = "") -> str:
    if value is None:
        return default
    return str(value).strip()


def _first_present(*values):
    for value in values:
        if value is not None:
            return value
    return None


def _normalize_artifact_candidate(candidate, index: int) -> tuple[dict | None, str]:
    if not isinstance(candidate, dict):
        return None, f"candidate {index} is not an object"
    geometry = candidate.get("geometry")
    if not isinstance(geometry, dict):
        geometry = {}
    candidate_id = _clean_text(candidate.get("id"), f"candidate_{index + 1}")
    geometry_type = _clean_text(
        _first_present(candidate.get("geometry_type"), geometry.get("type")),
        "unknown",
    )
    pose = _first_present(candidate.get("pose"), geometry.get("pose"))
    size = _first_present(candidate.get("size"), geometry.get("size"))
    if pose is None and ("x" in geometry or "y" in geometry):
        pose = {"x": geometry.get("x"), "y": geometry.get("y")}
    if size is None and ("width" in geometry or "height" in geometry):
        size = {"width": geometry.get("width"), "height": geometry.get("height")}
    normalized = {
        "id": candidate_id,
        "candidateType": _clean_text(candidate.get("candidate_type"), "unclassified"),
        "geometryType": geometry_type or "unknown",
        "points": _first_present(candidate.get("points"), geometry.get("points"), []),
        "pose": pose,
        "size": size,
        "radius": _first_present(candidate.get("radius"), geometry.get("radius")),
        "center": _first_present(candidate.get("center"), geometry.get("center")),
        "start": _first_present(candidate.get("start"), geometry.get("start")),
        "end": _first_present(candidate.get("end"), geometry.get("end")),
        "segments": _first_present(candidate.get("segments"), geometry.get("segments"), []),
        "source": _clean_text(candidate.get("source")),
        "raw": copy.deepcopy(candidate),
    }
    if not normalized["points"] and normalized["geometryType"] == "unknown":
        return normalized, f"{candidate_id}: unknown geometry"
    return normalized, ""


class UiNode(Node):
    """Embedded operator web UI for dashboard, camera, mapping, and teleop."""

    def __init__(self) -> None:
        super().__init__("ui_node")
        self._config = load_config()
        self._topics = self._config["topics"]

        self.declare_parameter("web_host", os.getenv("SIMBIOSYS_UI_HOST", DEFAULT_WEB_HOST))
        self.declare_parameter("web_port", env_web_port())
        self.declare_parameter("cmd_vel_topic", self._topics["cmdVel"])
        self.declare_parameter("teleop_queue_depth", DEFAULT_TELEOP_QUEUE_DEPTH)
        self.declare_parameter("image_topic", self._topics.get("baseCameraRaw") or "")
        self.declare_parameter("compressed_image_topic", self._topics["baseCameraCompressed"])
        self.declare_parameter("camera_max_fps", DEFAULT_CAMERA_MAX_FPS)
        self.declare_parameter("live_map_topic", self._topics["liveMap"])
        self._apply_topic_parameters()

        self._web_host = self.get_parameter("web_host").value
        self._web_port = int(self.get_parameter("web_port").value)
        self._camera_max_fps = max(
            0.1,
            min(float(self.get_parameter("camera_max_fps").value), DEFAULT_CAMERA_MAX_FPS),
        )
        self._camera_frame_period = 1.0 / self._camera_max_fps
        self._frame_lock = threading.Lock()
        self._frames = {"base": None, "arm": None}
        self._frame_versions = {"base": 0, "arm": 0}
        self._last_frame_update_time = {"base": 0.0, "arm": 0.0}
        self._shutting_down = threading.Event()
        self._active_page = "dashboard"
        self._camera_subscriptions = {}
        self._selected_camera = "base"
        self._last_command_time = 0.0
        self._active_controls = set()
        self._speed_mode = "normal"
        self._safety_paused = False
        self._manual_control_active = False
        self._plants = {}
        self._flower_counts = None
        self._external_report = None
        self._map = None
        self._last_status_map_sent_at = 0.0
        self._robot_pose = None
        self._battery_percent = None
        self._last_battery_update_at = 0.0
        self._bed_observations = {}
        self._mapping_status = None
        self._task_status = None
        self._artifact_candidates = []
        self._artifact_candidates_received_at = None
        self._artifact_candidates_parse_error = ""

        teleop_queue_depth = max(1, int(self.get_parameter("teleop_queue_depth").value))
        self._cmd_vel_publisher = self.create_publisher(Twist, self._topics["cmdVel"], teleop_queue_depth)
        self._sync_camera_subscriptions()
        if self._topics.get("liveMap"):
            self.create_subscription(OccupancyGrid, self._topics["liveMap"], self._on_map, 10)
        self.create_subscription(Odometry, self._topics["odom"], self._on_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, self._topics["amclPose"], self._on_amcl_pose, 10)
        self.create_subscription(String, self._topics["plantHealth"], self._on_plant_health, 10)
        self.create_subscription(String, self._topics["plantHealthReport"], self._on_plant_health_report, 10)
        if self._topics.get("flowerCounts"):
            self.create_subscription(String, self._topics["flowerCounts"], self._on_flower_counts, 10)
        self.create_subscription(BatteryState, self._topics["battery"], self._on_battery, 10)
        if PlantHealth is not None:
            self.create_subscription(PlantHealth, self._topics["typedPlantHealth"], self._on_typed_plant_health, 10)
        if BedObservation is not None:
            self.create_subscription(BedObservation, self._topics["bedObservation"], self._on_bed_observation, 10)
        if MappingStatus is not None:
            self.create_subscription(MappingStatus, self._topics["mappingStatus"], self._on_mapping_status, 10)
        if TaskStatus is not None:
            self.create_subscription(TaskStatus, self._topics["taskStatus"], self._on_task_status, 10)
        if self._topics.get("artifactCandidates"):
            self.create_subscription(String, self._topics["artifactCandidates"], self._on_artifact_candidates, 10)

        self._task_mode_client = self.create_client(SetRobotMode, self._topics["setTaskMode"]) if SetRobotMode is not None else None
        self._arm_pose_client = self.create_client(SendNamedArmPose, self._topics["sendNamedArmPose"]) if SendNamedArmPose is not None else None
        self._behavior_client = ActionClient(self, ExecuteBehavior, self._topics["executeBehavior"]) if ExecuteBehavior is not None else None
        self._start_mapping_client = self.create_client(Trigger, self._topics["startMapping"]) if self._topics.get("startMapping") else None
        self._done_mapping_client = self.create_client(Trigger, self._topics["doneMapping"]) if self._topics.get("doneMapping") else None
        self._save_safe_map_client = self.create_client(Trigger, self._topics["saveSafeMap"]) if self._topics.get("saveSafeMap") else None

        self.create_timer(0.1, self._on_teleop_timer)
        self._http_server = UiHttpServer((self._web_host, self._web_port), self._make_handler())
        self._http_thread = threading.Thread(target=self._http_server.serve_forever, daemon=True)
        self._http_thread.start()

        self._log_startup_urls()
        self.get_logger().info(f"Publishing Twist to {self._topics['cmdVel']}; using real ROS/project data only")

    def _apply_topic_parameters(self) -> None:
        configured_raw_image = self.get_parameter("image_topic").get_parameter_value().string_value or None
        configured_compressed_image = self.get_parameter("compressed_image_topic").get_parameter_value().string_value
        if configured_raw_image and configured_raw_image != self._topics["baseCameraRaw"] and configured_compressed_image == self._topics["baseCameraCompressed"]:
            configured_compressed_image = f"{configured_raw_image}/compressed"
        self._topics["cmdVel"] = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        self._topics["baseCameraRaw"] = None
        self._topics["baseCameraCompressed"] = configured_compressed_image
        self._topics["liveMap"] = self.get_parameter("live_map_topic").get_parameter_value().string_value

    def _desired_camera_subscription(self):
        if self._active_page != "teleop":
            return None
        topic_key = f"{self._selected_camera}CameraCompressed"
        topic = self._topics.get(topic_key)
        if not topic:
            return None
        return self._selected_camera, topic

    def _sync_camera_subscriptions(self) -> None:
        desired = self._desired_camera_subscription()
        desired_camera = desired[0] if desired else None
        for camera, subscription in list(self._camera_subscriptions.items()):
            if camera != desired_camera:
                self.destroy_subscription(subscription)
                del self._camera_subscriptions[camera]
                with self._frame_lock:
                    self._frames[camera] = None
                    self._frame_versions[camera] += 1
        if desired is None:
            return
        camera, topic = desired
        if camera in self._camera_subscriptions:
            return
        self._camera_subscriptions[camera] = self.create_subscription(
            CompressedImage,
            topic,
            lambda msg, camera=camera: self._on_compressed_image(camera, msg),
            1,
        )

    def destroy_node(self) -> bool:
        self._shutting_down.set()
        try:
            self._publish_twist(0.0, 0.0, 0.0)
        except Exception:
            pass
        try:
            self._http_server.shutdown()
            self._http_server.server_close()
        except Exception:
            pass
        return super().destroy_node()

    def _detect_lan_ip(self) -> str:
        if self._web_host not in ("", "0.0.0.0"):
            return self._web_host
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.connect(("8.8.8.8", 80))
                return sock.getsockname()[0]
        except OSError:
            pass
        try:
            for address in socket.gethostbyname_ex(socket.gethostname())[2]:
                if not address.startswith("127."):
                    return address
        except OSError:
            pass
        return ""

    def _log_startup_urls(self) -> None:
        self.get_logger().info(f"Web UI listening on {self._web_host}:{self._web_port}")
        self.get_logger().info(f"Local URL: http://localhost:{self._web_port}")
        if self._web_host in {"127.0.0.1", "localhost", "::1"}:
            self.get_logger().info("LAN access is disabled because the UI is bound to a loopback host.")
            return
        lan_ip = self._detect_lan_ip()
        if lan_ip:
            self.get_logger().info(f"LAN URL: http://{lan_ip}:{self._web_port} (same trusted local network only)")

    def _make_handler(self):
        node = self

        class RequestHandler(BaseHTTPRequestHandler):
            def log_message(self, _format, *_args):
                return

            def do_GET(self):
                parsed = urlparse(self.path)
                if parsed.path in ("/", "/index.html", "/teleop"):
                    self._send_html()
                elif parsed.path == "/api/status":
                    self._send_json(node.status_payload(self.headers.get("Host", "")))
                elif parsed.path == "/stream.mjpg":
                    self._send_stream()
                else:
                    self.send_error(HTTPStatus.NOT_FOUND)

            def do_POST(self):
                routes = {
                    "/api/teleop": node.set_teleop_command,
                    "/api/safety/toggle": node.toggle_safety,
                    "/api/navigation/goal": node.send_navigation_goal,
                    "/api/task_mode": node.set_task_mode,
                    "/api/take_control": node.take_control,
                    "/api/view": node.set_active_view,
                    "/api/camera/select": node.select_camera,
                    "/api/arm/pose": node.send_arm_pose,
                    "/api/mapping/start": node.start_mapping,
                    "/api/mapping/done": node.done_mapping,
                    "/api/mapping/save_safe_map": node.save_safe_map,
                }
                if self.path not in routes:
                    self.send_error(HTTPStatus.NOT_FOUND)
                    return
                length = int(self.headers.get("Content-Length", "0"))
                body = self.rfile.read(length)
                try:
                    payload = json.loads(body.decode("utf-8")) if body else {}
                except json.JSONDecodeError:
                    self.send_error(HTTPStatus.BAD_REQUEST, "Invalid JSON")
                    return
                result = routes[self.path](payload)
                status = HTTPStatus.OK if result.get("accepted", True) else HTTPStatus.BAD_REQUEST
                self._send_json(result, status=status)

            def _send_html(self):
                content = INDEX_HTML.encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(content)))
                self.end_headers()
                self.wfile.write(content)

            def _send_json(self, payload, status=HTTPStatus.OK):
                content = json.dumps(payload).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(content)))
                self.end_headers()
                self.wfile.write(content)

            def _send_stream(self):
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                last_version = None
                while not node._shutting_down.is_set():
                    frame_version = node.latest_frame_with_version()
                    if frame_version is None:
                        time.sleep(0.2)
                        continue
                    version, frame = frame_version
                    if version == last_version:
                        time.sleep(min(0.2, node._camera_frame_period))
                        continue
                    if frame is None:
                        time.sleep(0.2)
                        continue
                    try:
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii"))
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")
                        last_version = version
                        time.sleep(node._camera_frame_period)
                    except (BrokenPipeError, ConnectionResetError):
                        break

        return RequestHandler

    def latest_frame(self):
        if self._active_page != "teleop":
            return None
        with self._frame_lock:
            return self._frames.get(self._selected_camera)

    def latest_frame_with_version(self):
        if self._active_page != "teleop":
            return None
        with self._frame_lock:
            return self._frame_versions.get(self._selected_camera, 0), self._frames.get(self._selected_camera)

    def _service_ready(self, client) -> bool:
        return bool(client is not None and client.service_is_ready())

    def _action_ready(self, client) -> bool:
        return bool(client is not None and client.server_is_ready())

    def status_payload(self, host_header: str = "") -> dict:
        arm_available = bool(self._topics.get("armCameraCompressed"))
        return {
            "rosConnected": True,
            "rosbridgeUrl": dynamic_rosbridge_url(self._config["rosbridgeUrl"], host_header),
            "topics": self._topics,
            "safetyPaused": self._safety_paused,
            "manualControlActive": self._manual_control_active,
            "cameraReady": self.latest_frame() is not None,
            "camera": {
                "current": self._selected_camera,
                "armAvailable": arm_available,
            },
            "plants": list(self._plants.values()),
            "flowerCounts": copy.deepcopy(self._flower_counts),
            "beds": self._bed_payload(),
            "report": self._external_report or self._computed_report(),
            "map": self._status_map_payload(),
            "robotPose": self._robot_pose,
            "batteryPercent": self._battery_percent,
            "bedObservations": copy.deepcopy(self._bed_observations),
            "mappingStatus": copy.deepcopy(self._mapping_status),
            "taskMode": {
                "available": self._service_ready(self._task_mode_client),
                "current": self._task_status.get("current_state") if self._task_status else "",
            },
            "navigation": {"available": self._action_ready(self._behavior_client)},
            "takeControl": {
                "available": self._action_ready(self._behavior_client),
                "message": (
                    "Behavior TELEOP/IDLE backend available; no dedicated full motor stop backend found"
                    if self._action_ready(self._behavior_client)
                    else "Full motor/autonomy stop backend unavailable; UI STOP only sends zero velocity and disables UI commands"
                ),
            },
            "arm": {
                "poseBackendAvailable": self._service_ready(self._arm_pose_client),
                "availablePoses": list(ARM_POSES),
            },
            "mapping": {
                "startAvailable": self._service_ready(self._start_mapping_client),
                "doneAvailable": bool(self._map),
                "doneBackendAvailable": self._service_ready(self._done_mapping_client),
                "saveSafeMapAvailable": self._service_ready(self._save_safe_map_client),
            },
            "artifactCandidates": copy.deepcopy(self._artifact_candidates),
            "artifactCandidatesReceivedAt": self._artifact_candidates_received_at,
            "artifactCandidatesParseError": self._artifact_candidates_parse_error,
            "teleop": sorted(self._active_controls),
        }

    def _status_map_payload(self):
        if self._map is None:
            return None
        period_sec = max(1.0, float(self._topics.get("mapUpdatePeriodSec") or 10))
        now = time.monotonic()
        if now - self._last_status_map_sent_at < period_sec:
            return None
        self._last_status_map_sent_at = now
        return self._map

    def toggle_safety(self, _payload) -> dict:
        self._safety_paused = not self._safety_paused
        self._active_controls = set()
        self._publish_twist(0.0, 0.0, 0.0)
        if self._safety_paused:
            self._manual_control_active = False
            self._send_behavior_request(BehaviorType.IDLE if BehaviorType is not None else None)
        return {
            "ok": True,
            "paused": self._safety_paused,
            "manualControlActive": self._manual_control_active,
            "message": (
                "Full motor/autonomy stop backend unavailable; UI STOP only sends zero velocity and disables UI commands"
                if self._safety_paused and not self._action_ready(self._behavior_client)
                else ("UI command pause active" if self._safety_paused else "UI safety state enabled; Take Control required before teleop")
            ),
        }

    def set_teleop_command(self, payload) -> dict:
        if not isinstance(payload, dict):
            return {"accepted": False, "message": "Invalid teleop payload"}
        speed_mode = str(payload.get("speedMode", "normal")).strip().lower()
        if speed_mode not in SPEED_MODES:
            speed_mode = "normal"
        controls = payload.get("controls")
        if controls is None:
            command = str(payload.get("command", "stop")).strip().lower()
            legacy_controls = {
                "forward": {"forward"},
                "backward": {"backward"},
                "left": {"rotate-left"},
                "right": {"rotate-right"},
                "stop": set(),
            }
            if command not in legacy_controls:
                return {"accepted": False, "message": "Unknown teleop command"}
            controls = legacy_controls[command]
        if not isinstance(controls, (list, tuple, set)):
            return {"accepted": False, "message": "Invalid controls"}
        allowed = {"forward", "backward", "strafe-left", "strafe-right", "rotate-left", "rotate-right"}
        active_controls = {str(control).strip().lower() for control in controls}
        if not active_controls.issubset(allowed):
            return {"accepted": False, "message": "Unknown control"}
        if self._safety_paused or not self._manual_control_active:
            active_controls = set()
        self._active_controls = active_controls
        self._speed_mode = speed_mode
        self._last_command_time = time.monotonic()
        self._publish_active_twist()
        return {"ok": True, "message": "Teleop command accepted"}

    def send_navigation_goal(self, payload) -> dict:
        self._publish_twist(0.0, 0.0, 0.0)
        if self._safety_paused:
            return {"ok": False, "message": "START required before navigation"}
        if self._behavior_client is None or BehaviorType is None:
            return {"ok": False, "message": "Navigation backend unavailable"}
        if not self._behavior_client.server_is_ready():
            return {"ok": False, "message": "Navigation backend unavailable"}
        try:
            x = float(payload["x"])
            y = float(payload["y"])
        except (KeyError, TypeError, ValueError):
            return {"ok": False, "message": "No valid map position selected"}
        goal = ExecuteBehavior.Goal()
        goal.behavior.type = BehaviorType.NAVIGATE
        goal.target_pose.position.x = x
        goal.target_pose.position.y = y
        goal.target_pose.position.z = 0.0
        goal.target_pose.orientation.w = 1.0
        self._behavior_client.send_goal_async(goal)
        return {"ok": True, "message": "Navigation goal sent to real behavior action"}

    def set_task_mode(self, payload) -> dict:
        if self._safety_paused:
            return {"ok": False, "message": "START required before task mode commands"}
        if self._task_mode_client is None:
            return {"ok": False, "message": "Task mode backend unavailable"}
        if not self._task_mode_client.service_is_ready():
            return {"ok": False, "message": "Task mode backend unavailable"}
        mode = TASK_MODE_TO_BACKEND.get(str(payload.get("mode", "")).strip().lower())
        if not mode:
            return {"ok": False, "message": "Unknown task mode"}
        request = SetRobotMode.Request()
        request.mode = mode
        self._task_mode_client.call_async(request)
        return {"ok": True, "message": f"Requested task mode {mode}"}

    def take_control(self, _payload) -> dict:
        self._active_controls = set()
        self._publish_twist(0.0, 0.0, 0.0)
        if self._safety_paused:
            self._manual_control_active = False
            return {
                "ok": False,
                "manualControlActive": False,
                "message": "Press START before taking control",
            }
        if self._manual_control_active:
            self._manual_control_active = False
            self._send_behavior_request(BehaviorType.IDLE if BehaviorType is not None else None)
            return {
                "ok": True,
                "manualControlActive": False,
                "message": (
                    "Released UI teleop control; requested behavior IDLE"
                    if self._action_ready(self._behavior_client)
                    else "Released UI teleop control; release-control backend unavailable"
                ),
            }
        self._manual_control_active = True
        self._send_behavior_request(BehaviorType.TELEOP if BehaviorType is not None else None)
        return {
            "ok": True,
            "manualControlActive": True,
            "message": (
                "UI teleop control active; requested behavior TELEOP"
                if self._action_ready(self._behavior_client)
                else "UI-side teleop gating active; backend take-control interface unavailable"
            ),
        }

    def set_active_view(self, payload) -> dict:
        page = str(payload.get("page", "dashboard")).strip().lower()
        if page not in {"dashboard", "teleop"}:
            return {"accepted": False, "message": "Unknown view"}
        self._active_page = page
        if page != "teleop":
            self._active_controls = set()
            self._publish_twist(0.0, 0.0, 0.0)
        self._sync_camera_subscriptions()
        return {"ok": True, "page": self._active_page}

    def select_camera(self, payload) -> dict:
        camera = str(payload.get("camera", "base")).strip().lower()
        if camera not in {"base", "arm"}:
            return {"accepted": False, "message": "Unknown camera"}
        if camera == "arm" and not self._topics.get("armCameraCompressed"):
            return {"accepted": False, "message": "Arm camera unavailable"}
        if camera == "base" and not self._topics.get("baseCameraCompressed"):
            return {"accepted": False, "message": "Base camera unavailable"}
        self._selected_camera = camera
        self._sync_camera_subscriptions()
        return {"ok": True, "message": f"{camera} camera selected"}

    def send_arm_pose(self, payload) -> dict:
        if self._safety_paused:
            return {"ok": False, "message": "START required before arm commands"}
        if self._arm_pose_client is None:
            return {"ok": False, "message": "Arm pose backend unavailable"}
        if not self._arm_pose_client.service_is_ready():
            return {"ok": False, "message": "Arm pose backend unavailable"}
        pose = str(payload.get("pose", "")).strip().lower()
        if pose not in ARM_POSES:
            return {"ok": False, "message": "Unknown arm pose"}
        request = SendNamedArmPose.Request()
        request.pose_name = pose
        self._arm_pose_client.call_async(request)
        return {"ok": True, "message": f"Requested arm pose {pose}"}

    def start_mapping(self, _payload) -> dict:
        if self._safety_paused:
            return {"ok": False, "message": "START required before mapping commands"}
        return self._call_trigger_service(
            self._start_mapping_client,
            "Start mapping service unavailable",
            "Start mapping request sent",
        )

    def done_mapping(self, _payload) -> dict:
        if self._safety_paused:
            return {"ok": False, "message": "START required before mapping commands"}
        if self._map is None:
            return {"ok": False, "message": "No map received yet"}
        if not self._service_ready(self._done_mapping_client):
            return {
                "ok": True,
                "message": "No finalize backend available; reviewing latest received map locally",
            }
        return self._call_trigger_service(
            self._done_mapping_client,
            "Finalize mapping service unavailable",
            "Finalize mapping request sent",
        )

    def save_safe_map(self, payload) -> dict:
        if self._safety_paused:
            return {"ok": False, "message": "START required before safe-map save"}
        classifications = payload.get("classifications") if isinstance(payload, dict) else None
        candidates = payload.get("candidates") if isinstance(payload, dict) else None
        safe_map_review = payload.get("safeMapReview") if isinstance(payload, dict) else None
        if not isinstance(classifications, dict):
            classifications = {}
        if not isinstance(candidates, list) or not candidates:
            return {"ok": False, "message": "No artifact candidates received"}
        if not isinstance(safe_map_review, dict):
            safe_map_review = self._safe_map_review(candidates, classifications)
        result = self._call_trigger_service(
            self._save_safe_map_client,
            "Save safe map backend unavailable",
            "Safe map save request sent",
        )
        result["safeMapReview"] = safe_map_review
        if result.get("ok"):
            result["message"] = (
                f"{result.get('message') or 'Safe map save request sent'}; "
                f"kept {len(safe_map_review.get('objects', []))} objects, "
                f"removed {len(safe_map_review.get('removed_false_scans', []))} false scans"
            )
        return result

    def _safe_map_review(self, candidates: list, classifications: dict) -> dict:
        objects = []
        removed_false_scans = []
        for candidate in candidates:
            if not isinstance(candidate, dict):
                continue
            candidate_id = _clean_text(candidate.get("id"))
            if not candidate_id:
                continue
            classification = _clean_text(classifications.get(candidate_id), "unclassified")
            if classification not in {"wall", "bed", "obstacle", "false_scan"}:
                classification = "unclassified"
            if classification in {"unclassified", "false_scan"}:
                removed_false_scans.append(candidate_id)
                continue
            objects.append(
                {
                    "id": candidate_id,
                    "class": classification,
                    "geometry_type": candidate.get("geometryType", "unknown"),
                    "geometry": candidate.get("raw", {}).get("geometry", {}),
                    "source": candidate.get("source", ""),
                }
            )
        return {"objects": objects, "removed_false_scans": removed_false_scans}

    def _call_trigger_service(self, client, unavailable_message: str, accepted_message: str) -> dict:
        if not self._service_ready(client):
            return {"ok": False, "message": unavailable_message}
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 3.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            return {"ok": False, "message": f"{accepted_message}; waiting for backend response"}
        try:
            response = future.result()
        except Exception as exc:
            return {"ok": False, "message": f"{unavailable_message}: {exc}"}
        return {
            "ok": bool(response.success),
            "message": response.message or accepted_message,
        }

    def _on_compressed_image(self, camera: str, msg: CompressedImage) -> None:
        if msg.format and "jpeg" not in msg.format.lower() and "jpg" not in msg.format.lower():
            return
        now = time.monotonic()
        if now - self._last_frame_update_time.get(camera, 0.0) < self._camera_frame_period:
            return
        with self._frame_lock:
            self._frames[camera] = bytes(msg.data)
            self._frame_versions[camera] = self._frame_versions.get(camera, 0) + 1
            self._last_frame_update_time[camera] = now

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map = {
            "width": msg.info.width,
            "height": msg.info.height,
            "resolution": msg.info.resolution,
            "origin": {
                "x": msg.info.origin.position.x,
                "y": msg.info.origin.position.y,
            },
            "data": list(msg.data),
            "frameId": msg.header.frame_id,
            "receivedAt": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        }

    def _on_artifact_candidates(self, msg: String) -> None:
        received_at = datetime.now(timezone.utc).isoformat(timespec="seconds")
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self._artifact_candidates_parse_error = f"Could not parse artifact candidate JSON: {exc}"
            self.get_logger().warn(self._artifact_candidates_parse_error)
            return

        if isinstance(payload, dict):
            candidates = payload.get("candidates", [])
        elif isinstance(payload, list):
            candidates = payload
        else:
            self._artifact_candidates = []
            self._artifact_candidates_received_at = received_at
            self._artifact_candidates_parse_error = "Artifact candidate JSON must be an object with candidates or a list"
            self.get_logger().warn(self._artifact_candidates_parse_error)
            return

        if not isinstance(candidates, list):
            self._artifact_candidates = []
            self._artifact_candidates_received_at = received_at
            self._artifact_candidates_parse_error = "Artifact candidate payload field 'candidates' is not a list"
            self.get_logger().warn(self._artifact_candidates_parse_error)
            return

        normalized_candidates = []
        warnings = []
        for index, candidate in enumerate(candidates):
            normalized, warning = _normalize_artifact_candidate(candidate, index)
            if normalized is not None:
                normalized_candidates.append(normalized)
            if warning:
                warnings.append(warning)

        self._artifact_candidates = normalized_candidates
        self._artifact_candidates_received_at = received_at
        self._artifact_candidates_parse_error = "; ".join(warnings)
        if warnings:
            self.get_logger().warn(f"Artifact candidate warnings: {'; '.join(warnings)}")

    def _on_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        self._robot_pose = {"x": pose.position.x, "y": pose.position.y, "yaw": yaw_from_quaternion(pose.orientation)}

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        self._robot_pose = {"x": pose.position.x, "y": pose.position.y, "yaw": yaw_from_quaternion(pose.orientation)}

    def _on_plant_health(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Ignoring invalid plant health JSON: {exc}")
            return
        flower_id = str(payload.get("flower_id", "")).strip()
        if not flower_id:
            self.get_logger().warn("Ignoring plant health JSON without flower_id")
            return
        self._update_plant_from_payload(flower_id, payload)
        self._external_report = None

    def _on_typed_plant_health(self, msg) -> None:
        flower_id = str(msg.flower_id).strip()
        if not flower_id:
            self.get_logger().warn("Ignoring PlantHealth without flower_id")
            return
        payload = {
            "flower_id": flower_id,
            "bed_id": str(msg.bed_id).strip(),
            "height_cm": round(float(msg.height_cm), 1),
            "color": msg.color,
            "health": msg.health,
            "growth_stage": msg.growth_stage,
            "bug_detected": bool(msg.bug_detected),
            "flower_detected": bool(msg.flower_detected),
            "ready_for_harvest": bool(msg.ready_for_harvest),
            "confidence": float(msg.confidence),
            "last_scan_time": self._time_msg_to_iso(msg.last_scan_time),
            "notes": msg.notes,
            "position": {"x": msg.position.x, "y": msg.position.y, "z": msg.position.z},
        }
        self._update_plant_from_payload(flower_id, payload)
        self._external_report = None

    def _on_flower_counts(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Ignoring invalid flower count JSON: {exc}")
            return
        counts = {}
        for key in ("magenta", "light_pink", "white", "total", "bed_id"):
            if key not in payload:
                continue
            try:
                counts[key] = int(payload[key])
            except (TypeError, ValueError):
                continue
        if "total" not in counts:
            counts["total"] = sum(
                counts.get(color, 0) for color in ("magenta", "light_pink", "white")
            )
        counts["received_at"] = datetime.now(timezone.utc).isoformat(timespec="seconds")
        self._flower_counts = counts
        self._external_report = None

    def _on_bed_observation(self, msg) -> None:
        bed_id = str(msg.bed_id)
        self._bed_observations[bed_id] = {
            "bed_id": msg.bed_id,
            "co2": None,
            "humidity": None,
            "bugs_detected": None,
            "available": True,
            "last_seen": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        }

    def _on_mapping_status(self, msg) -> None:
        self._mapping_status = {
            "scanSeen": bool(msg.scan_seen),
            "odomSeen": bool(msg.odom_seen),
            "mapSeen": bool(msg.map_seen),
            "localized": bool(msg.localized),
            "activeMap": msg.active_map,
            "message": msg.message,
            "receivedAt": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        }

    def _on_task_status(self, msg) -> None:
        self._task_status = {
            "current_state": msg.current_state,
            "active": bool(msg.active),
            "error": bool(msg.error),
            "message": msg.message,
            "receivedAt": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        }

    def _time_msg_to_iso(self, msg) -> str:
        seconds = int(msg.sec)
        nanoseconds = int(msg.nanosec)
        if seconds <= 0 and nanoseconds <= 0:
            return ""
        return datetime.fromtimestamp(seconds + nanoseconds / 1_000_000_000.0, tz=timezone.utc).isoformat(timespec="seconds")

    def _update_plant_from_payload(self, flower_id: str, payload: dict) -> None:
        plant = self._plants.setdefault(flower_id, {"flower_id": flower_id})
        for key in ("bed_id", "height_cm", "color", "health", "growth_stage", "confidence", "last_scan_time", "notes", "position"):
            if key in payload:
                plant[key] = payload[key]
        for key in ("bug_detected", "flower_detected", "ready_for_harvest"):
            if key in payload:
                plant[key] = bool(payload[key])

    def _on_plant_health_report(self, msg: String) -> None:
        try:
            report = json.loads(msg.data)
        except json.JSONDecodeError:
            report = {"nextAction": msg.data}
        self._external_report = {
            "totalBeds": report.get("totalBeds", report.get("total_beds", len(self._bed_observations))),
            "totalFlowers": report.get("totalFlowers", report.get("total_flowers", len(self._plants))),
            "lastScanTime": report.get("lastScanTime", report.get("last_scan_time", "")),
            "nextAction": report.get("nextAction", report.get("next_action", "Real report received")),
        }

    def _on_battery(self, msg: BatteryState) -> None:
        if msg.percentage >= 0:
            now = time.monotonic()
            if self._battery_percent is not None and now - self._last_battery_update_at < 60.0:
                return
            self._battery_percent = max(0.0, min(100.0, float(msg.percentage) * 100.0))
            self._last_battery_update_at = now

    def _on_teleop_timer(self) -> None:
        if self._active_controls and time.monotonic() - self._last_command_time > 0.6:
            self._active_controls = set()
        self._publish_active_twist()

    def _publish_active_twist(self) -> None:
        if self._safety_paused or not self._manual_control_active:
            self._publish_twist(0.0, 0.0, 0.0)
            return
        linear_x = 0.0
        linear_y = 0.0
        angular_z = 0.0
        speeds = SPEED_MODES[self._speed_mode]
        linear_speed = min(float(speeds["linear"]), MAX_LINEAR_SPEED)
        angular_speed = min(float(speeds["angular"]), MAX_ANGULAR_SPEED)
        if "forward" in self._active_controls:
            linear_x += linear_speed
        if "backward" in self._active_controls:
            linear_x -= linear_speed
        if "strafe-left" in self._active_controls:
            linear_y += linear_speed
        if "strafe-right" in self._active_controls:
            linear_y -= linear_speed
        if "rotate-left" in self._active_controls:
            angular_z += angular_speed
        if "rotate-right" in self._active_controls:
            angular_z -= angular_speed
        planar_speed = math.hypot(linear_x, linear_y)
        if planar_speed > linear_speed > 0.0:
            scale = linear_speed / planar_speed
            linear_x *= scale
            linear_y *= scale
        self._publish_twist(linear_x, linear_y, angular_z)

    def _publish_twist(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        twist = Twist()
        twist.linear.x = max(-MAX_LINEAR_SPEED, min(MAX_LINEAR_SPEED, linear_x))
        twist.linear.y = max(-MAX_LINEAR_SPEED, min(MAX_LINEAR_SPEED, linear_y))
        twist.angular.z = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, angular_z))
        self._cmd_vel_publisher.publish(twist)

    def _send_behavior_request(self, behavior_type) -> None:
        if behavior_type is None or self._behavior_client is None:
            return
        if not self._behavior_client.server_is_ready():
            return
        goal = ExecuteBehavior.Goal()
        goal.behavior.type = behavior_type
        self._behavior_client.send_goal_async(goal)

    def _bed_payload(self) -> list[dict]:
        return sorted(copy.deepcopy(list(self._bed_observations.values())), key=lambda bed: str(bed.get("bed_id", "")))

    def _computed_report(self) -> dict:
        plants = list(self._plants.values())
        last_scan = max((plant.get("last_scan_time", "") for plant in plants), default="")
        total_flowers = (
            int(self._flower_counts.get("total", 0))
            if isinstance(self._flower_counts, dict)
            else len(plants)
        )
        if not last_scan and isinstance(self._flower_counts, dict):
            last_scan = self._flower_counts.get("received_at", "")
        return {
            "totalBeds": len(self._bed_observations),
            "totalFlowers": total_flowers,
            "lastScanTime": last_scan,
            "nextAction": "Waiting for real data" if not plants and not self._bed_observations and not self._flower_counts else "Real records received",
        }


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UiNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
