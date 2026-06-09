import copy
import ast
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
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, CompressedImage
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

try:
    from PIL import Image
except ImportError:
    Image = None

try:
    from nav2_msgs.action import NavigateToPose
except ImportError:
    NavigateToPose = None

try:
    from simbiosys_interfaces.action import ExecuteBehavior
    from simbiosys_interfaces.msg import BedObservation, BehaviorType, CurrentMission, HarvestStatus, MappingStatus, PlantHealth, ScanProgress, TaskStatus
    from simbiosys_interfaces.srv import SendNamedArmPose, SetRobotMode
except ImportError:
    ExecuteBehavior = None
    BedObservation = None
    BehaviorType = None
    CurrentMission = None
    HarvestStatus = None
    MappingStatus = None
    PlantHealth = None
    ScanProgress = None
    TaskStatus = None
    SendNamedArmPose = None
    SetRobotMode = None

CONFIG_PATH = Path(__file__).resolve().parent / "config" / "rosTopics.json"
ROBOT_ICON_PATH = Path(__file__).resolve().parent / "SimBioSys_logov2.png"
DEFAULT_WEB_HOST = "0.0.0.0"
DEFAULT_WEB_PORT = 8080
DEFAULT_TELEOP_QUEUE_DEPTH = 1
BED_IDS = ("1", "2", "3")
BED_TAG_TO_ID = {
    18: "1",
    10: "1",
    13: "2",
    2: "2",
    24: "2",
    16: "3",
}
BED_ID_TO_TAGS = {
    bed_id: tuple(tag for tag, mapped_bed_id in BED_TAG_TO_ID.items() if mapped_bed_id == bed_id)
    for bed_id in BED_IDS
}
DEFAULT_CONFIG = {
    "rosbridgeUrl": "ws://localhost:9090",
    "topics": {
        "cmdVel": "/mirte_base_controller/cmd_vel",
        "baseCameraCompressed": "/camera/color/image_raw/compressed",
        "baseCameraRaw": None,
        "armCameraCompressed": "/gripper_camera/image_raw/compressed",
        "armCameraRaw": None,
        "liveMap": "/map",
        "staticMapYaml": "maps/mirte_map.yaml",
        "mapUpdatePeriodSec": 10,
        "initialPose": "/initialpose",
        "goalPose": "/goal_pose",
        "navigateToPose": "/navigate_to_pose",
        "navPlan": "/plan",
        "homePose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
        "mappingStatus": "simbiosys/mapping_status",
        "taskStatus": "simbiosys/task_status",
        "currentMission": "simbiosys/current_mission",
        "scanProgress": "simbiosys/scan_progress",
        "harvestStatus": "simbiosys/harvest_status",
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
        "manualControlState": "simbiosys/ui/manual_control_active",
        "checkpointCommand": "/checkpoint_commands",
        "checkpointStatus": "/checkpoint_status",
        "odom": "/mirte_base_controller/odom",
        "amclPose": "/amcl_pose",
        "plantHealth": "/plant_health",
        "plantHealthReport": "/plant_health_report",
        "typedPlantHealth": "simbiosys/plant_health",
        "flowerCounts": "/simbiosys/flower_counts",
        "bedObservation": "simbiosys/bed_observation",
        "bedEnvironment": "/bed_environment",
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
      user-select: none;
      -webkit-user-select: none;
      -webkit-touch-callout: none;
      -webkit-tap-highlight-color: transparent;
    }
    button:disabled, select:disabled {
      cursor: not-allowed;
      color: #78847e;
      background: #141a18;
    }
    button:active:not(:disabled):not(.plant-dot), button.active:not(.plant-dot) {
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
      cursor: crosshair;
      touch-action: none;
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
      border: 4px solid var(--line);
      border-radius: 8px;
      padding: 12px;
      background: #17201d;
      display: grid;
      gap: 8px;
      min-height: 260px;
      align-content: start;
    }
    .bed-card.available, .bed-card.status-ok { border-color: #56c271; }
    .bed-card.status-warning { border-color: #e1b94b; }
    .bed-card.status-danger { border-color: #e2685d; }
    .bed-card.unavailable { border-color: #59615d; }
    .bed-update-age {
      margin: -2px 0 0;
      color: var(--muted);
      font-size: 0.82rem;
    }
    .bed-metrics {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 8px;
    }
    .bed-metrics p {
      margin: 0;
      color: var(--muted);
      font-size: 0.85rem;
    }
    .bed-plot {
      min-height: 210px;
      border: 1px solid #34433d;
      border-radius: 6px;
      background: #101815;
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      grid-template-rows: repeat(2, minmax(90px, 1fr));
      gap: 8px;
      padding: 10px;
    }
    .bed-compartment {
      border: 1px solid #34433d;
      border-radius: 6px;
      padding: 8px;
      display: grid;
      grid-template-rows: auto 1fr;
      gap: 8px;
      min-width: 0;
      background: #121b18;
    }
    .bed-compartment-label {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 6px;
      color: var(--muted);
      font-size: 0.78rem;
    }
    .bed-compartment-label strong {
      display: inline;
      color: var(--text);
      font-size: 0.82rem;
    }
    .compartment-empty {
      color: #78847e;
      font-size: 0.78rem;
      align-self: center;
    }
    .bed-row {
      display: flex;
      align-items: center;
      align-content: center;
      flex-wrap: wrap;
      gap: 6px;
      min-width: 0;
    }
    .plant-dot {
      flex: 0 0 44px;
      width: 44px;
      height: 44px;
      aspect-ratio: 1;
      padding: 0;
      border: 3px solid #d8e0dc;
      border-radius: 50%;
      cursor: pointer;
      appearance: none;
      display: grid;
      place-items: center;
      color: #07110b;
      font-size: 0.82rem;
      font-weight: 800;
      line-height: 1;
      box-shadow: 0 0 0 2px #0c1110;
      touch-action: manipulation;
    }
    .plant-dot:active, .plant-dot.pressed { border-color: #edf5ef; }
    .plant-dot.selected { outline: 2px solid #edf5ef; outline-offset: 3px; }
    .plant-detail {
      border-top: 1px solid var(--line);
      padding-top: 8px;
      color: var(--muted);
      font-size: 0.9rem;
    }
    .plant-detail strong { font-size: 1rem; }
    .plant-detail-grid {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 8px;
      margin-top: 8px;
    }
    .plant-detail-grid p {
      margin: 0;
      color: var(--muted);
      font-size: 0.85rem;
    }
    .plant-detail-grid strong {
      display: block;
      color: var(--text);
      font-size: 1rem;
    }
    .plant-notes {
      margin: 8px 0 0;
      color: var(--muted);
    }
    .plant-warning-list {
      display: grid;
      gap: 4px;
      margin-top: 10px;
      color: #ff6b5f;
      font-size: 0.84rem;
      font-weight: 700;
    }
    .plant-warning-list p {
      margin: 0;
      overflow-wrap: anywhere;
    }
    .plant-detail-panel {
      min-height: 118px;
    }
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
    .pad button { touch-action: none; }
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
              <span id="robot-pose-status" class="status-pill">Robot pose: waiting</span>
            </div>
            <div id="dashboard-map-placeholder" class="placeholder">Waiting for SLAM map</div>
            <canvas id="dashboard-map-canvas" width="900" height="520" style="display:none"></canvas>
            <div class="control-row">
              <span id="selected-target">No map position selected</span>
              <div class="topline">
                <button id="map-zoom-out">-</button>
                <button id="map-zoom-reset">1x</button>
                <button id="map-zoom-in">+</button>
                <button id="set-initial-pose" disabled>Set Start Pose</button>
                <button id="navigate-target" disabled>Navigate</button>
                <button id="cancel-navigation" disabled>Cancel Navigation</button>
                <button id="go-home" disabled>Go Home</button>
                <button id="checkpoint-next" disabled>Start Mission Route</button>
              </div>
            </div>
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
        <div class="panel-body plant-detail-panel" id="selected-plant-detail">
          <h2>Flower Info</h2>
          <p>Select a flower</p>
        </div>
      </section>
    </section>

    <section id="teleop-page" class="page">
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
      latestNavPlan: null,
      latestRobotPose: null,
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
      lastPlanStamp: null,
      lastRobotPoseStamp: null,
      selectedMapTarget: null,
      mapTargetDrag: null,
      mapZoom: 1,
      mapPan: {x: 0, y: 0},
      mapPanDrag: null,
      mapPointers: new Map(),
      navigationActive: false,
      checkpointCommandAvailable: false,
      selectedPlantId: null,
      manualControlActive: false,
      safetyPaused: false,
      topics: {}
    };
    const robotIcon = new Image();
    robotIcon.addEventListener("load", () => {
      renderMaps({map: state.reviewMode && state.frozenMap ? state.frozenMap : state.latestMap, robotPose: state.latestRobotPose, navPlan: state.latestNavPlan});
    });
    robotIcon.src = "/assets/robot.png";
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
      document.getElementById("set-initial-pose").disabled = state.safetyPaused || !state.selectedMapTarget || !state.initialPoseAvailable;
      document.getElementById("go-home").disabled = state.safetyPaused || !state.navigationAvailable;
      document.getElementById("checkpoint-next").disabled = state.safetyPaused || !state.checkpointCommandAvailable;
      document.getElementById("cancel-navigation").disabled = state.safetyPaused || !state.navigationActive;
      const takeButton = document.getElementById("take-control");
      takeButton.textContent = state.manualControlActive ? "Release Control" : "Take Control";
      takeButton.disabled = state.safetyPaused;
      const keyboardText = document.querySelector("#robot-controls .control-row:last-child strong");
      if (keyboardText) keyboardText.textContent = state.manualControlActive ? "W/S A/D Q/E" : "Press Take Control first";
    }

    function mapToCanvas(map, canvas) {
      const zoom = Math.max(0.35, Math.min(8, Number(state.mapZoom) || 1));
      const cell = Math.min(canvas.width / map.width, canvas.height / map.height) * zoom;
      return {
        cell,
        x0: (canvas.width - map.width * cell) / 2 + state.mapPan.x,
        y0: (canvas.height - map.height * cell) / 2 + state.mapPan.y,
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
      if (!container) return;
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
    function clamp(value, min, max) {
      return Math.max(min, Math.min(max, value));
    }
    function yawDegrees(yaw) {
      if (!Number.isFinite(yaw)) return "";
      let degrees = yaw * 180 / Math.PI;
      while (degrees > 180) degrees -= 360;
      while (degrees <= -180) degrees += 360;
      return `${Math.round(degrees)}deg`;
    }
    function updateSelectedTargetLabel() {
      const target = state.selectedMapTarget;
      document.getElementById("selected-target").textContent = target
        ? `Selected ${target.x.toFixed(2)}, ${target.y.toFixed(2)}, ${yawDegrees(Number(target.yaw) || 0)}`
        : "No target selected";
    }
    function robotPoseStatusText(robotPose) {
      if (!robotPose || !Number.isFinite(robotPose.x) || !Number.isFinite(robotPose.y)) return "Robot pose: waiting";
      const source = robotPose.source === "amcl" ? "AMCL" : "odom";
      const age = robotPose.receivedAt ? `, ${timeSince(robotPose.receivedAt)}` : "";
      return `Robot pose: ${source} ${robotPose.x.toFixed(2)}, ${robotPose.y.toFixed(2)}, ${yawDegrees(robotPose.yaw)}${age}`;
    }
    function mapStatusText(map) {
      if (!map) return "Map unavailable";
      return map.source === "static" ? "Static map loaded" : "Live map available";
    }
    function setMapZoom(zoom) {
      state.mapZoom = Math.max(0.35, Math.min(8, zoom));
      document.getElementById("map-zoom-reset").textContent = `${state.mapZoom.toFixed(1)}x`;
      renderMaps({map: state.reviewMode && state.frozenMap ? state.frozenMap : state.latestMap, robotPose: state.latestRobotPose, navPlan: state.latestNavPlan});
    }
    function drawRobotIcon(ctx, iconSize, heading) {
      if (robotIcon.complete && robotIcon.naturalWidth > 0) {
        ctx.save();
        ctx.rotate(-heading + Math.PI / 2);
        ctx.beginPath();
        ctx.arc(0, 0, iconSize / 2, 0, Math.PI * 2);
        ctx.clip();
        ctx.fillStyle = "#f7f8f7";
        ctx.fillRect(-iconSize / 2, -iconSize / 2, iconSize, iconSize);
        ctx.drawImage(robotIcon, -iconSize / 2, -iconSize / 2, iconSize, iconSize);
        ctx.restore();

        ctx.save();
        ctx.strokeStyle = "#66a8d9";
        ctx.lineWidth = iconSize * 0.08;
        ctx.beginPath();
        ctx.arc(0, 0, iconSize / 2, 0, Math.PI * 2);
        ctx.stroke();
        ctx.restore();
        return;
      }

      const length = iconSize * 0.78;
      const width = iconSize * 0.56;
      ctx.rotate(-heading);
      ctx.fillStyle = "#66a8d9";
      ctx.strokeStyle = "#eaf5fb";
      ctx.lineWidth = iconSize * 0.06;
      ctx.beginPath();
      ctx.moveTo(length * 0.52, 0);
      ctx.lineTo(length * 0.18, -width * 0.55);
      ctx.lineTo(-length * 0.48, -width * 0.55);
      ctx.lineTo(-length * 0.58, 0);
      ctx.lineTo(-length * 0.48, width * 0.55);
      ctx.lineTo(length * 0.18, width * 0.55);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
    }
    function drawRobotMarker(ctx, map, canvas, robotPose) {
      if (!robotPose || !Number.isFinite(robotPose.x) || !Number.isFinite(robotPose.y) || !map.resolution) return;
      const robot = worldToCanvas(map, canvas, robotPose.x, robotPose.y);
      const heading = Number.isFinite(robotPose.yaw) ? robotPose.yaw : 0;
      const pixelsPerMeter = mapToCanvas(map, canvas).cell / map.resolution;
      const iconSize = 0.35 * pixelsPerMeter;

      ctx.save();
      ctx.translate(robot.x, robot.y);
      drawRobotIcon(ctx, iconSize, heading);
      ctx.restore();

      const label = [
        `Robot ${robotPose.x.toFixed(2)}, ${robotPose.y.toFixed(2)}`,
        yawDegrees(robotPose.yaw),
        robotPose.source || "",
      ].filter(Boolean).join(" | ");
      ctx.save();
      ctx.font = "12px sans-serif";
      const textWidth = ctx.measureText(label).width;
      const labelX = clamp(robot.x + 12, 4, canvas.width - textWidth - 12);
      const labelY = clamp(robot.y - 20, 18, canvas.height - 8);
      ctx.fillStyle = "rgba(10, 17, 14, 0.82)";
      ctx.strokeStyle = "#66a8d9";
      ctx.lineWidth = 1;
      ctx.beginPath();
      if (typeof ctx.roundRect === "function") {
        ctx.roundRect(labelX - 6, labelY - 14, textWidth + 12, 20, 5);
      } else {
        ctx.rect(labelX - 6, labelY - 14, textWidth + 12, 20);
      }
      ctx.fill();
      ctx.stroke();
      ctx.fillStyle = "#eaf5fb";
      ctx.fillText(label, labelX, labelY);
      ctx.restore();
    }
    function drawOccupancyGrid(canvasId, placeholderId, map, robotPose, showRobot, selectedTarget, navPlan) {
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
        const yaw = Number(selectedTarget.yaw) || 0;
        const arrowLength = 28;
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
        ctx.beginPath();
        ctx.moveTo(target.x, target.y);
        ctx.lineTo(target.x + Math.cos(yaw) * arrowLength, target.y - Math.sin(yaw) * arrowLength);
        ctx.stroke();
        ctx.fillStyle = "#e1b94b";
        ctx.beginPath();
        ctx.arc(target.x + Math.cos(yaw) * arrowLength, target.y - Math.sin(yaw) * arrowLength, 4, 0, Math.PI * 2);
        ctx.fill();
      }
      const planPoints = navPlan && Array.isArray(navPlan.points) ? navPlan.points : [];
      if (planPoints.length >= 2) {
        ctx.save();
        ctx.strokeStyle = "#66a8d9";
        ctx.lineWidth = 3;
        ctx.setLineDash([]);
        drawWorldPolyline(ctx, map, canvas, planPoints, false, false);
        ctx.restore();
      }
      if (showRobot && robotPose && Number.isFinite(robotPose.x) && map.resolution) {
        drawRobotMarker(ctx, map, canvas, robotPose);
      }
      drawCandidateOverlays(ctx, map, canvas);
    }

    function mapEventWorld(event) {
      if (!state.latestMap) return;
      const canvas = event.currentTarget;
      const point = canvasEventPoint(canvas, event);
      return canvasToWorld(
        state.latestMap,
        canvas,
        point.x,
        point.y
      );
    }
    function canvasEventPoint(canvas, event) {
      const rect = canvas.getBoundingClientRect();
      return {
        x: (event.clientX - rect.left) * (canvas.width / rect.width),
        y: (event.clientY - rect.top) * (canvas.height / rect.height),
      };
    }
    function mapPointerCentroid() {
      const pointers = Array.from(state.mapPointers.values());
      if (!pointers.length) return null;
      return {
        x: pointers.reduce((sum, pointer) => sum + pointer.x, 0) / pointers.length,
        y: pointers.reduce((sum, pointer) => sum + pointer.y, 0) / pointers.length,
      };
    }
    function startMapPan(point, pointerId = null) {
      if (state.mapTargetDrag && Object.prototype.hasOwnProperty.call(state.mapTargetDrag, "previousTarget")) {
        state.selectedMapTarget = state.mapTargetDrag.previousTarget;
        updateSelectedTargetLabel();
        updateSafetyButton();
      }
      state.mapTargetDrag = null;
      if (!point) return;
      state.mapPanDrag = {
        pointerId,
        x: point.x,
        y: point.y,
        panX: state.mapPan.x,
        panY: state.mapPan.y,
      };
    }
    function updateMapPan(point) {
      if (!state.mapPanDrag || !point) return;
      state.mapPan = {
        x: state.mapPanDrag.panX + point.x - state.mapPanDrag.x,
        y: state.mapPanDrag.panY + point.y - state.mapPanDrag.y,
      };
      renderMaps({map: state.reviewMode && state.frozenMap ? state.frozenMap : state.latestMap, robotPose: state.latestRobotPose, navPlan: state.latestNavPlan});
    }
    function setSelectedMapTarget(target, yaw = null) {
      if (!target) return;
      state.selectedMapTarget = {
        x: target.x,
        y: target.y,
        yaw: Number.isFinite(yaw) ? yaw : (Number.isFinite(state.latestRobotPose?.yaw) ? state.latestRobotPose.yaw : 0),
      };
      updateSelectedTargetLabel();
      updateSafetyButton();
      renderMaps({map: state.latestMap, robotPose: state.latestRobotPose, navPlan: state.latestNavPlan});
    }
    document.getElementById("dashboard-map-canvas").addEventListener("pointerdown", (event) => {
      const canvas = event.currentTarget;
      const point = canvasEventPoint(canvas, event);
      state.mapPointers.set(event.pointerId, point);
      canvas.setPointerCapture(event.pointerId);
      if (event.button === 2 || event.buttons === 2) {
        event.preventDefault();
        startMapPan(point, event.pointerId);
        return;
      }
      if (event.pointerType === "touch" && state.mapPointers.size >= 2) {
        event.preventDefault();
        startMapPan(mapPointerCentroid());
        return;
      }
      const target = mapEventWorld(event);
      if (!target) return;
      state.mapTargetDrag = {x: target.x, y: target.y, yaw: null, previousTarget: state.selectedMapTarget};
      setSelectedMapTarget(target);
    });
    document.getElementById("dashboard-map-canvas").addEventListener("pointermove", (event) => {
      const canvas = event.currentTarget;
      const point = canvasEventPoint(canvas, event);
      if (state.mapPointers.has(event.pointerId)) {
        state.mapPointers.set(event.pointerId, point);
      }
      if (state.mapPanDrag) {
        event.preventDefault();
        updateMapPan(state.mapPanDrag.pointerId == null ? mapPointerCentroid() : point);
        return;
      }
      if (event.pointerType === "touch" && state.mapPointers.size >= 2) {
        event.preventDefault();
        startMapPan(mapPointerCentroid());
        return;
      }
      if (!state.mapTargetDrag) return;
      const current = mapEventWorld(event);
      if (!current) return;
      const dx = current.x - state.mapTargetDrag.x;
      const dy = current.y - state.mapTargetDrag.y;
      if (Math.hypot(dx, dy) < 0.03) return;
      state.mapTargetDrag.yaw = Math.atan2(dy, dx);
      setSelectedMapTarget({x: state.mapTargetDrag.x, y: state.mapTargetDrag.y}, state.mapTargetDrag.yaw);
    });
    document.getElementById("dashboard-map-canvas").addEventListener("pointerup", (event) => {
      if (state.mapTargetDrag && Number.isFinite(state.mapTargetDrag.yaw)) {
        setSelectedMapTarget({x: state.mapTargetDrag.x, y: state.mapTargetDrag.y}, state.mapTargetDrag.yaw);
      }
      state.mapPointers.delete(event.pointerId);
      state.mapTargetDrag = null;
      state.mapPanDrag = null;
      try {
        event.currentTarget.releasePointerCapture(event.pointerId);
      } catch (_error) {}
    });
    document.getElementById("dashboard-map-canvas").addEventListener("pointercancel", (event) => {
      state.mapPointers.delete(event.pointerId);
      state.mapTargetDrag = null;
      state.mapPanDrag = null;
      try {
        event.currentTarget.releasePointerCapture(event.pointerId);
      } catch (_error) {}
    });
    document.getElementById("dashboard-map-canvas").addEventListener("contextmenu", (event) => event.preventDefault());
    document.getElementById("dashboard-map-canvas").addEventListener("wheel", (event) => {
      event.preventDefault();
      const direction = event.deltaY > 0 ? -1 : 1;
      setMapZoom(state.mapZoom * (direction > 0 ? 1.2 : 1 / 1.2));
    }, {passive: false});
    document.getElementById("map-zoom-out").addEventListener("click", () => setMapZoom(state.mapZoom / 1.25));
    document.getElementById("map-zoom-in").addEventListener("click", () => setMapZoom(state.mapZoom * 1.25));
    document.getElementById("map-zoom-reset").addEventListener("click", () => setMapZoom(1));
    document.getElementById("navigate-target").addEventListener("click", async () => {
      if (!state.selectedMapTarget) return;
      const data = await postJson("/api/navigation/goal", state.selectedMapTarget);
      document.getElementById("navigation-message").textContent = data.message;
    });
    document.getElementById("set-initial-pose").addEventListener("click", async () => {
      if (!state.selectedMapTarget) return;
      const data = await postJson("/api/navigation/initial_pose", state.selectedMapTarget);
      document.getElementById("navigation-message").textContent = data.message;
    });
    document.getElementById("cancel-navigation").addEventListener("click", async () => {
      const data = await postJson("/api/navigation/cancel");
      document.getElementById("navigation-message").textContent = data.message;
    });
    document.getElementById("go-home").addEventListener("click", async () => {
      const data = await postJson("/api/navigation/home");
      document.getElementById("navigation-message").textContent = data.message;
    });
    document.getElementById("checkpoint-next").addEventListener("click", async () => {
      const data = await postJson("/api/checkpoint/next");
      document.getElementById("navigation-message").textContent = data.message;
    });

    function renderMaps(data) {
      const map = state.reviewMode && state.frozenMap ? state.frozenMap : (data.map || state.latestMap);
      drawOccupancyGrid("dashboard-map-canvas", "dashboard-map-placeholder", map, data.robotPose, true, state.selectedMapTarget, data.navPlan);
    }
    function maybeRenderMaps(data) {
      const map = state.reviewMode && state.frozenMap ? state.frozenMap : (data.map || state.latestMap);
      const stamp = map ? map.receivedAt : null;
      const candidateStamp = state.reviewMode ? "review" : state.artifactCandidatesReceivedAt;
      const planStamp = data.navPlan ? data.navPlan.receivedAt : null;
      const robotPoseStamp = data.robotPose ? data.robotPose.receivedAt : null;
      const periodMs = Math.max(1, Number(data.topics.mapUpdatePeriodSec || 10)) * 1000;
      const now = Date.now();
      if (!map) {
        renderMaps(data);
        return;
      }
      if (
        candidateStamp !== state.lastCandidateStamp ||
        planStamp !== state.lastPlanStamp ||
        robotPoseStamp !== state.lastRobotPoseStamp ||
        (stamp !== state.lastMapStamp && (now - state.lastMapRenderMs >= periodMs || !state.lastMapStamp))
      ) {
        state.lastMapStamp = stamp;
        state.lastCandidateStamp = candidateStamp;
        state.lastPlanStamp = planStamp;
        state.lastRobotPoseStamp = robotPoseStamp;
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
    function latestPlantUpdateAge(plants) {
      const latest = (plants || []).reduce((latestSeen, plant) => {
        const parsed = Date.parse(plant.last_scan_time || "");
        if (!Number.isFinite(parsed)) return latestSeen;
        return Math.max(latestSeen, parsed);
      }, 0);
      return latest ? timeSince(new Date(latest).toISOString()) : "no flower update yet";
    }
    function renderReport(report, plants) {
      document.getElementById("bed-count").textContent = report.totalBeds;
      document.getElementById("flower-count").textContent = report.totalFlowers;
      document.getElementById("last-scan").textContent = timeSince(report.lastScanTime);
      document.getElementById("next-action").textContent = report.nextAction || "Waiting for real data";
    }
    function numericValue(value) {
      const number = Number(value);
      return Number.isFinite(number) ? number : null;
    }
    function bedSeverity(bed) {
      if (bed.bugs_detected === true) return "danger";
      const co2 = numericValue(bed.co2);
      const humidity = numericValue(bed.humidity);
      let warnings = 0;
      if (co2 != null) {
        if (co2 < 250 || co2 > 2000) return "danger";
        if (co2 < 400 || co2 > 1000) warnings += 1;
      }
      if (humidity != null) {
        if (humidity < 30 || humidity > 90) return "danger";
        if (humidity < 45 || humidity > 75) warnings += 1;
      }
      if (warnings >= 2) return "danger";
      if (warnings === 1) return "warning";
      return bed.available ? "ok" : "unavailable";
    }
    function plantColor(plant) {
      const color = String(plant.color || "").toLowerCase();
      if (color === "magenta") return "#d657c9";
      if (color === "light_pink") return "#f0a9cc";
      if (color === "white") return "#edf5ef";
      return "#87918b";
    }
    function plantBedSide(plant) {
      const direct = String(plant.bed_side || plant.bedSide || plant.side_letter || plant.zijkant || "").toLowerCase();
      if (direct === "a" || direct === "b") return direct;
      const side = String(plant.side || "").toLowerCase();
      if (side === "a" || side === "b") return side;
      const notes = String(plant.notes || "").toLowerCase();
      const match = notes.match(/(?:bed[_ -]?side|zijkant|side)[:=](a|b)\b/);
      return match ? match[1] : "a";
    }
    function plantLane(plant, index) {
      const direct = String(plant.lane || plant.compartment || plant.section || plant.vak || plant.position_side || "").toLowerCase();
      if (direct === "left" || direct === "right") return direct;
      const side = String(plant.side || "").toLowerCase();
      if (side === "left" || side === "right") return side;
      const notes = String(plant.notes || "").toLowerCase();
      const match = notes.match(/(?:lane|section|vak|position|side)[:=](left|right)\b/);
      if (match) return match[1];
      return index % 2 === 0 ? "left" : "right";
    }
    function compartmentLabel(section) {
      return `${section.bedSide.toUpperCase()} ${section.lane}`;
    }
    function plantDetailMetric(label, value) {
      const item = document.createElement("p");
      const strong = document.createElement("strong");
      strong.textContent = value == null || value === "" ? "unavailable" : value;
      item.appendChild(strong);
      item.append(label);
      return item;
    }
    function visiblePlantNotes(plant) {
      return String(plant.notes || "")
        .replace(/flower detected:\\s*[\\w_-]+/gi, "")
        .replace(/\\s*side[:=](left|right|a|b)\\b/gi, "")
        .replace(/\\s*(bed[_ -]?side|zijkant|lane|section|vak|position)[:=](left|right|a|b)\\b/gi, "")
        .trim();
    }
    function renderPlantDetail(plant, label) {
      const detail = document.createElement("div");
      detail.className = "plant-detail";
      const name = document.createElement("strong");
      name.textContent = label || plant.flower_id || "plant";
      const grid = document.createElement("div");
      grid.className = "plant-detail-grid";
      grid.appendChild(plantDetailMetric("color", plant.color));
      grid.appendChild(plantDetailMetric("height", plant.height_cm == null ? null : `${plant.height_cm} cm`));
      grid.appendChild(plantDetailMetric("bed", plant.bed_id));
      grid.appendChild(plantDetailMetric("vak", `${plantBedSide(plant).toUpperCase()} ${plantLane(plant, 0)}`));
      detail.appendChild(name);
      detail.appendChild(grid);
      const visibleNotes = visiblePlantNotes(plant);
      if (visibleNotes) {
        const notes = document.createElement("p");
        notes.className = "plant-notes";
        notes.textContent = visibleNotes;
        detail.appendChild(notes);
      }
      return detail;
    }
    function bedCompartments(bedPlants) {
      const sections = [
        {bedSide: "a", lane: "left", plants: []},
        {bedSide: "a", lane: "right", plants: []},
        {bedSide: "b", lane: "left", plants: []},
        {bedSide: "b", lane: "right", plants: []},
      ];
      const lookup = new Map(sections.map((section) => [`${section.bedSide}:${section.lane}`, section]));
      (bedPlants || []).forEach((plant, index) => {
        const key = `${plantBedSide(plant)}:${plantLane(plant, index)}`;
        const section = lookup.get(key) || sections[0];
        section.plants.push(plant);
      });
      return sections;
    }
    function orderedBedPlants(bed) {
      return bedCompartments(Array.isArray(bed.plants) ? bed.plants : [])
        .flatMap((section) => section.plants);
    }
    function plantDisplayLabel(bed, plant) {
      const orderedPlants = orderedBedPlants(bed);
      const index = Math.max(0, orderedPlants.findIndex((candidate) => candidate.flower_id === plant.flower_id));
      return `${bed.bed_id}${String.fromCharCode(97 + (index % 26))}`;
    }
    function renderPlantRow(container, plants, bed, data) {
      const row = document.createElement("div");
      row.className = "bed-row";
      plants.slice(0, 10).forEach((plant) => {
        const dot = document.createElement("button");
        dot.type = "button";
        dot.className = `plant-dot ${state.selectedPlantId === plant.flower_id ? "selected" : ""}`;
        dot.style.background = plantColor(plant);
        dot.title = plant.height_cm == null ? (plant.flower_id || "plant") : `${plant.flower_id || "plant"} - ${plant.height_cm} cm`;
        dot.textContent = plantDisplayLabel(bed, plant);
        dot.addEventListener("pointerdown", () => dot.classList.add("pressed"));
        dot.addEventListener("pointerup", () => dot.classList.remove("pressed"));
        dot.addEventListener("pointercancel", () => dot.classList.remove("pressed"));
        dot.addEventListener("pointerleave", () => dot.classList.remove("pressed"));
        dot.addEventListener("click", () => {
          state.selectedPlantId = plant.flower_id;
          renderBeds(data);
        });
        row.appendChild(dot);
      });
      container.appendChild(row);
    }
    function renderCompartment(plot, section, bed, data) {
      const item = document.createElement("div");
      item.className = "bed-compartment";
      const colorPlant = section.plants.find((plant) => plant.color);
      const colorText = colorPlant ? colorPlant.color : "no flowers";
      item.innerHTML = `
        <div class="bed-compartment-label">
          <strong>${compartmentLabel(section)}</strong>
          <span>${colorText}</span>
        </div>
      `;
      if (section.plants.length) {
        renderPlantRow(item, section.plants, bed, data);
      } else {
        const empty = document.createElement("p");
        empty.className = "compartment-empty";
        empty.textContent = "empty";
        item.appendChild(empty);
      }
      plot.appendChild(item);
    }
    function plantWarningText(plant) {
      const warning = plant.warning == null ? "" : String(plant.warning).trim();
      return warning;
    }
    function renderBedPlantWarnings(card, bed, bedPlants) {
      const warnings = (bedPlants || [])
        .map((plant) => ({plant, warning: plantWarningText(plant)}))
        .filter((item) => item.warning);
      if (!warnings.length) return;

      const list = document.createElement("div");
      list.className = "plant-warning-list";
      warnings.forEach(({plant, warning}) => {
        const item = document.createElement("p");
        item.textContent = `${plantDisplayLabel(bed, plant)}: ${warning}`;
        list.appendChild(item);
      });
      card.appendChild(list);
    }
    function selectedPlantRecord(data) {
      for (const bed of data.beds || []) {
        const plant = (bed.plants || []).find((candidate) => candidate.flower_id === state.selectedPlantId);
        if (plant) return {bed, plant};
      }
      const plant = (data.plants || []).find((candidate) => candidate.flower_id === state.selectedPlantId);
      return plant ? {bed: {bed_id: plant.bed_id, plants: data.plants || []}, plant} : null;
    }
    function renderSelectedPlantDetail(data) {
      const container = document.getElementById("selected-plant-detail");
      container.innerHTML = "<h2>Flower Info</h2>";
      const selected = selectedPlantRecord(data);
      if (!selected) {
        const empty = document.createElement("p");
        empty.textContent = "Select a flower";
        container.appendChild(empty);
        return;
      }
      container.appendChild(renderPlantDetail(selected.plant, plantDisplayLabel(selected.bed, selected.plant)));
    }
    function renderBeds(data) {
      const container = document.getElementById("bed-overview");
      container.innerHTML = "";
      const beds = data.beds || [];
      const allPlants = data.plants || [];
      if (!beds.length) {
        const empty = document.createElement("p");
        empty.textContent = "Waiting for bed telemetry";
        container.appendChild(empty);
        return;
      }
      beds.forEach((bed) => {
        const card = document.createElement("article");
        const bedPlants = Array.isArray(bed.plants)
          ? bed.plants
          : allPlants.filter((plant) => String(plant.bed_id || "") === String(bed.bed_id || ""));
        const severity = bedSeverity(bed);
        card.className = `bed-card ${bed.available ? "available" : "unavailable"} status-${severity}`;
        card.innerHTML = `
          <div class="topline">
            <h3>Bed ${bed.bed_id}</h3>
            <button class="inspect-bed" data-bed-id="${bed.bed_id}">Inspect Bed</button>
          </div>
          <p class="bed-update-age">Flower update: ${latestPlantUpdateAge(bedPlants)}</p>
          <p class="bed-update-age">April tags: ${(bed.april_tags || []).join(", ") || "none"}</p>
          <div class="bed-metrics">
            <p><strong>${bed.co2 == null ? "unavailable" : bed.co2}</strong>CO2</p>
            <p><strong>${bed.humidity == null ? "unavailable" : bed.humidity}</strong>humidity</p>
            <p><strong>${bed.bugs_detected == null ? "unavailable" : (bed.bugs_detected ? "bugs" : "no bugs")}</strong>bug detection</p>
          </div>
          <div class="bed-plot">
          </div>
        `;
        const plot = card.querySelector(".bed-plot");
        bedCompartments(bedPlants).forEach((section) => renderCompartment(plot, section, bed, data));
        renderBedPlantWarnings(card, bed, bedPlants);
        const inspectButton = card.querySelector(".inspect-bed");
        inspectButton.disabled = state.safetyPaused || !data.behaviorAvailable;
        inspectButton.addEventListener("click", async () => {
          const result = await postJson("/api/bed/inspect", {bed_id: bed.bed_id});
          document.getElementById("navigation-message").textContent = result.message;
        });
        container.appendChild(card);
      });
      renderSelectedPlantDetail(data);
    }
    function renderCandidates() {
      const list = document.getElementById("candidate-list");
      if (!list) return;
      list.innerHTML = "";
      const detail = document.getElementById("candidate-detail");
      const candidates = candidateSet();
      const parseError = document.getElementById("candidate-parse-error");
      const renderWarnings = state.renderWarnings || [];
      const mappingMapTime = document.getElementById("mapping-map-time");
      if (mappingMapTime) mappingMapTime.textContent = mapStatusText(state.latestMap);
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
    const startMappingButton = document.getElementById("start-mapping");
    if (startMappingButton) {
      startMappingButton.addEventListener("click", async () => {
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
        renderMaps({map: state.frozenMap, robotPose: null, navPlan: null});
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
        renderMaps({map: state.latestMap, robotPose: null, navPlan: null});
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
    }

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
        state.initialPoseAvailable = data.navigation.initialPoseAvailable;
        state.navigationActive = data.navigation.active;
        state.checkpointCommandAvailable = Boolean(data.checkpoint && data.checkpoint.commandAvailable);
        if (data.map) state.latestMap = data.map;
        state.latestNavPlan = data.navPlan || state.latestNavPlan;
        state.latestRobotPose = data.robotPose || state.latestRobotPose;
        document.getElementById("connection").textContent = data.rosConnected ? "ROS node running" : "ROS waiting";
        document.getElementById("battery").textContent = data.batteryPercent == null ? "Battery --" : `Battery ${Math.round(data.batteryPercent)}%`;
        document.getElementById("camera-state").textContent = data.cameraReady ? `${data.camera.current} online` : `${data.camera.current} waiting`;
        document.getElementById("camera-feed").style.display = data.cameraReady ? "block" : "none";
        document.getElementById("camera-placeholder").style.display = data.cameraReady ? "none" : "grid";
        document.getElementById("camera-select").value = data.camera.current;
        document.querySelector('#camera-select option[value="arm"]').disabled = !data.camera.armAvailable;
        document.getElementById("robot-pose-status").textContent = robotPoseStatusText(state.latestRobotPose);
        maybeRenderMaps(data);
        renderReport(data.report, data.plants);
        renderBeds(data);
        document.getElementById("task-mode").disabled = state.safetyPaused || !data.taskMode.available;
        if (data.taskMode.available) {
          document.getElementById("task-message").textContent = data.taskMode.current ? `Current backend state: ${data.taskMode.current}` : "Task mode backend available";
        }
        document.getElementById("navigate-target").disabled = state.safetyPaused || !state.selectedMapTarget || !data.navigation.available;
        document.getElementById("set-initial-pose").disabled = state.safetyPaused || !state.selectedMapTarget || !data.navigation.initialPoseAvailable;
        document.getElementById("cancel-navigation").disabled = state.safetyPaused || !data.navigation.active;
        document.getElementById("go-home").disabled = state.safetyPaused || !data.navigation.available;
        document.getElementById("checkpoint-next").disabled = state.safetyPaused || !state.checkpointCommandAvailable;
        const mission = data.currentMission || null;
        const missionMessage = mission && mission.message
          ? `${mission.phase || "mission"} ${mission.activeBedId ? `bed ${mission.activeBedId}${mission.activeSide ? ` side ${mission.activeSide}` : ""}` : ""} ${mission.queueTotal ? `${mission.queueIndex}/${mission.queueTotal}` : ""}: ${mission.message}`
          : "";
        const checkpointMessage = data.checkpoint && data.checkpoint.status ? data.checkpoint.status.message : "";
        document.getElementById("navigation-message").textContent = missionMessage || checkpointMessage || data.navigation.message || (data.navigation.available ? "Select a map position to navigate" : "Navigation backend unavailable");
        document.getElementById("take-control-message").textContent = state.manualControlActive ? "Manual teleop control active" : data.takeControl.message;
        const startMappingButton = document.getElementById("start-mapping");
        if (startMappingButton) {
          const startReason = state.safetyPaused ? "START required before mapping commands" : (!data.mapping.startAvailable ? "Start mapping service unavailable" : "");
          const doneReason = !data.map ? "No map received yet" : (!data.mapping.doneBackendAvailable ? "No finalize backend; UI will freeze latest received map" : "");
          const saveReason = saveDisabledReason(data);
          startMappingButton.disabled = Boolean(startReason);
          startMappingButton.title = startReason;
          document.getElementById("done-mapping").disabled = !data.map;
          document.getElementById("done-mapping").title = doneReason;
          document.getElementById("retry-mapping").disabled = !state.frozenMap;
          document.getElementById("save-safe-map").disabled = Boolean(saveReason);
          document.getElementById("save-safe-map").title = saveReason;
          if (startReason && !state.reviewMode) document.getElementById("workflow-message").textContent = startReason;
          else if (saveReason && state.reviewMode) document.getElementById("workflow-message").textContent = saveReason;
        }
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


def quaternion_from_yaw(yaw: float) -> Quaternion:
    quat = Quaternion()
    quat.z = math.sin(yaw / 2.0)
    quat.w = math.cos(yaw / 2.0)
    return quat


def yaw_from_map_origin(origin: list) -> float:
    try:
        return float(origin[2])
    except (IndexError, TypeError, ValueError):
        return 0.0


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
        self.declare_parameter("camera_max_fps", 0.0)
        self.declare_parameter("live_map_topic", self._topics["liveMap"])
        self.declare_parameter("static_map_yaml", self._topics.get("staticMapYaml") or "")
        self._apply_topic_parameters()

        self._web_host = self.get_parameter("web_host").value
        self._web_port = int(self.get_parameter("web_port").value)
        self._frame_lock = threading.Lock()
        self._frames = {"base": None, "arm": None}
        self._frame_versions = {"base": 0, "arm": 0}
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
        self._map = self._load_static_map()
        self._nav_plan = None
        self._last_status_map_sent_at = 0.0
        self._robot_pose = None
        self._odom_pose = None
        self._amcl_pose = None
        self._home_pose = self._normalized_pose(self._topics.get("homePose") or {})
        self._active_nav_goal_handle = None
        self._active_nav_label = ""
        self._last_navigation_message = ""
        self._battery_percent = None
        self._last_battery_update_at = 0.0
        self._bed_observations = {}
        self._mapping_status = None
        self._task_status = None
        self._current_mission = None
        self._checkpoint_status = None
        self._artifact_candidates = []
        self._artifact_candidates_received_at = None
        self._artifact_candidates_parse_error = ""

        teleop_queue_depth = max(1, int(self.get_parameter("teleop_queue_depth").value))
        self._cmd_vel_publisher = self.create_publisher(Twist, self._topics["cmdVel"], teleop_queue_depth)
        self._initial_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, self._topics["initialPose"], 10) if self._topics.get("initialPose") else None
        self._goal_pose_publisher = self.create_publisher(PoseStamped, self._topics["goalPose"], 10) if self._topics.get("goalPose") else None
        self._checkpoint_command_publisher = self.create_publisher(String, self._topics["checkpointCommand"], 10) if self._topics.get("checkpointCommand") else None
        self._manual_control_publisher = None
        if self._topics.get("manualControlState"):
            manual_control_qos = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            self._manual_control_publisher = self.create_publisher(Bool, self._topics["manualControlState"], manual_control_qos)
            self._publish_manual_control_state()
        self._sync_camera_subscriptions()
        if self._topics.get("liveMap"):
            map_qos = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            self.create_subscription(OccupancyGrid, self._topics["liveMap"], self._on_map, map_qos)
        self.create_subscription(Odometry, self._topics["odom"], self._on_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, self._topics["amclPose"], self._on_amcl_pose, 10)
        if self._topics.get("navPlan"):
            self.create_subscription(NavPath, self._topics["navPlan"], self._on_nav_plan, 10)
        self.create_subscription(String, self._topics["plantHealth"], self._on_plant_health, 10)
        self.create_subscription(String, self._topics["plantHealthReport"], self._on_plant_health_report, 10)
        if self._topics.get("flowerCounts"):
            self.create_subscription(String, self._topics["flowerCounts"], self._on_flower_counts, 10)
        if self._topics.get("bedEnvironment"):
            self.create_subscription(String, self._topics["bedEnvironment"], self._on_bed_environment, 10)
        if self._topics.get("checkpointStatus"):
            checkpoint_status_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(String, self._topics["checkpointStatus"], self._on_checkpoint_status, checkpoint_status_qos)
        self.create_subscription(BatteryState, self._topics["battery"], self._on_battery, 10)
        if PlantHealth is not None:
            self._try_create_subscription(PlantHealth, self._topics["typedPlantHealth"], self._on_typed_plant_health, 10)
        if BedObservation is not None:
            self._try_create_subscription(BedObservation, self._topics["bedObservation"], self._on_bed_observation, 10)
        if MappingStatus is not None:
            self._try_create_subscription(MappingStatus, self._topics["mappingStatus"], self._on_mapping_status, 10)
        if TaskStatus is not None:
            self._try_create_subscription(TaskStatus, self._topics["taskStatus"], self._on_task_status, 10)
        if CurrentMission is not None and self._topics.get("currentMission"):
            self._try_create_subscription(CurrentMission, self._topics["currentMission"], self._on_current_mission, 10)
        if ScanProgress is not None and self._topics.get("scanProgress"):
            self._try_create_subscription(ScanProgress, self._topics["scanProgress"], self._on_scan_progress, 10)
        if HarvestStatus is not None and self._topics.get("harvestStatus"):
            self._try_create_subscription(HarvestStatus, self._topics["harvestStatus"], self._on_harvest_status, 10)
        if self._topics.get("artifactCandidates"):
            self.create_subscription(String, self._topics["artifactCandidates"], self._on_artifact_candidates, 10)

        self._task_mode_client = self._try_create_client(SetRobotMode, self._topics["setTaskMode"]) if SetRobotMode is not None else None
        self._arm_pose_client = self._try_create_client(SendNamedArmPose, self._topics["sendNamedArmPose"]) if SendNamedArmPose is not None else None
        self._behavior_client = self._try_create_action_client(ExecuteBehavior, self._topics["executeBehavior"]) if ExecuteBehavior is not None else None
        self._nav2_client = self._try_create_action_client(NavigateToPose, self._topics["navigateToPose"]) if NavigateToPose is not None and self._topics.get("navigateToPose") else None
        self._start_mapping_client = self.create_client(Trigger, self._topics["startMapping"]) if self._topics.get("startMapping") else None
        self._done_mapping_client = self.create_client(Trigger, self._topics["doneMapping"]) if self._topics.get("doneMapping") else None
        self._save_safe_map_client = self.create_client(Trigger, self._topics["saveSafeMap"]) if self._topics.get("saveSafeMap") else None

        self.create_timer(0.1, self._on_teleop_timer)
        self._http_server = UiHttpServer((self._web_host, self._web_port), self._make_handler())
        self._http_thread = threading.Thread(target=self._http_server.serve_forever, daemon=True)
        self._http_thread.start()

        self._log_startup_urls()
        self.get_logger().info(f"Publishing Twist to {self._topics['cmdVel']}; using real ROS/project data only")

    def _try_create_subscription(self, msg_type, topic: str, callback, qos: int):
        try:
            return self.create_subscription(msg_type, topic, callback, qos)
        except Exception as exc:
            self.get_logger().warn(f"Disabled typed subscription {topic}: {exc}")
            return None

    def _try_create_client(self, srv_type, service: str):
        try:
            return self.create_client(srv_type, service)
        except Exception as exc:
            self.get_logger().warn(f"Disabled typed service client {service}: {exc}")
            return None

    def _try_create_action_client(self, action_type, action: str):
        try:
            return ActionClient(self, action_type, action)
        except Exception as exc:
            self.get_logger().warn(f"Disabled typed action client {action}: {exc}")
            return None

    def _apply_topic_parameters(self) -> None:
        configured_raw_image = self.get_parameter("image_topic").get_parameter_value().string_value or None
        configured_compressed_image = self.get_parameter("compressed_image_topic").get_parameter_value().string_value
        if configured_raw_image and configured_raw_image != self._topics["baseCameraRaw"] and configured_compressed_image == self._topics["baseCameraCompressed"]:
            configured_compressed_image = f"{configured_raw_image}/compressed"
        self._topics["cmdVel"] = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        self._topics["baseCameraRaw"] = None
        self._topics["baseCameraCompressed"] = configured_compressed_image
        self._topics["liveMap"] = self.get_parameter("live_map_topic").get_parameter_value().string_value
        self._topics["staticMapYaml"] = self.get_parameter("static_map_yaml").get_parameter_value().string_value

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
                elif parsed.path == "/assets/robot.png":
                    self._send_robot_icon()
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
                    "/api/navigation/initial_pose": node.set_initial_pose,
                    "/api/navigation/cancel": node.cancel_navigation,
                    "/api/navigation/home": node.go_home,
                    "/api/checkpoint/next": node.move_to_next_checkpoint,
                    "/api/task_mode": node.set_task_mode,
                    "/api/take_control": node.take_control,
                    "/api/view": node.set_active_view,
                    "/api/camera/select": node.select_camera,
                    "/api/arm/pose": node.send_arm_pose,
                    "/api/bed/inspect": node.inspect_bed,
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

            def _send_robot_icon(self):
                try:
                    content = ROBOT_ICON_PATH.read_bytes()
                except OSError:
                    self.send_error(HTTPStatus.NOT_FOUND)
                    return
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "image/png")
                self.send_header("Cache-Control", "public, max-age=300")
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
                        time.sleep(0.02)
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
                        time.sleep(0.001)
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
            "robotPose": self._preferred_robot_pose(),
            "navPlan": copy.deepcopy(self._nav_plan),
            "batteryPercent": self._battery_percent,
            "bedObservations": copy.deepcopy(self._bed_observations),
            "mappingStatus": copy.deepcopy(self._mapping_status),
            "taskMode": {
                "available": self._service_ready(self._task_mode_client),
                "current": self._task_status.get("current_state") if self._task_status else "",
            },
            "navigation": {
                "available": self._navigation_available(),
                "nav2Available": self._action_ready(self._nav2_client),
                "behaviorAvailable": self._action_ready(self._behavior_client),
                "initialPoseAvailable": self._initial_pose_publisher is not None,
                "goalPoseAvailable": self._goal_pose_publisher is not None,
                "active": self._active_nav_goal_handle is not None,
                "message": self._navigation_message(),
            },
            "checkpoint": {
                "commandAvailable": self._action_ready(self._behavior_client),
                "status": copy.deepcopy(self._checkpoint_status),
            },
            "currentMission": copy.deepcopy(self._current_mission),
            "behaviorAvailable": self._action_ready(self._behavior_client),
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
        if self._map.get("source") == "static":
            return self._map
        period_sec = max(1.0, float(self._topics.get("mapUpdatePeriodSec") or 10))
        now = time.monotonic()
        if now - self._last_status_map_sent_at < period_sec:
            return None
        self._last_status_map_sent_at = now
        return self._map

    def _load_static_map(self) -> dict | None:
        configured = str(self._topics.get("staticMapYaml") or "").strip()
        if not configured:
            return None
        yaml_path = self._resolve_workspace_path(configured)
        if yaml_path is None:
            self.get_logger().warn(f"Static map YAML not found: {configured}")
            return None
        try:
            payload = self._map_payload_from_yaml(yaml_path)
        except (OSError, KeyError, ValueError, yaml.YAMLError) as exc:
            self.get_logger().warn(f"Could not load static map {yaml_path}: {exc}")
            return None
        self.get_logger().info(f"Loaded static UI map from {yaml_path}")
        return payload

    def _resolve_workspace_path(self, path_text: str) -> Path | None:
        path = Path(path_text).expanduser()
        if path.is_absolute():
            return path if path.exists() else None
        candidates = [
            Path.cwd() / path,
            Path(__file__).resolve().parents[4] / path,
            Path(__file__).resolve().parents[3] / path,
        ]
        for candidate in candidates:
            if candidate.exists():
                return candidate
        return None

    def _map_payload_from_yaml(self, yaml_path: Path) -> dict:
        with yaml_path.open("r", encoding="utf-8") as yaml_file:
            metadata = yaml.safe_load(yaml_file)
        if not isinstance(metadata, dict):
            raise ValueError("map YAML must contain an object")

        image_path = Path(str(metadata["image"]))
        if not image_path.is_absolute():
            image_path = yaml_path.parent / image_path
        width, height, pixels = self._load_grayscale_pixels(image_path)

        negate = int(metadata.get("negate", 0))
        occupied_thresh = float(metadata.get("occupied_thresh", 0.65))
        free_thresh = float(metadata.get("free_thresh", 0.25))
        data = []
        for y in range(height - 1, -1, -1):
            row_start = y * width
            for pixel in pixels[row_start : row_start + width]:
                probability = pixel / 255.0 if negate else (255.0 - pixel) / 255.0
                if probability > occupied_thresh:
                    data.append(100)
                elif probability < free_thresh:
                    data.append(0)
                else:
                    data.append(-1)

        origin = metadata.get("origin", [0.0, 0.0, 0.0])
        return {
            "width": width,
            "height": height,
            "resolution": float(metadata["resolution"]),
            "origin": {
                "x": float(origin[0]),
                "y": float(origin[1]),
                "yaw": yaw_from_map_origin(origin),
            },
            "data": data,
            "frameId": "map",
            "source": "static",
            "receivedAt": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        }

    def _load_grayscale_pixels(self, image_path: Path) -> tuple[int, int, list[int]]:
        if Image is None:
            raise ValueError("Pillow is required to load the static map image")
        with Image.open(image_path) as image:
            grayscale = image.convert("L")
            width, height = grayscale.size
            return width, height, list(grayscale.getdata())

    def toggle_safety(self, _payload) -> dict:
        self._safety_paused = not self._safety_paused
        self._active_controls = set()
        self._publish_twist(0.0, 0.0, 0.0)
        if self._safety_paused:
            self._manual_control_active = False
            self._publish_manual_control_state()
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

    def _navigation_available(self) -> bool:
        return bool(
            self._action_ready(self._nav2_client)
            or self._goal_pose_publisher is not None
            or self._action_ready(self._behavior_client)
        )

    def _navigation_message(self) -> str:
        if self._last_navigation_message:
            return self._last_navigation_message
        if self._action_ready(self._nav2_client):
            return "Nav2 NavigateToPose backend available"
        if self._action_ready(self._behavior_client):
            return "Behavior navigation backend available"
        if self._goal_pose_publisher is not None:
            return "Goal pose publisher available"
        return "Navigation backend unavailable"

    def _normalized_pose(self, payload) -> dict | None:
        if not isinstance(payload, dict):
            return None
        try:
            x = float(payload["x"])
            y = float(payload["y"])
            yaw = float(payload.get("yaw", 0.0))
        except (KeyError, TypeError, ValueError):
            return None
        return {"x": x, "y": y, "yaw": yaw}

    def _pose_stamped(self, pose: dict) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = pose["x"]
        msg.pose.position.y = pose["y"]
        msg.pose.position.z = 0.0
        msg.pose.orientation = quaternion_from_yaw(pose.get("yaw", 0.0))
        return msg

    def send_navigation_goal(self, payload) -> dict:
        self._publish_twist(0.0, 0.0, 0.0)
        if self._safety_paused:
            return {"ok": False, "message": "START required before navigation"}
        pose = self._normalized_pose(payload)
        if pose is None:
            return {"ok": False, "message": "No valid map position selected"}
        return self._send_navigation_pose(pose, "selected map position")

    def _send_navigation_pose(self, pose: dict, label: str) -> dict:
        if self._action_ready(self._nav2_client):
            goal = NavigateToPose.Goal()
            goal.pose = self._pose_stamped(pose)
            future = self._nav2_client.send_goal_async(
                goal,
                feedback_callback=lambda feedback: self._on_nav2_feedback(label, feedback),
            )
            future.add_done_callback(lambda done: self._on_nav2_goal_response(label, done))
            self._active_nav_label = label
            self._last_navigation_message = f"Sent Nav2 goal to {label}"
            return {"ok": True, "message": self._last_navigation_message}

        if self._behavior_client is not None and BehaviorType is not None and self._behavior_client.server_is_ready():
            goal = ExecuteBehavior.Goal()
            goal.behavior.type = BehaviorType.NAVIGATE
            goal.target_pose.position.x = pose["x"]
            goal.target_pose.position.y = pose["y"]
            goal.target_pose.position.z = 0.0
            goal.target_pose.orientation = quaternion_from_yaw(pose.get("yaw", 0.0))
            self._behavior_client.send_goal_async(goal)
            self._last_navigation_message = f"Navigation goal for {label} sent to behavior action"
            return {"ok": True, "message": self._last_navigation_message}

        if self._goal_pose_publisher is not None:
            self._goal_pose_publisher.publish(self._pose_stamped(pose))
            self._last_navigation_message = f"Published goal pose for {label}"
            return {"ok": True, "message": self._last_navigation_message}

        return {"ok": False, "message": "Navigation backend unavailable"}

    def _on_nav2_feedback(self, label: str, feedback_msg) -> None:
        feedback = getattr(feedback_msg, "feedback", None)
        distance = getattr(feedback, "distance_remaining", None)
        if distance is not None:
            self._last_navigation_message = f"Navigating to {label}: {float(distance):.2f} m remaining"

    def _on_nav2_goal_response(self, label: str, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._active_nav_goal_handle = None
            self._last_navigation_message = f"Nav2 goal error: {exc}"
            return
        if goal_handle is None or not goal_handle.accepted:
            self._active_nav_goal_handle = None
            self._last_navigation_message = f"Nav2 rejected goal to {label}"
            return
        self._active_nav_goal_handle = goal_handle
        self._last_navigation_message = f"Nav2 accepted goal to {label}"
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda done: self._on_nav2_result(label, done))

    def _on_nav2_result(self, label: str, future) -> None:
        self._active_nav_goal_handle = None
        try:
            future.result()
        except Exception as exc:
            self._last_navigation_message = f"Navigation to {label} failed: {exc}"
            return
        self._last_navigation_message = f"Navigation to {label} finished"

    def set_initial_pose(self, payload) -> dict:
        if self._safety_paused:
            return {"ok": False, "message": "START required before localization commands"}
        if self._initial_pose_publisher is None:
            return {"ok": False, "message": "Initial pose publisher unavailable"}
        pose = self._normalized_pose(payload)
        if pose is None:
            return {"ok": False, "message": "No valid map position selected"}
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = pose["x"]
        msg.pose.pose.position.y = pose["y"]
        msg.pose.pose.orientation = quaternion_from_yaw(pose.get("yaw", 0.0))
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        for _index in range(10):
            msg.header.stamp = self.get_clock().now().to_msg()
            self._initial_pose_publisher.publish(msg)
            time.sleep(0.02)
        self._last_navigation_message = f"Published initial pose at {pose['x']:.2f}, {pose['y']:.2f}"
        return {"ok": True, "message": self._last_navigation_message}

    def cancel_navigation(self, _payload) -> dict:
        if self._active_nav_goal_handle is None:
            self._last_navigation_message = "No active Nav2 goal to cancel"
            return {"ok": False, "message": self._last_navigation_message}
        self._active_nav_goal_handle.cancel_goal_async()
        self._active_nav_goal_handle = None
        self._last_navigation_message = "Cancel navigation requested"
        return {"ok": True, "message": self._last_navigation_message}

    def go_home(self, _payload) -> dict:
        self._publish_twist(0.0, 0.0, 0.0)
        if self._safety_paused:
            return {"ok": False, "message": "START required before navigation"}
        if self._home_pose is None:
            return {"ok": False, "message": "Home pose is not configured"}
        return self._send_navigation_pose(self._home_pose, "home")

    def move_to_next_checkpoint(self, _payload) -> dict:
        self._publish_twist(0.0, 0.0, 0.0)
        if self._safety_paused:
            return {"ok": False, "message": "START required before mission navigation"}
        if self._behavior_client is None or BehaviorType is None:
            return {"ok": False, "message": "Mission backend unavailable"}
        if not self._behavior_client.server_is_ready():
            return {"ok": False, "message": "Mission backend unavailable"}
        goal = ExecuteBehavior.Goal()
        goal.behavior.type = BehaviorType.INSPECT_BED
        goal.target_id = "all"
        self._behavior_client.send_goal_async(goal)
        self._last_navigation_message = "Started real mission route"
        return {"ok": True, "message": "Started real mission route"}

    def inspect_bed(self, payload) -> dict:
        if self._safety_paused:
            return {"ok": False, "message": "START required before bed inspection"}
        if self._behavior_client is None or BehaviorType is None:
            return {"ok": False, "message": "Bed inspection backend unavailable"}
        if not self._behavior_client.server_is_ready():
            return {"ok": False, "message": "Bed inspection backend unavailable"}
        bed_id = str(payload.get("bed_id", "")).strip()
        if not bed_id:
            return {"ok": False, "message": "No bed selected"}
        goal = ExecuteBehavior.Goal()
        goal.behavior.type = BehaviorType.INSPECT_BED
        goal.target_id = bed_id
        self._behavior_client.send_goal_async(goal)
        return {"ok": True, "message": f"Inspect bed {bed_id} requested"}

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
            self._publish_manual_control_state()
            return {
                "ok": False,
                "manualControlActive": False,
                "message": "Press START before taking control",
            }
        if self._manual_control_active:
            self._manual_control_active = False
            self._publish_manual_control_state()
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
        self._publish_manual_control_state()
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
        future = self._arm_pose_client.call_async(request)
        deadline = time.monotonic() + 3.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            return {"ok": False, "message": f"Requested arm pose {pose}; waiting for backend response"}
        try:
            response = future.result()
        except Exception as exc:
            return {"ok": False, "message": f"Arm pose backend error: {exc}"}
        return {
            "ok": bool(response.accepted),
            "message": response.message or f"Requested arm pose {pose}",
        }

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
        with self._frame_lock:
            self._frames[camera] = bytes(msg.data)
            self._frame_versions[camera] = self._frame_versions.get(camera, 0) + 1

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

    def _on_nav_plan(self, msg: NavPath) -> None:
        self._nav_plan = {
            "frameId": msg.header.frame_id,
            "receivedAt": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "points": [
                {
                    "x": pose.pose.position.x,
                    "y": pose.pose.position.y,
                    "yaw": yaw_from_quaternion(pose.pose.orientation),
                }
                for pose in msg.poses
            ],
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
        self._odom_pose = {
            "x": pose.position.x,
            "y": pose.position.y,
            "yaw": yaw_from_quaternion(pose.orientation),
            "source": "odom",
            "receivedAt": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "_receivedMonotonic": time.monotonic(),
        }

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        self._amcl_pose = {
            "x": pose.position.x,
            "y": pose.position.y,
            "yaw": yaw_from_quaternion(pose.orientation),
            "source": "amcl",
            "receivedAt": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "_receivedMonotonic": time.monotonic(),
        }

    def _preferred_robot_pose(self):
        if self._amcl_pose:
            return self._public_robot_pose(self._amcl_pose)
        return None

    def _public_robot_pose(self, pose: dict) -> dict:
        public_pose = copy.deepcopy(pose)
        public_pose.pop("_receivedMonotonic", None)
        return public_pose

    def _on_plant_health(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Ignoring invalid plant health JSON: {exc}")
            return
        flower_id = str(payload.get("flower_id", "")).strip()
        if not flower_id:
            if self._update_compartment_payload(payload):
                self._external_report = None
                return
            self.get_logger().warn("Ignoring plant health JSON without flower_id or compartment data")
            return
        self._update_plant_from_payload(flower_id, payload)
        self._external_report = None

    def _on_typed_plant_health(self, msg) -> None:
        flower_id = str(msg.flower_id).strip()
        if not flower_id:
            self.get_logger().warn("Ignoring PlantHealth without flower_id")
            return
        detected_colors = list(getattr(msg, "detected_colors", []))
        detected_heights = list(getattr(msg, "detected_heights_cm", []))
        detected_confidences = list(
            getattr(msg, "detected_confidences", [])
        )
        if not bool(msg.flower_detected):
            self._remove_scan_flowers(flower_id)
            self._external_report = None
            return
        if detected_colors or detected_heights:
            self._update_detected_flowers(
                flower_id,
                msg,
                detected_colors,
                detected_heights,
                detected_confidences,
            )
            self._external_report = None
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
        warning = str(getattr(msg, "warning", "")).strip()
        if not warning:
            warning = self._first_warning_from_text(msg.notes)
        if warning:
            payload["warning"] = warning
        bed_side = self._bed_side_from_notes(msg.notes)
        lane = self._lane_from_notes(msg.notes)
        if bed_side:
            payload["bed_side"] = bed_side
        if lane:
            payload["lane"] = lane
        self._update_plant_from_payload(flower_id, payload)
        self._external_report = None

    def _update_detected_flowers(
        self,
        scan_flower_id: str,
        msg,
        colors,
        heights,
        confidences,
    ) -> None:
        self._remove_scan_flowers(scan_flower_id)
        count = max(len(colors), len(heights), len(confidences))
        if count <= 0:
            return
        bed_side = self._bed_side_from_notes(msg.notes)
        lane = self._lane_from_notes(msg.notes)
        warnings = self._warnings_from_heights(
            [float(height) for height in heights]
        )
        last_scan_time = self._time_msg_to_iso(msg.last_scan_time)
        for index in range(count):
            color = (
                str(colors[index])
                if index < len(colors)
                else str(msg.color)
            )
            height = (
                round(float(heights[index]), 1)
                if index < len(heights)
                else round(float(msg.height_cm), 1)
            )
            confidence = (
                float(confidences[index])
                if index < len(confidences)
                else float(msg.confidence)
            )
            flower_id = f"{scan_flower_id}:{index + 1:02d}"
            payload = {
                "flower_id": flower_id,
                "bed_id": str(msg.bed_id).strip(),
                "bed_side": bed_side,
                "lane": lane,
                "height_cm": height,
                "color": color,
                "health": msg.health,
                "growth_stage": msg.growth_stage,
                "bug_detected": bool(msg.bug_detected),
                "flower_detected": True,
                "ready_for_harvest": bool(msg.ready_for_harvest),
                "confidence": confidence,
                "last_scan_time": last_scan_time,
                "notes": msg.notes,
                "position": {
                    "x": msg.position.x,
                    "y": msg.position.y,
                    "z": height,
                },
            }
            if index < len(warnings) and warnings[index]:
                payload["warning"] = warnings[index]
            self._update_plant_from_payload(flower_id, payload)

    def _remove_scan_flowers(self, scan_flower_id: str) -> None:
        prefix = f"{scan_flower_id}:"
        self._plants.pop(scan_flower_id, None)
        for flower_id in [
            key for key in self._plants if key.startswith(prefix)
        ]:
            self._plants.pop(flower_id, None)

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

    def _on_bed_environment(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Ignoring invalid bed environment JSON: {exc}")
            return
        bed_id = self._bed_id_from_payload(payload)
        if not bed_id:
            self.get_logger().warn("Ignoring bed environment JSON without known bed_id or tag_id")
            return
        bed = self._bed_observations.setdefault(
            bed_id,
            {
                "bed_id": bed_id,
                "co2": None,
                "humidity": None,
                "bugs_detected": None,
                "available": True,
            },
        )
        bed["bed_id"] = bed_id
        bed["available"] = True
        bed["last_seen"] = datetime.now(timezone.utc).isoformat(timespec="seconds")
        if "co2_ppm" in payload:
            bed["co2"] = payload["co2_ppm"]
        elif "co2" in payload:
            bed["co2"] = payload["co2"]
        if "humidity_percent" in payload:
            bed["humidity"] = payload["humidity_percent"]
        elif "humidity" in payload:
            bed["humidity"] = payload["humidity"]
        if "bugs_detected" in payload:
            bed["bugs_detected"] = bool(payload["bugs_detected"])
        if "side_counts" in payload and isinstance(payload["side_counts"], dict):
            bed["side_counts"] = copy.deepcopy(payload["side_counts"])

    def _on_bed_observation(self, msg) -> None:
        tag_ids = []
        for tag in getattr(msg, "tags", []):
            try:
                tag_ids.append(int(tag.id))
            except (TypeError, ValueError):
                continue
        try:
            tag_ids.append(int(msg.bed_id))
        except (TypeError, ValueError):
            pass
        bed_id = self._bed_id_from_tags(tag_ids) or self._behavior_bed_key(msg.bed_id)
        if bed_id not in BED_IDS:
            self.get_logger().warn(f"Ignoring bed observation for unknown bed/tag: {msg.bed_id}")
            return
        bed = self._bed_observations.setdefault(
            bed_id,
            {
                "bed_id": bed_id,
                "co2": None,
                "humidity": None,
                "bugs_detected": None,
                "available": True,
            },
        )
        bed["bed_id"] = bed_id
        bed["available"] = True
        bed["last_seen"] = datetime.now(timezone.utc).isoformat(timespec="seconds")
        if tag_ids:
            bed["tag_ids"] = tag_ids

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

    def _on_checkpoint_status(self, msg: String) -> None:
        parsed = None
        try:
            parsed = json.loads(msg.data)
        except json.JSONDecodeError:
            parsed = None
        if isinstance(parsed, dict):
            message = parsed.get("message", msg.data)
            next_target = parsed.get("next_target") or {}
            active_target = parsed.get("active_target") or {}
            arrived_target = parsed.get("arrived_target") or {}
            self._checkpoint_status = {
                "event": parsed.get("event", ""),
                "state": parsed.get("state", ""),
                "message": message,
                "nextIndex": parsed.get("next_index", 0),
                "routeLength": parsed.get("route_length", 0),
                "nextTarget": next_target,
                "activeTarget": active_target,
                "arrivedTarget": arrived_target,
                "error": bool(parsed.get("error", False)),
                "receivedAt": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            }
            return
        self._checkpoint_status = {
            "message": msg.data,
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

    def _on_current_mission(self, msg) -> None:
        self._current_mission = {
            "missionId": msg.mission_id,
            "phase": msg.phase,
            "activeBedId": msg.active_bed_id,
            "activeSide": msg.active_side,
            "activeScanPositionId": msg.active_scan_position_id,
            "queueIndex": int(msg.queue_index),
            "queueTotal": int(msg.queue_total),
            "error": bool(msg.error),
            "message": msg.message,
            "receivedAt": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        }

    def _on_scan_progress(self, msg) -> None:
        self._upsert_behavior_bed(
            msg.active_bed_id,
            msg.message,
            active_scan_position_id=getattr(msg, "active_scan_position_id", ""),
            active_flower_id=getattr(msg, "active_flower_id", ""),
            detection_status=getattr(msg, "detection_status", ""),
            error=bool(getattr(msg, "error", False)),
        )
        latest_health = getattr(msg, "latest_plant_health", None)
        if latest_health is not None and getattr(latest_health, "flower_id", ""):
            self._on_typed_plant_health(latest_health)

    def _on_harvest_status(self, msg) -> None:
        self._upsert_behavior_bed(
            msg.active_bed_id,
            msg.message,
            active_flower_id=getattr(msg, "active_flower_id", ""),
            phase=getattr(msg, "phase", ""),
            alignment_status=getattr(msg, "alignment_status", ""),
            error=bool(getattr(msg, "error", False)),
        )

    def _upsert_behavior_bed(self, bed_id, message: str = "", **status) -> None:
        bed_id = self._behavior_bed_key(bed_id)
        if not bed_id:
            return
        bed = self._bed_observations.setdefault(
            bed_id,
            {
                "bed_id": bed_id,
                "co2": None,
                "humidity": None,
                "bugs_detected": None,
                "available": True,
            },
        )
        bed["bed_id"] = bed_id
        bed["available"] = True
        bed["last_seen"] = datetime.now(timezone.utc).isoformat(timespec="seconds")
        bed["source"] = "behavior"
        if message:
            bed["message"] = message
        behavior_status = {key: value for key, value in status.items() if value not in ("", None)}
        if behavior_status:
            bed["behavior_status"] = behavior_status

    def _behavior_bed_key(self, bed_id) -> str:
        bed_id = str(bed_id or "").strip()
        if ":" in bed_id:
            bed_id = bed_id.split(":", 1)[0].strip()
        return bed_id if bed_id in BED_IDS else ""

    def _bed_id_from_tags(self, tag_ids) -> str:
        for tag_id in tag_ids:
            try:
                bed_id = BED_TAG_TO_ID.get(int(tag_id))
            except (TypeError, ValueError):
                continue
            if bed_id:
                return bed_id
        return ""

    def _bed_id_from_payload(self, payload: dict) -> str:
        bed_id = self._behavior_bed_key(payload.get("bed_id"))
        if bed_id:
            return bed_id

        tag_values = []
        for key in ("tag_id", "tag", "apriltag_id", "april_tag_id"):
            if key in payload:
                tag_values.append(payload[key])
        tags = payload.get("tags")
        if isinstance(tags, list):
            tag_values.extend(tags)
        return self._bed_id_from_tags(tag_values)

    def _time_msg_to_iso(self, msg) -> str:
        seconds = int(msg.sec)
        nanoseconds = int(msg.nanosec)
        if seconds <= 0 and nanoseconds <= 0:
            return ""
        return datetime.fromtimestamp(seconds + nanoseconds / 1_000_000_000.0, tz=timezone.utc).isoformat(timespec="seconds")

    def _update_plant_from_payload(self, flower_id: str, payload: dict) -> None:
        plant = self._plants.setdefault(flower_id, {"flower_id": flower_id})
        for key in ("bed_id", "side", "bed_side", "lane", "section", "compartment", "vak", "height_cm", "color", "health", "growth_stage", "confidence", "last_scan_time", "notes", "position", "warning"):
            if key in payload:
                plant[key] = payload[key]
        if "warning" not in payload:
            warning = self._first_warning_from_text(payload.get("notes", payload.get("message", "")))
            if warning:
                plant["warning"] = warning
        for key in ("bug_detected", "flower_detected", "ready_for_harvest"):
            if key in payload:
                plant[key] = bool(payload[key])

    def _update_compartment_payload(self, payload: dict) -> bool:
        section_payloads = self._compartment_payloads(payload)
        updated = False
        for section in section_payloads:
            if self._update_compartment_section(section):
                updated = True
        return updated

    def _compartment_payloads(self, payload: dict) -> list[dict]:
        bed_id = self._bed_id_from_payload(payload)
        sections = payload.get("sections", payload.get("compartments", payload.get("vakken")))
        if isinstance(sections, list):
            return [dict(section, bed_id=bed_id) for section in sections if isinstance(section, dict)]

        bed_side = self._normalized_bed_side(payload)
        nested = []
        if bed_side:
            for lane in ("left", "right"):
                value = payload.get(lane)
                if isinstance(value, dict):
                    nested.append(dict(value, bed_id=bed_id, bed_side=bed_side, lane=lane))
        if nested:
            return nested
        return [payload]

    def _update_compartment_section(self, payload: dict) -> bool:
        bed_id = self._bed_id_from_payload(payload)
        bed_side = self._normalized_bed_side(payload)
        lane = self._normalized_lane(payload)
        if not bed_id or not bed_side or not lane:
            return False

        heights = self._section_heights(payload)
        if heights is None:
            return False

        color = str(payload.get("color", payload.get("flower_color", payload.get("kleur", "")))).strip()
        warnings = self._warning_list_from_payload(payload)
        if len(warnings) != len(heights):
            height_warnings = self._warnings_from_heights(heights)
            if any(height_warnings):
                warnings = height_warnings
        last_scan_time = str(payload.get("last_scan_time", payload.get("timestamp", ""))).strip()
        if not last_scan_time:
            last_scan_time = datetime.now(timezone.utc).isoformat(timespec="seconds")

        prefix = f"{bed_id}:{bed_side}:{lane}:"
        for flower_id in [key for key in self._plants if key.startswith(prefix)]:
            self._plants.pop(flower_id, None)

        bed = self._bed_observations.setdefault(
            bed_id,
            {
                "bed_id": bed_id,
                "co2": None,
                "humidity": None,
                "bugs_detected": None,
                "available": True,
            },
        )
        bed["bed_id"] = bed_id
        bed["available"] = True
        bed["last_seen"] = last_scan_time

        for index, height in enumerate(heights, start=1):
            flower_id = f"{prefix}{index:02d}"
            plant = {
                "flower_id": flower_id,
                "bed_id": bed_id,
                "bed_side": bed_side,
                "lane": lane,
                "side": lane,
                "height_cm": height,
                "color": color,
                "last_scan_time": last_scan_time,
                "notes": f"bed_side:{bed_side} lane:{lane}",
            }
            if index - 1 < len(warnings) and warnings[index - 1]:
                plant["warning"] = warnings[index - 1]
            self._plants[flower_id] = plant
        return True

    def _warning_list_from_payload(self, payload: dict) -> list[str]:
        raw = payload.get("warnings", payload.get("warning", payload.get("height_warnings")))
        if raw is None and isinstance(payload.get("flowers"), list):
            raw = [
                flower.get("warning", flower.get("height_warning", ""))
                for flower in payload["flowers"]
                if isinstance(flower, dict)
            ]
        if raw is None:
            raw = self._warnings_from_text(payload.get("message", payload.get("notes", "")))
        if raw is None:
            return []
        if isinstance(raw, str):
            parsed = self._warnings_from_text(raw)
            raw = parsed if parsed is not None else [raw]
        if not isinstance(raw, list):
            raw = [raw]
        return [str(warning).strip() for warning in raw]

    def _first_warning_from_text(self, text) -> str:
        warnings = self._warnings_from_text(text)
        return warnings[0] if warnings else ""

    def _warnings_from_text(self, text) -> list[str] | None:
        text = str(text or "")
        marker = "warnings="
        if marker not in text:
            return None
        raw = text.split(marker, 1)[1].strip()
        if ";" in raw:
            raw = raw.split(";", 1)[0].strip()
        try:
            parsed = ast.literal_eval(raw)
        except (SyntaxError, ValueError):
            return [raw] if raw else []
        if isinstance(parsed, list):
            return [str(warning).strip() for warning in parsed]
        return [str(parsed).strip()] if parsed else []

    def _warnings_from_heights(self, heights: list[float]) -> list[str]:
        warnings = []
        for height in heights:
            if height <= 5.7:
                warnings.append("Low flower detection, probably flower detection from row behind!")
            elif height >= 8.8:
                warnings.append("Max flower height detected of 8.8cm so probably higher, inspect with wrist camera for accurate analysis!")
            else:
                warnings.append("")
        return warnings

    def _normalized_bed_side(self, payload: dict) -> str:
        for key in ("bed_side", "bedSide", "side_letter", "zijkant"):
            value = str(payload.get(key, "")).strip().lower()
            if value in ("a", "b"):
                return value
        side = str(payload.get("side", "")).strip().lower()
        return side if side in ("a", "b") else ""

    def _normalized_lane(self, payload: dict) -> str:
        for key in ("lane", "section", "compartment", "vak", "position", "position_side"):
            value = str(payload.get(key, "")).strip().lower()
            if value in ("left", "right"):
                return value
        side = str(payload.get("side", "")).strip().lower()
        return side if side in ("left", "right") else ""

    def _section_heights(self, payload: dict) -> list[float] | None:
        raw = payload.get("heights_cm", payload.get("heights", payload.get("plant_heights_cm", payload.get("flower_heights_cm"))))
        if raw is None and "height_cm" in payload:
            raw = [payload["height_cm"]]
        if raw is None and isinstance(payload.get("flowers"), list):
            raw = [
                flower.get("height_cm", flower.get("height"))
                for flower in payload["flowers"]
                if isinstance(flower, dict)
            ]
        if raw is None:
            return None
        if not isinstance(raw, list):
            raw = [raw]
        heights = []
        for value in raw:
            try:
                heights.append(round(float(value), 1))
            except (TypeError, ValueError):
                continue
        return heights

    def _bed_side_from_notes(self, notes: str) -> str:
        lowered = str(notes or "").lower()
        if (
            "bed_side:a" in lowered
            or "bed_side=a" in lowered
            or "side:a" in lowered
            or "side=a" in lowered
        ):
            return "a"
        if (
            "bed_side:b" in lowered
            or "bed_side=b" in lowered
            or "side:b" in lowered
            or "side=b" in lowered
        ):
            return "b"
        return ""

    @staticmethod
    def _lane_from_notes(notes: str) -> str:
        lowered = str(notes or "").lower()
        if "lane:right" in lowered or "lane=right" in lowered:
            return "right"
        if "lane:left" in lowered or "lane=left" in lowered:
            return "left"
        if "side:right" in lowered or "side=right" in lowered:
            return "right"
        if "side:left" in lowered or "side=left" in lowered:
            return "left"
        return ""

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
        # Do not compete with Nav2 or other controllers unless the UI owns
        # manual control. Control-release paths publish a one-shot zero Twist.
        if not self._manual_control_active:
            return
        if self._safety_paused:
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

    def _publish_manual_control_state(self) -> None:
        if self._manual_control_publisher is None:
            return
        msg = Bool()
        msg.data = bool(self._manual_control_active)
        self._manual_control_publisher.publish(msg)

    def _send_behavior_request(self, behavior_type) -> None:
        if behavior_type is None or self._behavior_client is None:
            return
        if not self._behavior_client.server_is_ready():
            return
        goal = ExecuteBehavior.Goal()
        goal.behavior.type = behavior_type
        self._behavior_client.send_goal_async(goal)

    def _bed_payload(self) -> list[dict]:
        beds_by_id = {
            bed_id: {
                "bed_id": bed_id,
                "april_tags": list(BED_ID_TO_TAGS.get(bed_id, ())),
                "co2": None,
                "humidity": None,
                "bugs_detected": None,
                "available": False,
            }
            for bed_id in BED_IDS
        }
        for bed_id, bed in self._bed_observations.items():
            bed_id = str(bed_id)
            if bed_id in beds_by_id:
                beds_by_id[bed_id].update(copy.deepcopy(bed))

        beds = list(beds_by_id.values())
        for bed in beds:
            bed_plants = [
                copy.deepcopy(plant)
                for plant in self._plants.values()
                if str(plant.get("bed_id", "")) == str(bed.get("bed_id", ""))
            ]
            bed["plants"] = bed_plants
            if bed_plants:
                compartment_counts = {"a": {"left": 0, "right": 0}, "b": {"left": 0, "right": 0}}
                for plant in bed_plants:
                    bed_side = str(plant.get("bed_side") or "a").lower()
                    lane = str(plant.get("lane") or plant.get("side") or "left").lower()
                    if bed_side in compartment_counts and lane in compartment_counts[bed_side]:
                        compartment_counts[bed_side][lane] += 1
                bed["compartment_counts"] = compartment_counts
        return sorted(beds, key=lambda bed: int(bed.get("bed_id", 0)))

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
        node._manual_control_active = False
        node._publish_manual_control_state()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
