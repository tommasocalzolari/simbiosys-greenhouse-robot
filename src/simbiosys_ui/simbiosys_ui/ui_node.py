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
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, CompressedImage, Image
from std_msgs.msg import String

try:
    from simbiosys_interfaces.msg import BedObservation, FlowerData, PlantHealth
except ImportError:
    BedObservation = None
    FlowerData = None
    PlantHealth = None

try:
    import cv2
    from cv_bridge import CvBridge
except ImportError:
    cv2 = None
    CvBridge = None


CONFIG_PATH = Path(__file__).resolve().parent / "config" / "rosTopics.json"
DEFAULT_WEB_HOST = "0.0.0.0"
DEFAULT_WEB_PORT = 8080
DEFAULT_CONFIG = {
    "rosbridgeUrl": "ws://localhost:9090",
    "dummyMode": True,
    "topics": {
        "cmdVel": "/mirte_base_controller/cmd_vel_unstamped",
        "cameraCompressed": "/camera/image_raw/compressed",
        "cameraRaw": "/camera/image_raw",
        "map": "/map",
        "odom": "/mirte_base_controller/odom",
        "amclPose": "/amcl_pose",
        "plantHealth": "/plant_health",
        "plantHealthReport": "/plant_health_report",
        "bedObservation": "simbiosys/bed_observation",
        "flowerDetections": "/flower_detections",
        "inspectBedCommand": "/ui/inspect_bed",
        "battery": "/battery_state",
    },
}

FLOWER_COUNTS = {"A": 20, "B": 18, "C": 22}
BED_IDS = tuple(FLOWER_COUNTS.keys())
SPEED_MODES = {
    "slow": {"linear": 0.15, "angular": 0.4},
    "normal": {"linear": 0.30, "angular": 0.7},
    "fast": {"linear": 0.50, "angular": 1.0},
}
MAX_LINEAR_SPEED = 1.0
MAX_ANGULAR_SPEED = 1.5


INDEX_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>SimBioSys Greenhouse UI</title>
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
      --soil: #6c5844;
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
      width: min(1220px, 100%);
      margin: 0 auto;
      padding: 18px;
      display: grid;
      gap: 14px;
    }

    header {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
    }

    h1 { margin: 0; font-size: 1.32rem; letter-spacing: 0; }
    h2 { margin: 0 0 10px; font-size: 1.02rem; letter-spacing: 0; }
    h3 { margin: 0 0 8px; font-size: 0.96rem; letter-spacing: 0; }
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

    button:active, button.active {
      background: var(--green);
      border-color: var(--green);
      color: #07110b;
    }

    select { min-height: 38px; padding: 0 10px; }

    .page { display: none; }
    .page.active { display: grid; gap: 14px; }
    .panel {
      border: 1px solid var(--line);
      border-radius: 8px;
      background: var(--panel);
      overflow: hidden;
    }
    .panel-body { padding: 14px; }
    .topline { display: flex; gap: 10px; align-items: center; flex-wrap: wrap; }
    .status-pill {
      border: 1px solid var(--line);
      border-radius: 999px;
      padding: 6px 10px;
      color: var(--muted);
      background: #121917;
      font-size: 0.9rem;
      white-space: nowrap;
    }

    .dashboard-grid {
      display: grid;
      grid-template-columns: minmax(0, 1.45fr) minmax(300px, 0.75fr);
      gap: 14px;
    }

    .map-wrap { position: relative; min-height: 470px; }
    .dummy-map {
      min-height: 470px;
      position: relative;
      display: grid;
      gap: 14px;
      padding: 14px;
      background:
        linear-gradient(90deg, rgba(255,255,255,0.035) 1px, transparent 1px),
        linear-gradient(0deg, rgba(255,255,255,0.035) 1px, transparent 1px),
        #0b100e;
      background-size: 34px 34px;
      border: 1px solid var(--line);
      border-radius: 8px;
      overflow: hidden;
    }
    .map-title {
      display: flex;
      justify-content: space-between;
      align-items: center;
      gap: 10px;
      color: var(--muted);
      font-weight: 700;
    }
    .bed-zone {
      position: relative;
      min-height: 118px;
      padding: 12px;
      border: 1px solid #4b654f;
      border-radius: 8px;
      background: rgba(35, 53, 46, 0.86);
    }
    .bed-zone-label {
      position: absolute;
      left: 12px;
      top: 10px;
      font-weight: 800;
      color: var(--text);
      z-index: 2;
    }
    .flower-grid {
      margin-top: 28px;
      display: grid;
      grid-template-columns: repeat(11, minmax(24px, 1fr));
      gap: 7px;
    }
    .flower-marker {
      min-width: 28px;
      height: 28px;
      border-radius: 50%;
      display: grid;
      place-items: center;
      background: #4fac67;
      color: #07110b;
      font-weight: 800;
      font-size: 0.72rem;
      border: 2px solid rgba(255,255,255,0.55);
      padding: 0;
    }
    .flower-marker[data-health="warning"] { background: var(--yellow); color: #211800; }
    .flower-marker[data-health="critical"] { background: var(--red); color: #210504; }
    .flower-marker[data-health="unknown"] { background: #718079; color: #07110b; }
    .flower-marker.selected {
      outline: 3px solid var(--blue);
      outline-offset: 2px;
      background: #f1cf5b;
    }
    .robot-marker {
      position: absolute;
      width: 18px;
      height: 18px;
      border-radius: 50%;
      background: var(--blue);
      border: 3px solid #d8efff;
      left: 50%;
      top: 50%;
      transform: translate(-50%, -50%);
      box-shadow: 0 0 0 5px rgba(102, 168, 217, 0.18);
      z-index: 3;
    }

    canvas {
      width: 100%;
      min-height: 470px;
      display: block;
      background: #0a0d0c;
    }

    .report-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px;
    }
    .metric {
      border-top: 1px solid var(--line);
      padding-top: 10px;
      color: var(--muted);
    }
    .metric strong {
      display: block;
      color: var(--text);
      font-size: 1.25rem;
    }
    .flower-detail {
      display: grid;
      gap: 8px;
      color: var(--muted);
    }
    .flower-detail strong { color: var(--text); }
    .bed-summaries {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 10px;
    }
    .bed-card {
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 12px;
      background: var(--panel);
      display: grid;
      gap: 8px;
      color: var(--muted);
    }
    .bed-card strong { color: var(--text); }
    .flower-list {
      max-height: 210px;
      overflow: auto;
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 6px;
      margin-top: 10px;
    }
    .flower-list button {
      min-height: 32px;
      padding: 0 8px;
      font-size: 0.85rem;
    }

    .teleop-grid {
      display: grid;
      grid-template-columns: minmax(0, 1.35fr) minmax(300px, 0.75fr);
      gap: 14px;
    }
    .camera {
      min-height: 470px;
      display: grid;
      place-items: center;
      background: #040605;
    }
    .camera img {
      width: 100%;
      height: 100%;
      max-height: calc(100vh - 170px);
      object-fit: contain;
      display: block;
    }
    .camera-placeholder { padding: 22px; text-align: center; color: var(--muted); }
    .pad {
      display: grid;
      grid-template-columns: repeat(3, minmax(70px, 1fr));
      grid-template-rows: repeat(3, 74px);
      gap: 10px;
    }
    .empty { visibility: hidden; }
    .control-row {
      display: flex;
      justify-content: space-between;
      gap: 10px;
      align-items: center;
      border-top: 1px solid var(--line);
      padding-top: 10px;
      color: var(--muted);
    }

    @media (max-width: 860px) {
      main { padding: 12px; }
      header { align-items: flex-start; flex-direction: column; }
      .dashboard-grid, .teleop-grid { grid-template-columns: 1fr; }
      .bed-summaries { grid-template-columns: 1fr; }
      .flower-grid { grid-template-columns: repeat(6, minmax(24px, 1fr)); }
      .camera { min-height: 300px; }
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
            <h2>Digital Twin Map</h2>
            <div id="map-panel" class="map-wrap">
              <div id="greenhouse-map" class="dummy-map"></div>
              <canvas id="map-canvas" width="900" height="520" style="display:none"></canvas>
            </div>
          </div>
        </section>
        <aside class="panel">
          <div class="panel-body">
            <h2>Plant Health Report</h2>
            <div class="report-grid">
              <div class="metric"><strong id="bed-count">0</strong>total beds</div>
              <div class="metric"><strong id="flower-count">0</strong>total flowers</div>
              <div class="metric"><strong id="healthy-count">0</strong>healthy flowers</div>
              <div class="metric"><strong id="risk-count">0</strong>warning / critical</div>
              <div class="metric"><strong id="ready-count">0</strong>ready for harvest</div>
              <div class="metric"><strong id="average-height">0 cm</strong>average height</div>
            </div>
            <div class="metric"><strong id="last-scan">unknown</strong>last scan</div>
            <div class="metric"><strong id="next-action">waiting</strong>recommended action</div>
          </div>
        </aside>
      </div>
      <section class="panel">
        <div class="panel-body flower-detail" id="flower-detail">
          <h2>Selected Flower</h2>
          <p>Click a flower marker on the map to inspect flower-specific data.</p>
        </div>
      </section>
      <section class="bed-summaries" id="bed-summaries"></section>
    </section>

    <section id="teleop-page" class="page">
      <div class="teleop-grid">
        <section class="panel camera" aria-label="Live camera feed">
          <img id="camera-feed" src="/stream.mjpg" alt="Live camera feed">
          <div id="camera-placeholder" class="camera-placeholder" style="display:none">Waiting for camera frames from /camera/image_raw/compressed</div>
        </section>
        <section class="panel">
          <div class="panel-body" style="display:grid; gap:14px">
            <h2>Teleoperation</h2>
            <div class="control-row">
              <span>Speed</span>
              <select id="speed-mode">
                <option value="slow">Slow 0.15 m/s</option>
                <option value="normal" selected>Normal 0.30 m/s</option>
                <option value="fast">Fast 0.50 m/s</option>
              </select>
            </div>
            <div class="pad">
              <button data-control="rotate-left">Rotate Left</button>
              <span class="empty"></span>
              <button data-control="rotate-right">Rotate Right</button>
              <button data-control="strafe-left">Strafe Left</button>
              <button data-control="forward">Forward</button>
              <button data-control="strafe-right">Strafe Right</button>
              <span class="empty"></span>
              <button data-control="backward">Back</button>
              <span class="empty"></span>
            </div>
            <div class="control-row"><span>Command</span><strong id="teleop-state">idle</strong></div>
            <div class="control-row"><span>Twist topic</span><strong id="cmd-topic">/mirte_base_controller/cmd_vel_unstamped</strong></div>
            <div class="control-row"><span>Camera</span><strong id="camera-state">waiting</strong></div>
            <div class="control-row"><span>Keyboard</span><strong>W/S A/D Q/E</strong></div>
          </div>
        </section>
      </div>
    </section>
  </main>

  <script>
    const state = {
      selectedFlower: null,
      page: "dashboard",
      activeControls: new Set(),
      activeSources: new Map(),
      pressedKeys: new Set()
    };
    const pages = {
      dashboard: document.getElementById("dashboard-page"),
      teleop: document.getElementById("teleop-page"),
    };

    function showPage(page) {
      state.page = page;
      pages.dashboard.classList.toggle("active", page === "dashboard");
      pages.teleop.classList.toggle("active", page === "teleop");
      document.getElementById("to-teleop").style.display = page === "dashboard" ? "" : "none";
      document.getElementById("to-dashboard").style.display = page === "teleop" ? "" : "none";
      if (page === "dashboard") {
        stopTeleop();
      }
    }

    document.getElementById("to-teleop").addEventListener("click", () => showPage("teleop"));
    document.getElementById("to-dashboard").addEventListener("click", () => showPage("dashboard"));

    function describeActiveControls() {
      if (!state.activeControls.size) return "idle";
      return Array.from(state.activeControls).sort().join(" + ");
    }
    function refreshActiveControls() {
      state.activeControls = new Set(state.activeSources.values());
    }

    async function sendTeleop() {
      const controls = Array.from(state.activeControls);
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
    function setButtonActive(button, active) {
      button.classList.toggle("active", active);
    }
    function ensureRepeat() {
      if (!repeatTimer) repeatTimer = setInterval(sendTeleop, 250);
    }
    function stopTeleop() {
      clearRepeat();
      state.activeSources.clear();
      refreshActiveControls();
      state.pressedKeys.clear();
      document.querySelectorAll("button[data-control]").forEach((button) => {
        setButtonActive(button, false);
      });
      sendTeleop();
    }
    function startControl(event) {
      event.preventDefault();
      const button = event.currentTarget;
      const control = button.dataset.control;
      state.activeSources.set(`button:${control}`, control);
      refreshActiveControls();
      setButtonActive(button, true);
      sendTeleop();
      ensureRepeat();
    }
    function stopControl(event) {
      if (event) event.preventDefault();
      const button = event ? event.currentTarget : null;
      if (button) {
        state.activeSources.delete(`button:${button.dataset.control}`);
        refreshActiveControls();
        setButtonActive(button, false);
      }
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
      if (state.page !== "teleop" || isTypingTarget(event.target)) return;
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
      if (isTypingTarget(event.target) && !state.pressedKeys.has(event.code)) return;
      if (stopKeys.has(event.code)) {
        event.preventDefault();
        return;
      }
      const command = keyCommands[event.code];
      if (!command) return;
      event.preventDefault();
      state.pressedKeys.delete(event.code);
      state.activeSources.delete(`key:${event.code}`);
      refreshActiveControls();
      if (!state.activeControls.size) clearRepeat();
      sendTeleop();
    });

    async function inspectFlower(flowerId) {
      await fetch("/api/inspect_bed", {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify({bedId: flowerId.slice(0, 1)})
      });
      const confirmation = document.getElementById("inspect-confirmation");
      if (confirmation) confirmation.textContent = `Inspect command sent for ${flowerId}`;
    }

    function selectFlower(flower, data) {
      state.selectedFlower = flower.flower_id;
      renderFlowerDetail(flower, data);
      renderDummyMap(data);
      renderFlowerList(data);
    }

    function renderDummyMap(data) {
      const map = document.getElementById("greenhouse-map");
      map.innerHTML = `
        <div class="map-title">
          <span>2D Greenhouse Map</span>
          <span>${data.flowers.length} flowers across ${data.beds.length} beds</span>
        </div>
        <span class="robot-marker" title="Robot placeholder"></span>
      `;
      data.beds.forEach((bed) => {
        const zone = document.createElement("section");
        zone.className = "bed-zone";
        zone.innerHTML = `<div class="bed-zone-label">Bed ${bed.bed_id}</div><div class="flower-grid"></div>`;
        const flowerGrid = zone.querySelector(".flower-grid");
        data.flowers.filter((flower) => flower.bed_id === bed.bed_id).forEach((flower) => {
          const marker = document.createElement("button");
          marker.className = "flower-marker";
          marker.dataset.health = flower.health || "unknown";
          marker.classList.toggle("selected", state.selectedFlower === flower.flower_id);
          marker.textContent = flower.flower_id;
          marker.title = `${flower.flower_id}: ${flower.health}, ${flower.height_cm} cm, ${flower.color}`;
          marker.addEventListener("click", () => selectFlower(flower, data));
          flowerGrid.appendChild(marker);
        });
        map.appendChild(zone);
      });
    }

    function renderBedSummaries(data) {
      const summaries = document.getElementById("bed-summaries");
      summaries.innerHTML = "";
      data.beds.forEach((bed) => {
        const card = document.createElement("article");
        card.className = "bed-card";
        card.innerHTML = `
          <h3>Bed ${bed.bed_id}</h3>
          <p><strong>${bed.totalFlowers}</strong> total flowers</p>
          <p><strong>${bed.averageHeightCm} cm</strong> average height</p>
          <p><strong>${bed.healthyFlowers}</strong> healthy</p>
          <p><strong>${bed.riskFlowers}</strong> warning / critical</p>
          <p><strong>${bed.readyFlowers}</strong> ready for harvest</p>
        `;
        summaries.appendChild(card);
      });
    }

    function renderFlowerList(data) {
      const existing = document.getElementById("flower-list");
      if (!existing) return;
      existing.innerHTML = "";
      data.flowers.forEach((flower) => {
        const button = document.createElement("button");
        button.textContent = flower.flower_id;
        button.classList.toggle("active", flower.flower_id === state.selectedFlower);
        button.addEventListener("click", () => selectFlower(flower, data));
        existing.appendChild(button);
      });
    }

    function renderFlowerDetail(flower, data) {
      const detail = document.getElementById("flower-detail");
      detail.innerHTML = `
        <h2>Flower ${flower.flower_id}</h2>
        <p><strong>Bed:</strong> ${flower.bed_id}</p>
        <p><strong>Height:</strong> ${flower.height_cm} cm</p>
        <p><strong>Color:</strong> ${flower.color}</p>
        <p><strong>Health:</strong> ${flower.health}</p>
        <p><strong>Growth stage:</strong> ${flower.growth_stage}</p>
        <p><strong>Bug detected:</strong> ${flower.bug_detected ? "yes" : "no"}</p>
        <p><strong>Ready for harvest:</strong> ${flower.ready_for_harvest ? "yes" : "no"}</p>
        <p><strong>Confidence:</strong> ${Math.round((flower.confidence || 0) * 100)}%</p>
        <p><strong>Last scan:</strong> ${flower.last_scan_time || "unknown"}</p>
        <p><strong>Notes:</strong> ${flower.notes || "none"}</p>
        <button id="inspect-bed">Inspect this flower's bed</button>
        <p id="inspect-confirmation"></p>
        <div class="flower-list" id="flower-list"></div>
      `;
      document.getElementById("inspect-bed").addEventListener("click", () => inspectFlower(flower.flower_id));
      renderFlowerList(data);
    }

    function renderReport(report) {
      document.getElementById("bed-count").textContent = report.totalBeds;
      document.getElementById("flower-count").textContent = report.totalFlowers;
      document.getElementById("healthy-count").textContent = report.healthyBeds;
      document.getElementById("risk-count").textContent = report.riskBeds;
      document.getElementById("ready-count").textContent = report.readyBeds;
      document.getElementById("average-height").textContent = `${report.averageHeightCm} cm`;
      document.getElementById("last-scan").textContent = report.lastScanTime || "unknown";
      document.getElementById("next-action").textContent = report.nextAction;
    }

    function drawMap(data) {
      const canvas = document.getElementById("map-canvas");
      const greenhouse = document.getElementById("greenhouse-map");
      if (!data.map || !data.map.width || !data.map.height) {
        canvas.style.display = "none";
        greenhouse.style.display = "grid";
        return;
      }
      greenhouse.style.display = "none";
      canvas.style.display = "block";
      const ctx = canvas.getContext("2d");
      const width = data.map.width;
      const height = data.map.height;
      const cell = Math.max(1, Math.floor(Math.min(canvas.width / width, canvas.height / height)));
      ctx.fillStyle = "#0a0d0c";
      ctx.fillRect(0, 0, canvas.width, canvas.height);
      const x0 = Math.floor((canvas.width - width * cell) / 2);
      const y0 = Math.floor((canvas.height - height * cell) / 2);
      for (let y = 0; y < height; y++) {
        for (let x = 0; x < width; x++) {
          const value = data.map.data[y * width + x];
          ctx.fillStyle = value < 0 ? "#26312d" : value > 50 ? "#d8e0dc" : "#111815";
          ctx.fillRect(x0 + x * cell, y0 + (height - 1 - y) * cell, cell, cell);
        }
      }
      if (data.robotPose && Number.isFinite(data.robotPose.x) && data.map.resolution) {
        const rx = Math.round((data.robotPose.x - data.map.origin.x) / data.map.resolution);
        const ry = Math.round((data.robotPose.y - data.map.origin.y) / data.map.resolution);
        ctx.fillStyle = "#66a8d9";
        ctx.beginPath();
        ctx.arc(x0 + rx * cell, y0 + (height - 1 - ry) * cell, 8, 0, Math.PI * 2);
        ctx.fill();
      }
    }

    async function refreshStatus() {
      try {
        const response = await fetch("/api/status");
        const data = await response.json();
        document.getElementById("connection").textContent = data.dummyMode
          ? "Dummy mode"
          : (data.rosConnected ? "ROS connected" : "ROS waiting");
        document.getElementById("battery").textContent = data.batteryPercent == null
          ? "Battery --"
          : `Battery ${Math.round(data.batteryPercent)}%`;
        document.getElementById("cmd-topic").textContent = data.topics.cmdVel;
        document.getElementById("camera-state").textContent = data.cameraReady ? "online" : "waiting";
        document.getElementById("camera-feed").style.display = data.cameraReady ? "block" : "none";
        document.getElementById("camera-placeholder").style.display = data.cameraReady ? "none" : "block";
        if (!state.selectedFlower && data.flowers.length) state.selectedFlower = data.flowers[0].flower_id;
        renderDummyMap(data);
        renderBedSummaries(data);
        if (state.selectedFlower) {
          const selected = data.flowers.find((flower) => flower.flower_id === state.selectedFlower);
          if (selected) renderFlowerDetail(selected, data);
        }
        renderReport(data.report);
        drawMap(data);
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
    if 1 <= port <= 65535:
        return port
    return DEFAULT_WEB_PORT


def request_hostname(host_header: str) -> str:
    if not host_header:
        return "localhost"
    host = host_header.rsplit("@", 1)[-1].strip()
    if host.startswith("[") and "]" in host:
        return host[1 : host.index("]")] or "localhost"
    return host.rsplit(":", 1)[0] or "localhost"


def dynamic_rosbridge_url(configured_url: str, host_header: str) -> str:
    """Use the current browser host when config points at local loopback."""
    parsed = urlparse(configured_url or "")
    if parsed.scheme not in {"ws", "wss"} or not parsed.hostname:
        return configured_url
    if parsed.hostname not in {"localhost", "127.0.0.1", "::1"}:
        return configured_url
    hostname = request_hostname(host_header)
    port = parsed.port or 9090
    netloc = f"[{hostname}]:{port}" if ":" in hostname else f"{hostname}:{port}"
    return urlunparse(
        (
            parsed.scheme,
            netloc,
            parsed.path or "",
            parsed.params,
            parsed.query,
            parsed.fragment,
        )
    )


def default_flowers() -> dict:
    colors = ("purple", "yellow", "red", "white", "pink")
    data = {}
    scan_time = datetime.now(timezone.utc).isoformat(timespec="seconds")
    running_index = 0
    for bed_id, count in FLOWER_COUNTS.items():
        for number in range(1, count + 1):
            running_index += 1
            flower_id = f"{bed_id}{number}"
            health = "healthy"
            if flower_id in {"B7", "C15"}:
                health = "critical"
            elif number in {5, 11, 18}:
                health = "warning"
            ready = number in {4, 8, 12, 16, 20, 22}
            growth_stage = "ready" if ready else ("seedling" if number % 9 == 0 else "growing")
            data[flower_id] = {
                "flower_id": flower_id,
                "bed_id": bed_id,
                "height_cm": round(24.0 + (number % 7) * 2.4 + (ord(bed_id) - 65) * 1.6, 1),
                "color": colors[(running_index + number) % len(colors)],
                "health": health,
                "growth_stage": growth_stage,
                "bug_detected": flower_id in {"B7", "C15"},
                "flower_detected": True,
                "ready_for_harvest": ready,
                "confidence": round(0.72 + (number % 6) * 0.035, 2),
                "last_scan_time": scan_time,
                "notes": "Inspect for pests" if health == "critical" else (
                    "Ready for harvest" if ready else "Normal growth"
                ),
            }
    return data


def summarize_beds(flowers: dict) -> list:
    summaries = []
    for bed_id in BED_IDS:
        bed_flowers = [
            flower for flower in flowers.values() if flower.get("bed_id") == bed_id
        ]
        total = len(bed_flowers)
        average_height = 0.0
        if total:
            average_height = sum(
                float(flower.get("height_cm") or 0.0) for flower in bed_flowers
            ) / total
        summaries.append(
            {
                "bed_id": bed_id,
                "totalFlowers": total,
                "averageHeightCm": round(average_height, 1),
                "healthyFlowers": sum(
                    1 for flower in bed_flowers if flower.get("health") == "healthy"
                ),
                "riskFlowers": sum(
                    1
                    for flower in bed_flowers
                    if flower.get("health") in {"warning", "critical"}
                ),
                "readyFlowers": sum(
                    1 for flower in bed_flowers if flower.get("ready_for_harvest")
                ),
            }
        )
    return summaries


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class UiNode(Node):
    """Embedded operator web UI for dashboard, camera, and safe teleoperation."""

    def __init__(self) -> None:
        super().__init__("ui_node")
        self._config = load_config()
        self.declare_parameter(
            "web_host",
            os.getenv("SIMBIOSYS_UI_HOST", DEFAULT_WEB_HOST),
        )
        self.declare_parameter("web_port", env_web_port())
        self.declare_parameter("dummy_mode", bool(self._config["dummyMode"]))

        parameter_dummy = self.get_parameter("dummy_mode").value
        self._config["dummyMode"] = bool(parameter_dummy)
        self._topics = self._config["topics"]
        self.declare_parameter("cmd_vel_topic", self._topics["cmdVel"])
        self.declare_parameter("image_topic", self._topics["cameraRaw"])
        self.declare_parameter(
            "compressed_image_topic",
            self._topics["cameraCompressed"],
        )
        self._apply_topic_parameters()

        self._web_host = self.get_parameter("web_host").value
        self._web_port = int(self.get_parameter("web_port").value)
        self._bridge = CvBridge() if CvBridge is not None else None
        self._frame_lock = threading.Lock()
        self._latest_jpeg = None
        self._last_compressed_frame_time = 0.0
        self._last_command_time = 0.0
        self._active_controls = set()
        self._speed_mode = "normal"
        self._flowers = default_flowers()
        self._map = None
        self._robot_pose = {"x": 1.5, "y": 1.0, "yaw": 0.0}
        self._battery_percent = 86.0 if self._config["dummyMode"] else None
        self._external_report = None
        self._bed_observations = {}
        self._last_dummy_tick = 0

        self._cmd_vel_publisher = self.create_publisher(
            Twist,
            self._topics["cmdVel"],
            10,
        )
        self._inspect_bed_publisher = self.create_publisher(
            String,
            self._topics["inspectBedCommand"],
            10,
        )

        self.create_subscription(
            Image,
            self._topics["cameraRaw"],
            self._on_image,
            10,
        )
        self.create_subscription(
            CompressedImage,
            self._topics["cameraCompressed"],
            self._on_compressed_image,
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            self._topics["map"],
            self._on_map,
            10,
        )
        self.create_subscription(Odometry, self._topics["odom"], self._on_odom, 10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            self._topics["amclPose"],
            self._on_amcl_pose,
            10,
        )
        self.create_subscription(
            String,
            self._topics["plantHealth"],
            self._on_plant_health,
            10,
        )
        if PlantHealth is not None:
            self.create_subscription(
                PlantHealth,
                "simbiosys/plant_health",
                self._on_typed_plant_health,
                10,
            )
        if BedObservation is not None:
            self.create_subscription(
                BedObservation,
                self._topics.get("bedObservation", "simbiosys/bed_observation"),
                self._on_bed_observation,
                10,
            )
        self.create_subscription(
            String,
            self._topics["plantHealthReport"],
            self._on_plant_health_report,
            10,
        )
        self.create_subscription(
            BatteryState,
            self._topics["battery"],
            self._on_battery,
            10,
        )
        if FlowerData is not None:
            self.create_subscription(
                FlowerData,
                "simbiosys/flower_data",
                self._on_legacy_flower_data,
                10,
            )

        self.create_timer(0.1, self._on_teleop_timer)
        self.create_timer(5.0, self._on_dummy_timer)

        self._http_server = ThreadingHTTPServer(
            (self._web_host, self._web_port),
            self._make_handler(),
        )
        self._http_thread = threading.Thread(
            target=self._http_server.serve_forever,
            daemon=True,
        )
        self._http_thread.start()

        self._log_startup_urls()
        self.get_logger().info(
            f"Publishing Twist to {self._topics['cmdVel']}; dummy mode: "
            f"{self._config['dummyMode']}"
        )

    def _apply_topic_parameters(self) -> None:
        configured_raw_image = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        configured_compressed_image = (
            self.get_parameter("compressed_image_topic")
            .get_parameter_value()
            .string_value
        )
        if (
            configured_raw_image != self._topics["cameraRaw"]
            and configured_compressed_image == self._topics["cameraCompressed"]
        ):
            configured_compressed_image = f"{configured_raw_image}/compressed"

        self._topics["cmdVel"] = (
            self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        )
        self._topics["cameraRaw"] = configured_raw_image
        self._topics["cameraCompressed"] = configured_compressed_image

    def destroy_node(self) -> bool:
        try:
            self._publish_twist(0.0, 0.0, 0.0)
        except Exception:
            pass
        self._http_server.shutdown()
        self._http_server.server_close()
        return super().destroy_node()

    def _detect_lan_ip(self) -> str:
        """Best-effort LAN IP detection for startup guidance."""
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
        self.get_logger().info(
            f"Web UI listening on {self._web_host}:{self._web_port}"
        )
        self.get_logger().info(f"Local URL: http://localhost:{self._web_port}")
        if self._web_host in {"127.0.0.1", "localhost", "::1"}:
            self.get_logger().info(
                "LAN access is disabled because the UI is bound to a loopback host."
            )
            return
        lan_ip = self._detect_lan_ip()
        if lan_ip:
            self.get_logger().info(
                f"LAN URL: http://{lan_ip}:{self._web_port} "
                "(same trusted local network only)"
            )
        else:
            self.get_logger().info(
                "LAN IP not detected. Run `hostname -I` and open "
                f"http://<LAPTOP_LAN_IP>:{self._web_port} from another device."
            )

    def _make_handler(self):
        node = self

        class RequestHandler(BaseHTTPRequestHandler):
            def log_message(self, _format, *_args):
                return

            def do_GET(self):
                if self.path in ("/", "/index.html", "/teleop"):
                    self._send_html()
                elif self.path == "/api/status":
                    self._send_json(node.status_payload(self.headers.get("Host", "")))
                elif self.path == "/stream.mjpg":
                    self._send_stream()
                else:
                    self.send_error(HTTPStatus.NOT_FOUND)

            def do_POST(self):
                if self.path not in {"/api/teleop", "/api/inspect_bed"}:
                    self.send_error(HTTPStatus.NOT_FOUND)
                    return
                length = int(self.headers.get("Content-Length", "0"))
                body = self.rfile.read(length)
                try:
                    payload = json.loads(body.decode("utf-8")) if body else {}
                except json.JSONDecodeError:
                    self.send_error(HTTPStatus.BAD_REQUEST, "Invalid JSON")
                    return

                if self.path == "/api/teleop":
                    accepted = node.set_teleop_command(payload)
                    if not accepted:
                        self.send_error(HTTPStatus.BAD_REQUEST, "Unknown command")
                        return
                    self._send_json({"ok": True})
                    return

                accepted = node.inspect_bed(payload.get("bedId", ""))
                if not accepted:
                    self.send_error(HTTPStatus.BAD_REQUEST, "Unknown bed")
                    return
                self._send_json({"ok": True})

            def _send_html(self):
                content = INDEX_HTML.encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(content)))
                self.end_headers()
                self.wfile.write(content)

            def _send_json(self, payload):
                content = json.dumps(payload).encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(content)))
                self.end_headers()
                self.wfile.write(content)

            def _send_stream(self):
                self.send_response(HTTPStatus.OK)
                self.send_header(
                    "Content-Type",
                    "multipart/x-mixed-replace; boundary=frame",
                )
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                while True:
                    frame = node.latest_frame()
                    if frame is None:
                        time.sleep(0.2)
                        continue
                    try:
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(
                            f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii")
                        )
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")
                        time.sleep(0.08)
                    except (BrokenPipeError, ConnectionResetError):
                        break

        return RequestHandler

    def latest_frame(self):
        with self._frame_lock:
            return self._latest_jpeg

    def status_payload(self, host_header: str = "") -> dict:
        return {
            "dummyMode": bool(self._config["dummyMode"]),
            "rosConnected": True,
            "rosbridgeUrl": dynamic_rosbridge_url(
                self._config["rosbridgeUrl"],
                host_header,
            ),
            "topics": self._topics,
            "cameraReady": self.latest_frame() is not None,
            "beds": summarize_beds(self._flowers),
            "flowers": list(self._flowers.values()),
            "report": self._external_report or self._computed_report(),
            "map": self._map,
            "robotPose": self._robot_pose,
            "batteryPercent": self._battery_percent,
            "bedObservations": copy.deepcopy(self._bed_observations),
            "teleop": sorted(self._active_controls),
        }

    def set_teleop_command(self, payload) -> bool:
        if not isinstance(payload, dict):
            return False
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
                return False
            controls = legacy_controls[command]
        if not isinstance(controls, (list, tuple, set)):
            return False
        allowed = {
            "forward",
            "backward",
            "strafe-left",
            "strafe-right",
            "rotate-left",
            "rotate-right",
        }
        active_controls = {str(control).strip().lower() for control in controls}
        if not active_controls.issubset(allowed):
            return False
        self._active_controls = active_controls
        self._speed_mode = speed_mode
        self._last_command_time = time.monotonic()
        self._publish_active_twist()
        return True

    def inspect_bed(self, bed_id: str) -> bool:
        bed_id = str(bed_id).strip().upper()[0:1]
        if bed_id not in BED_IDS:
            return False
        msg = String()
        msg.data = bed_id
        self._inspect_bed_publisher.publish(msg)
        self.get_logger().info(f"Inspect bed command sent: {bed_id}")
        return True

    def _on_image(self, msg: Image) -> None:
        if self._bridge is None or cv2 is None:
            return
        if time.monotonic() - self._last_compressed_frame_time < 2.0:
            return
        try:
            image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            success, buffer = cv2.imencode(".jpg", image)
        except Exception as exc:
            self.get_logger().warn(f"Could not encode camera frame: {exc}")
            return
        if success:
            with self._frame_lock:
                self._latest_jpeg = buffer.tobytes()

    def _on_compressed_image(self, msg: CompressedImage) -> None:
        if msg.format and "jpeg" not in msg.format.lower() and "jpg" not in msg.format.lower():
            if cv2 is None:
                return
        with self._frame_lock:
            self._latest_jpeg = bytes(msg.data)
            self._last_compressed_frame_time = time.monotonic()

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
        }

    def _on_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        self._robot_pose = {
            "x": pose.position.x,
            "y": pose.position.y,
            "yaw": yaw_from_quaternion(pose.orientation),
        }

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        self._robot_pose = {
            "x": pose.position.x,
            "y": pose.position.y,
            "yaw": yaw_from_quaternion(pose.orientation),
        }

    def _on_plant_health(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Ignoring invalid plant health JSON: {exc}")
            return

        flower_id = str(payload.get("flower_id", "")).upper()
        if flower_id:
            self._update_flower_from_payload(flower_id, payload)
            self._external_report = None
            return

        bed_id = str(payload.get("bed_id", "")).upper()
        if bed_id not in BED_IDS:
            self.get_logger().warn(
                f"Ignoring legacy plant health for unknown bed: {bed_id}"
            )
            return
        for flower in self._flowers.values():
            if flower.get("bed_id") != bed_id:
                continue
            for key in ("health", "growth_stage", "confidence", "last_scan_time", "notes"):
                if key in payload:
                    flower[key] = payload[key]
            if "bug_detected" in payload:
                flower["bug_detected"] = bool(payload["bug_detected"])
            if "flower_detected" in payload:
                flower["flower_detected"] = bool(payload["flower_detected"])
            if "ready_for_harvest" in payload:
                flower["ready_for_harvest"] = bool(payload["ready_for_harvest"])
        self.get_logger().info(
            f"Applied legacy bed-level plant health update to Bed {bed_id}"
        )
        self._external_report = None

    def _on_typed_plant_health(self, msg) -> None:
        flower_id = str(msg.flower_id).upper()
        if not flower_id:
            flower_id = f"{str(msg.bed_id).upper() or 'A'}1"

        payload = {
            "flower_id": flower_id,
            "bed_id": str(msg.bed_id).upper() or flower_id[0:1],
            "height_cm": round(float(msg.height_cm), 1),
            "color": msg.color or "unknown",
            "health": msg.health or "unknown",
            "growth_stage": msg.growth_stage or "unknown",
            "bug_detected": bool(msg.bug_detected),
            "flower_detected": bool(msg.flower_detected),
            "ready_for_harvest": bool(msg.ready_for_harvest),
            "confidence": float(msg.confidence),
            "last_scan_time": self._time_msg_to_iso(msg.last_scan_time),
            "notes": msg.notes,
        }
        self._update_flower_from_payload(flower_id, payload)
        self._external_report = None

    def _on_bed_observation(self, msg) -> None:
        bed_id = str(msg.bed_id)
        if msg.bed_id < 0:
            bed_id = "none"
        self._bed_observations[bed_id] = {
            "bed_id": msg.bed_id,
            "visible": bool(msg.visible),
            "message": msg.message,
            "tags": [
                {
                    "id": tag.id,
                    "center_x": tag.center_px.x,
                    "center_y": tag.center_px.y,
                    "area": tag.area,
                    "confidence": tag.confidence,
                }
                for tag in msg.tags
            ],
            "last_seen": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        }

    def _time_msg_to_iso(self, msg) -> str:
        seconds = int(msg.sec)
        nanoseconds = int(msg.nanosec)
        if seconds <= 0 and nanoseconds <= 0:
            return datetime.now(timezone.utc).isoformat(timespec="seconds")
        return datetime.fromtimestamp(
            seconds + nanoseconds / 1_000_000_000.0,
            tz=timezone.utc,
        ).isoformat(timespec="seconds")

    def _update_flower_from_payload(self, flower_id: str, payload: dict) -> None:
        bed_id = str(payload.get("bed_id", flower_id[0:1])).upper()
        if bed_id not in BED_IDS:
            self.get_logger().warn(f"Ignoring flower health for unknown bed: {bed_id}")
            return
        if flower_id not in self._flowers:
            self._flowers[flower_id] = {
                "flower_id": flower_id,
                "bed_id": bed_id,
                "height_cm": 0.0,
                "color": "unknown",
                "health": "unknown",
                "growth_stage": "unknown",
                "bug_detected": False,
                "flower_detected": False,
                "ready_for_harvest": False,
                "confidence": 0.0,
                "last_scan_time": "",
                "notes": "",
            }
        flower = self._flowers[flower_id]
        flower["bed_id"] = bed_id
        for key in (
            "height_cm",
            "color",
            "health",
            "growth_stage",
            "confidence",
            "last_scan_time",
            "notes",
        ):
            if key in payload:
                flower[key] = payload[key]
        for key in ("bug_detected", "flower_detected", "ready_for_harvest"):
            if key in payload:
                flower[key] = bool(payload[key])
        if not flower.get("last_scan_time"):
            flower["last_scan_time"] = datetime.now(timezone.utc).isoformat(
                timespec="seconds"
            )

    def _on_plant_health_report(self, msg: String) -> None:
        try:
            report = json.loads(msg.data)
        except json.JSONDecodeError:
            report = {"nextAction": msg.data}
        self._external_report = {
            "totalBeds": report.get("totalBeds", report.get("total_beds", len(BED_IDS))),
            "healthyBeds": report.get("healthyFlowers", report.get("healthy_flowers", 0)),
            "riskBeds": report.get("riskFlowers", report.get("risk_flowers", 0)),
            "readyBeds": report.get("readyFlowers", report.get("ready_flowers", 0)),
            "totalFlowers": report.get("totalFlowers", report.get("total_flowers", 0)),
            "averageHeightCm": report.get(
                "averageHeightCm", report.get("average_height_cm", 0)
            ),
            "lastScanTime": report.get("lastScanTime", report.get("last_scan_time", "")),
            "nextAction": report.get("nextAction", report.get("next_action", "Review report")),
        }

    def _on_battery(self, msg: BatteryState) -> None:
        if msg.percentage >= 0:
            self._battery_percent = max(0.0, min(100.0, float(msg.percentage) * 100.0))

    def _on_legacy_flower_data(self, msg) -> None:
        if not msg.detected:
            return
        flower = self._flowers["A1"]
        flower["flower_detected"] = True
        flower["confidence"] = max(
            float(msg.confidence), float(flower.get("confidence", 0.0))
        )
        flower["notes"] = msg.message or "Legacy flower detector reported a flower"

    def _on_teleop_timer(self) -> None:
        if (
            self._active_controls
            and time.monotonic() - self._last_command_time > 0.6
        ):
            self._active_controls = set()
        self._publish_active_twist()

    def _publish_active_twist(self) -> None:
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

    def _on_dummy_timer(self) -> None:
        if not self._config["dummyMode"]:
            return
        self._last_dummy_tick += 1
        moving_flower = "A1" if self._last_dummy_tick % 2 else "C15"
        self._flowers[moving_flower]["last_scan_time"] = datetime.now(
            timezone.utc
        ).isoformat(timespec="seconds")
        self._flowers[moving_flower]["confidence"] = min(
            0.99, float(self._flowers[moving_flower].get("confidence", 0.8)) + 0.01
        )
        self._battery_percent = max(20.0, float(self._battery_percent or 86.0) - 0.2)
        self._robot_pose = {
            "x": 1.5 + math.sin(self._last_dummy_tick / 3.0) * 0.5,
            "y": 1.0 + math.cos(self._last_dummy_tick / 3.0) * 0.3,
            "yaw": self._last_dummy_tick / 8.0,
        }

    def _computed_report(self) -> dict:
        flowers = list(self._flowers.values())
        healthy = sum(1 for flower in flowers if flower.get("health") == "healthy")
        risk = sum(
            1 for flower in flowers if flower.get("health") in {"warning", "critical"}
        )
        ready = sum(1 for flower in flowers if flower.get("ready_for_harvest"))
        total = len(flowers)
        average_height = 0.0
        if total:
            average_height = sum(
                float(flower.get("height_cm") or 0.0) for flower in flowers
            ) / total
        last_scan = max((flower.get("last_scan_time", "") for flower in flowers), default="")
        priority = next((flower for flower in flowers if flower.get("health") == "critical"), None)
        if priority is None:
            priority = next((flower for flower in flowers if flower.get("health") == "warning"), None)
        if priority is None:
            priority = next((flower for flower in flowers if flower.get("ready_for_harvest")), None)
        if priority:
            if priority.get("health") in {"critical", "warning"}:
                reason = priority["health"]
            else:
                reason = "ready for harvest"
            action = f"Inspect flower {priority['flower_id']} in Bed {priority['bed_id']} for {reason}"
        else:
            action = "No urgent action required."
        return {
            "totalBeds": len(BED_IDS),
            "healthyBeds": healthy,
            "riskBeds": risk,
            "readyBeds": ready,
            "totalFlowers": total,
            "averageHeightCm": round(average_height, 1),
            "lastScanTime": last_scan,
            "nextAction": action,
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
