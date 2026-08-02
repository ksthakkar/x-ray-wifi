(function () {
  "use strict";

  const canvas = document.getElementById("overlay");
  const context = canvas.getContext("2d");
  const hud = document.getElementById("hud");
  const calibrationPanel = document.getElementById("calibration");
  const geometry = globalThis.XrwGeometry;

  let siteConfig = null;
  let view = null;
  let estimate = null;
  let lastMessageMs = -Infinity;
  let connectionState = "starting";
  let reconnectDelayMs = 500;
  let showHud = false;
  let calibrationMode = false;
  let calibration = { yaw: 0, pitch: 0, fov: 0 };

  function resizeCanvas() {
    const rect = canvas.getBoundingClientRect();
    const ratio = window.devicePixelRatio || 1;
    canvas.width = Math.max(1, Math.round(rect.width * ratio));
    canvas.height = Math.max(1, Math.round(rect.height * ratio));
  }

  function viewport() {
    const rect = canvas.getBoundingClientRect();
    return { width: rect.width, height: rect.height };
  }

  function loadCalibration() {
    try {
      const saved = JSON.parse(localStorage.getItem("xrw-fixed-view-calibration"));
      if (saved && ["yaw", "pitch", "fov"].every((key) => Number.isFinite(saved[key]))) {
        calibration = saved;
      }
    } catch (_error) {
      localStorage.removeItem("xrw-fixed-view-calibration");
    }
  }

  function applyCalibration() {
    view = {
      ...siteConfig.display,
      yaw_deg: siteConfig.display.yaw_deg + calibration.yaw,
      pitch_deg: siteConfig.display.pitch_deg + calibration.pitch,
      horizontal_fov_deg: Math.max(
        10,
        Math.min(120, siteConfig.display.horizontal_fov_deg + calibration.fov)
      ),
    };
    localStorage.setItem("xrw-fixed-view-calibration", JSON.stringify(calibration));
  }

  function drawLine(start, finish, color, width) {
    if (!start || !finish) return;
    context.strokeStyle = color;
    context.lineWidth = width;
    context.beginPath();
    context.moveTo(start.x, start.y);
    context.lineTo(finish.x, finish.y);
    context.stroke();
  }

  function drawWallGrid(width, height) {
    if (!view || (!view.show_wall_grid && !calibrationMode)) return;
    const wall = siteConfig.wall;
    const color = calibrationMode ? "rgba(255,211,106,0.55)" : "rgba(70,190,255,0.10)";
    const lineWidth = calibrationMode ? 1.5 : 1;
    const y = wall.y_m || 0;
    const xStep = 0.5;
    const zStep = 0.5;
    for (let x = 0; x <= wall.width_m + 1e-6; x += xStep) {
      drawLine(
        geometry.projectPoint([x, y, 0], view, width, height),
        geometry.projectPoint([x, y, wall.height_m], view, width, height),
        color,
        lineWidth
      );
    }
    for (let z = 0; z <= wall.height_m + 1e-6; z += zStep) {
      drawLine(
        geometry.projectPoint([0, y, z], view, width, height),
        geometry.projectPoint([wall.width_m, y, z], view, width, height),
        color,
        lineWidth
      );
    }
  }

  function drawEstimate(projected) {
    const { center, ellipse } = projected;
    const quality = Math.max(0, Math.min(1, estimate.quality));
    if (ellipse) {
      context.save();
      context.translate(center.x, center.y);
      context.rotate(ellipse.angle);
      context.beginPath();
      context.ellipse(
        0,
        0,
        Math.min(ellipse.major, 800),
        Math.min(ellipse.minor, 800),
        0,
        0,
        Math.PI * 2
      );
      context.fillStyle = `rgba(0,190,255,${0.035 + quality * 0.045})`;
      context.fill();
      context.strokeStyle = `rgba(80,225,255,${0.20 + quality * 0.35})`;
      context.lineWidth = 2;
      context.shadowColor = "#00bfff";
      context.shadowBlur = 18;
      context.stroke();
      context.restore();
    }

    const radius = Math.max(14, Math.min(38, 48 / Math.sqrt(center.depth_m)));
    const glow = context.createRadialGradient(
      center.x, center.y, 0, center.x, center.y, radius * 2.8
    );
    glow.addColorStop(0, `rgba(180,250,255,${0.82 + quality * 0.18})`);
    glow.addColorStop(0.24, `rgba(0,220,255,${0.60 + quality * 0.25})`);
    glow.addColorStop(1, "rgba(0,140,255,0)");
    context.fillStyle = glow;
    context.beginPath();
    context.arc(center.x, center.y, radius * 2.8, 0, Math.PI * 2);
    context.fill();
  }

  function updatePanels(nowMs) {
    const age = Number.isFinite(lastMessageMs) ? Math.max(0, nowMs - lastMessageMs) : Infinity;
    const status = age > (view?.stale_after_ms || 1000) ? "stale" : connectionState;
    hud.classList.toggle("hidden", !showHud && !calibrationMode);
    hud.textContent = [
      `stream: ${status}`,
      `age: ${Number.isFinite(age) ? Math.round(age) + " ms" : "-"}`,
      `nodes: ${estimate ? estimate.nodes_online.length + "/" + estimate.nodes_expected.length : "-"}`,
      `quality: ${estimate ? estimate.quality.toFixed(2) : "-"}`,
      "H HUD   C calibrate   F fullscreen",
    ].join("\n");

    calibrationPanel.classList.toggle("hidden", !calibrationMode);
    calibrationPanel.textContent = [
      "CALIBRATION — stand on the marked point and recenter Anchor mode",
      "←/→ yaw   ↑/↓ pitch   +/- field of view   R reset   C finish",
      `yaw offset: ${calibration.yaw.toFixed(2)}°`,
      `pitch offset: ${calibration.pitch.toFixed(2)}°`,
      `effective horizontal FOV: ${view ? view.horizontal_fov_deg.toFixed(1) : "-"}°`,
    ].join("\n");
  }

  function render(nowMs) {
    const ratio = window.devicePixelRatio || 1;
    const { width, height } = viewport();
    context.setTransform(ratio, 0, 0, ratio, 0, 0);
    context.fillStyle = "#000";
    context.fillRect(0, 0, width, height);

    if (view) {
      drawWallGrid(width, height);
      if (geometry.shouldRenderEstimate(estimate, nowMs, lastMessageMs, view)) {
        const projected = geometry.projectEstimate(estimate, view, width, height);
        if (projected) drawEstimate(projected);
      }
    }
    updatePanels(nowMs);
    requestAnimationFrame(render);
  }

  function connect() {
    const protocol = location.protocol === "https:" ? "wss" : "ws";
    const websocket = new WebSocket(`${protocol}://${location.host}/ws/estimates`);
    connectionState = "connecting";

    websocket.onopen = function () {
      connectionState = "live";
      reconnectDelayMs = 500;
    };
    websocket.onmessage = function (event) {
      try {
        estimate = JSON.parse(event.data);
        lastMessageMs = performance.now();
      } catch (_error) {
        connectionState = "invalid message";
      }
    };
    websocket.onerror = function () {
      connectionState = "error";
    };
    websocket.onclose = function () {
      connectionState = "reconnecting";
      window.setTimeout(connect, reconnectDelayMs);
      reconnectDelayMs = Math.min(reconnectDelayMs * 1.7, 5000);
    };
  }

  window.addEventListener("resize", resizeCanvas);
  window.addEventListener("keydown", function (event) {
    if (event.key.toLowerCase() === "h") showHud = !showHud;
    if (event.key.toLowerCase() === "c") calibrationMode = !calibrationMode;
    if (event.key.toLowerCase() === "f") document.documentElement.requestFullscreen?.();
    if (!calibrationMode || !view) return;

    let changed = true;
    if (event.key === "ArrowLeft") calibration.yaw -= 0.25;
    else if (event.key === "ArrowRight") calibration.yaw += 0.25;
    else if (event.key === "ArrowUp") calibration.pitch += 0.25;
    else if (event.key === "ArrowDown") calibration.pitch -= 0.25;
    else if (event.key === "+" || event.key === "=") calibration.fov += 0.5;
    else if (event.key === "-" || event.key === "_") calibration.fov -= 0.5;
    else if (event.key.toLowerCase() === "r") calibration = { yaw: 0, pitch: 0, fov: 0 };
    else changed = false;

    if (changed) {
      event.preventDefault();
      applyCalibration();
    }
  });

  async function start() {
    resizeCanvas();
    loadCalibration();
    const response = await fetch("/api/config", { cache: "no-store" });
    if (!response.ok) throw new Error(`configuration request failed: ${response.status}`);
    siteConfig = await response.json();
    applyCalibration();
    connect();
  }

  requestAnimationFrame(render);
  start().catch(function (error) {
    connectionState = `startup error: ${error.message}`;
    showHud = true;
  });
})();
