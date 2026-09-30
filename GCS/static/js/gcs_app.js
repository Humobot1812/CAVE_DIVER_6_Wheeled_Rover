/* ==========================================================================
   SIH Ground Control Station — Frontend Logic
   Real-time Telemetry | Virtual Joystick | Drive Controls | Charts
   ========================================================================== */

'use strict';

// ─── STATE ───────────────────────────────────────────────────────────────────
const State = {
  telemetry: {},
  flashlight: false,
  hazardMode: false,
  estopActive: false,
  colormap: 'INFERNO',
  gimbalYaw: 0.0, gimbalPitch: 0.0,
  speedLin: 1.2, speedAng: 0.9,
  driveKeys: {},
  currentLayout: 'dual',
  logEntries: [],
  chartsInitialized: false,
  envChart: null,
};

// ─── INIT ─────────────────────────────────────────────────────────────────────
document.addEventListener('DOMContentLoaded', () => {
  initVirtualJoystick();
  initDriveKeypad();
  initKeyboard();
  initGimbalDpad();
  initColormap();
  initSpeedSliders();
  initLayoutToggle();
  initRadar();
  initCharts();
  startTelemetryLoop();
  log('GCS initialized', 'nominal');
  log('ROS 2 bridge connecting…', 'info');
  updateClockLoop();
  animateRadar();
});

// ─── TELEMETRY POLLING (10 Hz) ────────────────────────────────────────────────
function startTelemetryLoop() {
  setInterval(fetchTelemetry, 100);
}

async function fetchTelemetry() {
  try {
    const r = await fetch('/telemetry', { cache: 'no-store' });
    if (!r.ok) return;
    const d = await r.json();
    State.telemetry = d;
    updateEnvGauges(d.environmental || d.environment);
    updateCameraHUD(d.camera);
    updateRobotStatus(d.robot);
    updateSafetyBanner(d.environmental || d.environment);
    updateCharts(d.environmental || d.environment);
  } catch(e) {}
}

// ─── ENVIRONMENTAL GAUGES ────────────────────────────────────────────────────
function updateEnvGauges(env) {
  if (!env) return;

  const temp = typeof env.temperature === 'number' ? env.temperature : (typeof env.temperature_c === 'number' ? env.temperature_c : 22.4);
  const hum = typeof env.humidity === 'number' ? env.humidity : (typeof env.humidity_rh === 'number' ? env.humidity_rh : 64.0);
  const co2 = typeof env.co2 === 'number' ? env.co2 : (typeof env.co2_ppm === 'number' ? env.co2_ppm : 510);
  const o2 = typeof env.o2 === 'number' ? env.o2 : (typeof env.o2_percent === 'number' ? env.o2_percent : 20.9);
  const ch4 = typeof env.ch4 === 'number' ? env.ch4 : (typeof env.ch4_lel === 'number' ? env.ch4_lel : 0.02);
  const co = typeof env.co === 'number' ? env.co : (typeof env.co_ppm === 'number' ? env.co_ppm : 1.2);

  setGauge('g-temp', temp, 10, 45, '°C', temp.toFixed(1), 32, 36, false, 'NOMINAL', 'ELEVATED', 'HIGH TEMP');
  setGauge('g-hum', hum, 20, 100, '%', hum.toFixed(0), 80, 90, false, 'NORMAL', 'HUMID', 'CONDENSATION');
  setGauge('g-co2', co2, 350, 2000, 'ppm', Math.round(co2).toString(), 1000, 1500, false, 'SAFE', 'ELEVATED', 'HYPOXIA RISK');
  setGauge('g-o2', o2, 15, 23, '%', o2.toFixed(1), 19.5, 18.0, true, 'NORMAL', 'LOW O2', 'O2 DEFICIENT');
  setGauge('g-ch4', ch4, 0, 3, '%', ch4.toFixed(2), 0.5, 1.0, false, 'SAFE', 'DETECTED', 'EXPLOSIVE');
  setGauge('g-co', co, 0, 50, 'ppm', co.toFixed(1), 9, 25, false, 'SAFE', 'WARNING', 'TOXIC GAS');
}

function setGauge(id, value, min, max, unit, displayVal, warnThr, dangerThr, invertDanger, nomLbl, warnLbl, dngLbl) {
  const el = document.getElementById(id);
  if (!el) return;

  const pct = Math.max(0, Math.min(1, (value - min) / (max - min)));
  const arc = el.querySelector('.gauge-fill');
  const valEl = el.querySelector('.g-val');
  const unitEl = el.querySelector('.g-val-unit');
  const subEl = el.querySelector('.g-sub');

  const FULL = 113.1;
  const offset = FULL * (1.0 - pct);

  let isDanger = false;
  let isWarn = false;
  if (dangerThr !== undefined && warnThr !== undefined) {
    isDanger = invertDanger ? (value <= dangerThr) : (value >= dangerThr);
    isWarn = !isDanger && (invertDanger ? (value <= warnThr) : (value >= warnThr));
  }

  let strokeColor = '#10b981'; // Emerald
  let subColorClass = 'text-emerald';
  let subText = nomLbl || 'NOMINAL';

  if (isDanger) {
    strokeColor = '#ef4444'; // Red
    subColorClass = 'text-danger';
    subText = dngLbl || 'DANGER';
  } else if (isWarn) {
    strokeColor = '#f59e0b'; // Amber
    subColorClass = 'text-amber';
    subText = warnLbl || 'WARNING';
  }

  if (arc) {
    arc.style.strokeDashoffset = offset.toFixed(1);
    arc.style.stroke = strokeColor;
  }

  if (valEl) {
    valEl.textContent = displayVal;
  }
  if (unitEl && unit) {
    unitEl.textContent = unit;
  }

  if (subEl) {
    subEl.textContent = subText;
    subEl.className = 'g-sub ' + subColorClass;
  }

  el.classList.toggle('hazard-alert', isDanger);
}

// ─── SAFETY BANNER ───────────────────────────────────────────────────────────
function updateSafetyBanner(env) {
  if (!env) return;
  const banner = document.getElementById('safety-banner');
  const ttl = document.getElementById('safety-ttl');
  const dsc = document.getElementById('safety-dsc');
  const ico = document.getElementById('safety-ico');
  if (!banner) return;
  banner.className = 'safety-banner';
  const s = env.status || 'NOMINAL';
  if (s === 'DANGER') {
    banner.classList.add('banner-hazard');
    ttl.textContent = '⚠ DANGER — HAZARDOUS ENVIRONMENT';
    dsc.textContent = env.alerts?.join('  ·  ') || '';
    ico.textContent = '🚨';
  } else if (s === 'WARNING') {
    banner.classList.add('banner-warning');
    ttl.textContent = '⚡ WARNING — ELEVATED READINGS';
    dsc.textContent = env.alerts?.join('  ·  ') || '';
    ico.textContent = '⚡';
  } else {
    banner.classList.add('banner-nominal');
    ttl.textContent = '✔ ENVIRONMENT NOMINAL';
    dsc.textContent = 'All sensors within safe operating limits';
    ico.textContent = '✅';
  }
}

// ─── CAMERA HUD ──────────────────────────────────────────────────────────────
function updateCameraHUD(cam) {
  if (!cam) return;
  const rfps = document.getElementById('rgb-fps');
  const tfps = document.getElementById('thermal-fps');
  if (rfps) rfps.textContent = (cam.rgb_fps || 0).toFixed(1) + ' FPS';
  if (tfps) tfps.textContent = (cam.thermal_fps || 0).toFixed(1) + ' FPS';
  const spotEl = document.getElementById('spot-temp');
  if (spotEl && cam.spot_temp_c) spotEl.textContent = cam.spot_temp_c.toFixed(1) + '°C';
  if (cam.flashlight !== undefined) {
    updateFlashlightUI(cam.flashlight);
  }
}

// ─── ROBOT STATUS ────────────────────────────────────────────────────────────
function updateRobotStatus(robot) {
  if (!robot) return;
  const cx = document.getElementById('pos-x');
  const cy = document.getElementById('pos-y');
  const ch = document.getElementById('pos-heading');
  if (cx) cx.textContent = (robot.robot_x || 0).toFixed(2) + 'm';
  if (cy) cy.textContent = (robot.robot_y || 0).toFixed(2) + 'm';
  if (ch) ch.textContent = (robot.robot_yaw_deg || 0).toFixed(0) + '°';
  const conn = document.getElementById('ros-status');
  if (conn) {
    conn.textContent = robot.connected ? '● ROS2 ONLINE' : '○ STANDALONE';
    conn.className = robot.connected ? 'text-emerald' : 'text-amber';
  }
  // Update radar lidar data
  if (robot.lidar_ranges && robot.lidar_ranges.length > 0) {
    updateRadarPoints(robot.lidar_ranges);
  }
}

// ─── CHARTS (Chart.js via CDN) ───────────────────────────────────────────────
function initCharts() {
  const ctx = document.getElementById('envChart');
  if (!ctx || !window.Chart) return;
  State.envChart = new Chart(ctx, {
    type: 'line',
    data: {
      labels: [],
      datasets: [
        { label: 'Temp', borderColor: '#f87171', data: [], tension: .4, borderWidth: 1.5, pointRadius: 0, yAxisID: 'y' },
        { label: 'O2%', borderColor: '#10b981', data: [], tension: .4, borderWidth: 1.5, pointRadius: 0, yAxisID: 'y2' },
        { label: 'CH4', borderColor: '#f59e0b', data: [], tension: .4, borderWidth: 1.5, pointRadius: 0, yAxisID: 'y3' },
        { label: 'CO', borderColor: '#818cf8', data: [], tension: .4, borderWidth: 1.5, pointRadius: 0, yAxisID: 'y3' },
      ]
    },
    options: {
      animation: false, responsive: true, maintainAspectRatio: false,
      plugins: { legend: { labels: { color: '#94a3b8', font: { size: 9, family: 'JetBrains Mono' }, boxWidth: 9, padding: 8 } } },
      scales: {
        x: { ticks: { color: '#475569', font: { size: 9 }, maxTicksLimit: 8 }, grid: { color: 'rgba(255,255,255,.04)' } },
        y:  { position: 'left',  ticks: { color: '#f87171', font: { size: 9 } }, grid: { color: 'rgba(248,113,113,.07)' }, title:{ display:true, text:'T°C', color:'#f87171', font:{size:8} } },
        y2: { position: 'right', ticks: { color: '#10b981', font: { size: 9 } }, grid: { drawOnChartArea:false }, title:{ display:true, text:'O2%', color:'#10b981', font:{size:8} } },
        y3: { position: 'right', ticks: { color: '#f59e0b', font: { size: 9 }, maxTicksLimit:4 }, grid:{ drawOnChartArea:false }, title:{ display:true, text:'Gas', color:'#f59e0b', font:{size:8} } },
      }
    }
  });
}

function updateCharts(env) {
  if (!env?.history || !State.envChart) return;
  const h = env.history;
  const maxPts = 40;
  State.envChart.data.labels = h.timestamps.slice(-maxPts);
  State.envChart.data.datasets[0].data = h.temperature.slice(-maxPts);
  State.envChart.data.datasets[1].data = h.o2.slice(-maxPts);
  State.envChart.data.datasets[2].data = h.ch4.map(v => v * 100).slice(-maxPts);
  State.envChart.data.datasets[3].data = h.co.slice(-maxPts);
  State.envChart.update('none');
}

// ─── RADAR CANVAS ────────────────────────────────────────────────────────────
let radarCtx = null;
let radarAngle = 0;
let lidarPoints = [];

function initRadar() {
  const canvas = document.getElementById('radarCanvas');
  if (!canvas) return;
  radarCtx = canvas.getContext('2d');
  canvas.width = canvas.offsetWidth || 340;
  canvas.height = canvas.offsetHeight || 110;
}

function updateRadarPoints(ranges) {
  lidarPoints = ranges;
}

function animateRadar() {
  if (radarCtx) {
    const c = radarCtx.canvas;
    const w = c.width, h = c.height;
    const cx = w / 2, cy = h * 0.88;
    const r = Math.min(w, h) * 0.82;
    radarCtx.clearRect(0, 0, w, h);
    radarCtx.fillStyle = '#060810';
    radarCtx.fillRect(0, 0, w, h);
    // Rings
    for (let i = 1; i <= 3; i++) {
      radarCtx.beginPath();
      radarCtx.arc(cx, cy, r * i / 3, Math.PI, 0);
      radarCtx.strokeStyle = `rgba(0,242,254,${0.06 + i * 0.02})`;
      radarCtx.lineWidth = 0.8;
      radarCtx.stroke();
    }
    // Grid lines
    for (let a = 0; a <= 180; a += 30) {
      const rad = (a * Math.PI) / 180;
      radarCtx.beginPath();
      radarCtx.moveTo(cx, cy);
      radarCtx.lineTo(cx + r * Math.cos(Math.PI - rad), cy - r * Math.sin(Math.PI - rad));
      radarCtx.strokeStyle = 'rgba(0,242,254,0.07)';
      radarCtx.lineWidth = 0.7;
      radarCtx.stroke();
    }
    // Sweep
    radarAngle = (radarAngle + 2.5) % 360;
    const sweepRad = (radarAngle * Math.PI) / 180;
    const grad = radarCtx.createConicalGradient ? null : null;
    radarCtx.save();
    radarCtx.translate(cx, cy);
    radarCtx.rotate(sweepRad - Math.PI);
    radarCtx.beginPath();
    radarCtx.moveTo(0, 0);
    radarCtx.arc(0, 0, r, 0, 0.5);
    const sweepGrad = radarCtx.createLinearGradient(0, 0, r, 0);
    sweepGrad.addColorStop(0, 'rgba(0,242,254,0.42)');
    sweepGrad.addColorStop(1, 'rgba(0,242,254,0)');
    radarCtx.fillStyle = sweepGrad;
    radarCtx.fill();
    radarCtx.restore();
    // Sweep line
    radarCtx.beginPath();
    radarCtx.moveTo(cx, cy);
    const sweepLineRad = ((radarAngle % 360) - 180) * Math.PI / 180;
    radarCtx.lineTo(cx + r * Math.cos(sweepLineRad), cy + r * Math.sin(sweepLineRad));
    radarCtx.strokeStyle = 'rgba(0,242,254,0.85)';
    radarCtx.lineWidth = 1.4;
    radarCtx.stroke();
    // LiDAR points
    if (lidarPoints.length > 0) {
      const step = Math.PI / lidarPoints.length;
      lidarPoints.forEach((d, i) => {
        if (d < 0 || d > 8) return;
        const pointAngle = Math.PI - i * step;
        const dist = Math.min(d / 8, 1) * r;
        const px = cx + dist * Math.cos(pointAngle);
        const py = cy - dist * Math.sin(pointAngle);
        radarCtx.beginPath();
        radarCtx.arc(px, py, 2, 0, Math.PI * 2);
        radarCtx.fillStyle = d < 1.5 ? '#ef4444' : (d < 3.5 ? '#f59e0b' : '#10b981');
        radarCtx.fill();
      });
    } else {
      // Demo echo points when no LiDAR
      for (let i = 0; i < 18; i++) {
        const a = (i / 18) * Math.PI;
        const d = 0.3 + 0.5 * Math.abs(Math.sin(i * 0.8 + radarAngle * 0.04));
        const dist = d * r;
        const px = cx + dist * Math.cos(Math.PI - a);
        const py = cy - dist * Math.sin(Math.PI - a);
        radarCtx.beginPath();
        radarCtx.arc(px, py, 1.5, 0, Math.PI * 2);
        radarCtx.fillStyle = '#10b981';
        radarCtx.globalAlpha = 0.55;
        radarCtx.fill();
        radarCtx.globalAlpha = 1.0;
      }
    }
    // Bot icon
    radarCtx.beginPath();
    radarCtx.arc(cx, cy, 5, 0, Math.PI * 2);
    radarCtx.fillStyle = '#00f2fe';
    radarCtx.fill();
    radarCtx.strokeStyle = '#fff';
    radarCtx.lineWidth = 1;
    radarCtx.stroke();
  }
  requestAnimationFrame(animateRadar);
}

// ─── VIRTUAL JOYSTICK ─────────────────────────────────────────────────────────
function initVirtualJoystick() {
  const base = document.getElementById('joy-base');
  const handle = document.getElementById('joy-handle');
  if (!base || !handle) return;
  let active = false, startX, startY;
  const MAX_DIST = 42;
  const sendJoy = (nx, ny) => {
    const lin = -ny * State.speedLin;
    const ang = -nx * State.speedAng;
    fetch('/cmd_vel', { method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({ linear: lin, angular: ang }) }).catch(()=>{});
  };
  const onStart = e => {
    if (State.estopActive) return;
    active = true;
    const touch = e.touches ? e.touches[0] : e;
    const rect = base.getBoundingClientRect();
    startX = rect.left + rect.width / 2;
    startY = rect.top + rect.height / 2;
    e.preventDefault();
  };
  const onMove = e => {
    if (!active) return;
    const touch = e.touches ? e.touches[0] : e;
    let dx = touch.clientX - startX;
    let dy = touch.clientY - startY;
    const dist = Math.sqrt(dx*dx + dy*dy);
    if (dist > MAX_DIST) { dx = dx/dist*MAX_DIST; dy = dy/dist*MAX_DIST; }
    handle.style.transform = `translate(calc(-50% + ${dx}px), calc(-50% + ${dy}px))`;
    sendJoy(dx/MAX_DIST, dy/MAX_DIST);
    e.preventDefault();
  };
  const onEnd = () => {
    if (!active) return;
    active = false;
    handle.style.transform = 'translate(-50%,-50%)';
    fetch('/cmd_vel', { method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({ linear: 0, angular: 0 }) }).catch(()=>{});
  };
  base.addEventListener('mousedown', onStart);
  document.addEventListener('mousemove', onMove);
  document.addEventListener('mouseup', onEnd);
  base.addEventListener('touchstart', onStart, { passive: false });
  document.addEventListener('touchmove', onMove, { passive: false });
  document.addEventListener('touchend', onEnd);
}

// ─── DRIVE KEYPAD ─────────────────────────────────────────────────────────────
function initDriveKeypad() {
  const keys = {
    'btn-d-fwd': [1, 0], 'btn-d-bwd': [-1, 0],
    'btn-d-lft': [0, 1], 'btn-d-rgt': [0, -1],
    'btn-d-fl': [1, 1], 'btn-d-fr': [1, -1],
    'btn-d-bl': [-1, 1], 'btn-d-br': [-1, -1],
    'btn-d-stp': [0, 0]
  };
  Object.entries(keys).forEach(([id, [l, a]]) => {
    const btn = document.getElementById(id);
    if (!btn) return;
    btn.addEventListener('pointerdown', e => {
      if (State.estopActive) return;
      btn.classList.add('active');
      fetch('/cmd_vel', { method:'POST', headers:{'Content-Type':'application/json'},
        body: JSON.stringify({ linear: l * State.speedLin, angular: a * State.speedAng }) }).catch(()=>{});
    });
    btn.addEventListener('pointerup', () => {
      btn.classList.remove('active');
      if (id !== 'btn-d-stp') {
        fetch('/cmd_vel', { method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({ linear:0, angular:0 }) }).catch(()=>{});
      }
    });
    btn.addEventListener('pointerleave', () => btn.classList.remove('active'));
  });
}

// ─── KEYBOARD ─────────────────────────────────────────────────────────────────
function initKeyboard() {
  const held = new Set();
  const sendKb = () => {
    if (State.estopActive) return;
    let l = 0, a = 0;
    if (held.has('w') || held.has('arrowup')) l = State.speedLin;
    if (held.has('s') || held.has('arrowdown')) l = -State.speedLin;
    if (held.has('a') || held.has('arrowleft')) a = State.speedAng;
    if (held.has('d') || held.has('arrowright')) a = -State.speedAng;
    fetch('/cmd_vel', { method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({ linear:l, angular:a }) }).catch(()=>{});
  };
  document.addEventListener('keydown', e => {
    const k = e.key.toLowerCase();
    if (['w','a','s','d','arrowup','arrowdown','arrowleft','arrowright'].includes(k)) {
      e.preventDefault(); if (!held.has(k)) { held.add(k); sendKb(); }
    }
    if (k==='l') toggleFlashlight();
    if (k==='e') toggleEStop();
    if (k==='h') toggleHazard();
    if (k==='c') cycleColormapKey();
    if (k==='p') takeSnapshot();
  });
  document.addEventListener('keyup', e => {
    const k = e.key.toLowerCase();
    held.delete(k);
    if (['w','a','s','d','arrowup','arrowdown','arrowleft','arrowright'].includes(k)) {
      if (held.size === 0) fetch('/cmd_vel', { method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({ linear:0, angular:0 }) }).catch(()=>{});
      else sendKb();
    }
  });
}

// ─── GIMBAL D-PAD ─────────────────────────────────────────────────────────────
function initGimbalDpad() {
  const moves = {
    'gim-u': [0, 10], 'gim-d': [0, -10],
    'gim-l': [-15, 0], 'gim-r': [15, 0],
    'gim-c': null
  };
  Object.entries(moves).forEach(([id, delta]) => {
    const btn = document.getElementById(id);
    if (!btn) return;
    btn.addEventListener('click', () => {
      if (!delta) { State.gimbalYaw = 0; State.gimbalPitch = 0; }
      else { State.gimbalYaw = Math.max(-90, Math.min(90, State.gimbalYaw + delta[0])); State.gimbalPitch = Math.max(-30, Math.min(45, State.gimbalPitch + delta[1])); }
      updateGimbalUI();
      sendGimbal();
    });
  });
  const yawSlider = document.getElementById('gimbal-yaw-slider');
  const pitchSlider = document.getElementById('gimbal-pitch-slider');
  if (yawSlider) yawSlider.addEventListener('input', () => { State.gimbalYaw = parseFloat(yawSlider.value); updateGimbalUI(); sendGimbal(); });
  if (pitchSlider) pitchSlider.addEventListener('input', () => { State.gimbalPitch = parseFloat(pitchSlider.value); updateGimbalUI(); sendGimbal(); });
  // Presets
  [['prst-fwd',0,0],['prst-dn',0,-25],['prst-up',0,30],['prst-l',-45,0],['prst-r',45,0]].forEach(([id,y,p]) => {
    const el = document.getElementById(id);
    if (el) el.addEventListener('click', () => { State.gimbalYaw=y; State.gimbalPitch=p; updateGimbalUI(); sendGimbal(); });
  });
}

function updateGimbalUI() {
  const yd = document.getElementById('gimbal-yaw-disp'); if(yd) yd.textContent = State.gimbalYaw.toFixed(0)+'°';
  const pd = document.getElementById('gimbal-pitch-disp'); if(pd) pd.textContent = State.gimbalPitch.toFixed(0)+'°';
  const ys = document.getElementById('gimbal-yaw-slider'); if(ys) ys.value = State.gimbalYaw;
  const ps = document.getElementById('gimbal-pitch-slider'); if(ps) ps.value = State.gimbalPitch;
}

function sendGimbal() {
  fetch('/camera/gimbal', { method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({ yaw: State.gimbalYaw, pitch: State.gimbalPitch }) }).catch(()=>{});
}

// ─── SPEED SLIDERS ────────────────────────────────────────────────────────────
function initSpeedSliders() {
  const ls = document.getElementById('speed-lin-slider');
  const as = document.getElementById('speed-ang-slider');
  if (ls) ls.addEventListener('input', () => { State.speedLin = parseFloat(ls.value); const d = document.getElementById('speed-lin-val'); if(d) d.textContent = State.speedLin.toFixed(1)+' m/s'; });
  if (as) as.addEventListener('input', () => { State.speedAng = parseFloat(as.value); const d = document.getElementById('speed-ang-val'); if(d) d.textContent = State.speedAng.toFixed(1)+' rad/s'; });
}

// ─── COLORMAP ─────────────────────────────────────────────────────────────────
const CMAPS = ['INFERNO','JET','HOT','MAGMA','PLASMA','TURBO','GRAYSCALE'];
let cmIdx = 0;
function initColormap() {
  CMAPS.forEach(cm => {
    const btn = document.getElementById('cm-' + cm.toLowerCase());
    if (btn) btn.addEventListener('click', () => selectColormap(cm));
  });
  selectColormap('INFERNO');
}
function selectColormap(cm) {
  State.colormap = cm;
  CMAPS.forEach(c => { const b = document.getElementById('cm-'+c.toLowerCase()); if(b) b.classList.toggle('active', c===cm); });
  fetch('/camera/colormap', { method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({ colormap: cm }) }).catch(()=>{});
  log('Thermal colormap: ' + cm, 'info');
}
function cycleColormapKey() {
  cmIdx = (cmIdx + 1) % CMAPS.length;
  selectColormap(CMAPS[cmIdx]);
}

// ─── FLASHLIGHT ───────────────────────────────────────────────────────────────
function updateFlashlightUI(fl) {
  State.flashlight = !!fl;
  const btnHeader = document.getElementById('btn-flashlight');
  if (btnHeader) {
    btnHeader.classList.toggle('lit', State.flashlight);
    btnHeader.textContent = State.flashlight ? '💡 LIGHT ON' : '🔦 FLASHLIGHT';
  }
  const btnQC = document.getElementById('btn-flashlight-qc');
  if (btnQC) {
    btnQC.classList.toggle('lit', State.flashlight);
    btnQC.textContent = State.flashlight ? '💡 LIGHT ON' : '🔦 FLASHLIGHT';
  }
  const flpill = document.getElementById('fl-status-pill');
  if (flpill) {
    flpill.textContent = State.flashlight ? '💡 FLASHLIGHT ON' : '🔦 FLASHLIGHT OFF';
    flpill.style.color = State.flashlight ? '#ffd700' : 'rgba(255,255,255,0.45)';
  }
}

function toggleFlashlight() {
  const targetState = !State.flashlight;
  updateFlashlightUI(targetState);
  fetch('/camera/flashlight', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ enabled: targetState })
  })
  .then(r => r.json())
  .then(d => {
    if (d && d.flashlight !== undefined) {
      updateFlashlightUI(d.flashlight);
    }
    log('Flashlight ' + (State.flashlight ? 'ENABLED' : 'DISABLED'), State.flashlight ? 'nominal' : 'info');
  })
  .catch(() => {});
}

// ─── EMERGENCY STOP ───────────────────────────────────────────────────────────
function toggleEStop() {
  State.estopActive = !State.estopActive;
  const btn = document.getElementById('btn-estop');
  fetch('/robot/estop', { method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({ estop: State.estopActive }) }).catch(()=>{});
  if (btn) { btn.textContent = State.estopActive ? '✅ RESUME' : '🛑 E-STOP'; btn.classList.toggle('engaged', State.estopActive); }
  log(State.estopActive ? '🚨 EMERGENCY STOP ENGAGED' : '✅ MOTORS RESUMED', State.estopActive ? 'danger' : 'nominal');
}

// ─── HAZARD SIMULATION ────────────────────────────────────────────────────────
function toggleHazard() {
  State.hazardMode = !State.hazardMode;
  fetch('/telemetry/simulate_hazard', { method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({ enabled: State.hazardMode }) }).catch(()=>{});
  const btn = document.getElementById('btn-hazard');
  if (btn) { btn.textContent = State.hazardMode ? '🔴 HAZARD ACTIVE' : '⚠ SIM HAZARD'; btn.style.borderColor = State.hazardMode ? '#ef4444' : ''; }
  log(State.hazardMode ? '⚠ Hazard simulation INJECTED' : '✅ Hazard simulation cleared', State.hazardMode ? 'danger' : 'nominal');
}

// ─── SNAPSHOT ─────────────────────────────────────────────────────────────────
async function takeSnapshot() {
  try {
    const r = await fetch('/camera/snapshot', { method:'POST', headers:{'Content-Type':'application/json'}, body: '{}' });
    const d = await r.json();
    if (d.success) {
      const bar = document.getElementById('snap-bar');
      if (bar) {
        bar.style.display = 'flex';
        bar.innerHTML = `📷 Snapshot saved — <a href="${d.url_dual}" target="_blank" class="snap-link">View dual →</a>`;
        setTimeout(() => { bar.style.display = 'none'; }, 5000);
      }
      log('📷 Snapshot captured: ' + d.filename_rgb, 'nominal');
    }
  } catch(e) { log('Snapshot failed: ' + e, 'warn'); }
}

// ─── LAYOUT TOGGLE ────────────────────────────────────────────────────────────
function initLayoutToggle() {
  const layouts = ['dual','rgb-pip','thermal-pip','rgb-only','thermal-only'];
  layouts.forEach(l => {
    const btn = document.getElementById('layout-' + l);
    if (btn) btn.addEventListener('click', () => setLayout(l));
  });
  setLayout('dual');
}

function setLayout(layout) {
  State.currentLayout = layout;
  const stage = document.getElementById('video-stage');
  if (!stage) return;
  stage.className = 'video-stage layout-' + layout;
  document.querySelectorAll('.btn-layout').forEach(b => b.classList.toggle('active', b.id === 'layout-' + layout));
}

// ─── CLOCK ────────────────────────────────────────────────────────────────────
function updateClockLoop() {
  const el = document.getElementById('header-clock');
  const updateClock = () => {
    if (el) el.textContent = new Date().toISOString().slice(11, 22) + ' UTC';
  };
  updateClock();
  setInterval(updateClock, 1000);
}

// ─── EVENT LOG ────────────────────────────────────────────────────────────────
function log(msg, level='info') {
  const ts = new Date().toTimeString().slice(0, 8);
  const logEl = document.getElementById('log-stream');
  if (!logEl) return;
  const entry = document.createElement('div');
  entry.className = 'log-e';
  entry.innerHTML = `<span class="log-ts">${ts}</span><span class="log-${level}">${msg}</span>`;
  logEl.prepend(entry);
  while (logEl.children.length > 80) logEl.lastChild.remove();
}

function clearLog() {
  const el = document.getElementById('log-stream');
  if (el) el.innerHTML = '';
}

// Expose globals
window.toggleFlashlight = toggleFlashlight;
window.toggleEStop = toggleEStop;
window.toggleHazard = toggleHazard;
window.takeSnapshot = takeSnapshot;
window.selectColormap = selectColormap;
window.clearLog = clearLog;
window.setLayout = setLayout;
