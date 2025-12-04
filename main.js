// main.js - Millikan oil drop 2D simulator (module)

const canvas = document.getElementById('simCanvas');
const ctx = canvas.getContext('2d');
const historyCanvas = document.getElementById('historyCanvas');
const historyCtx = historyCanvas ? historyCanvas.getContext('2d') : null;
const runIndicator = document.getElementById('runIndicator');
const plateTopLabel = document.querySelector('.plate-top');
const plateBottomLabel = document.querySelector('.plate-bottom');

// Physics constants
const ELECTRON_CHARGE = 1.602e-19;
const OIL_DENSITY = 860; // kg/m^3
const AIR_DENSITY = 1.2; // kg/m^3
const BOLTZMANN = 1.380649e-23;
const AIR_MEAN_FREE_PATH = 65e-9; // m
const CUNNINGHAM_A = 1.257;
const CUNNINGHAM_B = 0.4;
const CUNNINGHAM_C = 1.1;

// Integrator constants
const MAX_FRAME_DT = 1 / 20;
const SUBSTEP_DT = 1 / 1800;
const BASE_MAX_SPEED = 0.12; // m/s
const BOUNCE_DAMPING = 0.3;
const HISTORY_WINDOW_SECONDS = 20;
const HISTORY_SAMPLE_INTERVAL = 1 / 45;
const MIN_GAP_METERS = 0.002;
const MAX_GAP_METERS = 0.01;

// Global state
const state = {
  running: true,
  voltageKV: 2.0,
  plateGapMeters: 0.005,
  radiusMicrons: 0.9,
  viscosity: 1.8e-5,
  temperatureK: 295,
  noiseBoost: 1,
  fieldEnabled: true,
  showTrail: true,
  showGrid: true,
  gravity: 9.81,
  pulseTimer: 0,
  fieldPolarity: 1,
};

const drop = {
  y: 0.005 * 0.35,
  velocity: 0,
  chargeMultiple: -8,
  chargeCoulombs: -8 * ELECTRON_CHARGE,
  radiusMeters: 0.9e-6,
  mass: 0,
  slipFactor: 1,
  dragCoeff: 0,
};

const trail = [];
const historySamples = [];
let historyAccumulator = 0;

// --- Utilities
function clamp(v, a, b) { return Math.min(Math.max(v, a), b); }
function gaussianRandom() {
  let u=0, v=0; while(u===0) u=Math.random(); while(v===0) v=Math.random();
  return Math.sqrt(-2*Math.log(u)) * Math.cos(2*Math.PI*v);
}

function computeSlipCorrection(radiusMeters) {
  const r = Math.max(radiusMeters, 5e-9);
  const kn = AIR_MEAN_FREE_PATH / r;
  return 1 + kn * (CUNNINGHAM_A + CUNNINGHAM_B * Math.exp(-CUNNINGHAM_C / kn));
}

function recomputeDropCoefficients() {
  drop.radiusMeters = state.radiusMicrons * 1e-6;
  const volume = (4/3) * Math.PI * Math.pow(drop.radiusMeters, 3);
  const effDensity = Math.max(OIL_DENSITY - AIR_DENSITY, 1);
  drop.mass = Math.max(volume * effDensity, 1e-20);
  drop.slipFactor = computeSlipCorrection(drop.radiusMeters);
  drop.dragCoeff = (6 * Math.PI * state.viscosity * drop.radiusMeters) / drop.slipFactor;
}

function getCanvasSize() {
  const ratio = window.devicePixelRatio || 1;
  return { width: canvas.width / ratio, height: canvas.height / ratio, ratio };
}

// Mapping between physical gap and drawing bounds
function getPlateBounds() {
  const { width, height } = getCanvasSize();
  const topMargin = 32;
  const bottomMargin = 32;
  // Make gap changes visually more obvious: map to large spacing range
  const minSpacing = 80;
  const maxSpacing = Math.max(120, height - topMargin - bottomMargin);
  const normalized = clamp((state.plateGapMeters - MIN_GAP_METERS) / (MAX_GAP_METERS - MIN_GAP_METERS), 0, 1);
  const spacing = minSpacing + normalized * (maxSpacing - minSpacing);
  const top = topMargin;
  const bottom = clamp(top + spacing, top + 40, height - bottomMargin);
  return { top, bottom };
}

function syncPlateLabels(bounds) {
  if (!plateTopLabel || !plateBottomLabel) return;
  const canvasRect = canvas.getBoundingClientRect();
  const paneRect = canvas.parentElement.getBoundingClientRect();
  const canvasOffsetTop = canvasRect.top - paneRect.top;
  const bottomHeight = plateBottomLabel.offsetHeight || 0;
  const topPx = canvasOffsetTop + bounds.top;
  const bottomPx = canvasOffsetTop + bounds.bottom - bottomHeight;
  plateTopLabel.style.top = `${topPx}px`;
  plateTopLabel.style.bottom = 'auto';
  const clampedBottom = Math.max(bottomPx, topPx + 24);
  plateBottomLabel.style.top = `${clampedBottom}px`;
  plateBottomLabel.style.bottom = 'auto';
}

function getDropScreenPosition() {
  const { width } = getCanvasSize();
  const bounds = getPlateBounds();
  const usable = Math.max(bounds.bottom - bounds.top, 1);
  const normalized = drop.y / state.plateGapMeters;
  const y = bounds.top + normalized * usable;
  const x = width / 2;
  return { x, y };
}

function getDropPixelRadius() {
  const minPx = 8, maxPx = 22;
  const normalized = (state.radiusMicrons - 0.3) / (1.5 - 0.3);
  return minPx + normalized * (maxPx - minPx);
}

function computeElectricField() {
  if (!state.fieldEnabled) return 0;
  const gap = Math.max(state.plateGapMeters, 1e-5);
  return (state.voltageKV * 1000 * state.fieldPolarity) / gap;
}

// Integrator step for the single drop
function integrateStep(dt, E) {
  const gravityForce = drop.mass * state.gravity;
  const electricForce = drop.chargeCoulombs * E;
  if (!Number.isFinite(drop.dragCoeff) || drop.dragCoeff <= 0) recomputeDropCoefficients();
  const dragCoeff = drop.dragCoeff;
  const dragForce = -dragCoeff * drop.velocity;

  const noiseStd = 0.6 * Math.sqrt((2 * BOLTZMANN * state.temperatureK * dragCoeff) / Math.max(dt, 1e-6));
  const noiseForce = state.noiseBoost * noiseStd * gaussianRandom();

  const netForce = gravityForce + electricForce + dragForce + noiseForce;
  const accel = netForce / drop.mass;
  drop.velocity += accel * dt;

  // Dynamic speed cap (so "wrong" inputs actually fall/rise noticeably)
  const deterministicNet = gravityForce + electricForce;
  const vtAnalytic = dragCoeff > 0 ? deterministicNet / dragCoeff : 0;
  const dynamicCap = Math.max(2 * Math.abs(vtAnalytic), 0.02);
  const speedCap = Math.max(dynamicCap, BASE_MAX_SPEED * 0.25);
  drop.velocity = clamp(drop.velocity, -speedCap, speedCap);

  drop.y += drop.velocity * dt;

  if (drop.y < 0) { drop.y = 0; drop.velocity *= -BOUNCE_DAMPING; }
  const maxY = state.plateGapMeters;
  if (drop.y > maxY) { drop.y = maxY; drop.velocity *= -BOUNCE_DAMPING; }
}

// Trail management
function advanceTrail(dt, appendSample) {
  if (!state.showTrail) { trail.length = 0; return; }
  if (appendSample) { trail.push({ y: drop.y, alpha: 1 }); if (trail.length > 140) trail.shift(); }
  const fade = clamp(dt, 0, 0.05) * 0.9;
  for (let i = trail.length - 1; i >= 0; i--) {
    const p = trail[i];
    p.alpha = Math.max(0, (p.alpha || 1) - fade);
    if (p.alpha <= 0.02) trail.splice(i, 1);
  }
}

function update(dt) {
  const safeDt = Number.isFinite(dt) ? Math.max(0, dt) : 0;
  let E = computeElectricField();
  if (!state.running) { advanceTrail(safeDt, false); return E; }
  let remaining = Math.min(safeDt, MAX_FRAME_DT);
  if (remaining === 0) { advanceTrail(0, false); return E; }

  while (remaining > 0) {
    const stepDt = Math.min(remaining, SUBSTEP_DT);
    state.pulseTimer = Math.max(0, state.pulseTimer - stepDt);
    if (state.pulseTimer === 0 && state.fieldPolarity !== 1) { state.fieldPolarity = 1; }
    E = computeElectricField();
    integrateStep(stepDt, E);
    remaining -= stepDt;
  }
  advanceTrail(safeDt, true);
  return E;
}

// Drawing functions
function drawBackground() {
  ctx.save();
  const { width: w, height: h } = getCanvasSize();
  ctx.clearRect(0, 0, w, h);
  const gradient = ctx.createLinearGradient(0, 0, 0, h);
  gradient.addColorStop(0, '#040a14'); gradient.addColorStop(1, '#030509');
  ctx.fillStyle = gradient; ctx.fillRect(0, 0, w, h);

  if (state.showGrid) {
    ctx.strokeStyle = 'rgba(255,255,255,0.05)'; ctx.lineWidth = 1; ctx.beginPath();
    const step = 40; for (let y = 40; y < h; y += step) { ctx.moveTo(40, y); ctx.lineTo(w - 40, y); }
    ctx.stroke();
  }
  ctx.restore();
}

function drawPlates() {
  ctx.save();
  const { width: w } = getCanvasSize(); const bounds = getPlateBounds();
  ctx.fillStyle = 'rgba(62, 226, 255, 0.25)'; ctx.fillRect(60, bounds.top - 6, w - 120, 8);
  ctx.fillStyle = 'rgba(255, 140, 66, 0.25)'; ctx.fillRect(60, bounds.bottom - 2, w - 120, 8);
  syncPlateLabels(bounds);
  ctx.restore();
}

function drawFieldLines() {
  if (!state.fieldEnabled) return;
  ctx.save(); const spacing = 80; const { width } = getCanvasSize(); const bounds = getPlateBounds();
  const top = bounds.top; const bottom = bounds.bottom; const eSign = Math.sign(drop.chargeMultiple || -1);
  for (let x = 80; x < width - 40; x += spacing) {
    const grad = ctx.createLinearGradient(0, top, 0, bottom);
    grad.addColorStop(0, eSign < 0 ? 'rgba(62,226,255,0.4)' : 'rgba(255,140,66,0.25)');
    grad.addColorStop(1, eSign < 0 ? 'rgba(255,140,66,0.25)' : 'rgba(62,226,255,0.4)');
    ctx.strokeStyle = grad; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(x, top); ctx.lineTo(x, bottom); ctx.stroke();
  }
  ctx.restore();
}

function drawTrail() {
  if (!state.showTrail || trail.length === 0) return;
  ctx.save(); ctx.lineWidth = 3; ctx.lineCap = 'round';
  const coords = trail.map(p => { const pos = getDropScreenPositionFromY(p.y); return { x: pos.x, y: pos.y, a: p.alpha || 1 }; });
  for (let i=1;i<coords.length;i++) {
    const from = coords[i-1], to = coords[i]; const alpha = clamp(to.a, 0.05, 1);
    ctx.strokeStyle = `rgba(62,226,255,${0.12 + alpha * 0.25})`;
    ctx.beginPath(); ctx.moveTo(from.x, from.y); ctx.lineTo(to.x, to.y); ctx.stroke();
  }
  ctx.restore();
}

function getDropScreenPositionFromY(yValue) { const { width } = getCanvasSize(); const bounds = getPlateBounds(); const usable = Math.max(bounds.bottom - bounds.top, 1); const normalized = yValue / state.plateGapMeters; const y = bounds.top + normalized * usable; const x = width / 2; return { x, y }; }

function drawDrop() {
  const { x, y } = getDropScreenPosition(); const radius = getDropPixelRadius(); ctx.save();
  const gradient = ctx.createRadialGradient(x - radius/3, y - radius/3, radius/4, x, y, radius);
  gradient.addColorStop(0, 'rgba(255,255,255,0.9)'); gradient.addColorStop(1, 'rgba(62,226,255,0.2)'); ctx.fillStyle = gradient;
  ctx.beginPath(); ctx.arc(x, y, radius, 0, Math.PI*2); ctx.fill(); ctx.lineWidth = 2; ctx.strokeStyle = drop.chargeMultiple < 0 ? '#3ee2ff' : '#ff8c42'; ctx.stroke(); ctx.restore();
  ctx.save(); ctx.fillStyle='rgba(255,255,255,0.85)'; ctx.font='12px "Segoe UI"'; ctx.textAlign='center'; ctx.fillText(`${drop.chargeMultiple}e`, x, y - radius - 10); ctx.restore();
}

function drawHUD() { ctx.save(); ctx.font='12px "Segoe UI"'; ctx.fillStyle='rgba(255,255,255,0.6)'; ctx.textAlign='left'; ctx.fillText(`Voltage: ${state.voltageKV.toFixed(1)} kV`, 16, 20); ctx.fillText(`Gap: ${(state.plateGapMeters*1000).toFixed(1)} mm`, 16, 36); ctx.restore(); }

function resizeCanvas() { const ratio = window.devicePixelRatio || 1; const width = canvas.clientWidth || canvas.width; const height = canvas.clientHeight || canvas.height; canvas.width = Math.floor(width * ratio); canvas.height = Math.floor(height * ratio); canvas.style.width = `${width}px`; canvas.style.height = `${height}px`; ctx.setTransform(1, 0, 0, 1, 0, 0); ctx.scale(ratio, ratio); }

function syncRunIndicator() { const playButton = document.getElementById('playPauseBtn'); if (playButton) playButton.textContent = state.running ? 'Pause' : 'Resume'; if (runIndicator) runIndicator.textContent = state.running ? 'running' : 'paused'; }

// History graph
function resizeHistoryCanvas() { if (!historyCanvas || !historyCtx) return; const ratio = window.devicePixelRatio || 1; const width = historyCanvas.clientWidth || historyCanvas.width; const height = historyCanvas.clientHeight || historyCanvas.height; historyCanvas.width = Math.floor(width * ratio); historyCanvas.height = Math.floor(height * ratio); historyCanvas.style.width = `${width}px`; historyCanvas.style.height = `${height}px`; historyCtx.setTransform(1, 0, 0, 1, 0, 0); historyCtx.scale(ratio, ratio); }
function captureHistorySample(dt, electricField, force=false) { if (!historyCtx) return; historyAccumulator += dt; if (!force && historyAccumulator < HISTORY_SAMPLE_INTERVAL) return; const now = performance.now() / 1000; const heightNorm = state.plateGapMeters > 0 ? clamp(drop.y / state.plateGapMeters, 0, 1) : 0; historySamples.push({ t: now, height: heightNorm, velocity: drop.velocity, field: electricField }); const cutoff = now - HISTORY_WINDOW_SECONDS; while (historySamples.length && historySamples[0].t < cutoff) historySamples.shift(); historyAccumulator = 0; }
function sampleInstantHistory() { captureHistorySample(0, computeElectricField(), true); }
function drawHistoryGraph() {
  if (!historyCtx || !historyCanvas) return;
  const ratio = window.devicePixelRatio || 1; const width = historyCanvas.width / ratio; const height = historyCanvas.height / ratio;
  historyCtx.save(); historyCtx.setTransform(1, 0, 0, 1, 0, 0); historyCtx.scale(ratio, ratio); historyCtx.clearRect(0, 0, width, height);
  historyCtx.fillStyle = 'rgba(5,8,14,0.92)'; historyCtx.fillRect(0, 0, width, height);
  historyCtx.strokeStyle = 'rgba(255,255,255,0.08)'; historyCtx.lineWidth = 1; historyCtx.beginPath(); for (let i=1;i<=3;i++){ const y=(height/4)*i; historyCtx.moveTo(0,y); historyCtx.lineTo(width,y); } historyCtx.stroke();
  if (historySamples.length >= 2) {
    const now = performance.now()/1000; historyCtx.lineWidth = 2; historyCtx.strokeStyle = '#3ee2ff'; historyCtx.beginPath(); historySamples.forEach((sample, index) => { const age = clamp((now - sample.t) / HISTORY_WINDOW_SECONDS, 0, 1); const x = width * (1 - age); const y = 8 + (1 - sample.height) * (height - 16); if (index === 0) historyCtx.moveTo(x, y); else historyCtx.lineTo(x, y); }); historyCtx.stroke();
    historyCtx.strokeStyle = 'rgba(255,140,66,0.9)'; historyCtx.beginPath(); historySamples.forEach((sample, index) => { const age = clamp((now - sample.t) / HISTORY_WINDOW_SECONDS, 0, 1); const x = width * (1 - age); const velocityNorm = clamp(sample.velocity / BASE_MAX_SPEED, -1, 1); const y = height/2 - velocityNorm * (height/2 - 12); if (index === 0) historyCtx.moveTo(x,y); else historyCtx.lineTo(x,y); }); historyCtx.stroke();
  } else { historyCtx.fillStyle='rgba(255,255,255,0.45)'; historyCtx.font='12px "Segoe UI"'; historyCtx.fillText('Graph warms up once the sim runs', 12, height/2); }
  historyCtx.fillStyle='rgba(255,255,255,0.65)'; historyCtx.font='11px "Segoe UI"'; historyCtx.fillText('height', 12, 16); historyCtx.fillText('velocity', 12, 30);
  historyCtx.restore();
}

function updateReadouts(E) {
  document.getElementById('fieldReadout').textContent = `${(E/1000).toFixed(1)} kV/m`;
  document.getElementById('chargeReadout').textContent = `${(drop.chargeMultiple).toFixed(0)} e (${drop.chargeCoulombs.toExponential(2)} C)`;
  const gravityForce = drop.mass * state.gravity; const electricForce = drop.chargeCoulombs * E;
  document.getElementById('forceReadout').textContent = `${(gravityForce*1e12).toFixed(2)} pN vs ${(electricForce*1e12).toFixed(2)} pN`;
  document.getElementById('velocityReadout').textContent = `${(drop.velocity*1000).toFixed(2)} mm/s`;
  const vt = drop.dragCoeff > 0 ? (gravityForce / drop.dragCoeff) : 0; document.getElementById('terminalReadout').textContent = `${(vt*1000).toFixed(2)} mm/s`;
  document.getElementById('multipleReadout').textContent = `${(drop.chargeMultiple).toFixed(1)} × e`;
  const balanceField = drop.chargeCoulombs !== 0 ? gravityForce / drop.chargeCoulombs : null; const balanceEl = document.getElementById('balanceReadout'); if (balanceEl) balanceEl.textContent = balanceField ? `${(balanceField/1000).toFixed(2)} kV/m` : '—';
  const slipEl = document.getElementById('slipReadout'); if (slipEl) slipEl.textContent = `${(drop.slipFactor || 1).toFixed(3)}`;
}

// UI helpers & wiring
function updateRange(id, value, outputId, precision=2, numberId) {
  const input = document.getElementById(id); const output = outputId ? document.getElementById(outputId) : null; const numeric = numberId ? document.getElementById(numberId) : null;
  if (input) input.value = value; if (output) output.textContent = Number(value).toFixed(precision); if (numeric) numeric.value = Number(value).toFixed(precision);
}

function setPlateGap(mmValue, options={}) {
  const { preservePosition=true, syncControl=false } = options; const previousGap = state.plateGapMeters; const cleanValue = Number.isFinite(mmValue) ? mmValue : previousGap*1000; const meters = clamp(cleanValue/1000, MIN_GAP_METERS, MAX_GAP_METERS); const mmDisplay = meters*1000; state.plateGapMeters = meters; if (syncControl) updateRange('gapControl', mmDisplay, null, 1, 'gapNumber'); if (preservePosition && previousGap>0) { const relative = clamp(drop.y / previousGap,0,1); drop.y = relative * meters; trail.forEach(pt => { const rel = clamp(pt.y / previousGap, 0, 1); pt.y = rel * meters; }); } else { drop.y = meters * 0.35; }
}

function setChargeMultiple(m) { const clamped = Math.round(clamp(m, -25, 25)); drop.chargeMultiple = clamped; drop.chargeCoulombs = clamped * ELECTRON_CHARGE; const cs = document.getElementById('chargeControl'); const cn = document.getElementById('chargeNumber'); if (cs) cs.value = clamped; if (cn) cn.value = clamped; }
function setRadiusMicrons(microns, options={ preserveVelocity:true }) { const clamped = clamp(microns, 0.3, 1.5); state.radiusMicrons = clamped; recomputeDropCoefficients(); drop.y = clamp(drop.y, 0, state.plateGapMeters); if (!options.preserveVelocity) drop.velocity = 0; }

// UI event wiring (sliders, numbers, toggles, buttons)
function handleUI() {
  // voltage
  const voltageSlider = document.getElementById('voltageControl'); const voltageNumber = document.getElementById('voltageNumber'); const applyVoltage = (v)=>{ const clamped = clamp(Number(v),0,8); state.voltageKV = clamped; if (voltageSlider) voltageSlider.value = clamped; if (voltageNumber) voltageNumber.value = clamped.toFixed(1); sampleInstantHistory(); };
  voltageSlider.addEventListener('input', e=>applyVoltage(e.target.value)); voltageNumber.addEventListener('change', e=>applyVoltage(e.target.value));

  // gap
  const gapSlider = document.getElementById('gapControl'); const gapNumber = document.getElementById('gapNumber'); const applyGap = (v)=>{ const mm = clamp(Number(v), MIN_GAP_METERS*1000, MAX_GAP_METERS*1000); setPlateGap(mm, { preservePosition:true, syncControl:true }); sampleInstantHistory(); };
  gapSlider.addEventListener('input', e=>applyGap(e.target.value)); gapNumber.addEventListener('change', e=>applyGap(e.target.value));

  // radius
  const radiusSlider = document.getElementById('radiusControl'); const radiusNumber = document.getElementById('radiusNumber'); const applyRadius = (v)=>{ const m = clamp(Number(v), 0.3, 1.5); setRadiusMicrons(m, { preserveVelocity:true }); if (radiusSlider) radiusSlider.value = m; if (radiusNumber) radiusNumber.value = m.toFixed(2); sampleInstantHistory(); };
  radiusSlider.addEventListener('input', e=>applyRadius(e.target.value)); radiusNumber.addEventListener('change', e=>applyRadius(e.target.value));

  // charge
  const chargeSlider = document.getElementById('chargeControl'); const chargeNumber = document.getElementById('chargeNumber'); const applyCharge = (v)=>{ const clamped = clamp(Number(v), -25, 25); setChargeMultiple(clamped); if (chargeSlider) chargeSlider.value = drop.chargeMultiple; if (chargeNumber) chargeNumber.value = drop.chargeMultiple; sampleInstantHistory(); };
  chargeSlider.addEventListener('input', e=>applyCharge(e.target.value)); chargeNumber.addEventListener('change', e=>applyCharge(e.target.value));

  // temperature
  const tempSlider = document.getElementById('temperatureControl'); const tempNumber = document.getElementById('temperatureNumber'); const applyTemp = (v)=>{ const t=clamp(Number(v),260,330); state.temperatureK = t; if (tempSlider) tempSlider.value = t; if (tempNumber) tempNumber.value = t.toFixed(0); sampleInstantHistory(); };
  tempSlider.addEventListener('input', e=>applyTemp(e.target.value)); tempNumber.addEventListener('change', e=>applyTemp(e.target.value));

  // noise
  const noiseSlider = document.getElementById('noiseControl'); const noiseNumber = document.getElementById('noiseNumber'); const applyNoise = (v)=>{ const n=clamp(Number(v),0,2); state.noiseBoost = n; if (noiseSlider) noiseSlider.value = n; if (noiseNumber) noiseNumber.value = n.toFixed(2); sampleInstantHistory(); };
  noiseSlider.addEventListener('input', e=>applyNoise(e.target.value)); noiseNumber.addEventListener('change', e=>applyNoise(e.target.value));

  // viscosity
  const viscSlider = document.getElementById('viscosityControl'); const viscNumber = document.getElementById('viscosityNumber'); const applyViscosity = (v)=>{ const s=clamp(Number(v),1,3); state.viscosity = s*1e-5; if (viscSlider) viscSlider.value = s; if (viscNumber) viscNumber.value = s.toFixed(2); recomputeDropCoefficients(); sampleInstantHistory(); };
  viscSlider.addEventListener('input', e=>applyViscosity(e.target.value)); viscNumber.addEventListener('change', e=>applyViscosity(e.target.value));

  document.getElementById('fieldToggle').addEventListener('change', e=>{ state.fieldEnabled = e.target.checked; });
  document.getElementById('trailToggle').addEventListener('change', e=>{ state.showTrail = e.target.checked; });
  document.getElementById('gridToggle').addEventListener('change', e=>{ state.showGrid = e.target.checked; });

  document.getElementById('playPauseBtn').addEventListener('click', ()=>{ state.running = !state.running; syncRunIndicator(); });
  document.getElementById('newDropBtn').addEventListener('click', ()=>{ resetDrop(true); sampleInstantHistory(); });
  document.getElementById('pulseBtn').addEventListener('click', ()=>{ state.fieldPolarity = -state.fieldPolarity; state.pulseTimer = 0.8; });
  document.getElementById('zeroBtn').addEventListener('click', ()=>{ drop.velocity = 0; sampleInstantHistory(); });

  const resetBtn = document.getElementById('resetBtn'); if (resetBtn) { resetBtn.addEventListener('click', ()=>{ state.running = true; state.voltageKV = 2.0; state.plateGapMeters = 0.005; state.radiusMicrons = 0.9; state.viscosity = 1.8e-5; state.temperatureK = 295; state.noiseBoost = 1; state.fieldEnabled = true; state.showTrail = true; state.showGrid = true; state.gravity = 9.81; state.pulseTimer = 0; state.fieldPolarity = 1; setChargeMultiple(-8); setRadiusMicrons(state.radiusMicrons, { preserveVelocity:false }); setPlateGap(state.plateGapMeters*1000, { preservePosition:false, syncControl:true }); updateRange('voltageControl', state.voltageKV, null, 1, 'voltageNumber'); updateRange('temperatureControl', state.temperatureK, null, 0, 'temperatureNumber'); updateRange('noiseControl', state.noiseBoost, null, 2, 'noiseNumber'); updateRange('viscosityControl', state.viscosity/1e-5, null, 2, 'viscosityNumber'); resetDrop(false); syncRunIndicator(); sampleInstantHistory(); }); }

  canvas.addEventListener('pointerdown', (e)=>{
    const rect = canvas.getBoundingClientRect(); const x = e.clientX - rect.left; const y = e.clientY - rect.top; const dropCoord = getDropScreenPosition(); const dx = x - dropCoord.x; const dy = y - dropCoord.y; const radiusPx = getDropPixelRadius(); if (Math.hypot(dx, dy) <= radiusPx*1.5) { const delta = e.shiftKey ? 1 : -1; setChargeMultiple(drop.chargeMultiple + delta); } });
}

function resetDrop(randomize=false) {
  let radiusUpdated = false;
  if (randomize) {
    const randomRadius = +(0.4 + Math.random()*0.9).toFixed(2);
    state.voltageKV = +(0.5 + Math.random()*5.5).toFixed(2);
    const randCharge = (Math.random() > 0.5 ? 1 : -1)*(2 + Math.floor(Math.random()*12));
    setChargeMultiple(randCharge);
    setRadiusMicrons(randomRadius, { preserveVelocity:false }); radiusUpdated = true; updateRange('voltageControl', state.voltageKV, null, 1, 'voltageNumber'); updateRange('radiusControl', randomRadius, null, 2, 'radiusNumber'); updateRange('chargeControl', drop.chargeMultiple, null, 0, 'chargeNumber'); const randomGap = +(2 + Math.random()*6).toFixed(1); setPlateGap(randomGap, { preservePosition:false, syncControl:true });
  }
  if (!radiusUpdated) setRadiusMicrons(state.radiusMicrons, { preserveVelocity:false });
  drop.y = state.plateGapMeters * 0.35; drop.velocity = 0; trail.length = 0;
}

// --- Main loop
let lastTime = performance.now(); function loop(timestamp) { const dt = Math.min(0.05, Math.max(0, (timestamp - lastTime) / 1000)); lastTime = timestamp; const E = update(dt); captureHistorySample(dt, E); drawBackground(); drawPlates(); drawFieldLines(); drawTrail(); drawDrop(); drawHUD(); updateReadouts(E); drawHistoryGraph(); requestAnimationFrame(loop); }

function bootstrap() { handleUI(); setPlateGap(state.plateGapMeters*1000, { preservePosition:false, syncControl:true }); setChargeMultiple(drop.chargeMultiple); recomputeDropCoefficients(); updateRange('temperatureControl', state.temperatureK, null, 0, 'temperatureNumber'); updateRange('noiseControl', state.noiseBoost, null, 2, 'noiseNumber'); resetDrop(false); resizeCanvas(); resizeHistoryCanvas(); syncRunIndicator(); sampleInstantHistory(); window.addEventListener('resize', () => { resizeCanvas(); resizeHistoryCanvas(); }); requestAnimationFrame(loop); }

bootstrap();
