// Headless physics smoke test for the oil-drop integrator
// Run: node dev/physics-smoke-test.js

const ELECTRON_CHARGE = 1.602e-19;
const OIL_DENSITY = 860;
const AIR_DENSITY = 1.2;
const BOLTZMANN = 1.380649e-23;
const AIR_MEAN_FREE_PATH = 65e-9;
const CUNNINGHAM_A = 1.257;
const CUNNINGHAM_B = 0.4;
const CUNNINGHAM_C = 1.1;

const MAX_FRAME_DT = 1/20;
const SUBSTEP_DT = 1/1800;
const BASE_MAX_SPEED = 0.12;
const HISTORY_SAMPLE_INTERVAL = 1/45;

const state = {
  running: true,
  voltageKV: 2.0,
  plateGapMeters: 0.005,
  radiusMicrons: 0.9,
  viscosity: 1.8e-5,
  temperatureK: 295,
  noiseBoost: 1,
  fieldEnabled: true,
  fieldPolarity: 1,
  gravity: 9.81,
};

let drop = {
  y: state.plateGapMeters * 0.35,
  velocity: 0,
  chargeMultiple: -8,
  chargeCoulombs: -8 * ELECTRON_CHARGE,
  radiusMeters: state.radiusMicrons * 1e-6,
  mass: 0,
  slipFactor: 1,
  dragCoeff: 0,
};

function clamp(v, a, b) { return Math.min(Math.max(v, a), b); }
function computeSlipCorrection(radiusMeters) {
  const r = Math.max(radiusMeters, 5e-9);
  const kn = AIR_MEAN_FREE_PATH / r;
  return 1 + kn * (CUNNINGHAM_A + CUNNINGHAM_B * Math.exp(-CUNNINGHAM_C / kn));
}
function recompute() {
  drop.radiusMeters = state.radiusMicrons * 1e-6;
  const volume = (4/3) * Math.PI * Math.pow(drop.radiusMeters, 3);
  const effDensity = Math.max(OIL_DENSITY - AIR_DENSITY, 1);
  drop.mass = Math.max(volume * effDensity, 1e-20);
  drop.slipFactor = computeSlipCorrection(drop.radiusMeters);
  drop.dragCoeff = (6 * Math.PI * state.viscosity * drop.radiusMeters) / drop.slipFactor;
}

function computeElectricField() { if (!state.fieldEnabled) return 0; const gap = Math.max(state.plateGapMeters, 1e-5); return (state.voltageKV * 1000 * state.fieldPolarity) / gap; }

function integrateStep(dt, E) {
  const gravityForce = drop.mass * state.gravity;
  const electricForce = drop.chargeCoulombs * E;
  if (!Number.isFinite(drop.dragCoeff) || drop.dragCoeff <= 0) recompute();
  const dragCoeff = drop.dragCoeff;
  const dragForce = -dragCoeff * drop.velocity;
  const noiseStd = 0.6 * Math.sqrt((2 * BOLTZMANN * state.temperatureK * dragCoeff) / Math.max(dt, 1e-6));
  // sample random number using a simple seeded pseudo-random (Math.random fine for smoke)
  const noiseForce = state.noiseBoost * noiseStd * ((Math.random() * 2) - 1);
  const net = gravityForce + electricForce + dragForce + noiseForce;
  const accel = net / drop.mass;
  drop.velocity += accel * dt;
  const deterministicNet = gravityForce + electricForce; const vtAnalytic = dragCoeff > 0 ? deterministicNet / dragCoeff : 0; const dynamicCap = Math.max(2*Math.abs(vtAnalytic), 0.02); const speedCap = Math.max(dynamicCap, BASE_MAX_SPEED*0.25);
  drop.velocity = clamp(drop.velocity, -speedCap, speedCap);
  drop.y += drop.velocity * dt;
  if (drop.y < 0) { drop.y = 0; drop.velocity *= -0.3; }
  if (drop.y > state.plateGapMeters) { drop.y = state.plateGapMeters; drop.velocity *= -0.3; }
}

recompute();

let t = 0; const TOTAL = 3.0; // seconds
while (t < TOTAL) {
  let step = Math.min(SUBSTEP_DT, (TOTAL - t));
  const E = computeElectricField();
  integrateStep(step, E);
  t += step;
}

console.log('Final height (mm):', (drop.y*1000).toFixed(3));
console.log('Final velocity (mm/s):', (drop.velocity*1000).toFixed(3));
console.log('Charge (e):', drop.chargeMultiple);
console.log('Mass (kg):', drop.mass);
console.log('Drag coeff (kg/s):', drop.dragCoeff);
