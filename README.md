# Millikan Oil Drop Simulator

A small web-based 2D Millikan oil-drop simulator with interactive sliders and a history graph. This project simulates gravity, electric force, Stokes slip-corrected drag, and Brownian thermal noise.

Files:
- `index.html` — Simulation UI and layout
- `styles.css` — Visual styles
- `main.js` — Main physics, UI wiring, and rendering logic (loaded as an ES module)
- `dev/physics-smoke-test.js` — Node smoke test to validate integrator behavior headless

Quick start
1. Open `index.html` in a modern browser (modules are used, so serve the page from localhost or file:// may be blocked by CORS in some browsers).
2. Tweak sliders: Plate voltage, gap, drop radius, charge, temperature, noise, viscosity.
3. Click the drop to inject/remove an electron (shift-key reverses sign). Use `New drop`, `Reset`, `Pulse field`, or `Zero velocity`.

Dev smoke test
- Run this with Node to verify integrator stability:
```
node dev/physics-smoke-test.js
```

Notes
-- The `main.js` is written as an ES module and runs the 2D canvas-driven simulation; there is no runtime dependency on three.js for the 2D sim.
- Terminal speed is dynamically clamped so that when inputs are "wrong" (far from balance) the particle falls/rises noticeably.
- The model uses Cunningham slip correction and a Brownian fluctuation–dissipation relation for thermal jitter.

License: MIT

Enjoy!

