#!/usr/bin/env python3
"""Generates the standalone gimbal viewer from the OpenSCAD dimensions.

The viewer is written rather than hand-maintained so that its geometry cannot
drift from the parts that get printed: every dimension is read out of
`gimbal_parts.scad` at generation time, the same way `check_clearances.py`
reads them.

Usage:
    python make_visualizer.py           # writes gimbal_viewer.html
"""
from __future__ import annotations

import json
import pathlib

import numpy as np

from check_clearances import (
    SCAD, TILT_LIMITS, clearance, cradle_points, obstacles, scad_values, to_global,
)

OUT = pathlib.Path(__file__).with_name("gimbal_viewer.html")
PAN_LIMITS = (-170.0, 170.0)

# Dimensions the viewer needs. Named here rather than dumping everything so a
# missing one is an error at generation time, not a silent NaN in the browser.
WANTED = [
    "motor_body", "motor_len", "motor_boss_d", "motor_boss_h",
    "shaft_d", "shaft_len", "driver_pcb", "driver_depth",
    "tilt_axis_h", "tilt_face_y", "plate_t", "col_w", "beam_z0", "beam_h",
    "pan_hub_z", "pan_hub_h", "pan_hub_od",
    "tilt_plate_w", "tilt_plate_h",
    "cradle_hub_h", "cradle_hub_od",
    "plat_l", "plat_w", "plat_t", "plat_drop", "rib_h",
    "base_w", "base_t", "leg_h", "leg_d",
]

HTML = """<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Pan/tilt gimbal &mdash; NEMA17 + SERVO42D</title>
<style>
:root {
  color-scheme: light dark;
  /* Neutrals carry a slight blue bias, so they read as chosen against the
     PLA-orange accent rather than as default greys. */
  --ink: #12151a;
  --surface: #ffffff;
  --panel: #f4f6f8;
  --rule: #d9dee5;
  --text: #12151a;
  --muted: #5d6773;
  --accent: #c25c17;      /* the orange the yoke prints in */
  --accent-soft: #f3e2d4;
  --cool: #2f7fae;        /* the cradle */
  --ok: #2f7d4f;
  --warn: #b4791d;
  --grid: #e6eaef;
}
@media (prefers-color-scheme: dark) {
  :root {
    --ink: #0d1014;
    --surface: #12151a;
    --panel: #1a1f27;
    --rule: #2b3340;
    --text: #e6e9ee;
    --muted: #8d97a6;
    --accent: #e0752f;
    --accent-soft: #2a1d13;
    --cool: #4aa3d8;
    --ok: #4ea86b;
    --warn: #d8a445;
    --grid: #1e242d;
  }
}
:root[data-theme="dark"] {
  --ink: #0d1014; --surface: #12151a; --panel: #1a1f27; --rule: #2b3340;
  --text: #e6e9ee; --muted: #8d97a6; --accent: #e0752f; --accent-soft: #2a1d13;
  --cool: #4aa3d8; --ok: #4ea86b; --warn: #d8a445; --grid: #1e242d;
}
:root[data-theme="light"] {
  --ink: #12151a; --surface: #ffffff; --panel: #f4f6f8; --rule: #d9dee5;
  --text: #12151a; --muted: #5d6773; --accent: #c25c17; --accent-soft: #f3e2d4;
  --cool: #2f7fae; --ok: #2f7d4f; --warn: #b4791d; --grid: #e6eaef;
}

* { box-sizing: border-box; }
body {
  margin: 0;
  background: var(--surface);
  color: var(--text);
  font: 15px/1.55 ui-sans-serif, system-ui, -apple-system, "Segoe UI", sans-serif;
}
.mono {
  font-family: ui-monospace, SFMono-Regular, "SF Mono", Menlo, Consolas, monospace;
  font-variant-numeric: tabular-nums;
}

header {
  border-bottom: 1px solid var(--rule);
  padding: 18px 22px 14px;
  display: flex; flex-wrap: wrap; align-items: baseline; gap: 6px 18px;
}
h1 { font-size: 17px; margin: 0; font-weight: 620; letter-spacing: -0.01em; }
header .sub {
  font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
  font-size: 12px; color: var(--muted); letter-spacing: 0.02em;
}
header .spacer { flex: 1 1 auto; }

.wrap {
  display: grid;
  grid-template-columns: minmax(0, 1fr) 310px;
  gap: 0;
  min-height: calc(100vh - 58px);
}
@media (max-width: 820px) { .wrap { grid-template-columns: minmax(0, 1fr); } }

.stage { position: relative; background: var(--ink); overflow: hidden; }
canvas { display: block; width: 100%; height: 100%; touch-action: none; cursor: grab; }
canvas:active { cursor: grabbing; }
.hint {
  position: absolute; left: 14px; bottom: 12px;
  font-family: ui-monospace, Menlo, monospace; font-size: 11px;
  color: #6d7embed; color: var(--muted); pointer-events: none; opacity: .85;
}

.rail {
  border-left: 1px solid var(--rule);
  background: var(--panel);
  padding: 18px 18px 26px;
  display: flex; flex-direction: column; gap: 20px;
  overflow-y: auto;
}
@media (max-width: 820px) { .rail { border-left: 0; border-top: 1px solid var(--rule); } }

.group { display: flex; flex-direction: column; gap: 9px; }
.eyebrow {
  font-family: ui-monospace, Menlo, monospace;
  font-size: 10.5px; letter-spacing: .1em; text-transform: uppercase;
  color: var(--muted);
}
.readout { display: flex; align-items: baseline; justify-content: space-between; gap: 10px; }
.readout .val {
  font-family: ui-monospace, Menlo, monospace; font-variant-numeric: tabular-nums;
  font-size: 21px; font-weight: 600; color: var(--accent);
}
.readout .lim { font-family: ui-monospace, Menlo, monospace; font-size: 11px; color: var(--muted); }

input[type=range] {
  width: 100%; accent-color: var(--accent); margin: 0;
}
input[type=range]:focus-visible { outline: 2px solid var(--accent); outline-offset: 3px; }

.btnrow { display: flex; flex-wrap: wrap; gap: 6px; }
button {
  font: inherit; font-size: 12px;
  font-family: ui-monospace, Menlo, monospace;
  padding: 5px 9px; border-radius: 3px;
  border: 1px solid var(--rule); background: var(--surface); color: var(--text);
  cursor: pointer;
}
button:hover { border-color: var(--accent); color: var(--accent); }
button:focus-visible { outline: 2px solid var(--accent); outline-offset: 2px; }
button[aria-pressed="true"] { background: var(--accent-soft); border-color: var(--accent); color: var(--accent); }

table.dims { border-collapse: collapse; width: 100%; font-size: 12px; }
table.dims td {
  padding: 3px 0; border-bottom: 1px solid var(--rule);
  font-family: ui-monospace, Menlo, monospace; font-variant-numeric: tabular-nums;
}
table.dims td:first-child { color: var(--muted); }
table.dims td:last-child { text-align: right; }

.status { display: flex; align-items: center; gap: 8px; font-size: 12.5px; }
.dot { width: 8px; height: 8px; border-radius: 50%; background: var(--ok); flex: none; }
.note {
  font-size: 12.5px; color: var(--muted);
  border-left: 2px solid var(--accent); padding-left: 10px;
}
.legend { display: flex; flex-wrap: wrap; gap: 4px 14px; font-size: 11.5px; color: var(--muted);
  font-family: ui-monospace, Menlo, monospace; }
.swatch { display: inline-block; width: 9px; height: 9px; border-radius: 2px; margin-right: 5px; }
</style>

<header>
  <h1>Two-axis camera gimbal</h1>
  <span class="sub">NEMA17 32&nbsp;mm &middot; MKS SERVO42D_CAN &middot; 2 printed parts</span>
  <span class="spacer"></span>
  <span class="sub" id="pose">pan +0.0&deg; &nbsp; tilt +0.0&deg;</span>
</header>

<div class="wrap">
  <div class="stage">
    <canvas id="view"></canvas>
    <div class="hint">drag to orbit &middot; scroll to zoom</div>
  </div>

  <aside class="rail">
    <div class="group">
      <div class="eyebrow">Pan &mdash; CAN&nbsp;id&nbsp;2</div>
      <div class="readout">
        <span class="val" id="panVal">+0.0&deg;</span>
        <span class="lim">__PAN_LO__&deg; to +__PAN_HI__&deg;</span>
      </div>
      <input type="range" id="pan" min="__PAN_LO__" max="__PAN_HI__" value="0" step="0.5"
             aria-label="Pan angle in degrees">
    </div>

    <div class="group">
      <div class="eyebrow">Tilt &mdash; CAN&nbsp;id&nbsp;3</div>
      <div class="readout">
        <span class="val" id="tiltVal">+0.0&deg;</span>
        <span class="lim">__TILT_LO__&deg; to +__TILT_HI__&deg;</span>
      </div>
      <input type="range" id="tilt" min="__TILT_LO__" max="__TILT_HI__" value="0" step="0.5"
             aria-label="Tilt angle in degrees">
    </div>

    <div class="group">
      <div class="eyebrow">Pose</div>
      <div class="btnrow">
        <button data-pose="0,0">home</button>
        <button data-pose="0,__TILT_HI__">tilt up</button>
        <button data-pose="0,__TILT_LO__">tilt down</button>
        <button data-pose="__PAN_LO__,0">pan min</button>
        <button data-pose="__PAN_HI__,0">pan max</button>
        <button id="sweep" aria-pressed="false">sweep</button>
      </div>
    </div>

    <div class="group">
      <div class="eyebrow">Clearance</div>
      <div class="status"><span class="dot" id="clrDot"></span><span id="clr">checking&hellip;</span></div>
      <p class="note">The cradle sits entirely beyond the tilt motor along the
        shaft, so it can never swing into the yoke column &mdash; the clearance
        is structural, not a tuned number.</p>
    </div>

    <div class="group">
      <div class="eyebrow">Key dimensions</div>
      <table class="dims" id="dimTable"></table>
    </div>

    <div class="group">
      <div class="eyebrow">Parts</div>
      <div class="legend">
        <span><span class="swatch" style="background:var(--accent)"></span>A &mdash; pan yoke</span>
        <span><span class="swatch" style="background:var(--cool)"></span>B &mdash; camera cradle</span>
        <span><span class="swatch" style="background:#6b7480"></span>C &mdash; base plate</span>
        <span><span class="swatch" style="background:#3c4249"></span>motor</span>
        <span><span class="swatch" style="background:#1d5c2f"></span>SERVO42D</span>
      </div>
      <p class="note">Nothing clamps the driver end. The OLED and its three
        buttons stay reachable &mdash; on firmware below&nbsp;v1.0.6 they are the
        only way to read the work mode and the current limit.</p>
    </div>
  </aside>
</div>

<script>
const D = __DIMS__;
const LIM = __LIMITS__;

/* ---------- tiny 3x4 transform helpers (column-major R|t) ---------- */
const I = () => [1,0,0, 0,1,0, 0,0,1, 0,0,0];
function mul(a, b) {                       // a then b?  -> b ∘ a
  const o = new Array(12);
  for (let c = 0; c < 3; c++)
    for (let r = 0; r < 3; r++)
      o[c*3+r] = a[0*3+r]*b[c*3+0] + a[1*3+r]*b[c*3+1] + a[2*3+r]*b[c*3+2];
  for (let r = 0; r < 3; r++)
    o[9+r] = a[0*3+r]*b[9+0] + a[1*3+r]*b[9+1] + a[2*3+r]*b[9+2] + a[9+r];
  return o;
}
const trans = (x,y,z) => [1,0,0, 0,1,0, 0,0,1, x,y,z];
function rotZ(d){ const t=d*Math.PI/180,c=Math.cos(t),s=Math.sin(t);
  return [c,s,0, -s,c,0, 0,0,1, 0,0,0]; }
function rotX(d){ const t=d*Math.PI/180,c=Math.cos(t),s=Math.sin(t);
  return [1,0,0, 0,c,s, 0,-s,c, 0,0,0]; }
const apply = (m,p) => [
  m[0]*p[0]+m[3]*p[1]+m[6]*p[2]+m[9],
  m[1]*p[0]+m[4]*p[1]+m[7]*p[2]+m[10],
  m[2]*p[0]+m[5]*p[1]+m[8]*p[2]+m[11],
];

/* ---------- primitives, emitted as quads ---------- */
const faces = [];
function box(m, x0,y0,z0, w,d,h, colour) {
  const p = [];
  for (const [i,j,k] of [[0,0,0],[1,0,0],[1,1,0],[0,1,0],[0,0,1],[1,0,1],[1,1,1],[0,1,1]])
    p.push(apply(m, [x0+i*w, y0+j*d, z0+k*h]));
  const q = [[0,1,2,3],[7,6,5,4],[0,4,5,1],[1,5,6,2],[2,6,7,3],[3,7,4,0]];
  for (const f of q) faces.push({ v: f.map(i => p[i]), c: colour });
}
function cyl(m, cx,cy,z0, dia, h, colour, seg = 20) {
  const r = dia/2, ring = [];
  for (let i = 0; i < seg; i++) {
    const a = i/seg*Math.PI*2;
    ring.push([cx + r*Math.cos(a), cy + r*Math.sin(a)]);
  }
  for (let i = 0; i < seg; i++) {
    const [x1,y1] = ring[i], [x2,y2] = ring[(i+1)%seg];
    faces.push({ v: [
      apply(m,[x1,y1,z0]), apply(m,[x2,y2,z0]),
      apply(m,[x2,y2,z0+h]), apply(m,[x1,y1,z0+h])], c: colour });
  }
  for (const z of [z0, z0+h])
    faces.push({ v: ring.map(([x,y]) => apply(m,[x,y,z])), c: colour });
}

/* ---------- the gimbal itself ---------- */
const C = {
  motor:  '#3c4249',
  driver: '#1d5c2f',
  oled:   '#0b0d12',
  metal:  '#9aa3ad',
  yoke:   getComputedStyle(document.documentElement).getPropertyValue('--accent').trim() || '#e0752f',
  cradle: getComputedStyle(document.documentElement).getPropertyValue('--cool').trim() || '#4aa3d8',
  base:   '#6b7480',
};

function motor(m) {
  const b = D.motor_body;
  box(m, -b/2, -b/2, -D.motor_len, b, b, D.motor_len, C.motor);
  cyl(m, 0, 0, 0, D.motor_boss_d, D.motor_boss_h, C.metal);
  cyl(m, 0, 0, 0, D.shaft_d, D.shaft_len, C.metal, 14);
  const p = D.driver_pcb;
  box(m, -p/2, -p/2, -D.motor_len - D.driver_depth, p, p, D.driver_depth, C.driver);
  // the OLED + button edge that must stay reachable
  box(m, -14, -p/2 - 1.2, -D.motor_len - D.driver_depth + 3, 28, 1.2, 9, C.oled);
}

function build(pan, tilt) {
  faces.length = 0;

  // Base plate (part C) and the pan motor hanging beneath it.
  const w = D.base_w;
  box(I(), -w/2, -w/2, 0, w, w, D.base_t, C.base);
  for (const sx of [-1,1]) for (const sy of [-1,1])
    cyl(I(), sx*(w/2 - D.leg_d/2 - 1), sy*(w/2 - D.leg_d/2 - 1), -D.leg_h, D.leg_d, D.leg_h, C.base);
  motor(I());

  // Everything from here turns with pan.
  const P = rotZ(pan);

  // Part A: hub, low beam, column, motor face.
  cyl(P, 0, 0, D.pan_hub_z, D.pan_hub_od, D.pan_hub_h, C.yoke);
  const beamLen = Math.abs(D.tilt_face_y) + D.plate_t;
  box(P, -D.col_w/2, D.tilt_face_y, D.beam_z0, D.col_w, beamLen, D.beam_h, C.yoke);
  box(P, -D.col_w/2, D.tilt_face_y, D.beam_z0, D.col_w, D.plate_t, D.tilt_axis_h - D.beam_z0, C.yoke);
  box(P, -D.tilt_plate_w/2, D.tilt_face_y, D.tilt_axis_h - D.tilt_plate_h/2,
      D.tilt_plate_w, D.plate_t, D.tilt_plate_h, C.yoke);

  // Tilt motor, bolted to that face, shaft pointing back toward the pan axis.
  const TM = mul(rotX(-90), mul(trans(0, D.tilt_face_y + D.plate_t, D.tilt_axis_h), P));
  motor(TM);

  // Part B: the cradle, on the tilt shaft.
  const CR = mul(rotZ(tilt),
             mul(rotX(-90),
             mul(trans(0, D.tilt_face_y + D.plate_t + 4, D.tilt_axis_h), P)));
  cyl(CR, 0, 0, 0, D.cradle_hub_od, D.cradle_hub_h, C.cradle);
  const yTop = -D.plat_drop - D.plat_t;
  box(CR, -D.plat_w/2, yTop, 0, D.plat_w, D.plat_t, D.plat_l, C.cradle);
  box(CR, -D.plat_w/2, yTop, 0, D.plat_w, D.plat_t + 2, D.cradle_hub_h, C.cradle);
  for (const s of [-1,1])
    box(CR, s*(D.plat_w/2 - 3.2), yTop - D.rib_h, 0, 3.2, D.plat_t + D.rib_h, D.plat_l, C.cradle);
  // Stand-in camera. It sits on the platform's inner face and extends back
  // across the tilt axis, which is the whole point of keeping plat_drop small:
  // the body straddles the axis instead of hanging off it.
  box(CR, -26, -D.plat_drop, 14, 52, 44, 34, '#23272e');
}

/* ---------- render ---------- */
const cv = document.getElementById('view');
const ctx = cv.getContext('2d');
let yaw = -38, pitch = 22, zoom = 1.0, dragging = false, lx = 0, ly = 0;

/* The view fits itself to the model once, at the home pose, and then holds
   that framing. Refitting on every pose change would make the gimbal appear to
   shrink as it reaches, which reads as a bug rather than as motion. */
let centre = [0,0,0], radius = 1, fitted = false;
function fitView() {
  const mn = [1e9,1e9,1e9], mx = [-1e9,-1e9,-1e9];
  for (const f of faces) for (const v of f.v) for (let i = 0; i < 3; i++) {
    if (v[i] < mn[i]) mn[i] = v[i];
    if (v[i] > mx[i]) mx[i] = v[i];
  }
  centre = [0,1,2].map(i => (mn[i] + mx[i]) / 2);
  radius = Math.hypot(mx[0]-mn[0], mx[1]-mn[1], mx[2]-mn[2]) / 2 || 1;
  fitted = true;
}

function shade(hex, k) {
  const n = parseInt(hex.replace('#',''), 16);
  const r = Math.min(255, ((n>>16)&255)*k) | 0;
  const g = Math.min(255, ((n>>8)&255)*k) | 0;
  const b = Math.min(255, (n&255)*k) | 0;
  return `rgb(${r},${g},${b})`;
}

function draw() {
  const dpr = window.devicePixelRatio || 1;
  const w = cv.clientWidth, h = cv.clientHeight;
  if (cv.width !== w*dpr || cv.height !== h*dpr) { cv.width = w*dpr; cv.height = h*dpr; }
  ctx.setTransform(dpr,0,0,dpr,0,0);
  ctx.clearRect(0,0,w,h);

  const cy = Math.cos(yaw*Math.PI/180), sy = Math.sin(yaw*Math.PI/180);
  const cp = Math.cos(pitch*Math.PI/180), sp = Math.sin(pitch*Math.PI/180);
  if (!fitted) fitView();
  const s = Math.min(w,h) * 0.44 / radius * zoom;
  const cxp = w/2, cyp = h/2;

  const proj = faces.map(f => {
    const pts = f.v.map(([px,py,pz]) => {
      const x = px - centre[0], y = py - centre[1], z = pz - centre[2];
      const X = x*cy - y*sy, Y = x*sy + y*cy;
      const Z = Y*sp + z*cp, dep = Y*cp - z*sp;
      return { x: cxp + X*s, y: cyp - Z*s, d: dep };
    });
    // face normal in view space, for flat shading
    const a = pts[0], b = pts[1], c = pts[2];
    const nz = (b.x-a.x)*(c.y-a.y) - (b.y-a.y)*(c.x-a.x);
    return { pts, c: f.c, d: pts.reduce((t,p)=>t+p.d,0)/pts.length, nz };
  });
  proj.sort((p,q) => q.d - p.d);

  for (const f of proj) {
    ctx.beginPath();
    ctx.moveTo(f.pts[0].x, f.pts[0].y);
    for (let i = 1; i < f.pts.length; i++) ctx.lineTo(f.pts[i].x, f.pts[i].y);
    ctx.closePath();
    const k = f.nz < 0 ? 0.62 : 1.0;
    ctx.fillStyle = shade(f.c, k);
    ctx.fill();
    ctx.strokeStyle = 'rgba(0,0,0,.35)';
    ctx.lineWidth = 0.6;
    ctx.stroke();
  }
}

/* ---------- interaction ---------- */
const panEl = document.getElementById('pan');
const tiltEl = document.getElementById('tilt');
const poseEl = document.getElementById('pose');

function update() {
  const p = +panEl.value, t = +tiltEl.value;
  document.getElementById('panVal').textContent  = (p>=0?'+':'') + p.toFixed(1) + '\\u00b0';
  document.getElementById('tiltVal').textContent = (t>=0?'+':'') + t.toFixed(1) + '\\u00b0';
  poseEl.textContent = `pan ${p>=0?'+':''}${p.toFixed(1)}\\u00b0   tilt ${t>=0?'+':''}${t.toFixed(1)}\\u00b0`;
  build(p, t);
  draw();
}

cv.addEventListener('pointerdown', e => { dragging = true; lx = e.clientX; ly = e.clientY;
  cv.setPointerCapture(e.pointerId); });
cv.addEventListener('pointermove', e => {
  if (!dragging) return;
  yaw += (e.clientX - lx) * 0.5;
  pitch = Math.max(-85, Math.min(85, pitch + (e.clientY - ly) * 0.4));
  lx = e.clientX; ly = e.clientY; draw();
});
cv.addEventListener('pointerup', e => { dragging = false; cv.releasePointerCapture(e.pointerId); });
cv.addEventListener('wheel', e => {
  e.preventDefault();
  zoom = Math.max(0.4, Math.min(4, zoom * (e.deltaY > 0 ? 0.9 : 1.1)));
  draw();
}, { passive: false });

panEl.addEventListener('input', update);
tiltEl.addEventListener('input', update);
for (const b of document.querySelectorAll('button[data-pose]')) {
  b.addEventListener('click', () => {
    const [p,t] = b.dataset.pose.split(',').map(Number);
    panEl.value = p; tiltEl.value = t; update();
  });
}

let sweeping = null;
const sweepBtn = document.getElementById('sweep');
sweepBtn.addEventListener('click', () => {
  if (sweeping) {
    cancelAnimationFrame(sweeping); sweeping = null;
    sweepBtn.setAttribute('aria-pressed', 'false');
    return;
  }
  sweepBtn.setAttribute('aria-pressed', 'true');
  const t0 = performance.now();
  const step = now => {
    const s = (now - t0) / 1000;
    panEl.value  = (LIM.pan[1] * 0.85 * Math.sin(s * 0.7)).toFixed(1);
    const mid = (LIM.tilt[0] + LIM.tilt[1]) / 2, amp = (LIM.tilt[1] - LIM.tilt[0]) / 2;
    tiltEl.value = (mid + amp * 0.8 * Math.sin(s * 0.45)).toFixed(1);
    update();
    sweeping = requestAnimationFrame(step);
  };
  sweeping = requestAnimationFrame(step);
});

if (window.matchMedia('(prefers-reduced-motion: reduce)').matches) sweepBtn.disabled = true;

/* dimension table */
const rows = [
  ['tilt axis height', D.tilt_axis_h + ' mm'],
  ['pan hub bore', D.shaft_d + ' mm D-shaft'],
  ['motor face pitch', '31.0 mm sq, M3'],
  ['camera face offset', D.plat_drop + ' mm from axis'],
  ['platform', D.plat_l + ' \\u00d7 ' + D.plat_w + ' mm'],
  ['base plate', D.base_w + ' mm sq'],
  ['leg height', D.leg_h + ' mm'],
];
document.getElementById('dimTable').innerHTML =
  rows.map(([k,v]) => `<tr><td>${k}</td><td>${v}</td></tr>`).join('');

const clr = __CLEARANCE__;
document.getElementById('clr').textContent =
  `min ${clr.min.toFixed(1)} mm at ${clr.where}`;
document.getElementById('clrDot').style.background =
  clr.min >= 3 ? 'var(--ok)' : 'var(--warn)';

window.addEventListener('resize', draw);
update();
</script>
"""


def worst_clearance(values: dict) -> dict:
    """Runs the real clearance sweep so the viewer cannot quote a stale number.

    Args:
        values: Dimension table from the .scad file.

    Returns:
        `{"min": mm, "where": label}` for the tightest point in the sweep.
    """
    pts = cradle_points(values)
    best = (1e9, "")
    for tilt in np.arange(TILT_LIMITS[0], TILT_LIMITS[1] + 0.5, 0.5):
        g = to_global(pts, float(tilt), values)
        for label, lo, hi in obstacles(values):
            c = clearance(g, lo, hi)
            if c < best[0]:
                best = (c, label)
    return {"min": round(best[0], 2), "where": best[1]}


def main() -> int:
    values = scad_values(SCAD)
    missing = [k for k in WANTED if k not in values]
    if missing:
        print(f"missing from {SCAD.name}: {missing}")
        return 1
    dims = {k: round(values[k], 3) for k in WANTED}

    html = (
        HTML.replace("__DIMS__", json.dumps(dims, indent=None))
        .replace("__LIMITS__", json.dumps({"pan": list(PAN_LIMITS), "tilt": list(TILT_LIMITS)}))
        .replace("__PAN_LO__", f"{PAN_LIMITS[0]:.0f}")
        .replace("__PAN_HI__", f"{PAN_LIMITS[1]:.0f}")
        .replace("__TILT_LO__", f"{TILT_LIMITS[0]:.0f}")
        .replace("__TILT_HI__", f"{TILT_LIMITS[1]:.0f}")
        .replace("__CLEARANCE__", json.dumps(worst_clearance(values)))
    )
    OUT.write_text(html, encoding="utf-8")
    print(f"wrote {OUT.name} ({len(html)/1024:.1f} kB) from {len(dims)} dimensions")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
