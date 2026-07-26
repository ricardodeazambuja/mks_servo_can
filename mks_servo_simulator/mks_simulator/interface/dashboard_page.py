"""
The browser dashboard served by the debug API.

A single self-contained HTML page, held here as a string so the simulator has
no static-file directory to package, no build step, and no external asset it
could fail to find at runtime.

It renders exactly what `/status` returns - the same JSON an agent consumes -
so a human and an agent looking at the same simulator cannot be told different
things. That property is the reason this exists rather than a fifth
hand-written view of the motor state.

Everything is inlined deliberately: no CDN, no fonts, no framework. The page
must work on a machine with a CAN adapter and no internet.
"""

DASHBOARD_HTML = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>MKS Servo Simulator</title>
<!-- Inline, so the page makes no request the CSP or an offline bench would
     refuse, and the browser does not log a 404 for /favicon.ico. -->
<link rel="icon" href="data:image/svg+xml,<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 16 16'><circle cx='8' cy='8' r='6' fill='none' stroke='%234fc3f7' stroke-width='2'/><circle cx='8' cy='8' r='1.5' fill='%234fc3f7'/></svg>">
<style>
  :root {
    --bg: #0e1116;
    --panel: #161b22;
    --line: #272e38;
    --text: #d6deeb;
    --dim: #7d8899;
    --accent: #4fc3f7;
    --ok: #6bd968;
    --warn: #f0b849;
    --bad: #f2626b;
    --tx: #4fc3f7;
    --rx: #b48ead;
  }
  @media (prefers-color-scheme: light) {
    :root {
      --bg: #f6f8fa; --panel: #ffffff; --line: #d8dee4; --text: #1f2328;
      --dim: #636c76; --accent: #0969da; --ok: #1a7f37; --warn: #9a6700;
      --bad: #cf222e; --tx: #0969da; --rx: #8250df;
    }
  }
  * { box-sizing: border-box; }
  body {
    margin: 0; background: var(--bg); color: var(--text);
    font: 14px/1.5 ui-monospace, "SF Mono", Menlo, Consolas, monospace;
  }
  header {
    display: flex; align-items: baseline; gap: 1.5rem; flex-wrap: wrap;
    padding: 0.75rem 1rem; border-bottom: 1px solid var(--line);
    background: var(--panel); position: sticky; top: 0; z-index: 10;
  }
  h1 { font-size: 1rem; margin: 0; font-weight: 600; letter-spacing: 0.02em; }
  .stat { color: var(--dim); font-size: 0.85rem; }
  .stat b { color: var(--text); font-weight: 600; }
  #conn { margin-left: auto; font-size: 0.85rem; }
  #conn.up { color: var(--ok); }
  #conn.down { color: var(--bad); }
  main { padding: 1rem; display: grid; gap: 1rem; }
  section {
    background: var(--panel); border: 1px solid var(--line); border-radius: 6px;
    overflow: hidden;
  }
  section > h2 {
    font-size: 0.75rem; text-transform: uppercase; letter-spacing: 0.08em;
    color: var(--dim); margin: 0; padding: 0.6rem 0.9rem;
    border-bottom: 1px solid var(--line); font-weight: 600;
  }
  .scroll { overflow-x: auto; }
  table { border-collapse: collapse; width: 100%; font-variant-numeric: tabular-nums; }
  th, td { padding: 0.45rem 0.9rem; text-align: right; white-space: nowrap; }
  th:first-child, td:first-child, th.l, td.l { text-align: left; }
  thead th {
    font-size: 0.7rem; text-transform: uppercase; letter-spacing: 0.06em;
    color: var(--dim); font-weight: 600; border-bottom: 1px solid var(--line);
  }
  tbody tr { border-bottom: 1px solid var(--line); cursor: pointer; }
  tbody tr:last-child { border-bottom: 0; }
  tbody tr:hover { background: rgba(127,127,127,0.08); }
  tbody tr.sel { background: rgba(79,195,247,0.12); }
  .pill {
    display: inline-block; padding: 0.05rem 0.5rem; border-radius: 999px;
    font-size: 0.75rem; border: 1px solid currentColor;
  }
  .moving { color: var(--ok); }
  .idle { color: var(--dim); }
  .fault { color: var(--bad); }
  .off { color: var(--warn); }
  .muted { color: var(--dim); }
  #plot { display: block; width: 100%; height: 220px; }
  .legend { padding: 0 0.9rem 0.6rem; font-size: 0.78rem; color: var(--dim); }
  .legend i { display: inline-block; width: 1.6rem; height: 2px; vertical-align: middle; margin-right: 0.35rem; }
  #log, #errors { max-height: 300px; overflow-y: auto; }
  #log table, #errors table { font-size: 0.82rem; }
  /* An anomaly's description is the whole value of the entry, so it wraps
     rather than being clipped at the panel edge. */
  #errors td.desc { white-space: normal; text-align: left; line-height: 1.45; }
  #errors td.when, #errors td.who { vertical-align: top; }
  .dir-tx { color: var(--tx); }
  .dir-rx { color: var(--rx); }
  .fail { color: var(--bad); }
  .empty { padding: 1.2rem 0.9rem; color: var(--dim); }
  @media (min-width: 1100px) {
    main { grid-template-columns: 1fr 1fr; }
    #motors, #plotwrap { grid-column: 1 / -1; }
  }
</style>
</head>
<body>
<header>
  <h1>MKS Servo Simulator</h1>
  <span class="stat">motors <b id="n-motors">-</b></span>
  <span class="stat">uptime <b id="uptime">-</b></span>
  <span class="stat">commands <b id="n-cmds">-</b></span>
  <span class="stat">rate <b id="rate">-</b></span>
  <span class="stat">latency <b id="latency">-</b></span>
  <span id="conn">connecting</span>
</header>

<main>
  <section id="motors">
    <h2>Motors</h2>
    <div class="scroll"><table>
      <thead><tr>
        <th class="l">ID</th><th class="l">state</th>
        <th>position</th><th>target</th><th>error</th>
        <th>speed</th><th>commanded</th>
        <th class="l">mode</th><th>mstep</th><th>current</th>
        <th class="l">CanRSP</th>
      </tr></thead>
      <tbody id="motor-rows"><tr><td colspan="11" class="empty">waiting for status…</td></tr></tbody>
    </table></div>
  </section>

  <section id="plotwrap">
    <h2>Position &amp; commanded target</h2>
    <canvas id="plot"></canvas>
    <div class="legend" id="plot-legend"></div>
  </section>

  <section>
    <h2>Recent commands</h2>
    <div id="log"><table>
      <thead><tr>
        <th class="l">time</th><th>motor</th><th class="l">code</th>
        <th class="l">name</th><th>resp</th><th class="l">result</th>
      </tr></thead>
      <tbody id="log-rows"><tr><td colspan="6" class="empty">no commands yet</td></tr></tbody>
    </table></div>
  </section>

  <section>
    <h2>Errors</h2>
    <div id="errors"><div class="empty">none</div></div>
  </section>
</main>

<script>
"use strict";

// Poll interval. Fast enough to look live, slow enough that a browser tab is
// never the reason the simulation misses its tick.
const PERIOD_MS = 250;
// Roughly two minutes of history at the poll rate.
const HISTORY = 480;

const history = new Map();   // can_id -> {t, pos, target}[]
const colours = ["#4fc3f7", "#6bd968", "#f0b849", "#b48ead", "#f2626b", "#5ccfe6"];
let selected = null;
let failures = 0;

const $ = (id) => document.getElementById(id);
const fmt = (v, d = 2) => (v === null || v === undefined ? "—" : v.toFixed(d));

function motorClass(m) {
  if (m.stalled || m.protected) return "fault";
  if (!m.enabled) return "off";
  return m.moving ? "moving" : "idle";
}

function motorState(m) {
  if (m.stalled) return "STALLED";
  if (m.protected) return "PROTECTED";
  if (!m.enabled) return "disabled";
  return m.status_text;
}

function renderMotors(motors) {
  const ids = Object.keys(motors).sort((a, b) => a - b);
  if (!ids.length) {
    $("motor-rows").innerHTML = '<tr><td colspan="11" class="empty">no motors</td></tr>';
    return;
  }
  if (selected === null) selected = ids[0];

  $("motor-rows").innerHTML = ids.map((id) => {
    const m = motors[id];
    const idLabel = m.can_id !== m.listening_can_id
      ? `${m.can_id} <span class="muted">→${m.listening_can_id}</span>`
      : `${m.can_id}`;
    return `<tr data-id="${id}" class="${id === selected ? "sel" : ""}">
      <td class="l">${idLabel}</td>
      <td class="l"><span class="pill ${motorClass(m)}">${motorState(m)}</span></td>
      <td>${fmt(m.position_degrees)}°</td>
      <td>${m.target_position_degrees === null ? '<span class="muted">—</span>' : fmt(m.target_position_degrees) + "°"}</td>
      <td>${m.position_error_steps === null ? '<span class="muted">—</span>' : fmt(m.position_error_steps, 0) + " ct"}</td>
      <td>${fmt(m.current_rpm, 1)} rpm</td>
      <td>${fmt(m.target_rpm, 1)} rpm</td>
      <td class="l">${m.work_mode_name}</td>
      <td>${m.microsteps}</td>
      <td>${m.working_current_ma} mA</td>
      <td class="l">${m.responses_enabled ? "on" : '<span class="off">off</span>'}${m.active_notifications_enabled ? "" : ' <span class="muted">(no CanACT)</span>'}</td>
    </tr>`;
  }).join("");

  for (const row of $("motor-rows").querySelectorAll("tr[data-id]")) {
    row.onclick = () => { selected = row.dataset.id; };
  }
}

function pushHistory(motors, t) {
  for (const [id, m] of Object.entries(motors)) {
    if (!history.has(id)) history.set(id, []);
    const series = history.get(id);
    series.push({ t, pos: m.position_degrees, target: m.target_position_degrees });
    if (series.length > HISTORY) series.shift();
  }
}

function drawPlot() {
  const canvas = $("plot");
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  canvas.width = w * dpr;
  canvas.height = h * dpr;
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);

  const style = getComputedStyle(document.documentElement);
  const dim = style.getPropertyValue("--dim").trim();
  const line = style.getPropertyValue("--line").trim();

  const ids = [...history.keys()].sort((a, b) => a - b);
  const points = ids.flatMap((id) => history.get(id));
  if (points.length < 2) {
    ctx.fillStyle = dim;
    ctx.fillText("collecting…", 12, h / 2);
    return;
  }

  const values = points.flatMap((p) => (p.target === null ? [p.pos] : [p.pos, p.target]));
  let lo = Math.min(...values);
  let hi = Math.max(...values);
  if (hi - lo < 1) { const mid = (hi + lo) / 2; lo = mid - 0.5; hi = mid + 0.5; }
  const pad = (hi - lo) * 0.1;
  lo -= pad; hi += pad;

  const t0 = Math.min(...points.map((p) => p.t));
  const t1 = Math.max(...points.map((p) => p.t));
  const span = Math.max(t1 - t0, 1e-3);

  const L = 52, R = 8, T = 10, B = 20;
  const x = (t) => L + ((t - t0) / span) * (w - L - R);
  const y = (v) => T + (1 - (v - lo) / (hi - lo)) * (h - T - B);

  // Axis: a few horizontal guides with labels, so the numbers mean something.
  ctx.strokeStyle = line;
  ctx.fillStyle = dim;
  ctx.lineWidth = 1;
  ctx.font = "11px ui-monospace, monospace";
  ctx.textAlign = "right";
  for (let i = 0; i <= 4; i++) {
    const v = lo + ((hi - lo) * i) / 4;
    const py = Math.round(y(v)) + 0.5;
    ctx.beginPath(); ctx.moveTo(L, py); ctx.lineTo(w - R, py); ctx.stroke();
    ctx.fillText(v.toFixed(1) + "°", L - 6, py + 3.5);
  }

  ids.forEach((id, i) => {
    const series = history.get(id);
    const colour = colours[i % colours.length];

    // Commanded target, drawn behind and dashed.
    ctx.setLineDash([3, 3]);
    ctx.strokeStyle = colour;
    ctx.globalAlpha = 0.55;
    ctx.beginPath();
    let open = false;
    for (const p of series) {
      if (p.target === null) { open = false; continue; }
      if (!open) { ctx.moveTo(x(p.t), y(p.target)); open = true; }
      else ctx.lineTo(x(p.t), y(p.target));
    }
    ctx.stroke();

    // Measured position, solid and on top.
    ctx.setLineDash([]);
    ctx.globalAlpha = 1;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    series.forEach((p, j) => (j ? ctx.lineTo(x(p.t), y(p.pos)) : ctx.moveTo(x(p.t), y(p.pos))));
    ctx.stroke();
  });
  ctx.globalAlpha = 1;

  $("plot-legend").innerHTML = ids.map((id, i) => {
    const c = colours[i % colours.length];
    return `<span style="margin-right:1.2rem"><i style="background:${c}"></i>motor ${id}</span>`;
  }).join("") + '<span class="muted">solid = measured, dashed = commanded target</span>';
}

function renderLog(commands) {
  if (!commands.length) return;
  $("log-rows").innerHTML = commands.slice().reverse().map((c) => {
    const t = new Date(c.timestamp * 1000).toISOString().substr(11, 12);
    return `<tr>
      <td class="l muted">${t}</td>
      <td>${c.motor_id}</td>
      <td class="l dir-tx">${c.command}</td>
      <td class="l">${c.name}</td>
      <td>${c.response_time_ms.toFixed(2)} ms</td>
      <td class="l ${c.success ? "" : "fail"}">${c.success ? "ok" : "no response"}</td>
    </tr>`;
  }).join("");
}

function renderErrors(errors) {
  if (!errors.length) {
    $("errors").innerHTML = '<div class="empty">none</div>';
    return;
  }
  $("errors").innerHTML = "<table><tbody>" + errors.slice().reverse().map((e) => {
    const t = new Date(e.timestamp * 1000).toISOString().substr(11, 12);
    return `<tr><td class="l muted when">${t}</td><td class="who">${e.motor_id}</td>
      <td class="l fail who">${e.error_type}</td><td class="desc">${e.description}</td></tr>`;
  }).join("") + "</tbody></table>";
}

function setConnection(up, detail) {
  const el = $("conn");
  el.className = up ? "up" : "down";
  el.textContent = up ? "● live" : "● " + detail;
}

async function poll() {
  try {
    const res = await fetch("status", { cache: "no-store" });
    if (!res.ok) throw new Error("HTTP " + res.status);
    const s = await res.json();
    failures = 0;
    setConnection(true);

    $("n-motors").textContent = s.motor_count;
    $("uptime").textContent = s.uptime_seconds.toFixed(0) + "s";
    $("n-cmds").textContent = s.communication.total_messages;
    $("rate").textContent = s.communication.messages_per_second.toFixed(0) + "/s";
    $("latency").textContent = s.communication.average_latency_ms.toFixed(2) + " ms";

    renderMotors(s.motors);
    pushHistory(s.motors, s.timestamp);
    drawPlot();
    renderLog(s.communication.last_10_commands);
    renderErrors(s.errors);
  } catch (err) {
    failures += 1;
    // One missed poll is noise; several in a row means the simulator went away.
    if (failures > 2) setConnection(false, String(err.message || err));
  }
}

poll();
setInterval(poll, PERIOD_MS);
window.addEventListener("resize", drawPlot);
</script>
</body>
</html>
"""
