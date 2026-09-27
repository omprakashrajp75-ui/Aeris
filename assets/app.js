/* Aeris companion app - front end.

   Data contract is unchanged:
     socket "live"   -> NOW + baseline, pushed once per second
     socket "alerts" -> today's alerts, pushed after a false-alarm mark
     GET /api/live | /api/strain | /api/alerts, re-pulled every 30s
     emit "mark_false_alarm" {ts}
   The chart plots `strain` (the server's active_strain), never psi.
   No image data is transferred anywhere.

   Tabs render from cached state - switching tabs never triggers a fetch. The
   socket keeps updating the cache in the background whatever tab is showing. */

const REST_POLL_MS = 30000;
const RING_CIRCUMFERENCE = 2 * Math.PI * 52;

const state = {
  tab: "home",
  live: null,    // last "live" payload
  strain: null,  // last /api/strain payload
  alerts: null,  // last /api/alerts payload
};

const $ = (id) => document.getElementById(id);

/* ---------------------------------------------------------------- helpers */

// Mirrors aeris.indices.psi_band().
function bandOf(v) {
  if (v === null || v === undefined) return null;
  if (v <= 2) return "NONE";
  if (v <= 4) return "LOW";
  if (v <= 6) return "MODERATE";
  if (v <= 8) return "HIGH";
  return "VERY_HIGH";
}

function fmt(v, digits) {
  return (v === null || v === undefined || Number.isNaN(v)) ? null : Number(v).toFixed(digits);
}

function hhmm(unixSeconds) {
  const d = new Date(unixSeconds * 1000);
  return String(d.getHours()).padStart(2, "0") + ":" + String(d.getMinutes()).padStart(2, "0");
}

function duration(seconds) {
  if (seconds === null || seconds === undefined) return "—";
  if (seconds < 60) return seconds + "s";
  return Math.floor(seconds / 60) + "m " + String(seconds % 60).padStart(2, "0") + "s";
}

/* Sets one stat/row field. A null value means "not currently valid": the field
   goes muted and shows an em dash rather than the last good number. */
function setField(id, value, note) {
  const el = $(id);
  if (!el) return;
  const valueEl = el.querySelector("[data-v]");
  const noteEl = el.querySelector("[data-note]");
  const stale = value === null || value === undefined;
  el.classList.toggle("stale", stale);
  if (valueEl) valueEl.textContent = stale ? "—" : value;
  if (noteEl && note !== undefined) noteEl.textContent = note;
}

function setBandDot(el, band) {
  if (!el) return;
  el.className = "dot" + (band ? " band-" + band : "");
}

/* The uncalibrated notice must appear wherever strain is shown, so it is
   rendered into whichever panel is currently mounted. */
function applyCalibrationNotice(root, live) {
  const note = root.querySelector(".note");
  if (!note) return;
  if (!live || live.calibrated) { note.hidden = true; return; }
  note.hidden = false;
  note.querySelector("[data-calib-text]").textContent =
    "Core-temperature model is uncalibrated — its coefficients are placeholders, " +
    "so full PSI is neither shown nor graded on. Strain here is the heart-rate " +
    "term only" + (live.strain_source ? " (" + live.strain_source + ")." : ".");
}

/* ------------------------------------------------------------ connection */

function setConn(mode, timeText) {
  const c = $("conn");
  if (!c) return;
  c.classList.remove("live", "down");
  if (mode === "live") c.classList.add("live");
  if (mode === "down") c.classList.add("down");
  $("conn-time").textContent = timeText;
}

/* ------------------------------------------------------------- 1. HOME */

const VITAL_TILES = ["tile-hr", "tile-spo2", "tile-skin", "tile-motion", "tile-amb", "tile-rh"];

function renderHome() {
  const root = $("screen");
  const d = state.live;
  const hero = $("hero");
  if (!hero) return;

  applyCalibrationNotice(root, d);

  // No reading yet.
  if (!d || d.status === "starting_up") {
    blankLivePanel("Aeris hasn't received a reading from the board yet. " +
                   "Check the wearable is powered and the sketch is running.");
    return;
  }

  // Bridge failed this tick: blank the whole live panel rather than leaving
  // the previous second's numbers on screen looking live.
  if (d.status === "bridge_error") {
    blankLivePanel("The sensor bridge did not respond. Values are hidden " +
                   "rather than shown stale — they'll return automatically.");
    return;
  }

  $("home-empty").hidden = true;

  const strain = fmt(d.strain, 1);
  hero.classList.toggle("stale", strain === null);
  hero.style.setProperty("--band-color", d.band ? "var(--band-" + d.band + ")" : "var(--band-NONE)");
  $("strain-value").textContent = strain === null ? "—" : strain;
  $("band-label").textContent = d.band || "Waiting for a resting reference";
  setBandDot($("band-dot"), d.band);

  const srcChip = $("hero-source");
  srcChip.hidden = !d.strain_source;
  if (d.strain_source) srcChip.textContent = d.strain_source;

  setField("tile-hr", d.hr_valid ? fmt(d.hr, 1) : null, d.hr_valid ? "" : "no valid pulse");
  setField("tile-spo2", d.spo2_valid ? fmt(d.spo2, 1) : null, d.spo2_valid ? "" : "signal too weak");
  setField("tile-skin", fmt(d.skin_c, 2), d.skin_c === null ? "sensor unavailable" : "");
  setField("tile-motion", d.motion_state || null,
           (d.motion_g === null || d.motion_g === undefined) ? "sensor unavailable" : fmt(d.motion_g, 3) + " g");
  setField("tile-amb", fmt(d.amb_c, 2));
  setField("tile-rh", fmt(d.rh, 1));
}

function blankLivePanel(message) {
  const hero = $("hero");
  hero.classList.add("stale");
  hero.style.setProperty("--band-color", "var(--band-NONE)");
  $("strain-value").textContent = "—";
  $("band-label").textContent = "No data";
  setBandDot($("band-dot"), null);
  $("hero-source").hidden = true;
  VITAL_TILES.forEach((id) => setField(id, null, ""));
  $("home-empty").hidden = false;
  $("home-empty-text").textContent = message;
}

/* ----------------------------------------------------------- 2. TRENDS */

const SVG_NS = "http://www.w3.org/2000/svg";

function svgEl(name, attrs, text) {
  const n = document.createElementNS(SVG_NS, name);
  for (const k in attrs) n.setAttribute(k, attrs[k]);
  if (text !== undefined) n.textContent = text;
  return n;
}

function renderTrends() {
  const root = $("screen");
  const svg = $("chart");
  if (!svg) return;

  applyCalibrationNotice(root, state.live);

  const data = state.strain;
  const points = (data && data.points) || [];
  const card = svg.closest(".chart-card");

  $("chart-title").textContent = "Strain · last " + (data ? data.window_hours : 8) + "h";
  const srcChip = $("chart-source");
  srcChip.hidden = !(data && data.source);
  if (data && data.source) srcChip.textContent = data.source;

  if (!points.length) {
    card.hidden = true;
    $("chart-empty").hidden = false;
    $("chart-table-wrap").hidden = true;
    return;
  }
  card.hidden = false;
  $("chart-empty").hidden = true;
  $("chart-table-wrap").hidden = false;

  svg.textContent = "";

  const W = Math.max(280, svg.clientWidth || 380);
  const H = 210;
  const M = { top: 10, right: 48, bottom: 20, left: 24 };
  svg.setAttribute("viewBox", "0 0 " + W + " " + H);

  const x0 = data.since;
  const x1 = Math.max(Date.now() / 1000, points[points.length - 1].ts);
  const plotW = W - M.left - M.right;
  const plotH = H - M.top - M.bottom;

  const sx = (t) => M.left + ((t - x0) / Math.max(1, x1 - x0)) * plotW;
  const sy = (v) => M.top + plotH - (Math.max(0, Math.min(10, v)) / 10) * plotH;

  // Fixed 0-10 scale, so the axis never rescales under the reader.
  for (let v = 0; v <= 10; v += 2) {
    svg.appendChild(svgEl("line", { class: "grid-line", x1: M.left, x2: M.left + plotW, y1: sy(v), y2: sy(v) }));
    svg.appendChild(svgEl("text", { class: "tick-text", x: M.left - 5, y: sy(v) + 3, "text-anchor": "end" }, String(v)));
  }

  // Band thresholds, labelled in the right gutter.
  (data.thresholds || []).forEach((t) => {
    if (t.upper >= 10) return;
    svg.appendChild(svgEl("line", {
      class: "threshold-line", x1: M.left, x2: M.left + plotW, y1: sy(t.upper), y2: sy(t.upper),
    }));
    svg.appendChild(svgEl("text", {
      class: "threshold-text", x: M.left + plotW + 5, y: sy(t.upper) + 3,
    }, t.label));
  });

  svg.appendChild(svgEl("line", {
    class: "axis-line", x1: M.left, x2: M.left + plotW, y1: M.top + plotH, y2: M.top + plotH,
  }));

  const ticks = W < 340 ? 2 : 3;
  for (let i = 0; i <= ticks; i++) {
    const t = x0 + ((x1 - x0) * i) / ticks;
    const anchor = i === 0 ? "start" : (i === ticks ? "end" : "middle");
    svg.appendChild(svgEl("text", {
      class: "tick-text", x: sx(t), y: H - 6, "text-anchor": anchor,
    }, hhmm(t)));
  }

  const line = points.map((p) => sx(p.ts) + "," + sy(p.strain)).join(" ");
  svg.appendChild(svgEl("polygon", {
    class: "series-area",
    points: sx(points[0].ts) + "," + (M.top + plotH) + " " + line + " " +
            sx(points[points.length - 1].ts) + "," + (M.top + plotH),
  }));
  svg.appendChild(svgEl("polyline", { class: "series-line", points: line }));

  // Hover / touch layer.
  const cross = svgEl("line", { class: "crosshair", y1: M.top, y2: M.top + plotH, opacity: 0 });
  const dot = svgEl("circle", { class: "hover-dot", r: 4.5, opacity: 0 });
  svg.appendChild(cross);
  svg.appendChild(dot);

  const hit = svgEl("rect", { x: M.left, y: M.top, width: plotW, height: plotH, fill: "transparent" });
  svg.appendChild(hit);

  const tooltip = $("tooltip");
  const track = (clientX) => {
    const box = svg.getBoundingClientRect();
    const px = ((clientX - box.left) / box.width) * W;
    const t = x0 + ((px - M.left) / plotW) * (x1 - x0);
    let best = points[0];
    for (const p of points) if (Math.abs(p.ts - t) < Math.abs(best.ts - t)) best = p;

    const cx = sx(best.ts), cy = sy(best.strain);
    cross.setAttribute("x1", cx); cross.setAttribute("x2", cx); cross.setAttribute("opacity", 1);
    dot.setAttribute("cx", cx); dot.setAttribute("cy", cy); dot.setAttribute("opacity", 1);

    const band = bandOf(best.strain);
    tooltip.hidden = false;
    tooltip.innerHTML =
      '<span class="tt-time">' + hhmm(best.ts) + "</span>" +
      '<span class="tt-row"><span class="dot band-' + band + '"></span>' +
      "<strong>" + best.strain.toFixed(1) + "</strong> " + band + "</span>";
    const left = (cx / W) * box.width;
    tooltip.style.left = Math.min(box.width - tooltip.offsetWidth - 4, Math.max(4, left - tooltip.offsetWidth / 2)) + "px";
    tooltip.style.top = Math.max(2, (cy / H) * box.height - 46) + "px";
  };
  const clear = () => {
    cross.setAttribute("opacity", 0);
    dot.setAttribute("opacity", 0);
    tooltip.hidden = true;
  };

  hit.addEventListener("mousemove", (e) => track(e.clientX));
  hit.addEventListener("mouseleave", clear);
  hit.addEventListener("touchstart", (e) => track(e.touches[0].clientX), { passive: true });
  hit.addEventListener("touchmove", (e) => track(e.touches[0].clientX), { passive: true });
  hit.addEventListener("touchend", clear);

  const tbody = $("chart-table");
  tbody.textContent = "";
  points.forEach((p) => {
    const tr = document.createElement("tr");
    tr.innerHTML = "<td>" + hhmm(p.ts) + "</td><td>" + p.strain.toFixed(1) +
                   "</td><td>" + bandOf(p.strain) + "</td>";
    tbody.appendChild(tr);
  });
}

/* --------------------------------------------------------- 3. BASELINE */

function renderBaseline() {
  const ring = $("ring-fill");
  if (!ring) return;

  const b = state.live && state.live.baseline;
  const empty = $("base-empty");
  const card = ring.closest(".ring-card");

  if (!b) {
    card.hidden = true;
    empty.hidden = false;
    return;
  }

  // Nothing learned at all yet - designed empty state instead of zeros.
  if (!b.still_samples) {
    card.hidden = true;
    empty.hidden = false;
    setField("tile-bmean", null);
    setField("tile-bsd", null);
    setField("tile-bstate", "Not yet");
    return;
  }

  card.hidden = false;
  empty.hidden = true;

  const pct = Math.min(1, b.still_samples / b.min_still_samples);
  ring.setAttribute("stroke-dasharray", RING_CIRCUMFERENCE.toFixed(2));
  ring.setAttribute("stroke-dashoffset", (RING_CIRCUMFERENCE * (1 - pct)).toFixed(2));

  $("ring-count").textContent = b.still_samples;
  $("ring-total").textContent = "of " + b.min_still_samples;

  const pill = $("base-pill");
  pill.textContent = b.is_ready ? "Ready" : "Learning";
  pill.classList.toggle("ready", !!b.is_ready);

  $("base-explain").textContent = b.is_ready
    ? "Your resting heart rate is established. Aeris will flag a reading more " +
      "than 3 standard deviations away from it while you are still."
    : "Aeris learns your resting heart rate only while you are completely still. " +
      b.samples_until_ready + " more still seconds to go — sit or lie down " +
      "without moving and the ring will fill.";

  setField("tile-bmean", fmt(b.hr_mean, 1));
  setField("tile-bsd", fmt(b.hr_sd, 2));
  setField("tile-bstate", b.is_ready ? "Active" : "Waiting");
}

/* ----------------------------------------------------------- 4. ALERTS */

function renderAlerts() {
  const list = $("alerts-list");
  if (!list) return;

  const rows = (state.alerts && state.alerts.alerts) || [];
  list.textContent = "";
  $("alerts-empty").hidden = rows.length > 0;

  rows.forEach((a) => {
    const li = document.createElement("li");
    li.className = "alert-item" + (a.false_alarm ? " is-false" : "");

    const head = document.createElement("div");
    head.className = "alert-head";
    head.innerHTML =
      '<span class="dot band-' + a.band + '"></span>' +
      '<span class="alert-band">' + a.band + "</span>" +
      '<span class="alert-time">' + hhmm(a.ts) + "</span>";

    const meta = document.createElement("div");
    meta.className = "alert-meta";
    meta.innerHTML =
      "<span>Acknowledged in <b>" +
        (a.acked_ts === null ? "— not yet" : duration(a.ack_seconds)) + "</b></span>" +
      "<span>Source <b>" + (a.strain_source || "—") + "</b></span>";

    const btn = document.createElement("button");
    btn.className = "btn";
    btn.type = "button";
    btn.textContent = a.false_alarm ? "Marked false alarm" : "Mark false alarm";
    btn.disabled = !!a.false_alarm;
    btn.addEventListener("click", () => {
      btn.disabled = true;
      btn.textContent = "Marked false alarm";
      socket.emit("mark_false_alarm", { ts: a.ts });
    });

    li.append(head, meta, btn);
    list.appendChild(li);
  });
}

/* ------------------------------------------------------------- routing */

const RENDERERS = {
  home: renderHome,
  trends: renderTrends,
  baseline: renderBaseline,
  alerts: renderAlerts,
};

/* Mounts one panel. Only the active tab's markup exists in the document.
   Renders from cached state - never fetches. */
function showTab(name) {
  state.tab = name;

  document.querySelectorAll(".tab").forEach((btn) => {
    if (btn.dataset.tab === name) btn.setAttribute("aria-current", "page");
    else btn.removeAttribute("aria-current");
  });

  const tpl = $("tpl-" + name);
  const screen = $("screen");
  screen.replaceChildren(tpl.content.cloneNode(true));
  screen.scrollTop = 0;

  RENDERERS[name]();
}

// Re-render the active panel when its data changes.
function refresh(...tabs) {
  if (tabs.includes(state.tab)) RENDERERS[state.tab]();
}

/* One second of live data. Home and Baseline read it directly. Trends only
   needs its calibration notice refreshed - rebuilding the chart every second
   would wipe an in-progress tooltip and burn CPU for no new chart data (the
   series only changes on the 30s REST pull). */
function applyLive(d) {
  state.live = d;

  const status = d && d.status;
  if (status === "ok") setConn("live", hhmm(d.ts));
  else if (status === "bridge_error") setConn("down", d && d.ts ? hhmm(d.ts) : "--:--");
  else setConn("waiting", "starting…");

  refresh("home", "baseline");
  if (state.tab === "trends") applyCalibrationNotice($("screen"), d);
}

/* ---------------------------------------------------------------- wiring */

$("tabbar").addEventListener("click", (e) => {
  const btn = e.target.closest(".tab");
  if (btn) showTab(btn.dataset.tab);
});

const socket = io();

socket.on("connect", () => setConn("waiting", "connected"));
socket.on("disconnect", () => setConn("down", "offline"));

socket.on("live", applyLive);

socket.on("alerts", (d) => {
  state.alerts = d;
  refresh("alerts");
});

async function pullRest() {
  try {
    const [strainRes, alertsRes] = await Promise.all([fetch("/api/strain"), fetch("/api/alerts")]);
    state.strain = await strainRes.json();
    state.alerts = await alertsRes.json();
    refresh("trends", "alerts");
  } catch (e) {
    // Leave the last good render in place; the socket surfaces connectivity.
    console.warn("REST pull failed", e);
  }
}

showTab("home");

// First paint before the first socket tick lands.
fetch("/api/live")
  .then((r) => r.json())
  .then(applyLive)
  .catch(() => {});

pullRest();
setInterval(pullRest, REST_POLL_MS);

let resizeTimer = null;
window.addEventListener("resize", () => {
  clearTimeout(resizeTimer);
  resizeTimer = setTimeout(() => { if (state.tab === "trends") renderTrends(); }, 150);
});
