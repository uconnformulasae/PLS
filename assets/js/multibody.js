/* Multibody page: the car as a multibody system instead of a string of steady states.
   Two parts:
   * Minimum-lap-time multibody solve (fsae_vd.mb_study: lumped-axle chassis, optimal control, CasADi + IPOPT).
     IPOPT is not in the browser build of CasADi, so these are pre-computed natively by web/export_mb.py into
     data/mb/<car>.json, for the baseline car.
   * Transient check (fsae_vd.multibody.Chassis: pitch, roll, heave, yaw and tire lag, RK4 at 2 ms, plain numpy).
     The baseline comes with the site (assets/data/mb/transient_<car>.json, from .github/build.py); "Run transient check" runs it
     in the browser on the car with the inputs as they stand (bridge transient()).
   Loaded after app.js; uses its helpers (V, plot, line, el, fmt, pretty, table, kvTable, call, setStatus). */
"use strict";

PAGES.splice(PAGES.findIndex((p) => p.id === "lap") + 1, 0,
  { id: "multibody", label: "Multibody", lap: true,
    intro: "The car as a multibody system: pitch, roll, heave, yaw and tire lag instead of a string of steady states. The minimum-lap-time solve finds the fastest racing line and inputs for the chassis model; the transient check drives the car in the time domain along the quasi-steady speed profile." });

const MB = {
  opt: {}, tr: {}, trBase: {}, trOv: {}, level: 0.98, running: false,
  load(path) { return getJSON(path).catch(() => null); },
  async ensure(car) {
    if (!(car in this.opt)) this.opt[car] = await this.load(`assets/data/mb/${car}.json`);
    if (!(car in this.trBase)) { this.trBase[car] = await this.load(`assets/data/mb/transient_${car}.json`); if (!this.tr[car]) { this.tr[car] = this.trBase[car]; this.trOv[car] = {}; } }
  },
};
const GRADE = { A: ["#0a7d33", "dynamically consistent, optimal, replay within 2 %"], B: ["#b58500", "consistent, one warning"],
  C: ["#d06a0a", "consistent, two or more warnings: read the checks"], F: ["#c0392b", "not dynamically consistent: do not use the time"] };
const CHECK_MARK = { ok: ["✓", "#0a7d33"], info: ["ℹ", "#2a78d6"], warn: ["⚠", "#b58500"], fail: ["✗", "#c0392b"] };
const CORNER_COL = { fl: "#2a78d6", fr: "#eb6834", rl: "#1baf7a", rr: "#6250d6" };
const C_OPT = "#000E2F", C_QSS = "#8a9099", C_LOG = "#eda100", C_TR = "#eb6834";
const kmh = (a) => (a || []).map((v) => (v == null ? null : v * 3.6));
const gradeTag = (g) => el("span", { class: "tag", style: `background:${(GRADE[g] || ["#7a859c"])[0]};color:#fff;font-size:12px`, title: (GRADE[g] || ["", ""])[1] }, `grade ${g || "?"}`);

/* async data for a card: the card shows "Loading" until the multibody files are in */
function withMB(b, draw) {
  const car = S.car;
  if (car in MB.opt && car in MB.trBase) return draw();
  b.append(el("div", { class: "muted" }, "Loading the multibody results…"));
  MB.ensure(car).then(() => { if (S.car !== car || !b.isConnected) return; b.replaceChildren(); try { draw(); } catch (e) { b.replaceChildren(el("div", { class: "err" }, "Not available: " + (e.message || e))); } });
}
const optEv = (ev = S.event) => { const d = MB.opt[S.car]; return d && d.events && d.events[ev] && !d.events[ev].error ? d.events[ev] : null; };
const trEv = (ev = S.event) => { const d = MB.tr[S.car]; return d && d.events && d.events[ev] && !d.events[ev].error ? d.events[ev] : null; };
function needOpt(ev = S.event) {
  const d = MB.opt[S.car];
  if (!d) throw new Error(S.car === "ev" ? "the minimum-lap-time multibody solve is built on the IC powertrain model (fsae_pt); the EV car has no multibody powertrain yet. The transient check below runs for both cars."
    : "no multibody runs exported yet. They come from export_mb.py in the team's research folder (fsae_lapsim/web/), copied to assets/data/mb/");
  const e = d.events[ev];
  if (!e) throw new Error(`${EVENT_NAMES[ev] || ev} has not been exported (python web/export_mb.py --events ${ev})`);
  if (e.error) throw new Error(e.error);
  return e;
}
function needTr(ev = S.event) {
  const d = MB.tr[S.car];
  if (!d) throw new Error("no transient check yet: press Run transient check");
  const e = d.events[ev];
  if (!e) throw new Error(`the transient check covers ${Object.keys(d.events).map((k) => EVENT_NAMES[k] || k).join(", ")}`);
  if (e.error) throw new Error(e.error);
  return e;
}

/* stacked panels on one shared distance axis: panels = [{title, series: [{name, x, y, color, dash, width}]}] */
function mbStack(node, panels) {
  panels = panels.map((p) => Object.assign({}, p, { series: p.series.filter((s) => s.y && s.y.length) })).filter((p) => p.series.length);
  if (!panels.length) return node.replaceWith(el("div", { class: "muted" }, "These channels are not in this run."));
  const n = panels.length, traces = [], layout = { grid: { rows: n, columns: 1, pattern: "independent" }, showlegend: true }, gap = 0.07, h = (1 - gap * (n - 1)) / n;
  const seen = new Set();
  panels.forEach((p, i) => {
    const ax = i === 0 ? "" : String(i + 1);
    for (const s of p.series) {
      traces.push(line(s.x, s.y, s.name, { xaxis: "x" + ax, yaxis: "y" + ax, legendgroup: s.name, showlegend: !seen.has(s.name),
        line: { width: s.width || 1.8, color: s.color, dash: s.dash } }));
      seen.add(s.name);
    }
    const top = 1 - i * (h + gap);
    layout["yaxis" + ax] = { domain: [Math.max(0, top - h), top], title: p.title };
    layout["xaxis" + ax] = { anchor: "y" + ax, matches: i ? "x" : undefined, showticklabels: i === n - 1, title: i === n - 1 ? "distance [m]" : "" };
  });
  node.style.height = Math.max(260, 80 + n * (window.innerWidth < 600 ? 140 : 165)) + "px";
  plot(node, traces, layout);
}

// ------------------------------------------------------------------ minimum-lap-time multibody solve
V("multibody", "mb_results", "Multibody against quasi-steady", "Fastest lap of the multibody chassis (optimal control) next to the quasi-steady lap and the logged runs, every exported event.", (b) => withMB(b, () => {
  const d = MB.opt[S.car];
  if (!d) { b.append(el("div", { class: "note warn" }, needOptMsg())); return; }
  const rows = Object.entries(d.events).map(([e, x]) => x.error ? [EVENT_NAMES[e] || e, null, null, null, null, null, "failed: " + x.error]
    : [EVENT_NAMES[e] || e, x.time, x.qss_time, x.qss_time ? 100 * (x.time / x.qss_time - 1) : null, x.log_time, x.replay_time, x.grade]);
  const tiles = Object.entries(d.events).filter(([, x]) => !x.error).map(([e, x]) => {
    const pct = x.qss_time ? 100 * (x.time / x.qss_time - 1) : null;
    return el("div", { class: "kpi", style: `border-top-color:${(GRADE[x.grade] || ["#000E2F"])[0]}` },
      el("div", { class: "k" }, EVENT_NAMES[e] || e), el("div", { class: "v" }, x.time.toFixed(3), el("small", {}, "s")),
      el("div", { class: "d" }, `quasi-steady ${x.qss_time != null ? x.qss_time.toFixed(3) + " s" : "–"}`),
      pct != null ? el("div", { class: "d " + (pct > 0.05 ? "up" : pct < -0.05 ? "down" : "") }, `${pct > 0 ? "+" : ""}${pct.toFixed(1)} % multibody vs quasi-steady`) : null,
      el("div", { class: "d" }, gradeTag(x.grade)));
  });
  b.append(el("div", { class: "kpis" }, tiles),
    table({ columns: ["event", "multibody optimum [s]", "quasi-steady [s]", "difference [%]", "logged [s]", "time-domain replay [s]", "grade"], rows }),
    el("p", { class: "muted", style: "font-size:12.5px" }, `Pre-computed for the baseline ${d.name || S.car.toUpperCase()} (${d.model} model, ${d.powertrain}), ${d.built}${d.commit ? ", commit " + d.commit : ""}. ` +
      "Most of the gain over the quasi-steady lap is the racing line: the quasi-steady lap drives the centre line."));
  if (Object.keys(S.applied[S.car]).length) b.append(el("div", { class: "note warn" }, "Your changed inputs are not in these numbers: the minimum-lap-time solve needs IPOPT, which does not run in a browser. The transient check below does use them."));
}), { wide: true, first: true });
const needOptMsg = () => S.car === "ev" ? "The minimum-lap-time multibody solve is built on the IC powertrain model (fsae_pt); the EV car has no multibody powertrain yet. The transient check below runs for both cars."
  : "No multibody runs exported yet. They come from export_mb.py in the team's research folder (fsae_lapsim/web/), copied to assets/data/mb/.";

V("multibody", "mb_checks", "How far to trust it", "The grade and the checks behind it, for the selected event.", (b) => withMB(b, () => {
  const x = needOpt();
  b.append(el("p", {}, gradeTag(x.grade), " ", el("span", { class: "muted" }, (GRADE[x.grade] || ["", ""])[1])),
    el("ul", { class: "prose", style: "list-style:none;padding-left:0" }, x.checks.map(([lv, t]) => { const [m, c] = CHECK_MARK[lv] || ["•", "#7a859c"];
      return el("li", { style: "margin:4px 0" }, el("b", { style: `color:${c};display:inline-block;width:1.4em` }, m), t); })),
    kvTable({ "lap time [s]": fmt(x.time, 6), "IPOPT iterations": x.iterations, "certified optimal": x.optimal, "largest constraint violation": fmt(x.constraint_violation),
      "mesh [m] / intervals": `${x.ds} / ${x.intervals}`, "track half width [m]": x.half_width, "track length [m]": fmt(x.length_m), "solve time [s]": fmt(x.solve_s, 3),
      "finer-mesh time [s]": x.mesh_time != null ? fmt(x.mesh_time, 6) : null, "logged": x.log_what }));
}), { car: "ic" });

V("multibody", "mb_line", "Optimal racing line", "Coloured by speed. Grey: track edges the optimiser had to stay inside, and the centre line the quasi-steady lap drives.", (b) => withMB(b, () => {
  const x = needOpt(), E = x.edges, p = plotDiv("tall"); b.append(p);
  const edge = (o, name, extra = {}) => Object.assign({ x: o.x, y: o.y, mode: "lines", name, hoverinfo: "skip", line: { color: "#c9ced8", width: 1 } }, extra);
  const v = kmh(x.channels.v), sx = x.channels.s;
  plot(p, [edge(E.left, "track edge"), edge(E.right, "track edge", { showlegend: false }), edge(E.centre, "centre line", { line: { color: "#c9ced8", width: 1, dash: "dash" } }),
    { x: x.line.x, y: x.line.y, mode: "markers+lines", name: "optimal line", line: { color: "rgba(0,14,47,.15)", width: 1 },
      marker: { size: 5, color: v, colorscale: SEQ, colorbar: { thickness: 10, len: 0.8, title: { text: "km/h", side: "right", font: { size: 11 } } } },
      customdata: sx.map((s, i) => [s, v[i], x.channels.n[i]]), hovertemplate: "s %{customdata[0]:.0f} m<br>%{customdata[1]:.1f} km/h<br>offset %{customdata[2]:.2f} m<extra></extra>" },
    { x: [x.line.x[0]], y: [x.line.y[0]], mode: "markers", name: "start", marker: { size: 11, symbol: "triangle-up", color: NAVY }, hoverinfo: "skip" }],
  { hovermode: "closest", xaxis: { visible: false }, yaxis: { visible: false, scaleanchor: "x" }, margin: { l: 6, r: 6, t: 28, b: 6 } });
}), { car: "ic" });

V("multibody", "mb_speed", "Speed", "Multibody optimum, quasi-steady lap and the fastest logged run.", (b) => withMB(b, () => {
  const x = needOpt(), p = plotDiv("tall"), t = [];
  if (x.log_trace) t.push(line(x.log_trace.s, kmh(x.log_trace.v), "logged", { line: { color: C_LOG, width: 1.2 } }));
  if (x.qss) t.push(line(x.qss.s, kmh(x.qss.v), `quasi-steady ${fmt(x.qss_time, 5)} s`, { line: { color: C_QSS, width: 1.5 } }));
  t.push(line(x.channels.s, kmh(x.channels.v), `multibody ${fmt(x.time, 5)} s`, { line: { color: C_OPT, width: 2 } }));
  b.append(p); plot(p, t, { xaxis: { title: "distance [m]" }, yaxis: { title: "speed [km/h]" } });
}), { wide: true, car: "ic" });

V("multibody", "mb_inputs", "Driver inputs", "Steer at the road wheels, and the drive and brake force the optimiser chose.", (b) => withMB(b, () => {
  const x = needOpt(), c = x.channels, s = c.s, p = plotDiv(); b.append(p);
  mbStack(p, [{ title: "steer [deg]", series: [{ name: "steer", x: s, y: c.delta_deg, color: NAVY }] },
    { title: "lateral offset [m]", series: [{ name: "offset from centre line", x: s, y: c.n, color: PAL[2] }] },
    { title: "force [N]", series: [{ name: "drive", x: s, y: c.F_drive, color: PAL[1] }, { name: "brake (neg.)", x: s, y: (c.F_brake || []).map((v) => -v), color: PAL[0] }] }]);
}), { wide: true, car: "ic" });

V("multibody", "mb_gg", "g-g diagram", "Multibody optimum coloured by speed, quasi-steady lap in grey.", (b) => withMB(b, () => {
  const x = needOpt(), c = x.channels, p = plotDiv("tall"), g = 9.80665, t = [];
  if (x.qss) t.push({ x: x.qss.ay.map((v) => v / g), y: x.qss.ax.map((v) => v / g), mode: "markers", name: "quasi-steady", marker: { size: 3, color: C_QSS, opacity: 0.45 }, hoverinfo: "skip" });
  t.push({ x: c.ay.map((v) => v / g), y: c.ax.map((v) => v / g), mode: "markers", name: "multibody", marker: { size: 5, color: kmh(c.v), colorscale: SEQ, showscale: true, colorbar: { thickness: 10, len: 0.8, title: { text: "km/h", side: "right", font: { size: 11 } } } },
    customdata: c.s, hovertemplate: "s %{customdata:.0f} m<br>ay %{x:.2f} g<br>ax %{y:.2f} g<extra></extra>" });
  b.append(p); plot(p, t, { hovermode: "closest", xaxis: { title: "lateral [g]", zeroline: true }, yaxis: { title: "longitudinal [g]", scaleanchor: "x", zeroline: true } });
}), { car: "ic" });

V("multibody", "mb_loads", "Wheel loads and tire use", "Each corner; tire use 1 is the friction limit.", (b) => withMB(b, () => {
  const x = needOpt(), c = x.channels, s = c.s, p = plotDiv(); b.append(p);
  const k = ["fl", "fr", "rl", "rr"];
  mbStack(p, [{ title: "wheel load [N]", series: k.map((w) => ({ name: w.toUpperCase(), x: s, y: c["Fz_" + w], color: CORNER_COL[w], width: 1.3 })) },
    { title: "tire use [-]", series: k.map((w) => ({ name: w.toUpperCase(), x: s, y: c["util_" + w], color: CORNER_COL[w], width: 1.3 })) },
    { title: "slip angle [deg]", series: k.map((w) => ({ name: w.toUpperCase(), x: s, y: c[`alpha_${w}_deg`], color: CORNER_COL[w], width: 1.3 })) }]);
}), { wide: true, car: "ic" });

V("multibody", "mb_attitude", "Body attitude", "Roll (+ right side down), pitch (+ nose down), heave (+ down), body slip and yaw rate on the optimal lap.", (b) => withMB(b, () => {
  const x = needOpt(), c = x.channels, s = c.s, p = plotDiv(); b.append(p);
  mbStack(p, [{ title: "angle [deg]", series: [{ name: "roll", x: s, y: c.roll_deg, color: PAL[0] }, { name: "pitch", x: s, y: c.pitch_deg, color: PAL[1] }] },
    { title: "heave [mm]", series: [{ name: "heave", x: s, y: c.heave_mm, color: NAVY }] },
    { title: "[deg] / [deg/s]", series: [{ name: "body slip [deg]", x: s, y: c.beta_deg, color: PAL[2] }, { name: "yaw rate [deg/s]", x: s, y: c.yaw_rate, color: PAL[6], width: 1.2 }] }]);
}), { wide: true, off: true, car: "ic" });

V("multibody", "mb_aero", "Aero on the optimal lap", "Downforce and drag at the instantaneous ride heights and speed.", (b) => withMB(b, () => {
  const x = needOpt(), c = x.channels, p = plotDiv(); b.append(p);
  mbStack(p, [{ title: "force [N]", series: [{ name: "downforce", x: c.s, y: c.downforce, color: PAL[0] }, { name: "drag", x: c.s, y: c.drag, color: PAL[1] }] }]);
}), { off: true, car: "ic" });

// ------------------------------------------------------------------ transient check (runs in the browser)
async function runTransient(btn, msg) {
  if (MB.running) return;
  const car = S.car, ov = Object.assign({}, S.overrides[car]), t0 = performance.now();
  MB.running = true; btn.disabled = true; msg.textContent = "Starting the simulation engine…";
  setStatus("Multibody transient check: starting the simulation engine…", "busy");
  const tick = setInterval(() => { msg.textContent = `Driving acceleration, skidpad and autocross in the time domain… ${Math.round((performance.now() - t0) / 1000)} s`; }, 1000);
  try {
    const r = await call("transient", [car, ov, null, MB.level]);
    MB.tr[car] = r; MB.trOv[car] = ov;
    setStatus(`Multibody transient check done in your browser in ${((performance.now() - t0) / 1000).toFixed(1)} s` + (Object.keys(ov).length ? ` with ${Object.keys(ov).length} changed input(s)` : " (baseline inputs)"), "ok");
    if (S.car === car && S.page === "multibody") render();
  } catch (e) { msg.textContent = "Failed: " + (e.message || e); setStatus("Transient check failed: " + (e.message || e), "err"); }
  clearInterval(tick); MB.running = false; btn.disabled = false;
}

V("multibody", "tr_run", "Transient check", "The car driven through each event in the time domain (pitch, roll, heave, yaw and tire lag, 2 ms steps) by a driver model following the quasi-steady speed profile. Runs in your browser with the inputs as they stand.", (b) => withMB(b, () => {
  const d = MB.tr[S.car], ov = MB.trOv[S.car] || {}, nOv = Object.keys(ov).length, nNow = Object.keys(S.overrides[S.car]).length;
  const msg = el("span", { class: "muted" });
  const lvl = el("input", { type: "number", min: 0.8, max: 1.0, step: 0.01, value: MB.level, style: "width:70px", title: "Share of the quasi-steady speed the driver aims for", onchange: () => { MB.level = Math.min(1, Math.max(0.8, +lvl.value || 0.98)); } });
  const btn = el("button", { class: "btn navy", onclick: () => runTransient(btn, msg) }, "Run transient check");
  b.append(el("div", { class: "ctl" }, "Driver level", lvl, btn, msg));
  if (!d) { b.append(el("div", { class: "muted" }, "No transient check yet: press Run transient check.")); return; }
  b.append(el("p", { class: "muted", style: "font-size:12.5px" }, nOv ? `Showing your run with ${nOv} changed input(s).` : "Showing the baseline car (pre-computed)."
    + (!same(ov, S.overrides[S.car]) && nNow ? " Inputs have changed since: run it again to include them." : "")));
  const rows = Object.entries(d.events).map(([e, x]) => {
    if (x.error) return [EVENT_NAMES[e] || e, null, null, null, "error: " + x.error, null, null, null, null, null, null];
    const sm = x.summary;
    return [EVENT_NAMES[e] || e, sm.time_s, sm.qss_time_s, sm.qss_time_s ? 100 * (sm.time_s / sm.qss_time_s - 1) : null, sm.completed ? "yes" : "left the track",
      sm.max_roll_deg, sm.max_pitch_deg, sm.max_beta_deg, sm.max_lateral_error_m, sm.min_fw_clearance_mm, sm.min_adaptive_grip_scale];
  });
  b.append(table({ columns: ["event", "transient [s]", "quasi-steady [s]", "difference [%]", "completed", "max roll [deg]", "max pitch [deg]", "max body slip [deg]", "max path error [m]", "min front-wing clearance [mm]", "min grip margin [-]"], rows }));
  const notes = [...(d.notes || []), ...Object.values(d.events).flatMap((x) => x.notes || [])];
  if (notes.length) b.append(el("ul", { class: "prose muted", style: "font-size:12.5px" }, notes.map((n) => el("li", {}, n))));
}), { wide: true });

V("multibody", "tr_attitude", "Transient attitude against quasi-steady", "Roll and pitch: the time-domain car lags and overshoots where the quasi-steady one jumps to its steady state.", (b) => withMB(b, () => {
  const x = needTr(), c = x.channels, q = x.qss || {}, s = c.s, p = plotDiv(); b.append(p);
  mbStack(p, [{ title: "speed [km/h]", series: [{ name: "quasi-steady", x: s, y: kmh(q.v), color: C_QSS, width: 1.4 }, { name: "transient", x: s, y: kmh(c.v), color: C_TR }] },
    { title: "roll [deg]", series: [{ name: "quasi-steady", x: s, y: q.roll_deg, color: C_QSS, width: 1.4 }, { name: "transient", x: s, y: c.roll_deg, color: C_TR }] },
    { title: "pitch [deg]", series: [{ name: "quasi-steady", x: s, y: q.pitch_deg, color: C_QSS, width: 1.4 }, { name: "transient", x: s, y: c.pitch_deg, color: C_TR }] }]);
}), { wide: true });

V("multibody", "tr_handling", "Transient handling", "Steer, body slip, yaw rate and how far the driver model is off the line.", (b) => withMB(b, () => {
  const x = needTr(), c = x.channels, q = x.qss || {}, s = c.s, p = plotDiv(); b.append(p);
  mbStack(p, [{ title: "steer [deg]", series: [{ name: "road-wheel steer", x: s, y: c.delta_deg, color: NAVY }] },
    { title: "body slip [deg]", series: [{ name: "quasi-steady", x: s, y: q.beta_deg, color: C_QSS, width: 1.4 }, { name: "transient", x: s, y: c.beta_deg, color: C_TR }] },
    { title: "yaw rate [deg/s]", series: [{ name: "transient", x: s, y: c.yaw_rate, color: C_TR }] },
    { title: "path error [m]", series: [{ name: "lateral error", x: s, y: c.e_lat, color: PAL[2] }] }]);
}), { wide: true, off: true });

V("multibody", "tr_loads", "Transient wheel loads", "Each corner, with the dynamic load transfer.", (b) => withMB(b, () => {
  const x = needTr(), c = x.channels, s = c.s, p = plotDiv(); b.append(p);
  mbStack(p, [{ title: "wheel load [N]", series: ["fl", "fr", "rl", "rr"].map((w) => ({ name: w.toUpperCase(), x: s, y: c["Fz_" + w], color: CORNER_COL[w], width: 1.3 })) },
    { title: "tire use [-]", series: [{ name: "most-used tire", x: s, y: c.util_max, color: NAVY, width: 1.3 }] }]);
}), { wide: true, off: true });

V("multibody", "tr_aero", "Transient aero", "Downforce and front-wing ground clearance with the body moving.", (b) => withMB(b, () => {
  const x = needTr(), c = x.channels, q = x.qss || {}, s = c.s, p = plotDiv(); b.append(p);
  mbStack(p, [{ title: "downforce [N]", series: [{ name: "quasi-steady", x: s, y: q.aero_F, color: C_QSS, width: 1.4 }, { name: "transient", x: s, y: c.aero_F, color: C_TR }] },
    { title: "front-wing clearance [mm]", series: [{ name: "front-wing clearance", x: s, y: c.fw_clear_mm, color: PAL[0] }] }]);
}), { wide: true, off: true });

V("multibody", "mb_about", "What the two models are", "", (b) => b.append(el("div", { class: "prose", html: `
<ul>
<li><b>Minimum-lap-time solve</b> (<code>fsae_vd.mb_study</code>, <code>ocp</code>, <code>chassis_sym</code>): a 15-state lumped-axle chassis in path coordinates, with the engine and gearbox drive limit, tire relaxation, heave, pitch and roll. The racing line, speed, steering, drive and brake come out of one optimisation (Radau collocation, IPOPT). It is graded A to F by its constraint violation, IPOPT's optimality, a time-domain replay of its inputs, the quasi-steady lap and the logged runs.</li>
<li><b>Transient check</b> (<code>fsae_vd.multibody</code>): the same car as the lap simulation driven in the time domain by a path-following driver model on the quasi-steady speed profile. It shows whether the quasi-steady lap is drivable and how far the attitude lags it.</li>
<li>The optimal-control solve needs IPOPT, which is not in the browser build of CasADi, so it is computed natively (<code>python web/export_mb.py</code>) for the baseline car. The transient check is plain NumPy and runs here.</li>
</ul>` })), { wide: true, off: true });
