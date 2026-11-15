/* UConn FSAE predictive lap simulation - page logic.
   Data: data/<car>.json holds the baseline (built by web/build.py). "Run simulation" re-runs the real
   Python model in the browser (assets/worker.js, Pyodide) with the edited inputs.
   To add a visual: push one entry to VISUALS (page, title, draw). */
"use strict";

// ------------------------------------------------------------------ state
const S = {
  car: "ic", page: "about", event: "autocross",
  data: { ic: null, ev: null },          // {schema, result, tracks, studies}
  base: { ic: null, ev: null },          // baseline result, for "vs baseline"
  overrides: { ic: {}, ev: {} },         // edited inputs, not yet run
  applied: { ic: {}, ev: {} },           // inputs of the result on screen
  hidden: JSON.parse(localStorage.getItem("lapsim.hidden") || "{}"),   // visual id -> true when switched off
  explorer: { x: "s", channels: ["v", "ax", "ay"], color: "v" },
  busy: false, meta: {},
};
const PAL = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#8a9099"];
const NAVY = "#000E2F";
const SEQ = [[0, "#d6e4f7"], [0.5, "#5b93dc"], [1, "#0B2152"]];           // one hue, light -> dark
const EVENT_NAMES = { acceleration: "Acceleration", skidpad: "Skidpad", autocross: "Autocross", endurance: "Endurance", efficiency: "Efficiency" };
const $ = (s, el = document) => el.querySelector(s);
const el = (tag, attrs = {}, ...kids) => {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k === "class") e.className = v; else if (k === "html") e.innerHTML = v;
    else if (k.startsWith("on")) e.addEventListener(k.slice(2), v); else if (v !== false && v != null) e.setAttribute(k, v === true ? "" : v);
  }
  for (const k of kids.flat()) if (k != null) e.append(k.nodeType ? k : document.createTextNode(k));
  return e;
};
const fmt = (x, sig = 4) => {
  if (x == null || x === "") return "–";
  if (typeof x === "boolean") return x ? "yes" : "no";
  if (typeof x !== "number") return String(x);
  if (!isFinite(x)) return "–";
  if (Number.isInteger(x) && Math.abs(x) < 1e6) return String(x);
  const a = Math.abs(x);
  if (a >= 1e5 || (a > 0 && a < 1e-3)) return x.toExponential(2);
  return String(+x.toPrecision(sig));
};
const mmss = (t) => `${Math.floor(t / 60)}:${(t % 60).toFixed(1).padStart(4, "0")}`;
const R = () => S.data[S.car].result;
const SUB = () => R().subsystems || {};
const lap = (ev = S.event) => (R().laps[ev] && !R().laps[ev].error ? R().laps[ev] : null);
const pretty = (k) => k.replace(/_/g, " ");

// ------------------------------------------------------------------ pages
const PAGES = [
  { id: "about", label: "Home", intro: "Orientation: what this site is, what each section does and how to use it." },
  { id: "overview", label: "Full car simulation", groups: ["events"], lap: true,
    intro: "The whole car on all four events: times and points, where it is limited around the lap, and what the model is assuming. The driver chosen under Driver is the one in the car." },
  { id: "drivers", label: "Driver", intro: "Who is in the car. A driver's mass, seating and driving numbers go into the full car simulation." },
  { id: "lap", label: "Lap explorer", lap: true, intro: "Every channel the simulation produces, along any event. Pick channels to stack them on a shared axis." },
  { id: "powertrain", label: "Powertrain", groups: ["powertrain"], lap: true,
    intro: (c) => c === "ic" ? "Restricted engine, six-speed gearbox, chain drive and differential." : "Motor, inverter, reduction, regen and the limits that cap drive torque at each speed." },
  { id: "energy", label: (c) => c === "ev" ? "Accumulator" : "Cooling & fuel", groups: (c) => c === "ev" ? ["accumulator"] : ["cooling"],
    intro: (c) => c === "ev" ? "State of charge, pack voltage, current and temperatures carried lap by lap through endurance." : "Coolant and oil temperature, fan duty and fuel carried lap by lap through endurance." },
  { id: "tires", label: "Tires & grip", groups: ["tires"], lap: true, intro: "The grip envelope the lap solver drives on, and the wheel loads behind it." },
  { id: "suspension", label: "Suspension", groups: ["suspension"], lap: true, intro: "Kinematics solved from the hardpoints, rates and roll stiffness, and body attitude around the lap." },
  { id: "aero", label: "Aero", groups: ["aero"], intro: "Downforce, drag and balance from the component aero model at the aero-loaded ride height." },
  { id: "brakes", label: "Brakes", groups: ["brakes"], lap: true, intro: "Brake balance against the ideal split, and brake force and power around the lap." },
  { id: "chassis", label: "Chassis & frame", groups: ["chassis"], intro: "Mass, geometry and the spaceframe tube model." },
  { id: "studies", label: "Studies", intro: "Sweep any input and see what it does to each event. Studies run in your browser." },
  { id: "inputs", label: "All inputs", intro: "Every input of the model. Values are SI unless you type a unit, e.g. 350 lbf/in, 11 in, 80 kW." },
];
const val = (v) => (typeof v === "function" ? v(S.car) : v);

// ------------------------------------------------------------------ charts
function plot(node, traces, layout = {}) {
  const axis = (a = {}) => Object.assign({ gridcolor: "#eef1f6", linecolor: "#dfe4ee", zerolinecolor: "#dfe4ee", ticks: "outside", tickcolor: "#dfe4ee",
    automargin: true, title: { text: "", standoff: 6, font: { size: 12 } } }, a, a.title && typeof a.title === "string" ? { title: { text: a.title, standoff: 6, font: { size: 12 } } } : {});
  const L = Object.assign({
    margin: { l: 50, r: 10, t: 6, b: 40 }, font: { family: getComputedStyle(document.body).fontFamily, size: 12, color: "#47526a" },
    paper_bgcolor: "#fff", plot_bgcolor: "#fff", colorway: PAL, hovermode: "x unified",
    showlegend: traces.filter((t) => t.showlegend !== false).length > 1,
    legend: { orientation: "h", y: 1.02, yanchor: "bottom", x: 0, font: { size: 11.5 } }, hoverlabel: { font: { size: 12 } },
  }, layout);
  for (const k of Object.keys(L)) if (/^[xy]axis\d*$/.test(k)) L[k] = axis(L[k]);
  if (!L.xaxis) L.xaxis = axis(); if (!L.yaxis) L.yaxis = axis();
  if (L.showlegend) L.margin = Object.assign({}, L.margin, { t: traces.length > 4 || window.innerWidth < 600 ? 50 : 28 });
  return Plotly.react(node, traces, L, { responsive: true, displaylogo: false, displayModeBar: false, scrollZoom: false });
}
const line = (x, y, name, extra = {}) => Object.assign({ x, y, name, mode: "lines", line: { width: 2 }, hovertemplate: "%{y:.4~g}<extra>" + name + "</extra>" }, extra);
const plotDiv = (cls = "") => el("div", { class: "plot " + cls });

function table(t, { fmtCell, maxRows } = {}) {
  if (!t) return el("div", { class: "muted" }, "No data.");
  if (t.error) return el("div", { class: "err" }, t.error);
  const rows = maxRows ? t.rows.slice(0, maxRows) : t.rows;
  return el("div", { class: "tablewrap" }, el("table", {},
    el("thead", {}, el("tr", {}, t.columns.map((c) => el("th", {}, pretty(c))))),
    el("tbody", {}, rows.map((r) => el("tr", {}, r.map((v, i) => el("td", {}, fmtCell ? fmtCell(v, t.columns[i]) : fmt(v))))))));
}
const kvTable = (obj) => table({ columns: ["quantity", "value"], rows: Object.entries(obj || {}).filter(([, v]) => v != null)
  .map(([k, v]) => [k, Array.isArray(v) ? v.map((x) => fmt(x)).join(", ") : typeof v === "object" ? Object.entries(v).map(([a, c]) => `${pretty(a)} ${fmt(c)}`).join(" · ") : v]) });

/* Channels along the lap as small multiples on one shared x axis. groups = [["v"], ["Fz_fo","Fz_fi"], ...]:
   channels in the same inner list share a panel (same unit). */
function lapStack(node, groups, { ev = S.event, xKey = "s", scale = {} } = {}) {
  const L = lap(ev);
  if (!L) return node.replaceWith(el("div", { class: "muted" }, `No ${ev} lap in this run.`));
  const c = L.channels;
  groups = groups.map((g) => g.filter((k) => Array.isArray(c[k]) && typeof c[k][0] !== "string")).filter((g) => g.length);
  if (!groups.length) return node.replaceWith(el("div", { class: "muted" }, "These channels are not produced for this car."));
  const n = groups.length, traces = [], layout = { grid: { rows: n, columns: 1, pattern: "independent" }, showlegend: groups.some((g) => g.length > 1) };
  const gap = 0.06, h = (1 - gap * (n - 1)) / n;
  groups.forEach((g, i) => {
    const ax = i === 0 ? "" : String(i + 1);
    g.forEach((k, j) => traces.push(line(c[xKey], scale[k] ? c[k].map((v) => v * scale[k]) : c[k], pretty(k), {
      xaxis: "x" + ax, yaxis: "y" + ax, line: { width: 1.8, color: g.length > 1 ? PAL[j % PAL.length] : PAL[0] }, showlegend: g.length > 1 })));
    const top = 1 - i * (h + gap);
    layout["yaxis" + ax] = { domain: [Math.max(0, top - h), top], title: g.length === 1 ? pretty(g[0]) : "" };
    layout["xaxis" + ax] = { anchor: "y" + ax, matches: i ? "x" : undefined, showticklabels: i === n - 1, title: i === n - 1 ? (xKey === "s" ? "distance [m]" : "time [s]") : "" };
  });
  node.style.height = Math.max(250, 60 + n * (window.innerWidth < 600 ? 130 : 150)) + "px";
  plot(node, traces, layout);
}

function trackMap(node, ev, colorKey) {
  const L = lap(ev);
  if (!L || !L.x) return node.replaceWith(el("div", { class: "muted" }, "No track geometry for this event."));
  const c = L.channels, traces = [];
  if (colorKey === "limit") {
    for (const name of R().limits) {
      const idx = c.limit.map((v, i) => (v === name ? i : -1)).filter((i) => i >= 0);
      if (idx.length) traces.push({ x: idx.map((i) => L.x[i]), y: idx.map((i) => L.y[i]), mode: "markers", name: pretty(name),
        marker: { size: 5, color: R().limit_colors[name] || "#8a9099" }, hovertemplate: pretty(name) + "<extra></extra>" });
    }
  } else {
    const z = colorKey === "v" ? c.v.map((v) => v * 3.6) : c[colorKey];
    traces.push({ x: L.x, y: L.y, mode: "markers", showlegend: false, marker: { size: 5, color: z, colorscale: SEQ,
      colorbar: { thickness: 10, len: 0.8, title: { text: colorKey === "v" ? "km/h" : pretty(colorKey), side: "right", font: { size: 11 } } } },
      text: z.map((v) => fmt(v)), hovertemplate: "%{text}<extra></extra>" });
  }
  plot(node, traces, { hovermode: "closest", xaxis: { visible: false }, yaxis: { visible: false, scaleanchor: "x" }, margin: { l: 6, r: 6, t: 6, b: 6 } });
}

function limitShare(ev) {
  const L = lap(ev); if (!L) return [];
  const t = L.channels.t, lim = L.channels.limit, acc = {};
  for (let i = 0; i < t.length - 1; i++) acc[lim[i]] = (acc[lim[i]] || 0) + (t[i + 1] - t[i]);
  const tot = Object.values(acc).reduce((a, b) => a + b, 0) || 1;
  return R().limits.filter((k) => acc[k]).map((k) => ({ k, pct: 100 * acc[k] / tot }));
}

// ------------------------------------------------------------------ visuals
const VISUALS = [];
const V = (page, id, title, sub, draw, opt = {}) => VISUALS.push(Object.assign({ page, id, title, sub, draw }, opt));
const need = (d) => { if (!d) throw new Error("not available in this run"); if (d.error) throw new Error(d.error); return d; };

// ---- overview
V("overview", "kpi", "Event results", "Times and points for the inputs on screen.", (b) => {
  const r = R(), base = S.base[S.car], changed = Object.keys(S.applied[S.car]).length > 0, tiles = [];
  for (const [e, d] of Object.entries(r.events)) {
    if (d.time_s == null && d.points == null) continue;
    const bt = base.events[e] && base.events[e].time_s, dlt = changed && bt != null && d.time_s != null ? d.time_s - bt : null;
    tiles.push(el("div", { class: "kpi" }, el("div", { class: "k" }, EVENT_NAMES[e] || e),
      el("div", { class: "v" }, d.time_s != null ? (e === "endurance" ? mmss(d.time_s) : d.time_s.toFixed(3)) : fmt(d.points, 3), el("small", {}, d.time_s != null ? (e === "endurance" ? "min" : "s") : "pts")),
      el("div", { class: "d" }, d.points != null && d.time_s != null ? `${d.points.toFixed(1)} pts` : d.time_s != null ? "points need a best time" : ""),
      dlt != null ? el("div", { class: "d " + (dlt > 0.0005 ? "up" : dlt < -0.0005 ? "down" : "") }, `${dlt > 0 ? "+" : ""}${dlt.toFixed(3)} s vs baseline`) : null));
  }
  tiles.push(el("div", { class: "kpi" }, el("div", { class: "k" }, "Mass with driver"), el("div", { class: "v" }, fmt(r.mass_kg, 4), el("small", {}, "kg"))));
  if (r.endurance) {
    const col = (n) => r.endurance.columns.indexOf(n), sum = (n) => r.endurance.rows.reduce((a, row) => a + (row[col(n)] || 0), 0);
    if (S.car === "ev") tiles.push(el("div", { class: "kpi" }, el("div", { class: "k" }, "Endurance energy"), el("div", { class: "v" }, sum("energy_kWh").toFixed(2), el("small", {}, "kWh"))));
    else tiles.push(el("div", { class: "kpi" }, el("div", { class: "k" }, "Endurance fuel"), el("div", { class: "v" }, sum("fuel_L").toFixed(2), el("small", {}, "L"))));
  }
  b.append(el("div", { class: "kpis" }, tiles));
}, { wide: true });
V("overview", "map", "Track map", "", (b) => {
  const sel = el("select", { onchange: () => trackMap(p, S.event, sel.value) }, el("option", { value: "v" }, "Colour: speed"), el("option", { value: "limit" }, "Colour: what limits the car"),
    el("option", { value: "ax" }, "Colour: longitudinal accel"), el("option", { value: "ay" }, "Colour: lateral accel"));
  const p = plotDiv("tall"); b.append(el("div", { class: "ctl" }, sel), p); trackMap(p, S.event, "v");
});
V("overview", "speed", "Speed", "Along the selected event.", (b) => {
  const L = need(lap()), p = plotDiv("tall"); b.append(p);
  plot(p, [line(L.channels.s, L.channels.v.map((v) => v * 3.6), "speed", { fill: "tozeroy", fillcolor: "rgba(42,120,214,.08)" })], { xaxis: { title: "distance [m]" }, yaxis: { title: "speed [km/h]" } });
});
V("overview", "limits", "What limits the car", "Share of lap time spent at each limit.", (b) => {
  const sh = limitShare(S.event), p = plotDiv("short"); b.append(p);
  plot(p, [{ type: "bar", orientation: "h", y: sh.map((s) => pretty(s.k)), x: sh.map((s) => s.pct), marker: { color: sh.map((s) => R().limit_colors[s.k] || "#8a9099") },
    text: sh.map((s) => s.pct.toFixed(1) + " %"), textposition: "auto", hovertemplate: "%{y}: %{x:.1f} %<extra></extra>" }],
  { hovermode: "closest", xaxis: { title: "% of lap time" }, yaxis: { autorange: "reversed" }, margin: { l: 90, r: 10, t: 6, b: 40 } });
});
V("overview", "notes", "Model notes", "Assumptions and warnings raised by this run.", (b) => {
  const r = R(), s = SUB();
  if (s.status_counts) b.append(el("p", {}, el("span", { class: "tag ph" }, `${s.status_counts.placeholder} placeholder`), " ", el("span", { class: "tag ok" }, `${s.status_counts.confirmed} confirmed`), " ",
    el("span", { class: "tag miss" }, `${s.status_counts.missing} missing`), " EV inputs. Results are those of a generic single-motor FSAE EV until the real values are entered."));
  b.append(el("ul", { class: "prose" }, r.notes.length ? r.notes.map((n) => el("li", {}, n)) : el("li", {}, "No warnings.")));
});

// ---- lap explorer
V("lap", "explorer", "Channels", "Tap channels to add or remove them. Channels are stacked, one axis each.", (b) => {
  const L = need(lap()), X = S.explorer, names = Object.keys(L.channels).filter((k) => !["s", "t", "limit"].includes(k) && typeof L.channels[k][0] !== "string");
  X.channels = X.channels.filter((k) => names.includes(k)); if (!X.channels.length) X.channels = ["v"];
  const p = plotDiv(), draw = () => lapStack(p, X.channels.map((k) => [k]), { xKey: X.x });
  const xs = el("select", { onchange: () => { X.x = xs.value; draw(); } }, el("option", { value: "s" }, "Against distance"), el("option", { value: "t" }, "Against time"));
  xs.value = X.x;
  const filt = el("input", { type: "search", placeholder: `Filter ${names.length} channels…`, oninput: () => { for (const c of box.children) c.hidden = !c.textContent.includes(filt.value.trim()); } });
  const box = el("div", { class: "channels" }, names.map((k) => el("button", { class: "chip" + (X.channels.includes(k) ? " on" : ""), onclick: (e) => {
    X.channels = X.channels.includes(k) ? X.channels.filter((c) => c !== k) : [...X.channels, k];
    e.currentTarget.classList.toggle("on"); if (X.channels.length) draw(); } }, k)));
  b.append(el("div", { class: "ctl" }, xs, filt), box, p); draw();
}, { wide: true });
V("lap", "lapmap", "Track map by channel", "Colour the track by any channel.", (b) => {
  const L = need(lap()), names = ["limit", ...Object.keys(L.channels).filter((k) => !["s", "t", "limit"].includes(k) && typeof L.channels[k][0] !== "string")];
  const p = plotDiv("tall"), sel = el("select", { onchange: () => { S.explorer.color = sel.value; trackMap(p, S.event, sel.value); } }, names.map((k) => el("option", { value: k }, k)));
  sel.value = names.includes(S.explorer.color) ? S.explorer.color : "v";
  b.append(el("div", { class: "ctl" }, "Colour by", sel), p); trackMap(p, S.event, sel.value);
});
V("lap", "lapsummary", "Lap summary", "Totals for the selected event.", (b) => b.append(kvTable(need(lap()).summary)));

// ---- powertrain (IC)
V("powertrain", "eng_torque", "Engine torque", "Wide-open throttle, restricted.", (b) => {
  const e = need(SUB().powertrain).engine, p = plotDiv(); b.append(p);
  plot(p, [line(e.rpm, e.torque, "torque")], { xaxis: { title: "engine speed [rpm]" }, yaxis: { title: "torque [N·m]", rangemode: "tozero" } });
}, { car: "ic" });
V("powertrain", "eng_power", "Engine power", "Wide-open throttle, restricted.", (b) => {
  const e = need(SUB().powertrain).engine, p = plotDiv(); b.append(p);
  plot(p, [line(e.rpm, e.power.map((w) => w / 1e3), "power", { line: { width: 2, color: PAL[1] } })], { xaxis: { title: "engine speed [rpm]" }, yaxis: { title: "power [kW]", rangemode: "tozero" } });
}, { car: "ic" });
V("powertrain", "tractive", "Tractive force by gear", "Wheel force at wide-open throttle against total resistance.", (b) => {
  const t = need(SUB().powertrain).traction, v = t.v.map((x) => x * 3.6), p = plotDiv("tall"); b.append(p);
  const tr = t.force_by_gear.map((f, i) => line(v, f.map((x) => (x > 0 ? x : null)), `gear ${i + 1}`, { line: { width: 2, color: SEQ_STEP(i, t.force_by_gear.length) } }));
  tr.push(line(v, t.resistance, "resistance", { line: { width: 2, color: "#8a9099", dash: "dot" } }));
  plot(p, tr, { xaxis: { title: "speed [km/h]" }, yaxis: { title: "force at the wheels [N]", rangemode: "tozero" } });
}, { car: "ic" });
V("powertrain", "gears", "Gearbox", "Ratios, speed at the rev limit and rotating mass per gear.", (b) => {
  const g = need(SUB().powertrain).gears, cols = Object.keys(g[0]);
  b.append(table({ columns: cols, rows: g.map((r) => cols.map((c) => r[c])) }), el("div", { class: "sub" }, "Engine: " + Object.entries(SUB().powertrain.engine_summary).filter(([, v]) => v != null).map(([k, v]) => `${pretty(k)} ${fmt(v)}`).join(" · ")));
}, { car: "ic" });
V("powertrain", "pt_lap_ic", "Engine around the lap", "", (b) => { const p = plotDiv(); b.append(p); lapStack(p, [["rpm"], ["gear"], ["throttle"], ["P_engine_kW"]]); }, { car: "ic", wide: true });
V("powertrain", "loss_ic", "Driveline losses", "Where power is lost between crank and tire.", (b) => { const p = plotDiv(); b.append(p);
  lapStack(p, [["loss_primary_W", "loss_gearbox_W", "loss_chain_W", "loss_differential_W", "loss_cv_joints_W", "loss_hubs_W"]]); }, { car: "ic", wide: true, off: true });
V("powertrain", "diff_ic", "Differential and halfshafts", "", (b) => { const p = plotDiv(); b.append(p);
  lapStack(p, [["T_halfshaft_inner", "T_halfshaft_outer"], ["diff_Mz"], ["chain_tension"]]); }, { wide: true, off: true });

// ---- powertrain (EV)
V("powertrain", "ev_torque", "Motor torque envelope", "Largest drive and regen torque inside every limit.", (b) => {
  const e = need(SUB().powertrain).envelope, p = plotDiv(); b.append(p);
  plot(p, [line(e.rpm, e.torque, "drive"), line(e.rpm, e.regen_torque, "regen", { line: { width: 2, color: PAL[2] } })], { xaxis: { title: "motor speed [rpm]" }, yaxis: { title: "torque [N·m]", rangemode: "tozero" } });
}, { car: "ev" });
V("powertrain", "ev_power", "Accumulator power", "Power at the accumulator terminals at the drive limit.", (b) => {
  const pt = need(SUB().powertrain), e = pt.envelope, v = e.v.map((x) => x * 3.6), p = plotDiv(); b.append(p);
  plot(p, [line(v, e.P_acc_kW, "drive"), line(v, e.P_regen_kW.map(Math.abs), "regen", { line: { width: 2, color: PAL[2] } })],
    { xaxis: { title: "speed [km/h]" }, yaxis: { title: "power [kW]", rangemode: "tozero" },
      shapes: [{ type: "line", xref: "paper", x0: 0, x1: 1, y0: pt.power_limit_kW, y1: pt.power_limit_kW, line: { color: "#8a9099", dash: "dot", width: 1.5 } }],
      annotations: [{ xref: "paper", x: 1, y: pt.power_limit_kW, text: `limit ${fmt(pt.power_limit_kW)} kW`, showarrow: false, yshift: 9, xanchor: "right", font: { size: 11 } }] });
}, { car: "ev" });
V("powertrain", "ev_force", "Tractive force", "Drive force at the wheels against total resistance.", (b) => {
  const pt = need(SUB().powertrain), e = pt.envelope, v = e.v.map((x) => x * 3.6), p = plotDiv(); b.append(p);
  plot(p, [line(v, e.force, "drive"), line(v, e.resistance, "resistance", { line: { width: 2, color: "#8a9099", dash: "dot" } })], { xaxis: { title: "speed [km/h]" }, yaxis: { title: "force at the wheels [N]", rangemode: "tozero" } });
  b.append(el("div", { class: "sub" }, `Overall ratio ${fmt(pt.overall_ratio)} · top speed ${fmt(e.v_top * 3.6, 3)} km/h · pack ${fmt(pt.pack_energy_kWh, 3)} kWh`));
}, { car: "ev" });
V("powertrain", "ev_eff", "Motor and inverter efficiency", "Shaft power over dc power, driving.", (b) => {
  const m = need(SUB().powertrain).efficiency_map, p = plotDiv(); b.append(p);
  plot(p, [{ type: "heatmap", x: m.rpm, y: m.torque, z: m.eta, zmin: 70, zmax: 100, colorscale: SEQ, colorbar: { thickness: 10, title: { text: "%", font: { size: 11 } } }, hovertemplate: "%{x:.0f} rpm, %{y:.0f} N·m: %{z:.1f} %<extra></extra>" }],
    { hovermode: "closest", xaxis: { title: "motor speed [rpm]" }, yaxis: { title: "torque [N·m]" } });
}, { car: "ev" });
V("powertrain", "ev_energy", "Where the energy goes", "Energy drawn from the cells over one lap.", (b) => {
  const ev = S.event === "endurance" ? "endurance" : "autocross", d = need(SUB()["energy_" + ev]), ks = Object.keys(d), p = plotDiv(); b.append(p);
  plot(p, [{ type: "bar", orientation: "h", y: ks, x: ks.map((k) => d[k] * 1e3), marker: { color: PAL[0] }, text: ks.map((k) => (d[k] * 1e3).toFixed(0)), textposition: "auto", hovertemplate: "%{y}: %{x:.1f} Wh<extra></extra>" }],
    { hovermode: "closest", xaxis: { title: `Wh per ${ev} lap` }, yaxis: { autorange: "reversed" }, margin: { l: 150, r: 10, t: 6, b: 40 } });
}, { car: "ev" });
V("powertrain", "pt_lap_ev", "Motor around the lap", "", (b) => { const p = plotDiv(); b.append(p); lapStack(p, [["motor_rpm"], ["motor_torque", "motor_torque_max"], ["P_acc_kW", "P_regen_kW"], ["motor_efficiency"]]); }, { car: "ev", wide: true });
V("powertrain", "loss_ev", "Losses around the lap", "", (b) => { const p = plotDiv(); b.append(p);
  lapStack(p, [["loss_copper_W", "loss_iron_W", "loss_inverter_W", "loss_driveline_W", "loss_pack_W"]]); }, { car: "ev", wide: true, off: true });

// ---- energy: endurance
const endCol = (n) => { const e = need(R().endurance), i = e.columns.indexOf(n); return i < 0 ? null : e.rows.map((r) => r[i]); };
const endTrace = (b, keys, ytitle, scale = 1) => {
  const t = need(R().endurance_trace), p = plotDiv(); b.append(p);
  plot(p, keys.filter((k) => t[k]).map((k, i) => line(t.t_elapsed.map((x) => x / 60), scale === 1 ? t[k] : t[k].map((v) => v * scale), pretty(k), { line: { width: 1.8, color: PAL[i] } })),
    { xaxis: { title: "endurance time [min]" }, yaxis: { title: ytitle } });
};
V("energy", "soc", "State of charge", "Whole endurance, driver change at half distance.", (b) => endTrace(b, ["soc"], "SOC [%]", 100), { car: "ev" });
V("energy", "vpack", "Pack voltage", "", (b) => endTrace(b, ["V_pack"], "pack voltage [V]"), { car: "ev" });
V("energy", "ipack", "Pack current", "Positive is discharge.", (b) => endTrace(b, ["I_pack"], "pack current [A]"), { car: "ev" });
V("energy", "temps_ev", "Cell and motor temperature", "", (b) => endTrace(b, ["cell_temp_C", "motor_temp_C"], "temperature [°C]"), { car: "ev" });
V("energy", "temps_ic", "Coolant and oil temperature", "Whole endurance, engine off at the driver change.", (b) => endTrace(b, ["coolant_C", "oil_C"], "temperature [°C]"), { car: "ic" });
V("energy", "fuel", "Fuel remaining", "At the end of each lap.", (b) => { const p = plotDiv(); b.append(p);
  plot(p, [line(endCol("lap"), endCol("fuel_left_L"), "fuel", { mode: "lines+markers", marker: { size: 7 } })], { xaxis: { title: "lap", dtick: 2 }, yaxis: { title: "fuel left [L]", rangemode: "tozero" } }); }, { car: "ic" });
V("energy", "elap", "Energy per lap", "", (b) => { const p = plotDiv(); b.append(p);
  plot(p, [{ type: "bar", x: endCol("lap"), y: endCol("energy_kWh").map((v) => v * 1e3), marker: { color: PAL[0] }, hovertemplate: "lap %{x}: %{y:.0f} Wh<extra></extra>" }], { hovermode: "closest", xaxis: { title: "lap", dtick: 2 }, yaxis: { title: "energy [Wh]" }, bargap: 0.25 }); }, { car: "ev" });
V("energy", "laptimes", "Lap time by lap", "", (b) => { const p = plotDiv(); b.append(p);
  plot(p, [line(endCol("lap"), endCol("lap_time_s"), "lap time", { mode: "lines+markers", marker: { size: 7 } })], { xaxis: { title: "lap", dtick: 2 }, yaxis: { title: "lap time [s]" } }); });
V("energy", "endtable", "Endurance, lap by lap", "", (b) => b.append(table(need(R().endurance))), { wide: true });
V("energy", "status", "EV input status", "Which accumulator and powertrain inputs are still placeholders.", (b) => b.append(table(need(SUB().status), {
  fmtCell: (v, c) => c === "status" ? el("span", { class: "tag " + (v === "confirmed" ? "ok" : v === "MISSING" ? "miss" : "ph") }, String(v).toLowerCase()) : fmt(v) })), { car: "ev", wide: true, off: true });

// ---- tires
const SEQ_STEP = (i, n) => ["#a9c8f0", "#7fadea", "#5b93dc", "#3a78cc", "#2259a8", "#0B2152"][Math.round(i * 5 / Math.max(1, n - 1))];
V("tires", "gg", "g-g envelope", "Grip limit at several speeds, with the lap's operating points.", (b) => {
  const g = need(SUB().ggv), L = lap(), tr = [], p = plotDiv("tall"); b.append(p);
  if (L) tr.push({ x: L.channels.ay.map((v) => v / 9.81), y: L.channels.ax.map((v) => v / 9.81), mode: "markers", name: "lap", marker: { size: 3.5, color: "#b9c0cc" }, hoverinfo: "skip" });
  g.speeds.forEach((s, i) => {
    const x = [...s.ay.map((a) => -a).reverse(), ...s.ay, ...s.ay.slice().reverse(), ...s.ay.map((a) => -a)];
    const y = [...s.ax_acc.slice().reverse(), ...s.ax_acc, ...s.ax_brk.slice().reverse().map((a) => -a), ...s.ax_brk.map((a) => -a)];
    tr.push(line(x, y, `${(s.v * 3.6).toFixed(0)} km/h`, { line: { width: 2, color: SEQ_STEP(i, g.speeds.length) }, hovertemplate: "ay %{x:.2f} g, ax %{y:.2f} g<extra>%{fullData.name}</extra>" }));
  });
  plot(p, tr, { hovermode: "closest", xaxis: { title: "lateral [g]" }, yaxis: { title: "longitudinal [g]", scaleanchor: "x" } });
});
V("tires", "aymax", "Lateral limit against speed", "Aero load raises it; load sensitivity lowers it.", (b) => {
  const g = need(SUB().ggv), p = plotDiv("tall"); b.append(p);
  plot(p, [line(g.v.map((v) => v * 3.6), g.ay_max_g, "lateral limit")], { xaxis: { title: "speed [km/h]" }, yaxis: { title: "lateral limit [g]" } });
});
V("tires", "fz", "Wheel loads", "Outer and inner, front and rear.", (b) => { const p = plotDiv(); b.append(p); lapStack(p, [["Fz_fo", "Fz_fi", "Fz_ro", "Fz_ri"], ["ax", "ay"]]); }, { wide: true });
V("tires", "slip", "Rear tire slip and force", "", (b) => { const p = plotDiv(); b.append(p); lapStack(p, [["slip_inner", "slip_outer"], ["F_rear_inner", "F_rear_outer"]]); }, { wide: true, off: true });

// ---- suspension
const kin = (b, key, ytitle) => {
  const k = need(SUB().suspension).kinematics, p = plotDiv(); b.append(p);
  plot(p, ["front", "rear"].filter((a) => k[a] && !k[a].error).map((a, i) => line(k[a].travel_mm, k[a][key], a, { line: { width: 2, color: PAL[i] } })), { xaxis: { title: "wheel travel [mm], bump +" }, yaxis: { title: ytitle } });
};
V("suspension", "sus_axles", "Front and rear axle", "Kinematics, rates and roll stiffness.", (b) => b.append(table(need(SUB().suspension).axles)));
V("suspension", "sus_car", "Whole car", "Roll gradient, load-transfer split, anti geometry and limit balance.", (b) => b.append(kvTable(need(SUB().suspension).car)));
V("suspension", "camber", "Camber gain", "From the hardpoints.", (b) => kin(b, "camber_deg", "camber [deg]"));
V("suspension", "rc", "Roll-centre height", "From the hardpoints.", (b) => kin(b, "rc_height_mm", "roll-centre height [mm]"));
V("suspension", "toe", "Bump steer", "Toe against travel.", (b) => kin(b, "toe_deg", "toe [deg]"), { off: true });
V("suspension", "damperlen", "Damper length", "The slope is the motion ratio.", (b) => kin(b, "damper_len_mm", "damper length [mm]"), { off: true });
V("suspension", "attitude", "Body attitude and dampers", "", (b) => { const p = plotDiv(); b.append(p);
  lapStack(p, [["roll_deg", "pitch_deg"], ["damper_fo", "damper_fi", "damper_ro", "damper_ri"], ["camber_fo", "camber_fi", "camber_ro", "camber_ri"]]); }, { wide: true });

// ---- aero
const aeroPlot = (b, keys, ytitle, names) => { const a = need(SUB().aero), p = plotDiv(); b.append(p);
  plot(p, keys.map((k, i) => line(a.v.map((v) => v * 3.6), a[k], names ? names[i] : pretty(k), { line: { width: 2, color: PAL[i] } })), { xaxis: { title: "speed [km/h]" }, yaxis: { title: ytitle } }); };
V("aero", "downforce", "Downforce", "On each axle, straight line.", (b) => aeroPlot(b, ["downforce_front_N", "downforce_rear_N"], "downforce [N]", ["front axle", "rear axle"]));
V("aero", "drag", "Drag", "", (b) => aeroPlot(b, ["drag_N"], "drag [N]"));
V("aero", "balance", "Aero balance", "Share of the aero load on the front axle.", (b) => aeroPlot(b, ["front_share_%"], "front share [%]"));
V("aero", "dragpower", "Drag power", "", (b) => aeroPlot(b, ["drag_power_kW"], "power [kW]"), { off: true });
V("aero", "aerocomp", "Aero components", "Front wing, rear wing and body.", (b) => b.append(table(need(SUB().aero).components)), { wide: true });

// ---- brakes
V("brakes", "balance", "Brake balance", "Front share of brake force against the ideal split.", (b) => {
  const t = need(need(SUB().brakes).balance), c = (n) => t.rows.map((r) => r[t.columns.indexOf(n)]), p = plotDiv(); b.append(p);
  plot(p, [line(c("decel_g"), c("front_share").map((v) => v * 100), "as built"), line(c("decel_g"), c("ideal_front_share").map((v) => v * 100), "ideal", { line: { width: 2, color: PAL[1], dash: "dash" } })],
    { xaxis: { title: "deceleration [g]" }, yaxis: { title: "front share [%]" } });
});
V("brakes", "brk_table", "Balance table", "Line pressure and which axle locks first.", (b) => b.append(table(need(SUB().brakes).balance)));
V("brakes", "brk_lap", "Braking around the lap", "", (b) => { const p = plotDiv(); b.append(p); lapStack(p, [["ax"], ["F_brake_front", "F_brake_rear", "F_regen"], ["P_brake_front_kW", "P_brake_rear_kW", "P_regen_kW"]]); }, { wide: true });
V("brakes", "brk_temp", "Rotor temperature", "Front and rear rotor through the selected event, from the brake power the lap needs. Endurance repeats the lap for every lap of the event.", (b) => {
  const ev = S.event === "endurance" ? "endurance" : "autocross", d = need(SUB()["brake_temps_" + ev]), p = plotDiv("tall");
  b.append(el("div", { class: "kpis" },
    el("div", { class: "kpi" }, el("div", { class: "k" }, "Peak front rotor"), el("div", { class: "v" }, fmt(d.peak_front_C, 3), el("small", {}, "°C"))),
    el("div", { class: "kpi" }, el("div", { class: "k" }, "Peak rear rotor"), el("div", { class: "v" }, fmt(d.peak_rear_C, 3), el("small", {}, "°C"))),
    el("div", { class: "kpi" }, el("div", { class: "k" }, "Heat per lap, one front brake"), el("div", { class: "v" }, fmt(d.energy_front_kJ_per_lap, 3), el("small", {}, "kJ"))),
    el("div", { class: "kpi" }, el("div", { class: "k" }, "Peak power, one front brake"), el("div", { class: "v" }, fmt(d.peak_power_front_kW, 3), el("small", {}, "kW")))), p);
  plot(p, [line(d.t.map((x) => x / 60), d.T_front_C, "front rotor"), line(d.t.map((x) => x / 60), d.T_rear_C, "rear rotor", { line: { width: 2, color: PAL[1] } })],
    { xaxis: { title: `${ev} time [min]` }, yaxis: { title: "rotor temperature [°C]" } });
  b.append(el("div", { class: "sub" }, `${ev === "endurance" ? d.laps + " laps, no driver-change cooling. " : "One lap from ambient. "}The car brakes at the tire limit everywhere, so this is the upper bound for a driver.`
    + (d.rear_rotor_assumed_equal_to_front ? " Rear rotor mass and area are not in the config: taken equal to the front." : "")));
}, { wide: true });
V("brakes", "brk_stops", "Repeated stops", "A fade and sizing check: brake from one speed to another, accelerate back, repeat. Peaks level off where cooling per cycle equals heating.", (b) => {
  const f = (label, v, attrs) => { const i = el("input", Object.assign({ type: "number", value: v, style: "width:78px" }, attrs)); return [el("label", {}, label, " ", i), i]; };
  const [l1, hi] = f("from [km/h]", 100, { min: 20, max: 160 }), [l2, lo] = f("to [km/h]", 30, { min: 0, max: 120 }), [l3, n] = f("stops", 15, { min: 2, max: 60 }), [l4, g] = f("braking [g]", 1.4, { min: 0.3, max: 2.5, step: 0.1 }), [l5, a] = f("accelerating [g]", 0.6, { min: 0.1, max: 1.5, step: 0.1 });
  const out = el("div"), go = el("button", { class: "btn navy", onclick: async () => {
    go.disabled = true; out.replaceChildren(el("div", { class: "muted" }, "Running…"));
    try {
      const d = await call("brake_stops", [S.car, S.overrides[S.car], +hi.value, +lo.value, +n.value, +g.value, +a.value]), p = plotDiv();
      out.replaceChildren(p, el("div", { class: "sub" }, `Each stop puts ${fmt(d.energy_per_stop_front_kJ, 3)} kJ into one front brake and ${fmt(d.energy_per_stop_rear_kJ, 3)} kJ into one rear. Peak front line pressure ${fmt(d.p_front_peak_bar, 3)} bar. Last-stop peaks: ${fmt(d.peaks_front_C.at(-1), 3)} °C front, ${fmt(d.peaks_rear_C.at(-1), 3)} °C rear.`));
      plot(p, [line(d.t, d.T_front_C, "front rotor"), line(d.t, d.T_rear_C, "rear rotor", { line: { width: 2, color: PAL[1] } })], { xaxis: { title: "time [s]" }, yaxis: { title: "rotor temperature [°C]" } });
    } catch (e) { out.replaceChildren(el("div", { class: "err" }, String(e.message || e))); }
    go.disabled = false; } }, "Run stops");
  b.append(el("div", { class: "ctl" }, l1, l2, l3, l4, l5, go), out);
}, { wide: true });
V("brakes", "brk_hw", "Corner hardware", "What the caliper, pads and rotor in the config give, next to the torque fitted from a log. Empty rows need a value under Quick adjust or Inputs.", (b) => {
  const h = need(SUB().brakes).hardware, N = { torque_per_bar_front_fit: "Front torque per bar, fitted from the log [N·m/bar]", torque_per_bar_rear_fit: "Rear torque per bar, fitted from the log [N·m/bar]",
    torque_per_bar_front_hardware: "Front torque per bar from the hardware [N·m/bar]", torque_per_bar_rear_hardware: "Rear torque per bar from the hardware [N·m/bar]",
    implied_pad_mu_front: "Pad friction that would match the fit, front", implied_pad_mu_rear: "Pad friction that would match the fit, rear", clamp_force_front_N: "Front clamp force at the reference pressure [N]",
    pad_pressure_front_MPa: "Front pad pressure [MPa]", pad_pressure_rear_MPa: "Rear pad pressure [MPa]", p_ref_bar: "Reference line pressure [bar]", force_fraction_front_at_p_ref: "Front share of brake force at the reference pressure" };
  b.append(table({ columns: ["quantity", "value"], rows: Object.entries(h).map(([k, v]) => [N[k] || pretty(k), v == null ? "needs hardware values" : v]) }));
}, { wide: true });

// ---- chassis
// ---- studies
V("studies", "sweep", "Sweep one input", "Runs acceleration, skidpad and autocross at each value.", (b) => {
  const inputs = S.data[S.car].schema.inputs.filter((i) => i.kind === "number" && i.value != null);
  const sel = el("select", {}, inputs.map((i) => el("option", { value: i.key }, i.key.split(":")[1] + (i.unit ? ` [${i.unit}]` : ""))));
  sel.value = "vehicle:mass.cg_height";
  const lo = el("input", { type: "number", step: "any", style: "width:96px" }), hi = el("input", { type: "number", step: "any", style: "width:96px" }), n = el("input", { type: "number", min: 2, max: 15, value: 5, style: "width:64px" });
  const fill = () => { const v = inputs.find((i) => i.key === sel.value).value; lo.value = +(v * 0.85).toPrecision(4); hi.value = +(v * 1.15).toPrecision(4); };
  sel.onchange = fill; fill();
  const out = el("div"), go = el("button", { class: "btn navy", onclick: async () => {
    const a = +lo.value, z = +hi.value, k = Math.max(2, Math.min(15, +n.value)), vals = Array.from({ length: k }, (_, i) => +(a + (z - a) * i / (k - 1)).toPrecision(6));
    go.disabled = true; out.replaceChildren(el("div", { class: "muted" }, `Running ${k} simulations…`));
    try {
      const r = await call("sweep", [S.car, sel.value, vals, ["acceleration", "skidpad", "autocross"], S.overrides[S.car]]);
      out.replaceChildren();
      const good = r.rows.filter((x) => !x.error), unit = inputs.find((i) => i.key === sel.value).unit;
      for (const e of ["acceleration", "skidpad", "autocross"]) {
        if (!good.some((x) => x[e + "_s"] != null)) continue;
        const p = plotDiv("short"); out.append(el("div", { class: "sub" }, EVENT_NAMES[e]), p);
        plot(p, [line(good.map((x) => x.value), good.map((x) => x[e + "_s"]), e, { mode: "lines+markers", marker: { size: 7 } })], { xaxis: { title: sel.value.split(":")[1] + (unit ? ` [${unit}]` : "") }, yaxis: { title: "time [s]" } });
      }
      const cols = ["value", ...Object.keys(good[0] || {}).filter((c) => c !== "value")];
      out.append(table({ columns: cols, rows: r.rows.map((x) => cols.map((c) => x[c] ?? x.error ?? null)) }));
    } catch (err) { out.replaceChildren(el("div", { class: "err" }, String(err.message || err))); }
    go.disabled = false;
  } }, "Run sweep");
  b.append(el("div", { class: "ctl" }, sel, "from", lo, "to", hi, "points", n, go), out);
}, { wide: true });
const studyChart = (b, key, xk, series, xt, rerun) => {
  const t = need(need(S.data.ev.studies)[key]), c = (n) => t.rows.map((r) => r[t.columns.indexOf(n)]);
  for (const [yk, yt] of series) { const p = plotDiv("short"); b.append(p);
    plot(p, [line(c(xk), c(yk), yt, { mode: "lines+markers", marker: { size: 7 } })], { xaxis: { title: xt }, yaxis: { title: yt } }); }
  b.append(table(t));
};
V("studies", "esweep", "Endurance against the power limit", "Baseline car. Which accumulator power limit finishes, and how fast.", (b) =>
  studyChart(b, "energy_sweep", "power_limit_kW", [["time_s", "endurance time [s]"], ["energy_kWh", "energy used [kWh]"], ["final_soc_%", "final SOC [%]"]], "accumulator power limit [kW]"), { car: "ev" });
V("studies", "gsweep", "Reduction ratio", "Baseline car. Acceleration, autocross and top speed against the overall ratio.", (b) =>
  studyChart(b, "gearing_sweep", "ratio", [["acceleration_s", "acceleration [s]"], ["autocross_s", "autocross [s]"], ["top_speed_kph", "top speed [km/h]"]], "overall reduction ratio"), { car: "ev" });

{ const B = { powertrain: ["eng_torque", "eng_power", "tractive", "gears", "ev_torque", "ev_power", "ev_force", "ev_eff"], ggv: ["gg", "aymax"],
    suspension: ["sus_axles", "sus_car", "camber", "rc", "toe", "damperlen"], aero: ["downforce", "drag", "balance", "dragpower", "aerocomp"], brakes: ["brk_hw", "balance", "brk_table", "brk_hw"] };
  for (const [b, ids] of Object.entries(B)) for (const v of VISUALS) if (ids.includes(v.id)) v.blocks = [b]; }

// ------------------------------------------------------------------ inputs panel
function inputsPanel(groups, open = false) {
  const car = S.car, all = S.data[car].schema.inputs.filter((i) => !groups || groups.includes(i.group));
  if (!all.length) return null;
  const bySec = new Map();
  for (const i of all) { const k = i.key.split(":")[0] + " · " + i.section; if (!bySec.has(k)) bySec.set(k, []); bySec.get(k).push(i); }
  const names = { vehicle: "car", pt: "powertrain", ev: "EV powertrain", pack: "accumulator" };
  const field = (i) => {
    const ov = S.overrides[car], leaf = i.key.split(":")[1].split(".").slice(1).join(".") || i.key.split(":")[1];
    const isList = i.kind === "list", show = (v) => (isList ? JSON.stringify(v).replace(/^\[|\]$/g, "").replace(/,/g, ", ") : v ?? "");
    const inp = el("input", { type: "text", inputmode: i.kind === "number" ? "decimal" : "text", value: show(i.key in ov ? ov[i.key] : i.value), placeholder: i.value == null ? "not set" : "", "aria-label": i.key,
      onchange: () => {
        const raw = inp.value.trim(), num = raw !== "" && !isNaN(raw) ? +raw : null;
        let v = i.kind === "bool" ? raw.toLowerCase() === "true" : num != null ? num : raw === "" ? null : raw;
        if (isList) { try { v = JSON.parse("[" + raw + "]"); } catch (e) { inp.value = show(i.key in ov ? ov[i.key] : i.value); return alert("A list is numbers separated by commas, e.g. 2.583, 2.0, 1.667"); } }
        if (JSON.stringify(v) === JSON.stringify(i.value) || (v == null && i.value == null)) delete ov[i.key]; else ov[i.key] = v;
        row.classList.toggle("changed", i.key in ov); pending();
      } });
    const row = el("div", { class: "field" + (isList ? " list" : "") + (i.key in ov ? " changed" : ""), "data-k": (i.key + " " + i.note).toLowerCase() },
      el("label", { title: [i.note, i.source && "source: " + i.source].filter(Boolean).join(" · ") }, el("span", { class: "n" }, leaf), el("span", { class: "h" }, i.note || i.source || "")),
      el("div", { class: "in" }, inp, el("span", { class: "u" }, i.unit || "")));
    return row;
  };
  const secs = [...bySec].map(([k, items]) => el("details", { class: "sec", open: bySec.size <= 2 || open },
    el("summary", {}, names[k.split(" · ")[0]] + " · " + k.split(" · ")[1] + ` (${items.length})`), el("div", { class: "fields" }, items.map(field))));
  const search = el("input", { type: "search", placeholder: `Search ${all.length} inputs…`, oninput: () => {
    const q = search.value.trim().toLowerCase();
    for (const s of secs) { let any = false; for (const f of s.querySelectorAll(".field")) { const hit = !q || f.dataset.k.includes(q); f.hidden = !hit; any = any || hit; } s.hidden = !any; if (q) s.open = true; }
  } });
  const tools = el("div", { class: "inputs-tools" }, search,
    el("button", { class: "btn small", onclick: () => { const a = el("a", { href: URL.createObjectURL(new Blob([JSON.stringify({ car, overrides: S.overrides[car] }, null, 1)], { type: "application/json" })), download: `lapsim_${car}_inputs.json` }); a.click(); } }, "Export changes"),
    el("label", { class: "btn small" }, "Import", el("input", { type: "file", accept: ".json", hidden: true, onchange: async (e) => {
      try { const j = JSON.parse(await e.target.files[0].text()); Object.assign(S.overrides[car], j.overrides || j); render(); pending(); } catch (err) { alert("Could not read that file: " + err.message); } } })));
  return el("details", { class: "inputs", open }, el("summary", {}, "Inputs", el("span", { class: "count" }, `${all.length} values · edit, then Run simulation`)), el("div", { class: "inputs-body" }, tools, secs));
}

// ------------------------------------------------------------------ live preview
/* A change to an input re-computes that page's subsystem data (not the lap) in the browser a moment later, so curves
   follow sliders and dragged points. Lap results only change with Run simulation. */
const CARDS = new Map();                 // visual id -> {v, body, ctx} of the cards on screen
const PAGE_BLOCKS = { powertrain: ["powertrain"], tires: ["ggv"], suspension: ["suspension"], aero: ["aero"], brakes: ["brakes"], chassis: ["frame"] };
S.live = (localStorage.getItem("lapsim.live") ?? (window.innerWidth >= 900 ? "1" : "0")) === "1";
let pvTimer = null, pvSeq = 0;
function redraw(blocks) {
  for (const { v, body, ctx } of CARDS.values()) {
    if (!v.blocks || !v.blocks.some((b) => blocks.includes(b))) continue;
    try { if (ctx.refresh) ctx.refresh(); else { body.replaceChildren(); v.draw(body, ctx); } }
    catch (e) { body.replaceChildren(el("div", { class: "err" }, "Not available: " + (e.message || e))); }
  }
}
function schedulePreview(force) {
  if (!S.live && !force) return;
  clearTimeout(pvTimer); pvTimer = setTimeout(doPreview, 450);
}
/* BLOCK_OV[car][block] = the inputs (as JSON) that block's data on screen was computed with. */
const BLOCK_OV = { ic: {}, ev: {} };
const blockKey = (car, b) => BLOCK_OV[car][b] ?? JSON.stringify(Object.entries(S.applied[car]).sort());
async function doPreview() {
  const car = S.car, blocks = PAGE_BLOCKS[S.page];
  if (!blocks || !S.data[car]) return;
  if (S.busy) return schedulePreview(true);
  const key = JSON.stringify(Object.entries(S.overrides[car]).sort());
  const need = blocks.filter((b) => blockKey(car, b) !== key);
  if (!need.length) return;                                   // what is on screen already matches the inputs
  const token = ++pvSeq, t0 = performance.now();
  setStatus("Updating the preview…", "busy");
  try {
    const r = await call("preview", [car, S.overrides[car], need]);
    if (token !== pvSeq) return;
    S.data[car].result = Object.assign({}, S.data[car].result, { subsystems: Object.assign({}, S.data[car].result.subsystems, r.blocks) });   // new objects: the baseline copy stays untouched
    for (const b of need) BLOCK_OV[car][b] = key;
    if (car !== S.car) return;
    const stale = !same(S.overrides[car], S.applied[car]);
    setStatus(`Preview updated in ${((performance.now() - t0) / 1000).toFixed(1)} s.` + (stale ? " Lap times and lap channels are from the last run: press Run simulation to update them." : ""), "ok");
    redraw(need);
  } catch (e) { if (token === pvSeq) setStatus("Preview failed: " + (e.message || e), "err"); }
}

// ------------------------------------------------------------------ render
function about() {
  const m = S.meta, go = (id) => { S.page = id; location.hash = `${S.car}/${id}`; render(); window.scrollTo(0, 0); };
  const GUIDE = [["overview", "Run the whole car on acceleration, skidpad, autocross and endurance. Start here for lap times and points."],
    ["drivers", "Pick who is driving: mass, seating position, sight line, and their personal grip and shift numbers."],
    ["lap", "Plot any channel the simulation produces along any event."],
    ["powertrain", "Engine or motor, gearing and differential, with a draggable breathing curve and gear sliders."],
    ["energy", "Endurance lap by lap: accumulator state for the EV, cooling and fuel for the IC car."],
    ["tires", "The grip envelope the solver drives on and the wheel loads."],
    ["suspension", "Drag the hard points or type them in; kinematics are re-solved against the frame."],
    ["aero", "Downforce, drag and balance, where the loads act, and each wing element modelled on its own."],
    ["brakes", "Balance, rotor temperatures, pad friction and the corner hardware."],
    ["chassis", "Move, add or remove frame joints, change tube sizes and materials, and see mass and stiffness."],
    ["studies", "Sweep any input and see what it does to each event."],
    ["inputs", "Every input of the model in one searchable list."]];
  const cards = GUIDE.map(([id, text]) => { const p = PAGES.find((x) => x.id === id);
    return el("button", { class: "guide", onclick: () => go(id) }, el("b", {}, val(p.label)), el("span", {}, text)); });
  return [el("div", { class: "card wide prose", html: `
<h3>What this is</h3>
<p>The team's predictive lap simulation for the CT-17 IC and CT-17 EV. Pick the car at the top right, then a section. Every page opens on the baseline results.
Change anything (a slider, a number, a dragged point) and press <b>Run simulation</b>: the same Python model the notebooks use runs inside your browser.
With <b>Live preview</b> on, a page's own curves follow your changes without running the lap.</p>` }),
    el("div", { class: "card wide" }, el("h3", {}, "Sections"), el("div", { class: "guides" }, cards)),
    el("div", { class: "card wide prose", html: `
<h3>What runs here</h3>
<ul><li>Quasi-steady-state four-wheel vehicle model, tire and traction envelope, hard-point kinematics, component aero model.</li>
<li>IC powertrain: restricted engine, gearbox, chain, differential, cooling, fuel. EV powertrain: motor, inverter, accumulator, regen.</li>
<li>All four dynamic events, endurance lap by lap, input sweeps, and the EV power-limit and gearing studies.</li>
<li>The spaceframe beam model, suspension kinematics, rotor temperatures, and a panel-method model of single wing sections.</li></ul>
<h3>What does not run in a browser</h3>
<ul><li>Multibody minimum-lap-time optimal control (CasADi + IPOPT) and the 3-D CFD solver (Numba). Run those from the notebooks.</li>
<li>Reading AiM <code>.xrk</code> logs and fitting from logged or TTC data. The site uses the values already in the config files.</li></ul>
<h3>Reading the numbers</h3>
<p>Inputs are stored in SI. Type a unit to enter anything else: <code>350 lbf/in</code>, <code>11 in</code>, <code>80 kW</code>, <code>245 kg</code>. Every EV input is currently a placeholder; see Accumulator → EV input status.
The first run downloads the Python runtime (about 40 MB, cached afterwards).</p>
<p class="muted">Built ${m.built || "?"}${m.commit ? " · commit " + m.commit : ""} · simulation bundle ${m.bundle_kB || "?"} kB</p>` })];
}

function render() {
  const car = S.car, page = PAGES.find((p) => p.id === S.page), main = $("#main");
  $("#tabs").replaceChildren(...PAGES.map((p) => el("button", { class: p.id === S.page ? "on" : "", onclick: () => { S.page = p.id; location.hash = `${S.car}/${p.id}`; render(); window.scrollTo(0, 0); } }, val(p.label))));
  const on = $("#tabs .on"); if (on) on.scrollIntoView({ block: "nearest", inline: "center" });
  document.querySelectorAll(".seg button").forEach((b) => b.classList.toggle("on", b.dataset.car === car));
  if (!S.data[car]) { main.replaceChildren(el("div", { class: "muted" }, "Loading…")); return; }
  const head = el("div", { class: "page-head" }, el("div", {}, el("h1", {}, val(page.label)), page.intro ? el("p", {}, val(page.intro)) : null));
  if (page.id === "about") return main.replaceChildren(head, el("div", { class: "grid" }, about()));
  if (page.id !== "drivers" && typeof driverBanner === "function") head.append(driverBanner());
  if (page.id === "inputs") return main.replaceChildren(head, inputsPanel(null, true));
  const vis = VISUALS.filter((v) => v.page === page.id && (!v.car || v.car === car)).sort((a, b) => (b.first ? 1 : 0) - (a.first ? 1 : 0));   // interactive figures lead the page
  const isOn = (v) => (v.id in S.hidden ? !S.hidden[v.id] : !v.off);
  const chips = el("div", { class: "chips" }, el("span", { class: "lbl" }, "Show"));
  if (page.lap) {
    const evs = Object.keys(R().laps).filter((e) => !R().laps[e].error);
    if (!evs.includes(S.event)) S.event = evs.includes("autocross") ? "autocross" : evs[0];
    const sel = el("select", { "aria-label": "Event", onchange: () => { S.event = sel.value; render(); } }, evs.map((e) => el("option", { value: e }, EVENT_NAMES[e])));
    sel.value = S.event; chips.prepend(el("span", { class: "lbl" }, "Event"), sel, el("span", { style: "width:10px" }));
  }
  for (const v of vis) chips.append(el("button", { class: "chip" + (isOn(v) ? " on" : ""), "aria-pressed": isOn(v), onclick: () => { S.hidden[v.id] = isOn(v); localStorage.setItem("lapsim.hidden", JSON.stringify(S.hidden)); render(); } }, v.title));
  const grid = el("div", { class: "grid" });
  main.replaceChildren(head, ...[typeof quickPanel === "function" ? quickPanel(page.id) : null, inputsPanel(val(page.groups) || [])].filter(Boolean), chips, grid);
  CARDS.clear();
  for (const v of vis.filter(isOn)) {
    const body = el("div"), card = el("section", { class: "card" + (v.wide ? " wide" : "") + (v.cls ? " " + v.cls : "") }, el("h3", {}, v.title), v.sub ? el("div", { class: "sub" }, v.sub) : null, body);
    grid.append(card);
    const ctx = { card }; CARDS.set(v.id, { v, body, ctx });
    try { v.draw(body, ctx); } catch (e) { body.replaceChildren(el("div", { class: "err" }, "Not available: " + (e.message || e))); }
  }
  if (!vis.some(isOn)) grid.append(el("div", { class: "card wide muted" }, "Nothing selected. Pick visuals above."));
  if (!same(S.overrides[car], S.applied[car])) schedulePreview();
}

// ------------------------------------------------------------------ loading files
/* Served over http(s) the page fetches its files. Opened straight from disk (file://) browsers block fetch and
   file workers, so build.py also writes every file as a small script (ic.json -> ic.js ...) that registers itself
   in window.__LAPSIM, and those are loaded with a <script> tag instead. ?nofetch=1 forces that path (for testing). */
const NOFETCH = location.protocol === "file:" || /[?&]nofetch=1/.test(location.search);
const ver = () => encodeURIComponent(S.meta.built || "");
function viaScript(path) {
  const reg = (window.__LAPSIM = window.__LAPSIM || {});
  if (path in reg) return Promise.resolve(reg[path]);
  return new Promise((resolve, reject) => {
    const sc = el("script", { src: path.replace(/\.\w+$/, (x) => (x === ".json" ? ".js" : x + ".js")) + "?v=" + ver() });
    sc.onload = () => (path in reg ? resolve(reg[path]) : reject(new Error(path + ": file is empty")));
    sc.onerror = () => reject(new Error(`${path} is missing. Rebuild the site: python web/build.py`));
    document.head.append(sc);
  });
}
async function getJSON(path) {
  if (!NOFETCH) {
    try { const r = await fetch(`${path}?v=${ver()}`); if (r.ok) return await r.json(); } catch (e) { /* fall through to the script copy */ }
  }
  return viaScript(path);
}
async function getBytes(path) {
  if (!NOFETCH) {
    try { const r = await fetch(`${path}?v=${ver()}`); if (r.ok) return await r.arrayBuffer(); } catch (e) { /* fall through */ }
  }
  const b64 = await viaScript(path), bin = atob(b64), out = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
  return out.buffer;
}
async function makeWorker() {
  if (!NOFETCH) { try { return new Worker("assets/worker.js"); } catch (e) { /* fall through */ } }
  const src = await viaScript("assets/worker.js");
  return new Worker(URL.createObjectURL(new Blob([src], { type: "text/javascript" })));
}

// ------------------------------------------------------------------ engine (Pyodide worker)
let worker = null, seq = 0; const waiting = new Map();
function setStatus(text, state = "") { $("#status").textContent = text; $("#dot").className = "dot " + state; }
let starting = null;
function startWorker() {
  return (starting = starting || (async () => {
    const w = await makeWorker();
    w.onmessage = (e) => {
      const m = e.data;
      if (m.status) { setStatus(m.status === "ready" ? "Running the simulation…" : m.status, "busy"); return; }
      if (m.fatal) return engineFailed(m.fatal);
      const p = waiting.get(m.id); if (!p) return; waiting.delete(m.id);
      m.ok ? p.resolve(JSON.parse(m.result)) : p.reject(new Error(m.error.split("\n").filter(Boolean).pop()));
    };
    w.onerror = (e) => engineFailed(e.message || "the worker could not be loaded");
    const zip = await getBytes("py/lapsim.zip");
    w.postMessage({ zip }, [zip]);
    return (worker = w);
  })());
}
function engineFailed(why) {
  const offline = !navigator.onLine ? " You appear to be offline: the first run needs the internet to download the Python runtime." : "";
  setStatus("The simulation engine could not start: " + why + "." + offline, "err");
  for (const p of waiting.values()) p.reject(new Error(why)); waiting.clear();
  if (worker) worker.terminate(); worker = null; starting = null;
}
function call(fn, args) {
  return new Promise((resolve, reject) => {
    const id = ++seq; waiting.set(id, { resolve, reject });
    startWorker().then((w) => w.postMessage({ id, fn, args }), (e) => { waiting.delete(id); starting = null; reject(e); });
  });
}
const same = (a, b) => JSON.stringify(Object.entries(a).sort()) === JSON.stringify(Object.entries(b).sort());
function pending() {
  const n = Object.keys(S.overrides[S.car]).length, dirty = !same(S.overrides[S.car], S.applied[S.car]);
  $("#changes").hidden = !n; $("#changes").textContent = `${n} input${n === 1 ? "" : "s"} changed`;
  $("#reset").hidden = !n;
  $("#runbar").hidden = !dirty || S.busy;
  $("#runbar-text").textContent = n ? `${n} input${n === 1 ? "" : "s"} changed, results not updated` : "Inputs reset, results not updated";
  const lv = $("#live"); if (lv) { lv.classList.toggle("on", S.live); lv.setAttribute("aria-pressed", S.live); }
  schedulePreview();
}
async function runSim() {
  if (S.busy) return;
  const car = S.car, ov = Object.assign({}, S.overrides[car]), t0 = performance.now();
  S.busy = true; $("#run").disabled = $("#run2").disabled = true; pending(); setStatus("Starting the simulation engine…", "busy");
  try {
    const p = call("run", [car, ov]);
    const tick = setInterval(() => { if ($("#dot").classList.contains("busy") && worker) $("#status").textContent = $("#status").textContent.replace(/( \(\d+ s\))?$/, ` (${Math.round((performance.now() - t0) / 1000)} s)`); }, 1000);
    const r = await p.finally(() => clearInterval(tick));
    S.data[car].result = r; S.applied[car] = ov; BLOCK_OV[car] = {};
    setStatus(`${car === "ic" ? "CT-17 IC" : "CT-17 EV"} simulated in your browser in ${((performance.now() - t0) / 1000).toFixed(1)} s` + (Object.keys(ov).length ? ` with ${Object.keys(ov).length} changed input(s)` : " (baseline inputs)"), "ok");
    if (S.car === car) render();
  } catch (e) { setStatus("Run failed: " + (e.message || e), "err"); }
  S.busy = false; $("#run").disabled = $("#run2").disabled = false; pending();
}

async function loadCar(car) {
  if (!S.data[car]) { S.data[car] = await getJSON(`data/${car}.json`); S.base[car] = S.data[car].result; }
  setStatus(Object.keys(S.applied[car]).length ? "Showing your last run" : "Showing the baseline (pre-computed). Change inputs and run to simulate in your browser.", "ok");
}
async function setCar(car) {
  S.car = car; render();
  try { await loadCar(car); } catch (e) {
    setStatus("Could not load the results: " + (e.message || e), "err");
    $("#main").replaceChildren(el("div", { class: "card wide prose", html: `<h3>The site's data did not load</h3>
<p>${String(e.message || e)}</p><p>From the <code>fsae_lapsim</code> folder run <code>python web/build.py --serve</code> and open <code>http://localhost:8000</code>.</p>` }));
    return;
  }
  if (typeof DR !== "undefined" && DR.chosen().id !== "design") DR.apply();      // the driver picked last time is still in the car
  render(); pending();
}

window.addEventListener("DOMContentLoaded", async () => {
  const [c, p] = location.hash.slice(1).split("/");
  if (c === "ic" || c === "ev") S.car = c; if (PAGES.some((x) => x.id === p)) S.page = p;
  document.querySelectorAll(".seg button").forEach((b) => b.addEventListener("click", () => { location.hash = `${b.dataset.car}/${S.page}`; setCar(b.dataset.car); }));
  $("#run").onclick = $("#run2").onclick = runSim;
  $("#reset").onclick = () => { S.overrides[S.car] = {}; if (typeof FR !== "undefined") delete FR._w[S.car]; render(); pending(); };
  $("#live").onclick = () => { S.live = !S.live; localStorage.setItem("lapsim.live", S.live ? "1" : "0"); pending(); };
  try { S.meta = await getJSON("data/meta.json"); } catch (e) { /* optional */ }
  try { S.driverFile = await getJSON("data/drivers.json"); } catch (e) { S.driverFile = null; }
  $("#buildinfo").textContent = "© Haley Harper" + (S.meta.built ? `, Built ${S.meta.built}` : "");
  await setCar(S.car);
  window.addEventListener("hashchange", () => {
    const [c, p] = location.hash.slice(1).split("/"), car = c === "ic" || c === "ev" ? c : S.car, page = PAGES.some((x) => x.id === p) ? p : S.page;
    if (car === S.car && page === S.page) return;
    S.page = page; car === S.car ? render() : setCar(car);
  });
  let rt; window.addEventListener("resize", () => { clearTimeout(rt); rt = setTimeout(() => document.querySelectorAll(".js-plotly-plot").forEach((n) => Plotly.Plots.resize(n)), 150); });
});
