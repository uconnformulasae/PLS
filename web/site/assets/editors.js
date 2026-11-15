/* Interactive figures and controls: draggable frame nodes, suspension hard points, car layout, curves and sliders.
   Loaded after app.js and uses its helpers (S, V, el, plot, table, pending, need, SUB ...).
   Every figure writes its change into S.overrides[car] like any other input, so Run simulation and the live
   preview see it:   geom:frame        the whole editable frame (nodes, tubes, sizes, materials)
                     geom:hardpoints   {front: {point: [x, y, z]}, rear: {...}}  (metres, changed points only)
                     geom:tracks       {autocross: file, endurance: file} */
"use strict";

// ------------------------------------------------------------------ shared helpers
const SVGNS = "http://www.w3.org/2000/svg";
const svg = (tag, attrs = {}) => { const e = document.createElementNS(SVGNS, tag); for (const [k, v] of Object.entries(attrs)) e.setAttribute(k, v); return e; };
const clone = (o) => JSON.parse(JSON.stringify(o));
const GEO = () => S.data[S.car].geometry || {};
const OV = () => S.overrides[S.car];
let SCHEMA_MAP = {};
function inputOf(key) {
  const m = SCHEMA_MAP[S.car] || (SCHEMA_MAP[S.car] = Object.fromEntries(S.data[S.car].schema.inputs.map((i) => [i.key, i])));
  return m[key];
}
/* Current value of an input (SI): the edit if there is one and it is a number, else the baseline. */
function cur(key) {
  const o = OV()[key], i = inputOf(key);
  if (typeof o === "number" || Array.isArray(o) || typeof o === "boolean") return o;
  return i ? i.value : undefined;
}
/* Set an input from a figure or slider and keep every other control showing that input in step. */
function setOv(key, v, { quiet } = {}) {
  const i = inputOf(key), ov = OV();
  const base = i ? i.value : undefined;
  const sameAsBase = Array.isArray(v) ? JSON.stringify(v) === JSON.stringify(base) : typeof v === "number" && typeof base === "number" ? Math.abs(v - base) <= 1e-9 * Math.max(1, Math.abs(base)) : v === base;
  if (sameAsBase) delete ov[key]; else ov[key] = v;
  document.querySelectorAll(`[aria-label="${CSS.escape(key)}"]`).forEach((inp) => {
    if (inp === document.activeElement) return;
    inp.value = Array.isArray(v) ? v.join(", ") : v ?? "";
    const row = inp.closest(".field"); if (row) row.classList.toggle("changed", key in ov);
  });
  document.querySelectorAll(`[data-quick="${CSS.escape(key)}"]`).forEach((row) => row._sync && row._sync());
  if (!quiet) pending();
}
const tile = (k, v, unit, d, cls = "") => el("div", { class: "kpi" }, el("div", { class: "k" }, k), el("div", { class: "v" }, v, unit ? el("small", {}, unit) : null), d ? el("div", { class: "d " + cls }, d) : null);
const delta = (now, base, digits = 2, unit = "") => (now == null || base == null || Math.abs(now - base) < 0.5 * 10 ** -digits ? "" : `${now > base ? "+" : ""}${(now - base).toFixed(digits)}${unit} vs baseline`);
const numIn = (value, onchange, attrs = {}) => { const i = el("input", Object.assign({ type: "number", step: "any", value }, attrs)); i.addEventListener("change", () => { if (i.value !== "" && isFinite(+i.value)) onchange(+i.value, i); }); return i; };

// ------------------------------------------------------------------ multi-view drag editor
/* Orthographic views of 3-D points; drag a point in a view to move it in that view's two axes.
   o.views     [{title, ax: [i, j], sign: [sx, sy]}]  screen X = sx * p[i], screen Y (up) = sy * p[j]   (metres in, mm drawn)
   o.points()  [{id, p: [x, y, z], label, color, fixed}]
   o.segs()    [{a, b, color, w, dash, pick}]          pick = value passed to o.onPick when the line is clicked
   o.decor(view, proj)  extra SVG markup behind everything (proj(p) -> "X Y" numbers in mm)
   o.onMove(id, p, done)   o.onSelect(id)   o.onPick(pick) */
function viewEditor(o) {
  const root = el("div", { class: "views" + (o.views.length === 3 ? " three" : "") }), views = [];
  let selected = o.selected ?? null;
  const proj = (v, p) => [v.sign[0] * p[v.ax[0]] * 1e3, -v.sign[1] * p[v.ax[1]] * 1e3];
  for (const v of o.views) {
    const s = svg("svg", { class: "view", role: "img", "aria-label": v.title });
    const gDecor = svg("g"), gSeg = svg("g"), gPts = svg("g");
    s.append(gDecor, gSeg, gPts);
    const box = el("div", { class: "viewbox" }, el("div", { class: "viewtitle" }, v.title), s);
    root.append(box);
    const vw = { v, s, gDecor, gSeg, gPts, dots: new Map(), bounds: null };
    views.push(vw);
    gSeg.addEventListener("click", (e) => { const t = e.target.closest("[data-pick]"); if (t && o.onPick) o.onPick(JSON.parse(t.dataset.pick)); });
  }
  function fit() {
    for (const vw of views) {
      let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
      const add = (p) => { const [X, Y] = proj(vw.v, p); x0 = Math.min(x0, X); x1 = Math.max(x1, X); y0 = Math.min(y0, Y); y1 = Math.max(y1, Y); };
      o.points().forEach((q) => add(q.p)); o.segs().forEach((g) => { if (!g.nofit) { add(g.a); add(g.b); } });
      (o.extent ? o.extent() : []).forEach(add);
      if (!isFinite(x0)) { x0 = y0 = -100; x1 = y1 = 100; }
      const pad = 0.07 * Math.max(x1 - x0, y1 - y0, 50);
      vw.bounds = [x0 - pad, y0 - pad, x1 - x0 + 2 * pad, y1 - y0 + 2 * pad];
      vw.s.setAttribute("viewBox", vw.bounds.map((n) => n.toFixed(1)).join(" "));
      vw.s.style.aspectRatio = `${vw.bounds[2]} / ${vw.bounds[3]}`;
    }
  }
  function draw() {
    if (!views[0].bounds) return;
    const pts = o.points(), segs = o.segs();
    for (const vw of views) {
      const ctm = vw.s.getScreenCTM(), px = ctm && ctm.a > 0 ? 1 / ctm.a : vw.bounds[2] / Math.max(1, vw.s.clientWidth || 320);   // mm per screen pixel
      const b = vw.bounds, lines = [], fs = 10 * px, names = vw.v.names || ["", ""];
      // major lines about every 70 screen pixels on a 1-2-5 step, four minor divisions between, numbered in mm
      const raw = 70 * px, mag = 10 ** Math.floor(Math.log10(raw)), step = [1, 2, 5, 10].map((k) => k * mag).find((k) => k >= raw) || 10 * mag, minor = step / (step / mag === 2 ? 4 : 5);
      for (let x = Math.ceil(b[0] / minor) * minor; x <= b[0] + b[2]; x += minor) { const maj = Math.abs(x / step - Math.round(x / step)) < 1e-6;
        lines.push(`<line x1="${x}" y1="${b[1]}" x2="${x}" y2="${b[1] + b[3]}" class="grid${Math.abs(x) < 1e-6 ? " zero" : maj ? " major" : ""}"/>`);
        if (maj) lines.push(`<text x="${x + 2 * px}" y="${b[1] + b[3] - 3 * px}" font-size="${fs}" class="tick">${Math.round(x / vw.v.sign[0])}</text>`); }
      for (let y = Math.ceil(b[1] / minor) * minor; y <= b[1] + b[3]; y += minor) { const maj = Math.abs(y / step - Math.round(y / step)) < 1e-6;
        lines.push(`<line x1="${b[0]}" y1="${y}" x2="${b[0] + b[2]}" y2="${y}" class="grid${Math.abs(y) < 1e-6 ? " zero" : maj ? " major" : ""}"/>`);
        if (maj) lines.push(`<text x="${b[0] + 3 * px}" y="${y - 2 * px}" font-size="${fs}" class="tick">${Math.round(-y / vw.v.sign[1])}</text>`); }
      lines.push(`<text x="${b[0] + b[2] - 4 * px}" y="${b[1] + b[3] - 14 * px}" font-size="${fs}" text-anchor="end" class="tick name">${names[0]} [mm]</text><text x="${b[0] + 3 * px}" y="${b[1] + 11 * px}" font-size="${fs}" class="tick name">${names[1]} [mm]</text>`);
      vw.gDecor.innerHTML = lines.join("") + (o.decor ? o.decor(vw.v, (p) => proj(vw.v, p), px) : "");
      vw.gSeg.innerHTML = segs.filter((g) => !g.only || g.only === vw.v.title).map((g) => {
        const [x1, y1] = proj(vw.v, g.a), [x2, y2] = proj(vw.v, g.b), at = `x1="${x1.toFixed(1)}" y1="${y1.toFixed(1)}" x2="${x2.toFixed(1)}" y2="${y2.toFixed(1)}"`;
        return `<line ${at} stroke="${g.color || "#000E2F"}" stroke-width="${(g.w || 2) * px}" stroke-linecap="round"${g.dash ? ` stroke-dasharray="${4 * px} ${4 * px}"` : ""}${g.sel ? ' class="selseg"' : ""}/>`
          + (g.pick !== undefined ? `<line ${at} stroke="transparent" stroke-width="${12 * px}" data-pick='${JSON.stringify(g.pick)}' style="cursor:pointer"/>` : "");
      }).join("");
      const seen = new Set();
      for (const q of pts) {
        seen.add(q.id);
        let d = vw.dots.get(q.id);
        if (!d) {
          d = svg("g", { class: "dot-g" }); d.append(svg("circle", { class: "hit" }), svg("circle", { class: "pt" }));
          d.append(svg("title")); vw.gPts.append(d); vw.dots.set(q.id, d);
          if (!q.fixed) attachDrag(vw, d, q.id);
          d.addEventListener("click", () => select(q.id));
        }
        const [X, Y] = proj(vw.v, q.p);
        d.setAttribute("transform", `translate(${X.toFixed(2)} ${Y.toFixed(2)})`);
        d.children[0].setAttribute("r", 13 * px); d.children[1].setAttribute("r", (q.id === selected ? 6.5 : 4.5) * px);
        d.children[1].setAttribute("stroke-width", 1.6 * px);
        d.children[1].setAttribute("fill", q.id === selected ? "#eb6834" : q.fixed ? "#8a9099" : q.color || "#2a78d6");
        d.children[2].textContent = q.label || String(q.id);
        d.classList.toggle("fixed", !!q.fixed);
      }
      for (const [id, d] of vw.dots) if (!seen.has(id)) { d.remove(); vw.dots.delete(id); }
    }
  }
  function attachDrag(vw, d, id) {
    let start = null;
    d.addEventListener("pointerdown", (e) => {
      const q = o.points().find((x) => x.id === id); if (!q || q.fixed) return;
      d.setPointerCapture(e.pointerId); select(id);
      start = { p: q.p.slice(), x: e.clientX, y: e.clientY }; e.preventDefault();
    });
    const move = (e, done) => {
      if (!start) return;
      const m = vw.s.getScreenCTM(); if (!m) return;
      const dx = (e.clientX - start.x) / m.a, dy = (e.clientY - start.y) / m.d, p = start.p.slice();
      p[vw.v.ax[0]] += dx / 1e3 / vw.v.sign[0]; p[vw.v.ax[1]] += -dy / 1e3 / vw.v.sign[1];
      o.onMove(id, p, done); draw();
    };
    d.addEventListener("pointermove", (e) => move(e, false));
    const end = (e) => { if (!start) return; move(e, true); start = null; fit(); draw(); };
    d.addEventListener("pointerup", end); d.addEventListener("pointercancel", end);
  }
  function select(id) { const changed = selected !== id; selected = id; if (changed) draw(); if (o.onSelect && id != null) o.onSelect(id); }
  requestAnimationFrame(() => { fit(); draw(); });
  const ro = new ResizeObserver(() => { if (views[0].bounds) draw(); }); ro.observe(root);
  return { node: root, draw, fit: () => { fit(); draw(); }, select, get selected() { return selected; } };
}
/* Coordinate boxes for the selected point (mm). */
function coordBar(labels, get, set) {
  const bar = el("div", { class: "coordbar" });
  const update = () => {
    const g = get();
    if (!g) return bar.replaceChildren(el("span", { class: "muted" }, "Click or drag a point. Its coordinates appear here."));
    bar.replaceChildren(el("b", {}, g.label), ...labels.map((l, a) => el("label", {}, l, numIn(+(g.p[a] * 1e3).toFixed(2), (v) => { const p = get().p.slice(); p[a] = v / 1e3; set(p); }, { style: "width:92px" }), "mm")), ...(g.extra || []));
  };
  update();
  return { node: bar, update };
}

// ------------------------------------------------------------------ quick-adjust sliders
/* [key, label, unit shown, factor SI -> shown, min, max, step, value used when the baseline is "not set"] */
const QUICK = {
  overview: { all: [["vehicle:events.endurance_distance", "Endurance distance", "km", 1e-3, 5, 30, 0.5], ["vehicle:events.skidpad_radius", "Skidpad radius (path)", "m", 1, 7, 12, 0.025], ["vehicle:events.accel_length", "Acceleration length", "m", 1, 50, 100, 1]] },
  chassis: { all: [["vehicle:mass.mass_no_driver", "Car mass without driver", "kg", 1, 150, 330, 0.5], ["vehicle:mass.driver_mass", "Driver mass", "kg", 1, 45, 110, 0.5],
    ["vehicle:mass.front_weight_fraction", "Weight on the front axle", "%", 100, 40, 60, 0.1], ["vehicle:mass.left_weight_fraction", "Weight on the left side", "%", 100, 45, 55, 0.1],
    ["vehicle:mass.cg_height", "CG height", "mm", 1e3, 200, 400, 1], ["vehicle:geometry.wheelbase", "Wheelbase", "mm", 1e3, 1525, 1800, 1],
    ["vehicle:geometry.track_front", "Front track", "mm", 1e3, 1050, 1400, 1], ["vehicle:geometry.track_rear", "Rear track", "mm", 1e3, 1050, 1400, 1]] },
  tires: { all: [["vehicle:tires.mu0", "Peak friction at the reference load", "", 1, 1.0, 2.4, 0.01], ["vehicle:tires.lambda_mu", "Rig-to-track grip scale", "", 1, 0.5, 1.2, 0.01],
    ["vehicle:tires.mux_over_muy", "Longitudinal / lateral grip", "", 1, 0.7, 1.3, 0.01], ["vehicle:tires.cornering_stiffness_per_load", "Cornering stiffness per load", "1/rad", 1, 8, 45, 0.5],
    ["vehicle:tires.pressure_front", "Front pressure", "psi", 1 / 6894.757, 6, 16, 0.25], ["vehicle:tires.pressure_rear", "Rear pressure", "psi", 1 / 6894.757, 6, 16, 0.25],
    ["vehicle:tires.rolling_resistance", "Rolling resistance coefficient", "", 1, 0.005, 0.04, 0.001, 0.02]] },
  suspension: { all: [["vehicle:front.spring_rate", "Front spring", "N/mm", 1e-3, 20, 110, 0.5], ["vehicle:rear.spring_rate", "Rear spring", "N/mm", 1e-3, 20, 110, 0.5],
    ["vehicle:front.static_camber_deg", "Front static camber", "deg", 1, -4, 0.5, 0.05], ["vehicle:rear.static_camber_deg", "Rear static camber", "deg", 1, -4, 0.5, 0.05],
    ["vehicle:front.static_toe_deg", "Front static toe", "deg", 1, -1, 1, 0.05], ["vehicle:front.damping_ratio_bump", "Front damping ratio, bump", "", 1, 0.3, 1.5, 0.01],
    ["vehicle:rear.damping_ratio_bump", "Rear damping ratio, bump", "", 1, 0.3, 1.5, 0.01], ["vehicle:steering.ackermann_fraction", "Ackermann", "%", 100, 0, 120, 1]] },
  aero: { all: [["vehicle:aero.calib_ClA", "Downforce coefficient × area (ClA)", "m²", 1, 0, 4, 0.01], ["vehicle:aero.calib_CdA", "Drag coefficient × area (CdA)", "m²", 1, 0.4, 2.2, 0.01],
    ["vehicle:aero.calib_front_axle_share", "Aero load on the front axle", "%", 100, 30, 70, 0.1], ["vehicle:aero.components.front_wing.cfd_downforce", "Front wing downforce at the CFD speed", "N", 1, 0, 500, 1],
    ["vehicle:aero.components.rear_wing.cfd_downforce", "Rear wing downforce at the CFD speed", "N", 1, 0, 500, 1], ["vehicle:aero.components.rear_wing.cfd_drag", "Rear wing drag at the CFD speed", "N", 1, 0, 200, 0.5],
    ["vehicle:aero.ambient_temperature_C", "Air temperature", "°C", 1, 0, 45, 0.5]] },
  brakes: { all: [["vehicle:brakes.front_bias", "Front brake bias", "%", 100, 40, 80, 0.5], ["vehicle:brakes.rear_pressure_ratio", "Rear / front line pressure", "", 1, 0.3, 1, 0.005],
    ["vehicle:brakes.torque_per_bar_front", "Front brake torque per bar", "N·m/bar", 1e5, 2, 10, 0.02], ["vehicle:brakes.torque_per_bar_rear", "Rear brake torque per bar", "N·m/bar", 1e5, 2, 10, 0.02]] },
  powertrain: {
    ic: [["pt:final_drive.front_sprocket", "Front sprocket", "teeth", 1, 9, 15, 1], ["pt:final_drive.rear_sprocket", "Rear sprocket", "teeth", 1, 26, 44, 1],
      ["pt:engine.rev_limit", "Rev limit", "rpm", 1, 9000, 14500, 50], ["pt:engine.launch_rpm", "Launch rpm", "rpm", 1, 5000, 13000, 50], ["pt:engine.torque_scale", "Engine torque scale", "", 1, 0.7, 1.3, 0.005],
      ["pt:intake.restrictor_diameter", "Restrictor diameter", "mm", 1e3, 15, 25, 0.1], ["pt:gearbox.upshift_time", "Upshift time", "ms", 1e3, 20, 300, 5], ["pt:differential.preload", "Differential preload", "N·m", 1, 0, 80, 1]],
    ev: [["ev:motor.peak_torque", "Motor peak torque", "N·m", 1, 80, 400, 1], ["ev:motor.peak_power", "Motor peak power", "kW", 1e-3, 30, 160, 1], ["ev:motor.max_rpm", "Motor maximum speed", "rpm", 1, 3000, 20000, 100],
      ["ev:motor.corner_rpm", "Motor corner speed", "rpm", 1, 1500, 12000, 50], ["ev:drivetrain.front_sprocket", "Front sprocket", "teeth", 1, 9, 16, 1], ["ev:drivetrain.rear_sprocket", "Rear sprocket", "teeth", 1, 28, 60, 1],
      ["ev:inverter.max_phase_current", "Inverter phase current limit", "A", 1, 100, 600, 5], ["ev:regen.max_torque", "Regen torque", "N·m", 1, 0, 200, 1], ["ev:regen.max_power", "Regen power", "kW", 1e-3, 0, 80, 1]] },
  energy: {
    ev: [["ev:strategy.endurance_power_limit", "Endurance power limit", "kW", 1e-3, 20, 80, 1, 80000], ["ev:strategy.power_limit", "Power limit, every event", "kW", 1e-3, 20, 80, 1, 80000],
      ["ev:accumulator.start_soc", "State of charge at the start", "%", 100, 50, 100, 1], ["pack:pack.series", "Cells in series", "", 1, 60, 160, 1], ["pack:pack.parallel", "Cells in parallel", "", 1, 1, 10, 1],
      ["pack:cell.capacity_ah", "Cell capacity", "Ah", 1, 2, 10, 0.1], ["pack:cell.r0", "Cell resistance", "mΩ", 1e3, 2, 40, 0.5], ["pack:limits.i_dis_peak", "Cell peak discharge current", "A", 1, 10, 120, 1]],
    ic: [["pt:cooling.ua_ref", "Radiator UA at the reference flow", "W/K", 1, 300, 2000, 10], ["pt:cooling.fan_on_temp", "Fan-on temperature", "°C", 1, 80, 110, 0.5], ["pt:cooling.start_temp", "Coolant temperature at the start", "°C", 1, 20, 100, 1],
      ["pt:ambient.temperature", "Ambient temperature", "°C", 1, 0, 45, 0.5], ["pt:fuel.start_fill", "Fuel at the start", "L", 1e3, 2, 7, 0.05], ["pt:engine.fuel_scale", "Fuel-use scale", "", 1, 0.8, 1.2, 0.005]] },
};
function quickPanel(pageId) {
  const q = QUICK[pageId]; if (!q) return null;
  const defs = (q.all || []).concat(q[S.car] || []).filter((d) => inputOf(d[0]));
  if (!defs.length) return null;
  const rows = defs.map(([key, label, unit, k, lo, hi, step, dflt]) => {
    const inp = inputOf(key), baseSI = inp.value ?? dflt, shown = (si) => +(si * k).toFixed(6);
    const dec = Math.max(0, -Math.floor(Math.log10(step) + 1e-9));
    const range = el("input", { type: "range", min: lo, max: hi, step, "aria-label": label });
    const box = el("input", { type: "number", step, "aria-label": label + " value" });
    const reset = el("button", { class: "qreset", title: "Back to the baseline value", onclick: () => { setOv(key, inp.value); } }, "↺");
    const row = el("div", { class: "quick", "data-quick": key, title: [inp.note, inp.source && "source: " + inp.source].filter(Boolean).join(" · ") },
      el("div", { class: "qlabel" }, el("span", {}, label), el("span", { class: "qbase" }, inp.value == null ? "not set in the config" : `baseline ${shown(baseSI).toFixed(dec)} ${unit}`)),
      range, el("div", { class: "qval" }, box, el("span", { class: "u" }, unit), reset));
    row._sync = () => {
      const c = cur(key), v = shown(typeof c === "number" ? c : baseSI ?? (lo + hi) / 2 / k);
      if (document.activeElement !== box) box.value = v.toFixed(dec);
      range.value = v; row.classList.toggle("changed", key in OV()); reset.hidden = !(key in OV());
    };
    const apply = (v) => { if (!isFinite(v)) return; setOv(key, +(v / k).toPrecision(10)); };
    range.addEventListener("input", () => apply(+range.value));
    box.addEventListener("change", () => apply(+box.value));
    row._sync();
    return row;
  });
  return el("details", { class: "inputs quickpanel", open: true }, el("summary", {}, "Quick adjust", el("span", { class: "count" }, "sliders for the main values · every value is under Inputs below")), el("div", { class: "inputs-body quickgrid" }, rows));
}

// ------------------------------------------------------------------ frame editor
const FR = {
  colour: "size", selTube: null, addFrom: null, mode: "move", mirror: true,
  _w: {},                 // per car: the working copy while it still equals the baseline
  get state() { return OV()["geom:frame"] || this._w[S.car] || (this._w[S.car] = clone(GEO().frame)); },
  base() { return GEO().frame; },
};
function frameEnds(st) {
  const P = st.tubes.map(() => [null, null]);
  for (let pass = 0; pass < 8; pass++) {
    let todo = false;
    st.tubes.forEach((t, i) => { for (const k of [0, 1]) {
      if (P[i][k]) continue;
      const o = t.off[k];
      if (t.n[k] != null && t.n[k] >= 0) { const n = st.nodes[t.n[k]]; P[i][k] = [n[0] + o[0], n[1] + o[1], n[2] + o[2]]; }
      else { const [j, sp] = t.host[k], h = P[j]; if (h && h[0] && h[1]) P[i][k] = [0, 1, 2].map((a) => h[0][a] + sp * (h[1][a] - h[0][a]) + o[a]); else todo = true; }
    } });
    if (!todo) break;
  }
  return P;
}
const tubeArea = (z) => (z.shape === "square" ? z.od ** 2 - Math.max(0, z.od - 2 * z.wall) ** 2 : Math.PI / 4 * (z.od ** 2 - Math.max(0, z.od - 2 * z.wall) ** 2));
const dist3 = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
function frameStats(st) {
  const P = frameEnds(st), len = P.map((p) => (p[0] && p[1] ? dist3(p[0], p[1]) : 0));
  const mass = st.tubes.map((t, i) => { const z = st.sizes[t.size]; return (st.materials[z.material] || st.materials["4130 steel"]).density * tubeArea(z) * len[i]; });
  const sizes = st.sizes.map(() => ({ count: 0, length: 0, mass: 0 }));
  st.tubes.forEach((t, i) => { const s = sizes[t.size]; s.count++; s.length += len[i]; s.mass += mass[i]; });
  return { P, len, mass, sizes, total: mass.reduce((a, b) => a + b, 0), length: len.reduce((a, b) => a + b, 0) };
}
function frameCommit(structural) {
  const st = FR.state;
  if (JSON.stringify(st) === JSON.stringify(FR.base())) delete OV()["geom:frame"]; else OV()["geom:frame"] = st;
  FR._w[S.car] = st;
  pending();
  for (const id of ["frame_edit", "frame3d", "frame_sizes", "frame_tubes", "frame_mats", "frame_nodes", "hp_edit", "driver_view"]) { const c = CARDS.get(id); if (c && c.ctx.local) c.ctx.local(structural); }
}
function frameMirrorMap(st) {
  const b = st.nodes, m = b.map(() => -1);          // pairs found from where the joints are now
  b.forEach((p, i) => { let best = -1, bd = 0.02; b.forEach((q, j) => { const d = Math.hypot(p[0] - q[0], p[1] + q[1], p[2] - q[2]); if (d < bd) { bd = d; best = j; } }); m[i] = best; });
  return m;
}
const SIZE_COLORS = ["#000E2F", "#2a78d6", "#eb6834", "#1baf7a", "#6250d6", "#eda100", "#e87ba4", "#008300", "#8a9099", "#5b93dc", "#b26a00", "#47526a"];
function tubeColor(st, i, energy) {
  if (FR.colour === "energy" && energy && energy.length === st.tubes.length) {
    const mx = Math.max(...energy, 1e-12), f = Math.sqrt(energy[i] / mx);
    return ["#c9d3e3", "#a9c8f0", "#7fadea", "#5b93dc", "#2259a8", "#0B2152"][Math.min(5, Math.floor(f * 6))];
  }
  return FR.colour === "size" ? SIZE_COLORS[st.tubes[i].size % SIZE_COLORS.length] : "#000E2F";
}
V("chassis", "frame_edit", "Frame editor", "Drag a node in any view. Tubes joined to it follow. The beam model recomputes mass, CG and torsional stiffness.", (b, ctx) => {
  need(GEO().frame); if (GEO().frame.error) throw new Error(GEO().frame.error);
  const tiles = el("div", { class: "kpis" }), warn = el("div");
  const st = () => FR.state;
  let mir = frameMirrorMap(st()), stats = frameStats(st());
  const ed = viewEditor({
    views: [{ title: "Side (front of the car on the left)", ax: [0, 2], sign: [1, 1], names: ["x", "z"] }, { title: "Top", ax: [0, 1], sign: [1, -1], names: ["x", "y"] }, { title: "Front", ax: [1, 2], sign: [1, 1], names: ["y", "z"] }],
    points: () => st().nodes.map((p, i) => ({ id: i, p, label: `node ${i + 1}`, color: FR.addFrom === i ? "#1baf7a" : "#2a78d6" })),
    segs: () => { const en = (SUB().frame || {}).energy; return stats.P.map((p, i) => ({ a: p[0], b: p[1], color: tubeColor(st(), i, en), w: FR.selTube === i ? 4.5 : 2.2, pick: i, sel: FR.selTube === i })); },
    decor: (v, proj, px) => (v.ax[1] === 2 ? `<line x1="-99999" x2="99999" y1="0" y2="0" stroke="#7a859c" stroke-width="${1.2 * px}"/>` : ""),
    onMove: (id, p) => {
      const s = st(), old = s.nodes[id], d = [p[0] - old[0], p[1] - old[1], p[2] - old[2]];
      s.nodes[id] = p;
      const m = FR.mirror ? mir[id] : -1;
      if (m >= 0 && m !== id) s.nodes[m] = [s.nodes[m][0] + d[0], s.nodes[m][1] - d[1], s.nodes[m][2] + d[2]];
      if (m === id) s.nodes[id][1] = old[1];                    // a centre-line node stays on the centre line
      stats = frameStats(s); bar.update(); frameCommit(false);
    },
    onSelect: (id) => {
      if (FR.mode === "add") {
        if (FR.addFrom == null) FR.addFrom = id;
        else if (FR.addFrom !== id) { st().tubes.push({ n: [FR.addFrom, id], off: [[0, 0, 0], [0, 0, 0]], host: [null, null], size: FR.selSize ?? 0, bend: false }); FR.addFrom = null; FR.mode = "move"; stats = frameStats(st()); frameCommit(true); tools(); }
      }
      bar.update(); ed.draw();
    },
    onPick: (i) => { FR.selTube = FR.selTube === i ? null : i; ed.draw(); const c = CARDS.get("frame_tubes"); if (c && c.ctx.local) c.ctx.local(false); tubeBar(); },
  });
  const bar = coordBar(["x (rearward)", "y (right)", "z (up)"], () => (ed.selected == null || !st().nodes[ed.selected] ? null : { label: `Node ${ed.selected + 1}`, p: st().nodes[ed.selected] }),
    (p) => { st().nodes[ed.selected] = p; stats = frameStats(st()); ed.draw(); frameCommit(false); });
  const tb = el("div", { class: "coordbar" });
  function tubeBar() {
    const i = FR.selTube, s = st();
    if (i == null || !s.tubes[i]) return tb.replaceChildren(el("span", { class: "muted" }, "Click a tube to select it."));
    const t = s.tubes[i];
    const sel = el("select", { onchange: () => { t.size = +sel.value; stats = frameStats(s); frameCommit(true); ed.draw(); } }, s.sizes.map((z, k) => el("option", { value: k }, sizeName(z, k))));
    sel.value = t.size;
    tb.replaceChildren(el("b", {}, `Tube ${i + 1}`), el("span", {}, `${(stats.len[i] * 1e3).toFixed(0)} mm · ${stats.mass[i].toFixed(3)} kg`), el("label", {}, "size", sel),
      el("button", { class: "btn small", onclick: () => { frameDeleteTube(i); FR.selTube = null; stats = frameStats(st()); frameCommit(true); ed.draw(); tubeBar(); } }, "Delete tube"));
  }
  const toolbar = el("div", { class: "ctl" });
  function tools() {
    const colour = el("select", { "aria-label": "Tube colour", onchange: () => { FR.colour = colour.value; ed.draw(); const c = CARDS.get("frame3d"); if (c) c.ctx.local(); } },
      el("option", { value: "size" }, "Colour: tube size"), el("option", { value: "energy" }, "Colour: share of twist energy"), el("option", { value: "none" }, "Colour: none"));
    colour.value = FR.colour;
    toolbar.replaceChildren(colour,
      el("label", { class: "check" }, el("input", { type: "checkbox", checked: FR.mirror, onchange: (e) => { FR.mirror = e.target.checked; } }), "Mirror left / right"),
      el("button", { class: "btn small" + (FR.mode === "add" ? " navy" : ""), onclick: () => { FR.mode = FR.mode === "add" ? "move" : "add"; FR.addFrom = null; tools(); ed.draw(); } }, FR.mode === "add" ? "Click two nodes… (cancel)" : "Add tube"),
      el("button", { class: "btn small", title: "A new free joint next to the selected one: drag it where it is wanted, then Add tube to connect it", onclick: () => { const s = st(), from = s.nodes[ed.selected ?? 0]; s.nodes.push([from[0], from[1], from[2] + 0.08]); mir = frameMirrorMap(s); ed.draw(); ed.select(s.nodes.length - 1); frameCommit(true); } }, "Add joint"),
      el("button", { class: "btn small", title: "Put a new joint in the middle of the selected tube", onclick: () => { if (FR.selTube == null) return alert("Click a tube first: the new joint goes at its middle and can then be dragged."); const n = frameSplitTube(FR.selTube); FR.selTube = null; mir = frameMirrorMap(st()); frameCommit(true); ed.select(n); } }, "Add joint on tube"),
      el("button", { class: "btn small", title: "Remove the selected joint and the tubes that end on it", onclick: () => { if (ed.selected == null) return alert("Click a joint first."); const k = st().tubes.filter((t) => t.n.includes(ed.selected)).length;
        if (!confirm(`Remove joint ${ed.selected + 1} and the ${k} tube${k === 1 ? "" : "s"} that end on it?`)) return; frameDeleteNode(ed.selected); FR.selTube = null; ed.select(null); mir = frameMirrorMap(st()); frameCommit(true); } }, "Remove joint"),
      el("button", { class: "btn small", onclick: () => ed.fit() }, "Fit views"),
      el("button", { class: "btn small", onclick: () => { delete OV()["geom:frame"]; delete FR._w[S.car]; FR.selTube = null; render(); pending(); } }, "Reset frame"),
      el("label", { class: "check" }, el("input", { type: "checkbox", checked: !!st().use_stiffness, onchange: (e) => { st().use_stiffness = e.target.checked; frameCommit(false); } }), "Include frame twist in the lap simulation"));
  }
  function showTiles() {
    const f = SUB().frame || {}, base = f.base || {}, live = stats.total, s = st();
    const fresh = f.count === s.tubes.length && Math.abs((f.mass ?? 0) - live) < 0.02;        // the Python numbers belong to this geometry
    tiles.replaceChildren(
      tile("Frame mass", live.toFixed(2), "kg", delta(live, base.mass, 2, " kg"), live > base.mass ? "up" : "down"),
      tile("Torsional stiffness", fresh && f.k_per_deg != null ? f.k_per_deg.toFixed(0) : "…", "N·m/deg", fresh ? delta(f.k_per_deg, base.k_per_deg, 0, "") : "updates with the preview", fresh && f.k_per_deg < base.k_per_deg ? "up" : "down"),
      tile("Stiffness per kg", fresh && f.k_per_deg != null ? (f.k_per_deg / live).toFixed(1) : "…", "N·m/deg/kg"),
      tile("Frame CG behind the front axle", fresh && f.cg ? (f.cg[0] * 1e3).toFixed(0) : "…", "mm"), tile("Frame CG height", fresh && f.cg ? (f.cg[2] * 1e3).toFixed(0) : "…", "mm"),
      tile("Tubes", String(s.tubes.length), "", `${stats.length.toFixed(2)} m of tube · ${s.nodes.length} nodes`),
      f.car ? tile("Car with this frame", fmt(f.car.mass_kg, 4), "kg", `CG ${fmt(f.car.cg_height_mm, 4)} mm · ${fmt(f.car["front_weight_%"], 3)} % front`) : null);
    const msgs = [];
    if (fresh && f.stiffness_error) msgs.push("Torsional stiffness could not be solved: " + f.stiffness_error);
    if (fresh && f.pieces > 1) msgs.push(`The frame is in ${f.pieces} separate pieces: a moved or deleted tube no longer reaches its joint.`);
    if (!S.live && !fresh) msgs.push("Turn on Live preview (top bar) to recompute stiffness and CG as you edit.");
    warn.replaceChildren(...msgs.map((m) => el("div", { class: "note warn" }, m)));
  }
  ctx.refresh = () => { showTiles(); if (FR.colour === "energy") ed.draw(); };
  ctx.local = (structural) => { stats = frameStats(st()); if (structural) mir = frameMirrorMap(st()); showTiles(); ed.draw(); bar.update(); tubeBar(); };
  tools(); showTiles(); tubeBar();
  b.append(tiles, warn, toolbar, ed.node, bar.node, tb);
}, { wide: true, blocks: ["frame"], first: true });
/* Remove a joint: the tubes that end on it go with it, and the joints after it are renumbered. */
function frameDeleteNode(n) {
  const s = FR.state;
  for (let i = s.tubes.length - 1; i >= 0; i--) if (s.tubes[i].n.includes(n)) frameDeleteTube(i);
  s.nodes.splice(n, 1);
  for (const t of s.tubes) t.n = t.n.map((k) => (k > n ? k - 1 : k));
}
/* Put a joint at the middle of a tube: the tube becomes two, and anything coped onto it stays where it was. */
function frameSplitTube(i) {
  const s = FR.state, P = frameEnds(s)[i], t = s.tubes[i], mid = [0, 1, 2].map((a) => 0.5 * (P[0][a] + P[1][a]));
  s.nodes.push(mid); const n = s.nodes.length - 1, j = s.tubes.length;
  s.tubes.push({ n: [n, t.n[1]], off: [[0, 0, 0], t.off[1]], host: [null, t.host[1]], size: t.size, bend: t.bend });
  t.n = [t.n[0], n]; t.off = [t.off[0], [0, 0, 0]]; t.host = [t.host[0], null];
  for (const u of s.tubes) for (const k of [0, 1]) if (u.host[k] && u.host[k][0] === i) u.host[k] = u.host[k][1] < 0.5 ? [i, u.host[k][1] * 2] : [j, (u.host[k][1] - 0.5) * 2];
  return n;
}
function frameDeleteTube(i) {
  const s = FR.state, P = frameEnds(s);
  s.tubes.forEach((t, j) => { for (const k of [0, 1]) {
    if (!t.host[k]) continue;
    if (t.host[k][0] === i) { s.nodes.push(P[j][k].slice()); t.n[k] = s.nodes.length - 1; t.off[k] = [0, 0, 0]; t.host[k] = null; }
    else if (t.host[k][0] > i) t.host[k] = [t.host[k][0] - 1, t.host[k][1]];
  } });
  s.tubes.splice(i, 1);
}
const sizeName = (z, k) => `${k + 1}: ${z.shape} ${(z.od * 1e3).toFixed(1)} × ${(z.wall * 1e3).toFixed(2)} mm`;

V("chassis", "frame3d", "Frame in 3-D", "Drag to rotate. Follows the editor.", (b, ctx) => {
  need(GEO().frame); const p = plotDiv("tall"); b.append(p);
  ctx.local = ctx.refresh = () => {
    const st = FR.state, P = frameEnds(st), en = (SUB().frame || {}).energy, groups = new Map();
    P.forEach((q, i) => { if (!q[0] || !q[1]) return; const c = tubeColor(st, i, en); if (!groups.has(c)) groups.set(c, { x: [], y: [], z: [] }); const g = groups.get(c); g.x.push(q[0][0], q[1][0], null); g.y.push(q[0][1], q[1][1], null); g.z.push(q[0][2], q[1][2], null); });
    const ax = { showgrid: true, gridcolor: "#eef1f6", zeroline: false, showticklabels: false, title: "" };
    Plotly.react(p, [...groups].map(([c, g]) => ({ type: "scatter3d", mode: "lines", x: g.x, y: g.y, z: g.z, line: { color: c, width: 5 }, hoverinfo: "skip", showlegend: false })),
      { margin: { l: 0, r: 0, t: 0, b: 0 }, paper_bgcolor: "#fff", uirevision: "frame", scene: { aspectmode: "data", xaxis: ax, yaxis: ax, zaxis: ax, camera: { eye: { x: -1.5, y: -1.6, z: 0.8 } } } },
      { responsive: true, displaylogo: false, displayModeBar: false });
  };
  ctx.local();
}, { blocks: ["frame"] });

V("chassis", "frame_sizes", "Tube sizes", "Each stock size used in the frame. Change the section or the material and every tube of that size follows.", (b, ctx) => {
  need(GEO().frame);
  const wrap = el("div");
  ctx.local = () => {
    const st = FR.state, stt = frameStats(st), base = FR.base();
    const mm = (v) => +(v * 1e3).toFixed(3);
    const rows = st.sizes.map((z, k) => {
      const changed = JSON.stringify(z) !== JSON.stringify(base.sizes[k]);
      const shape = el("select", { onchange: () => { z.shape = shape.value; frameCommit(true); } }, el("option", { value: "round" }, "round"), el("option", { value: "square" }, "square")); shape.value = z.shape;
      const mat = el("select", { onchange: () => { z.material = mat.value; frameCommit(true); } }, Object.keys(st.materials).map((m) => el("option", { value: m }, m))); mat.value = z.material;
      return el("tr", { class: changed ? "changed" : "" }, el("td", {}, String(k + 1)), el("td", {}, shape),
        el("td", {}, numIn(mm(z.od), (v, i) => { if (v <= 2 * z.wall * 1e3) { i.value = mm(z.od); return alert("The outer size must be more than twice the wall."); } z.od = v / 1e3; frameCommit(true); }, { min: 1, style: "width:84px", "aria-label": "outer size mm" })),
        el("td", {}, numIn(mm(z.od / 2), (v, i) => { if (v <= z.wall * 1e3) { i.value = mm(z.od / 2); return alert("The radius must be more than the wall."); } z.od = 2 * v / 1e3; frameCommit(true); }, { min: 0.5, style: "width:84px", "aria-label": "radius mm" })),
        el("td", {}, numIn(mm(z.wall), (v, i) => { if (v <= 0 || v >= z.od * 500) { i.value = mm(z.wall); return alert("The wall must be between 0 and half the outer size."); } z.wall = v / 1e3; frameCommit(true); }, { min: 0.1, style: "width:78px", "aria-label": "wall mm" })),
        el("td", {}, mat), el("td", {}, String(stt.sizes[k].count)),
        el("td", {}, numIn(+stt.sizes[k].length.toFixed(3), (v, i) => { const now = stt.sizes[k].length; if (!(v > 0) || !now) { i.value = now.toFixed(3); return; } frameScaleSize(k, v / now); frameCommit(true); }, { min: 0.01, style: "width:84px", disabled: stt.sizes[k].count ? null : true, "aria-label": "total length m" })),
        el("td", {}, stt.sizes[k].mass.toFixed(2)), el("td", {}, (1e3 * tubeArea(z) * (st.materials[z.material] || {}).density).toFixed(0)));
    });
    wrap.replaceChildren(el("div", { class: "tablewrap" }, el("table", { class: "edit" },
      el("thead", {}, el("tr", {}, ["#", "section", "outer size [mm]", "radius [mm]", "wall [mm]", "material", "tubes", "total length [m]", "mass [kg]", "g per m"].map((h) => el("th", {}, h)))), el("tbody", {}, rows))),
      el("div", { class: "ctl" }, el("button", { class: "btn small", onclick: () => { st.sizes.push({ shape: "round", od: 0.0254, wall: 0.0009, material: "4130 steel" }); frameCommit(true); } }, "Add a size"),
        el("span", { class: "muted" }, "Total length scales every tube of that size about its first node. Single tube lengths are in the Tubes table.")));
  };
  b.append(wrap); ctx.local();
}, { wide: true });
/* Scale every tube of one size about its first node (lengths adjustable per size). */
function frameScaleSize(k, f) {
  const st = FR.state, moved = new Set();
  st.tubes.forEach((t) => { if (t.size !== k || t.n[0] < 0 || t.n[1] < 0 || moved.has(t.n[1])) return;
    const a = st.nodes[t.n[0]], c = st.nodes[t.n[1]]; st.nodes[t.n[1]] = [0, 1, 2].map((x) => a[x] + f * (c[x] - a[x])); moved.add(t.n[1]); });
}

V("chassis", "frame_mats", "Materials", "Stiffness and density used by the beam model. Edit a value or add a material, then pick it in Tube sizes.", (b, ctx) => {
  need(GEO().frame);
  const wrap = el("div");
  ctx.local = () => {
    const st = FR.state, used = new Set(st.sizes.map((z) => z.material));
    const rows = Object.entries(st.materials).map(([name, m]) => el("tr", {}, el("td", {}, name),
      el("td", {}, numIn(m.E / 1e9, (v) => { m.E = v * 1e9; frameCommit(true); }, { min: 1, style: "width:84px" })), el("td", {}, numIn(m.G / 1e9, (v) => { m.G = v * 1e9; frameCommit(true); }, { min: 0.5, style: "width:84px" })),
      el("td", {}, numIn(m.density, (v) => { m.density = v; frameCommit(true); }, { min: 100, style: "width:90px" })), el("td", {}, (m.E / m.density / 1e6).toFixed(1)), el("td", {}, used.has(name) ? "in use" : "")));
    const nm = el("input", { type: "text", placeholder: "New material name", style: "width:190px" });
    wrap.replaceChildren(el("div", { class: "tablewrap" }, el("table", { class: "edit" }, el("thead", {}, el("tr", {}, ["material", "E [GPa]", "G [GPa]", "density [kg/m³]", "E / density [MN·m/kg]", ""].map((h) => el("th", {}, h)))), el("tbody", {}, rows))),
      el("div", { class: "ctl" }, nm, el("button", { class: "btn small", onclick: () => { const n = nm.value.trim(); if (!n || st.materials[n]) return; st.materials[n] = { E: 205e9, G: 80e9, density: 7850 }; frameCommit(true); } }, "Add material")));
  };
  b.append(wrap); ctx.local();
});

V("chassis", "frame_tubes", "Tubes", "Every tube: length, size, mass and its share of the strain energy when the frame is twisted.", (b, ctx) => {
  need(GEO().frame);
  const wrap = el("div");
  ctx.refresh = ctx.local = () => {
    const st = FR.state, stt = frameStats(st), f = SUB().frame || {}, en = f.energy && f.energy.length === st.tubes.length ? f.energy : null;
    const rows = st.tubes.map((t, i) => {
      const canLen = t.n[0] >= 0 || t.n[1] >= 0;
      const sel = el("select", { onchange: () => { t.size = +sel.value; frameCommit(true); } }, st.sizes.map((z, k) => el("option", { value: k }, sizeName(z, k)))); sel.value = t.size;
      const end = (k) => (t.n[k] >= 0 ? `node ${t.n[k] + 1}` : `on tube ${t.host[k][0] + 1}`);
      return el("tr", { class: FR.selTube === i ? "selrow" : "", onclick: (e) => { if (e.target.closest("input,select,button")) return; FR.selTube = FR.selTube === i ? null : i; const c = CARDS.get("frame_edit"); if (c) c.ctx.local(false); ctx.local(); } },
        el("td", {}, String(i + 1)), el("td", {}, `${end(0)} → ${end(1)}`),
        el("td", {}, numIn(+(stt.len[i] * 1e3).toFixed(1), (v, inp) => { if (!(v > 1)) { inp.value = (stt.len[i] * 1e3).toFixed(1); return; } frameSetLength(i, v / 1e3); frameCommit(true); }, { min: 1, style: "width:88px", disabled: canLen ? null : true, "aria-label": `tube ${i + 1} length mm` })),
        el("td", {}, sel), el("td", {}, stt.mass[i].toFixed(3)), el("td", {}, en ? (100 * en[i]).toFixed(2) : "–"),
        el("td", {}, el("button", { class: "btn small", title: "Delete this tube", onclick: () => { frameDeleteTube(i); FR.selTube = null; frameCommit(true); } }, "✕")));
    });
    wrap.replaceChildren(el("div", { class: "tablewrap" }, el("table", { class: "edit" }, el("thead", {}, el("tr", {}, ["#", "between", "length [mm]", "size", "mass [kg]", "twist energy [%]", ""].map((h) => el("th", {}, h)))), el("tbody", {}, rows))));
  };
  b.append(wrap); ctx.local();
}, { wide: true, blocks: ["frame"], off: true });
/* Change one tube's length by moving a node along the tube's axis (the far node if it has one). */
function frameSetLength(i, L) {
  const st = FR.state, t = st.tubes[i], P = frameEnds(st)[i], now = dist3(P[0], P[1]), k = t.n[1] >= 0 ? 1 : 0;
  const dir = [0, 1, 2].map((a) => (P[1][a] - P[0][a]) / now * (k === 1 ? 1 : -1)), n = st.nodes[t.n[k]];
  st.nodes[t.n[k]] = [0, 1, 2].map((a) => n[a] + dir[a] * (L - now));
}

// ------------------------------------------------------------------ suspension hard points
const HP = { axle: "front", onFrame: true, showFrame: true };
/* The frame in the hard-point export's axes (x forward, y left, z up): car axes are x rearward, y right. */
const frameInHpAxes = (st) => frameEnds(st).filter((p) => p[0] && p[1]).map((p) => p.map((q) => [-q[0], -q[1], q[2]]));
function nearestOnSegs(p, segs) {
  let best = [Infinity, p];
  for (const [a, b] of segs) {
    const d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]], L2 = d[0] ** 2 + d[1] ** 2 + d[2] ** 2 || 1e-12;
    const t = Math.min(1, Math.max(0, ((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1] + (p[2] - a[2]) * d[2]) / L2)), q = [a[0] + t * d[0], a[1] + t * d[1], a[2] + t * d[2]], dd = dist3(p, q);
    if (dd < best[0]) best = [dd, q];
  }
  return best;
}
/* How far a chassis-side point may sit from the nearest frame tube: its baseline tab length plus 5 mm, at least 30 mm. */
function hpAllowed(axle, name) {
  const base = GEO().hardpoints[axle].points[name];
  return Math.max(0.03, nearestOnSegs(base, frameInHpAxes(FR.base()))[0] + 0.005);
}
/* Keep a chassis-side point on the frame: pulled back towards the nearest tube when it is dragged too far off it. */
function hpOnFrame(axle, name, p) {
  if (!HP.onFrame || !name.startsWith("CHAS_") || !GEO().frame || GEO().frame.error) return p;
  const [d, q] = nearestOnSegs(p, frameInHpAxes(FR.state)), lim = hpAllowed(axle, name);
  return d <= lim ? p : [0, 1, 2].map((a) => q[a] + (p[a] - q[a]) * lim / d);
}
const HP_COLORS = { arm: "#000E2F", upright: "#47526a", tie: "#eb6834", rod: "#1baf7a", rocker: "#6250d6", damper: "#eda100", arb: "#e87ba4" };
const HP_LABELS = { arm: "A-arms", upright: "upright", tie: "tie rod", rod: "pull rod", rocker: "rocker", damper: "damper", arb: "anti-roll bar link" };
function hpPoints(axle) {
  const base = GEO().hardpoints[axle].points, ed = ((OV()["geom:hardpoints"] || {})[axle]) || {};
  return Object.fromEntries(Object.keys(base).map((k) => [k, ed[k] || base[k]]));
}
function hpSet(axle, name, p) {
  const all = OV()["geom:hardpoints"] ? clone(OV()["geom:hardpoints"]) : {}, base = GEO().hardpoints[axle].points[name];
  all[axle] = all[axle] || {};
  if (dist3(p, base) < 1e-6) delete all[axle][name]; else all[axle][name] = p.map((v) => +v.toFixed(6));
  if (!Object.keys(all[axle]).length) delete all[axle];
  if (Object.keys(all).length) OV()["geom:hardpoints"] = all; else delete OV()["geom:hardpoints"];
}
V("suspension", "hp_edit", "Hard-point editor", "Drag a suspension point in any view (left corner; the right side mirrors it). Kinematics are re-solved from the new geometry.", (b, ctx) => {
  const G = need(GEO().hardpoints); if (!G[HP.axle]) HP.axle = "front";
  const tiles = el("div", { class: "kpis" }), holder = el("div"), warn = el("div");
  let ed, bar;
  const build = () => {
    const axle = HP.axle, w = G[axle].wheel, R = w.tire_diameter / 2;
    ed = viewEditor({
      views: [{ title: "Side view (front of the car on the left)", ax: [0, 2], sign: [-1, 1], names: ["x", "z"] }, { title: "Top view", ax: [0, 1], sign: [-1, -1], names: ["x", "y"] }, { title: "Front view (left corner, outboard on the right)", ax: [1, 2], sign: [1, 1], names: ["y", "z"] }],
      points: () => Object.entries(hpPoints(axle)).map(([k, p]) => ({ id: k, p, label: `${G.names[k] || k} (${k})`, color: ((OV()["geom:hardpoints"] || {})[axle] || {})[k] ? "#eb6834" : "#2a78d6" })),
      segs: () => { const P = hpPoints(axle), out = [];
        if (HP.showFrame && GEO().frame && !GEO().frame.error) for (const [a, c] of frameInHpAxes(FR.state)) out.push({ a, b: c, color: "rgba(0,14,47,.16)", w: 3.2, nofit: true });
        for (const [a, c, kind] of G.links) if (P[a] && P[c]) out.push({ a: P[a], b: P[c], color: HP_COLORS[kind], w: kind === "arm" ? 3 : 2.4 });
        const st = (((SUB().suspension || {}).kinematics || {})[axle] || {}).static;
        if (st && st.fv_ic_y != null && P.UPRI_LowPnt) {                   // front-view instant centre and roll-centre line (from the solver)
          const x = P.UPRI_LowPnt[0], ic = [x, st.fv_ic_y, st.fv_ic_z], cp = [x, w.half_track, 0];
          if (Math.abs(st.fv_ic_y) < 6) out.push({ a: cp, b: ic, color: "#8a9099", w: 1.2, dash: true, nofit: true, only: "Front view (left corner, outboard on the right)" }, { a: P.UPRI_LowPnt, b: ic, color: "#b9c0cc", w: 1, dash: true, nofit: true, only: "Front view (left corner, outboard on the right)" }, { a: P.UPRI_UppPnt, b: ic, color: "#b9c0cc", w: 1, dash: true, nofit: true, only: "Front view (left corner, outboard on the right)" });
        }
        return out; },
      extent: () => [[w.x_offset - R, 0, 0], [w.x_offset + R, w.half_track + w.tire_width / 2, 2 * R], [w.x_offset, 0, 0]],
      decor: (v, proj, px) => {
        const sw = 1.4 * px, st = (((SUB().suspension || {}).kinematics || {})[axle] || {}).static || {};
        const rect = (p, q) => { const a = proj(p), c = proj(q); return `<rect x="${Math.min(a[0], c[0])}" y="${Math.min(a[1], c[1])}" width="${Math.abs(c[0] - a[0])}" height="${Math.abs(c[1] - a[1])}" rx="30" class="tire"/>`; };
        const ground = `<line x1="-99999" x2="99999" y1="0" y2="0" stroke="#7a859c" stroke-width="${sw}"/>`;
        if (v.ax[0] === 1 && v.ax[1] === 2) {             // front view: tire section, ground, centre line, roll centre
          const rc = st.rc_height != null ? proj([0, 0, st.rc_height]) : null;
          return rect([0, w.half_track - w.tire_width / 2, 0], [0, w.half_track + w.tire_width / 2, 2 * R]) + ground
            + `<line x1="0" x2="0" y1="-99999" y2="99999" stroke="#7a859c" stroke-width="${sw}" stroke-dasharray="${6 * px} ${4 * px}"/>`
            + (rc ? `<circle cx="${rc[0]}" cy="${rc[1]}" r="${5 * px}" class="rc"/><text x="${rc[0] + 9 * px}" y="${rc[1] - 7 * px}" font-size="${11.5 * px}" class="lbl">roll centre ${(st.rc_height * 1e3).toFixed(0)} mm</text>` : "");
        }
        if (v.ax[0] === 0 && v.ax[1] === 2) { const [cx, cy] = proj([w.x_offset, 0, R]); return `<circle cx="${cx}" cy="${cy}" r="${R * 1e3}" class="tire"/>` + ground; }
        return rect([w.x_offset - R, w.half_track - w.tire_width / 2, 0], [w.x_offset + R, w.half_track + w.tire_width / 2, 0]);
      },
      onMove: (id, p) => { hpSet(axle, id, hpOnFrame(axle, id, p)); bar.update(); pending(); hpTables(); },
      onSelect: () => bar.update(),
    });
    bar = coordBar(["x (forward)", "y (outboard)", "z (up)"], () => (ed.selected == null ? null : { label: G.names[ed.selected] || ed.selected, p: hpPoints(axle)[ed.selected],
      extra: [el("button", { class: "btn small", onclick: () => { hpSet(axle, ed.selected, GEO().hardpoints[axle].points[ed.selected]); ed.draw(); bar.update(); pending(); } }, "Reset point")] }),
      (p) => { hpSet(axle, ed.selected, hpOnFrame(axle, ed.selected, p)); ed.draw(); bar.update(); pending(); hpTables(); });
    holder.replaceChildren(ed.node, bar.node);
  };
  const tabs = el("div", { class: "ctl" });
  const drawTabs = () => tabs.replaceChildren(
    el("div", { class: "seg light" }, ["front", "rear"].filter((a) => G[a]).map((a) => el("button", { class: HP.axle === a ? "on" : "", onclick: () => { HP.axle = a; drawTabs(); build(); showTiles(); hpTables(); } }, a === "front" ? "Front axle" : "Rear axle"))),
    el("button", { class: "btn small", onclick: () => ed.fit() }, "Fit views"),
    el("label", { class: "check", title: "Chassis-side points (arm pivots, rocker pivot, damper mount, ARB pivot) cannot be dragged off the frame tubes" }, el("input", { type: "checkbox", checked: HP.onFrame, onchange: (e) => { HP.onFrame = e.target.checked; } }), "Keep chassis points on the frame"),
    el("label", { class: "check" }, el("input", { type: "checkbox", checked: HP.showFrame, onchange: (e) => { HP.showFrame = e.target.checked; ed.draw(); } }), "Show the frame"),
    el("button", { class: "btn small", onclick: () => { const all = clone(OV()["geom:hardpoints"] || {}); delete all[HP.axle]; if (Object.keys(all).length) OV()["geom:hardpoints"] = all; else delete OV()["geom:hardpoints"]; build(); pending(); hpTables(); } }, `Reset ${HP.axle} points`),
    el("span", { class: "legend" }, Object.entries(HP_LABELS).map(([k, l]) => el("span", {}, el("i", { style: `background:${HP_COLORS[k]}` }), l))));
  function showTiles() {
    const k = ((SUB().suspension || {}).kinematics || {})[HP.axle] || {}, s = k.static;
    const b0 = (((S.base[S.car].subsystems.suspension || {}).kinematics || {})[HP.axle] || {}).static || {};
    if (k.error || !s) { tiles.replaceChildren(); warn.replaceChildren(el("div", { class: "note warn" }, k.error ? "This geometry could not be solved: " + k.error : "Kinematics are not available.")); return; }
    warn.replaceChildren(...(!S.live && OV()["geom:hardpoints"] ? [el("div", { class: "note warn" }, "Turn on Live preview (top bar) to re-solve the kinematics as you drag.")] : []));
    const T = (name, key, k2, unit, dg) => tile(name, (s[key] * k2).toFixed(dg), unit, delta(s[key] * k2, b0[key] != null ? b0[key] * k2 : null, dg, ""));
    tiles.replaceChildren(T("Roll-centre height", "rc_height", 1e3, "mm", 1), T("Camber gain", "camber_gain_deg_per_mm", 25.4, "deg/in", 3), T("Swing-arm length", "fvsa", 1e3, "mm", 0),
      T("KPI", "kpi_deg", 1, "deg", 2), T("Caster", "caster_deg", 1, "deg", 2), T("Scrub radius", "scrub_radius", 1e3, "mm", 1), T("Mechanical trail", "mech_trail", 1e3, "mm", 1),
      T("Motion ratio, wheel / damper", "motion_ratio_wheel_per_damper", 1, "", 3), T("Bump steer", "bump_steer_deg_per_mm", 25.4, "deg/in", 3), T("Pull-rod angle from vertical", "pull_rod_angle_from_vertical_deg", 1, "deg", 1));
  }
  ctx.refresh = () => { showTiles(); ed.draw(); };
  ctx.local = () => { ed.draw(); bar.update(); };
  drawTabs(); build(); showTiles();
  b.append(tiles, warn, tabs, holder);
}, { wide: true, blocks: ["suspension"], first: true });

// ------------------------------------------------------------------ car layout (side view): CG, wheelbase, aero centres
function layoutCard(aero) {
  return (b, ctx) => {
    const K = { m: "vehicle:mass.", g: "vehicle:geometry.", a: "vehicle:aero.components." };
    const L = () => cur(K.g + "wheelbase"), R = () => cur("vehicle:tires.tire_radius") || 0.203;
    const info = el("div", { class: "coordbar" });
    const wings = aero ? ["front_wing", "rear_wing"].filter((w) => typeof cur(`${K.a}${w}.x_from_front_axle`) === "number") : [];
    const pts = () => {
      const out = [{ id: "cg", p: [(1 - cur(K.m + "front_weight_fraction")) * L(), 0, cur(K.m + "cg_height")], label: "centre of gravity", color: "#eb6834", fixed: aero },
        { id: "rear", p: [L(), 0, R()], label: aero ? "rear axle" : "rear axle (drag to change the wheelbase)", color: "#2a78d6", fixed: aero }, { id: "front", p: [0, 0, R()], label: "front axle", fixed: true }];
      for (const w of wings) out.push({ id: w, p: [-cur(`${K.a}${w}.x_from_front_axle`), 0, cur(`${K.a}${w}.z`)], label: w.replace("_", " ") + " centre of pressure", color: "#1baf7a" });
      return out;
    };
    const say = () => info.replaceChildren(el("span", {}, `Wheelbase ${(L() * 1e3).toFixed(0)} mm`), el("span", {}, `CG height ${(cur(K.m + "cg_height") * 1e3).toFixed(0)} mm`),
      el("span", {}, `${(100 * cur(K.m + "front_weight_fraction")).toFixed(1)} % front`), ...wings.map((w) => el("span", {}, `${w.replace("_", " ")}: ${(-cur(`${K.a}${w}.x_from_front_axle`) * 1e3).toFixed(0)} mm behind the front axle, ${(cur(`${K.a}${w}.z`) * 1e3).toFixed(0)} mm high`)));
    const ed = viewEditor({
      views: [{ title: aero ? "Side view (front of the car on the left). Green: wing centres of pressure, drag them." : "Side view (front of the car on the left). Orange: centre of gravity. Blue: rear axle.", ax: [0, 2], sign: [1, 1] }],
      points: pts,
      segs: () => { const p = Object.fromEntries(pts().map((q) => [q.id, q.p])), out = [{ a: [0, 0, R()], b: [L(), 0, R()], color: "#000E2F", w: 3 }, { a: [p.cg[0], 0, 0], b: p.cg, color: "#eb6834", w: 1.2, dash: true }];
        for (const w of wings) out.push({ a: [p[w][0] - 0.22, 0, p[w][2]], b: [p[w][0] + 0.22, 0, p[w][2]], color: "#1baf7a", w: 4 });
        return out; },
      extent: () => (aero ? [[-0.95, 0, 0], [L() + 0.45, 0, 1.15]] : [[-0.35, 0, 0], [L() + 0.35, 0, 0.62]]),
      decor: (v, proj, px) => { const f = proj([0, 0, R()]), r = proj([L(), 0, R()]);
        return `<circle cx="${f[0]}" cy="${f[1]}" r="${R() * 1e3}" class="tire"/><circle cx="${r[0]}" cy="${r[1]}" r="${R() * 1e3}" class="tire"/><line x1="-99999" x2="99999" y1="0" y2="0" stroke="#7a859c" stroke-width="${1.4 * px}"/>`; },
      onMove: (id, p) => {
        if (id === "cg") { setOv(K.m + "front_weight_fraction", +Math.min(0.75, Math.max(0.25, 1 - p[0] / L())).toFixed(4), { quiet: true }); setOv(K.m + "cg_height", +Math.max(0.1, p[2]).toFixed(4), { quiet: true }); }
        else if (id === "rear") setOv(K.g + "wheelbase", +Math.max(1.525, Math.min(2.2, p[0])).toFixed(4), { quiet: true });
        else { setOv(`${K.a}${id}.x_from_front_axle`, +(-p[0]).toFixed(4), { quiet: true }); setOv(`${K.a}${id}.z`, +Math.max(0.02, p[2]).toFixed(4), { quiet: true }); }
        say(); pending();
      },
    });
    ctx.refresh = () => { ed.draw(); say(); };
    say(); b.append(ed.node, info);
  };
}
V("chassis", "layout", "Car layout", "Drag the centre of gravity or the rear axle. The rules minimum wheelbase is 1525 mm.", layoutCard(false), { wide: true, cls: "low", first: true });
V("aero", "aero_layout", "Where the aero loads act", "Drag a wing's centre of pressure fore-aft or up and down: it changes the aero balance and the pitching moment from drag.", layoutCard(true), { wide: true, cls: "low", blocks: ["aero"], first: true });

// ------------------------------------------------------------------ draggable curve (IC volumetric efficiency) and gear ratios
function curveEditor({ xs, ys, yMin, yMax, xLabel, yLabel, onChange, base }) {
  const W = 640, H = 300, m = { l: 48, r: 12, t: 12, b: 38 }, s = svg("svg", { viewBox: `0 0 ${W} ${H}`, class: "curve" });
  const X = (x) => m.l + (x - xs[0]) / (xs[xs.length - 1] - xs[0]) * (W - m.l - m.r), Y = (y) => H - m.b - (y - yMin) / (yMax - yMin) * (H - m.t - m.b);
  const draw = () => {
    let h = "";
    for (let k = 0; k <= 5; k++) { const y = yMin + k * (yMax - yMin) / 5; h += `<line x1="${m.l}" x2="${W - m.r}" y1="${Y(y)}" y2="${Y(y)}" class="grid"/><text x="${m.l - 6}" y="${Y(y) + 4}" text-anchor="end" class="lbl">${y.toFixed(2)}</text>`; }
    xs.forEach((x, i) => { if (i % 2 === 0) h += `<text x="${X(x)}" y="${H - m.b + 16}" text-anchor="middle" class="lbl">${x}</text>`; });
    h += `<text x="${(W + m.l) / 2}" y="${H - 4}" text-anchor="middle" class="lbl">${xLabel}</text><text transform="translate(12 ${H / 2}) rotate(-90)" text-anchor="middle" class="lbl">${yLabel}</text>`;
    if (base) h += `<polyline points="${xs.map((x, i) => `${X(x)},${Y(base[i])}`).join(" ")}" fill="none" stroke="#b9c0cc" stroke-width="1.5" stroke-dasharray="5 4"/>`;
    h += `<polyline points="${xs.map((x, i) => `${X(x)},${Y(ys[i])}`).join(" ")}" fill="none" stroke="#2a78d6" stroke-width="2"/>`;
    s.innerHTML = h;
    xs.forEach((x, i) => {
      const g = svg("g", { class: "dot-g", transform: `translate(${X(x)} ${Y(ys[i])})` }); g.append(svg("circle", { class: "hit", r: 16 }), svg("circle", { class: "pt", r: 5.5, fill: "#2a78d6", "stroke-width": 1.6 }));
      const t = svg("title"); t.textContent = `${x} ${xLabel}: ${ys[i].toFixed(3)}`; g.append(t);
      let start = null;
      g.addEventListener("pointerdown", (e) => { g.setPointerCapture(e.pointerId); start = { y: e.clientY, v: ys[i] }; e.preventDefault(); });
      const mv = (e, done) => { if (!start) return; const c = s.getScreenCTM(); ys[i] = Math.min(yMax, Math.max(yMin, start.v - (e.clientY - start.y) / c.d / (H - m.t - m.b) * (yMax - yMin)));
        g.setAttribute("transform", `translate(${X(x)} ${Y(ys[i])})`); s.querySelectorAll("polyline")[base ? 1 : 0].setAttribute("points", xs.map((xx, k) => `${X(xx)},${Y(ys[k])}`).join(" ")); if (done) { start = null; onChange(ys.map((v) => +v.toFixed(4))); } };
      g.addEventListener("pointermove", (e) => mv(e, false)); g.addEventListener("pointerup", (e) => mv(e, true)); g.addEventListener("pointercancel", (e) => mv(e, true));
      s.append(g);
    });
  };
  draw();
  return s;
}
V("powertrain", "ve_edit", "Volumetric efficiency", "Drag the points to reshape how well the engine breathes at each speed. The torque and power curves follow. Dashed: baseline.", (b) => {
  const xs = cur("pt:engine.ve_rpm"), base = inputOf("pt:engine.ve").value, ys = (cur("pt:engine.ve") || base).slice();
  if (!Array.isArray(xs) || !Array.isArray(ys)) throw new Error("the engine has no volumetric-efficiency table");
  b.append(curveEditor({ xs, ys, base, yMin: 0.4, yMax: 1.15, xLabel: "engine speed [rpm]", yLabel: "volumetric efficiency", onChange: (v) => setOv("pt:engine.ve", v) }),
    el("div", { class: "ctl" }, el("button", { class: "btn small", onclick: () => { setOv("pt:engine.ve", base.slice()); render(); } }, "Reset curve")));
}, { car: "ic", first: true });
V("powertrain", "gear_edit", "Gear ratios", "One slider per gear. Tractive force by gear and the shift points follow.", (b) => {
  const base = inputOf("pt:gearbox.gear_ratios").value, now = (cur("pt:gearbox.gear_ratios") || base).slice();
  const rows = now.map((r, i) => {
    const range = el("input", { type: "range", min: 0.8, max: 3.2, step: 0.001, value: r, "aria-label": `gear ${i + 1} ratio` }), box = el("input", { type: "number", step: 0.001, value: r.toFixed(3) });
    const set = (v) => { if (!(v > 0)) return; now[i] = +(+v).toFixed(3); range.value = now[i]; if (document.activeElement !== box) box.value = now[i].toFixed(3); row.classList.toggle("changed", Math.abs(now[i] - base[i]) > 1e-9); setOv("pt:gearbox.gear_ratios", now.slice()); };
    range.addEventListener("input", () => set(range.value)); box.addEventListener("change", () => set(box.value));
    const row = el("div", { class: "quick" + (Math.abs(r - base[i]) > 1e-9 ? " changed" : "") }, el("div", { class: "qlabel" }, el("span", {}, `Gear ${i + 1}`), el("span", { class: "qbase" }, `baseline ${base[i].toFixed(3)}`)), range, el("div", { class: "qval" }, box, el("span", { class: "u" }, ": 1")));
    return row;
  });
  b.append(el("div", { class: "quickgrid one" }, rows), el("div", { class: "ctl" }, el("button", { class: "btn small", onclick: () => { setOv("pt:gearbox.gear_ratios", base.slice()); render(); } }, "Reset ratios")));
}, { car: "ic", first: true });

// ------------------------------------------------------------------ event tracks
V("overview", "trackpick", "Event tracks", "Which centre line each event is driven on. Press Run simulation after changing one.", (b) => {
  const files = need(GEO().tracks), choice = Object.assign({}, GEO().track_choice, OV()["geom:tracks"] || {});
  const row = (e) => {
    const opts = files.filter((f) => f.startsWith(e));
    const sel = el("select", { "aria-label": e + " track", onchange: () => { const t = Object.assign({}, OV()["geom:tracks"] || {}); if (sel.value === GEO().track_choice[e]) delete t[e]; else t[e] = sel.value;
      if (Object.keys(t).length) OV()["geom:tracks"] = t; else delete OV()["geom:tracks"]; pending(); } }, opts.map((f) => el("option", { value: f }, f.replace(/\.csv$/, "").replace(/_/g, " "))));
    sel.value = choice[e];
    return el("div", { class: "ctl" }, el("b", { style: "min-width:90px" }, EVENT_NAMES[e]), sel);
  };
  b.append(row("autocross"), row("endurance"), el("div", { class: "sub" }, "Acceleration is a straight of the length set in Quick adjust; skidpad is the rules circle."));
});

// Materials sits beside the 3-D view (two half-width cards), ahead of the full-width tables.
{ const i = VISUALS.findIndex((v) => v.id === "frame_mats"), m = VISUALS.splice(i, 1)[0]; VISUALS.splice(VISUALS.findIndex((v) => v.id === "frame3d") + 1, 0, m); }


// ------------------------------------------------------------------ numbers for every joint
const hpTables = () => { for (const id of ["hp_table", "hp_links"]) { const c = CARDS.get(id); if (c && c.ctx.local) c.ctx.local(); } };
const hpRefresh = () => { const c = CARDS.get("hp_edit"); if (c && c.ctx.local) c.ctx.local(); hpTables(); pending(); };
V("suspension", "hp_table", "Hard points by number", "Every suspension point of the axle chosen in the editor, in millimetres (x forward from the front axle, y outboard, z up). Type a value to move the point.", (b, ctx) => {
  const G = need(GEO().hardpoints), wrap = el("div"); b.append(wrap);
  ctx.local = () => {
    const axle = HP.axle, P = hpPoints(axle), ed = ((OV()["geom:hardpoints"] || {})[axle]) || {};
    const rows = Object.entries(P).map(([k, p]) => el("tr", { class: ed[k] ? "changed" : "" }, el("td", {}, G.names[k] || k), el("td", { class: "muted" }, k),
      ...[0, 1, 2].map((a) => el("td", {}, numIn(+(p[a] * 1e3).toFixed(3), (v) => { const q = hpPoints(axle)[k].slice(); q[a] = v / 1e3; hpSet(axle, k, hpOnFrame(axle, k, q)); hpRefresh(); }, { style: "width:96px", "aria-label": `${k} ${"xyz"[a]} mm` }))),
      el("td", {}, ed[k] ? el("button", { class: "btn small", onclick: () => { hpSet(axle, k, GEO().hardpoints[axle].points[k]); hpRefresh(); } }, "Reset") : "")));
    wrap.replaceChildren(el("div", { class: "sub" }, `${axle === "front" ? "Front" : "Rear"} axle, left corner. The right corner mirrors it.`),
      el("div", { class: "tablewrap" }, el("table", { class: "edit" }, el("thead", {}, el("tr", {}, ["point", "name in the export", "x [mm]", "y [mm]", "z [mm]", ""].map((h) => el("th", {}, h)))), el("tbody", {}, rows))));
  };
  ctx.local();
}, { wide: true });
V("suspension", "hp_links", "Link lengths", "Each member's length between its two joints. Type a length and its second joint moves along the member.", (b, ctx) => {
  const G = need(GEO().hardpoints), wrap = el("div"); b.append(wrap);
  const NAMES = { arm: "A-arm leg", upright: "upright", tie: "tie rod", rod: "pull rod", rocker: "rocker", damper: "damper", arb: "anti-roll bar link" };
  ctx.local = () => {
    const axle = HP.axle, P = hpPoints(axle), B = GEO().hardpoints[axle].points;
    const rows = G.links.filter(([a, c]) => P[a] && P[c]).map(([a, c, kind]) => { const L = dist3(P[a], P[c]), L0 = dist3(B[a], B[c]);
      return el("tr", { class: Math.abs(L - L0) > 1e-6 ? "changed" : "" }, el("td", {}, NAMES[kind] || kind), el("td", { class: "muted" }, `${G.names[a] || a} → ${G.names[c] || c}`),
        el("td", {}, numIn(+(L * 1e3).toFixed(2), (v, inp) => { if (!(v > 1)) { inp.value = (L * 1e3).toFixed(2); return; } const p = hpPoints(axle), f = v / 1e3 / L;
          hpSet(axle, c, hpOnFrame(axle, c, [0, 1, 2].map((x) => p[a][x] + f * (p[c][x] - p[a][x])))); hpRefresh(); }, { min: 1, style: "width:96px", "aria-label": `${a} to ${c} length mm` })),
        el("td", {}, (L0 * 1e3).toFixed(2))); });
    wrap.replaceChildren(el("div", { class: "tablewrap" }, el("table", { class: "edit" }, el("thead", {}, el("tr", {}, ["member", "between", "length [mm]", "baseline [mm]"].map((h) => el("th", {}, h)))), el("tbody", {}, rows))));
  };
  ctx.local();
}, { wide: true });
V("chassis", "frame_nodes", "Frame joints by number", "Every joint of the frame in millimetres (x rearward from the front axle, y to the right, z up). Type a value to move it.", (b, ctx) => {
  need(GEO().frame); const wrap = el("div"); b.append(wrap);
  const after = () => { const c = CARDS.get("frame_edit"); frameCommit(true); if (c) c.ctx.local(true); };
  ctx.local = () => {
    const st = FR.state, base = FR.base().nodes, uses = st.nodes.map(() => 0);
    st.tubes.forEach((t) => t.n.forEach((k) => { if (k >= 0) uses[k]++; }));
    const rows = st.nodes.map((p, i) => el("tr", { class: base[i] && dist3(base[i], p) < 1e-6 ? "" : "changed" }, el("td", {}, `joint ${i + 1}`),
      ...[0, 1, 2].map((a) => el("td", {}, numIn(+(p[a] * 1e3).toFixed(2), (v) => { st.nodes[i] = st.nodes[i].slice(); st.nodes[i][a] = v / 1e3; after(); }, { style: "width:96px", "aria-label": `joint ${i + 1} ${"xyz"[a]} mm` }))),
      el("td", {}, String(uses[i])), el("td", {}, el("button", { class: "btn small", title: "Remove this joint and the tubes that end on it", onclick: () => { if (!confirm(`Remove joint ${i + 1} and the ${uses[i]} tube${uses[i] === 1 ? "" : "s"} on it?`)) return; frameDeleteNode(i); FR.selTube = null; after(); } }, "✕"))));
    wrap.replaceChildren(el("div", { class: "tablewrap" }, el("table", { class: "edit" }, el("thead", {}, el("tr", {}, ["joint", "x [mm]", "y [mm]", "z [mm]", "tubes", ""].map((h) => el("th", {}, h)))), el("tbody", {}, rows))));
  };
  ctx.local();
}, { wide: true, off: true });

// ------------------------------------------------------------------ brakes: sliders for the corner hardware
QUICK.brakes.all.push(["vehicle:brakes.pad_mu", "Pad friction coefficient", "", 1, 0.2, 0.7, 0.01, 0.4], ["vehicle:brakes.piston_area_front", "Front caliper piston area (per side)", "mm²", 1e6, 200, 2500, 5, 0.0008],
  ["vehicle:brakes.piston_area_rear", "Rear caliper piston area (per side)", "mm²", 1e6, 200, 2500, 5, 0.0005], ["vehicle:brakes.rotor_radius_front", "Front effective rotor radius", "mm", 1e3, 50, 130, 0.5],
  ["vehicle:brakes.rotor_radius_rear", "Rear effective rotor radius", "mm", 1e3, 50, 130, 0.5, 0.08], ["vehicle:brakes.rotor_mass_front", "Front rotor mass", "kg", 1, 0.2, 2.5, 0.01],
  ["vehicle:brakes.rotor_mass_rear", "Rear rotor mass", "kg", 1, 0.2, 2.5, 0.01, 0.641], ["vehicle:brakes.rotor_area_front", "Front rotor cooled area", "cm²", 1e4, 100, 1200, 5],
  ["vehicle:brakes.rotor_heat_fraction", "Share of brake heat into the rotor", "%", 100, 50, 100, 1], ["vehicle:brakes.rotor_h1", "Rotor cooling with speed (h1)", "W/m²K", 1, 2, 30, 0.5, 9],
  ["vehicle:brakes.pedal_ratio", "Pedal ratio", "", 1, 2, 8, 0.05, 5]);

// ------------------------------------------------------------------ one wing section on its own
const AF = { i: 0, custom: null };
V("aero", "airfoil", "Wing elements, one at a time", "Each section alone in free air: shape, pressure and lift from a panel method. Pick an element, or change its section, chord, angle and speed.", (b) => {
  const els = need(GEO().wing_elements); if (els.error) throw new Error(els.error);
  const rho = GEO().air_density || 1.12;
  const state = AF.custom || (AF.custom = els.map((e) => ({ section: e.section, chord: e.chord, angle: e.angle_deg, speed: 15, coords: "", result: e.result })));
  const out = el("div"), ctl = el("div", { class: "ctl" }), tabl = el("div");
  const name = (e, k) => `${e.wing === "front_wing" ? "Front wing" : "Rear wing"} element ${e.element}`;
  const show = () => {
    const c = state[AF.i], r = c.result, shape = plotDiv("short"), cp = plotDiv("short"), curve = plotDiv("short");
    out.replaceChildren(el("div", { class: "kpis" }, tile("Lift coefficient", fmt(r.cl, 3), ""), tile("Lift per metre of span", fmt(r.lift_N_per_m, 3), "N/m", `${fmt(r.lift_N_per_m / 9.81, 3)} kgf/m at ${c.speed} m/s`),
      tile("Lift-curve slope", fmt(r.cl_per_deg, 3), "per deg"), tile("Zero-lift angle", fmt(r.alpha_zero_lift_deg, 3), "deg"), tile("Moment about ¼ chord", fmt(r.cm_quarter_chord, 3), ""),
      tile("Lowest pressure coefficient", fmt(r.min_cp, 3), ""), tile("Thickness", fmt(r["thickness_%"], 3), "% chord"), tile("Reynolds number", (r.reynolds / 1e3).toFixed(0) + "k", "")),
      el("div", { class: "grid inner" }, el("div", {}, el("div", { class: "sub" }, "Section shape (chord = 1)"), shape), el("div", {}, el("div", { class: "sub" }, "Pressure along the chord (suction up)"), cp), el("div", {}, el("div", { class: "sub" }, "Lift coefficient against angle"), curve)));
    plot(shape, [line(r.x, r.y, "section", { fill: "toself", fillcolor: "rgba(42,120,214,.10)" })], { hovermode: "closest", xaxis: { title: "x / chord", constrain: "domain" }, yaxis: { scaleanchor: "x", constrain: "domain", title: "" } });
    plot(cp, [{ x: r.cp_x, y: r.cp, mode: "lines", line: { width: 2, color: PAL[0] }, name: "Cp", hovertemplate: "x/c %{x:.2f}: Cp %{y:.2f}<extra></extra>" }], { hovermode: "closest", xaxis: { title: "x / chord" }, yaxis: { title: "Cp", autorange: "reversed" } });
    plot(curve, [line(r.alpha, r.cl_curve, "lift curve"), { x: [c.angle], y: [r.cl], mode: "markers", marker: { size: 10, color: PAL[1], line: { color: "#fff", width: 2 } }, name: "this angle", hovertemplate: "%{x}°: %{y:.3f}<extra></extra>" }],
      { xaxis: { title: "angle to the airflow [deg]" }, yaxis: { title: "lift coefficient" } });
  };
  const table8 = () => tabl.replaceChildren(table({ columns: ["element", "section", "chord [mm]", "angle [deg]", "lift coefficient", "lift [N per m span]"],
    rows: els.map((e, k) => [name(e, k), state[k].coords ? "pasted coordinates" : state[k].section, state[k].chord * 1e3, state[k].angle, state[k].result.cl, state[k].result.lift_N_per_m]) }));
  const solve = async () => {
    const c = state[AF.i];
    try { setStatus("Solving the section…", "busy"); c.result = await call("airfoil", [c.section, c.chord, c.angle, c.speed, rho, c.coords || null]); setStatus("Section solved.", "ok"); show(); table8(); }
    catch (e) { setStatus("The section could not be solved: " + (e.message || e), "err"); }
  };
  const controls = () => {
    const c = state[AF.i];
    const sel = el("select", { "aria-label": "Wing element", onchange: () => { AF.i = +sel.value; controls(); show(); } }, els.map((e, k) => el("option", { value: k }, name(e, k)))); sel.value = AF.i;
    const sec = el("input", { type: "text", value: c.section, style: "width:110px", "aria-label": "NACA section", onchange: () => { c.section = sec.value.trim(); c.coords = ""; solve(); } });
    const paste = el("textarea", { rows: 4, placeholder: "Or paste the real section's coordinates here (a Selig .dat file: x y on each line, trailing edge → upper surface → leading edge → lower surface), then press Solve.", style: "width:100%;font:12px var(--mono)" });
    paste.value = c.coords;
    ctl.replaceChildren(sel, el("label", {}, "NACA section ", sec), el("label", {}, "chord [mm] ", numIn(c.chord * 1e3, (v) => { c.chord = v / 1e3; solve(); }, { min: 10, style: "width:84px" })),
      el("label", {}, "angle [deg] ", numIn(c.angle, (v) => { c.angle = v; solve(); }, { min: -20, max: 60, style: "width:76px" })), el("label", {}, "speed [m/s] ", numIn(c.speed, (v) => { c.speed = v; solve(); }, { min: 1, max: 60, style: "width:76px" })),
      el("button", { class: "btn small navy", onclick: () => { c.coords = paste.value.trim(); solve(); } }, "Solve"), el("details", { style: "flex:1 1 100%" }, el("summary", { class: "muted", style: "cursor:pointer" }, "Use the real section (FX 76-MP-120, GOE 448, AH 74-7-6): paste coordinates"), paste));
  };
  controls(); show(); table8();
  b.append(ctl, out, el("div", { class: "sub", style: "margin-top:10px" }, "All eight elements, each on its own"), tabl,
    el("div", { class: "note warn" }, "Read these as section properties, not installed loads. The method is inviscid: no stall and no drag, so lift keeps rising with angle where a real flap would separate. In the car each flap sits in the flow turned by the element ahead of it, near the ground and with endplates. The config uses NACA stand-ins for the FX, GOE and AH sections until their coordinates are pasted in."));
}, { wide: true });

// ------------------------------------------------------------------ drivers
/* Profiles come from data/drivers.json (web/drivers.json at build time); edits on the Driver page are kept in this
   browser (localStorage) and can be exported back to that file. The chosen driver is applied as ordinary inputs:
   driver mass, the car's CG (the driver's mass at their seat), grip scale, shift time (IC), endurance power (EV). */
const DR = {
  load() {
    if (this.data) return this.data;
    let d = null;
    try { d = JSON.parse(localStorage.getItem("lapsim.drivers") || "null"); } catch (e) { d = null; }
    this.data = d && d.drivers && d.drivers.length ? d : clone(S.driverFile || { seat: { x_mm: 340, z_mm: 140, back_angle_deg: 35 }, drivers: [{ id: "design", name: "Design driver (baseline)", eye_height_mm: 740, head_top_mm: 940, cg_above_seat_mm: 230, grip_used_pct: 100 }] });
    return this.data;
  },
  save() { localStorage.setItem("lapsim.drivers", JSON.stringify(this.data)); },
  chosen() { const d = this.load(), id = localStorage.getItem("lapsim.driver." + S.car) || "design"; return d.drivers.find((x) => x.id === id) || d.drivers[0]; },
  choose(id) { localStorage.setItem("lapsim.driver." + S.car, id); this.apply(); },
  /* seat-back direction: up and rearward (car x is rearward) */
  along(mm) { const s = this.load().seat, a = (s.back_angle_deg ?? 35) * Math.PI / 180; return [s.x_mm / 1e3 + mm / 1e3 * Math.sin(a), 0, s.z_mm / 1e3 + mm / 1e3 * Math.cos(a)]; },
  cg(d) { const s = this.load().seat; return [s.x_mm / 1e3 - 0.15, 0, s.z_mm / 1e3 + (d.cg_above_seat_mm ?? 230) / 1e3]; },   // torso + legs forward of the hip (ASSUMED 150 mm)
  KEYS: ["vehicle:mass.driver_mass", "vehicle:mass.cg_height", "vehicle:mass.front_weight_fraction", "vehicle:tires.lambda_mu", "pt:gearbox.upshift_time", "ev:strategy.endurance_power_limit"],
  apply() {
    const d = this.chosen(), all = this.load().drivers, design = all.find((x) => x.id === "design") || all[0], base = (k) => (inputOf(k) || {}).value;
    if (d.id === "design") { for (const k of this.KEYS) if (inputOf(k)) setOv(k, base(k), { quiet: true }); return pending(); }
    const md0 = base("vehicle:mass.driver_mass"), m0 = base("vehicle:mass.mass_no_driver") + md0, L = base("vehicle:geometry.wheelbase");
    const md = d.mass_kg ?? md0, m1 = m0 - md0 + md, p0 = this.cg(design), p1 = this.cg(d);
    const x1 = (m0 * (1 - base("vehicle:mass.front_weight_fraction")) * L - md0 * p0[0] + md * p1[0]) / m1, h1 = (m0 * base("vehicle:mass.cg_height") - md0 * p0[2] + md * p1[2]) / m1;
    setOv("vehicle:mass.driver_mass", md, { quiet: true }); setOv("vehicle:mass.cg_height", +h1.toFixed(5), { quiet: true }); setOv("vehicle:mass.front_weight_fraction", +(1 - x1 / L).toFixed(5), { quiet: true });
    setOv("vehicle:tires.lambda_mu", +(base("vehicle:tires.lambda_mu") * (d.grip_used_pct ?? 100) / 100).toFixed(5), { quiet: true });
    if (inputOf("pt:gearbox.upshift_time")) setOv("pt:gearbox.upshift_time", d.upshift_ms != null ? d.upshift_ms / 1e3 : base("pt:gearbox.upshift_time"), { quiet: true });
    if (inputOf("ev:strategy.endurance_power_limit")) setOv("ev:strategy.endurance_power_limit", d.endurance_power_kW != null ? d.endurance_power_kW * 1e3 : base("ev:strategy.endurance_power_limit"), { quiet: true });
    pending();
  },
};
function driverBanner() {
  const d = DR.chosen();
  return el("button", { class: "driverchip", title: "Change the driver", onclick: () => { S.page = "drivers"; location.hash = `${S.car}/drivers`; render(); window.scrollTo(0, 0); } }, "Driver in the car: ", el("b", {}, d.name));
}
/* Where the driver can see: from the eye, the lowest line that clears every frame tube ahead within the cockpit's width. */
function driverSight(eye) {
  const P = frameEnds(FR.state); let dep = Math.PI / 2, lim = null;
  for (const q of P) { if (!q[0] || !q[1]) continue;
    for (let k = 0; k <= 4; k++) { const p = [0, 1, 2].map((a) => q[0][a] + k / 4 * (q[1][a] - q[0][a]));
      if (Math.abs(p[1]) > 0.26 || p[0] > eye[0] - 0.12) continue;
      const a = Math.atan2(eye[2] - p[2], eye[0] - p[0]); if (a < dep) { dep = a; lim = p; } } }
  return { dep, lim, ground: dep > 0.005 ? eye[0] - eye[2] / Math.tan(dep) : null };
}
function driverHoops() {
  const n = FR.state.nodes; if (!n.length) return null;
  const main = n.reduce((a, c) => (c[2] > a[2] ? c : a)), ahead = n.filter((p) => p[0] < main[0] - 0.3 && p[0] > 0.1);
  if (!ahead.length) return null;
  return { main, front: ahead.reduce((a, c) => (c[2] > a[2] ? c : a)) };
}
V("drivers", "driver_pick", "Driver in the car", "Pick a driver for this car. Their numbers go into the full car simulation: press Run simulation after choosing.", (b) => {
  const data = DR.load(), d = DR.chosen(), wrap = el("div"); b.append(wrap);
  const F = [["name", "Name", "", "text"], ["mass_kg", "Mass with race gear", "kg"], ["stature_mm", "Standing height", "mm"], ["eye_height_mm", "Eye above the seat reference, along the back", "mm"],
    ["head_top_mm", "Helmet top above the seat reference, along the back", "mm"], ["chest_depth_mm", "Chest depth", "mm"], ["cg_above_seat_mm", "Body CG above the seat reference", "mm"],
    ["grip_used_pct", "Grip the driver uses of what the tires have", "%"], ["upshift_ms", "Upshift time (IC car)", "ms"], ["endurance_power_kW", "Endurance power limit they drive to (EV)", "kW"], ["notes", "Notes", "", "text"]];
  const sel = el("select", { "aria-label": "Driver", onchange: () => { DR.choose(sel.value); render(); } }, data.drivers.map((x) => el("option", { value: x.id }, x.name))); sel.value = d.id;
  const form = el("div", { class: "dform" }, F.map(([k, label, unit, type]) => { const i = el("input", { type: type || "number", step: "any", value: d[k] ?? "", placeholder: "not measured", "aria-label": label });
    i.addEventListener("change", () => { d[k] = type === "text" ? i.value : i.value === "" ? null : +i.value; DR.save(); DR.apply(); if (k === "name") render(); else { const c = CARDS.get("driver_view"); if (c) c.ctx.local(); const e = CARDS.get("driver_effect"); if (e) e.ctx.local(); } });
    return el("label", { class: type === "text" ? "widef" : "" }, label, i, type === "text" ? null : el("span", { class: "u" }, unit)); }));
  const add = (copy) => { const id = "driver_" + Date.now().toString(36), n = Object.assign({}, copy ? d : { eye_height_mm: 740, head_top_mm: 940, cg_above_seat_mm: 230, grip_used_pct: 100 }, { id, name: copy ? d.name + " (copy)" : "New driver" }); data.drivers.push(n); DR.save(); DR.choose(id); render(); };
  wrap.append(el("div", { class: "ctl" }, sel, el("button", { class: "btn small", onclick: () => add(false) }, "Add a driver"), el("button", { class: "btn small", onclick: () => add(true) }, "Duplicate"),
    d.id !== "design" ? el("button", { class: "btn small", onclick: () => { if (!confirm(`Delete ${d.name}?`)) return; data.drivers.splice(data.drivers.indexOf(d), 1); DR.save(); DR.choose("design"); render(); } }, "Delete") : null,
    el("button", { class: "btn small", onclick: () => { const a = el("a", { href: URL.createObjectURL(new Blob([JSON.stringify(data, null, 1)], { type: "application/json" })), download: "drivers.json" }); a.click(); } }, "Export drivers.json"),
    el("label", { class: "btn small" }, "Import", el("input", { type: "file", accept: ".json", hidden: true, onchange: async (e) => { try { const j = JSON.parse(await e.target.files[0].text()); if (!j.drivers) throw new Error("no drivers in that file"); DR.data = j; DR.save(); render(); } catch (err) { alert("Could not read that file: " + err.message); } } })),
    el("button", { class: "btn small", onclick: () => { if (!confirm("Forget the driver edits made in this browser and go back to the site's drivers.json?")) return; localStorage.removeItem("lapsim.drivers"); DR.data = null; DR.choose("design"); render(); } }, "Reset to the site's list")),
    form, el("div", { class: "sub" }, d.id === "design" ? "The design driver is the one the car configs assume: choosing it changes nothing. Its mass comes from the car config." : "Edits are kept in this browser. Export drivers.json and replace web/drivers.json in the repository to share them with the team."));
}, { wide: true, first: true });
V("drivers", "driver_effect", "What this driver changes in the car", "The inputs set by the chosen driver, next to the baseline.", (b, ctx) => {
  const wrap = el("div"); b.append(wrap);
  ctx.local = () => {
    const N = { "vehicle:mass.driver_mass": ["Driver mass", 1, "kg"], "vehicle:mass.cg_height": ["Car CG height", 1e3, "mm"], "vehicle:mass.front_weight_fraction": ["Weight on the front axle", 100, "%"],
      "vehicle:tires.lambda_mu": ["Grip scale", 1, ""], "pt:gearbox.upshift_time": ["Upshift time", 1e3, "ms"], "ev:strategy.endurance_power_limit": ["Endurance power limit", 1e-3, "kW"] };
    const rows = DR.KEYS.filter((k) => inputOf(k)).map((k) => { const [n, f, u] = N[k], base = inputOf(k).value, now = cur(k); return [n, base == null ? "not set" : `${fmt(base * f, 5)} ${u}`, now == null ? "not set" : `${fmt(now * f, 5)} ${u}`]; });
    wrap.replaceChildren(table({ columns: ["input", "baseline", "with this driver"], rows }),
      el("div", { class: "note warn" }, "Grip used and shift time are the personal driving numbers the model can take today. They are entered by hand here: fitting them from a driver's logged laps (their g-g envelope against the car's) is done in the notebooks, not on the site. The car has no lift-and-coast or throttle-smoothness model yet, so those cannot be driver settings."));
  };
  ctx.local();
});
V("drivers", "driver_view", "Seating and sight line", "Drag the seat reference point. The eye and helmet follow the driver's measurements along the seat back; the sight line is the lowest one that clears the frame ahead.", (b, ctx) => {
  need(GEO().frame); const data = DR.load(), tiles = el("div", { class: "kpis" }), ctl = el("div", { class: "ctl" }), R = cur("vehicle:tires.tire_radius") || 0.203;
  const pts = () => { const d = DR.chosen(), s = data.seat; return { seat: [s.x_mm / 1e3, 0, s.z_mm / 1e3], eye: DR.along(d.eye_height_mm ?? 740), head: DR.along(d.head_top_mm ?? 940) }; };
  const ed = viewEditor({
    views: [{ title: "Side view (front of the car on the left). Blue: seat reference, drag it. Orange: eye. Green: helmet top.", ax: [0, 2], sign: [1, 1], names: ["x", "z"] }],
    points: () => { const p = pts(); return [{ id: "seat", p: p.seat, label: "seat reference point (hip)", color: "#2a78d6" }, { id: "eye", p: p.eye, label: "eye", color: "#eb6834", fixed: true }, { id: "head", p: p.head, label: "helmet top", color: "#1baf7a", fixed: true }]; },
    segs: () => { const p = pts(), out = frameEnds(FR.state).filter((q) => q[0] && q[1]).map((q) => ({ a: q[0], b: q[1], color: "rgba(0,14,47,.22)", w: 3 })), s = driverSight(p.eye), h = driverHoops();
      out.push({ a: p.seat, b: p.head, color: "#47526a", w: 4 });
      if (s.ground != null) out.push({ a: p.eye, b: [s.ground, 0, 0], color: "#eb6834", w: 1.6, dash: true, nofit: true });
      out.push({ a: p.eye, b: [p.eye[0] - 3, 0, p.eye[2]], color: "#b9c0cc", w: 1, dash: true, nofit: true });
      if (h) out.push({ a: h.main, b: h.front, color: "#1baf7a", w: 1.4, dash: true });
      return out; },
    extent: () => [[-1.2, 0, 0], [cur("vehicle:geometry.wheelbase") + 0.4, 0, 1.3]],
    decor: (v, proj, px) => { const f = proj([0, 0, R]), r = proj([cur("vehicle:geometry.wheelbase"), 0, R]);
      return `<circle cx="${f[0]}" cy="${f[1]}" r="${R * 1e3}" class="tire"/><circle cx="${r[0]}" cy="${r[1]}" r="${R * 1e3}" class="tire"/><line x1="-99999" x2="99999" y1="0" y2="0" stroke="#7a859c" stroke-width="${1.4 * px}"/>`; },
    onMove: (id, p) => { data.seat.x_mm = Math.round(p[0] * 1e3); data.seat.z_mm = Math.round(Math.max(20, p[2] * 1e3)); DR.save(); DR.apply(); show(); },
  });
  function show() {
    const p = pts(), s = driverSight(p.eye), h = driverHoops(), all = data.drivers, a = (data.seat.back_angle_deg ?? 35) * Math.PI / 180;
    const avg = all.reduce((t, d) => t + data.seat.z_mm + (d.eye_height_mm ?? 740) * Math.cos(a), 0) / all.length;
    let clr = null;
    if (h) { const dx = h.front[0] - h.main[0], dz = h.front[2] - h.main[2], zLine = h.main[2] + dz * (p.head[0] - h.main[0]) / dx;
      clr = (zLine - p.head[2]) * Math.abs(dx) / Math.hypot(dx, dz); }             // perpendicular distance, positive with the helmet below the line
    tiles.replaceChildren(tile("Eye height above the ground", (p.eye[2] * 1e3).toFixed(0), "mm"), tile("Average eye height, all drivers", avg.toFixed(0), "mm", `${all.length} driver${all.length === 1 ? "" : "s"} at this seat`),
      tile("Sight line below horizontal", s.dep > 0 ? (s.dep * 180 / Math.PI).toFixed(1) : "blocked", s.dep > 0 ? "deg" : "", s.dep <= 0 ? "a frame tube ahead is above eye level" : s.lim ? `limited by the frame ${(-(s.lim[0] - p.eye[0]) * 1e3).toFixed(0)} mm ahead of the eye` : ""),
      tile("Nearest ground in view", s.ground != null ? (-s.ground).toFixed(2) : "none", s.ground != null ? "m ahead of the front axle" : "", s.ground != null ? `${(p.eye[0] - s.ground).toFixed(2)} m ahead of the eye` : "the frame hides the ground"),
      clr != null ? tile("Helmet below the hoop line", (clr * 1e3).toFixed(0), "mm", clr * 1e3 >= 50 ? "more than the 50 mm the rules ask for" : "less than the 50 mm the rules ask for", clr * 1e3 >= 50 ? "down" : "up") : null);
  }
  const ang = numIn(data.seat.back_angle_deg ?? 35, (v) => { data.seat.back_angle_deg = v; DR.save(); DR.apply(); ed.draw(); show(); }, { min: 0, max: 70, style: "width:76px" });
  ctl.append(el("label", {}, "Seat-back angle from vertical [deg] ", ang), el("button", { class: "btn small", onclick: () => ed.fit() }, "Fit view"));
  ctx.local = () => { ed.draw(); show(); };
  show(); b.append(tiles, ctl, ed.node, el("div", { class: "note warn" }, "The seat reference point and seat-back angle are placeholders until they are measured on the car. The sight line is over the frame tubes only: bodywork, the steering wheel and the front wing are not in it. The hoop line joins the highest frame joint to the highest joint ahead of it."));
}, { wide: true, cls: "low", first: true });
