/* Wing designer: every wing element drawn where it sits on the car, to scale, and editable (section shape, chord,
   angle, position, slot to the element ahead, skin material), plus the endplates. Loaded after tunnel.js.

   The design lives in WD (kept in this browser per car, and saved with a team design). It does not touch the lap
   simulation until "Use this wing in the full car simulation" writes the change into the aero inputs.
   Geometry is built here in the page (so the 2-D wind tunnel follows it at once); loads come from the bridge's
   panel method (wing_stack) and are then corrected for the finite span and the endplates. */
"use strict";

const WD = {
  wing: "front_wing", el: "F1", zone: null, speed: 15, ground: true, cache: {}, timer: 0,
  load() { if (!this._d) this._d = {}; if (!this._d[S.car]) { try { this._d[S.car] = JSON.parse(localStorage.getItem("lapsim.wings." + S.car) || "null") || { els: {}, plates: {} }; } catch (e) { this._d[S.car] = { els: {}, plates: {} }; } } return this._d[S.car]; },
  save() { const d = this.load(); for (const k of Object.keys(d.els)) if (!Object.keys(d.els[k]).length) delete d.els[k]; localStorage.setItem("lapsim.wings." + S.car, JSON.stringify(d)); },
  set(design) { if (!this._d) this._d = {}; this._d[S.car] = design || { els: {}, plates: {} }; this.save(); },
  edited() { const d = this.load(); return Object.keys(d.els).length + Object.keys(d.plates).length > 0; },
};
const SKINS = { "Carbon, 2 plies on foam": 1.0, "Carbon, 3 plies": 1.3, "Carbon, 4 plies": 1.7, "Fibreglass, 3 plies": 1.9, "Aluminium sheet 0.8 mm": 2.2 };      // kg per m² of skin: typical figures, change them to yours
const PLATE_MATS = { "Carbon plate": 1550, "Carbon on foam core": 600, "Aluminium": 2700, "Plywood": 650, "ABS / printed": 1050 };                                // kg/m³
const wdBaseEls = () => { const e = GEO().wing_elements; return Array.isArray(e) ? e : []; };
const wdBase = (name) => wdBaseEls().find((e) => e.name === name);
const interp = (x, xs, ys) => { if (x <= xs[0]) return ys[0]; for (let i = 1; i < xs.length; i++) if (x <= xs[i]) return ys[i - 1] + (ys[i] - ys[i - 1]) * (x - xs[i - 1]) / (xs[i] - xs[i - 1] || 1); return ys[ys.length - 1]; };
/* NACA 4-digit section, unit chord, upright, from the trailing edge over the top to the leading edge and back. */
function nacaUnit(code, n = 46) {
  code = String(code).toLowerCase().replace("naca", "").trim().padStart(4, "0");
  if (!/^\d{4}$/.test(code)) throw new Error(`"${code}" is not a NACA 4-digit code (for example 6412)`);
  const m = +code[0] / 100, p = +code[1] / 10, t = +code.slice(2) / 100, up = [], lo = [];
  for (let i = 0; i < n; i++) { const x = 0.5 * (1 - Math.cos(Math.PI * i / (n - 1))), yt = 5 * t * (0.2969 * Math.sqrt(x) - 0.126 * x - 0.3516 * x * x + 0.2843 * x ** 3 - 0.1036 * x ** 4);
    const yc = m && p ? (x < p ? m / p ** 2 * (2 * p * x - x * x) : m / (1 - p) ** 2 * (1 - 2 * p + 2 * p * x - x * x)) : 0, dy = m && p ? (x < p ? 2 * m / p ** 2 * (p - x) : 2 * m / (1 - p) ** 2 * (p - x)) : 0, th = Math.atan(dy);
    up.push([x - yt * Math.sin(th), yc + yt * Math.cos(th)]); lo.push([x + yt * Math.sin(th), yc - yt * Math.cos(th)]); }
  return up.reverse().concat(lo.slice(1));
}
/* Pasted coordinates (Selig order: x y per line from the trailing edge over the top), scaled to unit chord. */
function pastedUnit(text) {
  const pts = String(text).split(/\n/).map((l) => l.replace(/,/g, " ").trim().split(/\s+/).map(Number)).filter((c) => c.length >= 2 && isFinite(c[0]) && isFinite(c[1]) && Math.abs(c[0]) <= 1.5);
  if (pts.length < 12) throw new Error("paste at least 12 rows of x y coordinates (trailing edge, upper surface, leading edge, lower surface)");
  const x0 = Math.min(...pts.map((p) => p[0])), c = Math.max(...pts.map((p) => p[0])) - x0, le = pts.reduce((a, p, i) => (p[0] < pts[a][0] ? i : a), 0), y0 = pts[le][1];
  return pts.map((p) => [(p[0] - x0) / c, (p[1] - y0) / c]);
}
/* A unit section with its thickness and camber scaled. */
function scaleUnit(u, kt, kc) {
  if (Math.abs(kt - 1) < 1e-9 && Math.abs(kc - 1) < 1e-9) return u;
  const le = u.reduce((a, p, i) => (p[0] < u[a][0] ? i : a), 0), up = u.slice(0, le + 1).reverse(), lo = u.slice(le).concat([u[0]]);
  const ux = up.map((p) => p[0]), uy = up.map((p) => p[1]), lx = lo.map((p) => p[0]), ly = lo.map((p) => p[1]), n = 46, top = [], bot = [];
  for (let i = 0; i < n; i++) { const x = 0.5 * (1 - Math.cos(Math.PI * i / (n - 1))), a = interp(x, ux, uy), b = interp(x, lx, ly), c = kc * 0.5 * (a + b), t = kt * (a - b);
    top.push([x, c + t / 2]); bot.push([x, c - t / 2]); }
  return top.reverse().concat(bot.slice(1));
}
/* One element as designed: its numbers, its unit section, and its outline on the car [m]. */
function wdEl(name) {
  const b = wdBase(name), d = WD.load().els[name] || {};
  const e = { name, wing: b.wing, chord: d.chord ?? b.chord, angle: d.angle ?? b.angle_deg, le: [b.le[0] + (d.dx || 0), b.le[1] + (d.dz || 0)], thick: d.thick ?? 1, camber: d.camber ?? 1,
    naca: d.naca || "", coords: d.coords || "", skin: d.skin ?? 1.0, inverted: b.inverted !== false, span: b.span, y: b.y, base: b, edited: Object.keys(d).length > 0 };
  let u; try { u = e.coords ? pastedUnit(e.coords) : e.naca ? nacaUnit(e.naca) : b.unit; } catch (err) { u = b.unit; e.error = err.message; }
  e.unit = scaleUnit(u, e.thick, e.camber);
  const a = e.angle * Math.PI / 180, ex = [Math.cos(a), Math.sin(a)], ez = [-Math.sin(a), Math.cos(a)], s = e.inverted ? -1 : 1;
  e.poly = e.unit.map(([x, y]) => [e.le[0] + e.chord * (x * ex[0] + s * y * ez[0]), e.le[1] + e.chord * (x * ex[1] + s * y * ez[1])]);
  e.te = e.poly[0];
  e.perimeter = e.poly.reduce((t, p, i) => t + Math.hypot(p[0] - e.poly[(i + 1) % e.poly.length][0], p[1] - e.poly[(i + 1) % e.poly.length][1]), 0);
  e.mass = e.perimeter * e.span * e.skin;
  return e;
}
const wdWingEls = (wing) => wdBaseEls().filter((e) => e.wing === wing).map((e) => wdEl(e.name));
function wdPatch(name, patch) {
  const d = WD.load(), b = wdBase(name), cur = Object.assign({}, d.els[name] || {}, patch);
  const same = { chord: b.chord, angle: b.angle_deg, dx: 0, dz: 0, thick: 1, camber: 1, naca: "", coords: "", skin: 1.0 };
  for (const k of Object.keys(cur)) if (cur[k] == null || cur[k] === same[k] || (typeof cur[k] === "number" && Math.abs(cur[k] - same[k]) < 1e-9)) delete cur[k];
  d.els[name] = cur; WD.save(); WD.cache = {};
}
const segDist = (p, a, b) => { const dx = b[0] - a[0], dy = b[1] - a[1], t = Math.min(1, Math.max(0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy || 1e-12))); return Math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy); };
/* The slot between an element and the one ahead: the narrowest distance from the forward element's trailing edge. */
const wdSlot = (prev, e) => e.poly.reduce((m, p, i) => Math.min(m, segDist(prev.te, p, e.poly[(i + 1) % e.poly.length])), Infinity);
const wdZones = (wing) => ((GEO().wing_stacks || {}).zones || []).filter((z) => z.wing === wing);

/* Endplates: the outer plate's height sets how much of the 2-D lift a finite wing keeps (Hoerner's rule for the
   effective aspect ratio, AR (1 + 1.9 h / b), then lifting-line theory). */
function wdPlates(wing) {
  const base = (((GEO().wing_stacks || {}).endplates || {})[wing] || []), d = WD.load().plates;
  return base.map((p, i) => { const e = d[p.name] || {}; const o = { name: p.name, label: wing === "rear_wing" ? "Endplate" : ["Inner endplate (by the nose)", "Middle endplate", "Outer endplate"][i] || p.name, base: p, y: p.y,
    height: e.height ?? p.height, length: e.length ?? p.length, thickness: e.thickness ?? p.thickness, material: e.material || "Carbon plate", density: e.density ?? PLATE_MATS[e.material || "Carbon plate"], outer: i === base.length - 1, edited: Object.keys(e).length > 0 };
    o.area = p.area * (o.height / p.height) * (o.length / p.length); o.mass = 2 * o.area * o.thickness * o.density; o.baseMass = 2 * p.area * p.thickness * PLATE_MATS["Carbon plate"]; return o; });
}
function wdFinite(wing, design = true) {
  const els = design ? wdWingEls(wing) : wdBaseEls().filter((e) => e.wing === wing).map((b) => ({ chord: b.chord, y: b.y, span: b.span }));
  const b = 2 * Math.max(...els.map((e) => e.y[1])), S = els.reduce((t, e) => t + e.chord * e.span, 0), AR = b * b / S;
  const plates = wdPlates(wing), outer = plates[plates.length - 1], h = outer ? (design ? outer.height : outer.base.height) : 0, ARe = AR * (1 + 1.9 * h / b);
  return { span: b, area: S, AR, ARe, keep: ARe / (ARe + 2), h };
}

// ------------------------------------------------------------------ the designer card
V("aero", "wingdesign", "Wing designer", "Every element where it sits on the car, to scale. Drag a leading edge to move an element, or change its section, chord, angle, slot and skin. The loads and the wind tunnel follow.", (b, ctx) => {
  const els0 = wdBaseEls(); if (!els0.length || !els0[0].le) throw new Error("the wing sections are not in this build: run python web/export_wings.py, then python web/build.py");
  const rho = GEO().air_density || 1.12, holder = el("div"), ctl = el("div", { class: "ctl" }), form = el("div"), out = el("div"), est = el("div");
  if (!wdBase(WD.el) || wdBase(WD.el).wing !== WD.wing) WD.el = wdBaseEls().find((e) => e.wing === WD.wing).name;
  let ed;
  const zones = () => wdZones(WD.wing), zoneOf = () => { const zs = zones(); let k = zs.findIndex((z) => z.elements.includes(WD.el) && z.elements.length > 1); if (k < 0) k = zs.findIndex((z) => z.elements.includes(WD.el)); return Math.max(0, WD.zone != null && zs[WD.zone] && zs[WD.zone].elements.includes(WD.el) ? WD.zone : k); };
  const after = (redrawForm = true) => { ed.draw(); if (redrawForm) drawForm(); clearTimeout(WD.timer); WD.timer = setTimeout(solve, 550); const t = CARDS.get("tunnel"); if (t && t.ctx.rebuild) t.ctx.rebuild(); const p = CARDS.get("endplates"); if (p && p.ctx.local) p.ctx.local(); };
  const build = () => {
    ed = viewEditor({
      views: [{ title: `${WD.wing === "front_wing" ? "Front" : "Rear"} wing, side view, to scale (front of the car on the left). Drag a leading edge to move its element.`, ax: [0, 2], sign: [1, 1], names: ["x", "z"] }],
      points: () => wdWingEls(WD.wing).map((e) => ({ id: e.name, p: [e.le[0], 0, e.le[1]], label: `${e.name} leading edge`, color: e.edited ? "#eb6834" : "#2a78d6" })),
      segs: () => { const o = [];
        wdWingEls(WD.wing).forEach((e, k) => { const c = e.name === WD.el ? "#eb6834" : PAL[k % PAL.length]; e.poly.forEach((p, i) => { const q = e.poly[(i + 1) % e.poly.length]; o.push({ a: [p[0], 0, p[1]], b: [q[0], 0, q[1]], color: c, w: e.name === WD.el ? 2.6 : 1.8, pick: e.name }); });
          if (e.edited) e.base.unit && wdBaseOutline(e.base).forEach((p, i, arr) => { const q = arr[(i + 1) % arr.length]; o.push({ a: [p[0], 0, p[1]], b: [q[0], 0, q[1]], color: "#b9c0cc", w: 1, dash: true, nofit: true }); }); });
        return o; },
      extent: () => (WD.wing === "front_wing" ? [[-0.95, 0, 0], [-0.2, 0, 0.45]] : [[1.2, 0, 0.7], [2.05, 0, 1.22]]),
      decor: (v, proj, px) => (WD.wing === "front_wing" ? `<line x1="-99999" x2="99999" y1="0" y2="0" stroke="#7a859c" stroke-width="${1.4 * px}"/><text x="${proj([-0.94, 0, 0])[0]}" y="${-6 * px}" font-size="${11 * px}" class="lbl">ground</text>` : ""),
      selected: WD.el,
      onMove: (id, p) => { const bs = wdBase(id); wdPatch(id, { dx: +(p[0] - bs.le[0]).toFixed(5), dz: +(Math.max(0.01 - 0, p[2]) - bs.le[1]).toFixed(5) }); WD.el = id; after(); },
      onSelect: (id) => { WD.el = id; WD.zone = null; drawCtl(); drawForm(); ed.draw(); show(); },
      onPick: (id) => { WD.el = id; WD.zone = null; ed.select(id); },
    });
    holder.replaceChildren(ed.node);
  };
  const drawCtl = () => {
    const zs = zones(), zsel = el("select", { "aria-label": "Stretch of span", onchange: () => { WD.zone = +zsel.value; if (!zs[WD.zone].elements.includes(WD.el)) WD.el = zs[WD.zone].elements[0]; drawCtl(); drawForm(); ed.select(WD.el); solve(); } },
      zs.map((z, k) => el("option", { value: k }, `${(z.y[0] * 1e3).toFixed(0)} to ${(z.y[1] * 1e3).toFixed(0)} mm from the centre line: ${z.elements.join(" + ")}`))); zsel.value = zoneOf();
    ctl.replaceChildren(el("div", { class: "seg light" }, [["front_wing", "Front wing"], ["rear_wing", "Rear wing"]].map(([w, n]) => el("button", { class: WD.wing === w ? "on" : "", onclick: () => { WD.wing = w; WD.zone = null; WD.el = wdBaseEls().find((e) => e.wing === w).name; build(); drawCtl(); drawForm(); solve(); } }, n))),
      el("div", { class: "seg light" }, wdBaseEls().filter((e) => e.wing === WD.wing).map((e) => el("button", { class: WD.el === e.name ? "on" : "", onclick: () => ed.select(e.name) }, e.name))), el("label", {}, "Solve the stretch ", zsel),
      el("label", {}, "speed [m/s] ", numIn(WD.speed, (v) => { WD.speed = v; WD.cache = {}; solve(); }, { min: 1, max: 60, style: "width:70px" })),
      el("label", { class: "check" }, el("input", { type: "checkbox", checked: WD.ground, onchange: (e) => { WD.ground = e.target.checked; WD.cache = {}; solve(); } }), "Ground"),
      el("button", { class: "btn small", onclick: () => ed.fit() }, "Fit view"),
      el("button", { class: "btn small", onclick: () => { if (!confirm("Put every wing element and endplate back to the CAD?")) return; WD.set(null); WD.cache = {}; build(); drawForm(); after(false); } }, "Reset the wings"));
  };
  const drawForm = () => {
    const e = wdEl(WD.el), all = wdWingEls(WD.wing), k = all.findIndex((x) => x.name === WD.el), prev = k > 0 ? all[k - 1] : null, bs = e.base;
    const f = (label, value, unit, on, attrs = {}) => el("label", {}, label, numIn(value, on, Object.assign({ style: "width:100%" }, attrs)), el("span", { class: "u" }, unit));
    const sec = el("input", { type: "text", value: e.naca, placeholder: "CAD section", "aria-label": "NACA code", style: "text-align:left" }); sec.addEventListener("change", () => { try { if (sec.value.trim()) nacaUnit(sec.value); wdPatch(e.name, { naca: sec.value.trim(), coords: "" }); after(); } catch (err) { alert(err.message); sec.value = e.naca; } });
    const skin = el("select", { "aria-label": "Skin", onchange: () => { wdPatch(e.name, { skin: +skin.value }); after(); } }, Object.entries(SKINS).map(([n, v]) => el("option", { value: v }, `${n} (${v} kg/m²)`))); skin.value = String(Object.values(SKINS).includes(e.skin) ? e.skin : 1.0);
    const paste = el("textarea", { rows: 3, placeholder: "Or paste a section's coordinates: x y on each line, from the trailing edge over the upper surface, round the leading edge and back (a Selig .dat file).", style: "width:100%;font:12px var(--mono)" }); paste.value = e.coords;
    form.replaceChildren(el("div", { class: "sub", style: "margin-top:8px" }, el("b", {}, `${e.name}`), ` · ${e.wing === "front_wing" ? "front wing" : "rear wing"} · runs ${(e.y[0] * 1e3).toFixed(0)} to ${(e.y[1] * 1e3).toFixed(0)} mm from the centre line, each side` + (e.edited ? " · changed from the CAD (dashed outline)" : "") + (e.error ? " · " + e.error : "")),
      el("div", { class: "dform wform" },
        f("Chord", +(e.chord * 1e3).toFixed(1), "mm", (v) => { if (v > 5) { wdPatch(e.name, { chord: v / 1e3 }); after(); } }, { min: 10 }),
        f("Angle, trailing edge up", +e.angle.toFixed(2), "deg", (v) => { wdPatch(e.name, { angle: v }); after(); }, { min: -20, max: 85, step: 0.5 }),
        f("Thickness", +(100 * e.thick).toFixed(0), "% of the section's", (v) => { if (v > 5) { wdPatch(e.name, { thick: v / 100 }); after(); } }, { min: 20, max: 300, step: 5 }),
        f("Camber", +(100 * e.camber).toFixed(0), "% of the section's", (v) => { wdPatch(e.name, { camber: v / 100 }); after(); }, { min: 0, max: 300, step: 5 }),
        f("Leading edge, behind the front axle", +(e.le[0] * 1e3).toFixed(1), "mm", (v) => { wdPatch(e.name, { dx: v / 1e3 - bs.le[0] }); after(); }),
        f("Leading edge, above the ground", +(e.le[1] * 1e3).toFixed(1), "mm", (v) => { wdPatch(e.name, { dz: v / 1e3 - bs.le[1] }); after(); }, { min: 5 }),
        prev ? f(`Leading edge ahead of ${prev.name}'s trailing edge (overlap)`, +((prev.te[0] - e.le[0]) * 1e3).toFixed(1), "mm", (v) => { wdPatch(e.name, { dx: prev.te[0] - v / 1e3 - bs.le[0] }); after(); }) : null,
        prev ? f(`Leading edge above ${prev.name}'s trailing edge`, +((e.le[1] - prev.te[1]) * 1e3).toFixed(1), "mm", (v) => { wdPatch(e.name, { dz: prev.te[1] + v / 1e3 - bs.le[1] }); after(); }) : null,
        el("label", {}, "Section: NACA code", sec, el("span", { class: "u" }, "")), el("label", {}, "Skin", skin, el("span", { class: "u" }, ""))),
      el("details", {}, el("summary", { class: "muted", style: "cursor:pointer" }, "Use another section's coordinates"), paste, el("div", { class: "ctl" }, el("button", { class: "btn small", onclick: () => { try { if (paste.value.trim()) pastedUnit(paste.value); wdPatch(e.name, { coords: paste.value.trim(), naca: "" }); after(); } catch (err) { alert(err.message); } } }, "Use these coordinates"))),
      el("div", { class: "kpis", style: "margin-top:8px" }, prev ? tile(`Slot between ${prev.name} and ${e.name}`, (wdSlot(prev, e) * 1e3).toFixed(1), "mm", "narrowest gap at the forward element's trailing edge") : null,
        tile("Skin mass, both sides of the car", e.mass.toFixed(2), "kg", `${(e.perimeter * 1e3).toFixed(0)} mm round the section × ${(e.span * 1e3).toFixed(0)} mm of span`),
        tile("Lowest point above the ground", (Math.min(...e.poly.map((p) => p[1])) * 1e3).toFixed(0), "mm"),
        el("div", { class: "kpi" }, el("div", { class: "k" }, "This element"), el("div", { class: "d" }, el("button", { class: "btn small", onclick: () => { const d = WD.load(); delete d.els[e.name]; WD.save(); WD.cache = {}; after(); } }, "Back to the CAD")))));
  };
  const key = (zi) => JSON.stringify([WD.wing, zi, WD.speed, WD.ground, WD.load().els]);
  const stack = async (zi) => {
    const z = zones()[zi], els = z.elements.map(wdEl);
    if (!WD.edited() && WD.speed === 15 && WD.ground && z.result) return z.result;
    if (WD.cache[key(zi)]) return WD.cache[key(zi)];
    const r = await call("wing_stack", [z.elements, WD.speed, rho, null, 0, WD.ground, els.map((e) => e.poly), els.map((e) => e.chord)]);
    return (WD.cache[key(zi)] = r);
  };
  const solve = async () => { try { if (WD.edited() || WD.speed !== 15 || !WD.ground) setStatus("Solving the wing…", "busy"); await show(); if ($("#dot").classList.contains("busy")) setStatus("Wing solved.", "ok"); } catch (e) { out.replaceChildren(el("div", { class: "err" }, "The wing could not be solved: " + (e.message || e))); setStatus("The wing could not be solved: " + (e.message || e), "err"); } };
  const show = async () => {
    const zi = zoneOf(), z = zones()[zi], r = await stack(zi), cp = plotDiv(), q = 0.5 * rho * WD.speed ** 2;
    out.replaceChildren(el("div", { class: "sub", style: "margin-top:10px" }, `Loads on ${z.elements.join(" + ")} solved together` + (WD.ground && WD.wing === "front_wing" ? ", in ground effect" : "") + ` at ${WD.speed} m/s`),
      el("div", { class: "kpis" }, tile("Downforce of the stack", fmt(r.downforce_N_per_m, 3), "N per m of span"), tile("Lift coefficient on the total chord", fmt(r.cl_on_total_chord, 3), "", `total chord ${fmt(r.total_chord_mm, 3)} mm`),
        ...r.elements.map((e) => tile(`${e.name}: share of the load`, (100 * e.downforce_N_per_m / r.downforce_N_per_m).toFixed(0), "%", `${fmt(e.downforce_N_per_m, 3)} N/m · lowest pressure coefficient ${fmt(Math.max(e.min_cp, -99), 3)}`))),
      el("div", { class: "sub" }, "Pressure along each element (suction up; axis cut at -10)"), cp);
    plot(cp, r.elements.map((e, i) => ({ x: e.cp_x.map((v) => v * 1e3), y: e.cp, mode: "lines", name: e.name, line: { width: 2, color: e.name === WD.el ? "#eb6834" : PAL[i % PAL.length] }, hovertemplate: e.name + ": Cp %{y:.2f}<extra></extra>" })),
      { hovermode: "closest", xaxis: { title: "x behind the front axle [mm]" }, yaxis: { title: "pressure coefficient", range: [1.2, -10] } });
    // whole wing: strips, then the finite-span and endplate correction, next to the 3-D CFD run
    const rows = [];
    for (const wing of ["front_wing", "rear_wing"]) {
      const zs = wdZones(wing); let now = 0, base = 0;
      for (let k = 0; k < zs.length; k++) { base += zs[k].result.downforce_N_per_m * (WD.speed / 15) ** 2 * zs[k].width; now += (wing === WD.wing ? (await stack(k)).downforce_N_per_m : (WD.edited() ? (await stackOf(wing, k)).downforce_N_per_m : zs[k].result.downforce_N_per_m * (WD.speed / 15) ** 2)) * zs[k].width; }
      const f1 = wdFinite(wing, true), f0 = wdFinite(wing, false);
      rows.push({ wing, strip: now, est: now * f1.keep, estBase: base * f0.keep, keep: f1.keep });
    }
    WD.last = rows;
    let cfd = null; try { const bs = await getJSON("data/cfd/base.json"), t = bs.group_table, gi = t.columns.indexOf("group"), di = t.columns.findIndex((c) => c.startsWith("downforce")); cfd = { speed: bs.speed, front_wing: t.rows.find((x) => x[gi] === "front_wing")[di], rear_wing: t.rows.find((x) => x[gi] === "rear_wing")[di] }; } catch (e) { cfd = null; }
    est.replaceChildren(el("div", { class: "sub", style: "margin-top:10px" }, "Whole wing at this speed"),
      table({ columns: ["wing", "2-D strips [N]", "kept by a wing of this span and endplates [%]", "3-D estimate [N]", "CAD wing, same estimate [N]", "change [%]", cfd ? `3-D CFD run scaled to ${WD.speed} m/s [N]` : ""],
        rows: rows.map((x) => [x.wing === "front_wing" ? "Front wing" : "Rear wing", +x.strip.toFixed(0), +(100 * x.keep).toFixed(0), +x.est.toFixed(0), +x.estBase.toFixed(0), +(100 * (x.est / x.estBase - 1)).toFixed(1), cfd ? +(cfd[x.wing] * (WD.speed / cfd.speed) ** 2).toFixed(0) : null]) }),
      el("div", { class: "ctl", style: "margin-top:8px" }, el("button", { class: "btn navy", title: "Scale each wing's downforce and induced drag in the car's aero inputs by the change computed here, and add the mass change", onclick: useInCar }, "Use this wing in the full car simulation"),
        el("span", { class: "muted" }, WD.edited() ? "Writes the change into the car's aero inputs; then press Run simulation." : "The wings are as in the CAD: nothing to apply.")));
  };
  const stackOf = async (wing, k) => { const z = wdZones(wing)[k], els = z.elements.map(wdEl), kk = JSON.stringify([wing, k, WD.speed, WD.ground, WD.load().els]);
    if (!WD.cache[kk]) WD.cache[kk] = await call("wing_stack", [z.elements, WD.speed, rho, null, 0, WD.ground, els.map((e) => e.poly), els.map((e) => e.chord)]); return WD.cache[kk]; };
  const useInCar = () => {
    if (!WD.last) return;
    const lines = [];
    for (const x of WD.last) { const k = `vehicle:aero.components.${x.wing}.`, ratio = x.est / x.estBase, f1 = wdFinite(x.wing, true), f0 = wdFinite(x.wing, false);
      const df = inputOf(k + "cfd_downforce"), dr = inputOf(k + "cfd_drag"), ind = (inputOf(k + "induced_drag") || {}).value ?? 0.5, ms = inputOf(k + "mass");
      if (df && df.value != null) { setOv(k + "cfd_downforce", +(df.value * ratio).toPrecision(6), { quiet: true }); lines.push(`${x.wing.replace("_", " ")}: downforce × ${ratio.toFixed(3)}`); }
      if (dr && dr.value != null) setOv(k + "cfd_drag", +(dr.value * (1 - ind + ind * ratio * ratio * f0.ARe / f1.ARe)).toPrecision(6), { quiet: true });
      const dm = wdWingEls(x.wing).reduce((t, e) => t + e.mass - e.base.span * wdBaseOutlinePerimeter(e.base) * 1.0, 0) + wdPlates(x.wing).reduce((t, p) => t + p.mass - p.baseMass, 0);
      if (ms && ms.value != null && Math.abs(dm) > 1e-3) { setOv(k + "mass", +(ms.value + dm).toFixed(4), { quiet: true }); lines.push(`${x.wing.replace("_", " ")}: mass ${dm > 0 ? "+" : ""}${dm.toFixed(2)} kg`); } }
    pending(); alert(lines.length ? "Written into the car's aero inputs:\n" + lines.join("\n") + "\n\nPress Run simulation to see what it does to the lap." : "Nothing changed.");
  };
  ctx.local = () => { ed.draw(); drawForm(); };
  ctx.resolve = solve;
  build(); drawCtl(); drawForm(); b.append(ctl, holder, form, out, est,
    el("div", { class: "note warn" }, "What to trust: the shapes, sizes and positions are the wing CAD, to scale. The loads come from a two-dimensional method with no viscosity, so nothing stalls and every slot works perfectly; the 3-D estimate then applies the span and endplate correction, and still sits above the CFD run. Use the change between two designs, not the absolute load. Wing elements are hollow skins here: the mass is skin area times the skin's mass per square metre, without ribs, spars or mounts."));
  show().catch((e) => out.replaceChildren(el("div", { class: "err" }, String(e.message || e))));
}, { wide: true });
/* The CAD outline of an element (for the dashed "before" shape) and its perimeter per metre of chord line. */
function wdBaseOutline(b) { const a = b.angle_deg * Math.PI / 180, ex = [Math.cos(a), Math.sin(a)], ez = [-Math.sin(a), Math.cos(a)], s = b.inverted !== false ? -1 : 1; return b.unit.map(([x, y]) => [b.le[0] + b.chord * (x * ex[0] + s * y * ez[0]), b.le[1] + b.chord * (x * ex[1] + s * y * ez[1])]); }
function wdBaseOutlinePerimeter(b) { const p = wdBaseOutline(b); return p.reduce((t, q, i) => t + Math.hypot(q[0] - p[(i + 1) % p.length][0], q[1] - p[(i + 1) % p.length][1]), 0); }

// ------------------------------------------------------------------ endplates
V("aero", "endplates", "Endplates", "Size and material of each plate. The outer plate's height decides how much of the section's downforce the finite wing keeps.", (b, ctx) => {
  need(GEO().wing_stacks); const wrap = el("div"); b.append(wrap);
  const resolve = () => { const c = CARDS.get("wingdesign"); if (c && c.ctx.resolve) c.ctx.resolve(); };
  const patch = (name, p) => { const d = WD.load(); d.plates[name] = Object.assign({}, d.plates[name] || {}, p); WD.save(); WD.cache = {}; ctx.local(); resolve(); };
  ctx.local = () => {
    const blocks = [];
    for (const wing of ["front_wing", "rear_wing"]) {
      const plates = wdPlates(wing), f1 = wdFinite(wing, true), f0 = wdFinite(wing, false);
      const rows = plates.map((p) => { const mat = el("select", { onchange: () => patch(p.name, { material: mat.value, density: PLATE_MATS[mat.value] }) }, Object.keys(PLATE_MATS).map((m) => el("option", { value: m }, m))); mat.value = p.material;
        return el("tr", { class: p.edited ? "changed" : "" }, el("td", {}, p.label), el("td", {}, (p.y * 1e3).toFixed(0)),
          el("td", {}, numIn(+(p.height * 1e3).toFixed(1), (v) => v > 5 && patch(p.name, { height: v / 1e3 }), { min: 10, style: "width:80px", "aria-label": p.label + " height mm" })),
          el("td", {}, numIn(+(p.length * 1e3).toFixed(1), (v) => v > 5 && patch(p.name, { length: v / 1e3 }), { min: 10, style: "width:80px", "aria-label": p.label + " length mm" })),
          el("td", {}, numIn(+(p.thickness * 1e3).toFixed(2), (v) => v > 0.1 && patch(p.name, { thickness: v / 1e3 }), { min: 0.2, step: 0.1, style: "width:72px", "aria-label": p.label + " thickness mm" })),
          el("td", {}, mat), el("td", {}, numIn(p.density, (v) => v > 10 && patch(p.name, { density: v }), { min: 20, style: "width:78px" })), el("td", {}, (p.area * 1e4).toFixed(0)), el("td", {}, p.mass.toFixed(2)),
          el("td", {}, p.edited ? el("button", { class: "btn small", onclick: () => { const d = WD.load(); delete d.plates[p.name]; WD.save(); WD.cache = {}; ctx.local(); resolve(); } }, "Reset") : "")); });
      blocks.push(el("div", { class: "sub", style: "margin-top:10px" }, el("b", {}, wing === "front_wing" ? "Front wing" : "Rear wing"), ` · span ${(f1.span * 1e3).toFixed(0)} mm · planform area ${f1.area.toFixed(3)} m²`),
        el("div", { class: "tablewrap" }, el("table", { class: "edit" }, el("thead", {}, el("tr", {}, ["plate", "from centre line [mm]", "height [mm]", "length [mm]", "thickness [mm]", "material", "density [kg/m³]", "area, one face [cm²]", "mass, both sides [kg]", ""].map((h) => el("th", {}, h)))), el("tbody", {}, rows))),
        el("div", { class: "kpis", style: "margin-top:8px" }, tile("Aspect ratio", f1.AR.toFixed(2), "", "span² ÷ planform area"), tile("Effective aspect ratio with the outer plate", f1.ARe.toFixed(2), "", delta(f1.ARe, f0.ARe, 2, "")),
          tile("Section downforce the wing keeps", (100 * f1.keep).toFixed(0), "%", delta(100 * f1.keep, 100 * f0.keep, 1, " points")), tile("Plates' mass", plates.reduce((t, p) => t + p.mass, 0).toFixed(2), "kg", delta(plates.reduce((t, p) => t + p.mass, 0), plates.reduce((t, p) => t + p.baseMass, 0), 2, " kg"))));
    }
    wrap.replaceChildren(...blocks, el("div", { class: "note warn" }, "Sizes are the plates' outlines in the CAD; change a height or length and the area scales with it. The aerodynamic effect is an estimate: the outer plate raises the wing's effective aspect ratio to AR × (1 + 1.9 × plate height ÷ span) (Hoerner), and lifting-line theory gives the share of the two-dimensional downforce a wing of that aspect ratio keeps. Inner and middle plates are counted for mass only; plate shape, cut-outs and the ground are not in the estimate. Densities are typical values: put in your layup's."));
  };
  ctx.local();
}, { wide: true });

// the old single-element and stack cards are replaced by the designer; order: tunnel, designer, endplates
for (const id of ["airfoil", "wingstack"]) { const i = VISUALS.findIndex((v) => v.id === id); if (i >= 0) VISUALS.splice(i, 1); }
for (const id of ["endplates", "wingdesign"]) { const i = VISUALS.findIndex((v) => v.id === id), m = VISUALS.splice(i, 1)[0]; VISUALS.splice(VISUALS.findIndex((v) => v.id === "tunnel") + 1, 0, m); }
