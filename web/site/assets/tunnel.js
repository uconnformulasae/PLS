/* 2-D virtual wind tunnel: a lattice-Boltzmann flow solver (D2Q9, BGK collision with a Smagorinsky eddy viscosity)
   running in the page, on side-view slices of the car from configs/ct17_cfd.yaml (data: geometry.tunnel).
   Loaded after editors.js. Lattice units inside; x = downstream (car rearward), j = 0 is the bottom row.

   What it is for: seeing where the air goes (wakes, slot flow, ground effect) and how a change moves the loads.
   What it is not: the car's forces. It is two-dimensional and at a Reynolds number far below the car's. */
"use strict";

const LB = { cx: [0, 1, 0, -1, 0, 1, -1, -1, 1], cy: [0, 0, 1, 0, -1, 1, 1, -1, -1], w: [4 / 9, 1 / 9, 1 / 9, 1 / 9, 1 / 9, 1 / 36, 1 / 36, 1 / 36, 1 / 36], opp: [0, 3, 4, 1, 2, 7, 8, 5, 6] };

class Tunnel {
  constructor(nx, ny, u0 = 0.08, tau0 = 0.503) {
    this.nx = nx; this.ny = ny; this.u0 = u0; this.tau0 = tau0; this.n = nx * ny;
    this.a = new Float32Array(9 * this.n); this.b = new Float32Array(9 * this.n);
    this.solid = new Uint8Array(this.n);                 // 0 = air, k = body number k
    this.ux = new Float32Array(this.n); this.uy = new Float32Array(this.n); this.rho = new Float32Array(this.n);
    this.fx = new Float64Array(16); this.fy = new Float64Array(16); this.steps = 0; this.forceSteps = 0;
    this.reset();
  }
  feq(i, rho, ux, uy) { const cu = 3 * (LB.cx[i] * ux + LB.cy[i] * uy); return LB.w[i] * rho * (1 + cu + 0.5 * cu * cu - 1.5 * (ux * ux + uy * uy)); }
  reset() {
    for (let c = 0; c < this.n; c++) { for (let i = 0; i < 9; i++) this.a[9 * c + i] = this.feq(i, 1, this.u0, 0); this.ux[c] = this.u0; this.uy[c] = 0; this.rho[c] = 1; }
    this.b.set(this.a); this.clearForces(); this.steps = 0; this.blown = false;
  }
  clearForces() { this.fx.fill(0); this.fy.fill(0); this.forceSteps = 0; }
  /* near[c] = 1 where a neighbouring cell is solid (those cells take the slower path with bounce-back). */
  markNear() {
    const { nx, ny, solid } = this, near = (this.near = new Uint8Array(this.n));
    for (let j = 1; j < ny - 1; j++) for (let x = 1; x < nx - 1; x++) { const c = j * nx + x;
      if (!solid[c] && (solid[c - 1] || solid[c + 1] || solid[c - nx] || solid[c + nx] || solid[c - 1 - nx] || solid[c + 1 - nx] || solid[c - 1 + nx] || solid[c + 1 + nx])) near[c] = 1; }
  }
  step() {
    if (!this.near) this.markNear();
    const { nx, ny, a, b, solid, near, u0, tau0, ux: UX, uy: UY, rho: RHO, fx: FX, fy: FY } = this, { cx, cy, opp } = LB;
    const f = this._f || (this._f = new Float64Array(9));
    const off = this._off || (this._off = Int32Array.from(cx.map((x, i) => x + cy[i] * nx)));
    const eq0 = this._eq0 || (this._eq0 = Float64Array.from(cx.map((_, i) => this.feq(i, 1, u0, 0))));       // the undisturbed flow
    const CS2 = 18 * 0.16 * 0.16 * Math.SQRT2, t2 = tau0 * tau0, n9 = 9 * nx;        // Smagorinsky constant 0.16
    for (let j = 0; j < ny; j++) {
      const edge = j === 0 || j === ny - 1;
      for (let x = 0; x < nx; x++) {
        const c = j * nx + x, k = 9 * c;
        if (solid[c]) continue;
        if (x === 0 || edge) {                            // inlet, moving ground, free stream above
          for (let i = 0; i < 9; i++) b[k + i] = eq0[i];
          UX[c] = u0; UY[c] = 0; RHO[c] = 1; continue;
        }
        if (x === nx - 1) { const s = k - 9; for (let i = 0; i < 9; i++) b[k + i] = a[s + i]; UX[c] = UX[c - 1]; UY[c] = UY[c - 1]; RHO[c] = RHO[c - 1]; continue; }   // outflow
        let f0, f1, f2, f3, f4, f5, f6, f7, f8;
        if (near[c]) {                                    // next to a body: populations bounce back and push on it
          for (let i = 0; i < 9; i++) { const sc = c - off[i], sb = solid[sc];
            if (sb) { const o = opp[i], v = a[k + o]; f[i] = v; FX[sb] += 2 * v * cx[o]; FY[sb] += 2 * v * cy[o]; } else f[i] = a[9 * sc + i]; }
          f0 = f[0]; f1 = f[1]; f2 = f[2]; f3 = f[3]; f4 = f[4]; f5 = f[5]; f6 = f[6]; f7 = f[7]; f8 = f[8];
        } else {                                          // pull each population from the cell it left
          f0 = a[k]; f1 = a[k - 9 + 1]; f2 = a[k - n9 + 2]; f3 = a[k + 9 + 3]; f4 = a[k + n9 + 4];
          f5 = a[k - 9 - n9 + 5]; f6 = a[k + 9 - n9 + 6]; f7 = a[k + 9 + n9 + 7]; f8 = a[k - 9 + n9 + 8];
        }
        let rho = f0 + f1 + f2 + f3 + f4 + f5 + f6 + f7 + f8, ux = (f1 - f3 + f5 - f6 - f7 + f8) / rho, uy = (f2 - f4 + f5 + f6 - f7 - f8) / rho;
        if (!(rho > 0.3 && rho < 3) || ux * ux + uy * uy > 0.16) {      // a cell that has gone unstable is reset to the free stream
          f0 = eq0[0]; f1 = eq0[1]; f2 = eq0[2]; f3 = eq0[3]; f4 = eq0[4]; f5 = eq0[5]; f6 = eq0[6]; f7 = eq0[7]; f8 = eq0[8]; rho = 1; ux = u0; uy = 0; this.blown = true; }
        const ux2 = ux * ux, uy2 = uy * uy, u15 = 1.5 * (ux2 + uy2), r9 = rho / 9, r36 = rho / 36, cp = ux + uy, cm = ux - uy;
        const e0 = 4 * r9 * (1 - u15), e1 = r9 * (1 + 3 * ux + 4.5 * ux2 - u15), e3 = r9 * (1 - 3 * ux + 4.5 * ux2 - u15), e2 = r9 * (1 + 3 * uy + 4.5 * uy2 - u15), e4 = r9 * (1 - 3 * uy + 4.5 * uy2 - u15);
        const e5 = r36 * (1 + 3 * cp + 4.5 * cp * cp - u15), e7 = r36 * (1 - 3 * cp + 4.5 * cp * cp - u15), e8 = r36 * (1 + 3 * cm + 4.5 * cm * cm - u15), e6 = r36 * (1 - 3 * cm + 4.5 * cm * cm - u15);
        const d1 = f1 - e1, d2 = f2 - e2, d3 = f3 - e3, d4 = f4 - e4, d5 = f5 - e5, d6 = f6 - e6, d7 = f7 - e7, d8 = f8 - e8, dg = d5 + d6 + d7 + d8;
        const pxx = d1 + d3 + dg, pyy = d2 + d4 + dg, pxy = d5 - d6 + d7 - d8;
        const om = 2 / (tau0 + Math.sqrt(t2 + CS2 * Math.sqrt(pxx * pxx + 2 * pxy * pxy + pyy * pyy) / rho));      // local relaxation rate with the eddy viscosity
        b[k] = f0 - om * (f0 - e0); b[k + 1] = f1 - om * d1; b[k + 2] = f2 - om * d2; b[k + 3] = f3 - om * d3; b[k + 4] = f4 - om * d4;
        b[k + 5] = f5 - om * d5; b[k + 6] = f6 - om * d6; b[k + 7] = f7 - om * d7; b[k + 8] = f8 - om * d8;
        UX[c] = ux; UY[c] = uy; RHO[c] = rho;
      }
    }
    this.a = b; this.b = a; this.steps++; this.forceSteps++;
  }
  /* Mean force on body k since clearForces(), as a coefficient per unit reference length L (cells): F / (0.5 rho u0^2 L). */
  coef(k, L) { const s = Math.max(1, this.forceSteps), q = 0.5 * this.u0 * this.u0 * L; return [this.fx[k] / s / q, this.fy[k] / s / q]; }
}

// ------------------------------------------------------------------ geometry -> grid
const TV = { view: "front", mode: "speed", smoke: true, running: true, speed: 15, dFront: 0, dRear: 0, ride: 0 };
const TUNNEL_VIEWS = {
  front: { name: "Front wing inboard of the middle endplate: F1 + F2 + F3", box: [-1.25, 0.25, 0, 0.62], ground: true, parts: ["front3"] },
  frontOut: { name: "Front wing outboard, ahead of the tire: F1 + F2", box: [-1.25, 0.25, 0, 0.62], ground: true, parts: ["front2"] },
  rear: { name: "Rear wing: R1 + R2 + R3 + R4", box: [0.85, 2.85, 0.4, 1.55], ground: false, parts: ["rear"] },
  centre: { name: "Car centre line: F1, body, driver, rear wing", box: [-1.7, 4.3, 0, 1.9], ground: true, parts: ["front1", "body", "helmet", "rear"] },
  cascade: { name: "0.35 m off centre: front stack and rear wing", box: [-1.7, 4.3, 0, 1.9], ground: true, parts: ["front3", "rear"] },
  wheels: { name: "Through the tires: F1 + F2, front and rear tire", box: [-1.7, 4.3, 0, 1.9], ground: true, parts: ["front2", "wheels"] },
};
const BODY_NAMES = ["", "front wing", "rear wing", "body", "driver's helmet", "front tire", "rear tire"];
/* The shapes of a view as [{id, poly | circle}], with the sliders applied (flap angle, front-wing ride height). */
function tunnelShapes(view) {
  const T = GEO().tunnel, out = [];
  const flaps = (polys, les, deg, dz) => polys.map((p, k) => {
    const a = (k ? deg : 0) * Math.PI / 180, o = les[1] || les[0], c = Math.cos(a), s = Math.sin(a);       // flaps turn together about the first flap's leading edge
    return p.map(([x, z]) => [o[0] + (x - o[0]) * c - (z - o[1]) * s, o[1] + (x - o[0]) * s + (z - o[1]) * c + dz]);
  });
  for (const part of TUNNEL_VIEWS[view].parts) {
    if (part.startsWith("front")) flaps(T.elements.front_wing.slice(0, +part.slice(5)), T.le.front_wing, TV.dFront, TV.ride / 1e3).forEach((p) => out.push({ id: 1, poly: p }));
    else if (part === "rear") flaps(T.elements.rear_wing, T.le.rear_wing, TV.dRear, 0).forEach((p) => out.push({ id: 2, poly: p }));
    else if (part === "body" && T.body) out.push({ id: 3, poly: T.body });
    else if (part === "helmet" && T.helmet) out.push({ id: 4, circle: T.helmet });
    else if (part === "wheels") { const r = T.wheels.radius; out.push({ id: 5, circle: [0, r, r] }, { id: 6, circle: [T.wheels.wheelbase, r, r] }); }
  }
  return out;
}
function inPoly(x, z, p) { let c = false; for (let i = 0, j = p.length - 1; i < p.length; j = i++) if ((p[i][1] > z) !== (p[j][1] > z) && x < (p[j][0] - p[i][0]) * (z - p[i][1]) / (p[j][1] - p[i][1]) + p[i][0]) c = !c; return c; }
function tunnelBuild(nxWanted) {
  const v = TUNNEL_VIEWS[TV.view], [x0, x1, z0, z1] = v.box, dx = (x1 - x0) / nxWanted, nx = nxWanted, ny = Math.round((z1 - z0) / dx);
  const t = new Tunnel(nx, ny); t.dx = dx; t.x0 = x0; t.z0 = z0; t.shapes = tunnelShapes(TV.view);
  for (const s of t.shapes) {
    let bx0, bx1, bz0, bz1;
    if (s.circle) { [bx0, bx1, bz0, bz1] = [s.circle[0] - s.circle[2], s.circle[0] + s.circle[2], s.circle[1] - s.circle[2], s.circle[1] + s.circle[2]]; }
    else { bx0 = Math.min(...s.poly.map((p) => p[0])); bx1 = Math.max(...s.poly.map((p) => p[0])); bz0 = Math.min(...s.poly.map((p) => p[1])); bz1 = Math.max(...s.poly.map((p) => p[1])); }
    const i0 = Math.max(1, Math.floor((bx0 - x0) / dx)), i1 = Math.min(nx - 2, Math.ceil((bx1 - x0) / dx)), j0 = Math.max(1, Math.floor((bz0 - z0) / dx)), j1 = Math.min(ny - 2, Math.ceil((bz1 - z0) / dx));
    for (let j = j0; j <= j1; j++) for (let i = i0; i <= i1; i++) {
      const x = x0 + (i + 0.5) * dx, z = z0 + (j + 0.5) * dx;
      if (s.circle ? (x - s.circle[0]) ** 2 + (z - s.circle[1]) ** 2 <= s.circle[2] ** 2 : inPoly(x, z, s.poly)) t.solid[j * nx + i] = s.id;
    }
  }
  return t;
}

// ------------------------------------------------------------------ drawing
const TCOL = { solid: [0, 14, 47] };
const lerp3 = (a, b, f) => [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f];
const seqBlue = (f) => (f < 0.5 ? lerp3([247, 250, 254], [127, 173, 234], f * 2) : lerp3([127, 173, 234], [20, 70, 160], (f - 0.5) * 2));           // slow = light, fast = blue
const diverge = (f) => (f < 0 ? lerp3([232, 235, 240], [42, 120, 214], Math.min(1, -f)) : lerp3([232, 235, 240], [235, 104, 52], Math.min(1, f)));   // blue below, grey at zero, orange above
function tunnelDraw(t, ctx, img) {
  const { nx, ny, ux, uy, rho, solid, u0 } = t, d = img.data;
  for (let j = 0; j < ny; j++) for (let i = 0; i < nx; i++) {
    const c = j * nx + i, o = 4 * ((ny - 1 - j) * nx + i);
    let col;
    if (solid[c]) col = TCOL.solid;
    else if (TV.mode === "speed") col = seqBlue(Math.min(1, Math.hypot(ux[c], uy[c]) / (1.7 * u0)));
    else if (TV.mode === "pressure") col = diverge((rho[c] - 1) / 3 / (0.5 * u0 * u0) / 1.6);                    // pressure coefficient, +-1.6 full scale
    else { const cu = i > 0 && i < nx - 1 && j > 0 && j < ny - 1 ? (uy[c + 1] - uy[c - 1] - ux[c + nx] + ux[c - nx]) * 0.5 : 0; col = diverge(cu / (0.12 * u0)); }
    d[o] = col[0]; d[o + 1] = col[1]; d[o + 2] = col[2]; d[o + 3] = 255;
  }
  ctx.putImageData(img, 0, 0);
}
/* Smoke: particles released from a rake at the inlet and carried by the flow. */
class Smoke {
  constructor(t, rows = 26) { this.t = t; this.rows = rows; this.p = []; this.tick = 0; }
  step(n) {
    const { nx, ny, ux, uy, solid } = this.t, p = this.p;
    if (this.tick++ % 2 === 0) for (let r = 0; r < this.rows; r++) p.push(1.5, (r + 0.5) / this.rows * (ny - 2) + 1);
    let k = 0;
    for (let q = 0; q < p.length; q += 2) {
      let x = p[q], y = p[q + 1];
      const c = (y | 0) * nx + (x | 0);
      x += ux[c] * n; y += uy[c] * n;
      if (x < 1 || x >= nx - 1 || y < 1 || y >= ny - 1 || solid[(y | 0) * nx + (x | 0)]) continue;
      p[k++] = x; p[k++] = y;
    }
    p.length = Math.min(k, 16000);
  }
  draw(ctx, w, h) {
    const { nx, ny } = this.t, sx = w / nx, sy = h / ny, p = this.p;
    ctx.clearRect(0, 0, w, h); ctx.fillStyle = TV.mode === "speed" ? "rgba(0,14,47,.55)" : "rgba(0,14,47,.42)";
    const r = Math.max(1, w / 520);
    for (let q = 0; q < p.length; q += 2) ctx.fillRect(p[q] * sx, h - p[q + 1] * sy, r, r);
  }
}

// ------------------------------------------------------------------ the card
V("aero", "tunnel", "Virtual wind tunnel (2-D)", "Air flowing over a side-view slice of the car, solved live in this page. Pick a slice, change a wing, and watch the flow and the loads move.", (b, ctx) => {
  const G = need(GEO().tunnel); if (G.error) throw new Error(G.error);
  const small = window.innerWidth < 700, NX = small ? 240 : 380;
  const wrap = el("div", { class: "tunnel" }), field = el("canvas", { class: "tfield" }), smokeC = el("canvas", { class: "tsmoke" }), tiles = el("div", { class: "kpis" }), legend = el("div", { class: "legend" }), info = el("div", { class: "sub" });
  wrap.append(field, smokeC);
  let t, img, smoke, fctx, sctx, raf = 0, visible = true, lastTiles = 0, stepsPerFrame = 3;
  const build = () => {
    t = tunnelBuild(NX); field.width = t.nx; field.height = t.ny; wrap.style.aspectRatio = `${t.nx} / ${t.ny}`;
    fctx = field.getContext("2d"); img = fctx.createImageData(t.nx, t.ny); smoke = new Smoke(t, Math.max(12, Math.round(t.ny / 6)));
    const v = TUNNEL_VIEWS[TV.view];
    info.textContent = `${v.name}. Window ${(v.box[1] - v.box[0]).toFixed(1)} m long, ${(t.dx * 1e3).toFixed(1)} mm per cell.` + (t.dx > 0.008 ? " The slots between wing elements are narrower than a cell here: use a wing view to see slot flow." : "");
    showLegend(); lastTiles = 0; tunnelDraw(t, fctx, img); showTiles();      // the shape shows at once, before the flow starts moving
  };
  const showLegend = () => {
    const sw = (c, l) => el("span", {}, el("i", { style: `background:rgb(${c.map(Math.round).join(",")})` }), l);
    legend.replaceChildren(...(TV.mode === "speed" ? [sw(seqBlue(0), "still"), sw(seqBlue(0.59), "free-stream speed"), sw(seqBlue(1), "1.7 × free stream")]
      : TV.mode === "pressure" ? [sw(diverge(-1), "suction"), sw(diverge(0), "free-stream pressure"), sw(diverge(1), "high pressure")] : [sw(diverge(-1), "spinning one way"), sw(diverge(0), "no spin"), sw(diverge(1), "spinning the other way")]), sw(TCOL.solid, "the car"));
  };
  const showTiles = () => {
    const ids = [...new Set(t.shapes.map((s) => s.id))], rho = GEO().air_density || 1.12, q = 0.5 * rho * TV.speed ** 2, out = [];
    for (const k of ids) { const [cd, cl] = t.coef(k, 1 / t.dx);                 // per metre of reference length
      out.push(tile(`${BODY_NAMES[k]}: downforce`, fmt(-cl * q, 3), "N per m width", `coefficient × 1 m: ${fmt(-cl, 3)}`), tile(`${BODY_NAMES[k]}: drag`, fmt(cd * q, 3), "N per m width", `coefficient × 1 m: ${fmt(cd, 3)}`)); }
    tiles.replaceChildren(...out);
    if (t.blown) info.textContent = "Part of the flow went unstable and was reset. Press Restart the flow; if it happens again, reduce the flap angle.";
  };
  const frame = (now) => {
    raf = 0;
    if (!field.isConnected) return;                       // the card was closed or the page changed
    if (TV.running && visible && !document.hidden) {
      const t0 = performance.now();
      for (let k = 0; k < stepsPerFrame; k++) t.step();
      const ms = performance.now() - t0;
      stepsPerFrame = Math.max(1, Math.min(10, Math.round(stepsPerFrame * (ms > 26 ? 0.8 : ms < 12 ? 1.25 : 1) + (ms < 12 ? 0.5 : 0))));
      if (t.steps === 1500) t.clearForces();              // forget the start-up transient
      tunnelDraw(t, fctx, img);
      const w = smokeC.clientWidth, h = smokeC.clientHeight, dpr = Math.min(2, window.devicePixelRatio || 1);
      if (smokeC.width !== Math.round(w * dpr)) { smokeC.width = Math.round(w * dpr); smokeC.height = Math.round(h * dpr); sctx = smokeC.getContext("2d"); }
      if (TV.smoke) { smoke.step(stepsPerFrame); smoke.draw(sctx, smokeC.width, smokeC.height); } else if (sctx) sctx.clearRect(0, 0, smokeC.width, smokeC.height);
      if (now - lastTiles > 700) { lastTiles = now; showTiles(); }
    }
    raf = requestAnimationFrame(frame);
  };
  const io = new IntersectionObserver((e) => { visible = e[0].isIntersecting; }); io.observe(wrap);
  const sel = (label, value, opts, on) => { const s = el("select", { "aria-label": label, onchange: () => on(s.value) }, opts.map(([v, n]) => el("option", { value: v }, n))); s.value = value; return s; };
  const slider = (label, key, lo, hi, step, unit, rebuild) => { const r = el("input", { type: "range", min: lo, max: hi, step, value: TV[key], "aria-label": label, style: "width:130px" }), o = el("b", {}, `${TV[key]} ${unit}`);
    r.addEventListener("input", () => { TV[key] = +r.value; o.textContent = `${TV[key]} ${unit}`; if (!rebuild) showTiles(); });
    r.addEventListener("change", () => { if (rebuild) build(); });
    return el("label", { class: "tslider" }, label, r, o); };
  const run = el("button", { class: "btn navy small", onclick: () => { TV.running = !TV.running; run.textContent = TV.running ? "Pause" : "Run"; } }, TV.running ? "Pause" : "Run");
  const ctl = el("div", { class: "ctl" }, sel("Slice", TV.view, Object.entries(TUNNEL_VIEWS).map(([k, v]) => [k, v.name]), (v) => { TV.view = v; build(); }),
    sel("Colour", TV.mode, [["speed", "Colour: air speed"], ["pressure", "Colour: pressure"], ["spin", "Colour: spin (vortices)"]], (v) => { TV.mode = v; showLegend(); tunnelDraw(t, fctx, img); }),
    el("label", { class: "check" }, el("input", { type: "checkbox", checked: TV.smoke, onchange: (e) => { TV.smoke = e.target.checked; } }), "Smoke"), run,
    el("button", { class: "btn small", onclick: () => build() }, "Restart the flow"));
  const ctl2 = el("div", { class: "ctl" }, slider("Front flaps", "dFront", -12, 12, 1, "deg", true), slider("Rear flaps", "dRear", -12, 12, 1, "deg", true), slider("Front wing height", "ride", -40, 60, 5, "mm", true), slider("Car speed", "speed", 5, 35, 1, "m/s", false));
  build();
  b.append(ctl, ctl2, wrap, legend, info, tiles, el("div", { class: "note warn" }, "Use this for flow patterns and for which way a change moves the loads, not for the car's forces. It is a two-dimensional slice (no wing tips, endplates or flow around the sides) at a Reynolds number hundreds of times below the car's, so separation and the size of the forces differ from the real car. The ground moves with the air, as on track; the tires are not spinning. Loads are per metre of width and settle after a few seconds."));
  raf = requestAnimationFrame(frame);
}, { wide: true, first: true });
// the tunnel sits just below the load-position figure on the Aero page
{ const i = VISUALS.findIndex((v) => v.id === "tunnel"), m = VISUALS.splice(i, 1)[0]; VISUALS.splice(VISUALS.findIndex((v) => v.id === "aero_layout") + 1, 0, m); }

// ------------------------------------------------------------------ 3-D flow replay (saved fsae_cfd runs)
/* Replays the 3-D CFD runs exported by web/export_cfd.py (data/cfd/*.json): the car's surface coloured by pressure,
   streamlines of the time-averaged flow, and smoke carried along them. Nothing is solved in the page. */
const CF = { state: "base", cache: {}, hide: new Set(["frame", "suspension"]), play: window.innerWidth >= 900, lines: true, view: "front34" };
const CF_VIEWS = { front34: ["3/4 front left", { x: -0.82, y: -0.86, z: 0.42 }], rear34: ["3/4 rear left", { x: 0.9, y: -0.86, z: 0.46 }], side: ["Side", { x: 0.03, y: -1.35, z: 0.14 }], top: ["Top", { x: 0.01, y: -0.01, z: 1.5 }], front: ["Front", { x: -1.35, y: 0, z: 0.2 }] };
const CF_CP = [[0, "#1f5fbf"], [0.5, "#7fadea"], [0.7143, "#e8ebf0"], [1, "#eb6834"]];      // Cp from -2.5 (suction, blue) through 0 (grey) to +1 (orange)
V("aero", "cfd3d", "3-D flow replay", "The saved 3-D CFD runs of the whole car: surface pressure, streamlines and smoke. Drag to rotate, scroll or pinch to zoom.", (b, ctx) => {
  const holder = el("div"), p = el("div", { class: "plot cfd3d" }), tiles = el("div", { class: "kpis" }), ctl = el("div", { class: "ctl" }), ctl2 = el("div", { class: "ctl" }), info = el("div", { class: "sub" }), tbl = el("details", {});
  b.append(holder); holder.append(el("div", { class: "muted" }, "Loading the CFD runs…"));
  let timer = 0, data = null, visible = true, tau = 0, smokeAt = -1;
  const load = async (state) => {
    if (!CF.index) CF.index = await getJSON("data/cfd/index.json");
    if (!CF.faces) CF.faces = (await getJSON("data/cfd/faces.json")).faces;
    if (!CF.cache[state]) {
      const d = await getJSON(`data/cfd/${CF.index.cases.find((c) => c.state === state).file}`), n = d.vertices.length / 3, f = d.faces || CF.faces;
      d.x = new Float32Array(n); d.y = new Float32Array(n); d.z = new Float32Array(n); d.c = new Float32Array(n);
      for (let k = 0; k < n; k++) { d.x[k] = d.vertices[3 * k] / 1e3; d.y[k] = d.vertices[3 * k + 1] / 1e3; d.z[k] = d.vertices[3 * k + 2] / 1e3; d.c[k] = d.cp[k] / 100; }
      d.tri = {};
      for (const [g, [f0, nf]] of Object.entries(d.groups)) { const i = new Int32Array(nf), j = new Int32Array(nf), k = new Int32Array(nf);
        for (let q = 0; q < nf; q++) { i[q] = f[3 * (f0 + q)]; j[q] = f[3 * (f0 + q) + 1]; k[q] = f[3 * (f0 + q) + 2]; } d.tri[g] = { i, j, k }; }
      const lx = [], ly = [], lz = [], lc = [];
      for (const L of d.lines) { for (let q = 0; q < L.s.length; q++) { lx.push(L.p[3 * q] / 1e3); ly.push(L.p[3 * q + 1] / 1e3); lz.push(L.p[3 * q + 2] / 1e3); lc.push(L.s[q] / 100); } lx.push(null); ly.push(null); lz.push(null); lc.push(0); }
      d.lx = lx; d.ly = ly; d.lz = lz; d.lc = lc; delete d.vertices; delete d.cp;
      CF.cache[state] = d;
    }
    return CF.cache[state];
  };
  /* Smoke positions at replay time tau [ms of flow time]: a few puffs per streamline, a fixed time apart. */
  const smoke = () => {
    const xs = [], ys = [], zs = [], gap = 120;
    for (const L of data.lines) { const T = L.t[L.t.length - 1]; if (T < gap) continue;
      for (let t = tau % gap; t < T; t += gap) { let q = 1; while (q < L.t.length - 1 && L.t[q] < t) q++;
        const f = (t - L.t[q - 1]) / Math.max(1, L.t[q] - L.t[q - 1]), a = 3 * (q - 1), c = 3 * q;
        xs.push((L.p[a] + f * (L.p[c] - L.p[a])) / 1e3); ys.push((L.p[a + 1] + f * (L.p[c + 1] - L.p[a + 1])) / 1e3); zs.push((L.p[a + 2] + f * (L.p[c + 2] - L.p[a + 2])) / 1e3); } }
    return [xs, ys, zs];
  };
  const draw = () => {
    const d = data, traces = [], ax = { visible: false, showgrid: false, zeroline: false, showbackground: false };
    let first = true;
    for (const g of Object.keys(d.groups)) { if (CF.hide.has(g)) continue;
      traces.push({ type: "mesh3d", x: d.x, y: d.y, z: d.z, i: d.tri[g].i, j: d.tri[g].j, k: d.tri[g].k, intensity: d.c, cmin: -2.5, cmax: 1, colorscale: CF_CP, name: pretty(g), showscale: first, flatshading: false,
        lighting: { ambient: 0.75, diffuse: 0.6, specular: 0.08, roughness: 0.9 }, hovertemplate: pretty(g) + "<br>Cp %{intensity:.2f}<extra></extra>",
        colorbar: { title: { text: "pressure coefficient", side: "right", font: { size: 11 } }, thickness: 10, len: 0.6, x: 1 } }); first = false; }
    traces.push({ type: "mesh3d", x: [-1.8, 4.0, 4.0, -1.8], y: [-1.2, -1.2, 1.2, 1.2], z: [0, 0, 0, 0], i: [0, 0], j: [1, 2], k: [2, 3], color: "#e9edf4", opacity: 0.55, hoverinfo: "skip", showscale: false, name: "ground" });
    if (CF.lines) traces.push({ type: "scatter3d", mode: "lines", x: d.lx, y: d.ly, z: d.lz, line: { width: 1.6, color: d.lc, colorscale: SEQ, cmin: 0.2, cmax: 1.5 }, opacity: 0.38, hoverinfo: "skip", name: "streamlines", showlegend: false });
    const [sx, sy, sz] = smoke(); smokeAt = traces.length;
    traces.push({ type: "scatter3d", mode: "markers", x: sx, y: sy, z: sz, marker: { size: 1.9, color: NAVY, opacity: 0.7 }, hoverinfo: "skip", name: "smoke", showlegend: false });
    Plotly.react(p, traces, { margin: { l: 0, r: 0, t: 0, b: 0 }, paper_bgcolor: "#fff", showlegend: false, uirevision: "cfd",
      scene: { aspectmode: "data", xaxis: ax, yaxis: ax, zaxis: ax, camera: { eye: CF_VIEWS[CF.view][1], up: { x: 0, y: 0, z: 1 }, center: { x: 0.02, y: 0, z: -0.12 } }, dragmode: "orbit" } },
      { responsive: true, displaylogo: false, displayModeBar: false, scrollZoom: true });
  };
  const show = () => {
    const d = data, s = d.summary, base = (CF.cache.base || {}).summary, T = (name, key, unit, k = 1, dg = 0) => { const v = s[key]; if (!v || v[0] == null) return null;
      const dl = base && d.state !== "base" && base[key] ? delta(v[0] * k, base[key][0] * k, dg, "") : ""; return tile(name, (v[0] * k).toFixed(dg), unit, dl || (v[1] != null ? `± ${(v[1] * k).toFixed(Math.max(dg, 1))} (95 % interval)` : "")); };
    tiles.replaceChildren(...[T("Downforce", "downforce_N", "N"), T("Drag", "drag_N", "N"), T("Front share of downforce", "front_balance", "%", 100, 1), T("Downforce coefficient × area", "ClA_m2", "m²", 1, 3), T("Drag coefficient × area", "CdA_m2", "m²", 1, 3), T("Side force", "side_N", "N")].filter(Boolean));
    info.textContent = `${d.label}. Air speed ${d.speed.toFixed(1)} m/s, ${d.resolution} grid (${d.cell_mm} mm cells near the car, ${(d.cells / 1e6).toFixed(1)} million cells). Pressure and streamlines are the time-averaged flow of that run.`;
    tbl.replaceChildren(el("summary", { class: "muted", style: "cursor:pointer;margin:8px 0" }, "Loads by part of the car"), table(d.group_table));
  };
  const controls = () => {
    const sel = el("select", { "aria-label": "CFD run", onchange: () => go(sel.value) }, CF.index.cases.map((c) => el("option", { value: c.state }, c.label))); sel.value = CF.state;
    const play = el("button", { class: "btn navy small", onclick: () => { CF.play = !CF.play; play.textContent = CF.play ? "Pause the smoke" : "Play the smoke"; } }, CF.play ? "Pause the smoke" : "Play the smoke");
    ctl.replaceChildren(sel, ...Object.entries(CF_VIEWS).map(([k, [name, eye]]) => el("button", { class: "btn small", onclick: () => { CF.view = k; Plotly.relayout(p, { "scene.camera": { eye, up: { x: 0, y: 0, z: 1 }, center: { x: 0.02, y: 0, z: -0.12 } } }); } }, name)), play);
    ctl2.replaceChildren(el("span", { class: "lbl", style: "font-size:11px;text-transform:uppercase;letter-spacing:.1em;color:var(--ink-3);font-weight:700" }, "Show"),
      ...Object.keys(data.groups).map((g) => el("button", { class: "chip" + (CF.hide.has(g) ? "" : " on"), onclick: (e) => { CF.hide.has(g) ? CF.hide.delete(g) : CF.hide.add(g); e.currentTarget.classList.toggle("on"); draw(); } }, pretty(g))),
      el("button", { class: "chip" + (CF.lines ? " on" : ""), onclick: (e) => { CF.lines = !CF.lines; e.currentTarget.classList.toggle("on"); draw(); } }, "streamlines"));
  };
  const go = async (state) => {
    try { setStatus("Loading the CFD run…", "busy"); data = await load(state); if (state !== "base" && !CF.cache.base) await load("base"); CF.state = state;
      if (!holder.contains(p)) holder.replaceChildren(ctl, ctl2, tiles, p, info, tbl,
        el("div", { class: "note warn" }, "These runs were solved with the wing CAD as it is, which includes a fourth front flap (F4) that is not on the car: re-solve them without it for the car as built. This replays runs that were solved beforehand with the team's fsae_cfd solver; nothing is solved here, so the car and the conditions are the ones of each run. To add a run, solve it in the notebook and run web/export_cfd.py. Each run is one speed and attitude on a laptop-size grid: use the differences between runs before the absolute numbers."));
      controls(); draw(); show(); setStatus("CFD run loaded.", "ok"); }
    catch (e) { holder.replaceChildren(el("div", { class: "err" }, /missing|HTTP|empty/i.test(String(e.message)) ? "No CFD runs are in this build of the site. Solve them with fsae_cfd, then run python web/export_cfd.py and python web/build.py." : "The CFD run could not be shown: " + (e.message || e))); setStatus("Showing the baseline (pre-computed).", "ok"); }
  };
  const io = new IntersectionObserver((e) => { visible = e[0].isIntersecting; }); io.observe(holder);
  timer = setInterval(() => {
    if (!holder.isConnected) return clearInterval(timer);
    if (!CF.play || !visible || document.hidden || !data || smokeAt < 0 || !p.data) return;
    tau += 4;                                             // 4 ms of flow per frame: about a fifteenth of real speed
    const [x, y, z] = smoke(); Plotly.restyle(p, { x: [x], y: [y], z: [z] }, [smokeAt]);
  }, 70);
  go(CF.state);
}, { wide: true, first: true });
{ const i = VISUALS.findIndex((v) => v.id === "cfd3d"), m = VISUALS.splice(i, 1)[0]; VISUALS.splice(VISUALS.findIndex((v) => v.id === "tunnel") + 1, 0, m); }
