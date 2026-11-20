/* Working on the whole car together: what was changed and what it did (Full car simulation), publishing decisions to
   the team (Team), files in and out (Import & export), and comparing with the built car and optimising (Optimization).
   Loaded last; uses the helpers in app.js / editors.js / wings.js. */
"use strict";

PAGES.splice(PAGES.findIndex((p) => p.id === "studies") + 1, 0,
  { id: "optimise", label: "Optimization", intro: "Set the prediction against what the car did on track, find which inputs matter, and let the optimiser search for a better set-up. These use the lap simulation's own tools and run in your browser." },
  { id: "data", label: "Import & export", intro: "Take the changed car out to CAD and the config files, and bring CAD geometry and analysis results in." },
  { id: "team", label: "Team", intro: "Design decisions the whole team shares: car designs, the driver list and corner assemblies. Publishing here changes what everyone sees; running a simulation never does." });

const download = (name, text, type = "text/plain") => { const a = el("a", { href: URL.createObjectURL(new Blob([text], { type })), download: name }); document.body.append(a); a.click(); a.remove(); };
const csvCell = (v) => (v == null ? "" : /[",\n]/.test(String(v)) ? `"${String(v).replace(/"/g, '""')}"` : String(v));
const csvOf = (cols, rows) => [cols, ...rows].map((r) => r.map(csvCell).join(",")).join("\n") + "\n";
const readFile = (input) => new Promise((res, rej) => { const f = input.files && input.files[0]; if (!f) return rej(new Error("no file chosen")); const r = new FileReader(); r.onload = () => res({ name: f.name, text: String(r.result) }); r.onerror = () => rej(new Error("the file could not be read")); r.readAsText(f); });
const GROUP_NAMES = { chassis: "Chassis & frame", tires: "Tires & grip", suspension: "Suspension", aero: "Aero", brakes: "Brakes", powertrain: "Powertrain", cooling: "Cooling & fuel", accumulator: "Accumulator", events: "Events & scoring" };
const dirty = () => !same(S.overrides[S.car], S.applied[S.car]);
const totalPoints = (ev) => Object.values(ev).reduce((t, d) => t + (d.points || 0), 0);

// ------------------------------------------------------------------ full car: what changed, and what it did
V("overview", "changes", "Changes in this car", "Everything changed from the baseline, in every section. All of it goes into the next run.", (b) => {
  const ov = OV(), rows = [];
  for (const [k, v] of Object.entries(ov)) {
    if (k.startsWith("geom:")) {
      const what = k === "geom:frame" ? ["Chassis & frame", "Frame geometry", `${v.nodes.length} joints, ${v.tubes.length} tubes`] : k === "geom:hardpoints" ? ["Suspension", "Hard points", Object.entries(v).map(([a, p]) => `${a}: ${Object.keys(p).length} moved`).join(", ")]
        : k === "geom:tracks" ? ["Events & scoring", "Event tracks", Object.entries(v).map(([e, f]) => `${e}: ${f.replace(/\.csv$/, "")}`).join(", ")] : ["Other", k, "edited"];
      rows.push([what[0], what[1], "as designed", what[2], k]); continue;
    }
    const i = inputOf(k); if (!i) continue;
    const show = (x) => (x == null ? "not set" : Array.isArray(x) ? x.map((q) => fmt(q)).join(", ") : `${fmt(x, 5)}${i.unit ? " " + i.unit : ""}`);
    rows.push([GROUP_NAMES[i.group] || i.group, k.split(":")[1], show(i.value), show(v), k]);
  }
  rows.sort((a, c) => a[0].localeCompare(c[0]) || a[1].localeCompare(c[1]));
  const wings = typeof WD !== "undefined" && WD.edited();
  if (!rows.length) return b.append(el("div", { class: "muted" }, "Nothing is changed: this is the baseline car." + (wings ? " The wing designer holds a changed wing that has not been used in the car yet (Aero → Wing designer)." : " Change anything in any section, then come back here and press Run simulation.")));
  b.append(el("div", { class: "tablewrap" }, el("table", { class: "edit" }, el("thead", {}, el("tr", {}, ["section", "input", "baseline", "this car", ""].map((h) => el("th", {}, h)))),
    el("tbody", {}, rows.map((r) => el("tr", {}, el("td", {}, r[0]), el("td", { style: "font-family:var(--mono);font-size:12px" }, r[1]), el("td", {}, r[2]), el("td", {}, el("b", {}, r[3])),
      el("td", {}, el("button", { class: "btn small", title: "Put this one back to the baseline", onclick: () => { delete OV()[r[4]]; if (r[4] === "geom:frame" && typeof FR !== "undefined") delete FR._w[S.car]; render(); pending(); } }, "Undo"))))))),
    el("div", { class: "ctl", style: "margin-top:8px" }, el("span", { class: "muted" }, `${rows.length} change${rows.length === 1 ? "" : "s"}.` + (dirty() ? " The results below are from before the latest of them: press Run simulation." : " The results on this page include all of them.")),
      el("button", { class: "btn small", onclick: () => { S.overrides[S.car] = {}; if (typeof FR !== "undefined") delete FR._w[S.car]; render(); pending(); } }, "Undo all"),
      el("button", { class: "btn publish small", onclick: () => teamPublishDesign() }, "Publish this car to the team…")),
    wings ? el("div", { class: "note warn" }, "The wing designer holds a changed wing. It reaches the lap simulation only after “Use this wing in the full car simulation” on the Aero page.") : null);
}, { wide: true });
V("overview", "compare", "This car against the baseline", "Every event, for the inputs of the last run.", (b) => {
  const r = R(), base = S.base[S.car], kept = JSON.parse(localStorage.getItem("lapsim.kept." + S.car) || "[]"), evs = ["acceleration", "skidpad", "autocross", "endurance"].filter((e) => r.events[e] || base.events[e]);
  const t = (d, e) => (d && d[e] && d[e].time_s != null ? d[e].time_s : null), p = (d, e) => (d && d[e] && d[e].points != null ? d[e].points : null);
  const cols = ["event", "baseline [s]", "this car [s]", "change [s]", "baseline [pts]", "this car [pts]", "change [pts]", ...kept.map((k) => `${k.name} [s]`)];
  const rows = evs.map((e) => [EVENT_NAMES[e], t(base.events, e), t(r.events, e), t(r.events, e) != null && t(base.events, e) != null ? +(t(r.events, e) - t(base.events, e)).toFixed(3) : null, p(base.events, e), p(r.events, e),
    p(r.events, e) != null && p(base.events, e) != null ? +(p(r.events, e) - p(base.events, e)).toFixed(2) : null, ...kept.map((k) => t(k.events, e))]);
  const tb = totalPoints(base.events), tr = totalPoints(r.events);
  rows.push(["Dynamic events, total points", null, null, null, tb ? +tb.toFixed(1) : null, tr ? +tr.toFixed(1) : null, tb && tr ? +(tr - tb).toFixed(2) : null, ...kept.map(() => null)]);
  const n = Object.keys(S.applied[S.car]).length;
  b.append(table({ columns: cols, rows }), el("div", { class: "ctl", style: "margin-top:8px" },
    el("span", { class: "muted" }, n ? `This car has ${n} change${n === 1 ? "" : "s"} from the baseline.` + (dirty() ? " Newer changes are waiting for Run simulation." : "") : dirty() ? "Changes are waiting: press Run simulation to see what they do." : "This is the baseline car."),
    el("button", { class: "btn small", title: "Keep these times in the table to compare with later runs (in this browser)", onclick: () => { const name = prompt("Name for this result (for example: softer rear springs)"); if (!name) return; kept.push({ name: name.slice(0, 40), events: r.events, changes: n }); localStorage.setItem("lapsim.kept." + S.car, JSON.stringify(kept.slice(-6))); render(); } }, "Keep this result to compare"),
    kept.length ? el("button", { class: "btn small", onclick: () => { localStorage.removeItem("lapsim.kept." + S.car); render(); } }, "Clear kept results") : null));
}, { wide: true });
{ // these two lead the full-car page, right under the event results
  for (const id of ["compare", "changes"]) { const i = VISUALS.findIndex((v) => v.id === id), m = VISUALS.splice(i, 1)[0]; VISUALS.splice(VISUALS.findIndex((v) => v.id === "kpi") + 1, 0, m); } }

// ------------------------------------------------------------------ team: publishing through the GitHub repository
/* Team data is a folder of JSON files in the site's repository (data/team/). Reading needs nothing. Publishing is a
   commit made with the GitHub API using the publisher's own access token, so GitHub decides who may do it (people
   with write access to the repository), and every save is in the repository's history with a name on it. */
const TEAM = {
  repo: "uconnformulasae/PLS", branch: "web/lapsim-site", dir: "web/site/data/team",
  index: null, user: null, canPush: false, adoptedLoaded: {},
  token() { return sessionStorage.getItem("lapsim.gh") || localStorage.getItem("lapsim.gh") || ""; },
  empty: () => ({ drivers: null, corners: [], designs: [], adopted: {}, log: [] }),
  async api(method, path, body, raw) {
    const h = { Accept: raw ? "application/vnd.github.raw+json" : "application/vnd.github+json", "X-GitHub-Api-Version": "2022-11-28" };
    if (this.token()) h.Authorization = "Bearer " + this.token();
    const r = await fetch("https://api.github.com" + path, { method, headers: h, body: body ? JSON.stringify(body) : undefined, cache: "no-store" });
    if (r.status === 404 && method === "GET") return null;
    if (!r.ok) { let m = ""; try { m = (await r.json()).message; } catch (e) { m = ""; } throw new Error(r.status === 401 ? "GitHub did not accept the token" : r.status === 403 ? "GitHub refused: " + (m || "no permission, or too many requests") : r.status === 409 ? "someone else published at the same moment: try again" : `GitHub error ${r.status}${m ? ": " + m : ""}`); }
    return raw ? r.text() : r.json();
  },
  path(f) { return `/repos/${this.repo}/contents/${this.dir}/${f}`; },
  /* The newest team data: straight from the repository when it can be reached, else the copy published with the site. */
  async load(force) {
    if (this.index && !force) return this.index;
    let idx = null;
    try { const t = await this.api("GET", this.path("index.json") + "?ref=" + encodeURIComponent(this.branch), null, true); if (t) idx = JSON.parse(t); } catch (e) { idx = null; }
    if (!idx) { try { const r = await fetch("data/team/index.json", { cache: "no-store" }); if (r.ok) idx = await r.json(); } catch (e) { idx = null; } }
    this.index = Object.assign(this.empty(), idx || {});
    return this.index;
  },
  async file(f) {
    try { const t = await this.api("GET", this.path(f) + "?ref=" + encodeURIComponent(this.branch), null, true); if (t) return JSON.parse(t); } catch (e) { /* fall back to the site's copy */ }
    const r = await fetch("data/team/" + f, { cache: "no-store" }); if (!r.ok) throw new Error(f + " is not on the site yet"); return r.json();
  },
  async put(f, obj, message) {
    const cur = await this.api("GET", this.path(f) + "?ref=" + encodeURIComponent(this.branch));
    const text = JSON.stringify(obj, null, f === "index.json" ? 1 : 0), b64 = this.b64(text);
    return this.api("PUT", this.path(f), { message, content: b64, branch: this.branch, sha: cur ? cur.sha : undefined });
  },
  b64(text) { const bytes = new TextEncoder().encode(text); let s = ""; for (let i = 0; i < bytes.length; i += 0x8000) s += String.fromCharCode.apply(null, bytes.subarray(i, i + 0x8000)); return btoa(s); },
  async signIn(token, remember) {
    sessionStorage.setItem("lapsim.gh", token); if (remember) localStorage.setItem("lapsim.gh", token); else localStorage.removeItem("lapsim.gh");
    try { const u = await this.api("GET", "/user"), r = await this.api("GET", `/repos/${this.repo}`); this.user = u.login; this.canPush = !!(r && r.permissions && r.permissions.push); }
    catch (e) { this.signOut(); throw e; }
    return this.user;
  },
  signOut() { sessionStorage.removeItem("lapsim.gh"); localStorage.removeItem("lapsim.gh"); this.user = null; this.canPush = false; },
  /* Change the shared index: always from the newest copy, so two people publishing do not undo each other. */
  async change(message, fn) {
    if (!this.token()) throw new Error("sign in on the Team page first");
    const idx = Object.assign(this.empty(), await this.load(true));
    await fn(idx);
    idx.log = [{ what: message, by: this.user || "?", date: new Date().toISOString().slice(0, 16).replace("T", " ") }, ...(idx.log || [])].slice(0, 40);
    await this.put("index.json", idx, `Team: ${message}` + (this.user ? ` (${this.user})` : ""));
    this.index = idx; return idx;
  },
};
const stamp = () => ({ by: TEAM.user || "?", date: new Date().toISOString().slice(0, 10) });
const teamBusy = async (what, fn) => { try { setStatus(what + "…", "busy"); const r = await fn(); setStatus(what + ": done. Everyone gets it the next time they open the site.", "ok"); return r; } catch (e) { setStatus(what + " failed: " + (e.message || e), "err"); alert(what + " failed.\n\n" + (e.message || e)); return null; } };
async function teamPublishDesign() {
  if (!TEAM.token()) { alert("Publishing changes the car everyone sees, so it needs a sign-in. Open the Team page and sign in with GitHub first."); S.page = "team"; location.hash = `${S.car}/team`; return render(); }
  if (dirty()) return alert("Press Run simulation first, so the published design carries its results.");
  const n = Object.keys(S.applied[S.car]).length, name = prompt(`Publish this ${S.car.toUpperCase()} car (${n} change${n === 1 ? "" : "s"} from the baseline) to the team.\n\nName of the design:`); if (!name) return;
  const note = prompt("One line on why (the design decision):", "") || "";
  if (!confirm(`Publish “${name}” for everyone?\n\nIt will appear in the team's list for all users. It does not replace the team's adopted car until someone adopts it.`)) return;
  const id = `${S.car}_${Date.now().toString(36)}`;
  await teamBusy("Publishing the design", async () => {
    await TEAM.put(`designs/${id}.json`, { car: S.car, name, overrides: S.applied[S.car], wings: typeof WD !== "undefined" ? WD.load() : null, result: R() }, `Team: design ${name}`);
    await TEAM.change(`design “${name}” published`, (idx) => { idx.designs.push(Object.assign({ id, car: S.car, name, note, changes: n, events: R().events }, stamp())); });
  });
  if (S.page === "team") render();
}
async function teamLoadDesign(d, quiet) {
  const f = await TEAM.file(`designs/${d.id}.json`), car = f.car;
  S.overrides[car] = Object.assign({}, f.overrides); S.applied[car] = Object.assign({}, f.overrides);
  if (f.result) S.data[car].result = f.result;
  if (f.wings && typeof WD !== "undefined" && car === S.car) WD.set(f.wings);
  if (typeof FR !== "undefined") delete FR._w[car];
  if (!quiet) setStatus(`Team design “${d.name}” loaded with its results (published by ${d.by}, ${d.date}).`, "ok");
}
/* Called when a car is opened: the team's driver list, and the team's adopted car if this visitor has not started their own. */
async function teamOnCar(car) {
  let idx; try { idx = await TEAM.load(); } catch (e) { return; }
  if (idx.drivers && idx.drivers.drivers && !localStorage.getItem("lapsim.drivers")) { S.driverFile = idx.drivers; if (typeof DR !== "undefined") DR.data = null; }
  const id = idx.adopted && idx.adopted[car], d = id && idx.designs.find((x) => x.id === id);
  if (d && !TEAM.adoptedLoaded[car] && !Object.keys(S.overrides[car]).length && !Object.keys(S.applied[car]).length) {
    try { await teamLoadDesign(d, true); TEAM.adoptedLoaded[car] = d; setStatus(`Showing the team's car: “${d.name}” (adopted ${d.adopted_date || d.date}). The baseline columns are the config files' car.`, "ok"); } catch (e) { /* the design file is not on the site yet */ }
  }
}
/* A chip on every page while the car on screen is the team's adopted design. */
function teamBanner() {
  const d = TEAM.adoptedLoaded[S.car]; if (!d) return null;
  const mine = !same(S.overrides[S.car], S.applied[S.car]);
  return el("button", { class: "driverchip", title: "Open the Team page", onclick: () => { S.page = "team"; location.hash = `${S.car}/team`; render(); } }, "Team car: ", el("b", {}, d.name), mine ? " + your changes" : "");
}
V("team", "team_sign", "Publishing to the team", "Reading the team's data needs nothing. Publishing needs a GitHub sign-in, and GitHub decides who may publish.", (b) => {
  const box = el("div"); b.append(box);
  const draw = () => {
    if (TEAM.user) return box.replaceChildren(el("p", {}, "Signed in as ", el("b", {}, TEAM.user), TEAM.canPush ? ". You can publish to the team." : ". This account cannot write to the team's repository, so it can read but not publish: ask a team lead for access."),
      el("div", { class: "ctl" }, el("button", { class: "btn small", onclick: () => { TEAM.signOut(); draw(); } }, "Sign out"), el("button", { class: "btn small", onclick: async () => { await TEAM.load(true); render(); } }, "Check for new team data")));
    const tok = el("input", { type: "password", placeholder: "GitHub access token", autocomplete: "off", style: "flex:1 1 260px", "aria-label": "GitHub access token" }), rem = el("input", { type: "checkbox" });
    box.replaceChildren(el("div", { class: "prose" }, el("p", {}, "Publishing saves a design decision for ", el("b", {}, "everyone"), ": it is written into the team's repository (", el("code", {}, TEAM.repo), "), with your name and the date, and the site shows it to every user. It is separate from Run simulation, which only ever changes what you see."),
      el("p", {}, "To publish you need write access to that repository and a personal access token of your own: on GitHub, Settings → Developer settings → Fine-grained tokens → generate one for that repository only, with “Contents: read and write”. Paste it here. It stays in this browser and is sent only to GitHub.")),
      el("div", { class: "ctl" }, tok, el("label", { class: "check" }, rem, "Remember on this device"), el("button", { class: "btn publish", onclick: async () => { if (!tok.value.trim()) return; const u = await teamBusy("Signing in", () => TEAM.signIn(tok.value.trim(), rem.checked)); if (u) { setStatus(`Signed in to GitHub as ${u}.`, "ok"); render(); } } }, "Sign in to publish")));
  };
  draw();
  if (!TEAM.user && TEAM.token()) TEAM.signIn(TEAM.token(), !!localStorage.getItem("lapsim.gh")).then(() => render(), () => draw());
}, { wide: true });
V("team", "team_designs", "Team car designs", "Cars the team has published, with their results. The adopted one is what everybody opens on.", (b) => {
  const box = el("div", { class: "muted" }, "Loading the team's data…"); b.append(box);
  TEAM.load().then((idx) => {
    const mine = idx.designs.filter((d) => d.car === S.car), adopted = idx.adopted[S.car], can = TEAM.user && TEAM.canPush;
    const rows = mine.map((d) => el("tr", { class: d.id === adopted ? "selrow" : "" }, el("td", {}, el("b", {}, d.name), d.id === adopted ? el("span", { class: "tag ok", style: "margin-left:6px" }, "the team's car") : null), el("td", {}, d.note || ""), el("td", {}, String(d.changes ?? "")),
      el("td", {}, d.events && d.events.autocross && d.events.autocross.time_s != null ? d.events.autocross.time_s.toFixed(3) : "–"), el("td", {}, d.events ? totalPoints(d.events).toFixed(1) : "–"), el("td", {}, `${d.by}, ${d.date}`),
      el("td", {}, el("button", { class: "btn small", onclick: async () => { if (Object.keys(OV()).length && !confirm("Loading replaces the changes you have made to this car in your browser. Continue?")) return; await teamBusy("Loading the design", () => teamLoadDesign(d)); render(); pending(); } }, "Open"),
        can ? el("button", { class: "btn publish small", style: "margin-left:4px", onclick: async () => { const on = d.id !== adopted; if (!confirm(on ? `Make “${d.name}” the team's ${S.car.toUpperCase()} car?\n\nEveryone will open on this car and its results from now on.` : `Stop using “${d.name}” as the team's car? Everyone goes back to the config files' baseline.`)) return;
          await teamBusy(on ? "Adopting the design" : "Removing the adoption", () => TEAM.change(on ? `“${d.name}” adopted as the ${S.car.toUpperCase()} car` : `${S.car.toUpperCase()} car back to the baseline`, (i) => { if (on) { i.adopted[S.car] = d.id; const x = i.designs.find((q) => q.id === d.id); if (x) x.adopted_date = stamp().date; } else delete i.adopted[S.car]; })); render(); } }, d.id === adopted ? "Un-adopt" : "Adopt for everyone") : null,
        can ? el("button", { class: "btn small", style: "margin-left:4px", onclick: async () => { if (!confirm(`Remove “${d.name}” from the team's list for everyone?`)) return; await teamBusy("Removing the design", () => TEAM.change(`design “${d.name}” removed`, (i) => { i.designs = i.designs.filter((q) => q.id !== d.id); if (i.adopted[S.car] === d.id) delete i.adopted[S.car]; })); render(); } }, "Remove") : null)));
    box.className = ""; box.replaceChildren(mine.length ? el("div", { class: "tablewrap" }, el("table", { class: "edit" }, el("thead", {}, el("tr", {}, ["design", "why", "changes", "autocross [s]", "points", "published", ""].map((h) => el("th", {}, h)))), el("tbody", {}, rows)))
      : el("div", { class: "muted" }, `No ${S.car.toUpperCase()} designs have been published yet.`),
      el("div", { class: "ctl", style: "margin-top:10px" }, el("button", { class: "btn publish", onclick: () => teamPublishDesign() }, "Publish the car I have now to the team…"),
        el("span", { class: "muted" }, "Includes every changed input, the frame, hard points and wings, and the results of the last run.")));
  });
}, { wide: true });
V("team", "team_drivers", "Team drivers", "The driver list everyone picks from on the Driver page.", (b) => {
  const box = el("div", { class: "muted" }, "Loading…"); b.append(box);
  TEAM.load().then((idx) => {
    const t = idx.drivers && idx.drivers.drivers, mine = typeof DR !== "undefined" ? DR.load() : null, local = !!localStorage.getItem("lapsim.drivers");
    box.className = ""; box.replaceChildren(...[el("p", {}, t ? `The team's list has ${t.length} driver${t.length === 1 ? "" : "s"}: ${t.map((d) => d.name).join(", ")}.` : "The team has not published a driver list yet: everyone sees the placeholders that came with the site."),
      local ? el("div", { class: "note warn" }, "This browser has its own edits to the driver list, so it is showing those instead of the team's.") : null,
      el("div", { class: "ctl" }, el("button", { class: "btn publish", onclick: async () => { if (!mine) return; if (!TEAM.token()) return alert("Sign in above first.");
        if (!confirm(`Replace the team's driver list with the ${mine.drivers.length} driver(s) in this browser (${mine.drivers.map((d) => d.name).join(", ")}), and the seat position?\n\nEveryone will see this list.`)) return;
        const ok = await teamBusy("Publishing the drivers", () => TEAM.change(`driver list published (${mine.drivers.length} drivers)`, (i) => { i.drivers = JSON.parse(JSON.stringify(mine)); })); if (ok) { localStorage.removeItem("lapsim.drivers"); S.driverFile = ok.drivers; DR.data = null; render(); } } }, "Publish my driver list to the team…"),
        local ? el("button", { class: "btn small", onclick: () => { localStorage.removeItem("lapsim.drivers"); if (t) S.driverFile = idx.drivers; DR.data = null; render(); } }, "Drop my edits and use the team's list") : null,
        el("button", { class: "btn small", onclick: () => { S.page = "drivers"; location.hash = `${S.car}/drivers`; render(); } }, "Edit drivers"))].filter(Boolean));
  });
});
/* A corner assembly: the brake hardware and the unsprung corner, as one named set of inputs. */
const CORNER_EXTRA = ["vehicle:mass.unsprung_mass_front", "vehicle:mass.unsprung_mass_rear", "vehicle:mass.unsprung_cg_height", "vehicle:tires.tire_radius"];
const cornerKeys = () => S.data[S.car].schema.inputs.filter((i) => (i.key.startsWith("vehicle:brakes.") || CORNER_EXTRA.includes(i.key)) && i.kind !== "list").map((i) => i.key);
const cornerNow = () => Object.fromEntries(cornerKeys().map((k) => [k, cur(k) ?? null]));
async function teamPublishCorner() {
  if (!TEAM.token()) { alert("Publishing a corner assembly changes the list everyone sees, so it needs a sign-in. Open the Team page and sign in with GitHub first."); S.page = "team"; location.hash = `${S.car}/team`; return render(); }
  const name = prompt("Publish the corner assembly as it is set now (brake hardware, rotors, pads, unsprung mass, tire radius).\n\nName (for example: CT-17 front ISR caliper, 190 mm rotor):"); if (!name) return;
  const note = prompt("One line of notes (part numbers, why):", "") || "", values = cornerNow();
  if (!confirm(`Publish “${name}” to the team's corner assemblies?\n\nEveryone will be able to apply it to their car.`)) return;
  await teamBusy("Publishing the corner assembly", () => TEAM.change(`corner assembly “${name}” published`, (i) => { i.corners.push(Object.assign({ id: "c_" + Date.now().toString(36), name, note, values }, stamp())); }));
  render();
}
V("team", "team_corners", "Corner assemblies", "Named sets of brake and corner hardware. Apply one to put its values into your car.", (b) => {
  const box = el("div", { class: "muted" }, "Loading…"); b.append(box);
  TEAM.load().then((idx) => {
    const can = TEAM.user && TEAM.canPush, rows = idx.corners.map((c) => el("tr", {}, el("td", {}, el("b", {}, c.name)), el("td", {}, c.note || ""), el("td", {}, String(Object.values(c.values).filter((v) => v != null).length)), el("td", {}, `${c.by}, ${c.date}`),
      el("td", {}, el("button", { class: "btn small", onclick: () => { let n = 0; for (const [k, v] of Object.entries(c.values)) if (inputOf(k) && v != null) { setOv(k, v, { quiet: true }); n++; } pending(); setStatus(`“${c.name}” applied to this car (${n} values). Press Run simulation to see what it does.`, "ok"); render(); } }, "Apply to my car"),
        can ? el("button", { class: "btn small", style: "margin-left:4px", onclick: async () => { if (!confirm(`Remove “${c.name}” for everyone?`)) return; await teamBusy("Removing the corner assembly", () => TEAM.change(`corner assembly “${c.name}” removed`, (i) => { i.corners = i.corners.filter((q) => q.id !== c.id); })); render(); } }, "Remove") : null)));
    box.className = ""; box.replaceChildren(idx.corners.length ? el("div", { class: "tablewrap" }, el("table", { class: "edit" }, el("thead", {}, el("tr", {}, ["corner assembly", "notes", "values set", "published", ""].map((h) => el("th", {}, h)))), el("tbody", {}, rows))) : el("div", { class: "muted" }, "No corner assemblies have been published yet."),
      el("div", { class: "ctl", style: "margin-top:10px" }, el("button", { class: "btn publish", onclick: teamPublishCorner }, "Publish my corner assembly to the team…"), el("span", { class: "muted" }, "Set it up on the Brakes page first.")));
  });
});
V("team", "team_log", "Recent team saves", "", (b) => { const box = el("div", { class: "muted" }, "Loading…"); b.append(box);
  TEAM.load().then((idx) => { box.className = ""; box.replaceChildren(idx.log.length ? table({ columns: ["when", "who", "what"], rows: idx.log.slice(0, 15).map((l) => [l.date, l.by, l.what]) }) : el("div", { class: "muted" }, "Nothing has been published yet.")); }); });
// publish buttons where the things are made
V("brakes", "corner_publish", "Share this corner assembly", "Publishing saves the brake and corner hardware as it is set now, by name, for the whole team. It is separate from running the simulation.", (b) => {
  const n = cornerKeys().filter((k) => k in OV()).length;
  b.append(el("div", { class: "ctl" }, el("button", { class: "btn publish", onclick: teamPublishCorner }, "Publish this corner assembly to the team…"), el("button", { class: "btn small", onclick: () => { S.page = "team"; location.hash = `${S.car}/team`; render(); } }, "See the team's corner assemblies"),
    el("span", { class: "muted" }, n ? `${n} corner value${n === 1 ? "" : "s"} changed from the baseline.` : "The corner is as in the config files.")));
}, { wide: true });
V("drivers", "driver_publish", "Share the driver list", "Publishing replaces the list every team member picks from. It is separate from running the simulation.", (b) => {
  b.append(el("div", { class: "ctl" }, el("button", { class: "btn publish", onclick: () => { S.page = "team"; location.hash = `${S.car}/team`; render(); setTimeout(() => { const c = CARDS.get("team_drivers"); if (c) c.ctx.card.scrollIntoView({ block: "center" }); }, 200); } }, "Publish the driver list to the team…"),
    el("span", { class: "muted" }, "Opens the Team page, where you sign in and confirm.")));
}, { wide: true });

// ------------------------------------------------------------------ import & export
V("data", "export_cad", "Export the changed car", "Geometry for CAD, the changed inputs for the config files, and the results.", (b) => {
  const out = el("div"), go = el("button", { class: "btn navy", onclick: async () => {
    go.disabled = true; out.replaceChildren(el("div", { class: "muted" }, "Preparing the files…"));
    try {
      const f = await call("export_files", [S.car, OV()]), r = R();
      const D = { "hardpoints_xyz.csv": "Every suspension point, both sides, in mm. Opens in a spreadsheet; use it for a 3-D sketch or a point cloud.", "solidworks_equations.txt": "The same points as SolidWorks global variables: Tools → Equations → Import (or link the file), then drive the sketch points from them.",
        "hardpoints_front_optimumk.csv": "Front axle in the OptimumK export format the model reads: replace IC/data/hardpoints/CT17_front.csv to make it the baseline.", "hardpoints_rear_optimumk.csv": "Rear axle, same format.",
        "frame_tubes.csv": "The frame's tubes in the frame CAD's axes: replace IC/data/frame/ct17_spaceframe_tubes.csv, or use the end points for a 3-D sketch.", "frame_joints.csv": "The frame's joints in car axes, mm.",
        "inputs_changed.yaml": "Every changed input by config file and dotted path, ready to paste into the YAML files." };
      const evs = Object.entries(r.events).map(([e, d]) => [e, d.time_s, d.points]);
      const files = Object.assign({}, f, { "results_events.csv": csvOf(["event", "time_s", "points"], evs) });
      for (const [e, L] of Object.entries(r.laps)) if (!L.error) { const cols = Object.keys(L.channels); files[`results_lap_${e}.csv`] = csvOf(cols, L.channels[cols[0]].map((_, i) => cols.map((c) => L.channels[c][i]))); }
      if (r.endurance) files["results_endurance_laps.csv"] = csvOf(r.endurance.columns, r.endurance.rows);
      files["site_changes.json"] = JSON.stringify({ car: S.car, overrides: OV(), wings: typeof WD !== "undefined" ? WD.load() : null }, null, 1);
      if (typeof wdWingEls === "function" && wdBaseEls().length && wdBaseEls()[0].le) for (const w of ["front_wing", "rear_wing"]) for (const e of wdWingEls(w)) {
        files[`wing_${e.name}_on_car_mm.csv`] = csvOf(["x_mm", "z_mm"], e.poly.map((p) => [+(p[0] * 1e3).toFixed(3), +(p[1] * 1e3).toFixed(3)]));
        files[`wing_${e.name}_section.dat`] = `${e.name} section, unit chord (chord ${(e.chord * 1e3).toFixed(1)} mm at ${e.angle.toFixed(2)} deg on the car)\n` + e.unit.map((p) => `${p[0].toFixed(6)} ${p[1].toFixed(6)}`).join("\n") + "\n"; }
      D["results_events.csv"] = "Event times and points of the last run."; D["results_endurance_laps.csv"] = "One row per endurance lap."; D["site_changes.json"] = "Everything changed on the site, to import here again or send to someone.";
      const rows = Object.keys(files).map((n) => el("tr", {}, el("td", { style: "font-family:var(--mono);font-size:12px" }, n), el("td", { style: "text-align:left;white-space:normal" }, D[n] || (n.startsWith("results_lap_") ? "Every channel along the lap." : n.endsWith(".dat") ? "The section's coordinates (Selig format), as designed." : n.startsWith("wing_") ? "The element's outline where it sits on the car: x behind the front axle, z above the ground." : "")),
        el("td", {}, (files[n].length / 1e3).toFixed(1) + " kB"), el("td", {}, el("button", { class: "btn small", onclick: () => download(n, files[n], n.endsWith(".json") ? "application/json" : "text/plain") }, "Download"))));
      out.replaceChildren(...[el("div", { class: "tablewrap" }, el("table", { class: "edit" }, el("thead", {}, el("tr", {}, ["file", "what it is for", "size", ""].map((h) => el("th", {}, h)))), el("tbody", {}, rows))),
        dirty() ? el("div", { class: "note warn" }, "The result files are from the last run, before your latest changes. The geometry and input files are current.") : null].filter(Boolean));
    } catch (e) { out.replaceChildren(el("div", { class: "err" }, String(e.message || e))); }
    go.disabled = false; } }, "Prepare the files");
  b.append(el("div", { class: "ctl" }, go, el("span", { class: "muted" }, "Built from the car as it is now, in your browser.")), out);
}, { wide: true });
V("data", "import_cad", "Import geometry from CAD", "Suspension points and frame tubes drawn elsewhere, brought in as changes to this car.", (b) => {
  const axle = el("select", { "aria-label": "Axle" }, el("option", { value: "auto" }, "Axle: work it out"), el("option", { value: "front" }, "Front axle"), el("option", { value: "rear" }, "Rear axle")), msg = el("div", { class: "sub" });
  const hp = el("input", { type: "file", accept: ".csv,.txt", "aria-label": "Hard points file" }), fr = el("input", { type: "file", accept: ".csv,.txt", "aria-label": "Frame tubes file" }), js = el("input", { type: "file", accept: ".json", "aria-label": "Site changes file" });
  hp.addEventListener("change", async () => { try { const f = await readFile(hp); setStatus("Reading the hard points…", "busy"); const r = await call("import_hardpoints", [f.text]);
    const all = JSON.parse(JSON.stringify(OV()["geom:hardpoints"] || {})); let n = 0;
    for (const [k, pts] of Object.entries(r.points)) { const a = k === "null" || k == null ? (axle.value === "auto" ? r.axle_guess : axle.value) : k; if (!GEO().hardpoints[a]) continue; all[a] = all[a] || {};
      for (const [name, p] of Object.entries(pts)) { const base = GEO().hardpoints[a].points[name]; if (!base) continue; if (dist3(base, p) > 1e-6) { all[a][name] = p; n++; } else delete all[a][name]; } if (!Object.keys(all[a]).length) delete all[a]; }
    if (Object.keys(all).length) OV()["geom:hardpoints"] = all; else delete OV()["geom:hardpoints"];
    pending(); setStatus(`${f.name}: ${n} suspension point${n === 1 ? "" : "s"} differ from the baseline and are now in this car.`, "ok"); msg.textContent = `${f.name} read (${r.format === "optimumk" ? "OptimumK export" + (r.title ? ", “" + r.title + "”" : "") : "point list"}): ${n} point(s) changed. See them on the Suspension page, then Run simulation.`; }
    catch (e) { setStatus("That file could not be used: " + (e.message || e), "err"); msg.textContent = "Not imported: " + (e.message || e); } hp.value = ""; });
  fr.addEventListener("change", async () => { try { const f = await readFile(fr); setStatus("Reading the frame…", "busy"); const st = await call("import_frame", [f.text]); st.materials = st.materials || GEO().frame.materials; OV()["geom:frame"] = st; if (typeof FR !== "undefined") delete FR._w[S.car];
    pending(); setStatus(`${f.name}: frame with ${st.tubes.length} tubes and ${st.nodes.length} joints is now in this car.`, "ok"); msg.textContent = `${f.name} read: ${st.tubes.length} tubes. See it on the Chassis & frame page, then Run simulation.`; }
    catch (e) { setStatus("That file could not be used: " + (e.message || e), "err"); msg.textContent = "Not imported: " + (e.message || e); } fr.value = ""; });
  js.addEventListener("change", async () => { try { const f = await readFile(js), j = JSON.parse(f.text); if (j.car && j.car !== S.car) throw new Error(`that file is for the ${j.car.toUpperCase()} car`); Object.assign(OV(), j.overrides || {}); if (j.wings && typeof WD !== "undefined") WD.set(j.wings); if (typeof FR !== "undefined") delete FR._w[S.car];
    pending(); msg.textContent = `${f.name} read: ${Object.keys(j.overrides || {}).length} change(s) added to this car.`; } catch (e) { msg.textContent = "Not imported: " + (e.message || e); } js.value = ""; });
  b.append(el("div", { class: "dform wform" }, el("label", { class: "widef" }, "Suspension points", el("span", { class: "ctl", style: "margin:0" }, hp, axle)), el("label", { class: "widef" }, "Frame tubes", fr), el("label", { class: "widef" }, "Changes saved from this site", js)), msg,
    el("div", { class: "prose" }, el("p", {}, el("b", {}, "Suspension points:"), " an OptimumK export (.csv, the format in IC/data/hardpoints), or the hardpoints_xyz.csv this site exports, edited in a spreadsheet or filled from SolidWorks. ",
      el("b", {}, "Frame:"), " a tube list with the columns x0,y0,z0,x1,y1,z1,od,wall,shape,bend,member in metres (the frame_tubes.csv this site exports; tools/frame.py writes one from a SolidWorks part). A SolidWorks or ANSYS file itself cannot be opened in a browser: export points or tubes from it first.")));
}, { wide: true });
V("data", "import_results", "Import analysis and test results", "Numbers from ANSYS, Fluent, a wind tunnel or the scales, put into the car as inputs.", (b) => {
  const g = (label, unit, attrs = {}) => { const i = el("input", Object.assign({ type: "number", step: "any", placeholder: "–" }, attrs)); return [el("label", {}, label, i, el("span", { class: "u" }, unit)), i]; };
  const [l1, v] = g("Aero result: air speed", "m/s"), [l2, df] = g("Aero result: total downforce", "N"), [l3, dr] = g("Aero result: total drag", "N"), [l4, fs] = g("Aero result: share on the front axle", "%"), [l5, kt] = g("Frame torsional stiffness (FEA or twist test)", "N·m/deg");
  const [l6, fl] = g("Corner weight, front left", "kg"), [l7, frr] = g("Corner weight, front right", "kg"), [l8, rl] = g("Corner weight, rear left", "kg"), [l9, rr] = g("Corner weight, rear right", "kg"), [l10, dm] = g("Driver mass on the scales", "kg");
  const msg = el("div", { class: "sub" });
  const apply = () => { const done = [], rho = GEO().air_density || 1.12;
    if (+v.value > 0 && (+df.value || +dr.value)) { const q = 0.5 * rho * (+v.value) ** 2; setOv("vehicle:aero.calib_speed", +v.value, { quiet: true });
      if (+df.value) { setOv("vehicle:aero.calib_ClA", +(+df.value / q).toFixed(4), { quiet: true }); done.push(`downforce coefficient × area ${(+df.value / q).toFixed(3)} m²`); }
      if (+dr.value) { setOv("vehicle:aero.calib_CdA", +(+dr.value / q).toFixed(4), { quiet: true }); done.push(`drag coefficient × area ${(+dr.value / q).toFixed(3)} m²`); } }
    if (+fs.value > 0) { setOv("vehicle:aero.calib_front_axle_share", +fs.value / 100, { quiet: true }); done.push(`aero front share ${fs.value} %`); }
    if (+kt.value > 0) { setOv("vehicle:mass.chassis_torsional_stiffness", +(+kt.value * 180 / Math.PI).toFixed(1), { quiet: true }); done.push(`frame stiffness ${kt.value} N·m/deg (the frame is no longer treated as rigid)`); }
    const w = [fl, frr, rl, rr].map((i) => +i.value);
    if (w.every((x) => x > 0)) { const tot = w[0] + w[1] + w[2] + w[3], drv = +dm.value > 0 ? +dm.value : cur("vehicle:mass.driver_mass"); setOv("vehicle:mass.front_weight_fraction", +((w[0] + w[1]) / tot).toFixed(4), { quiet: true }); setOv("vehicle:mass.left_weight_fraction", +((w[0] + w[2]) / tot).toFixed(4), { quiet: true });
      setOv("vehicle:mass.mass_no_driver", +(tot - drv).toFixed(2), { quiet: true }); if (+dm.value > 0) setOv("vehicle:mass.driver_mass", +dm.value, { quiet: true }); done.push(`mass ${tot.toFixed(1)} kg with driver, ${(100 * (w[0] + w[1]) / tot).toFixed(1)} % front, ${(100 * (w[0] + w[2]) / tot).toFixed(1)} % left`); }
    pending(); msg.textContent = done.length ? "Put into the car: " + done.join("; ") + ". Press Run simulation." : "Nothing to apply: fill in at least one complete group."; };
  b.append(el("div", { class: "dform wform" }, l1, l2, l3, l4, l5, l6, l7, l8, l9, l10), el("div", { class: "ctl" }, el("button", { class: "btn navy", onclick: apply }, "Put these into the car"), el("span", { class: "muted" }, "Corner weights are with the driver seated. Leave a group empty to skip it.")), msg,
    el("div", { class: "note warn" }, "Result files from ANSYS or Fluent come in many layouts, so this takes the numbers themselves rather than guessing at a file. Logged runs from RaceStudio are read on the Optimization page, where they are set against the prediction."));
}, { wide: true });

// ------------------------------------------------------------------ optimization: prediction against the car
const OPT = { log: null, logName: "", sens: null, opt: null, track: null, picks: null, measured: JSON.parse(localStorage.getItem("lapsim.measured") || "{}") };
V("optimise", "opt_times", "Predicted against measured: event times", "Type in what the car did. The prediction is the last run of this car.", (b) => {
  const m = (OPT.measured[S.car] = OPT.measured[S.car] || {}), r = R(), evs = ["acceleration", "skidpad", "autocross", "endurance"], wrap = el("div"), p = plotDiv("short");
  const draw = () => {
    const rows = evs.map((e) => { const pred = r.events[e] && r.events[e].time_s, me = m[e];
      return el("tr", {}, el("td", {}, EVENT_NAMES[e]), el("td", {}, pred != null ? pred.toFixed(3) : "–"), el("td", {}, numIn(me ?? "", (v) => { m[e] = v; localStorage.setItem("lapsim.measured", JSON.stringify(OPT.measured)); draw(); }, { min: 0, style: "width:96px", placeholder: "measured", "aria-label": e + " measured time" })),
        el("td", {}, me && pred != null ? (pred - me > 0 ? "+" : "") + (pred - me).toFixed(3) : "–"), el("td", {}, me && pred != null ? (100 * (pred - me) / me).toFixed(2) : "–")); });
    wrap.replaceChildren(el("div", { class: "tablewrap" }, el("table", { class: "edit" }, el("thead", {}, el("tr", {}, ["event", "predicted [s]", "measured [s]", "prediction − measured [s]", "error [%]"].map((h) => el("th", {}, h)))), el("tbody", {}, rows))));
    const have = evs.filter((e) => m[e] && r.events[e] && r.events[e].time_s != null);
    p.hidden = !have.length;
    if (have.length) plot(p, [{ type: "bar", x: have.map((e) => EVENT_NAMES[e]), y: have.map((e) => 100 * (r.events[e].time_s - m[e]) / m[e]), marker: { color: PAL[0] }, text: have.map((e) => (100 * (r.events[e].time_s - m[e]) / m[e]).toFixed(1) + " %"), textposition: "auto", hovertemplate: "%{x}: %{y:.2f} %<extra></extra>" }],
      { hovermode: "closest", yaxis: { title: "prediction error [%], + = model slower" }, bargap: 0.5 });
  };
  b.append(wrap, p, el("div", { class: "sub" }, "Endurance: total time for the distance in the car's inputs. A model that is too fast everywhere usually has too much grip: see the logged-run card for a measured grip scale.")); draw();
}, { wide: true });
V("optimise", "opt_log", "Predicted against measured: a logged run", "Open a run exported from RaceStudio as .csv. It is cleaned with the model's own pipeline, and the car is then simulated on the course the log was driven on.", (b) => {
  const out = el("div"), file = el("input", { type: "file", accept: ".csv,.txt", "aria-label": "Logged run" });
  const show = () => {
    const L = OPT.log; if (!L) return out.replaceChildren();
    const P = L.predicted, M = L.measured, cal = L.calibration || {}, bits = [];
    bits.push(el("div", { class: "kpis" }, tile("Measured lap", M.time_s.toFixed(3), "s", L.laps.length > 1 ? `best of ${L.laps.length} laps (lap ${M.lap})` : L.laps.length ? "the one timed lap" : "no lap marker: whole run"),
      P.error ? null : tile("Predicted on the same course", P.time_s.toFixed(3), "s", `${fmt(P.track_length_m, 4)} m, ${P.start} start`), P.error ? null : tile("Prediction − measured", (P.delta_s > 0 ? "+" : "") + P.delta_s.toFixed(3), "s", `${P.delta_pct > 0 ? "+" : ""}${P.delta_pct.toFixed(2)} % · ${P.delta_s < 0 ? "the model is faster than the car" : "the model is slower than the car"}`, Math.abs(P.delta_pct) > 2 ? "up" : "down"),
      tile("Run", fmt(L.duration_s, 4), "s logged", [L.meta.Track, L.meta.Racer, L.meta.Date].filter(Boolean).join(" · "))));
    if (P.error) bits.push(el("div", { class: "note warn" }, P.error));
    const sp = plotDiv("tall"), gg = plotDiv("tall"); bits.push(el("div", { class: "grid inner" }, el("div", {}, el("div", { class: "sub" }, "Speed along the lap"), sp), el("div", {}, el("div", { class: "sub" }, "Accelerations used: the car on track (dots) against the model's grip limits"), gg)));
    out.replaceChildren(...bits);
    const tr = []; if (M.s && M.v) tr.push(line(M.s, M.v.map((x) => x * 3.6), "measured", { line: { width: 2, color: "#47526a" } })); if (!P.error) tr.push(line(P.s, P.v.map((x) => x * 3.6), "predicted", { line: { width: 2, color: PAL[0] } }));
    plot(sp, tr, { xaxis: { title: "distance [m]" }, yaxis: { title: "speed [km/h]", rangemode: "tozero" } });
    const g = SUB().ggv, t2 = [{ x: L.gg.ay, y: L.gg.ax, mode: "markers", name: "measured", marker: { size: 3.5, color: "#8a9099" }, hoverinfo: "skip" }];
    if (g && g.speeds) [1, Math.floor(g.speeds.length / 2), g.speeds.length - 1].forEach((k, i) => { const s = g.speeds[k]; t2.push(line([...s.ay.map((a) => -a).reverse(), ...s.ay, ...s.ay.slice().reverse(), ...s.ay.map((a) => -a)], [...s.ax_acc.slice().reverse(), ...s.ax_acc, ...s.ax_brk.slice().reverse().map((a) => -a), ...s.ax_brk.map((a) => -a)], `model, ${(s.v * 3.6).toFixed(0)} km/h`, { line: { width: 2, color: SEQ_STEP(1 + 2 * i, 6) }, hoverinfo: "skip" })); });
    plot(gg, t2, { hovermode: "closest", xaxis: { title: "lateral [g]" }, yaxis: { title: "longitudinal [g]", scaleanchor: "x" } });
    // what the log says the inputs should be
    const sug = [];
    if (cal.grip && cal.grip.lambda_mu_suggested != null) sug.push(["Grip scale (tires.lambda_mu)", `${fmt(cal.grip.lambda_mu_now, 4)} → ${fmt(cal.grip.lambda_mu_suggested, 4)}`, `the car's cornering limit is ${(100 * cal.grip.ratio).toFixed(1)} % of the model's, from ${cal.grip.samples} cornering samples`, () => setOv("vehicle:tires.lambda_mu", cal.grip.lambda_mu_suggested)]);
    if (cal.grip && cal.grip.skipped) sug.push(["Grip scale", "–", cal.grip.skipped, null]);
    if (cal.coastdown && cal.coastdown.CdA != null) { sug.push(["Drag coefficient × area (aero.calib_CdA)", `${fmt(cur("vehicle:aero.calib_CdA"), 4)} → ${fmt(cal.coastdown.CdA, 4)} m²`, `coast-down fit, ± ${fmt(cal.coastdown.CdA_sd, 2)} from ${cal.coastdown.n} samples`, () => setOv("vehicle:aero.calib_CdA", cal.coastdown.CdA)]);
      sug.push(["Rolling resistance (tires.rolling_resistance)", `${fmt(cur("vehicle:tires.rolling_resistance") ?? 0.02, 3)} → ${fmt(cal.coastdown.Crr, 3)}`, `coast-down fit, ± ${fmt(cal.coastdown.Crr_sd, 2)}`, () => setOv("vehicle:tires.rolling_resistance", cal.coastdown.Crr)]); }
    if (sug.length) out.append(el("div", { class: "sub", style: "margin-top:12px" }, "What this run says the inputs should be"), el("div", { class: "tablewrap" }, el("table", { class: "edit" }, el("thead", {}, el("tr", {}, ["input", "now → from the log", "basis", ""].map((h) => el("th", {}, h)))),
      el("tbody", {}, sug.map((s) => el("tr", {}, el("td", {}, s[0]), el("td", {}, el("b", {}, s[1])), el("td", { style: "text-align:left;white-space:normal" }, s[2]), el("td", {}, s[3] ? el("button", { class: "btn small", onclick: () => { s[3](); setStatus("Applied to this car. Open the run again to see the new prediction, and Run simulation for the events.", "ok"); render(); } }, "Use in my car") : "")))))));
    if (cal.grip && cal.grip.v) { const lp = plotDiv("short"); out.append(el("div", { class: "sub", style: "margin-top:12px" }, "Cornering limit against speed"), lp);
      plot(lp, [line(cal.grip.v.map((x) => x * 3.6), cal.grip.measured_g, "measured", { mode: "lines+markers", marker: { size: 7 }, line: { width: 2, color: "#47526a" } }), line(cal.grip.v.map((x) => x * 3.6), cal.grip.model_g, "model", { line: { width: 2, color: PAL[0] } })], { xaxis: { title: "speed [km/h]" }, yaxis: { title: "lateral limit [g]" } }); }
    out.append(el("details", { style: "margin-top:10px" }, el("summary", { class: "muted", style: "cursor:pointer" }, `What the cleaning did to each channel (${L.channels.length} channels read)`), L.qc ? table(L.qc) : el("div", { class: "muted" }, L.channels.join(", "))));
  };
  const run = async (f) => { out.replaceChildren(el("div", { class: "muted" }, `Reading ${f.name} and simulating the car on its course…`)); setStatus("Reading the logged run…", "busy");
    try { OPT.log = await call("log_import", [S.car, f.text, OV()]); OPT.logName = f.name; OPT.logText = f.text; setStatus(`${f.name} read and compared.`, "ok"); show(); } catch (e) { OPT.log = null; out.replaceChildren(el("div", { class: "err" }, "That run could not be used: " + (e.message || e))); setStatus("That run could not be used: " + (e.message || e), "err"); } };
  file.addEventListener("change", async () => { try { await run(await readFile(file)); } catch (e) { /* nothing chosen */ } file.value = ""; });
  b.append(el("div", { class: "ctl" }, file, OPT.logText ? el("button", { class: "btn small", onclick: () => run({ name: OPT.logName, text: OPT.logText }) }, `Compare ${OPT.logName} again with the car as it is now`) : null,
    el("span", { class: "muted" }, "In RaceStudio: export the run as CSV, with GPS, accelerations and wheel speeds. An .xrk file itself cannot be read in a browser.")), out);
  show();
}, { wide: true });

// ------------------------------------------------------------------ optimization: what matters, and a better set-up
const OPT_DEFAULT = ["vehicle:mass.cg_height", "vehicle:mass.mass_no_driver", "vehicle:mass.front_weight_fraction", "vehicle:front.spring_rate", "vehicle:rear.spring_rate", "vehicle:front.static_camber_deg", "vehicle:rear.static_camber_deg", "vehicle:aero.calib_ClA", "vehicle:aero.calib_CdA", "vehicle:brakes.front_bias"];
const optInputs = () => S.data[S.car].schema.inputs.filter((i) => i.key.startsWith("vehicle:") && i.kind === "number" && i.value != null && i.value !== 0 && !/\.components\.|adjustab|^vehicle:(sim|events)\./.test(i.key));
function optPicker(picked, max) {
  const all = optInputs(), box = el("div", { class: "channels" }), filt = el("input", { type: "search", placeholder: `Filter ${all.length} inputs…`, style: "max-width:260px" });
  const draw = () => box.replaceChildren(...all.filter((i) => !filt.value.trim() || i.key.includes(filt.value.trim()) || picked.includes(i.key)).map((i) => el("button", { class: "chip" + (picked.includes(i.key) ? " on" : ""), title: i.note, onclick: (e) => {
    const k = picked.indexOf(i.key); if (k >= 0) picked.splice(k, 1); else { if (max && picked.length >= max) return alert(`Pick at most ${max}: each one multiplies the number of simulations.`); picked.push(i.key); } e.currentTarget.classList.toggle("on"); } }, i.key.split(":")[1])));
  filt.addEventListener("input", draw); draw();
  return el("div", {}, el("div", { class: "ctl" }, filt), box);
}
const optEvents = (sel) => el("span", { class: "ctl", style: "margin:0" }, ...["acceleration", "skidpad", "autocross", "endurance"].map((e) => el("label", { class: "check" }, el("input", { type: "checkbox", checked: sel.includes(e), onchange: (ev) => { const k = sel.indexOf(e); if (ev.target.checked && k < 0) sel.push(e); if (!ev.target.checked && k >= 0) sel.splice(k, 1); } }), EVENT_NAMES[e])));
V("optimise", "opt_sens", "Which inputs matter", "A 5 % step up and down in each chosen input, and what it does to every event and to the points (the lap simulation's sensitivity tool).", (b) => {
  const picks = (OPT.sensPicks = OPT.sensPicks || OPT_DEFAULT.filter((k) => inputOf(k) && inputOf(k).value)), evs = (OPT.sensEvents = OPT.sensEvents || ["acceleration", "skidpad", "autocross"]), out = el("div");
  const show = () => { const r = OPT.sens; if (!r || r.car !== S.car) return;
    const t = r.table, c = (n) => t.columns.indexOf(n), order = t.rows.slice().sort((a, d) => Math.abs(d[c("d_score_per_step")]) - Math.abs(a[c("d_score_per_step")])), p = plotDiv();
    out.replaceChildren(el("div", { class: "sub" }, `${r.simulations} simulations. Score: ${r.base.score_kind}. Bars: change in the score for a +5 % step in the input.`), p, table({ columns: ["input", "value", "step", "score change per step", ...r.events.map((e) => `${EVENT_NAMES[e]} change per step [s]`)],
      rows: order.map((x) => [x[c("param")], x[c("value")], x[c("step")], x[c("d_score_per_step")], ...r.events.map((e) => x[c(`dt_${e}_per_step`)])]) }));
    p.style.height = Math.max(220, 60 + 30 * order.length) + "px";
    plot(p, [{ type: "bar", orientation: "h", y: order.map((x) => x[c("param")]), x: order.map((x) => x[c("d_score_per_step")]), marker: { color: order.map((x) => (x[c("d_score_per_step")] >= 0 ? PAL[0] : PAL[1])) }, hovertemplate: "%{y}: %{x:.3f}<extra></extra>" }],
      { hovermode: "closest", xaxis: { title: "score change for a +5 % step (blue = better, orange = worse)" }, yaxis: { autorange: "reversed" }, margin: { l: 190, r: 10, t: 6, b: 40 } }); };
  const go = el("button", { class: "btn navy", onclick: async () => { if (!picks.length || !evs.length) return alert("Pick at least one input and one event."); go.disabled = true; out.replaceChildren(el("div", { class: "muted" }, `Running ${1 + 2 * picks.length} simulations…`));
    try { setStatus("Working out the sensitivities…", "busy"); OPT.sens = Object.assign(await call("sensitivity", [S.car, picks, OV(), evs]), { car: S.car }); setStatus("Sensitivities done.", "ok"); show(); } catch (e) { out.replaceChildren(el("div", { class: "err" }, String(e.message || e))); setStatus("Sensitivity failed: " + (e.message || e), "err"); } go.disabled = false; } }, "Run the sensitivity");
  b.append(optPicker(picks), el("div", { class: "ctl" }, el("span", { class: "muted" }, "Events:"), optEvents(evs), go), out); show();
}, { wide: true });
V("optimise", "opt_search", "Search for a better set-up", "The lap simulation's optimiser (differential evolution) moves the chosen inputs inside a range to score more points, then checks the result still wins when grip, air density and driver mass are off.", (b) => {
  const picks = (OPT.optPicks = OPT.optPicks || ["vehicle:front.spring_rate", "vehicle:rear.spring_rate", "vehicle:brakes.front_bias"].filter((k) => inputOf(k))), evs = (OPT.optEvents = OPT.optEvents || ["acceleration", "skidpad", "autocross"]), out = el("div");
  const span = el("input", { type: "number", value: 15, min: 2, max: 40, style: "width:66px", "aria-label": "range percent" }), effort = el("select", { "aria-label": "Effort" }, el("option", { value: "3,3" }, "Quick look"), el("option", { value: "5,4" }, "Normal"), el("option", { value: "9,5" }, "Thorough (slow)"));
  const show = () => { const r = OPT.opt; if (!r || r.car !== S.car) return; const p = plotDiv("short");
    out.replaceChildren(el("div", { class: "kpis" }, tile("Score, baseline of this search", fmt(r.score.base, 5), "", r.score.kind), tile("Score, best found", fmt(r.score.best, 5), "", `${r.score.best - r.score.base >= 0 ? "+" : ""}${fmt(r.score.best - r.score.base, 3)}`), tile("Simulations run", String(r.simulations), "")),
      ...r.verdict.map((v) => el("div", { class: "note " + (/DO NOT/.test(v) ? "warn" : ""), style: /RECOMMEND$/.test(v) ? "background:#e5f4ea;border:1px solid #bfe2cb;color:#0a5c26" : "" }, v)),
      el("div", { class: "grid inner" }, el("div", {}, el("div", { class: "sub" }, "Recommended values (SI)"), table(r.parameters)), el("div", {}, el("div", { class: "sub" }, "Events"), table(r.events))),
      r.robustness ? el("div", {}, el("div", { class: "sub", style: "margin-top:8px" }, "Does it still win when the uncertain inputs move?"), table(r.robustness)) : null,
      el("div", { class: "sub", style: "margin-top:8px" }, "Best score found so far, by simulation"), p,
      el("div", { class: "ctl" }, el("button", { class: "btn navy", onclick: () => { for (const [k, v] of Object.entries(r.best)) setOv(k, v, { quiet: true }); pending(); setStatus("The recommended values are in this car. Press Run simulation for the full results.", "ok"); render(); } }, "Put the recommended values into my car")));
    plot(p, [line(r.history.map((_, i) => i + 1), r.history, "best score")], { xaxis: { title: "simulation" }, yaxis: { title: "score" } }); };
  const go = el("button", { class: "btn navy", onclick: async () => { if (!picks.length || !evs.length) return alert("Pick at least one input and one event."); const [it, pop] = effort.value.split(",").map(Number), n = pop * picks.length * (it + 1) + 2 * picks.length + 10;
    if (!confirm(`This runs roughly ${n} lap simulations in your browser and can take several minutes${evs.includes("endurance") ? " (endurance makes each one much slower)" : ""}. The page stays usable. Start?`)) return;
    go.disabled = true; out.replaceChildren(el("div", { class: "muted" }, "Searching… progress shows in the status bar.")); const t0 = performance.now();
    try { setStatus("Optimising…", "busy"); OPT.opt = Object.assign(await call("optimise", [S.car, picks, OV(), evs, +span.value / 100, it, pop, true]), { car: S.car }); setStatus(`Optimisation finished in ${((performance.now() - t0) / 60000).toFixed(1)} min.`, "ok"); show(); }
    catch (e) { out.replaceChildren(el("div", { class: "err" }, String(e.message || e))); setStatus("Optimisation failed: " + (e.message || e), "err"); } go.disabled = false; } }, "Start the search");
  b.append(optPicker(picks, 5), el("div", { class: "ctl" }, el("label", {}, "Range each side of the current value [%] ", span), effort, el("span", { class: "muted" }, "Events:"), optEvents(evs), go), out); show();
}, { wide: true });
V("optimise", "opt_track", "Trackside adjustment", "With only what can be changed at the track (the spring rates, anti-roll bar blades and camber shims listed in the car's adjustability inputs), the best setting for each event.", (b) => {
  const evs = (OPT.trackEvents = OPT.trackEvents || ["skidpad"]), out = el("div");
  const show = () => { const r = OPT.track; if (!r || r.car !== S.car) return;
    out.replaceChildren(el("div", { class: "sub" }, `${r.simulations} simulations. Adjustments searched: ${r.features.join(", ") || "none"}.`), table(r.summary), r.settings ? el("div", {}, el("div", { class: "sub", style: "margin-top:8px" }, "Best setting of each adjustment, by event"), table(r.settings)) : null,
      r.not_optimised && r.not_optimised.rows.length ? el("div", {}, el("div", { class: "sub", style: "margin-top:8px" }, "Left out, and why"), table(r.not_optimised)) : null); };
  const go = el("button", { class: "btn navy", onclick: async () => { if (!evs.length) return alert("Pick at least one event."); if (!confirm(`This tries the combinations of adjustments for ${evs.length} event(s): on the order of 80 lap simulations per event, several minutes in a browser. Start?`)) return;
    go.disabled = true; out.replaceChildren(el("div", { class: "muted" }, "Searching… progress shows in the status bar."));
    try { setStatus("Searching the trackside adjustments…", "busy"); OPT.track = Object.assign(await call("trackside", [S.car, OV(), evs]), { car: S.car }); setStatus("Trackside search finished.", "ok"); show(); } catch (e) { out.replaceChildren(el("div", { class: "err" }, String(e.message || e))); setStatus("Trackside search failed: " + (e.message || e), "err"); } go.disabled = false; } }, "Find the best settings");
  b.append(el("div", { class: "ctl" }, el("span", { class: "muted" }, "Events:"), optEvents(evs), go), out); show();
}, { wide: true });
