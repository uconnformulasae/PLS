#!/usr/bin/env python
"""Cut the real wing sections out of the wing CAD (the STL files fsae_cfd builds the car from).

    python web/export_wings.py        # writes web/bridge/wing_sections.json, then run  python web/build.py

Run it natively (it needs fsae_cfd, so numba, and the STL files in IC/Aerodynamic Elements). For every wing element
(F1 ... F3, R1 ... R4) it slices the part's surface with a vertical plane at a span station and keeps:

    poly      the section as it sits on the car [m]: x rearward from the front axle, z up from the ground,
              from the trailing edge round the leading edge and back (cosine-spaced)
    unit      the same section on its own: chord = 1 along x, upright (suction side up), Selig order
    chord, angle_deg (chord line above horizontal, trailing edge up), thickness and camber [% chord]
    y         where along the span the element exists [m from the centre line]

and the span zones: which elements are stacked together between which endplates. The website's airfoil and
wing-stack models read this file; without it they fall back to the NACA stand-ins in configs/ct17_cfd.yaml.
"""
import json
import sys
import warnings
from pathlib import Path

import numpy as np

WEB = Path(__file__).resolve().parent
IC = WEB.parent / "IC"
sys.path.insert(0, str(IC))
warnings.filterwarnings("ignore")

# element name on the car -> part name in the CFD geometry (front parts are the right half, mirrored)
# The front wing on the car has three elements (team, 2026-11-15). The CAD assembly also holds a fourth flap,
# 611-11-17IC F4 (CFD part front_wing_el3), which is NOT on the car: it is left out here.
ELEMENTS = [("F1", "front_wing", "front_wing_el0_r"), ("F2", "front_wing", "front_wing_el1_r"), ("F3", "front_wing", "front_wing_el2_r"),
            ("R1", "rear_wing", "rear_wing_el0"), ("R2", "rear_wing", "rear_wing_el1"),
            ("R3", "rear_wing", "rear_wing_el2"), ("R4", "rear_wing", "rear_wing_el3")]
ENDPLATES = {"front_wing": ["fw_fence_in_r", "fw_fence_mid_r", "fw_endplate_r"], "rear_wing": ["rw_endplate_r"]}


def slice_mesh(V, F, y0):
    """Closed (x, z) loops where the plane y = y0 cuts a triangle mesh; the loop enclosing the most area first."""
    d = V[:, 1] - y0
    segs = []
    for tri in F:
        s = d[tri]
        if (s > 0).all() or (s < 0).all():
            continue
        pts = []
        for a, b in ((0, 1), (1, 2), (2, 0)):
            if (s[a] > 0) != (s[b] > 0):
                t = s[a] / (s[a] - s[b])
                p = V[tri[a]] + t * (V[tri[b]] - V[tri[a]])
                pts.append((p[0], p[2]))
        if len(pts) == 2:
            segs.append(pts)
    key = lambda p: (round(p[0] * 2e5), round(p[1] * 2e5))            # 5 micron grid to match shared edges
    nxt = {}
    for a, b in segs:
        nxt.setdefault(key(a), []).append((a, b))
        nxt.setdefault(key(b), []).append((b, a))
    loops, used = [], set()
    for a, b in segs:
        if (key(a), key(b)) in used or (key(b), key(a)) in used:
            continue
        loop, cur, prev = [a], b, a
        used.add((key(a), key(b)))
        for _ in range(len(segs) + 2):
            loop.append(cur)
            if key(cur) == key(loop[0]):
                break
            cand = [q for p, q in nxt[key(cur)] if key(q) != key(prev) and (key(cur), key(q)) not in used and (key(q), key(cur)) not in used]
            if not cand:
                break
            prev, cur = cur, cand[0]
            used.add((key(prev), key(cur)))
        if len(loop) > 8 and key(loop[-1]) == key(loop[0]):
            loops.append(np.array(loop[:-1]))
    area = lambda p: 0.5 * abs(np.dot(p[:, 0], np.roll(p[:, 1], -1)) - np.dot(p[:, 1], np.roll(p[:, 0], -1)))
    return sorted(loops, key=area, reverse=True)


def resample(loop, n=70):
    """Section from the trailing edge, round the leading edge and back: cosine spacing on each surface."""
    P = np.asarray(loop, float)
    D = np.linalg.norm(P[:, None] - P[None], axis=2)
    i, j = np.unravel_index(np.argmax(D), D.shape)                    # the two ends of the chord
    te, le = (i, j) if P[i, 0] > P[j, 0] else (j, i)
    P = np.roll(P, -te, axis=0)
    le = (le - te) % len(P)
    P = np.vstack([P, P[:1]])
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]
    beta = 0.5 * (1 - np.cos(np.linspace(0, np.pi, n)))
    ss = np.r_[s[le] * beta, s[le] + (s[-1] - s[le]) * beta[1:-1]]
    return np.stack([np.interp(ss, s, P[:, 0]), np.interp(ss, s, P[:, 1])], axis=1), n - 1


def describe(poly, i_le):
    te, le = poly[0], poly[i_le]
    c = te - le
    chord = float(np.linalg.norm(c))
    ang = float(np.degrees(np.arctan2(c[1], c[0])))
    ex, ez = c / chord, np.array([-c[1], c[0]]) / chord               # along the chord, and normal to it
    u = np.stack([(poly - le) @ ex, (poly - le) @ ez], axis=1) / chord
    a, b = u[:i_le + 1][::-1], u[i_le:]                                # leading edge -> trailing edge on each surface
    b = np.vstack([b, u[:1]])
    xs = np.linspace(0.01, 0.99, 80)
    za, zb = np.interp(xs, a[:, 0], a[:, 1]), np.interp(xs, b[:, 0], b[:, 1])
    camber = 0.5 * (za + zb)
    flip = -1.0 if camber[np.argmax(np.abs(camber))] < 0 else 1.0      # the wing is inverted on the car: show the section upright
    u[:, 1] *= flip
    if flip < 0:
        u = np.vstack([u[:1], u[1:][::-1]])                             # keep Selig order (upper surface first)
    if u[1:len(u) // 2, 1].mean() < u[len(u) // 2:, 1].mean():
        u = np.vstack([u[:1], u[1:][::-1]])
    return {"chord": round(chord, 5), "angle_deg": round(ang, 3), "thickness_pct": round(100 * float(np.max(np.abs(za - zb))), 2),
            "camber_pct": round(100 * float(np.max(np.abs(camber))), 2), "inverted": bool(flip < 0),
            "unit": np.round(u, 5).tolist()}


def main():
    from fsae_cfd import build_car
    car = build_car(str(IC / "configs" / "ct17_cfd.yaml"))
    parts = {p.name: p for p in car.parts}
    out = {"_about": "Wing sections cut from the wing CAD by web/export_wings.py. Car axes: x rearward from the front axle, z up, y from the centre line.",
           "elements": [], "endplates": {}, "zones": {}}
    for wing, names in ENDPLATES.items():
        out["endplates"][wing] = [round(float(np.mean([parts[n].mesh.vertices[:, 1].min(), parts[n].mesh.vertices[:, 1].max()])), 4) for n in names if n in parts]
    out["endplate_boxes"] = {}                              # each plate's size from the CAD: length along the car, height, thickness, lowest edge
    for wing, names in ENDPLATES.items():
        rows = []
        for n in names:
            if n not in parts:
                continue
            v = parts[n].mesh.vertices
            lo, hi = v.min(0), v.max(0)
            area = 0.5 * sum(float(np.linalg.norm(np.cross(v[f[1]] - v[f[0]], v[f[2]] - v[f[0]]))) for f in parts[n].mesh.faces) / 2   # one face of the plate
            rows.append({"name": n, "y": round(float(0.5 * (lo[1] + hi[1])), 4), "length": round(float(hi[0] - lo[0]), 4), "height": round(float(hi[2] - lo[2]), 4),
                         "thickness": round(float(hi[1] - lo[1]), 4), "bottom": round(float(lo[2]), 4), "front": round(float(lo[0]), 4), "area": round(area, 4)})
        out["endplate_boxes"][wing] = rows
    for name, wing, part in ELEMENTS:
        if part not in parts:
            print(f"  {name}: part {part} is not in the CAD")
            continue
        m = parts[part].mesh
        y0, y1 = float(m.vertices[:, 1].min()), float(m.vertices[:, 1].max())
        if wing == "rear_wing":
            y0 = 0.0                                                    # full span: quote the half span
        ys = y0 + (y1 - y0) * np.array([0.5, 0.35, 0.65, 0.2, 0.8])
        got = None
        for y in ys:
            loops = slice_mesh(m.vertices, m.faces, float(y))
            if loops:
                got = (float(y), loops[0])
                break
        if got is None:
            print(f"  {name}: no closed section found")
            continue
        poly, i_le = resample(got[1])
        d = describe(poly, i_le)
        d.update({"name": name, "wing": wing, "y": [round(y0, 4), round(y1, 4)], "cut_at_y": round(got[0], 4), "poly": np.round(poly, 5).tolist(),
                  "span": round(2 * (y1 - y0), 4)})
        out["elements"].append(d)
        print(f"  {name}: y {y0:.3f}-{y1:.3f} m, chord {d['chord'] * 1e3:.0f} mm at {d['angle_deg']:.1f} deg, {d['thickness_pct']:.1f} % thick, {d['camber_pct']:.1f} % camber")
    for wing in ("front_wing", "rear_wing"):                              # zones: stretches of span with the same stack
        els = [e for e in out["elements"] if e["wing"] == wing]
        cuts = sorted({round(v, 3) for e in els for v in e["y"]})
        zones = []
        for a, b in zip(cuts[:-1], cuts[1:]):
            if b - a < 0.02:
                continue
            mid = 0.5 * (a + b)
            stack = [e["name"] for e in els if e["y"][0] - 1e-6 <= mid <= e["y"][1] + 1e-6]
            if stack and zones and zones[-1]["elements"] == stack:
                zones[-1]["y"][1] = b
            elif stack:
                zones.append({"y": [a, b], "elements": stack})
        out["zones"][wing] = zones
        for z in zones:
            print(f"  {wing}: y {z['y'][0]:.3f}-{z['y'][1]:.3f} m: {' + '.join(z['elements'])}")
    f = WEB / "bridge" / "wing_sections.json"
    f.write_text(json.dumps(out, separators=(",", ":")))
    print(f"-> {f} ({f.stat().st_size / 1e3:.0f} kB)")


if __name__ == "__main__":
    main()
