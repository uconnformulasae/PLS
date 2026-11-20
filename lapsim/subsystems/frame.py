"""Chassis frame: the spaceframe's tubes, a beam finite-element model of it, and what it does to the car.

Tubes
    ``load_tubes`` reads the tube list (cars/<car>/frame_tubes.csv: end points in the frame CAD's axes,
    outer size, wall, shape), extracted once from the SolidWorks part.

Spaceframe as a beam finite-element model.

Car axes throughout: x rearward from the front axle, y to the driver's right, z up from the ground
(the CFD's axes). ``to_car`` maps the frame CAD's axes (x across, y up, z forward, origin at the
front axle centre on the ground) into them.

Model
  nodes     where tube centrelines end or meet. A tube ending on another tube's side splits it
            there (coped joints); ends closer than ``tol`` are one welded node.
  elements  3-D Euler-Bernoulli beams (axial, torsion, bending about both axes), rigid joints.
  pickups   suspension chassis points: each is tied to the nearest point of the nearest tube
            through a short stiff link (the tab), so loads enter where the car puts them in.

Torsion test (``torsion_test``): the rear pickups are held, the front pickups get equal and opposite
vertical loads left / right; stiffness = moment / twist of the front pickups. Same as a twist
fixture bolted to the suspension points.

Into the lap simulation
    mass     the frame's mass and where it sits move the car's total mass, CG height and front weight
             fraction (``swap_frame``)
    twist    its torsional stiffness sits between the front and rear roll stiffness
             (``mass.chassis_torsional_stiffness`` -> ``Vehicle.elastic_share_front``): a soft frame
             pulls the lateral load-transfer split towards the weight distribution.
"""
from __future__ import annotations

import csv
import dataclasses as dc
import math
from pathlib import Path

import numpy as np

INCH = 0.0254


# --------------------------------------------------------------------------- tubes
@dc.dataclass
class Tube:
    p0: np.ndarray               # end points [m], part axes
    p1: np.ndarray
    od: float                    # outer diameter (round) or side (square) [m]
    wall: float                  # wall thickness [m]
    shape: str = "round"         # "round" | "square"
    bend: bool = False           # a piece of a bent tube
    member: int = -1             # pieces of one bent tube share a member number

    @property
    def length(self):
        return float(np.linalg.norm(self.p1 - self.p0))

    @property
    def area(self):
        """Metal cross-section [m^2]."""
        if self.shape == "square":
            return self.od ** 2 - (self.od - 2 * self.wall) ** 2
        return math.pi / 4 * (self.od ** 2 - (self.od - 2 * self.wall) ** 2)

    @property
    def inertia(self):
        """(I bending, J torsion) of the section [m^4]."""
        di = self.od - 2 * self.wall
        if self.shape == "square":
            I = (self.od ** 4 - di ** 4) / 12
            am = (self.od - self.wall) ** 2                       # thin-wall closed section (Bredt)
            return I, 4 * am ** 2 * self.wall / (4 * (self.od - self.wall))
        I = math.pi / 64 * (self.od ** 4 - di ** 4)
        return I, 2 * I

# ----------------------------------------------------------------------------- face fits


FIELDS = ("x0", "y0", "z0", "x1", "y1", "z1", "od", "wall", "shape", "bend", "member")


def load_tubes(path):
    out = []
    with open(path) as fh:
        rows = csv.DictReader(ln for ln in fh if not ln.startswith("#"))
        for r in rows:
            out.append(Tube(np.array([float(r["x0"]), float(r["y0"]), float(r["z0"])]),
                            np.array([float(r["x1"]), float(r["y1"]), float(r["z1"])]),
                            float(r["od"]), float(r["wall"]), r["shape"], bool(int(r["bend"])), int(r["member"])))
    return out


# --------------------------------------------------------------------------- beam model
STEEL_4130 = {"E": 205e9, "G": 80e9, "density": 7850.0}
CAD_AXES = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])     # car <- frame CAD


def to_car(tubes, R=CAD_AXES, origin=(0.0, 0.0, 0.0)):
    """Tubes in car axes (new list)."""
    o = np.asarray(origin, float)
    return [dc.replace(t, p0=R @ t.p0 + o, p1=R @ t.p1 + o) for t in tubes]


def load_pickups(*csv_files, prefix="CHAS_"):
    """Chassis-side suspension points from OptimumK hard-point exports (mm; X forward from the front
    axle, Y left, Z up) -> {"<file#>_<name>_<L|R>": point in car axes [m]}. File 0 = front, 1 = rear."""
    out = {}
    for k, f in enumerate(csv_files):
        for ln in Path(f).read_text().splitlines():
            c = ln.strip().split(",")
            if len(c) > 8 and c[1].startswith(prefix):
                try:
                    L = np.array([float(c[2]), float(c[3]), float(c[4])]) / 1e3
                    Rr = np.array([float(c[6]), float(c[7]), float(c[8])]) / 1e3
                except ValueError:
                    continue
                axle = "front" if k == 0 else "rear"
                out[f"{axle}_{c[1][len(prefix):]}_L"] = np.array([-L[0], -L[1], L[2]])
                out[f"{axle}_{c[1][len(prefix):]}_R"] = np.array([-Rr[0], -Rr[1], Rr[2]])
    return out


def _seg_closest(p, a, b):
    d = b - a
    s = np.clip(((p - a) @ d) / (d @ d), 0.0, 1.0)
    return a + s * d, float(s)


# ----------------------------------------------------------------------------- beam element
def _beam_k(L, A, Iy, Iz, J, E, G):
    k = np.zeros((12, 12))
    a = E * A / L
    k[0, 0] = k[6, 6] = a
    k[0, 6] = k[6, 0] = -a
    t = G * J / L
    k[3, 3] = k[9, 9] = t
    k[3, 9] = k[9, 3] = -t
    for I, (v, r, sgn) in ((Iz, (1, 5, 1.0)), (Iy, (2, 4, -1.0))):       # bending in x-y (about z), x-z (about y)
        c = E * I / L ** 3
        i, j = v, r
        m, n = v + 6, r + 6
        k[i, i] = k[m, m] = 12 * c
        k[i, m] = k[m, i] = -12 * c
        k[j, j] = k[n, n] = 4 * c * L * L
        k[j, n] = k[n, j] = 2 * c * L * L
        for p, q, val in ((i, j, 6 * c * L), (i, n, 6 * c * L), (m, j, -6 * c * L), (m, n, -6 * c * L)):
            k[p, q] = k[q, p] = sgn * val
    return k


def _rot(p0, p1):
    ex = (p1 - p0) / np.linalg.norm(p1 - p0)
    ref = np.array([0.0, 0.0, 1.0]) if abs(ex[2]) < 0.95 else np.array([0.0, 1.0, 0.0])
    ey = np.cross(ref, ex)
    ey /= np.linalg.norm(ey)
    return np.array([ex, ey, np.cross(ex, ey)])


@dc.dataclass
class Element:
    i: int
    j: int
    tube: int                    # index into FrameModel.tubes, -1 for a pickup tab
    A: float
    I: float
    J: float


class FrameModel:
    def __init__(self, tubes, pickups=None, material=None, tol=0.022, tab_od=0.0254, tab_wall=0.003):
        """``tubes``: list of Tube in car axes. ``pickups``: {name: point} (car axes) or None.
        ``tol``: ends this close [m] are welded together (a coped end stops ~ one radius short)."""
        self.tubes = list(tubes)
        self.mat = dict(material or STEEL_4130)
        self.tol = tol
        self.pickups = dict(pickups or {})
        self._build(tab_od, tab_wall)

    # ------------------------------------------------------------------ graph
    def _build(self, tab_od, tab_wall):
        tb, tol = self.tubes, self.tol
        cuts = [[0.0, 1.0] for _ in tb]                   # parameters along each tube where a node sits
        for i, t in enumerate(tb):                        # an end that lands on another tube's side splits it
            for e in (t.p0, t.p1):
                for j, u in enumerate(tb):
                    if j == i:
                        continue
                    q, s = _seg_closest(e, u.p0, u.p1)
                    if np.linalg.norm(q - e) < tol and tol < s * u.length < u.length - tol:
                        cuts[j].append(s)
        pick_at = {}
        for name, p in self.pickups.items():              # a pickup's tab lands on the nearest tube
            best = min(((np.linalg.norm(_seg_closest(p, u.p0, u.p1)[0] - p), j) for j, u in enumerate(tb)))
            j = best[1]
            s = _seg_closest(p, tb[j].p0, tb[j].p1)[1]
            cuts[j].append(s)
            pick_at[name] = (j, s, best[0])
        pts, owner = [], []
        for j, u in enumerate(tb):
            ss = sorted(set(round(s, 6) for s in cuts[j]))
            merged = [ss[0]]
            for s in ss[1:]:
                if (s - merged[-1]) * u.length > 0.4 * tol:
                    merged.append(s)
            merged[-1] = 1.0
            cuts[j] = merged
            for s in merged:
                pts.append(u.p0 + s * (u.p1 - u.p0))
                owner.append((j, s))
        pts = np.array(pts)
        parent = list(range(len(pts)))

        def find(a):
            while parent[a] != a:
                parent[a] = parent[parent[a]]
                a = parent[a]
            return a

        for a in range(len(pts)):                         # weld
            d = np.linalg.norm(pts - pts[a], axis=1)
            for b in np.flatnonzero(d < tol):
                if b > a and owner[a][0] != owner[b][0]:
                    parent[find(b)] = find(a)
        roots = sorted({find(a) for a in range(len(pts))})
        idx = {r: k for k, r in enumerate(roots)}
        node_of = np.array([idx[find(a)] for a in range(len(pts))])
        nodes = np.array([pts[node_of == k].mean(0) for k in range(len(roots))])
        els, k0 = [], 0
        for j, u in enumerate(tb):
            n = len(cuts[j])
            I, J = u.inertia
            for a in range(n - 1):
                na, nb = node_of[k0 + a], node_of[k0 + a + 1]
                if na != nb:
                    els.append(Element(int(na), int(nb), j, u.area, I, J))
            k0 += n
        self.pickup_node, self.pickup_offset = {}, {}
        nodes = list(nodes)
        Ai = math.pi / 4 * (tab_od ** 2 - (tab_od - 2 * tab_wall) ** 2)
        Ii = math.pi / 64 * (tab_od ** 4 - (tab_od - 2 * tab_wall) ** 4)
        for name, p in self.pickups.items():
            j, s, dist = pick_at[name]
            k = int(np.argmin([abs(c - s) for c in cuts[j]]))
            base = int(node_of[sum(len(c) for c in cuts[:j]) + k])
            self.pickup_offset[name] = dist
            if dist < 2e-3:
                self.pickup_node[name] = base
                continue
            nodes.append(np.asarray(p, float))
            self.pickup_node[name] = len(nodes) - 1
            els.append(Element(base, len(nodes) - 1, -1, 8 * Ai, 60 * Ii, 120 * Ii))     # the tab: stiff, weightless
        self.nodes = np.array(nodes)
        self.elements = els
        self._K = None

    def connectivity(self):
        """(number of separate pieces, nodes with a single element that are not pickups)."""
        n = len(self.nodes)
        parent = list(range(n))

        def find(a):
            while parent[a] != a:
                parent[a] = parent[parent[a]]
                a = parent[a]
            return a

        deg = np.zeros(n, int)
        for e in self.elements:
            parent[find(e.i)] = find(e.j)
            deg[e.i] += 1
            deg[e.j] += 1
        free = [k for k in range(n) if deg[k] == 1 and k not in self.pickup_node.values()]
        return len({find(a) for a in range(n)}), free

    # ------------------------------------------------------------------ mass
    # ------------------------------------------------------------------ stiffness
    def stiffness(self):
        if self._K is None:
            n = len(self.nodes)
            K = np.zeros((6 * n, 6 * n))
            E, G = self.mat["E"], self.mat["G"]
            self._ke = []
            for e in self.elements:
                p0, p1 = self.nodes[e.i], self.nodes[e.j]
                L = float(np.linalg.norm(p1 - p0))
                R = _rot(p0, p1)
                T = np.kron(np.eye(4), R)
                ke = T.T @ _beam_k(L, e.A, e.I, e.I, e.J, E, G) @ T
                dof = np.r_[6 * e.i + np.arange(6), 6 * e.j + np.arange(6)]
                K[np.ix_(dof, dof)] += ke
                self._ke.append((dof, ke))
            self._K = K
        return self._K

    def solve(self, fixed: dict, loads: dict):
        """Static solution. ``fixed``: {node: iterable of dofs 0..5 held}; ``loads``: {node: 6-vector}.
        Returns displacements (n_nodes, 6)."""
        K = self.stiffness()
        n = len(self.nodes)
        f = np.zeros(6 * n)
        for k, v in loads.items():
            f[6 * k:6 * k + 6] += np.asarray(v, float)
        held = np.zeros(6 * n, bool)
        for k, dofs in fixed.items():
            for d in dofs:
                held[6 * k + d] = True
        free = ~held
        u = np.zeros(6 * n)
        Kff = K[np.ix_(free, free)]
        scale = np.sqrt(np.clip(np.diag(Kff), 1e-30, None))          # translations and rotations differ by 1e3+
        u[free] = np.linalg.solve(Kff / np.outer(scale, scale), f[free] / scale) / scale
        return u.reshape(n, 6)

    def element_energy(self, u):
        """Strain energy of every element [J] for a displacement field."""
        uf = u.reshape(-1)
        self.stiffness()
        return np.array([0.5 * uf[dof] @ ke @ uf[dof] for dof, ke in self._ke])

    def _axle_nodes(self, axle, which=None):
        names = [n for n in self.pickup_node if n.startswith(axle + "_") and (which is None or any(w in n for w in which))]
        left = [self.pickup_node[n] for n in names if n.endswith("_L")]
        right = [self.pickup_node[n] for n in names if n.endswith("_R")]
        if not left or not right:
            raise ValueError(f"no {axle} pickups in the model (give pickups=load_pickups(front_csv, rear_csv))")
        return left, right

    def torsion_test(self, moment=1000.0, which=("LowFor", "LowAft", "UppFor", "UppAft")):
        """Twist the frame between the axles through the A-arm pickups (``which``). Returns
        {"stiffness" [N*m/rad], "stiffness_per_deg", "twist_deg", "u", "energy" per element,
        "energy_by_tube" {tube index: share}}."""
        fl, fr = self._axle_nodes("front", which)
        rl, rr = self._axle_nodes("rear", which)
        yl = np.mean(self.nodes[fl, 1])
        yr = np.mean(self.nodes[fr, 1])
        w = abs(yr - yl)
        F = moment / w
        loads = {k: np.array([0, 0, +F / len(fl), 0, 0, 0.0]) for k in fl}
        loads.update({k: np.array([0, 0, -F / len(fr), 0, 0, 0.0]) for k in fr})
        fixed = {k: (0, 1, 2) for k in rl + rr}
        u = self.solve(fixed, loads)
        dz = np.mean(u[fl, 2]) - np.mean(u[fr, 2])
        twist = dz / w
        k = moment / twist
        en = self.element_energy(u)
        by = {}
        for e, v in zip(self.elements, en):
            by[e.tube] = by.get(e.tube, 0.0) + float(v)
        tot = sum(by.values())
        return {"stiffness": k, "stiffness_per_deg": k * math.pi / 180.0, "twist_deg": math.degrees(twist), "moment": moment,
                "u": u, "energy": en, "energy_by_tube": {t: v / tot for t, v in by.items()},
                "work_check": float(0.5 * moment * twist / en.sum())}

# --------------------------------------------------------------------------- into the car
def swap_frame(cfg, old, new):
    """Copy of the car with the frame ``old`` replaced by ``new`` (both ``mass_properties()`` dicts,
    car axes: x rearward from the front axle). The rest of the car keeps its mass and position."""
    c = cfg.copy()
    L = c.geometry.wheelbase
    m0 = c.mass.total
    dm = new["mass"] - old["mass"]
    m1 = m0 + dm
    x0 = (1 - c.mass.front_weight_fraction) * L                     # car CG behind the front axle
    x1 = (m0 * x0 - old["mass"] * old["cg"][0] + new["mass"] * new["cg"][0]) / m1
    h1 = (m0 * c.mass.cg_height - old["mass"] * old["cg"][2] + new["mass"] * new["cg"][2]) / m1
    c.mass.mass_no_driver = c.mass.mass_no_driver + dm
    c.mass.front_weight_fraction = 1 - x1 / L
    c.mass.cg_height = h1
    return c
