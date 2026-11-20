"""3-D suspension kinematics from OptimumK hardpoint exports (CT17 front and rear).

Coordinate system of the export (team note): mm, origin on the ground under the
centre of the front axle, x forward, y to the LEFT, z up. Only the left corner
is solved; the right is its mirror image.

For any wheel travel (and any pull-rod / tie-rod length adjustment) the solver
finds the pose of the upright from three constraints:
    |UBJ - LBJ| = constant,   |tie rod| = its length,   wheel-centre height = target
with the lower and upper arms rotating about their chassis axes. From the pose it
computes camber, toe, caster, KPI, scrub radius, mechanical trail, track change,
the front- and side-view instant centres (intersection of the arm planes), the
roll-centre height, anti-dive / anti-lift / anti-squat, the rocker angle (pull-rod
length is fixed), damper length (-> motion ratio), pull-rod angle and the ARB
drop-link / blade rotation (-> ARB roll stiffness from geometry).
"""
from __future__ import annotations

import csv
import dataclasses as dc
from pathlib import Path

import numpy as np
from scipy import optimize

MM = 1e-3


# ------------------------------------------------------------------ parsing
def load_optimumk_csv(path):
    """Parse an OptimumK suspension export (.csv). Returns dict with
    'points' (left side, metres), 'wheel' (SI / deg), 'attach', 'name', 'meta'."""
    with open(path, newline="", encoding="utf-8-sig", errors="replace") as fh:
        rows = list(csv.reader(fh))
    pts, wheel, attach, meta = {}, {}, {}, {}
    section = None
    for r in rows:
        r = [c.strip() for c in r] + [""] * 9
        if r[0]:
            section = r[0]
        if r[1] == "Name:":
            meta["name"] = r[2]
        if r[1] in ("Created on:", "Version:"):
            meta[r[1].rstrip(":")] = r[2]
        if section in ("Double A-Arm", "Push Pull", "U-Bar") and r[1] and r[1] not in ("Point Name", "") \
                and _num(r[2]) and _num(r[3]) and _num(r[4]):
            key = r[1][:-2] if r[1].endswith("_L") else r[1]
            pts[key] = np.array([float(r[2]), float(r[3]), float(r[4])]) * MM
        if section == "Double A-Arm" and r[1] == "Tierod Attachment":
            meta["tierod_attachment"] = r[2]
        if section == "Rack Pinion" or r[1] == "Steering Ratio":
            if r[1] == "Steering Ratio" and _num(r[2]):
                meta["steering_ratio_raw"] = float(r[2])
        if section == "Wheels" and r[1] and _num(r[2]):
            wheel[r[1]] = float(r[2])
        if section == "Attachment" and r[1] in ("Push Pull", "U-Bar"):
            attach[r[1]] = (r[2], r[3], r[4])
    w = {"half_track": wheel["Half Track"] * MM, "x_offset": wheel.get("Longitudinal Offset", 0) * MM,
         "camber_deg": wheel.get("Static Camber", 0.0), "toe_deg": wheel.get("Static Toe", 0.0),
         "rim_diameter": wheel.get("Rim Diameter", 0) * MM, "tire_diameter": wheel["Tire Diameter"] * MM,
         "tire_width": wheel.get("Tire Width", 0) * MM}
    return {"points": pts, "wheel": w, "attach": attach, "name": meta.get("name", Path(path).stem), "meta": meta}


def _num(s):
    try:
        float(s)
        return True
    except ValueError:
        return False


# ------------------------------------------------------------------ geometry helpers
def _rot(axis, ang):
    axis = axis / np.linalg.norm(axis)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K


def _rot_about_point(p, centre, axis, ang):
    return centre + _rot(axis, ang) @ (p - centre)


def _align(a, b):
    """Rotation matrix taking unit vector a onto unit vector b."""
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    s = np.linalg.norm(v)
    if s < 1e-12:
        return np.eye(3)
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]]) / s
    th = np.arctan2(s, c)
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


def _plane(p1, p2, p3):
    n = np.cross(p2 - p1, p3 - p1)
    return n / np.linalg.norm(n), p1


def _plane_intersection(P1, P2):
    (n1, a1), (n2, a2) = P1, P2
    d = np.cross(n1, n2)
    if np.linalg.norm(d) < 1e-12:
        return None
    A = np.array([n1, n2, d])
    b = np.array([n1 @ a1, n2 @ a2, d @ a1])
    return np.linalg.solve(A, b), d / np.linalg.norm(d)


def _line_plane(p, d, n, a):
    den = n @ d
    if abs(den) < 1e-12:
        return None
    return p + d * ((n @ (a - p)) / den)


# ------------------------------------------------------------------ corner
@dc.dataclass
class Pose:
    travel: float
    LBJ: np.ndarray
    UBJ: np.ndarray
    wc: np.ndarray
    cp: np.ndarray
    spin: np.ndarray
    up: np.ndarray
    rocker: float
    damper_len: float
    pp_upr: np.ndarray
    rod_pnt: np.ndarray
    blade_angle: float | None
    tie_upr: np.ndarray | None = None


class Corner:
    """Kinematics of one (left) corner. ``front`` selects brake/drive conventions only."""

    def __init__(self, data, front=True):
        self.d = data
        P = data["points"]
        self.P = P
        w = data["wheel"]
        self.front = front
        self.R = w["tire_diameter"] / 2
        g, t = np.radians(w["camber_deg"]), np.radians(w["toe_deg"])
        cp = np.array([w["x_offset"], w["half_track"], 0.0])
        up = np.array([0.0, np.sin(g), np.cos(g)])            # negative camber -> top toward -y (chassis)
        spin = np.array([0.0, np.cos(g), -np.sin(g)])
        Rt = _rot(np.array([0, 0, 1.0]), -t)                  # toe-in positive for the left wheel
        up, spin = Rt @ up, Rt @ spin
        wc = cp + self.R * up
        self.cp0, self.wc0, self.up0, self.spin0 = cp, wc, up, spin
        self.LBJ0, self.UBJ0 = P["UPRI_LowPnt"], P["UPRI_UppPnt"]
        self.kbj = np.linalg.norm(self.UBJ0 - self.LBJ0)
        self.tie_ch0 = P["CHAS_TiePnt"]
        self.tie_up0 = P["UPRI_TiePnt"]
        self.tie_len0 = np.linalg.norm(self.tie_up0 - self.tie_ch0)
        self.pp_up0 = P["NSMA_PPAttPnt"]                       # outboard end of the pull / push rod
        # what the rod's outboard end is bolted to (the file's Attachment row): the upright (CT17),
        # or an A-arm (the usual push-rod layout) - then the point swings with that arm
        att = str((data.get("attach", {}).get("Push Pull") or ("", "", "Upright"))[2]).lower()
        self.rod_on = "lower" if "low" in att else "upper" if "upp" in att else "upright"
        self.rod_len0 = np.linalg.norm(self.pp_up0 - P["ROCK_RodPnt"])
        self.rock_c = P["CHAS_RocPiv"]
        self.rock_ax = (P["CHAS_RocAxi"] - P["CHAS_RocPiv"])
        self.rock_ax /= np.linalg.norm(self.rock_ax)
        self.damper_len0 = np.linalg.norm(P["ROCK_CoiPnt"] - P["CHAS_AttPnt"])
        self.has_arb = all(k in P for k in ("NSMA_UBarAttPnt", "UBAR_AttPnt", "CHAS_PivPnt"))
        if self.has_arb:
            self.drop_len0 = np.linalg.norm(P["NSMA_UBarAttPnt"] - P["UBAR_AttPnt"])
            self.blade_len = np.linalg.norm((P["UBAR_AttPnt"] - P["CHAS_PivPnt"])[[0, 2]])
        # upright-fixed points expressed relative to LBJ
        self._upr = {"UBJ": self.UBJ0, "tie": self.tie_up0, "pp": self.pp_up0, "wc": wc, "cp": cp}
        self._dir = {"up": up, "spin": spin}

    # ---- arm rotation about chassis pivot axis
    def _arm(self, bj0, a, b, th):
        return _rot_about_point(bj0, a, b - a, th)

    def _upright(self, LBJ, UBJ, phi):
        """Place upright-fixed points for new ball-joint positions and rotation phi about the BJ axis."""
        e0 = (self.UBJ0 - self.LBJ0) / self.kbj
        e1 = (UBJ - LBJ) / np.linalg.norm(UBJ - LBJ)
        R = _rot(e1, phi) @ _align(e0, e1)
        pts = {k: LBJ + R @ (v - self.LBJ0) for k, v in self._upr.items()}
        dirs = {k: R @ v for k, v in self._dir.items()}
        return pts, dirs

    def solve(self, travel=0.0, d_tie=0.0, d_rod=0.0, guess=None):
        """Pose for wheel-centre vertical travel [m] (+ bump), tie-rod length change and
        pull-rod length change [m]."""
        P = self.P
        tie_len = self.tie_len0 + d_tie

        def F(x):
            thL, thU, phi = x
            LBJ = self._arm(self.LBJ0, P["CHAS_LowFor"], P["CHAS_LowAft"], thL)
            UBJ = self._arm(self.UBJ0, P["CHAS_UppFor"], P["CHAS_UppAft"], thU)
            pts, _ = self._upright(LBJ, UBJ, phi)
            return [np.linalg.norm(UBJ - LBJ) - self.kbj,
                    np.linalg.norm(pts["tie"] - self.tie_ch0) - tie_len,
                    pts["wc"][2] - (self.wc0[2] + travel)]
        x = optimize.fsolve(F, guess if guess is not None else [0.0, 0.0, 0.0], xtol=1e-10)
        thL, thU, phi = x
        LBJ = self._arm(self.LBJ0, P["CHAS_LowFor"], P["CHAS_LowAft"], thL)
        UBJ = self._arm(self.UBJ0, P["CHAS_UppFor"], P["CHAS_UppAft"], thU)
        pts, dirs = self._upright(LBJ, UBJ, phi)
        # tire contact: lowest point of the wheel along 'up'
        cp = pts["wc"] - self.R * dirs["up"]
        if self.rod_on == "lower":
            pp = self._arm(self.pp_up0, P["CHAS_LowFor"], P["CHAS_LowAft"], thL)
        elif self.rod_on == "upper":
            pp = self._arm(self.pp_up0, P["CHAS_UppFor"], P["CHAS_UppAft"], thU)
        else:
            pp = pts["pp"]
        beta, rodp = self._rocker(pp, self.rod_len0 + d_rod)
        coil = _rot_about_point(P["ROCK_CoiPnt"], self.rock_c, self.rock_ax, beta)
        dl = np.linalg.norm(coil - P["CHAS_AttPnt"])
        blade = self._blade(beta) if self.has_arb else None
        pose = Pose(travel, LBJ, UBJ, pts["wc"], cp, dirs["spin"], dirs["up"], beta, dl, pp, rodp, blade, pts["tie"])
        pose._x = x
        return pose

    def _rocker(self, pp, rod_len):
        P = self.P
        f = lambda b: np.linalg.norm(_rot_about_point(P["ROCK_RodPnt"], self.rock_c, self.rock_ax, b) - pp) - rod_len
        b = optimize.brentq(f, -1.2, 1.2) if f(-1.2) * f(1.2) < 0 else optimize.fsolve(f, 0.0)[0]
        return b, _rot_about_point(P["ROCK_RodPnt"], self.rock_c, self.rock_ax, b)

    def _blade(self, beta):
        """Blade rotation about the lateral torsion axis that keeps the drop-link length.
        The drop link is attached to the rocker (team sketch)."""
        P = self.P
        att = _rot_about_point(P["NSMA_UBarAttPnt"], self.rock_c, self.rock_ax, beta)
        ax = np.array([0, 1.0, 0])
        f = lambda a: np.linalg.norm(_rot_about_point(P["UBAR_AttPnt"], P["CHAS_PivPnt"], ax, a) - att) - self.drop_len0
        try:
            return optimize.brentq(f, -0.8, 0.8)
        except ValueError:
            return float(optimize.fsolve(f, 0.0)[0])

    # ---- derived quantities for a pose
    def angles(self, pose):
        s, u = pose.spin, pose.up
        camber = np.degrees(np.arcsin(np.clip(u[1], -1, 1)))          # + top outboard (left wheel: +y)
        toe = np.degrees(np.arctan2(-s[0], s[1]))                       # + toe-in
        k = pose.UBJ - pose.LBJ
        kpi = np.degrees(np.arctan2(-k[1], k[2]))                       # + top inboard
        caster = np.degrees(np.arctan2(-k[0], k[2]))                    # + top rearward
        t = -pose.LBJ[2] / k[2]
        g = pose.LBJ + t * k                                            # steering axis at ground
        scrub = pose.cp[1] - g[1]
        trail = g[0] - pose.cp[0]
        return {"camber_deg": camber, "toe_deg": toe, "kpi_deg": kpi, "caster_deg": caster,
                "scrub_radius": scrub, "mech_trail": trail, "track_half": pose.cp[1]}

    def instant_axis(self, pose):
        P = self.P
        lp = _plane(P["CHAS_LowFor"], P["CHAS_LowAft"], pose.LBJ)
        upl = _plane(P["CHAS_UppFor"], P["CHAS_UppAft"], pose.UBJ)
        return _plane_intersection(lp, upl)

    def front_view_ic(self, pose):
        p, d = self.instant_axis(pose)
        return _line_plane(p, d, np.array([1.0, 0, 0]), pose.cp)

    def side_view_ic(self, pose):
        p, d = self.instant_axis(pose)
        return _line_plane(p, d, np.array([0, 1.0, 0]), pose.cp)

    def roll_centre_height(self, pose):
        """Front-view RC for symmetric (heave) motion: line contact patch -> IC meets y = 0."""
        ic = self.front_view_ic(pose)
        cp = pose.cp
        t = -cp[1] / (ic[1] - cp[1])
        return float(cp[2] + t * (ic[2] - cp[2])), ic

    def fvsa(self, pose):
        ic = self.front_view_ic(pose)
        return float(np.hypot(ic[1] - pose.cp[1], ic[2] - pose.cp[2]))


@dc.dataclass
class AxleKinematics:
    """Pre-computed curves for one axle versus wheel travel (and at static)."""
    name: str
    travel: np.ndarray
    camber: np.ndarray          # deg, + top outboard (body-referenced, wheel travel only)
    toe: np.ndarray
    rc_height: np.ndarray
    damper_len: np.ndarray
    blade: np.ndarray | None
    static: dict

    def at(self, key, z):
        return np.interp(z, self.travel, getattr(self, key))


def analyse_corner(c: Corner, travel=np.linspace(-0.03, 0.03, 25), h_cg=None, wheelbase=None,
                   front_brake=None, d_rod=0.0, d_tie=0.0):
    """Sweep travel and return AxleKinematics plus a static summary."""
    poses, guess = [], None
    order = np.argsort(np.abs(travel))
    res = [None] * len(travel)
    # solve outward from static for robust convergence
    for sgn in (1, -1):
        guess = None
        for i in sorted([i for i in range(len(travel)) if np.sign(travel[i]) in (sgn, 0)], key=lambda i: abs(travel[i])):
            p = c.solve(travel[i], d_tie, d_rod, guess)
            guess = p._x
            res[i] = p
    cam = np.array([c.angles(p)["camber_deg"] for p in res])
    toe = np.array([c.angles(p)["toe_deg"] for p in res])
    rc = np.array([c.roll_centre_height(p)[0] for p in res])
    dl = np.array([p.damper_len for p in res])
    bl = np.array([p.blade_angle for p in res]) if c.has_arb else None
    p0 = c.solve(0.0, d_tie, d_rod)
    a0 = c.angles(p0)
    rc0, ic0 = c.roll_centre_height(p0)
    sv = c.side_view_ic(p0)
    # motion ratios (central differences)
    h = 1e-4
    pp, pm = c.solve(h, d_tie, d_rod, p0._x), c.solve(-h, d_tie, d_rod, p0._x)
    mr_dw = -(pp.damper_len - pm.damper_len) / (2 * h)               # damper shortening per wheel bump
    rod_dir = p0.rod_pnt - p0.pp_upr
    rod_angle = np.degrees(np.arccos(abs(rod_dir[2]) / np.linalg.norm(rod_dir)))
    summary = {**a0, "rc_height": rc0, "fv_ic_y": ic0[1], "fv_ic_z": ic0[2], "fvsa": c.fvsa(p0),
               "camber_gain_deg_per_mm": (c.angles(pp)["camber_deg"] - c.angles(pm)["camber_deg"]) / (2 * h * 1000),
               "bump_steer_deg_per_mm": (c.angles(pp)["toe_deg"] - c.angles(pm)["toe_deg"]) / (2 * h * 1000),
               "track_change_mm_per_mm": (c.angles(pp)["track_half"] - c.angles(pm)["track_half"]) / (2 * h),
               "motion_ratio_damper_per_wheel": mr_dw, "motion_ratio_wheel_per_damper": 1 / mr_dw,
               "damper_length_static": p0.damper_len, "pull_rod_length": c.rod_len0 + d_rod,
               "pull_rod_angle_from_vertical_deg": rod_angle, "sv_ic_x": sv[0], "sv_ic_z": sv[2],
               "kingpin_axis_len": c.kbj}
    if c.has_arb:
        summary["blade_rot_per_wheel_travel_rad_per_m"] = (pp.blade_angle - pm.blade_angle) / (2 * h)
        summary["blade_length_in_file"] = c.blade_len
        summary["drop_link_length"] = c.drop_len0
    if h_cg is not None and wheelbase is not None:
        # side-view anti geometry with the binder (B19) definitions; x is + forward
        k = h_cg / wheelbase
        if c.front:
            tan_dive = sv[2] / (p0.cp[0] - sv[0])                 # line contact patch -> IC, rising rearward
            if front_brake:
                summary["anti_dive"] = tan_dive * front_brake / k
        else:
            tan_lift = sv[2] / (sv[0] - p0.cp[0])                 # outboard brakes: from the contact patch
            tan_squat = (sv[2] - p0.wc[2]) / (sv[0] - p0.wc[0])   # inboard drive: from the wheel centre
            if front_brake:
                summary["anti_lift"] = tan_lift * (1 - front_brake) / k
            summary["anti_squat"] = tan_squat / k
    return AxleKinematics(c.d["name"], travel, cam - cam[np.argmin(np.abs(travel))], toe, rc, dl, bl, summary)


def camber_per_shim(c: Corner, thickness):
    """Camber change [deg] when shims of total ``thickness`` move the upper ball joint
    outboard relative to the upright (rotation about the lower ball joint)."""
    return np.degrees(np.arctan2(thickness, c.kbj))
