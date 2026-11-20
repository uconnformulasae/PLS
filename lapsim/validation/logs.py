"""Logged runs: loading RaceStudio / AiM .csv exports, cleaning, laps and the checks set against the model.

Loading
    Every channel keeps its own native time base and sample rate; nothing is
    interpolated at load time (the processing module resamples with proper
    anti-aliasing). Channels are mapped to standard names and converted to SI.

    Standard channel names used throughout the package
    ---------------------------------------------------
    speed_gps [m/s]  lat, lon [deg]  alt [m]  heading_gps [deg]  gps_nsat [-]
    gps_pos_acc [m]  gps_spd_acc [m/s]  ax, ay, az [m/s^2] (vehicle x fwd, y left, z up)
    yaw_rate, roll_rate, pitch_rate [rad/s]  steer [deg at steering wheel]
    wheel_fl ... wheel_rr [m/s]  damper_fl ... damper_rr [m] (compression +)
    throttle [%]  brake_f, brake_r [Pa]  rpm [1/min]  beacon [-]

Cleaning (``process_log``)
     1. SANITISE      sort, de-duplicate time stamps, drop NaN/inf, find gaps
     2. RANGE CHECK   physically impossible values -> masked (not clipped)
     3. HAMPEL        rolling median +- k*1.4826*MAD spike removal
     4. STUCK SENSOR  zero variance while the car is moving -> masked
     5. RESAMPLE      to one time base; anti-alias low-pass before any down-sampling,
                      monotone PCHIP for up-sampling, never across a gap
     6. CUT-OFF       per-channel low-pass cut-off from Winter's residual analysis
     7. ZERO-PHASE    4th-order Butterworth run forward and backward (sosfiltfilt)
     8. DERIVATIVES   Savitzky-Golay (damper velocity, yaw acceleration)
     9. IMU ALIGN     gyro bias from stationary periods; accelerometer mounting
                      rotation from gravity (roll/pitch) and from GPS (yaw)
    10. FUSION        Kalman filter + Rauch-Tung-Striebel smoother: accelerometer +
                      GPS speed (+ wheel speeds) -> speed, distance, accel bias,
                      with Mahalanobis gating of GPS glitches
    11. PATH          GPS -> local x,y; smoothing spline whose smoothing is set by
                      the GPS position variance; curvature fused with yaw-rate/speed
                      by inverse-variance weighting
    12. LAPS          logger laps, beacon or start/finish gate; outlier-lap rejection
    13. ENSEMBLE      laps mapped to distance, aligned by curvature cross-correlation,
                      combined with median / MAD (robust mean with a confidence band)

    Each step records what it did in a QC report so that the cleaning can be
    audited. See README section "Data processing" for why each step is used.

Checks against the model: ``gg_points``, ``lateral_envelope`` (grip), ``coastdown_mask`` / ``fit_coastdown``
(drag area and rolling resistance from coasting).
"""
from __future__ import annotations

import csv
import dataclasses as dc
import re
from pathlib import Path

import numpy as np
import pandas as pd
from scipy import interpolate, optimize, signal

from lapsim.config.schema import CarConfig, MissingDataError
from lapsim.config.units import G0

# regex aliases (case-insensitive) -> standard name. Extend or override with
# ``channel_map={"Exact Logger Name": "std_name"}`` when loading.
CHANNEL_ALIASES = {
    r"^gps[ _]?speed$": "speed_gps",
    r"^gps[ _]?lat(itude)?$": "lat",
    r"^gps[ _]?lon(gitude)?$": "lon",
    r"^gps[ _]?alt(itude)?$": "alt",
    r"^gps[ _]?heading$": "heading_gps",
    r"^gps[ _]?nsat$": "gps_nsat",
    r"^gps[ _]?pos[ _]?accuracy$": "gps_pos_acc",
    r"^gps[ _]?spd[ _]?accuracy$": "gps_spd_acc",
    r"^(inline[ _]?acc|acc[ _]?x|long(itudinal)?[ _]?acc)$": "ax",
    r"^(lateral[ _]?acc|acc[ _]?y|lat[ _]?acc)$": "ay",
    r"^(vertical[ _]?acc|acc[ _]?z|vert[ _]?acc)$": "az",
    r"^(yaw[ _]?rate|gyro[ _]?z)$": "yaw_rate",
    r"^(roll[ _]?rate|gyro[ _]?x)$": "roll_rate",
    r"^(pitch[ _]?rate|gyro[ _]?y)$": "pitch_rate",
    r"^(steer(ing)?([ _]?(angle|pos(ition)?))?)$": "steer",
    r"^(wheel[ _]?speed[ _]?|speed[ _]?|ws[ _]?)fl$": "wheel_fl",
    r"^(wheel[ _]?speed[ _]?|speed[ _]?|ws[ _]?)fr$": "wheel_fr",
    r"^(wheel[ _]?speed[ _]?|speed[ _]?|ws[ _]?)rl$": "wheel_rl",
    r"^(wheel[ _]?speed[ _]?|speed[ _]?|ws[ _]?)rr$": "wheel_rr",
    r"^(damper|shock|susp(ension)?)[ _]?(pos(ition)?[ _]?)?fl$": "damper_fl",
    r"^(damper|shock|susp(ension)?)[ _]?(pos(ition)?[ _]?)?fr$": "damper_fr",
    r"^(damper|shock|susp(ension)?)[ _]?(pos(ition)?[ _]?)?rl$": "damper_rl",
    r"^(damper|shock|susp(ension)?)[ _]?(pos(ition)?[ _]?)?rr$": "damper_rr",
    r"^(throttle|tps|pedal|apps)([ _]?pos(ition)?)?$": "throttle",
    r"^brake[ _]?(press(ure)?[ _]?)?(f|front)$": "brake_f",
    r"^brake[ _]?(press(ure)?[ _]?)?(r|rear)$": "brake_r",
    r"^(engine[ _]?|motor[ _]?)?rpm$": "rpm",
    r"^(beacon|lap[ _]?marker)$": "beacon",
}

# unit string -> factor to the SI/standard unit of that channel
_UNIT_FACTORS = {
    "km/h": 1 / 3.6, "kph": 1 / 3.6, "mph": 0.44704, "m/s": 1.0,
    "g": G0, "m/s2": 1.0, "m/s^2": 1.0,
    "deg/s": np.pi / 180, "rad/s": 1.0,
    "mm": 1e-3, "m": 1.0, "in": 0.0254,
    "bar": 1e5, "psi": 6894.757, "kpa": 1e3, "pa": 1.0,
    "%": 1.0, "deg": 1.0, "rpm": 1.0, "#": 1.0, "": 1.0,
}
_DEFAULT_UNITS = {"speed_gps": "km/h", "ax": "g", "ay": "g", "az": "g",
                  "yaw_rate": "deg/s", "roll_rate": "deg/s", "pitch_rate": "deg/s"}


@dc.dataclass
class Channel:
    t: np.ndarray            # time [s]
    x: np.ndarray            # values in standard units
    unit_raw: str = ""
    name_raw: str = ""

@dc.dataclass
class LogData:
    channels: dict            # std_name -> Channel
    raw_names: dict           # raw name -> std name (or None if unmapped)
    laps: list | None = None  # [(start_s, end_s), ...] from the logger, if present
    meta: dict = dc.field(default_factory=dict)
    source: str = ""

    def __getitem__(self, name) -> Channel:
        return self.channels[name]

def _std_name(raw: str, channel_map: dict | None):
    if channel_map and raw in channel_map:
        return channel_map[raw]
    key = raw.strip().lower()
    for pat, std in CHANNEL_ALIASES.items():
        if re.match(pat, key):
            return std
    return None


def _to_si(std: str, values: np.ndarray, unit: str) -> np.ndarray:
    u = (unit or _DEFAULT_UNITS.get(std, "")).strip().lower().replace("²", "2")
    if std in ("lat", "lon", "heading_gps", "steer", "throttle", "gps_nsat", "rpm", "beacon"):
        return values
    if std.startswith("wheel_") and u == "":
        u = "km/h"
    f = _UNIT_FACTORS.get(u)
    if f is None:
        raise ValueError(f"channel '{std}' has unrecognised unit '{unit}'; pass units in channel_map "
                         "or convert before loading")
    return values * f


# --------------------------------------------------------------------- loaders
def load_log(path: str | Path, channel_map: dict | None = None) -> LogData:
    """Load a RaceStudio / AiM .csv export. ``channel_map`` maps raw names to standard names.
    (.xrk / .xrz: export the session to .csv from RaceStudio first.)"""
    path = Path(path)
    ext = path.suffix.lower()
    if ext == ".csv":
        return _load_csv(path, channel_map)
    raise ValueError(f"unsupported file type {ext}: export the session to .csv from RaceStudio")


def _is_num(s: str) -> bool:
    try:
        float(s)
        return True
    except ValueError:
        return False


def _load_csv(path: Path, channel_map):
    """RaceStudio CSV export (metadata header + Time/units rows) or a plain CSV with a time column."""
    with open(path, newline="", encoding="utf-8-sig", errors="replace") as fh:
        rows = list(csv.reader(fh))
    # the channel header is the first row starting with 'Time' that has further non-empty cells
    # (RaceStudio's metadata block also has a 'Time' row: "Time","10:00:00")
    hdr = next((i for i, r in enumerate(rows)
                if r and r[0].strip().lower() in ("time", "time (s)", "time [s]", "t", "time_s", "timestamp")
                and sum(1 for c in r[1:] if c.strip()) >= 2), None)
    if hdr is None:
        raise ValueError(f"{path.name}: no column named 'Time' found")
    header = [h.strip() for h in rows[hdr]]
    units = [""] * len(header)
    start = hdr + 1
    if start < len(rows) and rows[start] and not _is_num(rows[start][0] or "x"):
        units = [u.strip() for u in rows[start]] + [""] * len(header)
        start += 1
    meta = {r[0].strip(): r[1].strip() for r in rows[:hdr] if len(r) >= 2 and r[0].strip()}
    # RaceStudio exports list lap (beacon) times in a 'Beacon Markers' header row
    beacons = next(([float(v) for v in r[1:] if _is_num(v)] for r in rows[:hdr]
                    if r and r[0].strip().lower() == "beacon markers"), [])
    data = []
    for r in rows[start:]:
        if not r or not _is_num(r[0]):
            continue
        data.append([float(v) if _is_num(v) else np.nan for v in (r + [""] * len(header))[:len(header)]])
    arr = np.asarray(data, dtype=float)
    t = arr[:, 0]
    if units[0].lower() in ("ms",):
        t = t / 1000.0
    chans, names = {}, {}
    for j, raw in enumerate(header[1:], start=1):
        std = _std_name(raw, channel_map)
        names[raw] = std
        if std is None:
            continue
        good = np.isfinite(arr[:, j])
        chans[std] = Channel(t[good], _to_si(std, arr[good, j], units[j]), units[j], raw)
    laps = [(a, b) for a, b in zip(beacons[:-1], beacons[1:])] if len(beacons) >= 2 else None
    if beacons:
        meta["beacon_markers"] = beacons
    return LogData(chans, names, laps, meta, str(path))


# physical limits in standard units; values outside are masked as invalid
PHYS_LIMITS = {
    "speed_gps": (0, 60), "ax": (-5 * G0, 5 * G0), "ay": (-5 * G0, 5 * G0), "az": (-2 * G0, 5 * G0),
    "yaw_rate": (-5, 5), "roll_rate": (-5, 5), "pitch_rate": (-5, 5), "steer": (-270, 270),
    "wheel_fl": (0, 60), "wheel_fr": (0, 60), "wheel_rl": (0, 60), "wheel_rr": (0, 60),
    "damper_fl": (-0.1, 0.2), "damper_fr": (-0.1, 0.2), "damper_rl": (-0.1, 0.2), "damper_rr": (-0.1, 0.2),
    "throttle": (-5, 105), "brake_f": (-2e5, 2.5e7), "brake_r": (-2e5, 2.5e7),
    "lat": (-90, 90), "lon": (-180, 180), "rpm": (0, 25000),
}
# channels whose content is useful only at low frequency (bounded cut-off search)
CUTOFF_RANGE = {"speed_gps": (0.5, 5), "ax": (1, 12), "ay": (1, 12), "az": (2, 20), "yaw_rate": (1, 10),
                "roll_rate": (1, 10), "pitch_rate": (1, 10), "steer": (1, 10), "throttle": (2, 15),
                "brake_f": (2, 20), "brake_r": (2, 20), "damper_fl": (5, 40), "damper_fr": (5, 40),
                "damper_rl": (5, 40), "damper_rr": (5, 40), "wheel_fl": (1, 10), "wheel_fr": (1, 10),
                "wheel_rl": (1, 10), "wheel_rr": (1, 10)}
NO_FILTER = {"lat", "lon", "gps_nsat", "beacon", "gps_pos_acc", "gps_spd_acc", "heading_gps", "alt"}


# ----------------------------------------------------------------- 1-4 cleaning
def sanitize(ch: Channel):
    t, x = np.asarray(ch.t, float), np.asarray(ch.x, float)
    good = np.isfinite(t) & np.isfinite(x)
    t, x = t[good], x[good]
    order = np.argsort(t, kind="stable")
    t, x = t[order], x[order]
    keep = np.r_[True, np.diff(t) > 0]
    n_dup = int((~keep).sum())
    return t[keep], x[keep], {"n_nonfinite": int((~good).sum()), "n_duplicate_t": n_dup}


def find_gaps(t: np.ndarray, factor: float = 3.0):
    """Return list of (t_start, t_end) where sampling stopped for > factor x nominal dt."""
    dt = np.diff(t)
    if dt.size == 0:
        return []
    nominal = np.median(dt)
    idx = np.where(dt > factor * nominal)[0]
    return [(t[i], t[i + 1]) for i in idx]


def hampel(x: np.ndarray, window: int, k: float = 4.0):
    """Hampel identifier. Points further than k robust-sigmas from the rolling
    median are replaced by the median. Returns (cleaned, outlier_mask)."""
    window = max(3, int(window) | 1)
    s = pd.Series(x)
    med = s.rolling(window, center=True, min_periods=1).median()
    mad = (s - med).abs().rolling(window, center=True, min_periods=1).median()
    sigma = 1.4826 * mad
    floor = 1e-9 + 1e-6 * np.nanmax(np.abs(x)) if x.size else 1e-9
    out = (s - med).abs() > k * np.maximum(sigma, floor)
    y = s.where(~out, med).to_numpy()
    return y, out.to_numpy()


def stuck_mask(x: np.ndarray, fs: float, moving: np.ndarray | None, seconds: float = 1.0):
    """True where the signal is perfectly constant for > seconds while moving."""
    n = max(3, int(seconds * fs))
    std = pd.Series(x).rolling(n, center=True, min_periods=n).std().to_numpy()
    m = np.nan_to_num(std, nan=1.0) == 0.0
    # every sample inside a zero-variance window is stuck (dilate by the window length)
    m = pd.Series(m.astype(float)).rolling(n, center=True, min_periods=1).max().to_numpy() > 0
    if moving is not None:
        m &= moving
    return m


# ----------------------------------------------------------------- 5 resampling
def resample(t: np.ndarray, x: np.ndarray, t_new: np.ndarray, gaps=(), fs_new: float | None = None):
    """Anti-aliased resampling onto t_new. Down-sampling: zero-phase low-pass at
    0.4*fs_new on a uniform native grid first. Up-sampling: PCHIP (monotone,
    no overshoot). Samples inside data gaps or outside the record become NaN."""
    if t.size < 4:
        return np.full_like(t_new, np.nan, dtype=float)
    fs_native = 1.0 / np.median(np.diff(t))
    fs_new = fs_new or 1.0 / np.median(np.diff(t_new))
    if fs_native > 1.5 * fs_new:
        tu = np.arange(t[0], t[-1], 1.0 / fs_native)
        xu = np.interp(tu, t, x)
        sos = signal.butter(6, 0.4 * fs_new, fs=fs_native, output="sos")
        xu = signal.sosfiltfilt(sos, xu)
        y = np.interp(t_new, tu, xu, left=np.nan, right=np.nan)
    else:
        y = interpolate.PchipInterpolator(t, x, extrapolate=False)(t_new)
    for a, b in gaps:
        y[(t_new > a) & (t_new < b)] = np.nan
    return y


# ------------------------------------------------------ 6-8 filtering, cut-off
def _fill_nan(x):
    x = np.asarray(x, float).copy()
    bad = ~np.isfinite(x)
    if bad.all():
        return x, bad
    if bad.any():
        idx = np.arange(x.size)
        x[bad] = np.interp(idx[bad], idx[~bad], x[~bad])
    return x, bad


def lowpass(x, fs, fc, order=4):
    """Zero-phase Butterworth low-pass (no time lag between channels). NaNs are
    bridged for filtering and restored afterwards."""
    if fc is None or fc >= 0.49 * fs:
        return np.asarray(x, float)
    xf, bad = _fill_nan(x)
    if np.isnan(xf).all() or xf.size < 3 * (order + 1) * 3:
        return np.asarray(x, float)
    sos = signal.butter(order, fc, fs=fs, output="sos")
    y = signal.sosfiltfilt(sos, xf)
    y[bad] = np.nan
    return y


def winter_cutoff(x, fs, f_lo=0.5, f_hi=None, n=40):
    """Winter's residual analysis.

    The RMS residual between the raw signal and its low-passed version is
    computed for a range of cut-offs. At high cut-offs the residual is pure
    noise and falls linearly with fc; extrapolating that line to fc=0 gives the
    noise RMS. The chosen cut-off is where the residual curve meets that noise
    level: below it, signal is being removed; above it, noise is let through.
    Returns (fc, noise_rms, (fcs, residuals)).
    """
    xf, bad = _fill_nan(x)
    f_hi = f_hi or 0.45 * fs
    f_hi = min(f_hi, 0.45 * fs)
    fcs = np.linspace(f_lo, f_hi, n)
    res = np.array([np.sqrt(np.mean((xf - lowpass(xf, fs, fc)) ** 2)) for fc in fcs])
    tail = fcs > fcs[0] + 0.6 * (fcs[-1] - fcs[0])
    p = np.polyfit(fcs[tail], res[tail], 1)
    noise = max(p[1], 1e-12)
    above = np.where(res <= noise)[0]
    fc = fcs[above[0]] if above.size else fcs[-1]
    return float(fc), float(noise), (fcs, res)


def savgol_derivative(x, fs, fc, polyorder=3):
    """First derivative with a Savitzky-Golay filter whose -3 dB point is ~fc.
    Window half-width M from Schafer (2011): fc/f_nyq ~ (N+1)/(3.2M-4.6)."""
    fn = fc / (0.5 * fs)
    M = int(np.ceil(((polyorder + 1) / max(fn, 1e-3) + 4.6) / 3.2))
    win = 2 * M + 1
    xf, bad = _fill_nan(x)
    if win >= xf.size:
        win = (xf.size // 2) * 2 - 1
    d = signal.savgol_filter(xf, win, polyorder, deriv=1, delta=1.0 / fs)
    d[bad] = np.nan
    return d


# ------------------------------------------------------------ 9 IMU alignment
def _rot_to_z(v):
    """Rotation matrix that maps vector v onto +z (Rodrigues)."""
    v = v / np.linalg.norm(v)
    z = np.array([0.0, 0.0, 1.0])
    c = float(np.dot(v, z))
    axis = np.cross(v, z)
    s = np.linalg.norm(axis)
    if s < 1e-12:
        return np.eye(3)
    axis /= s
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + s * K + (1 - c) * K @ K


def align_imu(df: pd.DataFrame, fs: float, qc: dict):
    """Remove gyro bias and accelerometer mounting misalignment in place."""
    if not {"ax", "ay"}.issubset(df):
        return
    v = df.get("speed_gps")
    stationary = None
    if v is not None:
        still = (v.fillna(99) < 0.3).to_numpy()
        # keep only stationary runs longer than 1 s
        n = int(fs)
        run = pd.Series(still.astype(int)).rolling(n, center=True, min_periods=n).sum().to_numpy() >= n
        stationary = still & run
    # gyro bias
    for g in ("yaw_rate", "roll_rate", "pitch_rate"):
        if g in df and stationary is not None and stationary.sum() > fs:
            b = float(np.nanmedian(df.loc[stationary, g]))
            df[g] = df[g] - b
            qc.setdefault("imu", {})[f"{g}_bias"] = b
    # roll/pitch mounting from gravity
    if "az" in df:
        sel = stationary if (stationary is not None and stationary.sum() > fs) else np.isfinite(df["az"].to_numpy())
        gvec = np.nanmean(df.loc[sel, ["ax", "ay", "az"]].to_numpy(), axis=0)
        if np.linalg.norm(gvec) > 0.5 * G0:
            R = _rot_to_z(gvec)
            A = df[["ax", "ay", "az"]].to_numpy().T
            A = R @ A
            df["ax"], df["ay"], df["az"] = A[0], A[1], A[2] - G0  # az now = vertical dynamic accel
            qc.setdefault("imu", {})["gravity_alignment_deg"] = float(np.degrees(np.arccos(
                np.clip(gvec[2] / np.linalg.norm(gvec), -1, 1))))
    # yaw mounting from GPS: ax_ref = dv/dt, ay_ref = v*r
    if v is not None and "yaw_rate" in df:
        vv = lowpass(v.to_numpy(), fs, 2.0)
        ax_ref = np.gradient(vv, 1.0 / fs)
        ay_ref = vv * lowpass(df["yaw_rate"].to_numpy(), fs, 2.0)
        ax_m = lowpass(df["ax"].to_numpy(), fs, 2.0)
        ay_m = lowpass(df["ay"].to_numpy(), fs, 2.0)
        ok = np.isfinite(ax_ref + ay_ref + ax_m + ay_m) & (vv > 5)
        if ok.sum() > 10 * fs:
            num = np.sum(ax_m[ok] * ay_ref[ok] - ay_m[ok] * ax_ref[ok])
            den = np.sum(ax_m[ok] * ax_ref[ok] + ay_m[ok] * ay_ref[ok])
            psi = np.arctan2(num, den)
            c, s = np.cos(psi), np.sin(psi)
            ax0, ay0 = df["ax"].to_numpy(), df["ay"].to_numpy()
            df["ax"], df["ay"] = c * ax0 - s * ay0, s * ax0 + c * ay0
            qc.setdefault("imu", {})["yaw_misalignment_deg"] = float(np.degrees(psi))


# ---------------------------------------------------------- 10 Kalman + RTS
def kalman_rts_speed(t, ax, v_meas, v_sigma, a_sigma=0.5, bias_walk=0.02, gate=5.0, wheel=None, wheel_sigma=None):
    """Fuse longitudinal acceleration with speed measurements.

    State x = [s, v, b] (distance, speed, accelerometer bias); input u = ax.
        s' = v,  v' = u - b + w_a,  b' = w_b
    Measurements: GPS speed (variance v_sigma^2, per sample if an array), and
    optionally mean undriven wheel speed. Innovations whose normalised size
    exceeds ``gate`` sigma are rejected (GPS glitches). A Rauch-Tung-Striebel
    backward pass then gives the optimal smoothed estimate using all data.
    Returns dict with s, v, b, v_std, n_rejected.
    """
    n = t.size
    dt = np.diff(t, prepend=t[0])
    x = np.array([0.0, np.nanmedian(v_meas[:10]) if np.isfinite(v_meas[:10]).any() else 0.0, 0.0])
    P = np.diag([1.0, 4.0, 0.5])
    xs, Ps, xp_s, Pp_s, Fs = (np.zeros((n, 3)), np.zeros((n, 3, 3)), np.zeros((n, 3)),
                              np.zeros((n, 3, 3)), np.zeros((n, 3, 3)))
    vs = np.broadcast_to(np.asarray(v_sigma, float), (n,))
    rejected, streak = 0, 0
    max_streak = int(1.0 / max(np.median(dt[1:]) if n > 1 else 0.01, 1e-3))   # ~1 s of rejections
    for k in range(n):
        h = dt[k]
        Fm = np.array([[1, h, -0.5 * h * h], [0, 1, -h], [0, 0, 1]])
        Bm = np.array([0.5 * h * h, h, 0.0])
        Q = np.diag([0.25 * h ** 4 * a_sigma ** 2, h ** 2 * a_sigma ** 2, h * bias_walk ** 2])
        u = ax[k] if np.isfinite(ax[k]) else 0.0
        x = Fm @ x + Bm * u
        P = Fm @ P @ Fm.T + Q
        xp_s[k], Pp_s[k], Fs[k] = x, P, Fm
        for z, r in ((v_meas[k], vs[k] ** 2),
                     ((wheel[k], wheel_sigma ** 2) if wheel is not None else (np.nan, 1.0))):
            if not np.isfinite(z):
                continue
            S = P[1, 1] + r
            innov = z - x[1]
            if innov * innov / S > gate * gate:
                streak += 1
                if streak < max_streak:
                    rejected += 1
                    continue
                # persistent disagreement = the filter, not the sensor, is wrong:
                # inflate the speed variance and accept (re-acquisition)
                P[1, 1] += innov * innov
                S = P[1, 1] + r
            streak = 0
            K = P[:, 1] / S
            x = x + K * innov
            P = P - np.outer(K, P[1, :])
        xs[k], Ps[k] = x, P
    # RTS backward pass
    xsm, Psm = xs.copy(), Ps.copy()
    for k in range(n - 2, -1, -1):
        C = Ps[k] @ Fs[k + 1].T @ np.linalg.inv(Pp_s[k + 1])
        xsm[k] = xs[k] + C @ (xsm[k + 1] - xp_s[k + 1])
        Psm[k] = Ps[k] + C @ (Psm[k + 1] - Pp_s[k + 1]) @ C.T
    return {"s": xsm[:, 0] - xsm[0, 0], "v": xsm[:, 1], "b": xsm[:, 2],
            "v_std": np.sqrt(np.maximum(Psm[:, 1, 1], 0)), "n_rejected": rejected}


# ---------------------------------------------------------------- 11 path
def latlon_to_xy(lat, lon):
    """Local tangent-plane coordinates [m] about the mean point (error < 1 mm/km)."""
    lat0, lon0 = np.nanmean(lat), np.nanmean(lon)
    R = 6378137.0
    x = np.radians(lon - lon0) * R * np.cos(np.radians(lat0))
    y = np.radians(lat - lat0) * R
    return x, y


def smooth_path_curvature(s, x, y, pos_sigma):
    """Smoothing splines x(s), y(s). The smoothing factor is N*sigma^2, the
    expected sum of squared residuals for white position noise of std sigma,
    so the spline removes noise but not real path shape (Reinsch criterion)."""
    ok = np.isfinite(s + x + y)
    s_, x_, y_ = s[ok], x[ok], y[ok]
    keep = np.r_[True, s_[1:] > np.maximum.accumulate(s_)[:-1] + 1e-3]
    s_, x_, y_ = s_[keep], x_[keep], y_[keep]
    N = s_.size
    sx = interpolate.UnivariateSpline(s_, x_, k=5, s=N * pos_sigma ** 2)
    sy = interpolate.UnivariateSpline(s_, y_, k=5, s=N * pos_sigma ** 2)
    d1x, d1y, d2x, d2y = sx.derivative(1)(s), sy.derivative(1)(s), sx.derivative(2)(s), sy.derivative(2)(s)
    kappa = (d1x * d2y - d1y * d2x) / np.maximum((d1x ** 2 + d1y ** 2) ** 1.5, 1e-9)
    return sx(s), sy(s), kappa


def fuse_curvature(kappa_gps, kappa_gps_var, yaw_rate, v, yaw_sigma):
    """Inverse-variance fusion of GPS-path curvature with r/v.
    var(r/v) = (yaw_sigma/v)^2, so r/v dominates at speed and GPS at low speed."""
    v = np.asarray(v, float)
    k_imu = np.where(v > 3, yaw_rate / np.maximum(v, 3), np.nan)
    var_imu = np.where(v > 3, (yaw_sigma / np.maximum(v, 3)) ** 2, np.inf)
    w1 = 1 / kappa_gps_var
    w2 = np.where(np.isfinite(k_imu), 1 / var_imu, 0.0)
    return (w1 * kappa_gps + w2 * np.nan_to_num(k_imu)) / (w1 + w2)


# ---------------------------------------------------------------- 12 laps
def laps_from_gate(t, x, y, gate_xy, gate_heading_deg=None, radius=15.0, min_lap_s=10.0):
    """Find start/finish crossings: closest-approach passes within ``radius``
    of the gate point, at least ``min_lap_s`` apart."""
    d = np.hypot(x - gate_xy[0], y - gate_xy[1])
    near = d < radius
    passes, k = [], 0
    while k < d.size:
        if near[k]:
            j = k
            while j < d.size and near[j]:
                j += 1
            i = k + int(np.nanargmin(d[k:j]))
            if not passes or t[i] - passes[-1] > min_lap_s:
                passes.append(t[i])
            k = j
        else:
            k += 1
    return [(a, b) for a, b in zip(passes[:-1], passes[1:])]


def laps_from_beacon(t, beacon, min_lap_s=10.0):
    edges = t[np.where(np.diff((np.nan_to_num(beacon) > 0.5).astype(int)) == 1)[0] + 1]
    edges = [e for i, e in enumerate(edges) if i == 0 or e - edges[i - 1] > min_lap_s]
    return list(zip(edges[:-1], edges[1:]))


def reject_outlier_laps(laps, z=3.0, max_ratio=1.07):
    """Drop laps whose time is > max_ratio x median or > z robust sigmas away."""
    if len(laps) < 3:
        return laps, []
    times = np.array([b - a for a, b in laps])
    med = np.median(times)
    mad = 1.4826 * np.median(np.abs(times - med)) + 1e-9
    good = (times < max_ratio * med) & (np.abs(times - med) / mad < z)
    return [l for l, g in zip(laps, good) if g], [l for l, g in zip(laps, good) if not g]


# -------------------------------------------------------------- 13 ensemble
def distance_ensemble(df, laps, channels, n_grid=2000, max_shift_frac=0.02):
    """Map each lap onto a normalised distance grid, align laps by maximising the
    cross-correlation of curvature (corrects distance-integration drift), then
    combine with median and MAD. Random noise in the median shrinks roughly as
    1/sqrt(N laps) while lap-to-lap driving variation is kept as the band.
    Returns DataFrame with s_frac, s, <ch>_med, <ch>_mad and lap length."""
    grid = np.linspace(0, 1, n_grid, endpoint=False)
    per_ch = {c: [] for c in channels}
    lengths, shifts = [], []
    ref = None
    for a, b in laps:
        seg = df[(df["t"] >= a) & (df["t"] < b)]
        if len(seg) < 50:
            continue
        s = seg["s"].to_numpy() - seg["s"].iloc[0]
        L = s[-1]
        lengths.append(L)
        frac = s / L
        curves = {c: np.interp(grid, frac, seg[c].to_numpy()) for c in channels if c in seg}
        if "kappa" in curves:
            if ref is None:
                ref = curves["kappa"]
                sh = 0
            else:
                m = int(max_shift_frac * n_grid)
                lags = np.arange(-m, m + 1)
                cc = [np.nansum(ref * np.roll(curves["kappa"], -L_)) for L_ in lags]
                sh = int(lags[int(np.argmax(cc))])
            shifts.append(sh)
            curves = {c: np.roll(v, -sh) for c, v in curves.items()}
        for c, v in curves.items():
            per_ch[c].append(v)
    out = {"s_frac": grid, "s": grid * (np.median(lengths) if lengths else 1.0)}
    for c, arrs in per_ch.items():
        if not arrs:
            continue
        A = np.vstack(arrs)
        med = np.nanmedian(A, axis=0)
        out[c + "_med"] = med
        out[c + "_mad"] = 1.4826 * np.nanmedian(np.abs(A - med), axis=0)
    res = pd.DataFrame(out)
    res.attrs.update({"n_laps": len(lengths), "lap_length_m": float(np.median(lengths)) if lengths else np.nan,
                      "shifts": shifts})
    return res


# ----------------------------------------------------------------- pipeline
@dc.dataclass
class ProcessedLog:
    df: pd.DataFrame            # cleaned, fused, common time base (column 't' in s)
    fs: float
    qc: dict                    # what every step did, per channel
    laps: list
    rejected_laps: list
    raw: LogData

def process_log(log: LogData, fs: float = 100.0, hampel_window_s: float = 0.15, hampel_k: float = 4.0,
                gate_latlon=None, gps_pos_sigma: float | None = None, gps_spd_sigma: float | None = None,
                cutoffs: dict | None = None) -> ProcessedLog:
    """Run the full cleaning / fusion pipeline. ``cutoffs`` overrides the
    automatic cut-off for named channels (Hz)."""
    qc = {"channels": {}}
    cutoffs = cutoffs or {}
    t0 = min(c.t.min() for c in log.channels.values())
    t1 = max(c.t.max() for c in log.channels.values())
    t_new = np.arange(t0, t1, 1.0 / fs)
    df = pd.DataFrame({"t": t_new})
    # steps 1-5 per channel
    for name, ch in log.channels.items():
        t, x, info = sanitize(ch)
        if t.size < 4:
            continue
        gaps = find_gaps(t)
        lo, hi = PHYS_LIMITS.get(name, (-np.inf, np.inf))
        bad = (x < lo) | (x > hi)
        info["n_out_of_range"] = int(bad.sum())
        t, x = t[~bad], x[~bad]
        if name not in NO_FILTER and t.size > 10:
            fs_c = 1.0 / np.median(np.diff(t))
            x, spikes = hampel(x, hampel_window_s * fs_c, hampel_k)
            info["n_spikes"] = int(spikes.sum())
        info.update({"fs_native": float(1.0 / np.median(np.diff(t))), "n_gaps": len(gaps)})
        if name in ("lat", "lon"):
            df[name] = np.interp(t_new, t, x, left=np.nan, right=np.nan)
            for a, b in gaps:
                df.loc[(t_new > a) & (t_new < b), name] = np.nan
        else:
            df[name] = resample(t, x, t_new, gaps, fs)
        qc["channels"][name] = info
    moving = (df["speed_gps"].fillna(0) > 2).to_numpy() if "speed_gps" in df else None
    for name in list(log.channels):
        if name in df and name not in NO_FILTER:
            m = stuck_mask(df[name].to_numpy(), fs, moving)
            if m.any():
                df.loc[m, name] = np.nan
            qc["channels"][name]["n_stuck"] = int(m.sum())
    # step 9 alignment (on resampled, unfiltered data)
    align_imu(df, fs, qc)
    # steps 6-7: cut-off by residual analysis then zero-phase filter
    for name in list(df.columns):
        if name in NO_FILTER or name == "t":
            continue
        x = df[name].to_numpy()
        if np.isfinite(x).sum() < 5 * fs:
            continue
        lo, hi = CUTOFF_RANGE.get(name, (0.5, 0.45 * fs))
        if name in cutoffs:
            fc, noise = cutoffs[name], np.nan
        else:
            fc, noise, _ = winter_cutoff(x, fs, lo, min(hi, 0.45 * fs))
        df[name + "_raw"] = x
        df[name] = lowpass(x, fs, fc)
        sig = np.nanstd(df[name])
        qc["channels"].setdefault(name, {}).update(
            {"cutoff_hz": fc, "noise_rms": noise, "snr_db": float(20 * np.log10(sig / noise)) if noise and noise > 0 else np.nan})
    # step 8 derivatives
    for c in ("damper_fl", "damper_fr", "damper_rl", "damper_rr"):
        if c in df:
            df[c + "_vel"] = savgol_derivative(df[c + "_raw"].to_numpy(), fs, qc["channels"][c]["cutoff_hz"])
    # step 10 fusion: speed / distance
    if "speed_gps" in df:
        vm = df["speed_gps_raw"].to_numpy() if "speed_gps_raw" in df else df["speed_gps"].to_numpy()
        if gps_spd_sigma is None:
            if "gps_spd_acc" in df and np.isfinite(df["gps_spd_acc"]).any():
                vsig = np.clip(np.nan_to_num(df["gps_spd_acc"].to_numpy(), nan=0.5), 0.05, 5.0)
            else:
                vsig = max(qc["channels"].get("speed_gps", {}).get("noise_rms", 0.2) or 0.2, 0.05)
        else:
            vsig = gps_spd_sigma
        ax = df["ax"].to_numpy() if "ax" in df else np.zeros(len(df))
        wheel = None
        fronts = [c for c in ("wheel_fl", "wheel_fr") if c in df]   # undriven (RWD): no wheel spin
        if fronts:
            wheel = np.nanmean(np.vstack([df[c].to_numpy() for c in fronts]), axis=0)
        # process noise: residual accelerometer error after filtering plus model error
        a_sig = max(qc["channels"].get("ax", {}).get("noise_rms", 0.3) or 0.3, 0.3)
        kf = kalman_rts_speed(df["t"].to_numpy(), ax, vm, vsig, a_sigma=a_sig,
                              wheel=wheel, wheel_sigma=0.3 if wheel is not None else None)
        df["v"], df["s"], df["v_std"], df["ax_bias"] = kf["v"], kf["s"], kf["v_std"], kf["b"]
        qc["fusion"] = {"gps_rejected": kf["n_rejected"], "ax_bias_mean": float(np.nanmean(kf["b"]))}
        if "ax" in df:
            df["ax"] = df["ax"] - df["ax_bias"]
    # step 11 path & curvature
    if {"lat", "lon", "s"}.issubset(df):
        x, y = latlon_to_xy(df["lat"].to_numpy(), df["lon"].to_numpy())
        df["x_raw"], df["y_raw"] = x, y
        if gps_pos_sigma is None:
            gps_pos_sigma = (float(np.nanmedian(df["gps_pos_acc"])) if "gps_pos_acc" in df else 0.5)
        moving_ok = df["v"].to_numpy() > 1.0
        s = df["s"].to_numpy()
        xs = np.where(moving_ok, x, np.nan)
        ys = np.where(moving_ok, y, np.nan)
        try:
            xsm, ysm, kgps = smooth_path_curvature(s, xs, ys, gps_pos_sigma)
            df["x"], df["y"] = xsm, ysm
            kvar = (np.nanstd(kgps - lowpass(kgps, fs, 0.5)) ** 2) + 1e-8
            if "yaw_rate" in df:
                ysig = qc["channels"].get("yaw_rate", {}).get("noise_rms", 0.01) or 0.01
                df["kappa"] = fuse_curvature(kgps, kvar, df["yaw_rate"].to_numpy(), df["v"].to_numpy(), ysig)
            else:
                df["kappa"] = kgps
            qc["path"] = {"gps_pos_sigma_m": gps_pos_sigma}
        except Exception as e:  # pragma: no cover
            qc["path"] = {"error": str(e)}
    elif {"yaw_rate", "v"}.issubset(df):
        df["kappa"] = np.where(df["v"] > 3, df["yaw_rate"] / df["v"].clip(lower=3), 0.0)
    # step 12 laps
    laps = []
    if log.laps:
        laps = [(a, b) for a, b in log.laps if b > a]
    elif "beacon" in df:
        laps = laps_from_beacon(df["t"].to_numpy(), df["beacon"].to_numpy())
    elif gate_latlon is not None and {"x", "y"}.issubset(df):
        lat0, lon0 = np.nanmean(df["lat"]), np.nanmean(df["lon"])
        R = 6378137.0
        gx = np.radians(gate_latlon[1] - lon0) * R * np.cos(np.radians(lat0))
        gy = np.radians(gate_latlon[0] - lat0) * R
        laps = laps_from_gate(df["t"].to_numpy(), df["x"].to_numpy(), df["y"].to_numpy(), (gx, gy))
    good, bad = reject_outlier_laps(laps)
    qc["laps"] = {"found": len(laps), "kept": len(good), "rejected": len(bad)}
    return ProcessedLog(df, fs, qc, good, bad, log)


def qc_table(plog: ProcessedLog) -> pd.DataFrame:
    rows = []
    for name, d in plog.qc["channels"].items():
        rows.append({"channel": name, **d})
    return pd.DataFrame(rows).set_index("channel")


# --------------------------------------------------------------------------- checks against the model
def gg_points(plog, v_min=5.0):
    d = plog.df
    sel = (d["v"] > v_min) if "v" in d else np.ones(len(d), bool)
    return d.loc[sel, ["v", "ax", "ay"]].dropna()


def lateral_envelope(plog, v_bins=np.arange(5, 31, 2.5), ax_tol=0.15 * G0, q=0.98):
    """Lateral limit vs speed from near-pure cornering samples (|ax| < ax_tol)."""
    p = gg_points(plog)
    p = p[np.abs(p["ax"]) < ax_tol]
    rows = []
    for a, b in zip(v_bins[:-1], v_bins[1:]):
        s = (p["v"] >= a) & (p["v"] < b)
        if s.sum() >= 30:
            rows.append({"v": 0.5 * (a + b), "ay_env": np.quantile(np.abs(p.loc[s, "ay"]), q), "n": int(s.sum())})
    return pd.DataFrame(rows)


def coastdown_mask(plog, throttle_max=2.0, brake_max=2e5, ay_max=0.1 * G0, v_min=6.0):
    d = plog.df
    m = (d["v"] > v_min) & (d["ay"].abs() < ay_max)
    if "throttle" in d:
        m &= d["throttle"] < throttle_max
    else:
        raise MissingDataError("coast-down fit needs a throttle channel (or pass your own mask)")
    for b in ("brake_f", "brake_r"):
        if b in d:
            m &= d[b] < brake_max
    return m.to_numpy()


def fit_coastdown(plog, cfg: CarConfig, mask=None):
    """Fit CdA and Crr from coasting:  m*ax = -(0.5*rho*CdA*v^2 + Crr*(m*g + downforce)).
    Linear in the unknowns; solved with a Huber loss."""
    d = plog.df
    mask = coastdown_mask(plog) if mask is None else mask
    # deceleration from the fused, GPS-anchored speed rather than the accelerometer: a slowly
    # drifting accelerometer bias (or pitch) would otherwise trade off directly against Crr
    dvdt = savgol_derivative(d["v"].to_numpy(), plog.fs, 1.0)
    v, ax = d["v"].to_numpy()[mask], dvdt[mask]
    m = cfg.mass.total
    cla = cfg.aero.cla() or (0, 0)
    q = 0.5 * cfg.aero.rho * v ** 2
    N = m * G0 + (cla[0] + cla[1]) * q
    A = np.column_stack([q, N])
    y = -m * ax
    r = optimize.least_squares(lambda p: A @ p - y, [1.0, 0.015], loss="huber", f_scale=20.0,
                               bounds=([0, 0], [5, 0.1]))
    cov = np.linalg.pinv(r.jac.T @ r.jac) * np.sum(r.fun ** 2) / max(len(y) - 2, 1)
    return {"CdA": r.x[0], "Crr": r.x[1], "CdA_sd": float(np.sqrt(cov[0, 0])),
            "Crr_sd": float(np.sqrt(cov[1, 1])), "n": int(mask.sum())}
