#!/usr/bin/env python
"""Regenerate the frozen reference results that tests/golden/ compares every run against.

    python scripts/make_golden.py                                  # refuses to overwrite existing references
    python scripts/make_golden.py --force --reason "Fixed driveline drag sign"

Do this ONLY when a change is meant to move a result, and say why: the reason is appended to CHANGELOG.md.
For each car the reference holds the event times, the endurance table and, for every event's lap, the
time, speed and acceleration at every 5th point (tests/golden/reference/<car>.json, full precision).
"""
import argparse
import datetime as dt
import json
import subprocess
import sys
import warnings
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
REF = ROOT / "tests" / "golden" / "reference"
CARS = ("ct17_ic", "ct17_ev")
EVENTS = ("acceleration", "skidpad", "autocross", "endurance")
STEP = 5


def snapshot(name):
    import lapsim
    res = lapsim.load_car(name).run(EVENTS)
    laps = {e: {c: [float(x) for x in getattr(res.laps[e], c)[::STEP]] for c in ("s", "t", "v", "ax")} for e in EVENTS}
    end = res.endurance[["lap_time_s"]].reset_index().to_dict(orient="list")
    return {"car": name, "times": {k: float(v) for k, v in res.times.items()}, "laps": laps, "endurance": end}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--force", action="store_true", help="overwrite existing references")
    ap.add_argument("--reason", default="", help="why the results are meant to change (goes in CHANGELOG.md)")
    a = ap.parse_args()
    warnings.filterwarnings("ignore")
    REF.mkdir(parents=True, exist_ok=True)
    exists = [c for c in CARS if (REF / f"{c}.json").exists()]
    if exists and not a.force:
        sys.exit(f"references exist for {exists}: pass --force --reason '...' to replace them")
    if exists and not a.reason:
        sys.exit("give --reason: why are the results meant to change?")
    for c in CARS:
        snap = snapshot(c)
        (REF / f"{c}.json").write_text(json.dumps(snap, indent=0))
        print(f"{c}: " + ", ".join(f"{e} {t:.4f} s" for e, t in snap["times"].items()))
    if a.reason:
        commit = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=ROOT, capture_output=True, text=True).stdout.strip()
        log = ROOT / "CHANGELOG.md"
        text = log.read_text() if log.exists() else "# Changelog\n"
        log.write_text(text.rstrip() + f"\n\n## {dt.date.today()}: golden results regenerated (from {commit})\n\n{a.reason}\n")


if __name__ == "__main__":
    main()
