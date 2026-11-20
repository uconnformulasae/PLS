"""The `lapsim` command.

    lapsim cars                                            # the car files in cars/
    lapsim check-config ct17_ev                            # load a car file, report problems and placeholders
    lapsim register ct17_ev [--csv out.csv]                # every parameter: value, unit, note, source
    lapsim run --car ct17_ic --event autocross             # one event (default: all four)
    lapsim run --car ct17_ev --set vehicle:mass.cg_height="11 in" --set ev:strategy.power_limit="60 kW"
    lapsim run --car ct17_ic --solver transient --event skidpad
    lapsim run --car ct17_ic --out results/accel_test      # write the run (events, laps, metadata)
    lapsim compare results/a results/b                     # two written runs side by side

``--set`` keys are '<section>:<dotted.path>' with section vehicle, pt (IC powertrain), ev (EV powertrain)
or pack (EV accumulator) - the same keys the website exports.
"""
from __future__ import annotations

import argparse
import sys
import warnings

import pandas as pd

from lapsim.core.events import EVENTS


def _car(name, sets=()):
    from lapsim.config.loader import load_car
    car = load_car(name)
    for s in sets:
        key, _, value = s.partition("=")
        if not value:
            sys.exit(f"--set {s!r}: expected section:path=value")
        car.set(key.strip(), value.strip())
    return car


def cmd_cars(a):
    from lapsim.config.loader import CARS
    for d in sorted(CARS.iterdir()):
        if (d / "vehicle.yaml").is_file():
            print(d.name)


def cmd_check(a):
    with warnings.catch_warnings(record=True):
        car = _car(a.car)
        veh = car.build()
    print(f"{car.name} ({car.kind}): {car.source}")
    for n in veh.warnings:
        print(f"  note: {n}")
    st = car.status()
    if st is not None:
        from lapsim.config.register import status_counts
        c = status_counts(st)
        print(f"  values: {c['confirmed']} confirmed, {c['placeholder']} placeholder, {c['missing']} missing")
    print("  OK: the car file loads and the vehicle model builds")


def cmd_register(a):
    from lapsim.config.register import parameters
    df = parameters(_car(a.car))
    if a.csv:
        df.to_csv(a.csv, index=False)
        print(f"wrote {a.csv} ({len(df)} parameters)")
    else:
        df["value"] = [v if isinstance(v, (int, float)) or v is None else str(v)[:40] for v in df["value"]]
        with pd.option_context("display.max_rows", None, "display.width", 200, "display.max_colwidth", 50):
            print(df.to_string(index=False))


def cmd_run(a):
    car = _car(a.car, a.set or [])
    events = tuple(a.event) if a.event else EVENTS
    res = car.run(events)
    if a.solver == "transient":
        from lapsim.solvers.transient import Chassis
        rows = []
        for e in events:
            lr = res.laps.get(e)
            if lr is None or e == "endurance":
                continue
            r = Chassis(res.vehicle).run(lr.track, lr, v0=1.0 if e in ("acceleration", "autocross") else None, name=e)
            rows.append({"event": e, "qss_s": lr.time, "transient_s": r.summary["time_s"],
                         "completed": r.summary["completed"]})
        print(pd.DataFrame(rows).set_index("event").round(3).to_string())
    else:
        print(res.table().round(3).to_string())
    for n in res.notes:
        print(f"note: {n}")
    if a.out:
        from lapsim.core.results import write_run
        print(f"wrote {write_run(res, car, a.out)}")


def cmd_compare(a):
    from lapsim.validation.compare import compare
    d = compare(a.a, a.b)
    print(d["events"].round(4).to_string())
    for e, df in d["channels"].items():
        if len(df):
            print(f"\n{e}: largest channel differences")
            print(df.head(8).to_string())


def main(argv=None):
    ap = argparse.ArgumentParser(prog="lapsim", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("cars", help="list the car files").set_defaults(fn=cmd_cars)
    p = sub.add_parser("check-config", help="load a car file and build the vehicle")
    p.add_argument("car")
    p.set_defaults(fn=cmd_check)
    p = sub.add_parser("register", help="every parameter with its unit, note and source")
    p.add_argument("car")
    p.add_argument("--csv")
    p.set_defaults(fn=cmd_register)
    p = sub.add_parser("run", help="run events")
    p.add_argument("--car", required=True)
    p.add_argument("--event", action="append", choices=EVENTS)
    p.add_argument("--solver", default="qss", choices=["qss", "transient"])
    p.add_argument("--set", action="append", metavar="SECTION:PATH=VALUE")
    p.add_argument("--out")
    p.set_defaults(fn=cmd_run)
    p = sub.add_parser("compare", help="compare two written runs")
    p.add_argument("a")
    p.add_argument("b")
    p.set_defaults(fn=cmd_compare)
    a = ap.parse_args(argv)
    warnings.filterwarnings("ignore")
    a.fn(a)


if __name__ == "__main__":
    main()
