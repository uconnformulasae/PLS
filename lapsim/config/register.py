"""Parameter register: every value of a car with its unit, note and source, and which values are
confirmed measurements and which are still placeholders."""
from __future__ import annotations

import dataclasses as dc

import pandas as pd


def parameters(car) -> pd.DataFrame:
    """One row per value of the car file: section, key, value (SI), unit, note, source."""
    rows = []

    def walk(obj, section, prefix):
        for f in dc.fields(obj):
            if f.name in ("source", "listed", "confirmed") or f.name.startswith("_"):
                continue
            v = getattr(obj, f.name)
            if dc.is_dataclass(v):
                walk(v, section, f"{prefix}{f.name}.")
            else:
                rows.append({"section": section, "key": f"{prefix}{f.name}", "value": v,
                             "unit": f.metadata.get("unit", ""), "note": f.metadata.get("note", ""),
                             "source": f.metadata.get("source", "")})
    for which, obj in car.objects().items():
        walk(obj, which, "")
    return pd.DataFrame(rows)


def input_status(cfg, overlay, evc, bcfg=None) -> pd.DataFrame:
    """One row per value listed in an overlay car file (the EV): its value and whether it is confirmed
    or still a placeholder. Pack values are all reported as assumed until the pack is characterised."""
    rows = []
    settings = ("_file", "_track")                      # file choices, not measurements
    for k in [k for k in overlay["listed"] if not k.endswith(settings)]:
        v = cfg.get(k)
        st = "confirmed" if k in overlay["confirmed"] else ("MISSING" if v is None else
                                                             "setting" if isinstance(v, str) else "PLACEHOLDER")
        rows.append({"file": "vehicle", "key": k, "value_SI": v, "status": st})
    for k in [k for k in getattr(evc, "listed", []) if not k.endswith(settings)]:
        v = evc.get(k)
        optional = v is None or k.split(".")[0] in ("strategy", "scoring") or k.endswith(("name", "type", "_options"))
        st = "confirmed" if k in evc.confirmed else ("setting" if optional else "PLACEHOLDER")
        rows.append({"file": "powertrain", "key": k, "value_SI": v, "status": st})
    if bcfg is not None:
        for sec in ("pack", "cell", "limits", "thermal"):
            for f in dc.fields(getattr(bcfg, sec)):
                v = getattr(getattr(bcfg, sec), f.name)
                if isinstance(v, (int, float, str)) and not isinstance(v, bool):
                    rows.append({"file": "pack", "key": f"{sec}.{f.name}", "value_SI": v,
                                 "status": "ASSUMED (see the pack section)"})
    return pd.DataFrame(rows)


def status_counts(status: pd.DataFrame) -> dict:
    s = status[~status["status"].str.startswith("ASSUMED")]
    return {"confirmed": int((s["status"] == "confirmed").sum()),
            "placeholder": int((s["status"] == "PLACEHOLDER").sum()),
            "missing": int((s["status"] == "MISSING").sum())}
