#!/usr/bin/env python
"""Build the lap-simulation into a website (a static site, ready for GitHub Pages).

    python web/build.py                # bundle the simulation + write the baseline results
    python web/build.py --serve        # ... then serve it at http://localhost:8000
    python web/build.py --no-studies   # skip the EV power-limit / gearing studies (faster)

What it writes:
    site/py/lapsim.zip      the Python packages, configs and track files the browser runs (Pyodide)
    site/data/ic.json       baseline CT-17 IC inputs + results (shown the moment the page opens)
    site/data/ev.json       baseline CT-17 EV inputs + results + studies
    site/data/meta.json     build time, git commit, file list
    site/**/*.js copies     the same files as scripts, so index.html also works opened by double-click

The baseline results are computed FROM THE BUNDLE (unzipped to a temporary folder), so a build that
succeeds proves the bundle holds every file the simulation needs.

Only the files listed in BUNDLE go on the website. Raw TTC tire data, logs, CAD and results are
never copied (see web/README.md, "What is published").
"""
import argparse
import datetime as dt
import json
import os
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path

WEB = Path(__file__).resolve().parent
# fsae_lapsim/ (the folder holding IC/ and ev/). If web/ lives in another repository, point at it:
#   LAPSIM_ROOT=/path/to/fsae_lapsim python web/build.py
ROOT = Path(os.environ.get("LAPSIM_ROOT") or WEB.parent).resolve()
SITE = WEB / "site"

# (folder relative to fsae_lapsim/, glob patterns)
BUNDLE = [
    ("IC/fsae_vd", ["*.py"]),
    ("IC/fsae_pt", ["*.py"]),
    ("IC/fsae_frame", ["*.py"]),
    ("IC/configs", ["ct17_baseline.yaml", "ct17_powertrain.yaml"]),
    ("IC/configs/tracks", ["*.yaml"]),
    ("IC/data/hardpoints", ["*.csv"]),
    ("IC/data/dyno", ["*.csv"]),
    ("IC/data/frame", ["*.csv"]),
    ("IC/data/tracks/ic_tracks2026", ["*.csv"]),
    ("IC/data/tracks", ["*.csv"]),
    ("ev/fsae_ev", ["*.py"]),
    ("ev/configs", ["*.yaml"]),
    ("ev/configs/tracks", ["*.yaml"]),
    ("ev/fsae_ev_bms/fsae_ev_bms", ["*.py"]),
    ("ev/fsae_ev_bms/configs", ["*.yaml"]),
    ("web/bridge", ["*.py"]),
]
# modules that need packages the browser cannot load (CasADi / IPOPT, ipywidgets): left out of the bundle
EXCLUDE = {"ocp.py", "chassis_sym.py", "chassis_corner_sym.py", "mf61_sym.py", "powertrain_sym.py", "validate_mb.py",
           "mb_panel.py", "mb_study.py", "plots_mb.py"}


def _rel(f: Path) -> str:
    base = WEB.parent if WEB in f.parents else ROOT
    return str(f.relative_to(base)).replace(os.sep, "/")


def bundle(out: Path):
    files = []
    for folder, patterns in BUNDLE:
        for pat in patterns:
            base = WEB.parent if folder.startswith("web/") else ROOT
            for f in sorted((base / folder).glob(pat)):
                if f.is_file() and f.name not in EXCLUDE:
                    files.append(f)
    out.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as z:
        for f in files:
            # fixed timestamp: the zip only changes when a file's contents change
            info = zipfile.ZipInfo(_rel(f), (2026, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            z.writestr(info, f.read_bytes())
    return files


def baseline(zip_path: Path, studies=True):
    """Run both cars from the unzipped bundle, in a fresh interpreter with matplotlib stubbed (as in the browser)."""
    code = r"""
import json, sys
import lapsim_web as w
out = {}
for car in ("ic", "ev"):
    d = {"schema": w.schema(car), "result": w.run(car), "tracks": w.tracks(car), "geometry": w.geometry(car)}
    if car == "ev" and sys.argv[2] == "1":
        d["studies"] = w.ev_studies()
    json.dump(d, open(f"{sys.argv[1]}/{car}.json", "w"), allow_nan=False, separators=(",", ":"))
    print(f"  {car}: " + ", ".join(f"{e} {v['time_s']} s" for e, v in d["result"]["events"].items() if v["time_s"]))
"""
    (SITE / "data").mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        zipfile.ZipFile(zip_path).extractall(tmp)
        env = dict(os.environ, LAPSIM_HOME=tmp, LAPSIM_STUB_MPL="1", PYTHONPATH=str(Path(tmp) / "web" / "bridge"))
        subprocess.run([sys.executable, "-c", code, str(SITE / "data"), "1" if studies else "0"], check=True, env=env,
                       cwd=str(Path(tmp) / "IC"))


def script_copies():
    """Each file the page reads, again as a script that registers it in window.__LAPSIM: these load from a page
    opened straight from disk (file://), where browsers block fetch() and file workers."""
    import base64
    def write(path, key, js_value):
        path.write_text(f"(window.__LAPSIM=window.__LAPSIM||{{}})[{json.dumps(key)}]={js_value};\n")
    for name in ("ic", "ev", "meta"):
        f = SITE / "data" / f"{name}.json"
        if f.exists():
            write(f.with_suffix(".js"), f"data/{name}.json", json.dumps(json.loads(f.read_text()), separators=(",", ":"), allow_nan=False))
    z = SITE / "py" / "lapsim.zip"
    write(SITE / "py" / "lapsim.zip.js", "py/lapsim.zip", json.dumps(base64.b64encode(z.read_bytes()).decode()))
    w = SITE / "assets" / "worker.js"
    write(SITE / "assets" / "worker.js.js", "assets/worker.js", json.dumps(w.read_text()))


def stamp_assets():
    """index.html asks for its scripts and style sheet with ?v=<hash of their contents>, so a browser never runs an
    old cached copy of one file against a new copy of another."""
    import hashlib
    import re
    names = ("app.css", "app.js", "editors.js")
    h = hashlib.sha1(b"".join((SITE / "assets" / n).read_bytes() for n in names)).hexdigest()[:10]
    f = SITE / "index.html"
    f.write_text(re.sub(r'(assets/(?:%s))(\?v=\w+)?"' % "|".join(re.escape(n) for n in names), r'\1?v=%s"' % h, f.read_text()))


def git_commit():
    try:
        return subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=ROOT, capture_output=True, text=True).stdout.strip()
    except OSError:
        return ""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--serve", action="store_true", help="serve web/site at http://localhost:8000 after building")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--no-studies", action="store_true")
    ap.add_argument("--bundle-only", action="store_true", help="only rebuild site/py/lapsim.zip")
    a = ap.parse_args()

    if not (ROOT / "IC" / "fsae_vd").is_dir():
        sys.exit(f"no IC/fsae_vd under {ROOT}: set LAPSIM_ROOT to the fsae_lapsim folder")
    z = SITE / "py" / "lapsim.zip"
    files = bundle(z)
    print(f"bundle: {len(files)} files, {z.stat().st_size / 1e3:.0f} kB -> {z}")
    if not a.bundle_only:
        print("baseline results (run from the bundle):")
        baseline(z, studies=not a.no_studies)
    meta = {"built": dt.datetime.now().strftime("%Y-%m-%d %H:%M"), "commit": git_commit(),
            "bundle_kB": round(z.stat().st_size / 1e3), "files": [_rel(f) for f in files]}
    (SITE / "data" / "meta.json").write_text(json.dumps(meta, indent=1))
    (SITE / ".nojekyll").write_text("")
    stamp_assets()
    script_copies()
    print(f"site: {SITE}")
    if a.serve:
        import functools
        import http.server
        handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=str(SITE))
        print(f"serving http://localhost:{a.port}  (Ctrl-C to stop)")
        http.server.ThreadingHTTPServer(("", a.port), handler).serve_forever()


if __name__ == "__main__":
    main()
