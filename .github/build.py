#!/usr/bin/env python
"""Build the website's simulation bundle and baseline results from a checkout of the main branch.

    python .github/build.py --lapsim ../PLS-main            # bundle + baseline results
    python .github/build.py --lapsim ../PLS-main --serve    # ... then serve the site at http://localhost:8000
    python .github/build.py --serve                         # serve the site as it is (no rebuild)

--lapsim is a checkout of main (the lapsim package, cars/, tracks/). Without it nothing is rebuilt.

What it writes (everything under assets/):
    py/lapsim.zip                 lapsim/, cars/, tracks/ from main + the bridge (py/lapsim_web.py,
                                  py/wing_sections.json) as bridge/: what the browser runs (Pyodide)
    data/ic.json, data/ev.json    baseline inputs + results of each car (shown the moment the page opens)
    data/mb/transient_<car>.json  baseline transient check (Multibody page)
    data/meta.json                build time, main's commit, the bundle's file list
and stamps index.html's scripts and style sheet with ?v=<hash of their contents>.

The baseline results are computed FROM THE BUNDLE (unzipped to a temporary folder), so a build that succeeds
proves the bundle holds every file the simulation needs.

Pre-computed data is not touched: data/mb/ic.json (minimum-lap-time runs), data/cfd/ (3-D CFD replay) and
py/wing_sections.json come from tools in the team's research folder (see docs/website.md).
The team's published designs (data/team/) are written by the site itself.
"""
import argparse
import datetime as dt
import functools
import hashlib
import http.server
import json
import os
import re
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path

SITE = Path(__file__).resolve().parents[1]          # the website branch's root (index.html, assets/)
ASSETS = SITE / "assets"
BRIDGE = ASSETS / "py"

# (folder in the main checkout, glob patterns)
FROM_MAIN = [
    ("lapsim", ["**/*.py"]),
    ("cars", ["**/*.yaml", "**/*.csv"]),
    ("tracks", ["**/*.csv"]),
]
FROM_SITE = ["lapsim_web.py", "wing_sections.json"]   # in assets/py/, zipped as bridge/


def bundle(main: Path, out: Path):
    files = []
    for folder, patterns in FROM_MAIN:
        for pat in patterns:
            for f in sorted((main / folder).glob(pat)):
                if f.is_file() and "__pycache__" not in f.parts and ".ipynb_checkpoints" not in f.parts:
                    files.append((str(f.relative_to(main)).replace(os.sep, "/"), f))
    if not files:
        sys.exit(f"{main} has no lapsim/: pass a checkout of the main branch with --lapsim")
    files += [(f"bridge/{n}", BRIDGE / n) for n in FROM_SITE if (BRIDGE / n).exists()]
    with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as z:
        for name, f in files:
            info = zipfile.ZipInfo(name, (2026, 1, 1, 0, 0, 0))    # fixed timestamp: the zip changes only with its contents
            info.compress_type = zipfile.ZIP_DEFLATED
            z.writestr(info, f.read_bytes())
    return [name for name, _ in files]


BASELINE = r"""
import json, os, sys
import lapsim_web as w
out = sys.argv[1]
for car in ("ic", "ev"):
    d = {"schema": w.schema(car), "result": w.run(car), "tracks": w.tracks(car), "geometry": w.geometry(car)}
    if car == "ev" and sys.argv[2] == "1":
        d["studies"] = w.ev_studies()
    json.dump(d, open(f"{out}/{car}.json", "w"), allow_nan=False, separators=(",", ":"))
    print(f"  {car}: " + ", ".join(f"{e} {v['time_s']} s" for e, v in d["result"]["events"].items() if v["time_s"]))
    tr = w.transient(car)
    json.dump(tr, open(f"{out}/mb/transient_{car}.json", "w"), allow_nan=False, separators=(",", ":"))
    print(f"  {car} transient: " + ", ".join(f"{e} {v['summary']['time_s']} s" for e, v in tr["events"].items() if "summary" in v))
"""


def baseline(zip_path: Path, studies=True):
    """Run both cars from the unzipped bundle, in a fresh interpreter with matplotlib stubbed (as in the browser)."""
    (ASSETS / "data" / "mb").mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        zipfile.ZipFile(zip_path).extractall(tmp)
        env = dict(os.environ, LAPSIM_HOME=tmp, LAPSIM_STUB_MPL="1", PYTHONPATH=str(Path(tmp) / "bridge"))
        subprocess.run([sys.executable, "-c", BASELINE, str(ASSETS / "data"), "1" if studies else "0"],
                       check=True, env=env, cwd=tmp)


def stamp_assets():
    """index.html asks for its scripts and style sheet with ?v=<hash of their contents>, so a browser never runs an
    old cached copy of one file against a new copy of another."""
    names = ["css/app.css"] + [f"js/{n}.js" for n in ("app", "editors", "tunnel", "wings", "workbench", "multibody", "worker")]
    h = hashlib.sha1(b"".join((ASSETS / n).read_bytes() for n in names)).hexdigest()[:10]
    f = SITE / "index.html"
    f.write_text(re.sub(r'(assets/(?:%s))(\?v=\w+)?"' % "|".join(re.escape(n) for n in names), r'\1?v=%s"' % h, f.read_text()))


def git_commit(repo: Path):
    try:
        return subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=repo, capture_output=True, text=True).stdout.strip()
    except OSError:
        return ""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--lapsim", type=Path, help="a checkout of the main branch (lapsim/, cars/, tracks/)")
    ap.add_argument("--no-studies", action="store_true", help="skip the EV power-limit / gearing studies (faster)")
    ap.add_argument("--bundle-only", action="store_true", help="only rebuild assets/py/lapsim.zip")
    ap.add_argument("--serve", action="store_true", help="serve the site at http://localhost:PORT")
    ap.add_argument("--port", type=int, default=8000)
    a = ap.parse_args()

    if a.lapsim:
        main_dir = a.lapsim.resolve()
        z = BRIDGE / "lapsim.zip"
        files = bundle(main_dir, z)
        print(f"bundle: {len(files)} files, {z.stat().st_size / 1e3:.0f} kB -> {z}")
        if not a.bundle_only:
            print("baseline results (run from the bundle):")
            baseline(z, studies=not a.no_studies)
        meta = {"built": dt.datetime.now().strftime("%Y-%m-%d %H:%M"), "commit": git_commit(main_dir),
                "bundle_kB": round(z.stat().st_size / 1e3), "files": files}
        (ASSETS / "data" / "meta.json").write_text(json.dumps(meta, indent=1))
        stamp_assets()
    elif not a.serve:
        ap.error("nothing to do: pass --lapsim <main checkout> to rebuild, or --serve")
    if a.serve:
        handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=str(SITE))
        print(f"serving http://localhost:{a.port}  (Ctrl-C to stop)")
        http.server.ThreadingHTTPServer(("", a.port), handler).serve_forever()


if __name__ == "__main__":
    main()
