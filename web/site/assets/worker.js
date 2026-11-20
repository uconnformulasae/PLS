/* Runs the Python lap simulation (the lapsim package) in WebAssembly via Pyodide,
   off the main thread so the page stays responsive. Messages: {zip} once, then {id, fn, args} -> {id, ok, result}. */
const PYODIDE = "https://cdn.jsdelivr.net/pyodide/v0.27.7/full/";
const status = (text) => postMessage({ status: text });
// the page sends the simulation bundle (py/lapsim.zip) as its first message: {zip: ArrayBuffer}
let haveZip; const gotZip = new Promise((r) => { haveZip = r; });

const ready = (async () => {
  status("Downloading the Python runtime…");
  importScripts(PYODIDE + "pyodide.js");
  const py = await loadPyodide({ indexURL: PYODIDE });
  status("Loading NumPy, SciPy and pandas…");
  await py.loadPackage(["numpy", "scipy", "pandas", "pyyaml"]);
  status("Loading the lap simulation…");
  const zip = await gotZip;
  py.FS.mkdirTree("/lapsim");
  py.unpackArchive(zip, "zip", { extractDir: "/lapsim" });
  py.runPython(`
import os, sys, json
os.environ["LAPSIM_HOME"] = "/lapsim"
sys.path.insert(0, "/lapsim/web/bridge")
os.chdir("/lapsim")
import lapsim_web
`);
  status("ready");
  return py;
})();
ready.catch((e) => postMessage({ fatal: String(e) }));

onmessage = async (e) => {
  if (e.data.zip) return haveZip(e.data.zip);
  const { id, fn, args } = e.data;
  try {
    const py = await ready;
    py.globals.set("_fn", fn);
    py.globals.set("_args", JSON.stringify(args || []));
    const out = py.runPython("json.dumps(getattr(lapsim_web, _fn)(*json.loads(_args)))");
    postMessage({ id, ok: true, result: out });
  } catch (err) {
    postMessage({ id, ok: false, error: String(err && err.message ? err.message : err) });
  }
};
