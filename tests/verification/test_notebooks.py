"""Every notebook runs top to bottom, with errors inside widget output areas surfaced."""
from pathlib import Path

import pytest

NOTEBOOKS = sorted((Path(__file__).resolve().parents[2] / "notebooks").glob("*.ipynb"))
SHIM = """
import ipywidgets as _W
class _StrictOutput(_W.Output):
    def __exit__(self, etype, evalue, tb):
        super().__exit__(None, None, None)
        return False
_W.Output = _StrictOutput
"""


@pytest.mark.slow
@pytest.mark.parametrize("path", NOTEBOOKS, ids=[p.stem for p in NOTEBOOKS])
def test_notebook_runs(path):
    nbformat = pytest.importorskip("nbformat")
    nbclient = pytest.importorskip("nbclient")
    nb = nbformat.read(path, 4)
    nb.cells.insert(0, nbformat.v4.new_code_cell(SHIM))
    nbclient.NotebookClient(nb, timeout=900, kernel_name="python3",
                            resources={"metadata": {"path": str(path.parent)}}).execute()
