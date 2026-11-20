"""The command line, writing a run and comparing two runs."""
import json

from lapsim.cli import main
from lapsim.core.results import write_run
from lapsim.validation.compare import max_relative_difference


def test_cli_cars(capsys):
    main(["cars"])
    assert capsys.readouterr().out.split() == ["ct17_ev", "ct17_ic"]


def test_cli_check_config(capsys):
    main(["check-config", "ct17_ev"])
    assert "OK" in capsys.readouterr().out


def test_cli_run_with_set(capsys):
    main(["run", "--car", "ct17_ic", "--event", "acceleration", "--set", "vehicle:mass.driver_mass=80 kg"])
    assert "acceleration" in capsys.readouterr().out


def test_write_and_compare(ic, tmp_path):
    res = ic.run(("acceleration",))
    a = write_run(res, ic, tmp_path / "a")
    b = write_run(res, ic, tmp_path / "b")
    meta = json.loads((a / "metadata.json").read_text())
    assert meta["car"] == "CT17IC" and len(meta["car_file_sha1"]) == 40
    assert max_relative_difference(a, b) == 0.0
