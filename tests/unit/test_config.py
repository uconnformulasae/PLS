"""Car files load, overrides apply, mistakes are caught."""
import pytest

import lapsim
from lapsim.config.loader import car_file
from lapsim.config.register import parameters


def test_both_cars_load(ic, ev):
    assert (ic.kind, ev.kind) == ("ic", "ev")
    assert ic.vehicle.mass.mass_no_driver == pytest.approx(211.4)
    assert ev.vehicle.mass.mass_no_driver == pytest.approx(240.0)


def test_ev_inherits_the_ic_chassis(ic, ev):
    assert ev.vehicle.geometry.wheelbase == ic.vehicle.geometry.wheelbase
    assert ev.vehicle.front.hardpoint_file == ic.vehicle.front.hardpoint_file


def test_file_paths_resolve(ic, ev):
    for c in (ic, ev):
        for f in (c.vehicle.front.hardpoint_file, c.vehicle.events.autocross_track, c.vehicle.events.endurance_track):
            assert f and __import__("pathlib").Path(f).is_file()
    assert __import__("pathlib").Path(ic.power.engine.dyno_file).is_file()


def test_units_are_converted(ic):
    assert ic.vehicle.mass.cg_height == pytest.approx(11 * 0.0254)


def test_override_with_units():
    car = lapsim.load_car("ct17_ic", {"vehicle:mass.cg_height": "12 in", "pt:final_drive.rear_sprocket": 40})
    assert car.vehicle.mass.cg_height == pytest.approx(12 * 0.0254)
    assert car.power.final_drive.rear_sprocket == 40
    assert car.overrides["pt:final_drive.rear_sprocket"] == 40


def test_unknown_override_is_an_error():
    with pytest.raises(AttributeError):
        lapsim.load_car("ct17_ic", {"vehicle:mass.no_such_value": 1})
    with pytest.raises(KeyError):
        lapsim.load_car("ct17_ic", {"pack:cell.r0": 0.01})        # the IC car has no pack


def test_unknown_key_in_a_car_file(tmp_path):
    src = car_file("ct17_ic").read_text().replace("  mass:\n", "  mass:\n    typo_kg: 3\n", 1)
    f = tmp_path / "vehicle.yaml"
    f.write_text(src.replace("hardpoints/", str(car_file("ct17_ic").parent / "hardpoints") + "/"))
    with pytest.raises(KeyError, match="typo_kg"):
        lapsim.load_car(f)


def test_register_lists_every_value(ic, ev):
    p = parameters(ev)
    assert {"vehicle", "ev", "pack"} == set(p["section"])
    assert (p["unit"] != "").sum() > 50
    assert len(parameters(ic)) > 200


def test_ev_status_marks_placeholders(ev):
    st = ev.status()
    assert (st["status"] == "PLACEHOLDER").sum() > 0
    assert not st["key"].str.endswith("_track").any()
