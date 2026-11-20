"""Regression: today's results against the frozen reference results (scripts/make_golden.py)."""
import json
from pathlib import Path

import numpy as np
import pytest

from scripts.make_golden import CARS, EVENTS, REF, snapshot

RTOL = 1e-9


@pytest.fixture(scope="module", params=CARS)
def pair(request):
    f = REF / f"{request.param}.json"
    if not f.exists():
        pytest.skip(f"no reference for {request.param}: run python scripts/make_golden.py")
    return json.loads(f.read_text()), snapshot(request.param)


def test_event_times(pair):
    ref, now = pair
    for e in EVENTS:
        assert now["times"][e] == pytest.approx(ref["times"][e], rel=RTOL), e


def test_every_lap_point(pair):
    ref, now = pair
    for e in EVENTS:
        for c in ("s", "t", "v", "ax"):
            np.testing.assert_allclose(now["laps"][e][c], ref["laps"][e][c], rtol=RTOL, atol=1e-9, err_msg=f"{e} {c}")


def test_endurance_laps(pair):
    ref, now = pair
    np.testing.assert_allclose(now["endurance"]["lap_time_s"], ref["endurance"]["lap_time_s"], rtol=RTOL)
