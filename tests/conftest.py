import sys
import warnings
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
warnings.filterwarnings("ignore")


@pytest.fixture(scope="session")
def ic():
    import lapsim
    return lapsim.load_car("ct17_ic")


@pytest.fixture(scope="session")
def ev():
    import lapsim
    return lapsim.load_car("ct17_ev")


@pytest.fixture(scope="session")
def ic_vehicle(ic):
    return ic.build()


@pytest.fixture(scope="session")
def ev_vehicle(ev):
    return ev.build()
