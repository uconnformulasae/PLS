"""Studies that run the car many times: sensitivity, setup optimisation, trackside adjustment."""
from __future__ import annotations


def runner_for(car, progress=None):
    """``runner(cfg, events)`` for the studies: the car's own QSS solver on a changed vehicle section.
    ``progress(n)`` is called after each simulation (optional)."""
    count = {"n": 0}

    def runner(cfg, events):
        c = car.copy()
        c.vehicle = cfg
        count["n"] += 1
        if progress:
            progress(count["n"])
        return c.run(events)
    runner.count = count
    return runner
