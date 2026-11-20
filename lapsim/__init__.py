"""lapsim - predictive lap simulation of the UConn FSAE cars (CT-17 IC and CT-17 EV).

    import lapsim
    car = lapsim.load_car("ct17_ev")                 # cars/ct17_ev/vehicle.yaml
    car.set("vehicle:mass.cg_height", "11 in")        # change any value for this run
    res = car.run()                                   # acceleration, skidpad, autocross, endurance
    res.table()                                       # times and points
    res.laps["autocross"].frame()                     # every channel along the lap

Layout: config/ (schema, loader, register), core/ (vehicle, track, events, results, integrator),
solvers/ (qss, qss_ic, qss_ev, transient), subsystems/ (aero, tires, suspension, hardpoints, brakes,
driveline, frame, power/), studies/ (sensitivity, optimisation, trackside), validation/ (compare, logs).
"""
from lapsim.config.loader import Car, load_car

__version__ = "1.0.0"
__all__ = ["Car", "load_car", "__version__"]
