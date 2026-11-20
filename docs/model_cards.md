# Model cards

One card per model: what it computes, what it assumes, where it is valid, and how far it has been checked.
"Checked" means against hand calculations or exact solutions (`tests/`); agreement with track data is
validation, which is still to come (README, Next steps).

## Tires (`subsystems/tires.py`)
- **Computes** peak friction per wheel `mu(Fz, gamma) = lambda (mu0 + mu_fz (Fz - Fz0)) (1 - k_gamma (gamma - gamma_opt)^2)`,
  lateral force vs slip angle (simplified Magic Formula, C = 1.4, slope = cornering stiffness), friction ellipse.
- **Assumes** no pressure or temperature effect; longitudinal peak = `mux_over_muy` x lateral peak.
- **Inputs** `vehicle:tires.*`. mu0, mu_fz, k_gamma, gamma_opt are TTC fits marked CHECK; cornering stiffness
  is a typical FSAE value (the TTC fit was too soft).
- **Checked** peak friction and the lateral curve's peak against hand calculation.

## Aerodynamics (`subsystems/aero.py`)
- **Computes** downforce, drag, side force and moments of front wing, rear wing and body at the car's
  attitude: ride height (front-wing ground effect), pitch, roll, body slip and yaw rate.
- **Assumes** component shares from the binder's CFD, scaled to the coast-down ClA / CdA at 45 mph;
  pitch and yaw sensitivities are ASSUMED. DRS off.
- **Valid** for the CT-17 aero package near its calibration point.
- **Checked** whole-car ClA and CdA equal the calibration values at the calibration speed (by construction);
  drag grows with speed squared.

## Suspension and kinematics (`subsystems/suspension.py`, `subsystems/hardpoints.py`)
- **Computes** wheel, ride and roll rates (springs and ARB in series with the tire table); camber, toe,
  roll-centre height, anti-dive / squat, motion ratio and ARB blade rate over wheel travel from the
  OptimumK hard points (3-D linkage solved per travel step).
- **Assumes** rigid links and bushings; the rear export is titled CT-16 but is the CT-17 rear (team).
- **Checked** wheel rate against hand calculation; static wheel loads sum to weight plus downforce.

## Vehicle (`core/vehicle.py`)
- **Computes** for any (v, ax, ay): aero at the solved attitude, longitudinal and lateral load transfer
  (geometric, elastic with frame torsion, unsprung), roll, heave, pitch, camber of each wheel, per-wheel
  grip, the differential's split and yaw moment; and from these the g-g-v envelope.
- **Assumes** quasi-static: no damper forces, no transient attitude.

## Brakes (`subsystems/brakes.py`)
- **Computes** torque per wheel from line pressure, the front share against the ideal share (which axle
  locks first), rotor temperature (one lumped mass per rotor, speed-dependent convection, radiation).
- **Checked** torque gain against hand calculation.

## IC engine and driveline (`subsystems/power/ic.py`, `subsystems/driveline.py`)
- **Computes** restricted R6 torque from restrictor flow, volumetric efficiency, indicated efficiency,
  friction and pumping; part load; cooling (two thermal nodes, radiator effectiveness-NTU, fan); alternator
  drag; gearbox, chain and differential losses; rotating inertia per gear; Drexler LSD locking.
- **Assumes** VE shape and several efficiencies ASSUMED (tagged in the car file); dyno curve fitted.
- **Checked** the unit conversion `omega`.

## EV motor and inverter (`subsystems/power/ev.py`)
- **Computes** torque limited by motor peak torque, inverter current, accumulator power, field weakening
  (corner speed scales with pack voltage) and the cells' current limit; copper, iron and inverter losses;
  regen within motor, inverter and cell charge limits.
- **Assumes** every value is a PLACEHOLDER (generic axial-flux motor) until confirmed in the car file.

## Accumulator (`subsystems/power/battery.py`)
- **Computes** pack voltage from the chemistry's OCV curve, ohmic resistance and two RC branches with an
  Arrhenius temperature factor; state of charge; one lumped thermal node per cell; temperature derating of
  the current limits; rest during the driver change.
- **Assumes** all cells identical (no spread); generic NMC 21700 parameters, ASSUMED.
- **Checked** Arrhenius factor against hand calculation; OCV monotonic; charge drawn over a lap equals the
  state-of-charge drop (to 1e-9).

## Frame (`subsystems/frame.py`)
- **Computes** a 3-D beam finite-element model of the spaceframe: mass, CG and torsional stiffness from a
  twist test on the suspension pick-ups. Its mass and CG move the car's; its stiffness enters the elastic
  lateral load transfer split.
- **Assumes** rigid welded joints, pick-ups tied to the nearest tube by a stiff tab.

## Quasi-steady lap solver (`solvers/qss.py`, `qss_ic.py`, `qss_ev.py`)
- **Computes** the speed profile on each track (lateral limit, braking pass, forward pass with the power
  module), what limits every point, and all powertrain / accumulator channels; endurance lap by lap with
  temperatures and state of charge carried over.
- **Assumes** a steady state at every point; the driver is ideal; flat tracks.
- **Checked** skidpad left equals right; flying laps periodic; speed inside the lateral limit; skidpad
  time equals 2 pi R / v at the steady lateral limit; the car accelerates except while shifting;
  regression against frozen results (`tests/golden/`).

## Transient check (`solvers/transient.py`, `core/integrator.py`)
- **Computes** the car in the time domain: planar motion, heave, pitch, roll, tire lag (relaxation length),
  aero at the instantaneous attitude, a preview driver following the quasi-steady speed profile.
- **Assumes** inertias ESTIMATED from mass and geometry (supply them from CAD). The drive limit is the IC
  car's best-gear WOT force, or the EV's motor force at the start-of-event accumulator state (not updated
  for state of charge or temperature during the lap).
- **Checked** RK4 reaches fourth order on an exact solution; on the skidpad it agrees with the
  quasi-steady time within 5 %.
