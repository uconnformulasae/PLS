# Changelog

Changes that move a calibrated result are recorded here (`scripts/make_golden.py --reason` appends to it).

## 2026-11-19: one package, `lapsim`

The simulation (`fsae_vd`, `fsae_pt`, `fsae_ev`, the accumulator model of `fsae_ev_bms`, `fsae_frame`) moved into
`lapsim/`, one YAML file per car (`cars/`) and `tracks/`; see docs/decision_records.md 1-4. No result changed:
every lap channel of both cars matches the previous packages (IC identical, EV within 5e-15 relative), and the
golden references were created from this version.

## 2026-11-19: EV transient check uses the motor limit

The transient check of the EV ran without a drive limit (decision record 5). It now uses the EV's motor force.
EV transient times: acceleration 3.756 -> 3.81 s, skidpad 5.046 s (unchanged), autocross 43.898 -> 43.934 s.
The QSS results and the golden references are unchanged.
