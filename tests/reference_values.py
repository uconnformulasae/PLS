"""Hand-calculated reference numbers, each with its derivation, so anyone can check them."""
import math

LBF_PER_IN = 4.4482216152605 / 0.0254          # N/m per lbf/in

# Tire peak friction  mu = lambda (mu0 + mu_fz (Fz - Fz0)) (1 - k_gamma (gamma - gamma_opt)^2)
# CT-17 IC: mu0 1.65, mu_fz -0.00047 /N, Fz0 1000 N, k_gamma 0.00625 /deg^2, gamma_opt -0.5 deg, lambda 1
#   Fz = 800 N, gamma = -2 deg:  (1.65 + 0.094) (1 - 0.00625 * 2.25) = 1.744 * 0.9859375
TIRE_MU_800N_M2DEG = 1.744 * (1 - 0.00625 * 1.5 ** 2)

# Wheel rate = spring rate x (damper travel / wheel travel)^2; motion ratio 1.18 wheel/damper
#   350 lbf/in x (1/1.18)^2 = 251.4 lbf/in  (the hard-point motion ratio replaces 1.18 in the car)
WHEEL_RATE_350_MR118 = 350 * LBF_PER_IN / 1.18 ** 2

# Brake torque per wheel per pascal = 2 faces x piston area x pad mu x effective radius
#   area 1e-3 m^2, mu 0.45, radius 0.08 m:  2 * 1e-3 * 0.45 * 0.08 = 7.2e-5 N m / Pa
BRAKE_GAIN = 2 * 1e-3 * 0.45 * 0.08

# FSAE scoring at Tmin: the maximum points, A + B
#   acceleration 95.5 + 4.5 = 100, skidpad 71.5 + 3.5 = 75, autocross 118.5 + 6.5 = 125, endurance 250 + 25 = 275
MAX_POINTS = {"acceleration": 100.0, "skidpad": 75.0, "autocross": 125.0, "endurance": 275.0}

# Skidpad: a car with a constant lateral limit a on a circle of radius R laps in 2 pi R / sqrt(a R)
#   R 9.125 m, a 1.5 g:  2 pi 9.125 / sqrt(1.5 * 9.80665 * 9.125) = 4.948 s
SKIDPAD_R, SKIDPAD_A = 9.125, 1.5 * 9.80665
SKIDPAD_TIME = 2 * math.pi * SKIDPAD_R / math.sqrt(SKIDPAD_A * SKIDPAD_R)

# Arrhenius resistance factor: exp(Ea/R (1/T - 1/Tref)); Ea 28 kJ/mol, T 0 degC, Tref 25 degC
ARRHENIUS_0C = math.exp(28e3 / 8.314462618 * (1 / 273.15 - 1 / 298.15))
