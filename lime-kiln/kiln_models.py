"""
Lime kiln digital twin – two‑state thermal model with feed transport delay,
PI temperature controller, and calcination‑target MPC with sintering constraint.
"""
import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import minimize
import math
from collections import deque

R = 8.314               # J/(mol·K)
k0c = 1.5e5             # pre‑exponential calcination (1/s)
Eac = 200e3             # J/mol
k0s = 1e6               # pre‑exponential sintering (1/s)
Eas = 250e3             # J/mol
dH_calc = 178e3         # J/kg CaCO3
Cp_solid = 1000.0       # J/(kg·K)
T_feed = 300.0          # K
T_amb = 300.0           # K

V_eff = 80.0            # m³
rho_bulk = 800.0        # kg/m³
nominal_feed = 8.0      # kg/s
beta_sint = 2.0

max_firing_power = 15e6 # W

C_wall = 1e7            # J/K
C_solid = 1e6           # J/K
UA_ws = 2e5             # W/K  wall → solids
UA_wa = 5e3             # W/K  wall → ambient

dt_ctrl = 1.0           # min
FEED_DELAY_MIN = 30     # min

# ---------- Kinetic functions ----------
def kinetics(t, y, T):
    Xc, Xs = y
    rc = k0c * np.exp(-Eac/(R*T)) * (1.0 - np.clip(Xc,0,1))
    rs = k0s * np.exp(-Eas/(R*T)) * np.clip(Xc,0,1)
    return [rc, rs]

def final_extents(T, tau):
    if tau <= 0:
        return np.array([0.0, 0.0])
    sol = solve_ivp(lambda t, y: kinetics(t, y, T), [0, tau], [0.0, 0.0],
                    method='RK45', rtol=1e-6, atol=1e-8)
    Xc = np.clip(sol.y[0,-1], 0.0, 1.0)
    Xs = np.clip(sol.y[1,-1], 0.0, 1.0)
    return np.array([Xc, Xs])

def quality(Xc, Xs):
    return Xc * math.exp(-beta_sint * Xs)

# ---------- PI controller ----------
class PI:
    def __init__(self, Kc, Ti, dt, out_lo, out_hi):
        self.Kc = Kc
        self.Ti = Ti
        self.dt = dt
        self.out_lo = out_lo
        self.out_hi = out_hi
        self.integral = 0.0

    def compute(self, sp, meas):
        err = sp - meas
        P = self.Kc * err
        self.integral += self.Kc / self.Ti * err * self.dt
        self.integral = np.clip(self.integral, self.out_lo, self.out_hi)
        out = P + self.integral
        return np.clip(out, self.out_lo, self.out_hi)

    def reset(self, value):
        self.integral = value

# ---------- Lime Kiln Plant ----------
class LimeKilnPlant:
    def __init__(self):
        self.T_wall = 1200.0
        self.T_solid = 1200.0
        self.F = nominal_feed
        self.u_fire = 0.5
        self.Xc = 0.0
        self.Xs = 0.0
        self.Q = 0.0

        self.delay_len = int(FEED_DELAY_MIN / dt_ctrl)
        self.feed_buffer = deque([nominal_feed] * self.delay_len, maxlen=self.delay_len)

    def set_feed(self, F):
        self.F = max(0.1, F)
        self.feed_buffer.append(self.F)

    def get_residence_time(self):
        mass = V_eff * rho_bulk
        return mass / self.F

    def step(self, pi_controller, T_sp):
        self.u_fire = pi_controller.compute(T_sp, self.T_solid)

        F_delayed = self.feed_buffer[0]
        tau = self.get_residence_time()
        Xc, Xs = final_extents(self.T_solid, tau)
        self.Xc, self.Xs = Xc, Xs
        self.Q = quality(Xc, Xs)

        Q_firing = self.u_fire * max_firing_power
        Q_calc = F_delayed * dH_calc * Xc
        Q_heat_solids = F_delayed * Cp_solid * (self.T_solid - T_feed)

        T_w = self.T_wall
        T_s = self.T_solid
        seconds = int(dt_ctrl * 60)
        for _ in range(seconds):
            dT_w = (Q_firing - UA_ws * (T_w - T_s) - UA_wa * (T_w - T_amb)) / C_wall
            dT_s = (UA_ws * (T_w - T_s) - Q_heat_solids - Q_calc) / C_solid
            T_w += dT_w
            T_s += dT_s

        self.T_wall = np.clip(T_w, 500.0, 2000.0)
        self.T_solid = np.clip(T_s, 500.0, 2000.0)

        return self.T_solid, Xc, Xs, self.Q, self.u_fire

    @staticmethod
    def compute_steady_state(F=nominal_feed, T_target=1200.0):
        mass = V_eff * rho_bulk
        tau = mass / F
        Xc, Xs = final_extents(T_target, tau)
        Q = quality(Xc, Xs)

        Q_calc = F * dH_calc * Xc
        Q_heat_solids = F * Cp_solid * (T_target - T_feed)
        Q_needed_solids = Q_heat_solids + Q_calc

        T_w = T_target + Q_needed_solids / UA_ws
        Q_firing = Q_needed_solids + UA_wa * (T_w - T_amb)
        u_fire = Q_firing / max_firing_power
        u_fire = np.clip(u_fire, 0.0, 1.0)

        return T_w, T_target, u_fire, Xc, Xs, Q


# ---------- Calcination‑target MPC (with sintering & temp constraints) ----------
def kiln_mpc_step(plant, pi_controller, T_current, u_prev,
                  F_feed, Np=6, move_w=0.5,
                  xc_setpoint=0.92, xc_weight=100.0,
                  xs_max=0.15, temp_max=1450.0,
                  soft_penalty=1e4):
    """
    MPC that drives calcination Xc to xc_setpoint.
    Soft constraints: Xs ≤ xs_max, T_solid ≤ temp_max.
    """
    # Save full original state
    orig = {
        'T_wall': plant.T_wall, 'T_solid': plant.T_solid,
        'F': plant.F, 'Xc': plant.Xc, 'Xs': plant.Xs,
        'Q': plant.Q, 'u_fire': plant.u_fire,
        'feed_buffer': plant.feed_buffer.copy()
    }
    orig_pi_int = pi_controller.integral

    T_min, T_max = 900.0, 1500.0
    bounds = [(T_min, T_max) for _ in range(Np)]
    T_sp_guess = np.full(Np, u_prev)

    pi_copy = PI(pi_controller.Kc, pi_controller.Ti, pi_controller.dt,
                 pi_controller.out_lo, pi_controller.out_hi)
    pi_copy.integral = orig_pi_int

    def cost(T_sp_seq):
        # Reset plant to original state
        plant.T_wall = orig['T_wall']
        plant.T_solid = T_current
        plant.F = F_feed
        plant.feed_buffer = orig['feed_buffer'].copy()
        plant.u_fire = 0.5
        pi_copy.reset(orig_pi_int)

        u_prev_local = u_prev
        J = 0.0
        for k in range(Np):
            # --- Advance feed delay (exactly as the real simulator does) ---
            plant.set_feed(F_feed)

            T_s, Xc, Xs, Q, _ = plant.step(pi_copy, T_sp_seq[k])

            J += xc_weight * (Xc - xc_setpoint) ** 2
            J += move_w * (T_sp_seq[k] - u_prev_local) ** 2

            if Xs > xs_max:
                J += soft_penalty * (Xs - xs_max) ** 2
            if T_s > temp_max:
                J += soft_penalty * (T_s - temp_max) ** 2

            u_prev_local = T_sp_seq[k]
        return J

    res = minimize(cost, T_sp_guess, method='L-BFGS-B', bounds=bounds,
                   options={'maxiter': 200, 'ftol': 1e-8})
    if not res.success:
        print(f"MPC warning: {res.message}")
        T_sp_opt = u_prev
    else:
        T_sp_opt = res.x[0]

    # Restore original plant state
    for key, val in orig.items():
        setattr(plant, key, val)
    pi_controller.integral = orig_pi_int

    return np.clip(T_sp_opt, T_min, T_max)