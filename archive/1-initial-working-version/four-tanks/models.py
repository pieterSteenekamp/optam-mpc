"""
Shared modules for the four‑tank ring process digital twin and APC controller.
Includes a robust generic MPC with tunable weights.

(No changes required.)
"""
import numpy as np
from scipy.integrate import odeint
from scipy.optimize import minimize

# ---------- Plant constants ----------
A_tanks = np.ones(4)          # m²
tau_f = 0.5                   # min
dt_ctrl = 1.0                 # min
dt_sim = 0.1                  # min
max_flow_map = {1: 5.0, 2: 2.5, 3: 5.0, 4: 5.5, 5: 5.0}
L_min_tank, L_max_tank = 0.5, 3.5   # control level bounds

# Physical tank limits
PHYS_L_MIN = 0.0
PHYS_L_MAX = 5.0


class FourTankPlant:
    """Digital twin of the four‑tank ring process."""
    def __init__(self):
        self.A = A_tanks
        self.tau_f = tau_f
        self.dt_ctrl = dt_ctrl
        self.dt_sim = dt_sim
        self.reset()

    def reset(self, steady_state=True):
        if steady_state:
            self.X = np.array([2.0, 2.0, 2.0, 2.0, 2.0, 1.8, 1.8, 2.0, 0.2])
        else:
            self.X = np.array([2.0, 2.0, 2.0, 2.0, 2.0, 1.8, 1.8, 2.0, 0.2])
        self.leak = 0.2
        self.max_flows = max_flow_map.copy()
        self.L_min = L_min_tank
        self.L_max = L_max_tank

    @property
    def levels(self):
        return self.X[0:4].copy()

    @property
    def flows(self):
        return self.X[4:9].copy()

    def simulate_interval(self, u_sp, leak=None):
        if leak is None:
            leak = self.leak
        t_eval = np.arange(0, self.dt_ctrl + self.dt_sim/2, self.dt_sim)
        sol = odeint(self._ode, self.X, t_eval, args=(u_sp, leak))
        self.X = sol[-1].copy()
        # Enforce physical level limits
        self.X[0:4] = np.clip(self.X[0:4], PHYS_L_MIN, PHYS_L_MAX)
        self.X[4:9] = np.clip(self.X[4:9], 0, None)
        return self.X.copy()

    def _ode(self, X, t, u_sp, leak):
        L1, L2, L3, L4, F1, F2, F3, F4, Fm = X
        dL1 = (F4 - F1) / self.A[0]
        dL2 = (F1 - F2 - leak) / self.A[1]
        dL3 = (F2 - F3) / self.A[2]
        dL4 = (F3 + Fm - F4) / self.A[3]
        dF1 = (u_sp[0] - F1) / self.tau_f
        dF2 = (u_sp[1] - F2) / self.tau_f
        dF3 = (u_sp[2] - F3) / self.tau_f
        dF4 = (u_sp[3] - F4) / self.tau_f
        dFm = (u_sp[4] - Fm) / self.tau_f
        return [dL1, dL2, dL3, dL4, dF1, dF2, dF3, dF4, dFm]


# ---------- PI controller (tunable) ----------
class PI:
    def __init__(self, Kc, Ti, dt, out_lo, out_hi):
        self.Kc = Kc
        self.Ti = Ti
        self.dt = dt
        self.out_lo = out_lo
        self.out_hi = out_hi
        self.integral = 0.0

    def update(self, Kc=None, Ti=None):
        if Kc is not None:
            self.Kc = Kc
        if Ti is not None:
            self.Ti = Ti

    def compute(self, sp, meas):
        err = sp - meas
        P = self.Kc * err
        self.integral += self.Kc / self.Ti * err * self.dt
        self.integral = np.clip(self.integral, self.out_lo, self.out_hi)
        out = P + self.integral
        return np.clip(out, self.out_lo, self.out_hi)

    def reset(self, value):
        self.integral = value


class PIBaseLayer:
    """Four PI controllers – pure level regulation, tunable."""
    def __init__(self, dt):
        self.pi1 = PI(-0.8, 4.0, dt, 0, 5.0)   # reverse acting
        self.pi2 = PI(-0.8, 4.0, dt, 0, 5.0)
        self.pi3 = PI(-0.8, 4.0, dt, 0, 5.0)
        self.pi4 = PI( 0.8, 4.0, dt, 0, 5.0)   # direct acting (make‑up)
        self.pis = [self.pi1, self.pi2, self.pi3, self.pi4]

    def initialize_steady_state(self, flows):
        """Set integrators so that initial outputs = flows (steady state)."""
        for pi, f in zip(self.pis, flows):
            pi.reset(np.clip(f, pi.out_lo, pi.out_hi))

    def update_pi(self, idx, Kc=None, Ti=None):
        """Update Kc or Ti for pi[idx] (0..3)."""
        if 0 <= idx < 4:
            self.pis[idx].update(Kc=Kc, Ti=Ti)

    def compute_setpoints(self, levels, max_f2, leak, max_f4=5.5):
        L1, L2, L3, L4 = levels
        u1 = np.clip(self.pi1.compute(2.0, L1), 0, 5.0)
        u2 = np.clip(self.pi2.compute(2.0, L2), 0, max_f2)
        u3 = np.clip(self.pi3.compute(2.0, L3), 0, 5.0)
        u_mkp = np.clip(self.pi4.compute(2.0, L4), 0, 5.0)
        u4 = 2.0   # constant F4 in base layer (no economic push)
        return np.array([u1, u2, u3, u4, u_mkp])


# ---------- Standalone simulation function ----------
def simulate_interval(plant, X, u_sp, leak):
    t_eval = np.arange(0, plant.dt_ctrl + plant.dt_sim/2, plant.dt_sim)
    sol = odeint(plant._ode, X, t_eval, args=(u_sp, leak))
    return sol[-1]


# ---------- Robust generic MPC with tunable weights ----------
def generic_mpc_step(plant, L_meas, X_current, u_prev,
                     Np=6, econ_w=1.0, level_w=5.0, move_w=1.0,
                     soft_penalty=1000.0):
    """
    Soft‑constrained MPC with configurable weights.
    econ_w   : weight on F4 (maximise)
    level_w  : weight on (L - 2.0)^2
    move_w   : weight on Δu^2
    """
    Np = min(Np, 10)
    u_min = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    u_max = np.array([5.0, plant.max_flows[2], 5.0, 5.5, 5.0])

    X0 = X_current.copy()
    u_init = np.tile(u_prev, (Np, 1)).flatten()
    bounds = [(u_min[j], u_max[j]) for _ in range(Np) for j in range(5)]

    def cost(u_flat):
        u_seq = u_flat.reshape(Np, 5)
        X = X0.copy()
        J = 0.0
        for k in range(Np):
            t_eval = np.arange(0, plant.dt_ctrl + plant.dt_sim/2, plant.dt_sim)
            sol = odeint(plant._ode, X, t_eval, args=(u_seq[k], plant.leak))
            X = sol[-1]
            L = X[:4]
            over = np.maximum(0.0, L - L_max_tank)
            under = np.maximum(0.0, L_min_tank - L)
            J += soft_penalty * np.sum(over**2) + soft_penalty * np.sum(under**2)
            J += level_w * np.sum((L - 2.0)**2)
            du = u_seq[0] - u_prev if k == 0 else u_seq[k] - u_seq[k-1]
            J += move_w * np.sum(du**2)
            if econ_w > 0 and np.all((L >= 1.5) & (L <= 3.0)):
                J -= econ_w * u_seq[k, 3]
        return J

    res = minimize(cost, u_init, method='SLSQP', bounds=bounds,
                   options={'maxiter': 500, 'ftol': 1e-8, 'disp': False})
    if not res.success:
        print(f"⚠️ MPC fallback: {res.message}")
        u_opt = u_prev.copy()
    else:
        u_opt = res.x.reshape(Np, 5)[0]
    return np.clip(u_opt, u_min, u_max)
