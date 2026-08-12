#!/usr/bin/env python3
"""
MIMO MPC test harness with support for FOPDT, Integrating, and Non-linear processes.
"""

from __future__ import annotations

import math
import warnings
from abc import ABC, abstractmethod
from dataclasses import dataclass
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import minimize


# =============================================================================
# 1. PROCESS MODELS
# =============================================================================

class Process(ABC):
    """Base class for all MIMO and SISO process models."""
    def __init__(self, ny: int, nu: int, dt: float):
        self.ny = ny
        self.nu = nu
        self.dt = dt
        self.y = np.zeros(ny)
        
    @abstractmethod
    def reset(self, y0: np.ndarray, u0: np.ndarray) -> None:
        pass

    @abstractmethod
    def step(self, u: np.ndarray) -> np.ndarray:
        pass

    @abstractmethod
    def clone(self) -> 'Process':
        pass


class MIMO_FOPDT(Process):
    """MIMO First-Order-Plus-Dead-Time process."""
    def __init__(self, K: np.ndarray, tau: np.ndarray, theta: np.ndarray, dt: float):
        self.K = np.atleast_2d(K)
        self.tau = np.atleast_2d(tau)
        self.theta = np.atleast_2d(theta)
        ny, nu = self.K.shape
        super().__init__(ny, nu, dt)

        self._a = np.exp(-self.dt / np.maximum(self.tau, 1e-6))
        self._delay_steps = np.floor(self.theta / self.dt).astype(int)
        self._history_length = max(1, np.max(self._delay_steps) + 1)
        
        self._u_history = np.zeros((self._history_length, self.nu))
        self.y_internal = np.zeros((self.ny, self.nu))

    def reset(self, y0: np.ndarray, u0: np.ndarray) -> None:
        self.y = np.array(y0, dtype=float)
        self.y_internal = np.zeros((self.ny, self.nu))
        for i in range(self.ny):
            for j in range(self.nu):
                if self.K[i, j] != 0:
                    self.y_internal[i, j] = self.y[i] / np.count_nonzero(self.K[i, :])
        
        self._u_history = np.tile(u0, (self._history_length, 1))

    def clone(self) -> 'MIMO_FOPDT':
        other = MIMO_FOPDT(self.K, self.tau, self.theta, self.dt)
        other.y = self.y.copy()
        other.y_internal = self.y_internal.copy()
        other._u_history = self._u_history.copy()
        return other

    def step(self, u: np.ndarray) -> np.ndarray:
        self._u_history = np.vstack((u, self._u_history[:-1, :]))
        for i in range(self.ny):
            for j in range(self.nu):
                delay = self._delay_steps[i, j]
                u_delayed = self._u_history[delay, j]
                
                a = self._a[i, j]
                k = self.K[i, j]
                self.y_internal[i, j] = a * self.y_internal[i, j] + (1.0 - a) * k * u_delayed
        
        self.y = np.sum(self.y_internal, axis=1)
        return self.y


class IntegratingTank(Process):
    """Single input, single output integrating tank model."""
    def __init__(self, K: float, theta: float, dt: float):
        super().__init__(ny=1, nu=1, dt=dt)
        self.K = K
        self.delay_steps = int(math.floor(theta / dt))
        self.history_length = max(1, self.delay_steps + 1)
        self._u_history = np.zeros(self.history_length)

    def reset(self, y0: np.ndarray, u0: np.ndarray) -> None:
        self.y = np.array(y0, dtype=float)
        self._u_history = np.full(self.history_length, u0[0])

    def clone(self) -> 'IntegratingTank':
        other = IntegratingTank(self.K, self.delay_steps * self.dt, self.dt)
        other.y = self.y.copy()
        other._u_history = self._u_history.copy()
        return other

    def step(self, u: np.ndarray) -> np.ndarray:
        self._u_history = np.insert(self._u_history, 0, u[0])[:-1]
        u_delayed = self._u_history[self.delay_steps]
        # y[k+1] = y[k] + K * dt * u[k-d]
        self.y[0] += self.K * self.dt * u_delayed
        return self.y


class NonlinearCSTR(Process):
    """
    Exothermic CSTR (Non-linear process).
    y (States) = [Concentration (mol/L), Temperature (K)]
    u (Inputs) = [Flowrate (L/min), Coolant Temp (K)]
    """
    def __init__(self, dt: float):
        super().__init__(ny=2, nu=2, dt=dt)
        # Process parameters
        self.V = 100.0        # Volume (L)
        self.Cin = 1.0        # Inlet concentration (mol/L)
        self.Tin = 350.0      # Inlet temperature (K)
        self.k0 = 7.2e10      # Pre-exponential factor (1/min)
        self.E_R = 8750.0     # Activation energy / R (K)
        self.dH = -5e4        # Heat of reaction (J/mol)
        self.rhoCp = 500.0    # Density * Heat Capacity (J/L/K)
        self.UA = 5e4         # Heat transfer coefficient (J/min/K)

    def reset(self, y0: np.ndarray, u0: np.ndarray) -> None:
        self.y = np.array(y0, dtype=float)

    def clone(self) -> 'NonlinearCSTR':
        other = NonlinearCSTR(self.dt)
        other.y = self.y.copy()
        return other
        
    def _derivatives(self, state, u):
        C, T = state
        F, Tc = u
        rate = self.k0 * np.exp(-self.E_R / T) * C
        dCdt = (F / self.V) * (self.Cin - C) - rate
        dTdt = (F / self.V) * (self.Tin - T) + (-self.dH / self.rhoCp) * rate - (self.UA / (self.V * self.rhoCp)) * (T - Tc)
        return np.array([dCdt, dTdt])

    def step(self, u: np.ndarray) -> np.ndarray:
        # 4th Order Runge-Kutta (RK4) integration over dt
        k1 = self._derivatives(self.y, u)
        k2 = self._derivatives(self.y + 0.5 * self.dt * k1, u)
        k3 = self._derivatives(self.y + 0.5 * self.dt * k2, u)
        k4 = self._derivatives(self.y + self.dt * k3, u)
        
        self.y += (self.dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
        return self.y


# =============================================================================
# 2. CONFIGURATION
# =============================================================================

@dataclass
class Scenario:
    name: str
    n_steps: int
    dt: float
    y0: np.ndarray
    u0: np.ndarray
    setpoints: np.ndarray
    
    plant: Process
    model: Process
    
    Np: int = 15
    Nc: int = 5
    
    # Tuning matrices (diagonal elements)
    W_y: np.ndarray = None
    W_du: np.ndarray = None
    
    # Constraints
    u_min: np.ndarray = None
    u_max: np.ndarray = None
    du_min: np.ndarray = None
    du_max: np.ndarray = None
    
    disturbance_filter: float = 0.25

    def __post_init__(self):
        ny, nu = self.plant.ny, self.plant.nu
        if self.W_y is None: self.W_y = np.ones(ny)
        if self.W_du is None: self.W_du = np.full(nu, 0.1)
        if self.u_min is None: self.u_min = np.full(nu, -np.inf)
        if self.u_max is None: self.u_max = np.full(nu, np.inf)
        if self.du_min is None: self.du_min = np.full(nu, -np.inf)
        if self.du_max is None: self.du_max = np.full(nu, np.inf)


# =============================================================================
# 3. MIMO MPC CONTROLLER
# =============================================================================

class MIMOMPC:
    """MIMO Non-linear programming MPC with offset-free output-bias tracking."""
    def __init__(self, scenario: Scenario) -> None:
        self.cfg = scenario
        self._internal_model = scenario.model.clone()
        self._d_hat = np.zeros(self.cfg.plant.ny)
        self._prev_u = np.array(scenario.u0, dtype=float)
        self._initialised = False

    def reset(self, y0: np.ndarray, u0: np.ndarray) -> None:
        self._internal_model.reset(y0, u0)
        self._prev_u = np.clip(u0, self.cfg.u_min, self.cfg.u_max)
        self._d_hat = np.zeros(self.cfg.plant.ny)
        self._initialised = True

    def step(self, y_measured: np.ndarray, setpoint: np.ndarray) -> np.ndarray:
        if not self._initialised:
            self.reset(y0=y_measured, u0=self.cfg.u0)

        cfg = self.cfg
        ny, nu = cfg.plant.ny, cfg.plant.nu
        Np, Nc = cfg.Np, cfg.Nc

        # Additive disturbance estimation (Output bias)
        model_output_now = self._internal_model.y
        raw_bias = y_measured - model_output_now
        alpha = cfg.disturbance_filter
        self._d_hat = alpha * raw_bias + (1.0 - alpha) * self._d_hat
        d_hat = self._d_hat
        u_previous = self._prev_u

        def expand_moves(x: np.ndarray) -> np.ndarray:
            """Reshapes 1D decision variables to (Np, nu). Holds final move."""
            moves_Nc = x.reshape((Nc, nu))
            if Nc == Np:
                return moves_Nc
            tail = np.tile(moves_Nc[-1, :], (Np - Nc, 1))
            return np.vstack((moves_Nc, tail))

        def predict_from_moves(x: np.ndarray) -> np.ndarray:
            planned_input = expand_moves(x)
            predictor = self._internal_model.clone()
            predicted_output = np.empty((Np, ny))
            for j in range(Np):
                predicted_output[j, :] = predictor.step(planned_input[j]) + d_hat
            return predicted_output, planned_input

        def objective(x: np.ndarray) -> float:
            y_pred, u_pred = predict_from_moves(x)
            
            # Tracking Cost
            error = y_pred - setpoint
            cost_y = np.sum(error**2 * cfg.W_y)
            
            # Move suppression cost
            moves = x.reshape((Nc, nu))
            u_seq = np.vstack((u_previous, moves))
            delta_u = np.diff(u_seq, axis=0)
            cost_du = np.sum(delta_u**2 * cfg.W_du)
            
            return float(cost_y + cost_du)

        # Build initial guess (hold previous)
        x0 = np.tile(u_previous, Nc)
        
        # Absolute Input Bounds
        bounds = [(cfg.u_min[j], cfg.u_max[j]) for _ in range(Nc) for j in range(nu)]

        # Move rate limits constraint
        def rate_constraints(x: np.ndarray) -> np.ndarray:
            moves = x.reshape((Nc, nu))
            u_seq = np.vstack((u_previous, moves))
            delta_u = np.diff(u_seq, axis=0)
            
            res = []
            for j in range(nu):
                if np.isfinite(cfg.du_min[j]):
                    res.append(delta_u[:, j] - cfg.du_min[j])
                if np.isfinite(cfg.du_max[j]):
                    res.append(cfg.du_max[j] - delta_u[:, j])
            return np.concatenate(res) if res else np.array([1.0])

        constraint_spec = {"type": "ineq", "fun": rate_constraints}

        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            result = minimize(
                objective, x0, method="SLSQP", bounds=bounds, constraints=[constraint_spec]
            )

        optimal_moves = result.x.reshape((Nc, nu))
        u_optimal = optimal_moves[0, :]
        
        # Advance internal model
        self._internal_model.step(u_optimal)
        self._prev_u = u_optimal

        return u_optimal


# =============================================================================
# 4. CLOSED-LOOP SIMULATION & PLOTTING
# =============================================================================

def run_simulation(scenario: Scenario):
    print(f"\nRunning Scenario: {scenario.name}...")
    
    n = scenario.n_steps
    y_true = np.zeros((n + 1, scenario.plant.ny))
    u_applied = np.zeros((n, scenario.plant.nu))
    time = np.arange(n + 1) * scenario.dt
    
    scenario.plant.reset(scenario.y0, scenario.u0)
    
    controller = MIMOMPC(scenario)
    controller.reset(scenario.y0, scenario.u0)
    
    y_true[0, :] = scenario.y0
    
    for k in range(n):
        y_meas = y_true[k, :]
        sp = scenario.setpoints[k, :]
        
        u = controller.step(y_meas, sp)
        u_applied[k, :] = u
        y_true[k + 1, :] = scenario.plant.step(u)

    # Plotting
    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    fig.suptitle(scenario.name)
    
    # Process Outputs
    ax = axes[0]
    for i in range(scenario.plant.ny):
        ax.plot(time, y_true[:, i], label=f"y{i+1}")
        ax.step(time[:-1], scenario.setpoints[:, i], '--', label=f"SP y{i+1}")
    ax.set_ylabel("Output")
    ax.legend()
    ax.grid(True)
    
    # Manipulated Variables
    ax = axes[1]
    for i in range(scenario.plant.nu):
        ax.step(time[:-1], u_applied[:, i], where='post', label=f"u{i+1}")
    ax.set_ylabel("Input")
    ax.set_xlabel("Time")
    ax.legend()
    ax.grid(True)
    plt.tight_layout()
    plt.show()


# =============================================================================
# 5. DEMONSTRATIONS
# =============================================================================

def demo_mimo_fopdt():
    dt = 1.0
    # 2x2 Transfer Matrix: Cross-coupled MIMO dynamics
    K = np.array([[1.5, 0.5], 
                  [-0.2, 1.0]])
    tau = np.array([[5.0, 3.0], 
                    [2.0, 4.0]])
    theta = np.array([[2.0, 1.0], 
                      [0.0, 1.5]])
    
    plant = MIMO_FOPDT(K, tau, theta, dt)
    model = MIMO_FOPDT(K, tau, theta, dt)  # Perfect model
    
    setpoints = np.zeros((60, 2))
    setpoints[10:, 0] = 1.0  # Step y1 at t=10
    setpoints[30:, 1] = 0.5  # Step y2 at t=30
    
    scenario = Scenario(
        name="2x2 MIMO Cross-Coupled Linear Process",
        n_steps=60, dt=dt,
        y0=np.zeros(2), u0=np.zeros(2), setpoints=setpoints,
        plant=plant, model=model,
        Np=15, Nc=5,
        du_min=np.array([-0.5, -0.5]),
        du_max=np.array([0.5, 0.5])
    )
    run_simulation(scenario)


def demo_integrating_tank():
    dt = 0.5
    plant = IntegratingTank(K=0.2, theta=1.0, dt=dt)
    model = IntegratingTank(K=0.2, theta=1.0, dt=dt)
    
    setpoints = np.zeros((50, 1))
    setpoints[5:, 0] = 10.0 # Change level to 10
    
    scenario = Scenario(
        name="Integrating Process (Tank Level Control)",
        n_steps=50, dt=dt,
        y0=np.array([2.0]), u0=np.array([0.0]), setpoints=setpoints,
        plant=plant, model=model,
        Np=20, Nc=5,
        W_y=np.array([1.0]), W_du=np.array([2.0]),
        u_min=np.array([-5.0]), u_max=np.array([5.0])
    )
    run_simulation(scenario)


def demo_nonlinear_cstr():
    dt = 0.5
    plant = NonlinearCSTR(dt=dt)
    model = NonlinearCSTR(dt=dt)
    
    # Equilibrium starting point approximations
    y0 = np.array([0.8, 320.0]) # [Ca, T]
    u0 = np.array([10.0, 300.0]) # [F, Tc]
    
    setpoints = np.zeros((80, 2))
    setpoints[:, 0] = 0.8
    setpoints[:, 1] = 320.0
    
    # Increase reactor temperature setpoint
    setpoints[15:, 1] = 330.0 
    # Notice Ca drops as temperature increases due to higher reaction rate
    
    scenario = Scenario(
        name="Non-linear Exothermic CSTR",
        n_steps=80, dt=dt,
        y0=y0, u0=u0, setpoints=setpoints,
        plant=plant, model=model,
        Np=10, Nc=3,
        W_y=np.array([0.0, 1.0]), # Only control Temperature, let Concentration float
        W_du=np.array([0.1, 0.05]),
        u_min=np.array([5.0, 270.0]),
        u_max=np.array([20.0, 350.0])
    )
    run_simulation(scenario)

if __name__ == "__main__":
    demo_mimo_fopdt()
    demo_integrating_tank()
    demo_nonlinear_cstr()