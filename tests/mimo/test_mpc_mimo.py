#!/usr/bin/env python3
"""Self-contained MIMO model-predictive-control test harness.

The controller in this file is nonlinear-model capable: it predicts by
simulating an arbitrary ``Process`` object and uses SciPy's SLSQP nonlinear
optimizer. The FOPDT and tank demonstrations are linear; the CSTR
demonstration is genuinely nonlinear and controls two outputs with two inputs.

Implemented features
--------------------
* SISO and MIMO dynamic models behind one small interface.
* Exact zero-order-hold FOPDT discretisation, including fractional dead time.
* RK4 integration with substeps for the nonlinear CSTR.
* Input, input-move, and predicted-output constraints.
* Optional explicit slack variables for soft output constraints.
* Output and input scaling, terminal tracking cost, and shifted warm starts.
* Filtered additive output-bias correction for moderate plant/model mismatch.
* Solver diagnostics and deterministic hold-last-input fallback on failure.
* Reproducible closed-loop simulations, metrics, plots, and self-tests.

Scope and limitations
---------------------
This is an educational and development harness, not production APC software.
It has no general state observer, robust/stochastic MPC formulation, formal
closed-loop stability guarantee, plant communications, interlocks, or safety
system. Additive output-bias correction is useful but does not mathematically
guarantee offset-free control for every nonlinear process. Real plant outputs
can violate model-predicted constraints when there is noise, disturbance, or
model mismatch.

Run all standard demonstrations::

    python test_mpc_mimo.py

Run without plot windows or execute the deterministic checks::

    python test_mpc_mimo.py --no-plots
    python test_mpc_mimo.py --self-test
"""

from __future__ import annotations

import argparse
import math
import time
import warnings
from abc import ABC, abstractmethod
from dataclasses import dataclass, replace

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import minimize


Array = np.ndarray


# =============================================================================
# 1. SMALL VALIDATION HELPERS
# =============================================================================


def _vector(value, length: int, name: str, default: float | None = None) -> Array:
    """Return a copied one-dimensional float array with a required length."""
    if value is None:
        if default is None:
            raise ValueError(f"{name} must be provided.")
        return np.full(length, default, dtype=float)

    array = np.asarray(value, dtype=float)
    if array.ndim == 0:
        array = np.full(length, float(array), dtype=float)
    if array.shape != (length,):
        raise ValueError(f"{name} must have shape ({length},), got {array.shape}.")
    if np.any(np.isnan(array)):
        raise ValueError(f"{name} must not contain NaN values.")
    return array.copy()


def _require_finite(array: Array, name: str) -> None:
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain only finite values.")


def _maximum_bound_violation(values: Array, lower: Array, upper: Array) -> float:
    """Largest positive violation of element-wise lower/upper bounds."""
    lower_violation = np.where(np.isfinite(lower), lower - values, 0.0)
    upper_violation = np.where(np.isfinite(upper), values - upper, 0.0)
    return float(max(0.0, np.max(lower_violation), np.max(upper_violation)))


# =============================================================================
# 2. PROCESS MODELS
# =============================================================================


class Process(ABC):
    """Minimal interface required by the simulation-based MPC predictor."""

    def __init__(self, ny: int, nu: int, dt: float) -> None:
        if ny < 1 or nu < 1:
            raise ValueError("A process must have at least one output and one input.")
        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError("dt must be a positive finite number.")
        self.ny = int(ny)
        self.nu = int(nu)
        self.dt = float(dt)
        self.y = np.zeros(self.ny, dtype=float)

    def _checked_output(self, y0) -> Array:
        output = _vector(y0, self.ny, "y0")
        _require_finite(output, "y0")
        return output

    def _checked_input(self, u) -> Array:
        input_value = _vector(u, self.nu, "u")
        _require_finite(input_value, "u")
        return input_value

    @abstractmethod
    def reset(self, y0: Array, u0: Array) -> None:
        """Reset all dynamic state to a condition consistent with y0 and u0."""

    @abstractmethod
    def step(self, u: Array) -> Array:
        """Advance the process by one sampling interval and return a copy of y."""

    @abstractmethod
    def clone(self) -> "Process":
        """Return an independent deep copy, including all dynamic state."""


class MIMOFOPDT(Process):
    """Matrix of independent first-order-plus-dead-time response paths.

    Each path obeys ``tau[i,j] * dx/dt + x = K[i,j] * u[j]``. The output is
    the sum of the path states plus a constant operating-point offset.
    Fractional dead time is integrated exactly for zero-order-held inputs.
    """

    def __init__(self, K: Array, tau: Array, theta: Array, dt: float) -> None:
        gain = np.atleast_2d(np.asarray(K, dtype=float)).copy()
        time_constant = np.atleast_2d(np.asarray(tau, dtype=float)).copy()
        dead_time = np.atleast_2d(np.asarray(theta, dtype=float)).copy()

        if gain.shape != time_constant.shape or gain.shape != dead_time.shape:
            raise ValueError("K, tau, and theta must have identical (ny, nu) shapes.")
        if gain.size == 0:
            raise ValueError("K, tau, and theta must not be empty.")
        _require_finite(gain, "K")
        _require_finite(time_constant, "tau")
        _require_finite(dead_time, "theta")
        if np.any(time_constant <= 0.0):
            raise ValueError("Every time constant in tau must be positive.")
        if np.any(dead_time < 0.0):
            raise ValueError("Dead times in theta must be non-negative.")

        ny, nu = gain.shape
        super().__init__(ny=ny, nu=nu, dt=dt)
        self.K = gain
        self.tau = time_constant
        self.theta = dead_time

        delay_samples = self.theta / self.dt
        self._whole_delay = np.floor(delay_samples + 1e-12).astype(int)
        self._fractional_delay = self.theta - self._whole_delay * self.dt
        self._fractional_delay[np.abs(self._fractional_delay) < 1e-12] = 0.0
        self._history_length = int(np.max(self._whole_delay)) + 2

        self._u_history = np.zeros((self._history_length, self.nu), dtype=float)
        self._path_state = np.zeros((self.ny, self.nu), dtype=float)
        self._output_offset = np.zeros(self.ny, dtype=float)

    def reset(self, y0: Array, u0: Array) -> None:
        output = self._checked_output(y0)
        input_value = self._checked_input(u0)

        # Initialise every path at steady state for u0, then use a constant
        # operating-point offset to make the total output exactly equal y0.
        self._path_state = self.K * input_value[np.newaxis, :]
        self._output_offset = output - np.sum(self._path_state, axis=1)
        self._u_history = np.tile(input_value, (self._history_length, 1))
        self.y = output.copy()

    def clone(self) -> "MIMOFOPDT":
        other = MIMOFOPDT(self.K, self.tau, self.theta, self.dt)
        other.y = self.y.copy()
        other._path_state = self._path_state.copy()
        other._output_offset = self._output_offset.copy()
        other._u_history = self._u_history.copy()
        return other

    @staticmethod
    def _first_order_update(x: float, target: float, duration: float, tau: float) -> float:
        if duration <= 0.0:
            return x
        decay = math.exp(-duration / tau)
        return decay * x + (1.0 - decay) * target

    def step(self, u: Array) -> Array:
        input_value = self._checked_input(u)
        self._u_history[1:, :] = self._u_history[:-1, :].copy()
        self._u_history[0, :] = input_value

        for i in range(self.ny):
            for j in range(self.nu):
                whole = int(self._whole_delay[i, j])
                fraction = float(self._fractional_delay[i, j])
                x = float(self._path_state[i, j])
                gain = float(self.K[i, j])
                tau = float(self.tau[i, j])

                # During the fractional first part of the interval, the older
                # held input is still active. The newer delayed sample acts
                # during the remainder. This is exact under zero-order hold.
                if fraction > 0.0:
                    older_input = self._u_history[whole + 1, j]
                    x = self._first_order_update(x, gain * older_input, fraction, tau)

                newer_input = self._u_history[whole, j]
                x = self._first_order_update(
                    x,
                    gain * newer_input,
                    self.dt - fraction,
                    tau,
                )
                self._path_state[i, j] = x

        self.y = self._output_offset + np.sum(self._path_state, axis=1)
        return self.y.copy()


class IntegratingTank(Process):
    """SISO integrator ``dy/dt = K * u(t - theta)`` with fractional delay."""

    def __init__(self, K: float, theta: float, dt: float) -> None:
        if not np.isfinite(K):
            raise ValueError("K must be finite.")
        if not np.isfinite(theta) or theta < 0.0:
            raise ValueError("theta must be a non-negative finite number.")
        super().__init__(ny=1, nu=1, dt=dt)
        self.K = float(K)
        self.theta = float(theta)
        self._whole_delay = int(math.floor(theta / dt + 1e-12))
        self._fractional_delay = theta - self._whole_delay * dt
        if abs(self._fractional_delay) < 1e-12:
            self._fractional_delay = 0.0
        self._history_length = self._whole_delay + 2
        self._u_history = np.zeros(self._history_length, dtype=float)

    def reset(self, y0: Array, u0: Array) -> None:
        self.y = self._checked_output(y0)
        input_value = self._checked_input(u0)
        self._u_history.fill(input_value[0])

    def clone(self) -> "IntegratingTank":
        other = IntegratingTank(self.K, self.theta, self.dt)
        other.y = self.y.copy()
        other._u_history = self._u_history.copy()
        return other

    def step(self, u: Array) -> Array:
        input_value = self._checked_input(u)
        self._u_history[1:] = self._u_history[:-1].copy()
        self._u_history[0] = input_value[0]

        fraction = self._fractional_delay
        older_input = self._u_history[self._whole_delay + 1]
        newer_input = self._u_history[self._whole_delay]
        delayed_input_integral = (
            fraction * older_input + (self.dt - fraction) * newer_input
        )
        self.y[0] += self.K * delayed_input_integral
        return self.y.copy()


@dataclass(frozen=True)
class CSTRParameters:
    """Physical parameters for the exothermic continuous stirred-tank reactor."""

    volume: float = 100.0              # L
    inlet_concentration: float = 1.0   # mol/L
    inlet_temperature: float = 350.0   # K
    pre_exponential_factor: float = 7.2e10  # 1/min
    activation_temperature: float = 8750.0  # E/R, K
    heat_of_reaction: float = -5.0e4   # J/mol
    density_heat_capacity: float = 500.0  # J/L/K
    heat_transfer: float = 5.0e4       # J/min/K


class NonlinearCSTR(Process):
    """Nonlinear exothermic CSTR with both states treated as measured outputs.

    Outputs/states: concentration [mol/L], reactor temperature [K].
    Inputs: feed flow [L/min], coolant temperature [K].
    """

    def __init__(
        self,
        dt: float,
        parameters: CSTRParameters | None = None,
        integration_substeps: int = 4,
    ) -> None:
        super().__init__(ny=2, nu=2, dt=dt)
        if integration_substeps < 1:
            raise ValueError("integration_substeps must be at least one.")
        self.parameters = parameters or CSTRParameters()
        self.integration_substeps = int(integration_substeps)
        parameter_values = np.asarray(list(vars(self.parameters).values()), dtype=float)
        _require_finite(parameter_values, "CSTR parameters")
        positive_indices = [0, 1, 2, 3, 4, 6, 7]
        if np.any(parameter_values[positive_indices] <= 0.0):
            raise ValueError("CSTR volume, kinetic and heat-transfer parameters must be positive.")

    def reset(self, y0: Array, u0: Array) -> None:
        self._checked_input(u0)
        self.y = self._checked_output(y0)
        if self.y[1] <= 1.0:
            raise ValueError("CSTR temperature must be above absolute zero.")

    def clone(self) -> "NonlinearCSTR":
        other = NonlinearCSTR(self.dt, self.parameters, self.integration_substeps)
        other.y = self.y.copy()
        return other

    def _rate_constant(self, temperature: float) -> float:
        if not np.isfinite(temperature) or temperature <= 1.0:
            raise FloatingPointError("The predicted CSTR temperature is invalid.")
        p = self.parameters
        return p.pre_exponential_factor * math.exp(
            -p.activation_temperature / temperature
        )

    def _derivatives(self, state: Array, u: Array) -> Array:
        concentration, temperature = state
        flow, coolant_temperature = u
        p = self.parameters

        rate = self._rate_constant(float(temperature)) * concentration
        dilution = flow / p.volume
        heat_release = -p.heat_of_reaction / p.density_heat_capacity
        cooling = p.heat_transfer / (p.volume * p.density_heat_capacity)

        concentration_rate = (
            dilution * (p.inlet_concentration - concentration) - rate
        )
        temperature_rate = (
            dilution * (p.inlet_temperature - temperature)
            + heat_release * rate
            - cooling * (temperature - coolant_temperature)
        )
        derivative = np.array([concentration_rate, temperature_rate], dtype=float)
        if not np.all(np.isfinite(derivative)):
            raise FloatingPointError("The CSTR derivative became non-finite.")
        return derivative

    def step(self, u: Array) -> Array:
        input_value = self._checked_input(u)
        state = self.y.copy()
        h = self.dt / self.integration_substeps

        for _ in range(self.integration_substeps):
            k1 = self._derivatives(state, input_value)
            k2 = self._derivatives(state + 0.5 * h * k1, input_value)
            k3 = self._derivatives(state + 0.5 * h * k2, input_value)
            k4 = self._derivatives(state + h * k3, input_value)
            state = state + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
            if not np.all(np.isfinite(state)) or state[1] <= 1.0:
                raise FloatingPointError("The integrated CSTR state became invalid.")

        self.y = state
        return self.y.copy()

    def equilibrium_for_temperature_and_flow(
        self,
        temperature: float,
        flow: float,
    ) -> tuple[Array, Array]:
        """Return the equilibrium state and coolant input for chosen T and F."""
        if flow <= 0.0:
            raise ValueError("Equilibrium flow must be positive.")
        p = self.parameters
        kinetic_constant = self._rate_constant(temperature)
        dilution = flow / p.volume
        concentration = (
            dilution * p.inlet_concentration / (dilution + kinetic_constant)
        )
        rate = kinetic_constant * concentration
        heat_release = -p.heat_of_reaction / p.density_heat_capacity
        cooling = p.heat_transfer / (p.volume * p.density_heat_capacity)
        coolant_temperature = temperature - (
            dilution * (p.inlet_temperature - temperature) + heat_release * rate
        ) / cooling
        return (
            np.array([concentration, temperature], dtype=float),
            np.array([flow, coolant_temperature], dtype=float),
        )


# Backwards-compatible spelling used by the original program.
MIMO_FOPDT = MIMOFOPDT


# =============================================================================
# 3. CONTROLLER CONFIGURATION AND RESULTS
# =============================================================================


@dataclass
class MPCConfig:
    """Configuration for the simulation-based MIMO MPC controller."""

    prediction_horizon: int = 15
    control_horizon: int = 5

    output_weights: Array | None = None
    move_weights: Array | None = None
    terminal_weights: Array | None = None
    output_scale: Array | None = None
    input_scale: Array | None = None

    input_min: Array | None = None
    input_max: Array | None = None
    move_min: Array | None = None
    move_max: Array | None = None
    output_min: Array | None = None
    output_max: Array | None = None

    # None means hard predicted-output constraints. Providing positive
    # weights adds one non-negative slack per constrained output. Each slack
    # softens that output's bounds across the full prediction horizon.
    soft_output_weights: Array | None = None
    maximum_output_slack: Array | float = np.inf

    bias_filter: float = 0.25
    solver_max_iterations: int = 200
    solver_tolerance: float = 1e-8
    constraint_tolerance: float = 1e-6
    raise_on_failure: bool = False


@dataclass
class MPCStepResult:
    """Control action plus the diagnostics needed to judge its reliability."""

    u: Array
    success: bool
    fallback_used: bool
    status: int
    message: str
    objective: float
    iterations: int
    solve_time_seconds: float
    minimum_constraint_margin: float
    maximum_slack: float
    planned_inputs: Array
    predicted_outputs: Array
    bias_estimate: Array


class MIMOMPC:
    """MIMO MPC using direct nonlinear optimisation of future input values."""

    def __init__(self, model: Process, config: MPCConfig) -> None:
        self._internal_model = model.clone()
        self.config = config
        self.ny = model.ny
        self.nu = model.nu
        self._resolve_and_validate_config()

        self._previous_u = np.zeros(self.nu, dtype=float)
        self._bias = np.zeros(self.ny, dtype=float)
        self._last_plan: Array | None = None
        self._initialised = False

    def _resolve_and_validate_config(self) -> None:
        cfg = self.config
        if cfg.prediction_horizon < 1:
            raise ValueError("prediction_horizon must be at least one.")
        if not 1 <= cfg.control_horizon <= cfg.prediction_horizon:
            raise ValueError(
                "control_horizon must be between one and prediction_horizon."
            )
        if not 0.0 <= cfg.bias_filter <= 1.0:
            raise ValueError("bias_filter must lie between zero and one.")
        if cfg.solver_max_iterations < 1:
            raise ValueError("solver_max_iterations must be positive.")
        if cfg.solver_tolerance <= 0.0 or cfg.constraint_tolerance <= 0.0:
            raise ValueError("Solver and constraint tolerances must be positive.")
        self.Np = int(cfg.prediction_horizon)
        self.Nc = int(cfg.control_horizon)
        self.output_weights = _vector(cfg.output_weights, self.ny, "output_weights", 1.0)
        self.move_weights = _vector(cfg.move_weights, self.nu, "move_weights", 0.1)
        self.terminal_weights = _vector(
            cfg.terminal_weights, self.ny, "terminal_weights", 0.0
        )
        self.output_scale = _vector(cfg.output_scale, self.ny, "output_scale", 1.0)
        self.input_scale = _vector(cfg.input_scale, self.nu, "input_scale", 1.0)

        for values, name in (
            (self.output_weights, "output_weights"),
            (self.move_weights, "move_weights"),
            (self.terminal_weights, "terminal_weights"),
        ):
            _require_finite(values, name)
            if np.any(values < 0.0):
                raise ValueError(f"{name} must be non-negative.")
        for values, name in (
            (self.output_scale, "output_scale"),
            (self.input_scale, "input_scale"),
        ):
            _require_finite(values, name)
            if np.any(values <= 0.0):
                raise ValueError(f"{name} must be positive.")
        if not np.any(self.output_weights + self.terminal_weights > 0.0):
            raise ValueError("At least one output or terminal weight must be positive.")

        self.input_min = _vector(cfg.input_min, self.nu, "input_min", -np.inf)
        self.input_max = _vector(cfg.input_max, self.nu, "input_max", np.inf)
        self.move_min = _vector(cfg.move_min, self.nu, "move_min", -np.inf)
        self.move_max = _vector(cfg.move_max, self.nu, "move_max", np.inf)
        self.output_min = _vector(cfg.output_min, self.ny, "output_min", -np.inf)
        self.output_max = _vector(cfg.output_max, self.ny, "output_max", np.inf)
        self.maximum_output_slack = _vector(
            cfg.maximum_output_slack,
            self.ny,
            "maximum_output_slack",
        )
        if np.any(self.maximum_output_slack <= 0.0):
            raise ValueError("Every maximum_output_slack value must be positive.")

        for lower, upper, name in (
            (self.input_min, self.input_max, "input"),
            (self.move_min, self.move_max, "move"),
            (self.output_min, self.output_max, "output"),
        ):
            if np.any(lower > upper):
                raise ValueError(f"Every {name} lower bound must be <= its upper bound.")
        if np.any(self.move_min > 0.0) or np.any(self.move_max < 0.0):
            raise ValueError(
                "Move limits must permit a zero move so hold-last-input is feasible."
            )

        constrained = np.isfinite(self.output_min) | np.isfinite(self.output_max)
        self._constrained_outputs = np.flatnonzero(constrained)
        if cfg.soft_output_weights is None:
            self.soft_output_weights = None
        else:
            weights = _vector(
                cfg.soft_output_weights,
                self.ny,
                "soft_output_weights",
            )
            _require_finite(weights, "soft_output_weights")
            if np.any(weights[self._constrained_outputs] <= 0.0):
                raise ValueError(
                    "Soft-output weights must be positive for every constrained output."
                )
            self.soft_output_weights = weights

        self._soft_constraints = (
            self.soft_output_weights is not None
            and self._constrained_outputs.size > 0
        )
        self._n_slack = (
            self._constrained_outputs.size if self._soft_constraints else 0
        )

        output_inequalities_per_step = 0
        for output_index in self._constrained_outputs:
            output_inequalities_per_step += int(np.isfinite(self.output_min[output_index]))
            output_inequalities_per_step += int(np.isfinite(self.output_max[output_index]))
        self._n_output_inequalities = self.Np * output_inequalities_per_step

    def reset(self, y0: Array, u0: Array) -> None:
        output = _vector(y0, self.ny, "y0")
        input_value = _vector(u0, self.nu, "u0")
        _require_finite(output, "y0")
        _require_finite(input_value, "u0")
        violation = _maximum_bound_violation(
            input_value[np.newaxis, :], self.input_min, self.input_max
        )
        if violation > self.config.constraint_tolerance:
            raise ValueError("u0 violates the configured absolute input bounds.")

        self._internal_model.reset(output, input_value)
        self._previous_u = np.clip(input_value, self.input_min, self.input_max)
        self._bias = np.zeros(self.ny, dtype=float)
        self._last_plan = None
        self._initialised = True

    def _reference_horizon(self, reference) -> Array:
        values = np.asarray(reference, dtype=float)
        if values.shape == (self.ny,):
            horizon = np.tile(values, (self.Np, 1))
        elif values.ndim == 2 and values.shape[1] == self.ny and values.shape[0] >= 1:
            if values.shape[0] >= self.Np:
                horizon = values[: self.Np, :].copy()
            else:
                tail = np.tile(values[-1, :], (self.Np - values.shape[0], 1))
                horizon = np.vstack((values, tail))
        else:
            raise ValueError(
                f"reference must have shape ({self.ny},) or (steps, {self.ny})."
            )
        _require_finite(horizon, "reference")
        return horizon

    def _project_plan(self, proposed_plan: Array) -> Array:
        """Project an absolute-input plan sequentially onto input/move limits."""
        plan = np.asarray(proposed_plan, dtype=float)
        if plan.shape != (self.Nc, self.nu):
            raise ValueError("The proposed input plan has an invalid shape.")
        projected = np.empty_like(plan)
        previous = self._previous_u.copy()
        for step in range(self.Nc):
            lower = np.maximum(self.input_min, previous + self.move_min)
            upper = np.minimum(self.input_max, previous + self.move_max)
            if np.any(lower > upper + self.config.constraint_tolerance):
                raise RuntimeError("Input and move bounds are mutually infeasible.")
            projected[step, :] = np.clip(plan[step, :], lower, upper)
            previous = projected[step, :]
        return projected

    def _initial_input_guess(self) -> Array:
        if self._last_plan is None:
            guess = np.tile(self._previous_u, (self.Nc, 1))
        elif self.Nc == 1:
            guess = self._last_plan.copy()
        else:
            guess = np.vstack((self._last_plan[1:, :], self._last_plan[-1, :]))
        return self._project_plan(guess)

    def _expand_plan(self, plan: Array) -> Array:
        if self.Nc == self.Np:
            return plan.copy()
        tail = np.tile(plan[-1, :], (self.Np - self.Nc, 1))
        return np.vstack((plan, tail))

    def _predict(self, plan: Array) -> tuple[Array, Array]:
        expanded_plan = self._expand_plan(plan)
        predictor = self._internal_model.clone()
        predicted_output = np.empty((self.Np, self.ny), dtype=float)
        for step in range(self.Np):
            predicted_output[step, :] = (
                predictor.step(expanded_plan[step, :]) + self._bias
            )
        if not np.all(np.isfinite(predicted_output)):
            raise FloatingPointError("The model prediction became non-finite.")
        return predicted_output, expanded_plan

    def _required_slack(self, predicted_output: Array) -> Array:
        if not self._soft_constraints:
            return np.empty(0, dtype=float)
        slack = np.zeros(self._constrained_outputs.size, dtype=float)
        for position, output_index in enumerate(self._constrained_outputs):
            if np.isfinite(self.output_min[output_index]):
                slack[position] = max(
                    slack[position],
                    float(np.max(
                        self.output_min[output_index]
                        - predicted_output[:, output_index]
                    )),
                )
            if np.isfinite(self.output_max[output_index]):
                slack[position] = max(
                    slack[position],
                    float(np.max(
                        predicted_output[:, output_index]
                        - self.output_max[output_index]
                    )),
                )
        return np.maximum(slack, 0.0)

    def _split_decision(self, decision: Array) -> tuple[Array, Array]:
        input_size = self.Nc * self.nu
        plan = decision[:input_size].reshape(self.Nc, self.nu)
        if self._soft_constraints:
            slack = decision[input_size:]
        else:
            slack = np.empty(0, dtype=float)
        return plan, slack

    def control(self, y_measured: Array, reference: Array) -> MPCStepResult:
        """Calculate and return one receding-horizon control action."""
        measurement = _vector(y_measured, self.ny, "y_measured")
        _require_finite(measurement, "y_measured")
        if not self._initialised:
            raise RuntimeError("Call reset(y0, u0) before requesting a control move.")
        reference_horizon = self._reference_horizon(reference)

        raw_bias = measurement - self._internal_model.y
        alpha = self.config.bias_filter
        self._bias = alpha * raw_bias + (1.0 - alpha) * self._bias

        input_guess = self._initial_input_guess()
        try:
            initial_prediction, _ = self._predict(input_guess)
            slack_guess = self._required_slack(initial_prediction)
        except (FloatingPointError, ValueError, OverflowError):
            slack_guess = np.zeros(self._constrained_outputs.size, dtype=float)
        if self._soft_constraints:
            slack_maximum = self.maximum_output_slack[self._constrained_outputs]
            slack_guess = np.minimum(
                slack_guess + self.config.constraint_tolerance,
                slack_maximum,
            )
            initial_decision = np.concatenate((input_guess.ravel(), slack_guess.ravel()))
        else:
            initial_decision = input_guess.ravel()

        bounds = [
            (self.input_min[input_index], self.input_max[input_index])
            for _ in range(self.Nc)
            for input_index in range(self.nu)
        ]
        if self._soft_constraints:
            bounds.extend(
                (0.0, self.maximum_output_slack[output_index])
                for output_index in self._constrained_outputs
            )

        cache_decision: Array | None = None
        cache_prediction: Array | None = None
        cache_expanded_plan: Array | None = None

        def evaluate(decision: Array) -> tuple[Array, Array, Array]:
            nonlocal cache_decision, cache_prediction, cache_expanded_plan
            if cache_decision is not None and np.array_equal(decision, cache_decision):
                _, slack = self._split_decision(decision)
                return cache_prediction, cache_expanded_plan, slack
            plan, slack = self._split_decision(decision)
            prediction, expanded = self._predict(plan)
            cache_decision = decision.copy()
            cache_prediction = prediction
            cache_expanded_plan = expanded
            return prediction, expanded, slack

        def objective(decision: Array) -> float:
            try:
                predicted, _, slack = evaluate(decision)
                scaled_error = (
                    predicted - reference_horizon
                ) / self.output_scale[np.newaxis, :]
                tracking_cost = np.sum(
                    scaled_error**2 * self.output_weights[np.newaxis, :]
                )
                terminal_cost = np.sum(
                    scaled_error[-1, :] ** 2 * self.terminal_weights
                )

                plan, _ = self._split_decision(decision)
                input_sequence = np.vstack((self._previous_u, plan))
                scaled_moves = np.diff(input_sequence, axis=0) / self.input_scale
                move_cost = np.sum(
                    scaled_moves**2 * self.move_weights[np.newaxis, :]
                )

                slack_cost = 0.0
                if self._soft_constraints:
                    constrained_scale = self.output_scale[self._constrained_outputs]
                    constrained_weight = self.soft_output_weights[
                        self._constrained_outputs
                    ]
                    scaled_slack = slack / constrained_scale
                    slack_cost = np.sum(scaled_slack**2 * constrained_weight)

                total = tracking_cost + terminal_cost + move_cost + slack_cost
                return float(total) if np.isfinite(total) else 1.0e30
            except (FloatingPointError, ValueError, OverflowError):
                return 1.0e30

        constraints: list[dict] = []
        rate_inequality_count = self.Nc * (
            int(np.sum(np.isfinite(self.move_min)))
            + int(np.sum(np.isfinite(self.move_max)))
        )

        if rate_inequality_count:

            def rate_constraints(decision: Array) -> Array:
                plan, _ = self._split_decision(decision)
                moves = np.diff(np.vstack((self._previous_u, plan)), axis=0)
                residuals = []
                for input_index in range(self.nu):
                    if np.isfinite(self.move_min[input_index]):
                        residuals.append(
                            moves[:, input_index] - self.move_min[input_index]
                        )
                    if np.isfinite(self.move_max[input_index]):
                        residuals.append(
                            self.move_max[input_index] - moves[:, input_index]
                        )
                return np.concatenate(residuals)

            constraints.append({"type": "ineq", "fun": rate_constraints})

        if self._n_output_inequalities:

            def output_constraints(decision: Array) -> Array:
                try:
                    predicted, _, slack = evaluate(decision)
                except (FloatingPointError, ValueError, OverflowError):
                    return np.full(self._n_output_inequalities, -1.0e12)
                residuals = []
                for position, output_index in enumerate(self._constrained_outputs):
                    allowance = slack[position] if self._soft_constraints else 0.0
                    if np.isfinite(self.output_min[output_index]):
                        residuals.append(
                            predicted[:, output_index]
                            - self.output_min[output_index]
                            + allowance
                        )
                    if np.isfinite(self.output_max[output_index]):
                        residuals.append(
                            self.output_max[output_index]
                            - predicted[:, output_index]
                            + allowance
                        )
                return np.concatenate(residuals)

            constraints.append({"type": "ineq", "fun": output_constraints})

        caught_warnings: list[str] = []
        optimizer_result = None
        optimizer_exception = ""
        start = time.perf_counter()
        try:
            with warnings.catch_warnings(record=True) as warning_records:
                warnings.simplefilter("always")
                optimizer_result = minimize(
                    objective,
                    initial_decision,
                    method="SLSQP",
                    bounds=bounds,
                    constraints=constraints,
                    options={
                        "maxiter": self.config.solver_max_iterations,
                        "ftol": self.config.solver_tolerance,
                        "disp": False,
                    },
                )
                caught_warnings = [str(record.message) for record in warning_records]
        except Exception as exc:  # The deterministic fallback still protects u.
            optimizer_exception = f"{type(exc).__name__}: {exc}"
        solve_time = time.perf_counter() - start

        candidate_ok = optimizer_result is not None and bool(optimizer_result.success)
        minimum_margin = np.inf
        candidate_decision = None

        if candidate_ok:
            candidate_decision = np.asarray(optimizer_result.x, dtype=float)
            expected_size = self.Nc * self.nu + self._n_slack
            candidate_ok = (
                candidate_decision.shape == (expected_size,)
                and np.all(np.isfinite(candidate_decision))
            )

        if candidate_ok:
            try:
                margins = []
                for constraint in constraints:
                    margins.extend(np.asarray(constraint["fun"](candidate_decision)).ravel())
                minimum_margin = float(min(margins)) if margins else np.inf
                candidate_ok = minimum_margin >= -self.config.constraint_tolerance
            except (FloatingPointError, ValueError, OverflowError):
                candidate_ok = False

        if candidate_ok:
            plan, slack = self._split_decision(candidate_decision)
            plan = self._project_plan(plan)
            projected_decision = np.concatenate((plan.ravel(), slack))
            try:
                projected_margins = []
                for constraint in constraints:
                    projected_margins.extend(
                        np.asarray(constraint["fun"](projected_decision)).ravel()
                    )
                projected_margin = (
                    float(min(projected_margins)) if projected_margins else np.inf
                )
                minimum_margin = min(minimum_margin, projected_margin)
                candidate_ok = (
                    projected_margin >= -self.config.constraint_tolerance
                )
            except (FloatingPointError, ValueError, OverflowError):
                candidate_ok = False

        if candidate_ok:
            predicted_output, expanded_plan = self._predict(plan)
            u_command = plan[0, :].copy()
            self._last_plan = plan.copy()
            maximum_slack = float(np.max(slack)) if slack.size else 0.0
        else:
            # Holding the last feasible input satisfies all configured absolute
            # input and move constraints. It is deterministic and auditable.
            plan = np.tile(self._previous_u, (self.Nc, 1))
            expanded_plan = self._expand_plan(plan)
            u_command = self._previous_u.copy()
            self._last_plan = plan.copy()
            try:
                predicted_output, expanded_plan = self._predict(plan)
                fallback_slack = self._required_slack(predicted_output)
                maximum_slack = (
                    float(np.max(fallback_slack)) if fallback_slack.size else 0.0
                )
            except (FloatingPointError, ValueError, OverflowError):
                predicted_output = np.full((self.Np, self.ny), np.nan)
                maximum_slack = np.nan

        if optimizer_result is None:
            status = -1
            iterations = 0
            objective_value = np.nan
            base_message = optimizer_exception or "Optimizer did not return a result."
        else:
            status = int(getattr(optimizer_result, "status", -1))
            iterations = int(getattr(optimizer_result, "nit", 0))
            objective_value = float(getattr(optimizer_result, "fun", np.nan))
            base_message = str(getattr(optimizer_result, "message", ""))
        if caught_warnings:
            base_message = f"{base_message} Warnings: {' | '.join(caught_warnings)}".strip()
        if not candidate_ok:
            base_message = f"{base_message} Hold-last-input fallback used.".strip()

        self._internal_model.step(u_command)
        self._previous_u = u_command.copy()

        step_result = MPCStepResult(
            u=u_command,
            success=bool(candidate_ok),
            fallback_used=not bool(candidate_ok),
            status=status,
            message=base_message,
            objective=objective_value,
            iterations=iterations,
            solve_time_seconds=solve_time,
            minimum_constraint_margin=minimum_margin,
            maximum_slack=maximum_slack,
            planned_inputs=expanded_plan.copy(),
            predicted_outputs=predicted_output.copy(),
            bias_estimate=self._bias.copy(),
        )
        if step_result.fallback_used and self.config.raise_on_failure:
            raise RuntimeError(step_result.message)
        return step_result

    # Familiar name retained for callers of the original controller. The new
    # return value contains both u and diagnostics instead of silently hiding
    # optimizer failures.
    step = control


# =============================================================================
# 4. CLOSED-LOOP SCENARIOS, METRICS, AND PLOTS
# =============================================================================


@dataclass
class Scenario:
    name: str
    n_steps: int
    plant: Process
    model: Process
    y0: Array
    u0: Array
    setpoints: Array
    controller_config: MPCConfig
    output_labels: tuple[str, ...] | None = None
    input_labels: tuple[str, ...] | None = None
    plant_input_disturbance: Array | None = None
    measurement_noise_std: Array | None = None
    random_seed: int = 7
    use_setpoint_preview: bool = False

    def __post_init__(self) -> None:
        if self.n_steps < 1:
            raise ValueError("n_steps must be at least one.")
        if self.plant.ny != self.model.ny or self.plant.nu != self.model.nu:
            raise ValueError("Plant and model input/output dimensions must match.")
        if not math.isclose(self.plant.dt, self.model.dt, rel_tol=0.0, abs_tol=1e-12):
            raise ValueError("Plant and model sampling intervals must match.")

        self.y0 = _vector(self.y0, self.plant.ny, "scenario y0")
        self.u0 = _vector(self.u0, self.plant.nu, "scenario u0")
        _require_finite(self.y0, "scenario y0")
        _require_finite(self.u0, "scenario u0")

        self.setpoints = np.asarray(self.setpoints, dtype=float).copy()
        expected_shape = (self.n_steps, self.plant.ny)
        if self.setpoints.shape != expected_shape:
            raise ValueError(
                f"setpoints must have shape {expected_shape}, got {self.setpoints.shape}."
            )
        _require_finite(self.setpoints, "setpoints")

        if self.output_labels is None:
            self.output_labels = tuple(f"y{i + 1}" for i in range(self.plant.ny))
        if self.input_labels is None:
            self.input_labels = tuple(f"u{i + 1}" for i in range(self.plant.nu))
        if len(self.output_labels) != self.plant.ny:
            raise ValueError("output_labels length must equal the number of outputs.")
        if len(self.input_labels) != self.plant.nu:
            raise ValueError("input_labels length must equal the number of inputs.")

        if self.plant_input_disturbance is None:
            self.plant_input_disturbance = np.zeros(
                (self.n_steps, self.plant.nu), dtype=float
            )
        else:
            self.plant_input_disturbance = np.asarray(
                self.plant_input_disturbance, dtype=float
            ).copy()
            if self.plant_input_disturbance.shape != (self.n_steps, self.plant.nu):
                raise ValueError(
                    "plant_input_disturbance must have shape (n_steps, nu)."
                )
            _require_finite(self.plant_input_disturbance, "plant_input_disturbance")

        self.measurement_noise_std = _vector(
            self.measurement_noise_std,
            self.plant.ny,
            "measurement_noise_std",
            0.0,
        )
        _require_finite(self.measurement_noise_std, "measurement_noise_std")
        if np.any(self.measurement_noise_std < 0.0):
            raise ValueError("measurement_noise_std must be non-negative.")


@dataclass
class PerformanceMetrics:
    final_error: Array
    rmse: Array
    integral_absolute_error: Array
    maximum_input_violation: float
    maximum_move_violation: float
    maximum_true_output_violation: float
    solver_failures: int
    fallback_count: int
    mean_solve_time_seconds: float
    maximum_solve_time_seconds: float
    maximum_slack: float


@dataclass
class SimulationResult:
    scenario: Scenario
    time: Array
    outputs: Array
    measurements: Array
    commanded_inputs: Array
    plant_inputs: Array
    setpoints: Array
    controller_steps: list[MPCStepResult]
    metrics: PerformanceMetrics


def _calculate_metrics(
    scenario: Scenario,
    controller: MIMOMPC,
    outputs: Array,
    commanded_inputs: Array,
    steps: list[MPCStepResult],
) -> PerformanceMetrics:
    error = outputs[1:, :] - scenario.setpoints
    moves = np.diff(np.vstack((scenario.u0, commanded_inputs)), axis=0)

    maximum_input_violation = _maximum_bound_violation(
        commanded_inputs, controller.input_min, controller.input_max
    )
    maximum_move_violation = _maximum_bound_violation(
        moves, controller.move_min, controller.move_max
    )
    maximum_output_violation = _maximum_bound_violation(
        outputs[1:, :], controller.output_min, controller.output_max
    )
    solve_times = np.array([step.solve_time_seconds for step in steps], dtype=float)
    slacks = np.array([step.maximum_slack for step in steps], dtype=float)

    return PerformanceMetrics(
        final_error=outputs[-1, :] - scenario.setpoints[-1, :],
        rmse=np.sqrt(np.mean(error**2, axis=0)),
        integral_absolute_error=np.sum(np.abs(error), axis=0) * scenario.plant.dt,
        maximum_input_violation=maximum_input_violation,
        maximum_move_violation=maximum_move_violation,
        maximum_true_output_violation=maximum_output_violation,
        solver_failures=sum(not step.success for step in steps),
        fallback_count=sum(step.fallback_used for step in steps),
        mean_solve_time_seconds=float(np.mean(solve_times)),
        maximum_solve_time_seconds=float(np.max(solve_times)),
        maximum_slack=float(np.nanmax(slacks)) if slacks.size else 0.0,
    )


def print_summary(result: SimulationResult) -> None:
    scenario = result.scenario
    metrics = result.metrics
    print(f"\n{scenario.name}")
    print("-" * len(scenario.name))
    for index, label in enumerate(scenario.output_labels):
        final_value = result.outputs[-1, index]
        target = result.setpoints[-1, index]
        print(
            f"{label}: final={final_value:.6g}, target={target:.6g}, "
            f"final error={metrics.final_error[index]:+.3g}, "
            f"RMSE={metrics.rmse[index]:.3g}, "
            f"IAE={metrics.integral_absolute_error[index]:.3g}"
        )
    print(
        f"Solver: {len(result.controller_steps)} calls, "
        f"{metrics.solver_failures} failures, "
        f"{metrics.fallback_count} fallbacks, "
        f"mean={1000.0 * metrics.mean_solve_time_seconds:.2f} ms, "
        f"max={1000.0 * metrics.maximum_solve_time_seconds:.2f} ms"
    )
    print(
        "Maximum violations: "
        f"input={metrics.maximum_input_violation:.3g}, "
        f"move={metrics.maximum_move_violation:.3g}, "
        f"true output={metrics.maximum_true_output_violation:.3g}; "
        f"maximum predicted-output slack={metrics.maximum_slack:.3g}"
    )


def run_simulation(scenario: Scenario, verbose: bool = True) -> SimulationResult:
    """Run one deterministic closed-loop experiment and return all signals."""
    n = scenario.n_steps
    ny, nu = scenario.plant.ny, scenario.plant.nu
    rng = np.random.default_rng(scenario.random_seed)

    scenario.plant.reset(scenario.y0, scenario.u0)
    controller = MIMOMPC(scenario.model, scenario.controller_config)
    controller.reset(scenario.y0, scenario.u0)

    outputs = np.zeros((n + 1, ny), dtype=float)
    measurements = np.zeros((n, ny), dtype=float)
    commanded_inputs = np.zeros((n, nu), dtype=float)
    plant_inputs = np.zeros((n, nu), dtype=float)
    outputs[0, :] = scenario.y0
    controller_steps: list[MPCStepResult] = []

    for sample in range(n):
        measurement = outputs[sample, :] + rng.normal(
            loc=0.0, scale=scenario.measurement_noise_std
        )
        measurements[sample, :] = measurement

        if scenario.use_setpoint_preview:
            reference = scenario.setpoints[sample:, :]
        else:
            reference = scenario.setpoints[sample, :]

        control_result = controller.control(measurement, reference)
        command = control_result.u
        plant_input = command + scenario.plant_input_disturbance[sample, :]

        commanded_inputs[sample, :] = command
        plant_inputs[sample, :] = plant_input
        outputs[sample + 1, :] = scenario.plant.step(plant_input)
        controller_steps.append(control_result)

    time_axis = np.arange(n + 1, dtype=float) * scenario.plant.dt
    metrics = _calculate_metrics(
        scenario, controller, outputs, commanded_inputs, controller_steps
    )
    result = SimulationResult(
        scenario=scenario,
        time=time_axis,
        outputs=outputs,
        measurements=measurements,
        commanded_inputs=commanded_inputs,
        plant_inputs=plant_inputs,
        setpoints=scenario.setpoints.copy(),
        controller_steps=controller_steps,
        metrics=metrics,
    )
    if verbose:
        print_summary(result)
    return result


def plot_simulation(result: SimulationResult):
    """Plot each output and input on its own correctly scaled axis."""
    scenario = result.scenario
    ny, nu = scenario.plant.ny, scenario.plant.nu
    figure, axes = plt.subplots(
        ny + nu,
        1,
        figsize=(11, 2.5 * (ny + nu)),
        sharex=True,
        squeeze=False,
    )
    axes = axes[:, 0]
    figure.suptitle(scenario.name)

    output_min = _vector(
        scenario.controller_config.output_min, ny, "output_min", -np.inf
    )
    output_max = _vector(
        scenario.controller_config.output_max, ny, "output_max", np.inf
    )
    for output_index in range(ny):
        axis = axes[output_index]
        reference = np.append(
            result.setpoints[:, output_index], result.setpoints[-1, output_index]
        )
        axis.plot(
            result.time,
            result.outputs[:, output_index],
            linewidth=2.0,
            label=scenario.output_labels[output_index],
        )
        axis.step(
            result.time,
            reference,
            where="post",
            linestyle="--",
            label="setpoint",
        )
        if np.isfinite(output_min[output_index]):
            axis.axhline(output_min[output_index], color="tab:red", linestyle=":")
        if np.isfinite(output_max[output_index]):
            axis.axhline(output_max[output_index], color="tab:red", linestyle=":")
        axis.set_ylabel(scenario.output_labels[output_index])
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best")

    input_min = _vector(
        scenario.controller_config.input_min, nu, "input_min", -np.inf
    )
    input_max = _vector(
        scenario.controller_config.input_max, nu, "input_max", np.inf
    )
    for input_index in range(nu):
        axis = axes[ny + input_index]
        commanded_signal = np.append(
            result.commanded_inputs[:, input_index],
            result.commanded_inputs[-1, input_index],
        )
        axis.step(
            result.time,
            commanded_signal,
            where="post",
            linewidth=1.8,
            label=f"commanded {scenario.input_labels[input_index]}",
        )
        if not np.allclose(
            result.commanded_inputs[:, input_index],
            result.plant_inputs[:, input_index],
        ):
            plant_signal = np.append(
                result.plant_inputs[:, input_index],
                result.plant_inputs[-1, input_index],
            )
            axis.step(
                result.time,
                plant_signal,
                where="post",
                alpha=0.7,
                label=f"actual {scenario.input_labels[input_index]}",
            )
        if np.isfinite(input_min[input_index]):
            axis.axhline(input_min[input_index], color="tab:red", linestyle=":")
        if np.isfinite(input_max[input_index]):
            axis.axhline(input_max[input_index], color="tab:red", linestyle=":")
        axis.set_ylabel(scenario.input_labels[input_index])
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best")

    axes[-1].set_xlabel("Time")
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.97))
    return figure


# =============================================================================
# 5. DEMONSTRATION BUILDERS
# =============================================================================


def build_mimo_fopdt_scenario() -> Scenario:
    dt = 1.0
    gain = np.array([[1.5, 0.5], [-0.2, 1.0]])
    time_constant = np.array([[5.0, 3.0], [2.0, 4.0]])
    dead_time = np.array([[2.0, 1.0], [0.0, 1.5]])
    plant = MIMOFOPDT(gain, time_constant, dead_time, dt)
    model = MIMOFOPDT(gain, time_constant, dead_time, dt)

    n_steps = 65
    setpoints = np.zeros((n_steps, 2), dtype=float)
    setpoints[10:, 0] = 1.0
    setpoints[32:, 1] = 0.5

    config = MPCConfig(
        prediction_horizon=15,
        control_horizon=5,
        output_weights=np.array([1.0, 1.0]),
        move_weights=np.array([0.15, 0.15]),
        terminal_weights=np.array([2.0, 2.0]),
        output_scale=np.array([1.0, 0.5]),
        input_scale=np.array([1.0, 1.0]),
        input_min=np.array([-2.0, -2.0]),
        input_max=np.array([2.0, 2.0]),
        move_min=np.array([-0.5, -0.5]),
        move_max=np.array([0.5, 0.5]),
        output_min=np.array([-0.15, -0.15]),
        output_max=np.array([1.15, 0.65]),
        bias_filter=0.25,
    )
    return Scenario(
        name="2x2 cross-coupled FOPDT process",
        n_steps=n_steps,
        plant=plant,
        model=model,
        y0=np.zeros(2),
        u0=np.zeros(2),
        setpoints=setpoints,
        controller_config=config,
        output_labels=("Output 1", "Output 2"),
        input_labels=("Input 1", "Input 2"),
    )


def build_integrating_tank_scenario() -> Scenario:
    dt = 0.5
    plant = IntegratingTank(K=0.2, theta=1.25, dt=dt)
    model = IntegratingTank(K=0.2, theta=1.25, dt=dt)
    n_steps = 70
    setpoints = np.full((n_steps, 1), 2.0, dtype=float)
    setpoints[8:, 0] = 10.0

    config = MPCConfig(
        prediction_horizon=24,
        control_horizon=6,
        output_weights=np.array([5.0]),
        move_weights=np.array([0.5]),
        terminal_weights=np.array([8.0]),
        output_scale=np.array([10.0]),
        input_scale=np.array([5.0]),
        input_min=np.array([-5.0]),
        input_max=np.array([5.0]),
        move_min=np.array([-1.0]),
        move_max=np.array([1.0]),
        output_min=np.array([0.0]),
        output_max=np.array([12.0]),
        bias_filter=0.25,
    )
    return Scenario(
        name="Integrating tank with 1.25-unit dead time",
        n_steps=n_steps,
        plant=plant,
        model=model,
        y0=np.array([2.0]),
        u0=np.array([0.0]),
        setpoints=setpoints,
        controller_config=config,
        output_labels=("Tank level",),
        input_labels=("Net inflow",),
    )


def build_cstr_scenario(model_mismatch: bool = False) -> Scenario:
    dt = 0.5
    nominal_parameters = CSTRParameters()
    model = NonlinearCSTR(dt, nominal_parameters, integration_substeps=2)

    if model_mismatch:
        plant_parameters = replace(
            nominal_parameters,
            pre_exponential_factor=1.03 * nominal_parameters.pre_exponential_factor,
            heat_transfer=1.07 * nominal_parameters.heat_transfer,
        )
    else:
        plant_parameters = nominal_parameters
    plant = NonlinearCSTR(dt, plant_parameters, integration_substeps=2)

    # This is an actual equilibrium, not an approximate arbitrary state.
    y0, u0 = plant.equilibrium_for_temperature_and_flow(
        temperature=320.0,
        flow=10.0,
    )
    n_steps = 75
    setpoints = np.tile(y0, (n_steps, 1))
    setpoints[12:, :] = np.array([0.45, 330.0])

    config = MPCConfig(
        prediction_horizon=10,
        control_horizon=3,
        output_weights=np.array([3.0, 2.0]),
        move_weights=np.array([0.3, 0.2]),
        terminal_weights=np.array([8.0, 6.0]),
        output_scale=np.array([0.10, 10.0]),
        input_scale=np.array([5.0, 20.0]),
        input_min=np.array([5.0, 280.0]),
        input_max=np.array([25.0, 340.0]),
        move_min=np.array([-3.0, -8.0]),
        move_max=np.array([3.0, 8.0]),
        output_min=np.array([0.25, 300.0]),
        output_max=np.array([1.05, 335.0]),
        # Soft constraints keep the NMPC problem feasible if mismatch makes a
        # hard predicted-output limit temporarily unattainable.
        soft_output_weights=np.array([2.0e4, 2.0e4]),
        maximum_output_slack=np.array([0.20, 5.0]),
        bias_filter=0.20,
        solver_max_iterations=180,
    )

    disturbance = np.zeros((n_steps, 2), dtype=float)
    noise = np.zeros(2, dtype=float)
    name = "Nonlinear 2x2 exothermic CSTR"
    if model_mismatch:
        disturbance[48:, 0] = 0.35
        noise = np.array([0.0005, 0.03])
        name += " with plant/model mismatch, disturbance, and noise"

    return Scenario(
        name=name,
        n_steps=n_steps,
        plant=plant,
        model=model,
        y0=y0,
        u0=u0,
        setpoints=setpoints,
        controller_config=config,
        output_labels=("Concentration [mol/L]", "Temperature [K]"),
        input_labels=("Feed flow [L/min]", "Coolant temperature [K]"),
        plant_input_disturbance=disturbance,
        measurement_noise_std=noise,
        random_seed=17,
    )


# Familiar demonstration entry points retained from the original file.
def demo_mimo_fopdt(show_plot: bool = True) -> SimulationResult:
    result = run_simulation(build_mimo_fopdt_scenario())
    if show_plot:
        plot_simulation(result)
        plt.show()
    return result


def demo_integrating_tank(show_plot: bool = True) -> SimulationResult:
    result = run_simulation(build_integrating_tank_scenario())
    if show_plot:
        plot_simulation(result)
        plt.show()
    return result


def demo_nonlinear_cstr(show_plot: bool = True) -> SimulationResult:
    result = run_simulation(build_cstr_scenario(model_mismatch=False))
    if show_plot:
        plot_simulation(result)
        plt.show()
    return result


# =============================================================================
# 6. DETERMINISTIC SELF-TESTS
# =============================================================================


def _assert(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def run_self_tests() -> None:
    """Exercise timing, initialisation, constraints, tracking, and fallback."""
    print("Running deterministic self-tests...")

    # Exact fractional FOPDT response: old input acts for 0.5 time units and
    # the new unit input then acts for 0.5 time units with tau=1.
    fractional_fopdt = MIMOFOPDT(
        K=np.array([[1.0]]),
        tau=np.array([[1.0]]),
        theta=np.array([[0.5]]),
        dt=1.0,
    )
    fractional_fopdt.reset(np.array([0.0]), np.array([0.0]))
    response = fractional_fopdt.step(np.array([1.0]))[0]
    _assert(
        math.isclose(response, 1.0 - math.exp(-0.5), rel_tol=0.0, abs_tol=1e-12),
        "Fractional FOPDT timing test failed.",
    )

    # Arbitrary operating-point initialisation must remain steady at u0.
    initialised_fopdt = MIMOFOPDT(
        K=np.array([[2.0, -0.5]]),
        tau=np.array([[3.0, 4.0]]),
        theta=np.array([[0.0, 0.75]]),
        dt=0.5,
    )
    initialised_fopdt.reset(np.array([7.0]), np.array([1.2, -0.3]))
    steady_output = initialised_fopdt.step(np.array([1.2, -0.3]))[0]
    _assert(
        math.isclose(steady_output, 7.0, rel_tol=0.0, abs_tol=1e-12),
        "FOPDT operating-point initialisation test failed.",
    )

    fractional_integrator = IntegratingTank(K=1.0, theta=0.25, dt=1.0)
    fractional_integrator.reset(np.array([0.0]), np.array([0.0]))
    integrated = fractional_integrator.step(np.array([1.0]))[0]
    _assert(
        math.isclose(integrated, 0.75, rel_tol=0.0, abs_tol=1e-12),
        "Fractional integrating-delay test failed.",
    )

    equilibrium_cstr = NonlinearCSTR(dt=0.5, integration_substeps=2)
    equilibrium_state, equilibrium_input = (
        equilibrium_cstr.equilibrium_for_temperature_and_flow(320.0, 10.0)
    )
    equilibrium_cstr.reset(equilibrium_state, equilibrium_input)
    equilibrium_after_step = equilibrium_cstr.step(equilibrium_input)
    _assert(
        np.allclose(
            equilibrium_after_step,
            equilibrium_state,
            rtol=0.0,
            atol=1e-10,
        ),
        "CSTR equilibrium calculation or integration test failed.",
    )

    standard_results = [
        run_simulation(build_mimo_fopdt_scenario(), verbose=False),
        run_simulation(build_integrating_tank_scenario(), verbose=False),
        run_simulation(build_cstr_scenario(model_mismatch=False), verbose=False),
    ]
    for result in standard_results:
        metrics = result.metrics
        _assert(metrics.fallback_count == 0, f"Unexpected fallback in {result.scenario.name}.")
        _assert(
            metrics.maximum_input_violation <= 2e-6,
            f"Input constraint failure in {result.scenario.name}.",
        )
        _assert(
            metrics.maximum_move_violation <= 2e-6,
            f"Move constraint failure in {result.scenario.name}.",
        )

    _assert(
        np.max(np.abs(standard_results[0].metrics.final_error)) < 2.0e-3,
        "FOPDT final tracking test failed.",
    )
    _assert(
        abs(standard_results[1].metrics.final_error[0]) < 2.0e-2,
        "Integrating-tank final tracking test failed.",
    )
    cstr_error = standard_results[2].metrics.final_error
    _assert(
        abs(cstr_error[0]) < 2.0e-3 and abs(cstr_error[1]) < 0.08,
        "CSTR two-output final tracking test failed.",
    )

    # A constant output outside its soft maximum needs explicit positive slack.
    constant_model = MIMOFOPDT(
        K=np.array([[0.0]]),
        tau=np.array([[1.0]]),
        theta=np.array([[0.0]]),
        dt=1.0,
    )
    soft_config = MPCConfig(
        prediction_horizon=3,
        control_horizon=1,
        output_weights=np.array([1.0]),
        move_weights=np.array([0.1]),
        input_min=np.array([-1.0]),
        input_max=np.array([1.0]),
        output_max=np.array([1.0]),
        soft_output_weights=np.array([100.0]),
    )
    soft_controller = MIMOMPC(constant_model, soft_config)
    soft_controller.reset(np.array([2.0]), np.array([0.0]))
    soft_result = soft_controller.control(np.array([2.0]), np.array([2.0]))
    _assert(soft_result.success, "Soft-output-constraint solve failed.")
    _assert(
        0.999 <= soft_result.maximum_slack <= 1.001,
        "Explicit output slack test failed.",
    )

    # Force a solver iteration-limit failure and verify deterministic fallback.
    fallback_model = MIMOFOPDT(
        K=np.array([[1.0]]),
        tau=np.array([[2.0]]),
        theta=np.array([[0.0]]),
        dt=1.0,
    )
    fallback_config = MPCConfig(
        prediction_horizon=6,
        control_horizon=3,
        input_min=np.array([-2.0]),
        input_max=np.array([2.0]),
        move_min=np.array([-0.5]),
        move_max=np.array([0.5]),
        solver_max_iterations=1,
    )
    fallback_controller = MIMOMPC(fallback_model, fallback_config)
    fallback_controller.reset(np.array([0.0]), np.array([0.0]))
    fallback_result = fallback_controller.control(np.array([0.0]), np.array([1.0]))
    _assert(fallback_result.fallback_used, "Forced solver failure did not use fallback.")
    _assert(
        np.allclose(fallback_result.u, np.array([0.0])),
        "Fallback did not hold the last feasible input.",
    )

    print("All deterministic self-tests passed.")


# =============================================================================
# 7. COMMAND-LINE ENTRY POINT
# =============================================================================


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--demo",
        choices=("all", "mimo-fopdt", "tank", "cstr", "cstr-mismatch"),
        default="all",
        help="Select one demonstration; 'all' runs the three standard cases.",
    )
    parser.add_argument(
        "--no-plots",
        action="store_true",
        help="Run and print metrics without opening Matplotlib windows.",
    )
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="Run deterministic validation checks and exit.",
    )
    args = parser.parse_args()

    if args.self_test:
        run_self_tests()
        return

    builders = {
        "mimo-fopdt": build_mimo_fopdt_scenario,
        "tank": build_integrating_tank_scenario,
        "cstr": lambda: build_cstr_scenario(model_mismatch=False),
        "cstr-mismatch": lambda: build_cstr_scenario(model_mismatch=True),
    }
    selected = (
        ("mimo-fopdt", "tank", "cstr")
        if args.demo == "all"
        else (args.demo,)
    )

    for name in selected:
        simulation_result = run_simulation(builders[name]())
        if not args.no_plots:
            plot_simulation(simulation_result)
    if not args.no_plots:
        plt.show()


if __name__ == "__main__":
    main()
