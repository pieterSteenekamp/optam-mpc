#!/usr/bin/env python3
"""
Enhanced MPC test harness for a SISO FOPDT process.

The program is deliberately self-contained and intended for both teaching and
controller verification.  It includes:

* exact zero-order-hold discretisation of the first-order dynamics;
* zero, integer and fraction-of-a-sample dead time;
* offset-free output-bias estimation with configurable filtering;
* separate plant state, load disturbance, output disturbance and noise;
* prediction and control horizons;
* input, input-move and hard/soft output constraints;
* explicit slack variables for soft output constraints;
* shifted-solution warm starts and terminal tracking cost;
* optional economic optimisation independent of the constraints;
* aligned time-series recording, solver diagnostics and prediction snapshots;
* closed-loop performance metrics and an automated validation suite.

Time convention
---------------
At sample k the controller receives y_measured[k], computes u[k], and the
plant then advances to y_true[k+1].  A load disturbance is added to the plant
input over that interval.  Output disturbance and measurement noise are added
only when the measurement at the next sample is formed.

Examples
--------
    python test_mpc.py                 # Run demonstrations and show plots
    python test_mpc.py --no-plots      # Run demonstrations without plots
    python test_mpc.py --self-test     # Run automated validation scenarios
"""

from __future__ import annotations

import argparse
import math
import warnings
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Literal, Optional

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import minimize


SignalFunction = Callable[[int], float]
ConstraintMode = Literal["none", "hard", "soft"]
EconomicVariable = Literal["input", "output"]


# =============================================================================
# 1. PROCESS MODEL
# =============================================================================


class FOPDTProcess:
    """First-order-plus-dead-time process with exact discrete dynamics.

    Continuous transfer function::

                  K
        G(s) = -------- exp(-theta s)
               tau s + 1

    Inputs are treated as zero-order-held between samples.  For fractional
    dead time, the delayed input is linearly interpolated between the two
    surrounding samples.  The input-history list is stored most-recent first.
    """

    def __init__(
        self,
        gain: float = 1.0,
        tau: float = 5.0,
        dead_time: float = 2.0,
        dt: float = 1.0,
    ) -> None:
        self.K = float(gain)
        self.tau = float(tau)
        self.theta = float(dead_time)
        self.dt = float(dt)
        self._validate_parameters()

        ratio = self.theta / self.dt
        delay_steps = int(math.floor(ratio + 1e-12))
        fraction = ratio - delay_steps
        if fraction < 1e-12:
            fraction = 0.0
        elif 1.0 - fraction < 1e-12:
            delay_steps += 1
            fraction = 0.0

        self._delay_steps = delay_steps
        self._delay_fraction = fraction
        required = delay_steps + (1 if fraction > 0.0 else 0)
        self._history_length = max(1, required)
        self._a = math.exp(-self.dt / self.tau)

        self.y = 0.0
        self._u_history: list[float] = []
        self.reset()

    def _validate_parameters(self) -> None:
        values = (self.K, self.tau, self.theta, self.dt)
        if not all(np.isfinite(values)):
            raise ValueError("FOPDT parameters must all be finite.")
        if self.tau <= 0.0:
            raise ValueError("The process time constant tau must be positive.")
        if self.dead_time < 0.0:
            raise ValueError("The process dead time must not be negative.")
        if self.dt <= 0.0:
            raise ValueError("The sample interval dt must be positive.")

    @property
    def dead_time(self) -> float:
        return self.theta

    @property
    def history_length(self) -> int:
        return self._history_length

    def reset(
        self,
        y0: float = 0.0,
        u0: float = 0.0,
        input_history: Optional[list[float]] = None,
    ) -> None:
        """Reset the state and delayed-input history.

        ``input_history`` must be ordered most-recent first.  If it is omitted,
        the process is assumed to have received ``u0`` throughout its dead-time
        pipeline.  ``y0`` and ``u0`` need not describe a steady state.
        """

        if not np.isfinite(y0) or not np.isfinite(u0):
            raise ValueError("Initial output and input must be finite.")
        self.y = float(y0)

        if input_history is None:
            self._u_history = [float(u0)] * self._history_length
        else:
            if len(input_history) < self._history_length:
                raise ValueError(
                    f"At least {self._history_length} historical inputs are required."
                )
            history = np.asarray(input_history[: self._history_length], dtype=float)
            if not np.all(np.isfinite(history)):
                raise ValueError("Input history must contain only finite values.")
            self._u_history = history.tolist()

    def clone(self) -> "FOPDTProcess":
        """Return an independent model with identical state and input history."""

        other = FOPDTProcess(self.K, self.tau, self.theta, self.dt)
        other.y = self.y
        other._u_history = self._u_history.copy()
        return other

    def _delayed_input(self, u_current: float) -> float:
        """Interpolate the input at the requested dead time."""

        q = self._delay_steps
        f = self._delay_fraction

        if q == 0:
            newer = u_current
        else:
            newer = self._u_history[q - 1]

        if f == 0.0:
            return float(newer)

        older = self._u_history[q]
        return float((1.0 - f) * newer + f * older)

    def step(self, u: float) -> float:
        """Apply ``u`` over one interval and return y at the next sample."""

        if not np.isfinite(u):
            raise ValueError("Process input must be finite.")
        u = float(u)
        u_delayed = self._delayed_input(u)
        self.y = self._a * self.y + (1.0 - self._a) * self.K * u_delayed

        self._u_history.insert(0, u)
        del self._u_history[self._history_length :]
        return self.y


# =============================================================================
# 2. CONFIGURATION AND RESULT TYPES
# =============================================================================


@dataclass
class Scenario:
    """Complete definition of one closed-loop simulation."""

    name: str = "Unnamed"
    n_steps: int = 100
    dt: float = 1.0
    y0: float = 0.0
    u0: float = 0.0

    # Real plant.
    process_gain: float = 1.0
    process_tau: float = 5.0
    process_dead_time: float = 2.0

    # Controller model.  None means use the corresponding plant parameter.
    model_gain: Optional[float] = None
    model_tau: Optional[float] = None
    model_dead_time: Optional[float] = None

    # Setpoint changes at measurement sample indices.
    setpoint_sequence: dict[int, float] = field(default_factory=lambda: {0: 1.0})

    # Disturbances.  A load disturbance is added to the plant input.  An output
    # disturbance is added after the plant dynamics when a measurement is made.
    load_disturbance: Optional[SignalFunction] = None
    output_disturbance: Optional[SignalFunction] = None
    measurement_noise_std: float = 0.0
    random_seed: int = 1

    # MPC horizons and tuning.
    prediction_horizon: int = 15
    control_horizon: int = 5
    tracking_weight: float = 1.0
    move_weight: float = 0.1
    terminal_weight: float = 2.0
    disturbance_filter: float = 0.25
    y_scale: float = 1.0
    u_scale: float = 1.0

    # Input and move constraints.
    u_min: float = -5.0
    u_max: float = 5.0
    delta_u_min: float = -np.inf
    delta_u_max: float = np.inf

    # Output constraints.  Soft constraints use explicit non-negative slacks.
    y_min: float = -np.inf
    y_max: float = np.inf
    output_constraint_mode: ConstraintMode = "none"
    slack_quadratic_weight: float = 10_000.0
    slack_linear_weight: float = 100.0

    # Economic stage objective, kept independent from the constraints.
    economic_weight: float = 0.0
    economic_direction: float = 1.0
    economic_variable: EconomicVariable = "input"

    # Optimiser and reporting.
    optimizer_max_iterations: int = 300
    optimizer_tolerance: float = 1e-8
    settling_tolerance: float = 0.02
    snapshot_steps: tuple[int, ...] = ()

    def resolved_model_parameters(self) -> tuple[float, float, float]:
        return (
            self.process_gain if self.model_gain is None else self.model_gain,
            self.process_tau if self.model_tau is None else self.model_tau,
            self.process_dead_time
            if self.model_dead_time is None
            else self.model_dead_time,
        )

    def validate(self) -> None:
        if not isinstance(self.n_steps, int) or self.n_steps < 1:
            raise ValueError("n_steps must be a positive integer.")
        if not isinstance(self.prediction_horizon, int) or self.prediction_horizon < 1:
            raise ValueError("prediction_horizon must be a positive integer.")
        if not isinstance(self.control_horizon, int) or self.control_horizon < 1:
            raise ValueError("control_horizon must be a positive integer.")
        if self.control_horizon > self.prediction_horizon:
            raise ValueError("control_horizon must not exceed prediction_horizon.")
        if not isinstance(self.optimizer_max_iterations, int) or self.optimizer_max_iterations < 1:
            raise ValueError("optimizer_max_iterations must be a positive integer.")
        if not np.isfinite(self.y0) or not np.isfinite(self.u0):
            raise ValueError("y0 and u0 must be finite.")
        if self.u_min > self.u_max:
            raise ValueError("u_min must not exceed u_max.")
        if not self.u_min <= self.u0 <= self.u_max:
            raise ValueError("u0 must lie between u_min and u_max.")
        if self.y_min > self.y_max:
            raise ValueError("y_min must not exceed y_max.")
        if self.delta_u_min > self.delta_u_max:
            raise ValueError("delta_u_min must not exceed delta_u_max.")
        if not 0.0 <= self.disturbance_filter <= 1.0:
            raise ValueError("disturbance_filter must lie between zero and one.")
        if self.measurement_noise_std < 0.0:
            raise ValueError("measurement_noise_std must not be negative.")
        if self.y_scale <= 0.0 or self.u_scale <= 0.0:
            raise ValueError("y_scale and u_scale must be positive.")
        if self.output_constraint_mode not in ("none", "hard", "soft"):
            raise ValueError("output_constraint_mode must be none, hard or soft.")
        if self.economic_variable not in ("input", "output"):
            raise ValueError("economic_variable must be input or output.")

        nonnegative = {
            "tracking_weight": self.tracking_weight,
            "move_weight": self.move_weight,
            "terminal_weight": self.terminal_weight,
            "economic_weight": self.economic_weight,
            "slack_quadratic_weight": self.slack_quadratic_weight,
            "slack_linear_weight": self.slack_linear_weight,
            "optimizer_tolerance": self.optimizer_tolerance,
            "settling_tolerance": self.settling_tolerance,
        }
        for label, value in nonnegative.items():
            if value < 0.0 or not np.isfinite(value):
                raise ValueError(f"{label} must be finite and non-negative.")

        if not self.setpoint_sequence:
            raise ValueError("setpoint_sequence must contain at least one entry.")
        if any(not isinstance(k, int) or k < 0 for k in self.setpoint_sequence):
            raise ValueError("Setpoint sequence keys must be non-negative integers.")
        if any(not np.isfinite(v) for v in self.setpoint_sequence.values()):
            raise ValueError("Setpoints must be finite.")

        # Constructing these objects also validates tau, dead time and dt.
        FOPDTProcess(
            self.process_gain, self.process_tau, self.process_dead_time, self.dt
        )
        model_gain, model_tau, model_dead_time = self.resolved_model_parameters()
        FOPDTProcess(model_gain, model_tau, model_dead_time, self.dt)


@dataclass
class PredictionSnapshot:
    """Prediction made at one control sample."""

    sample: int
    future_time: np.ndarray
    predicted_output: np.ndarray
    planned_input: np.ndarray
    setpoint: float


@dataclass
class SolverDiagnostics:
    """Details of one MPC optimisation."""

    success: bool
    message: str
    iterations: int
    objective: float
    minimum_constraint_residual: float
    maximum_predicted_violation: float
    maximum_slack: float
    warnings: tuple[str, ...]
    model_output: float
    disturbance_estimate: float
    predicted_output: np.ndarray
    planned_input: np.ndarray


@dataclass
class PerformanceMetrics:
    """Closed-loop performance measures."""

    steady_state_error: float
    steady_state_std: float
    integral_absolute_error: float
    integral_squared_error: float
    maximum_absolute_error: float
    overshoot: float
    settling_time: float
    total_input_movement: float
    maximum_output_constraint_violation: float
    maximum_input_constraint_violation: float
    maximum_move_constraint_violation: float
    solver_failures: int
    maximum_slack: float
    saturation_fraction: float
    oscillation_detected: bool


@dataclass
class Result:
    """Aligned time-series results from a closed-loop simulation."""

    scenario_name: str
    time: np.ndarray
    setpoint: np.ndarray
    y_true: np.ndarray
    y_controlled: np.ndarray
    y_measured: np.ndarray
    y_model: np.ndarray
    u_time: np.ndarray
    u_applied: np.ndarray
    u_effective: np.ndarray
    load_disturbance: np.ndarray
    output_disturbance: np.ndarray
    measurement_noise: np.ndarray
    disturbance_estimate: np.ndarray
    solver_success: np.ndarray
    solver_iterations: np.ndarray
    solver_objective: np.ndarray
    solver_messages: tuple[str, ...]
    solver_warnings: tuple[tuple[str, ...], ...]
    minimum_constraint_residual: np.ndarray
    maximum_predicted_violation: np.ndarray
    maximum_slack: np.ndarray
    prediction_snapshots: dict[int, PredictionSnapshot]
    metrics: Optional[PerformanceMetrics] = None


# =============================================================================
# 3. MPC CONTROLLER
# =============================================================================


class SISOMPC:
    """SISO nonlinear-programming MPC for an FOPDT model.

    Offset-free action is provided by an additive output-bias estimate:

        d_hat[k] = alpha * (y_measured[k] - y_model[k])
                   + (1-alpha) * d_hat[k-1]

    The internal model is not forced to the measurement.  It is advanced with
    the actual commanded input, and its full delayed-input pipeline is cloned
    into every prediction.  A constant d_hat is added to future model outputs.

    This approach removes offset for reachable steady states and constant
    disturbances under suitable closed-loop conditions; it is not a universal
    stability or zero-offset guarantee under arbitrary mismatch or constraints.
    """

    def __init__(self, model: FOPDTProcess, scenario: Scenario) -> None:
        scenario.validate()
        self.model = model
        self.config = scenario
        self.Np = scenario.prediction_horizon
        self.Nc = scenario.control_horizon

        self._internal_model = FOPDTProcess(
            model.K, model.tau, model.theta, model.dt
        )
        self._d_hat = 0.0
        self._prev_u = scenario.u0
        self._last_plan: Optional[np.ndarray] = None
        self._initialised = False

    def reset(self, y0: float = 0.0, u0: float = 0.0) -> None:
        """Initialise model state, previous move and dead-time pipeline."""

        self._internal_model.reset(y0=y0, u0=u0)
        self._prev_u = float(np.clip(u0, self.config.u_min, self.config.u_max))
        self._d_hat = 0.0
        self._last_plan = None
        self._initialised = True

    def _expand_moves(self, moves: np.ndarray) -> np.ndarray:
        """Hold the final control-horizon move to the prediction horizon."""

        if self.Nc == self.Np:
            return moves.copy()
        return np.concatenate(
            (moves, np.full(self.Np - self.Nc, moves[-1], dtype=float))
        )

    def _project_moves(self, moves: np.ndarray, u_previous: float) -> np.ndarray:
        """Project an initial guess onto input and move constraints."""

        cfg = self.config
        projected = np.empty(self.Nc)
        previous = u_previous
        for j, target in enumerate(np.asarray(moves, dtype=float)):
            target = float(np.clip(target, cfg.u_min, cfg.u_max))
            delta = target - previous
            delta = float(np.clip(delta, cfg.delta_u_min, cfg.delta_u_max))
            target = float(np.clip(previous + delta, cfg.u_min, cfg.u_max))
            projected[j] = target
            previous = target
        return projected

    def _initial_move_guess(self, setpoint: float) -> np.ndarray:
        cfg = self.config
        if self._last_plan is not None:
            shifted = np.concatenate((self._last_plan[1:], self._last_plan[-1:]))
            return self._project_moves(shifted, self._prev_u)

        if self.model.K != 0.0:
            target = (setpoint - self._d_hat) / self.model.K
        else:
            target = self._prev_u
        target = float(np.clip(target, cfg.u_min, cfg.u_max))
        guess = np.linspace(self._prev_u, target, self.Nc + 1)[1:]
        return self._project_moves(guess, self._prev_u)

    def step(self, y_measured: float, setpoint: float) -> tuple[float, SolverDiagnostics]:
        """Compute and return the current input and optimisation diagnostics."""

        if not np.isfinite(y_measured) or not np.isfinite(setpoint):
            raise ValueError("Measured output and setpoint must be finite.")
        if not self._initialised:
            self.reset(y0=y_measured, u0=self.config.u0)

        cfg = self.config
        model_output_now = self._internal_model.y
        raw_bias = y_measured - model_output_now
        alpha = cfg.disturbance_filter
        self._d_hat = alpha * raw_bias + (1.0 - alpha) * self._d_hat
        d_hat = self._d_hat
        u_previous = self._prev_u

        has_low = np.isfinite(cfg.y_min)
        has_high = np.isfinite(cfg.y_max)
        use_soft = cfg.output_constraint_mode == "soft"
        n_low_slack = self.Np if use_soft and has_low else 0
        n_high_slack = self.Np if use_soft and has_high else 0

        def unpack(x: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
            cursor = self.Nc
            low_slack = x[cursor : cursor + n_low_slack]
            cursor += n_low_slack
            high_slack = x[cursor : cursor + n_high_slack]
            return x[: self.Nc], low_slack, high_slack

        def predict_from_moves(moves: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
            planned_input = self._expand_moves(moves)
            predictor = self._internal_model.clone()
            predicted_output = np.empty(self.Np)
            for j, u_future in enumerate(planned_input):
                predicted_output[j] = predictor.step(float(u_future)) + d_hat
            return predicted_output, planned_input

        def objective(x: np.ndarray) -> float:
            moves, low_slack, high_slack = unpack(x)
            y_pred, u_pred = predict_from_moves(moves)
            error = (y_pred - setpoint) / cfg.y_scale
            delta_moves = np.diff(np.concatenate(([u_previous], moves))) / cfg.u_scale

            value = cfg.tracking_weight * float(np.dot(error, error))
            value += cfg.move_weight * float(np.dot(delta_moves, delta_moves))
            value += cfg.terminal_weight * float(error[-1] ** 2)

            if cfg.economic_weight > 0.0:
                if cfg.economic_variable == "input":
                    economic_signal = u_pred / cfg.u_scale
                else:
                    economic_signal = y_pred / cfg.y_scale
                benefit = cfg.economic_direction * float(np.sum(economic_signal))
                value -= cfg.economic_weight * benefit

            if low_slack.size:
                normalised = low_slack / cfg.y_scale
                value += cfg.slack_quadratic_weight * float(
                    np.dot(normalised, normalised)
                )
                value += cfg.slack_linear_weight * float(np.sum(normalised))
            if high_slack.size:
                normalised = high_slack / cfg.y_scale
                value += cfg.slack_quadratic_weight * float(
                    np.dot(normalised, normalised)
                )
                value += cfg.slack_linear_weight * float(np.sum(normalised))
            return float(value)

        def inequality_constraints(x: np.ndarray) -> np.ndarray:
            moves, low_slack, high_slack = unpack(x)
            residuals: list[np.ndarray] = []

            move_deltas = np.diff(np.concatenate(([u_previous], moves)))
            if np.isfinite(cfg.delta_u_min):
                residuals.append(move_deltas - cfg.delta_u_min)
            if np.isfinite(cfg.delta_u_max):
                residuals.append(cfg.delta_u_max - move_deltas)

            if cfg.output_constraint_mode != "none" and (has_low or has_high):
                y_pred, _ = predict_from_moves(moves)
                if has_low:
                    residual = y_pred - cfg.y_min
                    if use_soft:
                        residual = residual + low_slack
                    residuals.append(residual)
                if has_high:
                    residual = cfg.y_max - y_pred
                    if use_soft:
                        residual = residual + high_slack
                    residuals.append(residual)

            if not residuals:
                return np.array([1.0])
            return np.concatenate(residuals)

        initial_moves = self._initial_move_guess(setpoint)
        initial_y, _ = predict_from_moves(initial_moves)
        initial_parts = [initial_moves]
        if n_low_slack:
            initial_parts.append(np.maximum(cfg.y_min - initial_y, 0.0) + 1e-8)
        if n_high_slack:
            initial_parts.append(np.maximum(initial_y - cfg.y_max, 0.0) + 1e-8)
        x0 = np.concatenate(initial_parts)

        bounds: list[tuple[Optional[float], Optional[float]]] = [
            (cfg.u_min, cfg.u_max)
        ] * self.Nc
        bounds.extend([(0.0, None)] * (n_low_slack + n_high_slack))

        constraint_spec = {"type": "ineq", "fun": inequality_constraints}
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            result = minimize(
                objective,
                x0,
                method="SLSQP",
                bounds=bounds,
                constraints=[constraint_spec],
                options={
                    "maxiter": cfg.optimizer_max_iterations,
                    "ftol": cfg.optimizer_tolerance,
                    "disp": False,
                },
            )

        warning_messages = tuple(str(item.message) for item in caught)
        candidate_x = np.asarray(
            result.x if result.x is not None else x0, dtype=float
        ).copy()
        if use_soft:
            # Soft constraints are feasible by construction.  Numerical line
            # searches can leave a slack a few digits below the minimum value
            # required by its associated prediction.  Project the explicit
            # slack variables back onto their feasibility boundary before
            # judging the candidate; this does not alter the control moves.
            candidate_moves, candidate_low, candidate_high = unpack(candidate_x)
            candidate_y, _ = predict_from_moves(candidate_moves)
            if candidate_low.size:
                candidate_low[:] = np.maximum(
                    candidate_low, np.maximum(cfg.y_min - candidate_y, 0.0)
                )
            if candidate_high.size:
                candidate_high[:] = np.maximum(
                    candidate_high, np.maximum(candidate_y - cfg.y_max, 0.0)
                )
        residual = inequality_constraints(candidate_x)
        minimum_residual = float(np.min(residual))
        # SLSQP sometimes returns status 8 (positive directional derivative)
        # at a feasible active-constraint solution, especially when a hard
        # output limit or a heavily weighted slack is exactly active.  For
        # this convex tracking problem the feasible candidate is still usable;
        # retain the non-success status in the message, but do not discard an
        # otherwise finite and constraint-consistent solution.
        feasible_stationary_point = bool(
            getattr(result, "status", None) == 8 and minimum_residual >= -1e-5
        )
        success = bool(
            (result.success or feasible_stationary_point)
            and np.all(np.isfinite(candidate_x))
            and minimum_residual >= -1e-5
        )

        if success:
            chosen_x = candidate_x
            moves, low_slack, high_slack = unpack(chosen_x)
            moves = self._project_moves(moves, u_previous)
            self._last_plan = moves.copy()
            message = str(result.message)
            if feasible_stationary_point and not result.success:
                message += "; accepted feasible active-constraint solution"
        else:
            # A safe deterministic fallback: hold the previous command, subject
            # to the absolute bounds.  The failure remains visible in Result.
            moves = np.full(self.Nc, np.clip(u_previous, cfg.u_min, cfg.u_max))
            moves = self._project_moves(moves, u_previous)
            low_slack = np.array([], dtype=float)
            high_slack = np.array([], dtype=float)
            chosen_x = moves.copy()
            self._last_plan = None
            message = f"{result.message}; fallback held the previous input"

        predicted_output, planned_input = predict_from_moves(moves)
        u_optimal = float(moves[0])

        lower_violation = (
            float(np.max(np.maximum(cfg.y_min - predicted_output, 0.0)))
            if has_low
            else 0.0
        )
        upper_violation = (
            float(np.max(np.maximum(predicted_output - cfg.y_max, 0.0)))
            if has_high
            else 0.0
        )
        maximum_predicted_violation = max(lower_violation, upper_violation)
        maximum_slack = max(
            float(np.max(low_slack)) if low_slack.size else 0.0,
            float(np.max(high_slack)) if high_slack.size else 0.0,
        )

        # Advance the internal model after the decision so it represents the
        # model output at sample k+1 when the next measurement arrives.
        self._internal_model.step(u_optimal)
        self._prev_u = u_optimal

        if success:
            final_objective = float(objective(chosen_x))
        else:
            final_objective = float(objective(x0))

        diagnostics = SolverDiagnostics(
            success=success,
            message=message,
            iterations=int(getattr(result, "nit", 0)),
            objective=final_objective,
            minimum_constraint_residual=minimum_residual,
            maximum_predicted_violation=maximum_predicted_violation,
            maximum_slack=maximum_slack,
            warnings=warning_messages,
            model_output=float(model_output_now),
            disturbance_estimate=float(d_hat),
            predicted_output=predicted_output,
            planned_input=planned_input,
        )
        return u_optimal, diagnostics


# =============================================================================
# 4. CLOSED-LOOP SIMULATION AND METRICS
# =============================================================================


def _signal_values(
    function: Optional[SignalFunction], count: int
) -> np.ndarray:
    if function is None:
        return np.zeros(count)
    values = np.asarray([function(k) for k in range(count)], dtype=float)
    if not np.all(np.isfinite(values)):
        raise ValueError("A disturbance function returned a non-finite value.")
    return values


def _setpoint_array(scenario: Scenario) -> np.ndarray:
    values = np.empty(scenario.n_steps + 1)
    current = 0.0
    for k in range(scenario.n_steps + 1):
        if k in scenario.setpoint_sequence:
            current = scenario.setpoint_sequence[k]
        values[k] = current
    return values


def calculate_metrics(result: Result, scenario: Scenario) -> PerformanceMetrics:
    """Calculate tracking, constraint, movement and robustness metrics."""

    controlled = result.y_controlled
    error = controlled - result.setpoint
    n_points = error.size
    window = min(20, max(5, n_points // 5))
    final_error = error[-window:]

    last_change = max(
        (k for k in scenario.setpoint_sequence if k <= scenario.n_steps), default=0
    )
    final_sp = result.setpoint[-1]
    before_index = max(0, last_change - 1)
    prior_sp = result.setpoint[before_index]
    segment = controlled[last_change:]
    if final_sp > prior_sp:
        overshoot = max(0.0, float(np.max(segment) - final_sp))
    elif final_sp < prior_sp:
        overshoot = max(0.0, float(final_sp - np.min(segment)))
    else:
        overshoot = 0.0

    tolerance = max(scenario.settling_tolerance * scenario.y_scale, 1e-9)
    settling_time = float("nan")
    segment_error = np.abs(error[last_change:])
    for offset in range(segment_error.size):
        if np.all(segment_error[offset:] <= tolerance):
            settling_time = offset * scenario.dt
            break

    initial_and_applied = np.concatenate(([scenario.u0], result.u_applied))
    move_deltas = np.diff(initial_and_applied)
    input_lower_violation = np.maximum(scenario.u_min - result.u_applied, 0.0)
    input_upper_violation = np.maximum(result.u_applied - scenario.u_max, 0.0)
    max_input_violation = float(
        max(np.max(input_lower_violation), np.max(input_upper_violation))
    )

    move_violation = np.zeros_like(move_deltas)
    if np.isfinite(scenario.delta_u_min):
        move_violation = np.maximum(
            move_violation, scenario.delta_u_min - move_deltas
        )
    if np.isfinite(scenario.delta_u_max):
        move_violation = np.maximum(
            move_violation, move_deltas - scenario.delta_u_max
        )

    lower_output_violation = (
        np.maximum(scenario.y_min - controlled, 0.0)
        if np.isfinite(scenario.y_min)
        else np.zeros_like(controlled)
    )
    upper_output_violation = (
        np.maximum(controlled - scenario.y_max, 0.0)
        if np.isfinite(scenario.y_max)
        else np.zeros_like(controlled)
    )
    max_output_violation = float(
        max(np.max(lower_output_violation), np.max(upper_output_violation))
    )

    saturation_tolerance = 1e-7 * max(1.0, scenario.u_scale)
    saturated = np.logical_or(
        np.abs(result.u_applied - scenario.u_min) <= saturation_tolerance,
        np.abs(result.u_applied - scenario.u_max) <= saturation_tolerance,
    )

    oscillation_window = controlled[-min(24, controlled.size) :]
    centred = oscillation_window - np.mean(oscillation_window)
    oscillation_detected = False
    if centred.size >= 8 and np.std(centred) > 2.0 * tolerance:
        correlation = np.corrcoef(centred[:-1], centred[1:])[0, 1]
        oscillation_detected = bool(np.isfinite(correlation) and correlation < -0.70)

    return PerformanceMetrics(
        steady_state_error=float(np.mean(final_error)),
        steady_state_std=float(np.std(final_error)),
        integral_absolute_error=float(np.sum(np.abs(error)) * scenario.dt),
        integral_squared_error=float(np.sum(error**2) * scenario.dt),
        maximum_absolute_error=float(np.max(np.abs(error))),
        overshoot=overshoot,
        settling_time=settling_time,
        total_input_movement=float(np.sum(np.abs(move_deltas))),
        maximum_output_constraint_violation=max_output_violation,
        maximum_input_constraint_violation=max_input_violation,
        maximum_move_constraint_violation=float(np.max(move_violation)),
        solver_failures=int(np.count_nonzero(~result.solver_success)),
        maximum_slack=float(np.max(result.maximum_slack)),
        saturation_fraction=float(np.mean(saturated)),
        oscillation_detected=oscillation_detected,
    )


def run_simulation(scenario: Scenario, verbose: bool = True) -> Result:
    """Run one aligned closed-loop MPC simulation."""

    scenario.validate()
    model_gain, model_tau, model_dead_time = scenario.resolved_model_parameters()
    plant = FOPDTProcess(
        scenario.process_gain,
        scenario.process_tau,
        scenario.process_dead_time,
        scenario.dt,
    )
    plant.reset(y0=scenario.y0, u0=scenario.u0)
    model = FOPDTProcess(model_gain, model_tau, model_dead_time, scenario.dt)

    setpoint = _setpoint_array(scenario)
    load = _signal_values(scenario.load_disturbance, scenario.n_steps)
    output_disturbance = _signal_values(
        scenario.output_disturbance, scenario.n_steps + 1
    )
    rng = np.random.default_rng(scenario.random_seed)
    noise = rng.normal(0.0, scenario.measurement_noise_std, scenario.n_steps + 1)

    n = scenario.n_steps
    time = np.arange(n + 1, dtype=float) * scenario.dt
    y_true = np.empty(n + 1)
    y_controlled = np.empty(n + 1)
    y_measured = np.empty(n + 1)
    y_model = np.empty(n + 1)
    u_applied = np.empty(n)
    u_effective = np.empty(n)
    d_hat = np.empty(n + 1)
    solver_success = np.empty(n, dtype=bool)
    solver_iterations = np.empty(n, dtype=int)
    solver_objective = np.empty(n)
    minimum_constraint_residual = np.empty(n)
    maximum_predicted_violation = np.empty(n)
    maximum_slack = np.empty(n)
    solver_messages: list[str] = []
    solver_warnings: list[tuple[str, ...]] = []
    snapshots: dict[int, PredictionSnapshot] = {}

    y_true[0] = scenario.y0
    y_controlled[0] = y_true[0] + output_disturbance[0]
    y_measured[0] = y_controlled[0] + noise[0]

    controller = SISOMPC(model, scenario)
    controller.reset(y0=y_measured[0], u0=scenario.u0)

    for k in range(n):
        command, diagnostics = controller.step(y_measured[k], setpoint[k])
        u_applied[k] = command
        u_effective[k] = command + load[k]
        y_model[k] = diagnostics.model_output
        d_hat[k] = diagnostics.disturbance_estimate
        solver_success[k] = diagnostics.success
        solver_iterations[k] = diagnostics.iterations
        solver_objective[k] = diagnostics.objective
        minimum_constraint_residual[k] = diagnostics.minimum_constraint_residual
        maximum_predicted_violation[k] = diagnostics.maximum_predicted_violation
        maximum_slack[k] = diagnostics.maximum_slack
        solver_messages.append(diagnostics.message)
        solver_warnings.append(diagnostics.warnings)

        if k in scenario.snapshot_steps:
            snapshots[k] = PredictionSnapshot(
                sample=k,
                future_time=time[k]
                + np.arange(1, scenario.prediction_horizon + 1) * scenario.dt,
                predicted_output=diagnostics.predicted_output.copy(),
                planned_input=diagnostics.planned_input.copy(),
                setpoint=float(setpoint[k]),
            )

        y_true[k + 1] = plant.step(u_effective[k])
        y_controlled[k + 1] = y_true[k + 1] + output_disturbance[k + 1]
        y_measured[k + 1] = y_controlled[k + 1] + noise[k + 1]

    # The final model state is aligned with the final plant measurement, but
    # that measurement has not yet been processed by the bias estimator.
    y_model[-1] = controller._internal_model.y
    d_hat[-1] = controller._d_hat

    result = Result(
        scenario_name=scenario.name,
        time=time,
        setpoint=setpoint,
        y_true=y_true,
        y_controlled=y_controlled,
        y_measured=y_measured,
        y_model=y_model,
        u_time=time[:-1],
        u_applied=u_applied,
        u_effective=u_effective,
        load_disturbance=load,
        output_disturbance=output_disturbance,
        measurement_noise=noise,
        disturbance_estimate=d_hat,
        solver_success=solver_success,
        solver_iterations=solver_iterations,
        solver_objective=solver_objective,
        solver_messages=tuple(solver_messages),
        solver_warnings=tuple(solver_warnings),
        minimum_constraint_residual=minimum_constraint_residual,
        maximum_predicted_violation=maximum_predicted_violation,
        maximum_slack=maximum_slack,
        prediction_snapshots=snapshots,
    )
    result.metrics = calculate_metrics(result, scenario)

    if verbose:
        print_result_summary(result, scenario)
    return result


def print_result_summary(result: Result, scenario: Scenario) -> None:
    """Print a compact, numerically meaningful scenario summary."""

    metrics = result.metrics
    assert metrics is not None
    model_gain, model_tau, model_dead_time = scenario.resolved_model_parameters()
    settling = (
        "not settled"
        if np.isnan(metrics.settling_time)
        else f"{metrics.settling_time:.1f} min"
    )
    print(f"\n{'=' * 72}")
    print(f"Scenario: {scenario.name}")
    print(f"{'=' * 72}")
    print(
        f"Plant K/tau/theta = {scenario.process_gain:g}/{scenario.process_tau:g}/"
        f"{scenario.process_dead_time:g}; model = {model_gain:g}/{model_tau:g}/"
        f"{model_dead_time:g}"
    )
    print(
        f"Horizons Np/Nc = {scenario.prediction_horizon}/{scenario.control_horizon}; "
        f"constraint mode = {scenario.output_constraint_mode}"
    )
    print(
        f"Final-window error = {metrics.steady_state_error:+.5f} "
        f"(std {metrics.steady_state_std:.5f}); settling = {settling}"
    )
    print(
        f"IAE = {metrics.integral_absolute_error:.3f}; overshoot = "
        f"{metrics.overshoot:.4f}; total |delta u| = "
        f"{metrics.total_input_movement:.3f}"
    )
    print(
        f"Max violations: output={metrics.maximum_output_constraint_violation:.3e}, "
        f"input={metrics.maximum_input_constraint_violation:.3e}, "
        f"move={metrics.maximum_move_constraint_violation:.3e}"
    )
    print(
        f"Solver failures = {metrics.solver_failures}; max slack = "
        f"{metrics.maximum_slack:.4f}; oscillation detected = "
        f"{metrics.oscillation_detected}"
    )


# =============================================================================
# 5. PLOTTING
# =============================================================================


def plot_result(
    result: Result, scenario: Scenario, title: Optional[str] = None
) -> plt.Figure:
    """Plot aligned plant, controller, disturbance and solver signals."""

    fig, axes = plt.subplots(4, 1, figsize=(11, 11), sharex=True)
    fig.suptitle(title or scenario.name, fontsize=14, fontweight="bold")

    ax = axes[0]
    ax.plot(result.time, result.setpoint, "k--", linewidth=1.4, label="Setpoint")
    ax.plot(result.time, result.y_true, color="tab:blue", linewidth=2, label="True output")
    ax.plot(
        result.time,
        result.y_measured,
        color="tab:cyan",
        linewidth=1,
        alpha=0.65,
        label="Measured output",
    )
    ax.plot(
        result.time,
        result.y_model,
        color="tab:purple",
        linestyle=":",
        linewidth=1.2,
        label="Internal model",
    )
    if np.isfinite(scenario.y_min):
        ax.axhline(scenario.y_min, color="tab:red", linestyle="--", alpha=0.7, label="Output limits")
    if np.isfinite(scenario.y_max):
        ax.axhline(scenario.y_max, color="tab:red", linestyle="--", alpha=0.7)
    for snapshot in result.prediction_snapshots.values():
        ax.plot(
            snapshot.future_time,
            snapshot.predicted_output,
            color="tab:orange",
            linestyle="--",
            linewidth=1,
            alpha=0.7,
        )
    ax.set_ylabel("Output")
    ax.legend(loc="best", ncol=2)
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    ax.step(result.u_time, result.u_applied, where="post", color="tab:red", label="Commanded input")
    if np.any(np.abs(result.load_disturbance) > 0.0):
        ax.step(
            result.u_time,
            result.u_effective,
            where="post",
            color="tab:orange",
            alpha=0.8,
            label="Input plus load disturbance",
        )
    if np.isfinite(scenario.u_min):
        ax.axhline(scenario.u_min, color="gray", linestyle=":", alpha=0.7)
    if np.isfinite(scenario.u_max):
        ax.axhline(scenario.u_max, color="gray", linestyle=":", alpha=0.7)
    ax.set_ylabel("Input")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)

    ax = axes[2]
    ax.plot(result.time, result.disturbance_estimate, color="tab:purple", label="Estimated output bias")
    ax.plot(result.time, result.output_disturbance, color="tab:green", label="True output disturbance")
    if scenario.measurement_noise_std > 0.0:
        ax.plot(result.time, result.measurement_noise, color="gray", alpha=0.45, label="Measurement noise")
    if np.any(np.abs(result.load_disturbance) > 0.0):
        ax.step(result.u_time, result.load_disturbance, where="post", color="tab:orange", label="Load disturbance")
    ax.axhline(0.0, color="black", linewidth=0.7, alpha=0.4)
    ax.set_ylabel("Disturbance")
    ax.legend(loc="best", ncol=2)
    ax.grid(True, alpha=0.3)

    ax = axes[3]
    ax.step(result.u_time, result.solver_iterations, where="post", color="tab:blue", label="Solver iterations")
    failures = np.flatnonzero(~result.solver_success)
    if failures.size:
        ax.scatter(
            result.u_time[failures],
            result.solver_iterations[failures],
            marker="x",
            color="tab:red",
            s=50,
            label="Solver failure",
        )
    if np.any(result.maximum_slack > 0.0):
        slack_axis = ax.twinx()
        slack_axis.plot(result.u_time, result.maximum_slack, color="tab:orange", alpha=0.7, label="Max slack")
        slack_axis.set_ylabel("Slack")
    ax.set_xlabel("Time [min]")
    ax.set_ylabel("Iterations")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)

    fig.tight_layout(rect=(0, 0, 1, 0.97))
    return fig


# =============================================================================
# 6. DEMONSTRATION SCENARIOS
# =============================================================================


def demonstration_scenarios() -> dict[str, Scenario]:
    """Return demonstrations covering the main controller capabilities."""

    baseline = Scenario(
        name="Baseline - exact model and setpoint changes",
        n_steps=75,
        process_gain=1.5,
        process_tau=8.0,
        process_dead_time=3.0,
        setpoint_sequence={0: 0.0, 5: 1.0, 42: 0.5},
        prediction_horizon=20,
        control_horizon=5,
        move_weight=0.08,
        delta_u_min=-0.8,
        delta_u_max=0.8,
        snapshot_steps=(5, 42),
    )

    mismatch = Scenario(
        name="Model mismatch - gain, time constant and fractional dead time",
        n_steps=110,
        process_gain=1.5,
        process_tau=8.0,
        process_dead_time=3.5,
        model_gain=1.0,
        model_tau=6.0,
        model_dead_time=2.0,
        setpoint_sequence={0: 0.0, 5: 1.0, 60: 1.4},
        prediction_horizon=22,
        control_horizon=5,
        move_weight=0.2,
        delta_u_min=-0.45,
        delta_u_max=0.45,
        disturbance_filter=0.18,
        snapshot_steps=(5, 60),
    )

    def output_step(k: int) -> float:
        return 0.0 if k < 30 else 0.4

    disturbance = Scenario(
        name="Output disturbance rejection with measurement noise",
        n_steps=95,
        process_gain=1.0,
        process_tau=6.0,
        process_dead_time=2.5,
        setpoint_sequence={0: 0.0, 5: 1.0},
        output_disturbance=output_step,
        measurement_noise_std=0.02,
        random_seed=7,
        prediction_horizon=20,
        control_horizon=5,
        move_weight=0.15,
        delta_u_min=-0.35,
        delta_u_max=0.35,
        disturbance_filter=0.15,
        snapshot_steps=(5, 30),
    )

    hard_constraint = Scenario(
        name="Hard output and move constraints",
        n_steps=90,
        process_gain=1.0,
        process_tau=5.0,
        process_dead_time=2.0,
        setpoint_sequence={0: 0.0, 5: 2.5, 55: 1.0},
        prediction_horizon=20,
        control_horizon=5,
        move_weight=0.08,
        u_min=0.0,
        u_max=5.0,
        delta_u_min=-0.55,
        delta_u_max=0.55,
        y_max=2.0,
        output_constraint_mode="hard",
        snapshot_steps=(5, 20, 55),
    )

    economic = Scenario(
        name="Economic MPC - maximise throughput inside a hard operating window",
        n_steps=75,
        y0=2.0,
        u0=2.0,
        process_gain=1.0,
        process_tau=5.0,
        process_dead_time=2.0,
        setpoint_sequence={0: 2.0},
        prediction_horizon=20,
        control_horizon=5,
        tracking_weight=0.15,
        move_weight=0.12,
        terminal_weight=0.5,
        economic_weight=0.45,
        economic_variable="input",
        u_min=0.0,
        u_max=5.0,
        delta_u_min=-0.3,
        delta_u_max=0.3,
        y_min=1.5,
        y_max=3.0,
        output_constraint_mode="hard",
        snapshot_steps=(0, 15),
    )

    return {
        "baseline": baseline,
        "mismatch": mismatch,
        "disturbance": disturbance,
        "constraint": hard_constraint,
        "economic": economic,
    }


# =============================================================================
# 7. AUTOMATED VALIDATION SUITE
# =============================================================================


@dataclass
class ValidationCase:
    scenario: Scenario
    checks: tuple[tuple[str, Callable[[Result], bool]], ...]


def _standard_checks(
    max_error: float = 0.03,
    maximum_failures: int = 0,
    require_no_oscillation: bool = True,
) -> tuple[tuple[str, Callable[[Result], bool]], ...]:
    checks: list[tuple[str, Callable[[Result], bool]]] = [
        (
            f"absolute final-window error <= {max_error:g}",
            lambda r, limit=max_error: abs(r.metrics.steady_state_error) <= limit,
        ),
        (
            f"solver failures <= {maximum_failures}",
            lambda r, limit=maximum_failures: r.metrics.solver_failures <= limit,
        ),
        (
            "input constraints respected",
            lambda r: r.metrics.maximum_input_constraint_violation <= 1e-6,
        ),
        (
            "move constraints respected",
            lambda r: r.metrics.maximum_move_constraint_violation <= 1e-6,
        ),
    ]
    if require_no_oscillation:
        checks.append(
            ("no sustained alternating oscillation", lambda r: not r.metrics.oscillation_detected)
        )
    return tuple(checks)


def validation_cases() -> tuple[ValidationCase, ...]:
    """Construct independent scenarios for correctness and robustness checks."""

    cases: list[ValidationCase] = []

    cases.append(
        ValidationCase(
            Scenario(
                name="Perfect model",
                n_steps=55,
                process_tau=5.0,
                process_dead_time=2.0,
                setpoint_sequence={0: 0.0, 4: 1.0},
                delta_u_min=-0.7,
                delta_u_max=0.7,
            ),
            _standard_checks(0.015)
            + (
                (
                    "internal model trace is time-aligned",
                    lambda r: np.max(np.abs(r.y_model - r.y_true)) <= 1e-10,
                ),
            ),
        )
    )

    cases.append(
        ValidationCase(
            Scenario(
                name="Combined gain, tau and dead-time mismatch",
                n_steps=100,
                process_gain=1.35,
                process_tau=7.0,
                process_dead_time=3.5,
                model_gain=1.0,
                model_tau=5.0,
                model_dead_time=2.0,
                setpoint_sequence={0: 0.0, 5: 1.0},
                prediction_horizon=22,
                control_horizon=5,
                move_weight=0.25,
                delta_u_min=-0.35,
                delta_u_max=0.35,
                disturbance_filter=0.15,
            ),
            _standard_checks(0.04),
        )
    )

    cases.append(
        ValidationCase(
            Scenario(
                name="Exactly zero dead time",
                n_steps=45,
                process_tau=4.0,
                process_dead_time=0.0,
                setpoint_sequence={0: 0.0, 3: 1.0},
                prediction_horizon=12,
                control_horizon=4,
                delta_u_min=-0.8,
                delta_u_max=0.8,
            ),
            _standard_checks(0.015),
        )
    )

    cases.append(
        ValidationCase(
            Scenario(
                name="Fractional dead time",
                n_steps=60,
                process_tau=5.0,
                process_dead_time=2.5,
                setpoint_sequence={0: 0.0, 4: 1.0},
                prediction_horizon=18,
                control_horizon=5,
                delta_u_min=-0.6,
                delta_u_max=0.6,
            ),
            _standard_checks(0.02),
        )
    )

    cases.append(
        ValidationCase(
            Scenario(
                name="Measurement noise",
                n_steps=85,
                process_tau=6.0,
                process_dead_time=2.0,
                setpoint_sequence={0: 0.0, 5: 1.0},
                measurement_noise_std=0.04,
                random_seed=19,
                move_weight=0.25,
                delta_u_min=-0.3,
                delta_u_max=0.3,
                disturbance_filter=0.10,
            ),
            _standard_checks(0.05),
        )
    )

    def persistent_output_disturbance(k: int) -> float:
        return 0.0 if k < 25 else 0.35

    cases.append(
        ValidationCase(
            Scenario(
                name="Constant output disturbance",
                n_steps=80,
                process_tau=6.0,
                process_dead_time=2.0,
                setpoint_sequence={0: 0.0, 4: 1.0},
                output_disturbance=persistent_output_disturbance,
                delta_u_min=-0.45,
                delta_u_max=0.45,
                disturbance_filter=0.22,
            ),
            _standard_checks(0.025),
        )
    )

    def load_pulse(k: int) -> float:
        return 0.35 if 25 <= k < 42 else 0.0

    cases.append(
        ValidationCase(
            Scenario(
                name="Dynamic load-disturbance pulse",
                n_steps=85,
                process_tau=6.0,
                process_dead_time=2.5,
                setpoint_sequence={0: 0.0, 4: 1.0},
                load_disturbance=load_pulse,
                move_weight=0.2,
                delta_u_min=-0.4,
                delta_u_max=0.4,
                disturbance_filter=0.16,
            ),
            _standard_checks(0.025),
        )
    )

    saturation_scenario = Scenario(
        name="Unreachable setpoint with actuator and rate limits",
        n_steps=65,
        process_tau=5.0,
        process_dead_time=1.0,
        setpoint_sequence={0: 0.0, 3: 1.0},
        u_min=0.0,
        u_max=0.6,
        delta_u_min=-0.12,
        delta_u_max=0.12,
    )
    saturation_checks = (
        ("settles at the reachable actuator limit", lambda r: abs(r.y_controlled[-1] - 0.6) <= 0.015),
        ("reports non-zero tracking offset", lambda r: abs(r.metrics.steady_state_error) >= 0.35),
        ("input constraints respected", lambda r: r.metrics.maximum_input_constraint_violation <= 1e-6),
        ("move constraints respected", lambda r: r.metrics.maximum_move_constraint_violation <= 1e-6),
        ("no solver failures", lambda r: r.metrics.solver_failures == 0),
    )
    cases.append(ValidationCase(saturation_scenario, saturation_checks))

    hard_scenario = Scenario(
        name="Hard output constraint",
        n_steps=70,
        process_tau=5.0,
        process_dead_time=2.0,
        setpoint_sequence={0: 0.0, 4: 2.0},
        prediction_horizon=20,
        control_horizon=5,
        u_min=0.0,
        u_max=4.0,
        delta_u_min=-0.45,
        delta_u_max=0.45,
        y_max=1.2,
        output_constraint_mode="hard",
    )
    hard_checks = (
        ("hard output limit respected", lambda r: r.metrics.maximum_output_constraint_violation <= 2e-5),
        ("operates close to the active limit", lambda r: abs(r.y_controlled[-1] - 1.2) <= 0.02),
        ("no solver failures", lambda r: r.metrics.solver_failures == 0),
        ("no sustained alternating oscillation", lambda r: not r.metrics.oscillation_detected),
    )
    cases.append(ValidationCase(hard_scenario, hard_checks))

    soft_scenario = Scenario(
        name="Initially infeasible soft output constraint",
        n_steps=50,
        process_tau=5.0,
        process_dead_time=1.0,
        setpoint_sequence={0: 1.5},
        prediction_horizon=15,
        control_horizon=4,
        u_min=0.0,
        u_max=0.25,
        delta_u_min=-0.08,
        delta_u_max=0.08,
        y_min=1.0,
        output_constraint_mode="soft",
    )
    soft_checks = (
        ("slack explicitly records infeasibility", lambda r: r.metrics.maximum_slack >= 0.70),
        ("soft problem remains solvable", lambda r: r.metrics.solver_failures == 0),
        ("input constraints respected", lambda r: r.metrics.maximum_input_constraint_violation <= 1e-6),
        ("move constraints respected", lambda r: r.metrics.maximum_move_constraint_violation <= 1e-6),
    )
    cases.append(ValidationCase(soft_scenario, soft_checks))

    cases.append(
        ValidationCase(
            Scenario(
                name="Negative process gain",
                n_steps=60,
                process_gain=-1.0,
                process_tau=5.0,
                process_dead_time=2.0,
                setpoint_sequence={0: 0.0, 4: 1.0},
                u_min=-3.0,
                u_max=3.0,
                delta_u_min=-0.5,
                delta_u_max=0.5,
            ),
            _standard_checks(0.02),
        )
    )

    return tuple(cases)


def structural_model_checks() -> tuple[tuple[str, bool], ...]:
    """Direct checks for the process discretisation and delay pipeline."""

    coefficient = 1.0 - math.exp(-1.0)

    zero_delay = FOPDTProcess(gain=1.0, tau=1.0, dead_time=0.0, dt=1.0)
    zero_delay.reset(y0=0.0, u0=0.0)
    zero_delay_response = zero_delay.step(1.0)

    fractional_delay = FOPDTProcess(
        gain=1.0, tau=1.0, dead_time=0.5, dt=1.0
    )
    fractional_delay.reset(y0=0.0, u0=0.0)
    fractional_response = fractional_delay.step(1.0)

    delayed = FOPDTProcess(gain=1.0, tau=3.0, dead_time=2.0, dt=1.0)
    delayed.reset(y0=0.0, u0=0.0)
    delayed.step(0.4)
    delayed.step(0.9)
    copied = delayed.clone()
    original_next = delayed.step(-0.2)
    copied_next = copied.step(-0.2)

    return (
        (
            "zero dead time responds in the current interval",
            bool(abs(zero_delay_response - coefficient) <= 1e-12),
        ),
        (
            "fractional dead time is interpolated",
            bool(abs(fractional_response - 0.5 * coefficient) <= 1e-12),
        ),
        (
            "cloned predictor preserves delayed-input history",
            bool(abs(original_next - copied_next) <= 1e-12),
        ),
    )


def run_self_tests(verbose: bool = True, raise_on_failure: bool = False) -> bool:
    """Run automated scenarios and return True only when every check passes."""

    all_passed = True
    if verbose:
        print("\nEnhanced MPC automated validation suite")
        print("=" * 72)

    for label, outcome in structural_model_checks():
        all_passed = all_passed and outcome
        if verbose:
            print(f"{'PASS' if outcome else 'FAIL':4s}  {label}")

    for case in validation_cases():
        result = run_simulation(case.scenario, verbose=False)
        outcomes = [(label, bool(check(result))) for label, check in case.checks]
        case_passed = all(outcome for _, outcome in outcomes)
        all_passed = all_passed and case_passed

        if verbose:
            status = "PASS" if case_passed else "FAIL"
            metrics = result.metrics
            print(
                f"{status:4s}  {case.scenario.name:<48s} "
                f"error={metrics.steady_state_error:+.4f} "
                f"failures={metrics.solver_failures}"
            )
            for label, outcome in outcomes:
                if not outcome:
                    print(f"      FAILED CHECK: {label}")

    if verbose:
        print("=" * 72)
        print("ALL VALIDATION CHECKS PASSED" if all_passed else "VALIDATION FAILED")

    if raise_on_failure and not all_passed:
        raise AssertionError("One or more enhanced MPC validation checks failed.")
    return all_passed


# =============================================================================
# 8. COMMAND-LINE ENTRY POINT
# =============================================================================


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="run the automated validation suite and return a non-zero exit code on failure",
    )
    parser.add_argument(
        "--no-plots",
        action="store_true",
        help="run demonstrations and print metrics without opening plot windows",
    )
    parser.add_argument(
        "--demo",
        choices=("all", *demonstration_scenarios().keys()),
        default="all",
        help="select one demonstration (default: all)",
    )
    parser.add_argument(
        "--save-plots",
        type=Path,
        default=None,
        metavar="DIRECTORY",
        help="save demonstration plots as PNG files",
    )
    args = parser.parse_args()

    if args.self_test:
        return 0 if run_self_tests(verbose=True) else 1

    scenarios = demonstration_scenarios()
    selected = scenarios if args.demo == "all" else {args.demo: scenarios[args.demo]}
    if args.save_plots is not None:
        args.save_plots.mkdir(parents=True, exist_ok=True)

    print("\nEnhanced SISO MPC demonstration suite")
    for key, scenario in selected.items():
        result = run_simulation(scenario, verbose=True)
        if not args.no_plots or args.save_plots is not None:
            figure = plot_result(result, scenario)
            if args.save_plots is not None:
                figure.savefig(args.save_plots / f"{key}.png", dpi=160)
            if not args.no_plots:
                plt.show()
            else:
                plt.close(figure)

    print("\nDemonstrations complete. Run with --self-test for automated checks.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
