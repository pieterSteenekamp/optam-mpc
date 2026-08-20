"""Model Predictive Controller implementation for OptAM-MPC.

This module provides the MPC controller that uses process models for
prediction and optimization to calculate optimal control moves.
"""

from __future__ import annotations

import time
import warnings
from dataclasses import dataclass, field
from typing import Optional, List, Tuple, Callable

import numpy as np
from scipy.optimize import minimize

from optam_mpc.core.models import Process, MIMOFOPDT
from optam_mpc.utils.validation import (
    vectorize,
    require_finite,
    maximum_bound_violation,
)


Array = np.ndarray


@dataclass
class MPCResult:
    """Result from a single MPC control calculation.

    Attributes
    ----------
    u : Array
        Optimal control action to apply.
    success : bool
        Whether the optimization succeeded.
    fallback_used : bool
        Whether the fallback (hold last input) was used.
    status : int
        Solver status code.
    message : str
        Solver message or error description.
    objective : float
        Objective function value at the solution.
    iterations : int
        Number of solver iterations.
    solve_time_seconds : float
        Time taken to solve the optimization problem.
    predicted_outputs : Array
        Predicted outputs over the prediction horizon.
    planned_inputs : Array
        Planned inputs over the prediction horizon.
    bias_estimate : Array
        Current bias correction estimate.
    """

    u: Array
    success: bool
    fallback_used: bool
    status: int
    message: str
    objective: float
    iterations: int
    solve_time_seconds: float
    predicted_outputs: Array
    planned_inputs: Array
    bias_estimate: Array


class MPCController:
    """Model Predictive Controller with nonlinear optimization.

    This controller uses a process model to predict future outputs and
    optimize future control moves subject to constraints. It supports
    both linear and nonlinear models through a common interface.

    Parameters
    ----------
    model : Process
        Process model used for prediction.
    config : dict or MPCConfig
        Controller configuration.

    Attributes
    ----------
    model : Process
        Internal copy of the process model used for prediction.
    ny : int
        Number of outputs.
    nu : int
        Number of inputs.
    prediction_horizon : int
        Number of future time steps to predict.
    control_horizon : int
        Number of future control moves to optimize.
    """

    def __init__(self, model: Process, config: dict) -> None:
        """Initialize the MPC controller.

        Parameters
        ----------
        model : Process
            Process model to use for prediction.
        config : dict
            Controller configuration dictionary with keys matching
            MPCConfig fields.
        """
        self.model = model.clone()
        self.config = self._validate_config(config)
        self.ny = model.ny
        self.nu = model.nu
        
        # Extract configuration values
        self.prediction_horizon = self.config["prediction_horizon"]
        self.control_horizon = self.config["control_horizon"]
        
        # Initialize state
        self._previous_u = np.zeros(self.nu, dtype=float)
        self._bias = np.zeros(self.ny, dtype=float)
        self._last_plan: Optional[Array] = None
        self._initialized = False
        
        # Resolve vector parameters
        self._resolve_vector_parameters()

    def _validate_config(self, config: dict) -> dict:
        """Validate and complete the configuration dictionary.

        Parameters
        ----------
        config : dict
            User-provided configuration.

        Returns
        -------
        dict
            Complete configuration with defaults filled in.
        """
        # Default configuration
        defaults = {
            "prediction_horizon": 15,
            "control_horizon": 5,
            "output_weights": None,
            "move_weights": None,
            "terminal_weights": None,
            "output_scale": None,
            "input_scale": None,
            "input_min": None,
            "input_max": None,
            "move_min": None,
            "move_max": None,
            "output_min": None,
            "output_max": None,
            "soft_output_weights": None,
            "maximum_output_slack": np.inf,
            "bias_filter": 0.25,
            "solver_max_iterations": 200,
            "solver_tolerance": 1e-8,
            "constraint_tolerance": 1e-6,
            "raise_on_failure": False,
        }
        
        # Update with user-provided values
        validated = {**defaults, **config}
        
        # Basic validation
        if validated["prediction_horizon"] < 1:
            raise ValueError("prediction_horizon must be at least 1.")
        if not 1 <= validated["control_horizon"] <= validated["prediction_horizon"]:
            raise ValueError(
                "control_horizon must be between 1 and prediction_horizon."
            )
        if not 0.0 <= validated["bias_filter"] <= 1.0:
            raise ValueError("bias_filter must be between 0 and 1.")
        
        return validated

    def _resolve_vector_parameters(self) -> None:
        """Convert vector parameters to numpy arrays with correct shape."""
        self.output_weights = vectorize(
            self.config["output_weights"], self.ny, "output_weights", 1.0
        )
        self.move_weights = vectorize(
            self.config["move_weights"], self.nu, "move_weights", 0.1
        )
        self.terminal_weights = vectorize(
            self.config["terminal_weights"], self.ny, "terminal_weights", 0.0
        )
        self.output_scale = vectorize(
            self.config["output_scale"], self.ny, "output_scale", 1.0
        )
        self.input_scale = vectorize(
            self.config["input_scale"], self.nu, "input_scale", 1.0
        )
        self.input_min = vectorize(
            self.config["input_min"], self.nu, "input_min", -np.inf
        )
        self.input_max = vectorize(
            self.config["input_max"], self.nu, "input_max", np.inf
        )
        self.move_min = vectorize(
            self.config["move_min"], self.nu, "move_min", -np.inf
        )
        self.move_max = vectorize(
            self.config["move_max"], self.nu, "move_max", np.inf
        )
        self.output_min = vectorize(
            self.config["output_min"], self.ny, "output_min", -np.inf
        )
        self.output_max = vectorize(
            self.config["output_max"], self.ny, "output_max", np.inf
        )
        
        # Validate weights
        for values, name in [
            (self.output_weights, "output_weights"),
            (self.move_weights, "move_weights"),
            (self.terminal_weights, "terminal_weights"),
        ]:
            require_finite(values, name)
            if np.any(values < 0.0):
                raise ValueError(f"{name} must be non-negative.")
        
        # Validate scales
        for values, name in [
            (self.output_scale, "output_scale"),
            (self.input_scale, "input_scale"),
        ]:
            require_finite(values, name)
            if np.any(values <= 0.0):
                raise ValueError(f"{name} must be positive.")
        
        # Validate bounds
        for lower, upper, name in [
            (self.input_min, self.input_max, "input"),
            (self.move_min, self.move_max, "move"),
            (self.output_min, self.output_max, "output"),
        ]:
            if np.any(lower > upper):
                raise ValueError(f"Every {name} lower bound must be <= its upper bound.")
        
        # Validate move limits allow zero move
        if np.any(self.move_min > 0.0) or np.any(self.move_max < 0.0):
            raise ValueError(
                "Move limits must permit a zero move so hold-last-input is feasible."
            )
        
        # Handle soft constraints
        self.soft_output_weights = None
        if self.config["soft_output_weights"] is not None:
            self.soft_output_weights = vectorize(
                self.config["soft_output_weights"],
                self.ny,
                "soft_output_weights",
            )
            require_finite(self.soft_output_weights, "soft_output_weights")
            if np.any(self.soft_output_weights < 0.0):
                raise ValueError("soft_output_weights must be non-negative.")
        
        # Handle maximum output slack
        self.maximum_output_slack = vectorize(
            self.config["maximum_output_slack"],
            self.ny,
            "maximum_output_slack",
        )
        if np.any(self.maximum_output_slack <= 0.0):
            raise ValueError("maximum_output_slack must be positive.")
        
        # Identify constrained outputs
        constrained = np.isfinite(self.output_min) | np.isfinite(self.output_max)
        self._constrained_outputs = np.flatnonzero(constrained)
        
        # Check that soft weights are positive for constrained outputs
        if self.soft_output_weights is not None and self._constrained_outputs.size > 0:
            if np.any(self.soft_output_weights[self._constrained_outputs] <= 0.0):
                raise ValueError(
                    "Soft-output weights must be positive for every constrained output."
                )
        
        self._soft_constraints = (
            self.soft_output_weights is not None
            and self._constrained_outputs.size > 0
        )
        self._n_slack = (
            self._constrained_outputs.size if self._soft_constraints else 0
        )

    def reset(self, y0: Array, u0: Array) -> None:
        """Reset the controller to an initial state.

        Parameters
        ----------
        y0 : array-like
            Initial output values.
        u0 : array-like
            Initial input values.
        """
        output = vectorize(y0, self.ny, "y0")
        input_value = vectorize(u0, self.nu, "u0")
        require_finite(output, "y0")
        require_finite(input_value, "u0")
        
        # Check input bounds
        violation = maximum_bound_violation(
            input_value[np.newaxis, :], self.input_min, self.input_max
        )
        if violation > self.config["constraint_tolerance"]:
            raise ValueError("u0 violates the configured absolute input bounds.")
        
        self.model.reset(output, input_value)
        self._previous_u = np.clip(input_value, self.input_min, self.input_max)
        self._bias = np.zeros(self.ny, dtype=float)
        self._last_plan = None
        self._initialized = True

    def _reference_horizon(self, reference) -> Array:
        """Convert reference to a full prediction horizon matrix.

        Parameters
        ----------
        reference : array-like
            Reference trajectory. Can be:
            - Shape (ny,): Constant reference
            - Shape (steps, ny): Time-varying reference

        Returns
        -------
        Array
            Reference matrix with shape (prediction_horizon, ny).
        """
        values = np.asarray(reference, dtype=float)
        
        if values.shape == (self.ny,):
            horizon = np.tile(values, (self.prediction_horizon, 1))
        elif values.ndim == 2 and values.shape[1] == self.ny and values.shape[0] >= 1:
            if values.shape[0] >= self.prediction_horizon:
                horizon = values[: self.prediction_horizon, :].copy()
            else:
                tail = np.tile(
                    values[-1, :], (self.prediction_horizon - values.shape[0], 1)
                )
                horizon = np.vstack((values, tail))
        else:
            raise ValueError(
                f"reference must have shape ({self.ny},) or (steps, {self.ny})."
            )
        
        require_finite(horizon, "reference")
        return horizon

    def _project_plan(self, proposed_plan: Array) -> Array:
        """Project an input plan onto input and move constraints.

        Parameters
        ----------
        proposed_plan : Array
            Proposed input plan with shape (control_horizon, nu).

        Returns
        -------
        Array
            Projected input plan satisfying all input and move constraints.
        """
        plan = np.asarray(proposed_plan, dtype=float)
        if plan.shape != (self.control_horizon, self.nu):
            raise ValueError("The proposed input plan has an invalid shape.")
        
        projected = np.empty_like(plan)
        previous = self._previous_u.copy()
        
        for step in range(self.control_horizon):
            lower = np.maximum(self.input_min, previous + self.move_min)
            upper = np.minimum(self.input_max, previous + self.move_max)
            projected[step, :] = np.clip(plan[step, :], lower, upper)
            previous = projected[step, :]
        
        return projected

    def _initial_input_guess(self) -> Array:
        """Generate an initial guess for the optimizer.

        Returns
        -------
        Array
            Initial input plan with shape (control_horizon, nu).
        """
        if self._last_plan is None:
            guess = np.tile(self._previous_u, (self.control_horizon, 1))
        elif self.control_horizon == 1:
            guess = self._last_plan.copy()
        else:
            guess = np.vstack((
                self._last_plan[1:, :],
                self._last_plan[-1, :],
            ))
        
        return self._project_plan(guess)

    def _expand_plan(self, plan: Array) -> Array:
        """Expand control horizon plan to full prediction horizon.

        Parameters
        ----------
        plan : Array
            Input plan with shape (control_horizon, nu).

        Returns
        -------
        Array
            Expanded plan with shape (prediction_horizon, nu).
        """
        if self.control_horizon == self.prediction_horizon:
            return plan.copy()
        tail = np.tile(plan[-1, :], (self.prediction_horizon - self.control_horizon, 1))
        return np.vstack((plan, tail))

    def _predict(self, plan: Array) -> Tuple[Array, Array]:
        """Predict future outputs for a given input plan.

        Parameters
        ----------
        plan : Array
            Input plan with shape (control_horizon, nu).

        Returns
        -------
        Tuple[Array, Array]
            Predicted outputs and expanded input plan.
        """
        expanded_plan = self._expand_plan(plan)
        predictor = self.model.clone()
        predicted_output = np.empty((self.prediction_horizon, self.ny), dtype=float)
        
        for step in range(self.prediction_horizon):
            predicted_output[step, :] = (
                predictor.step(expanded_plan[step, :]) + self._bias
            )
        
        if not np.all(np.isfinite(predicted_output)):
            raise FloatingPointError("The model prediction became non-finite.")
        
        return predicted_output, expanded_plan

    def _required_slack(self, predicted_output: Array) -> Array:
        """Calculate required slack for soft output constraints.

        Parameters
        ----------
        predicted_output : Array
            Predicted outputs with shape (prediction_horizon, ny).

        Returns
        -------
        Array
            Required slack values for each constrained output.
        """
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

    def _split_decision(self, decision: Array) -> Tuple[Array, Array]:
        """Split optimization decision vector into plan and slack.

        Parameters
        ----------
        decision : Array
            Full decision vector.

        Returns
        -------
        Tuple[Array, Array]
            Input plan and slack variables.
        """
        input_size = self.control_horizon * self.nu
        plan = decision[:input_size].reshape(self.control_horizon, self.nu)
        
        if self._soft_constraints:
            slack = decision[input_size:]
        else:
            slack = np.empty(0, dtype=float)
        
        return plan, slack

    def control(self, y_measured: Array, reference: Array) -> MPCResult:
        """Calculate one receding-horizon control action.

        Parameters
        ----------
        y_measured : array-like
            Current measured output values.
        reference : array-like
            Desired output values or trajectory.

        Returns
        -------
        MPCResult
            Control result with optimal action and diagnostics.
        """
        measurement = vectorize(y_measured, self.ny, "y_measured")
        require_finite(measurement, "y_measured")
        
        if not self._initialized:
            raise RuntimeError("Call reset(y0, u0) before requesting a control move.")
        
        reference_horizon = self._reference_horizon(reference)
        
        # Update bias correction
        raw_bias = measurement - self.model.y
        alpha = self.config["bias_filter"]
        self._bias = alpha * raw_bias + (1.0 - alpha) * self._bias
        
        # Generate initial guess
        input_guess = self._initial_input_guess()
        
        # Estimate initial slack
        try:
            initial_prediction, _ = self._predict(input_guess)
            slack_guess = self._required_slack(initial_prediction)
        except (FloatingPointError, ValueError, OverflowError):
            slack_guess = np.zeros(self._constrained_outputs.size, dtype=float)
        
        # Prepare optimization problem
        if self._soft_constraints:
            slack_maximum = self.maximum_output_slack[self._constrained_outputs]
            slack_guess = np.minimum(
                slack_guess + self.config["constraint_tolerance"],
                slack_maximum,
            )
            initial_decision = np.concatenate((
                input_guess.ravel(),
                slack_guess.ravel(),
            ))
        else:
            initial_decision = input_guess.ravel()
        
        # Define bounds
        bounds = [
            (self.input_min[i], self.input_max[i])
            for _ in range(self.control_horizon)
            for i in range(self.nu)
        ]
        if self._soft_constraints:
            bounds.extend(
                (0.0, self.maximum_output_slack[i])
                for i in self._constrained_outputs
            )
        
        # Setup caching for efficient evaluation
        cache_decision: Optional[Array] = None
        cache_prediction: Optional[Array] = None
        cache_expanded_plan: Optional[Array] = None
        
        def evaluate(decision: Array) -> Tuple[Array, Array, Array]:
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
        
        # Define objective function
        def objective(decision: Array) -> float:
            try:
                predicted, _, slack = evaluate(decision)
                
                # Calculate tracking cost
                scaled_error = (
                    predicted - reference_horizon
                ) / self.output_scale[np.newaxis, :]
                tracking_cost = np.sum(
                    scaled_error**2 * self.output_weights[np.newaxis, :]
                )
                
                # Calculate terminal cost
                terminal_cost = np.sum(
                    scaled_error[-1, :] ** 2 * self.terminal_weights
                )
                
                # Calculate move cost
                plan, _ = self._split_decision(decision)
                input_sequence = np.vstack((self._previous_u, plan))
                scaled_moves = np.diff(input_sequence, axis=0) / self.input_scale
                move_cost = np.sum(
                    scaled_moves**2 * self.move_weights[np.newaxis, :]
                )
                
                # Calculate slack cost
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
        
        # Define constraints
        constraints = []
        
        # Rate constraints
        rate_inequality_count = self.control_horizon * (
            int(np.sum(np.isfinite(self.move_min)))
            + int(np.sum(np.isfinite(self.move_max)))
        )
        
        if rate_inequality_count:
            def rate_constraints(decision: Array) -> Array:
                plan, _ = self._split_decision(decision)
                moves = np.diff(np.vstack((self._previous_u, plan)), axis=0)
                residuals = []
                for i in range(self.nu):
                    if np.isfinite(self.move_min[i]):
                        residuals.append(moves[:, i] - self.move_min[i])
                    if np.isfinite(self.move_max[i]):
                        residuals.append(self.move_max[i] - moves[:, i])
                return np.concatenate(residuals)
            
            constraints.append({"type": "ineq", "fun": rate_constraints})
        
        # Output constraints
        output_inequalities_per_step = 0
        for output_index in self._constrained_outputs:
            output_inequalities_per_step += int(
                np.isfinite(self.output_min[output_index])
            )
            output_inequalities_per_step += int(
                np.isfinite(self.output_max[output_index])
            )
        
        if output_inequalities_per_step > 0:
            def output_constraints(decision: Array) -> Array:
                try:
                    predicted, _, slack = evaluate(decision)
                except (FloatingPointError, ValueError, OverflowError):
                    return np.full(
                        self.prediction_horizon * output_inequalities_per_step,
                        -1.0e12,
                    )
                
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
        
        # Solve optimization problem
        caught_warnings = []
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
                        "maxiter": self.config["solver_max_iterations"],
                        "ftol": self.config["solver_tolerance"],
                        "disp": False,
                    },
                )
                caught_warnings = [
                    str(record.message) for record in warning_records
                ]
        except Exception as exc:
            optimizer_exception = f"{type(exc).__name__}: {exc}"
        
        solve_time = time.perf_counter() - start
        
        # Process optimization result
        candidate_ok = optimizer_result is not None and bool(optimizer_result.success)
        candidate_decision = None
        
        if candidate_ok:
            candidate_decision = np.asarray(optimizer_result.x, dtype=float)
            expected_size = self.control_horizon * self.nu + self._n_slack
            candidate_ok = (
                candidate_decision.shape == (expected_size,)
                and np.all(np.isfinite(candidate_decision))
            )
        
        # Validate constraints
        minimum_margin = np.inf
        if candidate_ok:
            try:
                margins = []
                for constraint in constraints:
                    margins.extend(
                        np.asarray(constraint["fun"](candidate_decision)).ravel()
                    )
                minimum_margin = float(min(margins)) if margins else np.inf
                candidate_ok = minimum_margin >= -self.config["constraint_tolerance"]
            except (FloatingPointError, ValueError, OverflowError):
                candidate_ok = False
        
        # Final projection and validation
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
                candidate_ok = projected_margin >= -self.config["constraint_tolerance"]
            except (FloatingPointError, ValueError, OverflowError):
                candidate_ok = False
        
        # Apply result or fallback
        if candidate_ok:
            predicted_output, expanded_plan = self._predict(plan)
            u_command = plan[0, :].copy()
            self._last_plan = plan.copy()
            maximum_slack = float(np.max(slack)) if slack.size else 0.0
        else:
            # Fallback: hold last input
            plan = np.tile(self._previous_u, (self.control_horizon, 1))
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
                predicted_output = np.full(
                    (self.prediction_horizon, self.ny), np.nan
                )
                maximum_slack = np.nan
        
        # Prepare result message
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
            base_message = (
                f"{base_message} Warnings: {' | '.join(caught_warnings)}"
            ).strip()
        if not candidate_ok:
            base_message = f"{base_message} Hold-last-input fallback used.".strip()
        
        # Update internal model
        self.model.step(u_command)
        self._previous_u = u_command.copy()
        
        # Create result
        result = MPCResult(
            u=u_command,
            success=bool(candidate_ok),
            fallback_used=not bool(candidate_ok),
            status=status,
            message=base_message,
            objective=objective_value,
            iterations=iterations,
            solve_time_seconds=solve_time,
            predicted_outputs=predicted_output.copy(),
            planned_inputs=expanded_plan.copy(),
            bias_estimate=self._bias.copy(),
        )
        
        if result.fallback_used and self.config["raise_on_failure"]:
            raise RuntimeError(result.message)
        
        return result
    
    # Alias for backward compatibility
    step = control
