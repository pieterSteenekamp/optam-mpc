"""Application-independent nonlinear model predictive control engine.

The engine has no knowledge of vessels, temperatures, flows, engineering
units, or a particular process model.  An application supplies a discrete
nonlinear model and declarative CV/MV configuration.  The model dimensions
therefore determine the number of states, controlled outputs, manipulated
variables, and measured disturbances.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, replace
import math
import time
from typing import Any, Protocol

import casadi as ca

from .prediction import (
    NonlinearPredictionForm,
    evaluate_transition,
    nonlinear_velocity_step_values,
)


class NMPCOptimizationError(RuntimeError):
    """Raised when the nonlinear optimiser cannot produce a valid move."""


class DiscreteNonlinearModel(Protocol):
    """Interface that connects an application model to the generic NMPC.

    Both methods must accept ordinary numbers and CasADi symbolic values.
    ``transition_values`` returns the complete state at the next sample.
    ``output_values`` maps a state to the outputs available for CV control.
    """

    state_dimension: int
    manipulated_input_dimension: int
    disturbance_dimension: int
    output_dimension: int

    def transition_values(
        self,
        state_values: Sequence[Any],
        input_values: Sequence[Any],
        disturbance_values: Sequence[Any],
    ) -> Sequence[Any]:
        """Return the state one sample later."""

    def output_values(
        self,
        state_values: Sequence[Any],
        input_values: Sequence[Any],
        disturbance_values: Sequence[Any],
    ) -> Sequence[Any]:
        """Return the model outputs used by configured CVs."""


@dataclass(frozen=True)
class ControlledVariableConfiguration:
    """Target or zone objective for one model output."""

    name: str
    output_index: int
    weight: float
    normalization: float
    target: float | None = None
    zone_low: float | None = None
    zone_high: float | None = None
    constraint_low: float | None = None
    constraint_high: float | None = None
    constraint_weight: float | None = None
    constraint_normalization: float | None = None
    rate_weight: float | None = None
    rate_normalization: float | None = None

    @classmethod
    def for_target(
        cls,
        name: str,
        output_index: int,
        target: float,
        weight: float,
        normalization: float,
        constraint_low: float | None = None,
        constraint_high: float | None = None,
        constraint_weight: float | None = None,
        constraint_normalization: float | None = None,
        rate_weight: float | None = None,
        rate_normalization: float | None = None,
    ) -> ControlledVariableConfiguration:
        """Create a CV that is driven to one target value."""

        return cls(
            name=name,
            output_index=output_index,
            target=target,
            weight=weight,
            normalization=normalization,
            constraint_low=constraint_low,
            constraint_high=constraint_high,
            constraint_weight=constraint_weight,
            constraint_normalization=constraint_normalization,
            rate_weight=rate_weight,
            rate_normalization=rate_normalization,
        )

    @classmethod
    def for_zone(
        cls,
        name: str,
        output_index: int,
        zone_low: float,
        zone_high: float,
        weight: float,
        normalization: float,
        constraint_low: float | None = None,
        constraint_high: float | None = None,
        constraint_weight: float | None = None,
        constraint_normalization: float | None = None,
        rate_weight: float | None = None,
        rate_normalization: float | None = None,
    ) -> ControlledVariableConfiguration:
        """Create a CV that is penalised only outside a permitted zone."""

        return cls(
            name=name,
            output_index=output_index,
            zone_low=zone_low,
            zone_high=zone_high,
            weight=weight,
            normalization=normalization,
            constraint_low=constraint_low,
            constraint_high=constraint_high,
            constraint_weight=constraint_weight,
            constraint_normalization=constraint_normalization,
            rate_weight=rate_weight,
            rate_normalization=rate_normalization,
        )


@dataclass(frozen=True)
class ManipulatedVariableConfiguration:
    """Limits and move suppression for one manipulated variable."""

    name: str
    lower_bound: float
    upper_bound: float
    maximum_change_per_sample: float
    move_weight: float
    move_normalization: float
    maximum_increase_per_sample: float | None = None
    maximum_decrease_per_sample: float | None = None

    @property
    def increase_limit(self) -> float:
        """Return the permitted positive move for one control sample."""

        return (
            self.maximum_change_per_sample
            if self.maximum_increase_per_sample is None
            else self.maximum_increase_per_sample
        )

    @property
    def decrease_limit(self) -> float:
        """Return the magnitude of the permitted negative move."""

        return (
            self.maximum_change_per_sample
            if self.maximum_decrease_per_sample is None
            else self.maximum_decrease_per_sample
        )


@dataclass(frozen=True)
class ManipulatedMoveCombinationConfiguration:
    """Penalty on any declared linear combination of simultaneous MV moves."""

    name: str
    coefficients: tuple[float, ...]
    weight: float
    normalization: float


@dataclass(frozen=True)
class GenericNMPCConfiguration:
    """Horizon, CV objectives, MV constraints, and solver limit."""

    prediction_horizon_samples: int
    control_horizon_samples: int
    controlled_variables: tuple[ControlledVariableConfiguration, ...]
    manipulated_variables: tuple[ManipulatedVariableConfiguration, ...]
    move_combinations: tuple[ManipulatedMoveCombinationConfiguration, ...] = ()
    maximum_solver_iterations: int = 300
    maximum_solver_cpu_seconds: float | None = None


@dataclass(frozen=True)
class GenericNMPCMoveResult:
    """One successful generic nonlinear optimisation."""

    success: bool
    solver_status: str
    solver_iterations: int
    objective_value: float
    calculation_time_seconds: float
    solver_time_seconds: float
    first_input: tuple[float, ...]
    optimized_inputs: tuple[tuple[float, ...], ...]
    predicted_states: tuple[tuple[float, ...], ...]
    predicted_outputs: tuple[tuple[float, ...], ...]


class GenericNonlinearMPC:
    """Dimension-independent direct single-shooting nonlinear MPC."""

    def __init__(
        self,
        model: DiscreteNonlinearModel,
        configuration: GenericNMPCConfiguration,
        prediction_form: NonlinearPredictionForm | str = (
            NonlinearPredictionForm.ABSOLUTE
        ),
    ) -> None:
        self.model = model
        self.configuration = configuration
        try:
            self.prediction_form = NonlinearPredictionForm(prediction_form)
        except ValueError as exc:
            choices = ", ".join(item.value for item in NonlinearPredictionForm)
            raise ValueError(
                f"prediction_form must be one of: {choices}"
            ) from exc

        self.state_dimension = self._read_dimension(
            "state_dimension", minimum=1
        )
        self.manipulated_input_dimension = self._read_dimension(
            "manipulated_input_dimension", minimum=1
        )
        self.disturbance_dimension = self._read_dimension(
            "disturbance_dimension", minimum=0
        )
        self.output_dimension = self._read_dimension(
            "output_dimension", minimum=1
        )
        self.controlled_variable_dimension = len(
            configuration.controlled_variables
        )

        self._last_solution: list[float] | None = None
        self._previous_state: tuple[float, ...] | None = None
        self._previous_disturbances: tuple[float, ...] | None = None
        self._validate_configuration()
        self._build_problem()

    def calculate_move(
        self,
        state_values: Sequence[float],
        disturbance_values: Sequence[float],
        previous_input_values: Sequence[float],
    ) -> GenericNMPCMoveResult:
        """Calculate the next MV vector from the current measurements."""

        calculation_started = time.perf_counter()
        state = self._validate_runtime_vector(
            "state", state_values, self.state_dimension
        )
        disturbances = self._validate_runtime_vector(
            "disturbances",
            disturbance_values,
            self.disturbance_dimension,
        )
        previous_input = self._validate_runtime_vector(
            "previous inputs",
            previous_input_values,
            self.manipulated_input_dimension,
        )
        self._validate_previous_inputs(previous_input)

        parameter_values = [*state, *disturbances, *previous_input]
        if self.prediction_form is NonlinearPredictionForm.VELOCITY:
            history_available = (
                self._previous_state is not None
                and self._previous_disturbances is not None
            )
            previous_state = self._previous_state or state
            previous_disturbances = (
                self._previous_disturbances or disturbances
            )
            parameter_values.extend(
                (*previous_state, *previous_disturbances, float(history_available))
            )
        parameter_values.extend(self._runtime_tuning_values())

        parameters = ca.DM(parameter_values)
        initial_guess = self._make_initial_guess(previous_input)
        solver_started = time.perf_counter()
        try:
            solution = self._solver(
                x0=initial_guess,
                p=parameters,
                lbx=self._decision_lower_bounds,
                ubx=self._decision_upper_bounds,
                lbg=self._rate_lower_bounds,
                ubg=self._rate_upper_bounds,
            )
        except RuntimeError as exc:
            raise NMPCOptimizationError(
                f"nonlinear optimisation failed: {exc}"
            ) from exc
        solver_time_seconds = time.perf_counter() - solver_started

        statistics = self._solver.stats()
        status = str(statistics.get("return_status", "unknown"))
        if not bool(statistics.get("success", False)):
            raise NMPCOptimizationError(
                f"nonlinear optimisation was not accepted: {status}"
            )

        flat_solution = [
            float(value) for value in solution["x"].full().ravel()
        ]
        self._last_solution = flat_solution
        input_count = self.manipulated_input_dimension
        control_horizon = self.configuration.control_horizon_samples
        optimized_inputs = tuple(
            tuple(
                flat_solution[input_count * move_index + input_index]
                for input_index in range(input_count)
            )
            for move_index in range(control_horizon)
        )

        trajectory_matrix, output_matrix = self._trajectory_function(
            parameters, solution["x"]
        )
        prediction_points = (
            self.configuration.prediction_horizon_samples + 1
        )
        predicted_states = tuple(
            tuple(
                float(trajectory_matrix[state_index, point_index])
                for state_index in range(self.state_dimension)
            )
            for point_index in range(prediction_points)
        )
        predicted_outputs = tuple(
            tuple(
                float(output_matrix[output_index, point_index])
                for output_index in range(self.output_dimension)
            )
            for point_index in range(prediction_points)
        )

        self._previous_state = state
        self._previous_disturbances = disturbances
        calculation_time_seconds = time.perf_counter() - calculation_started
        return GenericNMPCMoveResult(
            success=True,
            solver_status=status,
            solver_iterations=int(statistics.get("iter_count", 0)),
            objective_value=float(solution["f"]),
            calculation_time_seconds=calculation_time_seconds,
            solver_time_seconds=solver_time_seconds,
            first_input=optimized_inputs[0],
            optimized_inputs=optimized_inputs,
            predicted_states=predicted_states,
            predicted_outputs=predicted_outputs,
        )

    def reset_warm_start(self) -> None:
        """Forget the previous optimum before an unrelated run."""

        self._last_solution = None

    def reset_measurement_history(self) -> None:
        """Forget the measurement pair used by velocity-form prediction."""

        self._previous_state = None
        self._previous_disturbances = None

    def update_tuning(
        self,
        *,
        cv_weights: dict[int, float] | None = None,
        cv_rate_weights: dict[int, float] | None = None,
        cv_targets: dict[int, float] | None = None,
        mv_move_weights: dict[int, float] | None = None,
        move_combination_weights: dict[int, float] | None = None,
    ) -> None:
        """Apply objective tuning without rebuilding the nonlinear program."""

        cv_weights = cv_weights or {}
        cv_rate_weights = cv_rate_weights or {}
        cv_targets = cv_targets or {}
        mv_move_weights = mv_move_weights or {}
        move_combination_weights = move_combination_weights or {}
        for index in cv_targets:
            if not 0 <= index < len(self.configuration.controlled_variables):
                raise ValueError("Target update has invalid CV index.")
            if self.configuration.controlled_variables[index].target is None:
                raise ValueError("Target updates require target-based CVs, not zones.")
        cvs = tuple(
            replace(
                variable,
                weight=cv_weights.get(index, variable.weight),
                rate_weight=cv_rate_weights.get(index, variable.rate_weight),
                target=cv_targets.get(index, variable.target),
            )
            for index, variable in enumerate(self.configuration.controlled_variables)
        )
        mvs = tuple(
            replace(variable, move_weight=mv_move_weights.get(index, variable.move_weight))
            for index, variable in enumerate(self.configuration.manipulated_variables)
        )
        combinations = tuple(
            replace(variable, weight=move_combination_weights.get(index, variable.weight))
            for index, variable in enumerate(self.configuration.move_combinations)
        )
        for updates, size, label in (
            (cv_weights, len(cvs), "CV weight"),
            (cv_rate_weights, len(cvs), "CV rate weight"),
            (mv_move_weights, len(mvs), "MV move weight"),
            (move_combination_weights, len(combinations), "move-combination weight"),
        ):
            if any(not 0 <= index < size for index in updates):
                raise ValueError(f"{label} update has an invalid index.")
        previous_configuration = self.configuration
        updated_configuration = replace(
            self.configuration,
            controlled_variables=cvs,
            manipulated_variables=mvs,
            move_combinations=combinations,
        )
        self.configuration = updated_configuration
        try:
            self._validate_configuration()
        except Exception:
            self.configuration = previous_configuration
            raise

    def _runtime_tuning_values(self) -> list[float]:
        """Flatten all online objective values in the NLP parameter order."""

        values: list[float] = []
        for variable in self.configuration.controlled_variables:
            values.extend((
                float(variable.weight),
                0.0 if variable.rate_weight is None else float(variable.rate_weight),
                0.0 if variable.target is None else float(variable.target),
            ))
        values.extend(float(item.move_weight) for item in self.configuration.manipulated_variables)
        values.extend(float(item.weight) for item in self.configuration.move_combinations)
        return values

    def _make_initial_guess(
        self, previous_input: tuple[float, ...]
    ) -> list[float]:
        """Shift the prior optimum and project it onto all MV limits."""

        horizon = self.configuration.control_horizon_samples
        input_count = self.manipulated_input_dimension
        if self._last_solution is None:
            candidates = [previous_input] * horizon
        else:
            control_value_count = input_count * horizon
            old = self._last_solution[:control_value_count]
            candidates = [
                tuple(
                    old[input_count * move_index + input_index]
                    for input_index in range(input_count)
                )
                for move_index in range(1, horizon)
            ]
            candidates.append(tuple(old[-input_count:]))

        current = list(previous_input)
        guess: list[float] = []
        for candidate in candidates:
            for input_index, variable in enumerate(
                self.configuration.manipulated_variables
            ):
                projected = min(
                    current[input_index] + variable.increase_limit,
                    max(
                        current[input_index] - variable.decrease_limit,
                        candidate[input_index],
                    ),
                )
                current[input_index] = min(
                    variable.upper_bound,
                    max(variable.lower_bound, projected),
                )
            guess.extend(current)
        guess.extend(0.0 for _ in range(self._soft_slack_count))
        return guess

    def _build_problem(self) -> None:
        nx = self.state_dimension
        nu = self.manipulated_input_dimension
        nd = self.disturbance_dimension
        ny = self.output_dimension
        config = self.configuration
        prediction_horizon = config.prediction_horizon_samples
        control_horizon = config.control_horizon_samples

        symbolic_state = ca.MX.sym("state", nx)
        symbolic_input = ca.MX.sym("input", nu)
        symbolic_disturbance = ca.MX.sym("disturbance", nd)
        symbolic_state_values = tuple(
            symbolic_state[index] for index in range(nx)
        )
        symbolic_input_values = tuple(
            symbolic_input[index] for index in range(nu)
        )
        symbolic_disturbance_values = tuple(
            symbolic_disturbance[index] for index in range(nd)
        )

        next_values = tuple(
            self.model.transition_values(
                symbolic_state_values,
                symbolic_input_values,
                symbolic_disturbance_values,
            )
        )
        if len(next_values) != nx:
            raise ValueError(
                "model transition must return one value per declared state"
            )
        output_values = tuple(
            self.model.output_values(
                symbolic_state_values,
                symbolic_input_values,
                symbolic_disturbance_values,
            )
        )
        if len(output_values) != ny:
            raise ValueError(
                "model output function must return the declared output dimension"
            )

        step_function = ca.Function(
            "generic_nmpc_step",
            [symbolic_state, symbolic_input, symbolic_disturbance],
            [ca.vertcat(*next_values)],
        )
        output_function = ca.Function(
            "generic_nmpc_output",
            [symbolic_state, symbolic_input, symbolic_disturbance],
            [ca.vertcat(*output_values)],
        )

        controls = ca.MX.sym("controls", nu, control_horizon)
        soft_constraint_sides: list[tuple[int, str]] = []
        for cv_index, variable in enumerate(config.controlled_variables):
            if variable.constraint_low is not None:
                soft_constraint_sides.append((cv_index, "low"))
            if variable.constraint_high is not None:
                soft_constraint_sides.append((cv_index, "high"))
        slack_side_count = len(soft_constraint_sides)
        self._soft_slack_count = slack_side_count * prediction_horizon
        soft_slacks = ca.MX.sym(
            "soft_slacks", slack_side_count, prediction_horizon
        )
        decision = ca.vertcat(ca.vec(controls), ca.vec(soft_slacks))
        common_parameter_count = nx + nd + nu
        parameter_count = common_parameter_count
        if self.prediction_form is NonlinearPredictionForm.VELOCITY:
            parameter_count += nx + nd + 1
        tuning_start = parameter_count
        parameter_count += 3 * len(config.controlled_variables) + nu + len(config.move_combinations)
        parameters = ca.MX.sym("parameters", parameter_count)

        cv_weight_parameters = []
        cv_rate_weight_parameters = []
        cv_target_parameters = []
        cursor = tuning_start
        for _ in config.controlled_variables:
            cv_weight_parameters.append(parameters[cursor])
            cv_rate_weight_parameters.append(parameters[cursor + 1])
            cv_target_parameters.append(parameters[cursor + 2])
            cursor += 3
        mv_weight_parameters = [parameters[cursor + index] for index in range(nu)]
        cursor += nu
        combination_weight_parameters = [
            parameters[cursor + index] for index in range(len(config.move_combinations))
        ]

        state_end = nx
        disturbance_end = state_end + nd
        previous_input_end = disturbance_end + nu
        current_state = parameters[0:state_end]
        disturbance = parameters[state_end:disturbance_end]
        previous_input = parameters[disturbance_end:previous_input_end]
        predicted_states = [current_state]
        predicted_outputs = [
            output_function(current_state, previous_input, disturbance)
        ]
        objective = 0
        constraints = []
        constraint_lower_bounds: list[float] = []
        constraint_upper_bounds: list[float] = []

        def transition(
            state_values: Sequence[Any],
            input_values: Sequence[Any],
            disturbance_values: Sequence[Any],
        ) -> tuple[Any, ...]:
            values = step_function(
                _casadi_vector(state_values),
                _casadi_vector(input_values),
                _casadi_vector(disturbance_values),
            )
            return tuple(values[index] for index in range(nx))

        if self.prediction_form is NonlinearPredictionForm.VELOCITY:
            previous_state_end = previous_input_end + nx
            previous_disturbance_end = previous_state_end + nd
            previous_state = parameters[
                previous_input_end:previous_state_end
            ]
            previous_disturbance = parameters[
                previous_state_end:previous_disturbance_end
            ]
            history_available = parameters[previous_disturbance_end]
            previous_model_next = evaluate_transition(
                transition,
                tuple(previous_state[index] for index in range(nx)),
                tuple(previous_input[index] for index in range(nu)),
                tuple(previous_disturbance[index] for index in range(nd)),
            )

        for move_index in range(control_horizon):
            change = controls[:, move_index] - (
                previous_input
                if move_index == 0
                else controls[:, move_index - 1]
            )
            constraints.append(change)
            for input_index, variable in enumerate(
                config.manipulated_variables
            ):
                constraint_lower_bounds.append(-variable.decrease_limit)
                constraint_upper_bounds.append(variable.increase_limit)
                normalized_change = (
                    change[input_index] / variable.move_normalization
                )
                objective += mv_weight_parameters[input_index] * normalized_change**2
            for combination_index, combination in enumerate(config.move_combinations):
                combined_change = sum(
                    coefficient * change[input_index]
                    for input_index, coefficient in enumerate(combination.coefficients)
                )
                objective += combination_weight_parameters[combination_index] * (
                    combined_change / combination.normalization
                ) ** 2

        for prediction_index in range(prediction_horizon):
            previous_output = predicted_outputs[-1]
            move_index = min(prediction_index, control_horizon - 1)
            current_input = controls[:, move_index]
            if self.prediction_form is NonlinearPredictionForm.VELOCITY:
                velocity_next, current_model_next = (
                    nonlinear_velocity_step_values(
                        tuple(current_state[index] for index in range(nx)),
                        tuple(current_input[index] for index in range(nu)),
                        tuple(disturbance[index] for index in range(nd)),
                        previous_model_next,
                        transition,
                    )
                )
                current_state = ca.if_else(
                    history_available > 0.5,
                    ca.vertcat(*velocity_next),
                    ca.vertcat(*current_model_next),
                )
                previous_model_next = current_model_next
            else:
                current_state = step_function(
                    current_state, current_input, disturbance
                )

            predicted_states.append(current_state)
            current_output = output_function(
                current_state, current_input, disturbance
            )
            predicted_outputs.append(current_output)
            for cv_index, variable in enumerate(config.controlled_variables):
                output_value = current_output[variable.output_index]
                if variable.rate_weight is not None:
                    output_change = (
                        output_value - previous_output[variable.output_index]
                    ) / variable.rate_normalization
                    objective += (
                        cv_rate_weight_parameters[cv_index] * output_change * output_change
                    )
                if variable.target is not None:
                    error = (
                        output_value - cv_target_parameters[cv_index]
                    ) / variable.normalization
                else:
                    below_zone = _smooth_positive(
                        variable.zone_low - output_value
                    )
                    above_zone = _smooth_positive(
                        output_value - variable.zone_high
                    )
                    error = (
                        below_zone + above_zone
                    ) / variable.normalization
                objective += cv_weight_parameters[cv_index] * error * error

            for side_index, (cv_index, side) in enumerate(
                soft_constraint_sides
            ):
                variable = config.controlled_variables[cv_index]
                output_value = current_output[variable.output_index]
                slack = soft_slacks[side_index, prediction_index]
                normalized_slack = (
                    slack / variable.constraint_normalization
                )
                objective += (
                    variable.constraint_weight
                    * normalized_slack
                    * normalized_slack
                )
                if side == "low":
                    constraints.append(output_value + slack)
                    constraint_lower_bounds.append(variable.constraint_low)
                    constraint_upper_bounds.append(math.inf)
                else:
                    constraints.append(output_value - slack)
                    constraint_lower_bounds.append(-math.inf)
                    constraint_upper_bounds.append(variable.constraint_high)

        constraint_vector = ca.vertcat(*constraints)
        nonlinear_program = {
            "x": decision,
            "p": parameters,
            "f": objective,
            "g": constraint_vector,
        }
        options = {
            "print_time": False,
            "ipopt.print_level": 0,
            "ipopt.sb": "yes",
            "ipopt.max_iter": config.maximum_solver_iterations,
            "ipopt.tol": 1e-7,
            "ipopt.acceptable_tol": 1e-6,
        }
        if config.maximum_solver_cpu_seconds is not None:
            options["ipopt.max_cpu_time"] = (
                config.maximum_solver_cpu_seconds
            )
        self._solver = ca.nlpsol(
            "generic_nmpc", "ipopt", nonlinear_program, options
        )
        self._trajectory_function = ca.Function(
            "generic_nmpc_trajectory",
            [parameters, decision],
            [
                ca.hcat(predicted_states),
                ca.hcat(predicted_outputs),
            ],
        )

        self._decision_lower_bounds = []
        self._decision_upper_bounds = []
        self._rate_lower_bounds = list(constraint_lower_bounds)
        self._rate_upper_bounds = list(constraint_upper_bounds)
        for _ in range(control_horizon):
            for variable in config.manipulated_variables:
                self._decision_lower_bounds.append(variable.lower_bound)
                self._decision_upper_bounds.append(variable.upper_bound)
        self._decision_lower_bounds.extend(
            0.0 for _ in range(self._soft_slack_count)
        )
        self._decision_upper_bounds.extend(
            math.inf for _ in range(self._soft_slack_count)
        )

    def _validate_configuration(self) -> None:
        config = self.configuration
        for name in (
            "prediction_horizon_samples",
            "control_horizon_samples",
            "maximum_solver_iterations",
        ):
            value = getattr(config, name)
            if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        if config.maximum_solver_cpu_seconds is not None:
            self._validate_positive(
                config.maximum_solver_cpu_seconds,
                "maximum_solver_cpu_seconds",
            )
        if config.control_horizon_samples > config.prediction_horizon_samples:
            raise ValueError(
                "control horizon must not exceed prediction horizon"
            )
        if not config.controlled_variables:
            raise ValueError("at least one controlled variable is required")
        if len(config.manipulated_variables) != self.manipulated_input_dimension:
            raise ValueError(
                "one manipulated-variable configuration is required per "
                "declared model input"
            )

        cv_names: set[str] = set()
        cv_indices: set[int] = set()
        for variable in config.controlled_variables:
            self._validate_name(variable.name, "controlled variable", cv_names)
            if (
                not isinstance(variable.output_index, int)
                or isinstance(variable.output_index, bool)
                or not 0 <= variable.output_index < self.output_dimension
            ):
                raise ValueError(
                    f"CV {variable.name!r} has an invalid output index"
                )
            if variable.output_index in cv_indices:
                raise ValueError(
                    "each configured CV must refer to a different model output"
                )
            cv_indices.add(variable.output_index)
            self._validate_positive(variable.weight, f"CV {variable.name} weight")
            self._validate_positive(
                variable.normalization,
                f"CV {variable.name} normalization",
            )
            if (variable.rate_weight is None) != (
                variable.rate_normalization is None
            ):
                raise ValueError(
                    f"CV {variable.name!r} output-rate tuning requires both "
                    "rate_weight and rate_normalization"
                )
            if variable.rate_weight is not None:
                self._validate_positive(
                    variable.rate_weight, f"CV {variable.name} rate weight"
                )
                self._validate_positive(
                    variable.rate_normalization,
                    f"CV {variable.name} rate normalization",
                )

            constraint_values = (
                variable.constraint_low,
                variable.constraint_high,
            )
            if any(value is not None for value in constraint_values):
                if (
                    variable.constraint_low is not None
                    and variable.constraint_high is not None
                    and variable.constraint_low >= variable.constraint_high
                ):
                    raise ValueError(
                        f"CV {variable.name!r} constraint low must be below high"
                    )
                self._validate_positive(
                    variable.constraint_weight,
                    f"CV {variable.name} constraint weight",
                )
                self._validate_positive(
                    variable.constraint_normalization,
                    f"CV {variable.name} constraint normalization",
                )
                for bound in constraint_values:
                    if bound is not None and not math.isfinite(float(bound)):
                        raise ValueError(
                            f"CV {variable.name!r} constraint bounds must be finite"
                        )
            elif (
                variable.constraint_weight is not None
                or variable.constraint_normalization is not None
            ):
                raise ValueError(
                    f"CV {variable.name!r} constraint tuning requires a bound"
                )

            has_target = variable.target is not None
            has_low = variable.zone_low is not None
            has_high = variable.zone_high is not None
            if has_target and (has_low or has_high):
                raise ValueError(
                    f"CV {variable.name!r} must use either a target or a zone"
                )
            if not has_target and not (has_low and has_high):
                raise ValueError(
                    f"CV {variable.name!r} requires a target or complete zone"
                )
            if has_target:
                self._validate_finite(
                    variable.target, f"CV {variable.name} target"
                )
            else:
                self._validate_finite(
                    variable.zone_low, f"CV {variable.name} zone low"
                )
                self._validate_finite(
                    variable.zone_high, f"CV {variable.name} zone high"
                )
                if variable.zone_low >= variable.zone_high:
                    raise ValueError(
                        f"CV {variable.name!r} zone limits must be increasing"
                    )

        mv_names: set[str] = set()
        for variable in config.manipulated_variables:
            self._validate_name(variable.name, "manipulated variable", mv_names)
            self._validate_finite(
                variable.lower_bound, f"MV {variable.name} lower bound"
            )
            self._validate_finite(
                variable.upper_bound, f"MV {variable.name} upper bound"
            )
            if variable.lower_bound >= variable.upper_bound:
                raise ValueError(
                    f"MV {variable.name!r} limits must be increasing"
                )
            self._validate_positive(
                variable.maximum_change_per_sample,
                f"MV {variable.name} maximum change per sample",
            )
            if variable.maximum_increase_per_sample is not None:
                self._validate_positive(
                    variable.maximum_increase_per_sample,
                    f"MV {variable.name} maximum increase per sample",
                )
            if variable.maximum_decrease_per_sample is not None:
                self._validate_positive(
                    variable.maximum_decrease_per_sample,
                    f"MV {variable.name} maximum decrease per sample",
                )
            self._validate_positive(
                variable.move_weight, f"MV {variable.name} move weight"
            )
            self._validate_positive(
                variable.move_normalization,
                f"MV {variable.name} move normalization",
            )

        combination_names: set[str] = set()
        for combination in config.move_combinations:
            self._validate_name(combination.name, "move combination", combination_names)
            if len(combination.coefficients) != self.manipulated_input_dimension:
                raise ValueError(
                    f"move combination {combination.name!r} requires exactly "
                    f"{self.manipulated_input_dimension} coefficients"
                )
            if not any(float(value) != 0.0 for value in combination.coefficients):
                raise ValueError(f"move combination {combination.name!r} cannot be all zero")
            for value in combination.coefficients:
                self._validate_finite(value, f"move combination {combination.name} coefficient")
            self._validate_positive(combination.weight, f"move combination {combination.name} weight")
            self._validate_positive(
                combination.normalization,
                f"move combination {combination.name} normalization",
            )

    def _validate_previous_inputs(
        self, previous_inputs: tuple[float, ...]
    ) -> None:
        for value, variable in zip(
            previous_inputs,
            self.configuration.manipulated_variables,
            strict=True,
        ):
            if not variable.lower_bound <= value <= variable.upper_bound:
                raise ValueError(
                    f"previous input for MV {variable.name!r} is outside limits"
                )

    def _read_dimension(self, name: str, minimum: int) -> int:
        value = getattr(self.model, name, None)
        if (
            not isinstance(value, int)
            or isinstance(value, bool)
            or value < minimum
        ):
            qualifier = "non-negative" if minimum == 0 else "positive"
            raise ValueError(f"model {name} must be a {qualifier} integer")
        return value

    @staticmethod
    def _validate_runtime_vector(
        name: str, values: Sequence[float], expected_dimension: int
    ) -> tuple[float, ...]:
        result = tuple(float(value) for value in values)
        if len(result) != expected_dimension:
            raise ValueError(
                f"{name} must contain exactly {expected_dimension} values"
            )
        if any(not math.isfinite(value) for value in result):
            raise ValueError(f"{name} must contain only finite values")
        return result

    @staticmethod
    def _validate_name(
        name: str, kind: str, existing_names: set[str]
    ) -> None:
        if not isinstance(name, str) or not name.strip():
            raise ValueError(f"{kind} name must be a non-empty string")
        if name in existing_names:
            raise ValueError(f"duplicate {kind} name: {name!r}")
        existing_names.add(name)

    @staticmethod
    def _validate_finite(value: float | None, name: str) -> None:
        if value is None or not math.isfinite(float(value)):
            raise ValueError(f"{name} must be finite")

    @classmethod
    def _validate_positive(cls, value: float, name: str) -> None:
        cls._validate_finite(value, name)
        if float(value) <= 0.0:
            raise ValueError(f"{name} must be positive")


def _casadi_vector(values: Sequence[Any]) -> ca.MX:
    """Return a CasADi column vector, including the zero-length case."""

    items = tuple(values)
    if items:
        return ca.vertcat(*items)
    return ca.MX.zeros(0, 1)


def _smooth_positive(value: Any, smoothing: float = 1e-6) -> Any:
    """Smooth approximation of max(value, 0) for zone penalties."""

    return 0.5 * (
        value + ca.sqrt(value * value + smoothing * smoothing)
    )
