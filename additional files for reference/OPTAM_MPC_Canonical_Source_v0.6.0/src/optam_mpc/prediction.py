"""Reusable nonlinear velocity-form prediction primitives.

The module knows nothing about a vessel, its variables, CasADi, or a solver.
An application supplies only its discrete nonlinear transition function

    x_next = F(x, u, d)

The complete velocity-form recursion is

    delta_x_next = F(x, u, d) - F(x_previous, u_previous, d_previous)
    x_next = x + delta_x_next

Successive calls carry the latest absolute model prediction forward.  When the
measured process is stationary and the input is unchanged, the two model
predictions are identical and the predicted state remains anchored at the
measurement.  That is the integral-action property being evaluated here.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from enum import Enum
from typing import Any


class NonlinearPredictionForm(str, Enum):
    """Prediction structures supported by the nonlinear controller."""

    ABSOLUTE = "absolute"
    VELOCITY = "velocity"


TransitionFunction = Callable[
    [Sequence[Any], Sequence[Any], Sequence[Any]], Sequence[Any]
]


def evaluate_transition(
    transition: TransitionFunction,
    state_values: Sequence[Any],
    input_values: Sequence[Any],
    disturbance_values: Sequence[Any],
) -> tuple[Any, ...]:
    """Evaluate an application transition and return a fixed value tuple."""

    result = tuple(transition(state_values, input_values, disturbance_values))
    if len(result) != len(state_values):
        raise ValueError(
            "the nonlinear transition must return one value per state"
        )
    return result


def nonlinear_velocity_step_values(
    state_values: Sequence[Any],
    input_values: Sequence[Any],
    disturbance_values: Sequence[Any],
    previous_model_next_values: Sequence[Any],
    transition: TransitionFunction,
) -> tuple[tuple[Any, ...], tuple[Any, ...]]:
    """Advance one complete nonlinear velocity-form prediction step.

    ``previous_model_next_values`` is F evaluated for the preceding state,
    input and disturbance.  The second returned tuple is the current F value;
    pass it back as ``previous_model_next_values`` at the following horizon
    step.  Returning it avoids evaluating the nonlinear model twice.
    """

    current_state = tuple(state_values)
    previous_model_next = tuple(previous_model_next_values)
    if len(previous_model_next) != len(current_state):
        raise ValueError(
            "previous model prediction must have the state dimension"
        )

    current_model_next = evaluate_transition(
        transition,
        current_state,
        input_values,
        disturbance_values,
    )
    next_state = tuple(
        state_value + current_model_value - previous_model_value
        for state_value, current_model_value, previous_model_value in zip(
            current_state,
            current_model_next,
            previous_model_next,
            strict=True,
        )
    )
    return next_state, current_model_next
