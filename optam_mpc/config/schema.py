"""Configuration schemas for OptAM-MPC.

This module defines Pydantic models for validating MPC configuration
parameters. Using Pydantic provides automatic type checking, validation,
and clear error messages for configuration errors.
"""

from __future__ import annotations

from typing import Optional, List, Union

import numpy as np
from pydantic import BaseModel, Field, field_validator


class MPCConfig(BaseModel):
    """Configuration for the MPC controller.

    Parameters
    ----------
    prediction_horizon : int
        Number of future time steps to predict.
    control_horizon : int
        Number of future control moves to optimize.
    output_weights : list[float] or None
        Weights for output tracking error.
    move_weights : list[float] or None
        Weights for input move suppression.
    terminal_weights : list[float] or None
        Weights for terminal output tracking error.
    output_scale : list[float] or None
        Scaling factors for outputs.
    input_scale : list[float] or None
        Scaling factors for inputs.
    input_min : list[float] or None
        Minimum input limits.
    input_max : list[float] or None
        Maximum input limits.
    move_min : list[float] or None
        Minimum input move limits.
    move_max : list[float] or None
        Maximum input move limits.
    output_min : list[float] or None
        Minimum output limits.
    output_max : list[float] or None
        Maximum output limits.
    soft_output_weights : list[float] or None
        Weights for soft output constraint violations.
    maximum_output_slack : list[float] or float
        Maximum allowed soft constraint violation.
    bias_filter : float
        Filter factor for output bias correction (0-1).
    solver_max_iterations : int
        Maximum iterations for the optimizer.
    solver_tolerance : float
        Tolerance for optimizer convergence.
    constraint_tolerance : float
        Tolerance for constraint satisfaction.
    raise_on_failure : bool
        Whether to raise an exception on solver failure.
    """

    # Horizon settings
    prediction_horizon: int = Field(default=15, gt=0, description="Prediction horizon")
    control_horizon: int = Field(default=5, gt=0, description="Control horizon")

    # Weight settings
    output_weights: Optional[List[float]] = Field(
        default=None, description="Output tracking weights"
    )
    move_weights: Optional[List[float]] = Field(
        default=None, description="Move suppression weights"
    )
    terminal_weights: Optional[List[float]] = Field(
        default=None, description="Terminal tracking weights"
    )

    # Scaling settings
    output_scale: Optional[List[float]] = Field(
        default=None, description="Output scaling factors"
    )
    input_scale: Optional[List[float]] = Field(
        default=None, description="Input scaling factors"
    )

    # Input constraints
    input_min: Optional[List[float]] = Field(
        default=None, description="Minimum input limits"
    )
    input_max: Optional[List[float]] = Field(
        default=None, description="Maximum input limits"
    )
    move_min: Optional[List[float]] = Field(
        default=None, description="Minimum move limits"
    )
    move_max: Optional[List[float]] = Field(
        default=None, description="Maximum move limits"
    )

    # Output constraints
    output_min: Optional[List[float]] = Field(
        default=None, description="Minimum output limits"
    )
    output_max: Optional[List[float]] = Field(
        default=None, description="Maximum output limits"
    )
    soft_output_weights: Optional[List[float]] = Field(
        default=None, description="Soft output constraint weights"
    )
    maximum_output_slack: Union[List[float], float] = Field(
        default=float("inf"), description="Maximum soft constraint violation"
    )

    # Controller settings
    bias_filter: float = Field(
        default=0.25, ge=0.0, le=1.0, description="Bias correction filter factor"
    )
    solver_max_iterations: int = Field(
        default=200, gt=0, description="Maximum solver iterations"
    )
    solver_tolerance: float = Field(
        default=1e-8, gt=0, description="Solver convergence tolerance"
    )
    constraint_tolerance: float = Field(
        default=1e-6, gt=0, description="Constraint satisfaction tolerance"
    )
    raise_on_failure: bool = Field(
        default=False, description="Raise exception on solver failure"
    )

    @field_validator("control_horizon")
    @classmethod
    def validate_control_horizon(cls, v: int, info) -> int:
        """Validate that control horizon doesn't exceed prediction horizon."""
        if "prediction_horizon" in info.data and v > info.data["prediction_horizon"]:
            raise ValueError("control_horizon must be <= prediction_horizon")
        return v

    @field_validator(
        "output_weights",
        "move_weights",
        "terminal_weights",
        "output_scale",
        "input_scale",
        "input_min",
        "input_max",
        "move_min",
        "move_max",
        "output_min",
        "output_max",
        "soft_output_weights",
    )
    @classmethod
    def validate_list_length(cls, v: Optional[List[float]], info) -> Optional[List[float]]:
        """Validate that list lengths match model dimensions."""
        if v is not None:
            # Basic validation - more specific checks will be done in controller
            if len(v) == 0:
                raise ValueError(f"{info.field_name} must not be empty")
        return v

    def to_numpy(self) -> dict:
        """Convert configuration to numpy arrays for use in controller.

        Returns
        -------
        dict
            Dictionary with numpy arrays for all settings.
        """
        result = {}
        for key, value in self.model_dump().items():
            if isinstance(value, list):
                result[key] = np.array(value, dtype=float)
            elif isinstance(value, (int, float, bool)):
                result[key] = value
        return result


class ModelConfig(BaseModel):
    """Configuration for process models.

    Parameters
    ----------
    model_type : str
        Type of model ('fopdt', 'integrating_tank', 'nonlinear_cstr').
    model_parameters : dict
        Dictionary of model-specific parameters.
    dt : float
        Sampling time in seconds.
    """

    model_type: str = Field(..., description="Type of process model")
    model_parameters: dict = Field(
        default_factory=dict, description="Model-specific parameters"
    )
    dt: float = Field(..., gt=0, description="Sampling time in seconds")

    @field_validator("model_type")
    @classmethod
    def validate_model_type(cls, v: str) -> str:
        """Validate that model type is supported."""
        supported = ["fopdt", "integrating_tank", "nonlinear_cstr"]
        if v.lower() not in supported:
            raise ValueError(f"model_type must be one of {supported}")
        return v.lower()


class ControllerConfig(BaseModel):
    """Complete controller configuration.

    Parameters
    ----------
    mpc : MPCConfig
        MPC controller settings.
    model : ModelConfig
        Process model configuration.
    name : str, optional
        Controller name for identification.
    """

    name: str = Field(default="MPC Controller", description="Controller name")
    mpc: MPCConfig = Field(default_factory=MPCConfig, description="MPC settings")
    model: ModelConfig = Field(..., description="Model configuration")


# Example usage and validation
if __name__ == "__main__":
    # Example configuration
    config = MPCConfig(
        prediction_horizon=10,
        control_horizon=3,
        output_weights=[1.0, 1.0],
        move_weights=[0.1, 0.1],
        input_min=[-1.0, -1.0],
        input_max=[1.0, 1.0],
    )
    
    print("Configuration created successfully:")
    print(config.model_dump_json(indent=2))
