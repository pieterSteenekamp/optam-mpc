"""Configuration loading utilities for OptAM-MPC.

This module provides functions to load MPC configurations from YAML files,
making it easy for users to configure controllers without writing Python code.
"""

from pathlib import Path
from typing import Dict, Any, Optional

import numpy as np
import yaml

from optam_mpc.config.schema import MPCConfig, ModelConfig, ControllerConfig


def load_yaml_config(filepath: str) -> Dict[str, Any]:
    """Load configuration from a YAML file.

    Parameters
    ----------
    filepath : str
        Path to the YAML configuration file.

    Returns
    -------
    dict
        Configuration dictionary.

    Raises
    ------
    FileNotFoundError
        If the file doesn't exist.
    yaml.YAMLError
        If the file has invalid YAML syntax.
    """
    path = Path(filepath)
    if not path.exists():
        raise FileNotFoundError(f"Configuration file not found: {filepath}")
    
    with open(path, 'r') as f:
        config = yaml.safe_load(f)
    
    if config is None:
        raise ValueError(f"Configuration file is empty: {filepath}")
    
    return config


def load_mpc_config(config_dict: Dict[str, Any]) -> MPCConfig:
    """Create MPCConfig from a dictionary.

    Parameters
    ----------
    config_dict : dict
        Configuration dictionary.

    Returns
    -------
    MPCConfig
        Validated MPC configuration.
    """
    return MPCConfig(**config_dict)


def load_model_config(config_dict: Dict[str, Any]) -> ModelConfig:
    """Create ModelConfig from a dictionary.

    Parameters
    ----------
    config_dict : dict
        Configuration dictionary.

    Returns
    -------
    ModelConfig
        Validated model configuration.
    """
    return ModelConfig(**config_dict)


def load_controller_config(filepath: str) -> ControllerConfig:
    """Load complete controller configuration from YAML file.

    Parameters
    ----------
    filepath : str
        Path to YAML configuration file.

    Returns
    -------
    ControllerConfig
        Complete validated controller configuration.

    Examples
    --------
    >>> config = load_controller_config("controller_config.yaml")
    >>> controller = create_controller_from_config(config)
    """
    config_dict = load_yaml_config(filepath)
    return ControllerConfig(**config_dict)


def create_model_from_config(config: ModelConfig):
    """Create a process model from configuration.

    Parameters
    ----------
    config : ModelConfig
        Model configuration.

    Returns
    -------
    Process
        Process model instance.

    Raises
    ------
    ValueError
        If model type is not supported or parameters are invalid.
    """
    from optam_mpc.core.models import MIMOFOPDT, IntegratingTank, NonlinearCSTR, CSTRParameters
    
    model_type = config.model_type
    params = config.model_parameters
    dt = config.dt
    
    if model_type == "fopdt":
        # Required parameters: K, tau, theta
        required = ["K", "tau", "theta"]
        for key in required:
            if key not in params:
                raise ValueError(f"FOPDT model requires parameter: {key}")
        
        K = np.array(params["K"], dtype=float)
        tau = np.array(params["tau"], dtype=float)
        theta = np.array(params["theta"], dtype=float)
        
        return MIMOFOPDT(K=K, tau=tau, theta=theta, dt=dt)
    
    elif model_type == "integrating_tank":
        # Required parameters: K, theta
        required = ["K", "theta"]
        for key in required:
            if key not in params:
                raise ValueError(f"Integrating tank model requires parameter: {key}")
        
        return IntegratingTank(
            K=float(params["K"]),
            theta=float(params["theta"]),
            dt=dt,
        )
    
    elif model_type == "nonlinear_cstr":
        # Optional parameters with defaults
        cstr_params = CSTRParameters(
            volume=float(params.get("volume", 100.0)),
            inlet_concentration=float(params.get("inlet_concentration", 1.0)),
            inlet_temperature=float(params.get("inlet_temperature", 350.0)),
            pre_exponential_factor=float(params.get("pre_exponential_factor", 7.2e10)),
            activation_temperature=float(params.get("activation_temperature", 8750.0)),
            heat_of_reaction=float(params.get("heat_of_reaction", -5.0e4)),
            density_heat_capacity=float(params.get("density_heat_capacity", 500.0)),
            heat_transfer=float(params.get("heat_transfer", 5.0e4)),
        )
        
        integration_substeps = int(params.get("integration_substeps", 4))
        
        return NonlinearCSTR(
            dt=dt,
            parameters=cstr_params,
            integration_substeps=integration_substeps,
        )
    
    else:
        raise ValueError(f"Unsupported model type: {model_type}")


def create_controller_from_config(config: ControllerConfig):
    """Create an MPC controller from complete configuration.

    Parameters
    ----------
    config : ControllerConfig
        Complete controller configuration.

    Returns
    -------
    MPCController
        Configured MPC controller.
    """
    from optam_mpc.core.mpc import MPCController
    
    # Create model
    model = create_model_from_config(config.model)
    
    # Convert MPC config to dictionary
    mpc_config = config.mpc.model_dump()
    
    # Create controller
    controller = MPCController(model, mpc_config)
    
    return controller


def save_config_template(filepath: str):
    """Save a configuration template to a file.

    Parameters
    ----------
    filepath : str
        Path where template should be saved.
    """
    template = {
        "name": "My MPC Controller",
        "model": {
            "model_type": "fopdt",
            "dt": 1.0,
            "model_parameters": {
                "K": [[1.0, 0.5], [0.2, 1.0]],
                "tau": [[5.0, 3.0], [2.0, 4.0]],
                "theta": [[1.0, 0.5], [0.0, 1.0]],
            },
        },
        "mpc": {
            "prediction_horizon": 15,
            "control_horizon": 5,
            "output_weights": [1.0, 1.0],
            "move_weights": [0.15, 0.15],
            "terminal_weights": [2.0, 2.0],
            "output_scale": [1.0, 1.0],
            "input_scale": [1.0, 1.0],
            "input_min": [-2.0, -2.0],
            "input_max": [2.0, 2.0],
            "move_min": [-0.5, -0.5],
            "move_max": [0.5, 0.5],
            "output_min": [-0.15, -0.15],
            "output_max": [1.15, 0.65],
            "bias_filter": 0.25,
            "solver_max_iterations": 200,
            "solver_tolerance": 1e-8,
        },
    }
    
    with open(filepath, 'w') as f:
        yaml.dump(template, f, default_flow_style=False, sort_keys=False)
    
    print(f"Configuration template saved to: {filepath}")
