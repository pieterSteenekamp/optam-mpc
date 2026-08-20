"""Generate configuration templates for OptAM-MPC.

This script creates example configuration files that users can modify
for their own applications.
"""

from pathlib import Path

import yaml


def generate_fopdt_template() -> dict:
    """Generate a template for FOPDT model configuration.

    Returns
    -------
    dict
        Configuration template.
    """
    return {
        "name": "FOPDT MPC Controller",
        "description": "Template for first-order-plus-dead-time model",
        "model": {
            "model_type": "fopdt",
            "dt": 1.0,
            "model_parameters": {
                "K": [[1.5, 0.5], [-0.2, 1.0]],
                "tau": [[5.0, 3.0], [2.0, 4.0]],
                "theta": [[2.0, 1.0], [0.0, 1.5]],
            },
        },
        "mpc": {
            "prediction_horizon": 15,
            "control_horizon": 5,
            "output_weights": [1.0, 1.0],
            "move_weights": [0.15, 0.15],
            "terminal_weights": [2.0, 2.0],
            "output_scale": [1.0, 0.5],
            "input_scale": [1.0, 1.0],
            "input_min": [-2.0, -2.0],
            "input_max": [2.0, 2.0],
            "move_min": [-0.5, -0.5],
            "move_max": [0.5, 0.5],
            "output_min": [-0.15, -0.15],
            "output_max": [1.15, 0.65],
            "bias_filter": 0.25,
            "solver_max_iterations": 200,
            "solver_tolerance": 1.0e-8,
        },
    }


def generate_integrating_tank_template() -> dict:
    """Generate a template for integrating tank model configuration.

    Returns
    -------
    dict
        Configuration template.
    """
    return {
        "name": "Integrating Tank MPC Controller",
        "description": "Template for integrating tank model",
        "model": {
            "model_type": "integrating_tank",
            "dt": 0.5,
            "model_parameters": {
                "K": 0.2,
                "theta": 1.25,
            },
        },
        "mpc": {
            "prediction_horizon": 24,
            "control_horizon": 6,
            "output_weights": [5.0],
            "move_weights": [0.5],
            "terminal_weights": [8.0],
            "output_scale": [10.0],
            "input_scale": [5.0],
            "input_min": [-5.0],
            "input_max": [5.0],
            "move_min": [-1.0],
            "move_max": [1.0],
            "output_min": [0.0],
            "output_max": [12.0],
            "bias_filter": 0.25,
            "solver_max_iterations": 200,
            "solver_tolerance": 1.0e-8,
        },
    }


def generate_cstr_template() -> dict:
    """Generate a template for nonlinear CSTR model configuration.

    Returns
    -------
    dict
        Configuration template.
    """
    return {
        "name": "CSTR MPC Controller",
        "description": "Template for nonlinear CSTR model",
        "model": {
            "model_type": "nonlinear_cstr",
            "dt": 0.5,
            "model_parameters": {
                "volume": 100.0,
                "inlet_concentration": 1.0,
                "inlet_temperature": 350.0,
                "pre_exponential_factor": 7.2e10,
                "activation_temperature": 8750.0,
                "heat_of_reaction": -5.0e4,
                "density_heat_capacity": 500.0,
                "heat_transfer": 5.0e4,
                "integration_substeps": 2,
            },
        },
        "mpc": {
            "prediction_horizon": 10,
            "control_horizon": 3,
            "output_weights": [3.0, 2.0],
            "move_weights": [0.3, 0.2],
            "terminal_weights": [8.0, 6.0],
            "output_scale": [0.10, 10.0],
            "input_scale": [5.0, 20.0],
            "input_min": [5.0, 280.0],
            "input_max": [25.0, 340.0],
            "move_min": [-3.0, -8.0],
            "move_max": [3.0, 8.0],
            "output_min": [0.25, 300.0],
            "output_max": [1.05, 335.0],
            "soft_output_weights": [2.0e4, 2.0e4],
            "maximum_output_slack": [0.20, 5.0],
            "bias_filter": 0.20,
            "solver_max_iterations": 180,
            "solver_tolerance": 1.0e-8,
        },
    }


def save_template(template: dict, filename: str):
    """Save a configuration template to a YAML file.

    Parameters
    ----------
    template : dict
        Configuration template to save.
    filename : str
        Output filename.
    """
    with open(filename, 'w') as f:
        yaml.dump(template, f, default_flow_style=False, sort_keys=False)
    print(f"Saved template to: {filename}")


def main():
    """Generate all configuration templates."""
    print("=" * 60)
    print("OptAM-MPC Configuration Template Generator")
    print("=" * 60)
    
    # Create output directory if it doesn't exist
    output_dir = Path("config_templates")
    output_dir.mkdir(exist_ok=True)
    
    # Generate templates
    templates = {
        "fopdt_template.yaml": generate_fopdt_template(),
        "integrating_tank_template.yaml": generate_integrating_tank_template(),
        "cstr_template.yaml": generate_cstr_template(),
    }
    
    for filename, template in templates.items():
        filepath = output_dir / filename
        save_template(template, filepath)
    
    print("\nTemplates generated in 'config_templates' directory:")
    print("  1. fopdt_template.yaml - Linear FOPDT model")
    print("  2. integrating_tank_template.yaml - Integrating tank model")
    print("  3. cstr_template.yaml - Nonlinear CSTR model")
    print("\nCopy and modify these templates for your application.")


if __name__ == "__main__":
    main()
