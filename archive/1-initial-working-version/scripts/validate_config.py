"""Validate MPC configuration files.

This script checks YAML configuration files for correctness before
they are used with the MPC controller.

Usage:
    python scripts/validate_config.py path/to/config.yaml
"""

import sys
from pathlib import Path

from optam_mpc.config.loader import load_controller_config, create_controller_from_config


def validate_config_file(filepath: str) -> bool:
    """Validate a configuration file.

    Parameters
    ----------
    filepath : str
        Path to YAML configuration file.

    Returns
    -------
    bool
        True if configuration is valid, False otherwise.
    """
    print(f"Validating configuration: {filepath}")
    print("-" * 60)
    
    try:
        # Load and validate configuration
        config = load_controller_config(filepath)
        print(f"✓ Configuration name: {config.name}")
        print(f"✓ Model type: {config.model.model_type}")
        print(f"✓ Prediction horizon: {config.mpc.prediction_horizon}")
        print(f"✓ Control horizon: {config.mpc.control_horizon}")
        
        # Try to create controller
        controller = create_controller_from_config(config)
        print(f"✓ Controller created successfully")
        print(f"  - Number of inputs: {controller.nu}")
        print(f"  - Number of outputs: {controller.ny}")
        
        print("-" * 60)
        print("✓ Configuration is valid!")
        return True
        
    except FileNotFoundError as e:
        print(f"✗ Error: {e}")
        return False
    except ValueError as e:
        print(f"✗ Configuration error: {e}")
        return False
    except Exception as e:
        print(f"✗ Unexpected error: {type(e).__name__}: {e}")
        return False


def main():
    """Main entry point."""
    if len(sys.argv) != 2:
        print("Usage: python validate_config.py <config.yaml>")
        sys.exit(1)
    
    filepath = sys.argv[1]
    if validate_config_file(filepath):
        sys.exit(0)
    else:
        sys.exit(1)


if __name__ == "__main__":
    main()
