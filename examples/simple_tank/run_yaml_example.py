"""Example using YAML configuration to create and run MPC controller."""

import numpy as np
import matplotlib.pyplot as plt

from optam_mpc.config.loader import load_controller_config, create_controller_from_config
from optam_mpc.core.models import MIMOFOPDT


def main():
    """Run MPC controller configured from YAML file."""
    print("=" * 60)
    print("OptAM-MPC: YAML Configuration Example")
    print("=" * 60)
    
    # Load controller configuration from YAML
    config = load_controller_config("controller_config.yaml")
    print(f"\nLoaded controller: {config.name}")
    print(f"Model type: {config.model.model_type}")
    print(f"Horizons: Np={config.mpc.prediction_horizon}, Nc={config.mpc.control_horizon}")
    
    # Create controller from configuration
    controller = create_controller_from_config(config)
    
    # Create plant model (with slight mismatch to demonstrate bias correction)
    plant = MIMOFOPDT(
        K=np.array([[1.5, 0.5], [-0.2, 1.0]]),  # True plant
        tau=np.array([[5.0, 3.0], [2.0, 4.0]]),
        theta=np.array([[2.0, 1.0], [0.0, 1.5]]),
        dt=1.0,
    )
    
    # Initialize
    y0 = np.array([0.0, 0.0])
    u0 = np.array([0.0, 0.0])
    controller.reset(y0, u0)
    plant.reset(y0, u0)
    
    # Simulation settings
    n_steps = 60
    setpoints = np.zeros((n_steps, 2))
    setpoints[10:, 0] = 1.0
    setpoints[30:, 1] = 0.5
    
    # Arrays to store results
    y = y0.copy()
    u = u0.copy()
    outputs = np.zeros((n_steps + 1, 2))
    inputs = np.zeros((n_steps, 2))
    outputs[0, :] = y0
    
    # Run closed-loop simulation
    print("\nRunning simulation...")
    for step in range(n_steps):
        # Get measurement (no noise for simplicity)
        y_measured = y.copy()
        
        # Calculate control action
        result = controller.control(y_measured, setpoints[step, :])
        
        # Apply to plant
        u = result.u
        y = plant.step(u)
        
        # Store results
        outputs[step + 1, :] = y
        inputs[step, :] = u
        
        # Check for solver issues
        if not result.success:
            print(f"  Warning at step {step}: {result.message}")
    
    # Plot results
    fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    time = np.arange(n_steps + 1) * plant.dt
    
    for i in range(2):
        axes[i].plot(time, outputs[:, i], 'b-', linewidth=2, label=f'Output {i+1}')
        axes[i].plot(time[:-1], setpoints[:, i], 'r--', linewidth=1.5, label=f'Setpoint {i+1}')
        axes[i].set_ylabel(f'Output {i+1}')
        axes[i].grid(True, alpha=0.3)
        axes[i].legend()
    
    for i in range(2):
        axes[2+i].step(time[:-1], inputs[:, i], 'g-', where='post', label=f'Input {i+1}')
        axes[2+i].set_ylabel(f'Input {i+1}')
        axes[2+i].grid(True, alpha=0.3)
        axes[2+i].legend()
    
    axes[-1].set_xlabel('Time')
    plt.suptitle(f'MPC Control: {config.name}')
    plt.tight_layout()
    plt.show()
    
    # Print summary
    final_error = outputs[-1, :] - setpoints[-1, :]
    print(f"\nSimulation complete:")
    print(f"  Final error: {final_error}")
    print(f"  Final outputs: {outputs[-1, :]}")
    print(f"  Target: {setpoints[-1, :]}")


if __name__ == "__main__":
    main()
