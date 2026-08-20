"""Complete MPC example: 2x2 MIMO system control.

This example demonstrates:
1. Creating a process model
2. Configuring the MPC controller
3. Running a closed-loop simulation
4. Visualizing the results
5. Analyzing performance metrics

The example uses a 2x2 first-order-plus-dead-time (FOPDT) system
representing a typical chemical process with cross-coupling.
"""

import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import List, Optional

from optam_mpc.core.models import MIMOFOPDT
from optam_mpc.core.mpc import MPCController, MPCResult


@dataclass
class SimulationConfig:
    """Configuration for closed-loop simulation."""
    n_steps: int = 60
    measurement_noise_std: Optional[List[float]] = None
    input_disturbance: Optional[np.ndarray] = None
    random_seed: int = 42


@dataclass
class SimulationResult:
    """Results from closed-loop simulation."""
    time: np.ndarray
    outputs: np.ndarray
    inputs: np.ndarray
    setpoints: np.ndarray
    controller_results: List[MPCResult]


class ProcessSimulator:
    """Simulates a process plant with optional noise and disturbances."""
    
    def __init__(
        self,
        plant_model,
        noise_std: Optional[List[float]] = None,
        input_disturbance: Optional[np.ndarray] = None,
        random_seed: int = 42,
    ):
        """Initialize the simulator.
        
        Parameters
        ----------
        plant_model : Process
            Model representing the actual plant.
        noise_std : list of float, optional
            Standard deviation of measurement noise for each output.
        input_disturbance : np.ndarray, optional
            Disturbance added to inputs, shape (n_steps, nu).
        random_seed : int
            Seed for random number generation.
        """
        self.plant = plant_model
        self.noise_std = noise_std or [0.0] * plant_model.ny
        self.input_disturbance = input_disturbance
        self.rng = np.random.default_rng(random_seed)
        
    def reset(self, y0: np.ndarray, u0: np.ndarray):
        """Reset the simulator to initial conditions."""
        self.plant.reset(y0, u0)
        
    def step(self, u: np.ndarray, step: int) -> np.ndarray:
        """Advance the simulation by one step.
        
        Parameters
        ----------
        u : np.ndarray
            Commanded inputs.
        step : int
            Current step number (for disturbance lookup).
            
        Returns
        -------
        np.ndarray
            Measured outputs (with noise).
        """
        # Apply input disturbance if configured
        if self.input_disturbance is not None:
            u_actual = u + self.input_disturbance[step, :]
        else:
            u_actual = u
        
        # Get true output
        y_true = self.plant.step(u_actual)
        
        # Add measurement noise
        noise = self.rng.normal(0.0, self.noise_std)
        y_measured = y_true + noise
        
        return y_measured, y_true


def create_models():
    """Create plant and controller models.
    
    Returns
    -------
    tuple
        (plant_model, controller_model) - The plant has slight mismatch
        to demonstrate bias correction.
    """
    # Sampling time
    dt = 1.0
    
    # Model parameters for a 2x2 system
    # This represents a process with cross-coupling between inputs and outputs
    K_plant = np.array([
        [1.5, 0.5],   # Output 1 responds to both inputs
        [-0.2, 1.0],  # Output 2 responds to both inputs
    ])
    K_model = np.array([
        [1.4, 0.5],   # Slight mismatch in gain
        [-0.2, 0.9],
    ])
    
    tau = np.array([
        [5.0, 3.0],
        [2.0, 4.0],
    ])
    
    theta = np.array([
        [2.0, 1.0],
        [0.0, 1.5],
    ])
    
    plant = MIMOFOPDT(K_plant, tau, theta, dt)
    model = MIMOFOPDT(K_model, tau, theta, dt)
    
    return plant, model


def create_controller_config():
    """Create MPC controller configuration.
    
    Returns
    -------
    dict
        Controller configuration dictionary.
    """
    return {
        # Horizon settings
        "prediction_horizon": 15,
        "control_horizon": 5,
        
        # Weight settings
        "output_weights": [1.0, 1.0],       # Equal importance on both outputs
        "move_weights": [0.15, 0.15],       # Moderate move suppression
        "terminal_weights": [2.0, 2.0],     # Extra weight on final prediction
        
        # Scaling
        "output_scale": [1.0, 0.5],         # Output 2 has smaller range
        "input_scale": [1.0, 1.0],
        
        # Input constraints
        "input_min": [-2.0, -2.0],
        "input_max": [2.0, 2.0],
        "move_min": [-0.5, -0.5],
        "move_max": [0.5, 0.5],
        
        # Output constraints
        "output_min": [-0.15, -0.15],
        "output_max": [1.15, 0.65],
        
        # Controller settings
        "bias_filter": 0.25,
        "solver_max_iterations": 200,
        "solver_tolerance": 1e-8,
    }


def create_setpoints(n_steps: int) -> np.ndarray:
    """Create setpoint trajectory.
    
    Parameters
    ----------
    n_steps : int
        Number of simulation steps.
        
    Returns
    -------
    np.ndarray
        Setpoint array with shape (n_steps, ny).
    """
    setpoints = np.zeros((n_steps, 2))
    
    # Step change in output 1 at t=10
    setpoints[10:, 0] = 1.0
    
    # Step change in output 2 at t=30
    setpoints[30:, 1] = 0.5
    
    return setpoints


def run_simulation() -> SimulationResult:
    """Run the complete closed-loop simulation.
    
    Returns
    -------
    SimulationResult
        Complete simulation results.
    """
    # Create models
    plant, model = create_models()
    
    # Create controller
    config = create_controller_config()
    controller = MPCController(model, config)
    
    # Create setpoints
    n_steps = 60
    setpoints = create_setpoints(n_steps)
    
    # Initialize
    y0 = np.array([0.0, 0.0])
    u0 = np.array([0.0, 0.0])
    
    controller.reset(y0, u0)
    plant.reset(y0, u0)
    
    # Add some measurement noise and disturbance for realism
    simulator = ProcessSimulator(
        plant,
        noise_std=[0.01, 0.01],
        input_disturbance=None,
        random_seed=42,
    )
    
    # Arrays to store results
    time = np.arange(n_steps + 1, dtype=float) * plant.dt
    outputs = np.zeros((n_steps + 1, plant.ny))
    inputs = np.zeros((n_steps, plant.nu))
    controller_results = []
    
    outputs[0, :] = y0
    
    # Run closed-loop simulation
    print("Starting closed-loop simulation...")
    print(f"{'Step':>4} {'Time':>6} {'y1':>8} {'y2':>8} {'u1':>8} {'u2':>8} {'Status':>8}")
    print("-" * 60)
    
    for step in range(n_steps):
        # Get measurement
        y_measured, y_true = simulator.step(inputs[step-1, :] if step > 0 else u0, step)
        outputs[step, :] = y_true
        
        # Calculate control action
        result = controller.control(y_measured, setpoints[step, :])
        
        # Store results
        inputs[step, :] = result.u
        controller_results.append(result)
        
        # Print progress
        status = "OK" if result.success else "FALLBACK"
        if step % 5 == 0 or not result.success:
            print(
                f"{step:4d} {time[step]:6.1f} "
                f"{y_true[0]:8.3f} {y_true[1]:8.3f} "
                f"{result.u[0]:8.3f} {result.u[1]:8.3f} "
                f"{status:>8}"
            )
    
    # Final output
    outputs[n_steps, :] = simulator.step(inputs[-1, :], n_steps)[1]
    
    print("-" * 60)
    print(f"Simulation complete: {n_steps} steps")
    print(f"Solver failures: {sum(not r.success for r in controller_results)}")
    print(f"Mean solve time: {np.mean([r.solve_time_seconds for r in controller_results])*1000:.2f} ms")
    
    return SimulationResult(
        time=time,
        outputs=outputs,
        inputs=inputs,
        setpoints=setpoints,
        controller_results=controller_results,
    )


def calculate_metrics(result: SimulationResult) -> dict:
    """Calculate performance metrics.
    
    Parameters
    ----------
    result : SimulationResult
        Simulation results.
        
    Returns
    -------
    dict
        Dictionary of performance metrics.
    """
    # Calculate tracking error (excluding initial condition)
    error = result.outputs[1:, :] - result.setpoints
    
    metrics = {
        "final_error": error[-1, :],
        "rmse": np.sqrt(np.mean(error**2, axis=0)),
        "max_abs_error": np.max(np.abs(error), axis=0),
        "total_input_movement": np.sum(np.abs(np.diff(result.inputs, axis=0)), axis=0),
        "solver_failures": sum(not r.success for r in result.controller_results),
        "mean_solve_time_ms": np.mean([r.solve_time_seconds for r in result.controller_results]) * 1000,
    }
    
    return metrics


def plot_results(result: SimulationResult):
    """Plot simulation results.
    
    Parameters
    ----------
    result : SimulationResult
        Simulation results to plot.
    """
    fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    
    # Plot outputs
    for i in range(2):
        ax = axes[i]
        ax.plot(result.time, result.outputs[:, i], 'b-', linewidth=2, label=f'Output {i+1}')
        ax.plot(result.time[:-1], result.setpoints[:, i], 'r--', linewidth=1.5, label=f'Setpoint {i+1}')
        ax.set_ylabel(f'Output {i+1}')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best')
        ax.set_title(f'Output {i+1} Tracking')
    
    # Plot inputs
    for i in range(2):
        ax = axes[2 + i]
        ax.step(result.time[:-1], result.inputs[:, i], 'g-', where='post', linewidth=1.5, label=f'Input {i+1}')
        ax.set_ylabel(f'Input {i+1}')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best')
        ax.set_title(f'Input {i+1}')
    
    axes[-1].set_xlabel('Time')
    plt.suptitle('MPC Control of 2x2 MIMO System', fontsize=14)
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    return fig


def main():
    """Run the example."""
    print("=" * 60)
    print("OptAM-MPC: 2x2 MIMO System Example")
    print("=" * 60)
    
    # Run simulation
    result = run_simulation()
    
    # Calculate metrics
    metrics = calculate_metrics(result)
    
    print("\nPerformance Metrics:")
    print(f"  Final tracking error: {metrics['final_error']}")
    print(f"  RMSE: {metrics['rmse']}")
    print(f"  Max absolute error: {metrics['max_abs_error']}")
    print(f"  Total input movement: {metrics['total_input_movement']}")
    print(f"  Solver failures: {metrics['solver_failures']}")
    print(f"  Mean solve time: {metrics['mean_solve_time_ms']:.2f} ms")
    
    # Plot results
    fig = plot_results(result)
    plt.show()
    
    return result, metrics


if __name__ == "__main__":
    result, metrics = main()
