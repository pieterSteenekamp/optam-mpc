"""Complete OPC UA integration example - Version 3 (realistic operation).

This version uses step changes as a plant operator would, with proper
synchronization between the digital twin and controller.
"""

import time
import numpy as np
import matplotlib.pyplot as plt

from optam_mpc.core.models import MIMOFOPDT
from optam_mpc.core.mpc import MPCController
from optam_mpc.interfaces.opcua_server import DigitalTwinServer
from optam_mpc.interfaces.opcua_client import OPCUAControllerInterface


def create_process_models():
    """Create plant and controller models."""
    dt = 1.0
    
    # Plant model (true process)
    plant_model = MIMOFOPDT(
        K=np.array([[1.5, 0.5], [-0.2, 1.0]]),
        tau=np.array([[5.0, 3.0], [2.0, 4.0]]),
        theta=np.array([[2.0, 1.0], [0.0, 1.5]]),
        dt=dt,
    )
    
    # Controller model (with slight mismatch)
    controller_model = MIMOFOPDT(
        K=np.array([[1.4, 0.5], [-0.2, 0.9]]),
        tau=np.array([[5.0, 3.0], [2.0, 4.0]]),
        theta=np.array([[2.0, 1.0], [0.0, 1.5]]),
        dt=dt,
    )
    
    return plant_model, controller_model


def create_controller():
    """Create MPC controller with robust tuning."""
    _, model = create_process_models()
    
    config = {
        "prediction_horizon": 20,      # Longer horizon for step changes
        "control_horizon": 5,
        "output_weights": [1.0, 1.0],
        "move_weights": [0.5, 0.5],    # More move suppression for stability
        "terminal_weights": [5.0, 5.0], # Stronger terminal cost
        "output_scale": [1.0, 0.5],
        "input_scale": [1.0, 1.0],
        "input_min": [-2.0, -2.0],
        "input_max": [2.0, 2.0],
        "move_min": [-0.3, -0.3],      # Slower moves for stability
        "move_max": [0.3, 0.3],
        "output_min": [-0.5, -0.5],    # Wider output limits
        "output_max": [1.5, 1.0],
        "bias_filter": 0.3,            # Slightly more bias filtering
    }
    
    return MPCController(model, config)


class SynchronizedDigitalTwin(DigitalTwinServer):
    """Digital twin that runs synchronously with the controller."""
    
    def __init__(self, process, endpoint="opc.tcp://localhost:4840", name="Digital Twin"):
        super().__init__(
            process=process,
            endpoint=endpoint,
            name=name,
            update_interval=0.1,
            noise_std=[0.005, 0.005],
        )
    
    def _run_loop(self):
        """Step-by-step simulation matching controller interval."""
        while self._running:
            try:
                # Read inputs from OPC UA
                inputs = np.array([node.get_value() for node in self._input_nodes])
                
                # Simulate one process step
                outputs = self.process.step(inputs)
                
                # Add measurement noise
                noise = self.rng.normal(0.0, self.noise_std)
                measured = outputs + noise
                
                # Write outputs to OPC UA
                for i, node in enumerate(self._output_nodes):
                    node.set_value(float(measured[i]))
                
                # Wait for next control interval
                time.sleep(1.0)
                
            except Exception as e:
                print(f"Digital twin error: {e}")
                time.sleep(0.1)


def run_integration_test():
    """Run the integration test with realistic step changes."""
    print("=" * 70)
    print("OptAM-MPC: OPC UA Integration Test (v3 - Realistic)")
    print("=" * 70)
    
    # Create models
    plant_model, _ = create_process_models()
    
    # Initialize
    y0 = np.array([0.0, 0.0])
    u0 = np.array([0.0, 0.0])
    plant_model.reset(y0, u0)
    
    # Create digital twin
    print("\n1. Starting Digital Twin Server...")
    twin = SynchronizedDigitalTwin(
        process=plant_model,
        endpoint="opc.tcp://localhost:4840",
        name="Digital Twin",
    )
    
    # Create MPC controller
    print("2. Creating MPC Controller...")
    controller = create_controller()
    
    # Create OPC UA interface
    print("3. Setting up OPC UA Client...")
    interface = OPCUAControllerInterface(
        controller=controller,
        endpoint="opc.tcp://localhost:4840",
    )
    
    # Start the digital twin
    print("4. Starting Digital Twin...")
    twin.start()
    time.sleep(2.0)
    
    # Connect the controller
    print("5. Connecting MPC Controller...")
    interface.connect()
    
    # Initialize
    print("6. Initializing...")
    interface.initialize(y0, u0)
    time.sleep(1.5)
    
    # Define setpoint trajectory with STEP changes (like real operators)
    n_steps = 50
    setpoints = np.zeros((n_steps, 2))
    
    # Step change in Output 1 at step 5 (operator changes setpoint)
    setpoints[5:, 0] = 0.8
    
    # Step change in Output 2 at step 25 (operator changes setpoint)
    setpoints[25:, 1] = 0.4
    
    # Another step change in Output 1 at step 35 (operator adjusts)
    setpoints[35:, 0] = 0.5
    
    print("\nSetpoint changes:")
    print("  Step 5:  Output 1 -> 0.8")
    print("  Step 25: Output 2 -> 0.4")
    print("  Step 35: Output 1 -> 0.5")
    
    # Arrays to store results
    measurements = np.zeros((n_steps, 2))
    inputs = np.zeros((n_steps, 2))
    results = []
    
    # Run control loop
    print("\n7. Running Control Loop...")
    print(f"{'Step':>4} {'y1':>8} {'y2':>8} {'u1':>8} {'u2':>8} {'Status':>8}")
    print("-" * 60)
    
    for step in range(n_steps):
        setpoint = setpoints[step, :]
        
        # Execute control step
        result = interface.control_step(setpoint)
        
        # Store results
        measurements[step, :] = result["measurement"]
        inputs[step, :] = result["input"]
        results.append(result)
        
        # Print progress
        status = "OK" if result["success"] else "FALLBACK"
        print(
            f"{step:4d} "
            f"{result['measurement'][0]:8.3f} "
            f"{result['measurement'][1]:8.3f} "
            f"{result['input'][0]:8.3f} "
            f"{result['input'][1]:8.3f} "
            f"{status:>8}"
        )
        
        # Wait for next control interval
        time.sleep(1.0)
    
    print("-" * 60)
    
    # Stop everything
    print("\n8. Shutting down...")
    interface.disconnect()
    twin.stop()
    
    # Plot results
    print("9. Plotting results...")
    plot_results(measurements, inputs, setpoints)
    
    # Print summary
    solver_failures = sum(not r["success"] for r in results)
    mean_solve_time = np.mean([r["solve_time"] for r in results])
    
    # Calculate final errors
    final_error = measurements[-1, :] - setpoints[-1, :]
    max_error = np.max(np.abs(measurements - setpoints), axis=0)
    
    print("\n" + "=" * 70)
    print("Integration Test Complete")
    print("=" * 70)
    print(f"Total steps: {n_steps}")
    print(f"Solver failures: {solver_failures}")
    print(f"Mean solve time: {mean_solve_time*1000:.2f} ms")
    print(f"Final outputs: {measurements[-1, :]}")
    print(f"Final setpoints: {setpoints[-1, :]}")
    print(f"Final error: {final_error}")
    print(f"Max absolute error: {max_error}")


def plot_results(measurements, inputs, setpoints):
    """Plot the integration test results."""
    n_steps = len(measurements)
    time_axis = np.arange(n_steps)
    
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    
    # Plot outputs
    for i in range(2):
        ax = axes[i]
        ax.plot(time_axis, measurements[:, i], 'b-', linewidth=2, label=f'Measured Output {i+1}')
        ax.plot(time_axis, setpoints[:, i], 'r--', linewidth=1.5, label=f'Setpoint {i+1}')
        ax.set_ylabel(f'Output {i+1}')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best')
        ax.set_title(f'Output {i+1} Tracking (Step Changes)')
    
    # Plot inputs
    for i in range(2):
        ax = axes[2 + i]
        ax.step(time_axis, inputs[:, i], 'g-', where='post', linewidth=1.5, label=f'Input {i+1}')
        ax.set_ylabel(f'Input {i+1}')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best')
        ax.set_title(f'Input {i+1}')
    
    axes[-1].set_xlabel('Time (steps)')
    plt.suptitle('MPC Control with Realistic Step Changes', fontsize=14)
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    # Save the plot
    plt.savefig('opcua_integration_results_v3.png', dpi=150, bbox_inches='tight')
    print("Plot saved to: opcua_integration_results_v3.png")
    
    plt.show()


def main():
    """Main entry point."""
    try:
        run_integration_test()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\n\nError: {type(e).__name__}: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
