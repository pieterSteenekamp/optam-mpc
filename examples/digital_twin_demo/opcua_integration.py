"""Complete OPC UA integration example.

This example demonstrates:
1. Starting a digital twin with OPC UA server
2. Connecting an MPC controller via OPC UA client
3. Running a closed-loop control simulation
4. Monitoring performance in real-time

The architecture:
    MPC Controller ←→ OPC UA Client ←→ OPC UA Server ←→ Digital Twin
"""

import time
import threading
import numpy as np
import matplotlib.pyplot as plt

from optam_mpc.core.models import MIMOFOPDT
from optam_mpc.core.mpc import MPCController
from optam_mpc.interfaces.opcua_server import DigitalTwinServer
from optam_mpc.interfaces.opcua_client import OPCUAControllerInterface


def create_process_models():
    """Create plant and controller models.

    Returns
    -------
    tuple
        (plant_model, controller_model)
    """
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
    """Create MPC controller.

    Returns
    -------
    MPCController
        Configured MPC controller.
    """
    _, model = create_process_models()
    
    config = {
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
    }
    
    return MPCController(model, config)


def run_integration_test():
    """Run the complete OPC UA integration test."""
    print("=" * 70)
    print("OptAM-MPC: OPC UA Integration Test")
    print("=" * 70)
    
    # Create models
    plant_model, _ = create_process_models()
    
    # Initialize the process
    y0 = np.array([0.0, 0.0])
    u0 = np.array([0.0, 0.0])
    plant_model.reset(y0, u0)
    
    # Create digital twin server
    print("\n1. Starting Digital Twin Server...")
    twin = DigitalTwinServer(
        process=plant_model,
        endpoint="opc.tcp://localhost:4840",
        name="Digital Twin",
        update_interval=0.5,  # Faster than control interval for responsiveness
        noise_std=[0.01, 0.01],  # Add measurement noise
    )
    
    # Create MPC controller
    print("\n2. Creating MPC Controller...")
    controller = create_controller()
    
    # Create OPC UA interface
    print("\n3. Setting up OPC UA Client...")
    interface = OPCUAControllerInterface(
        controller=controller,
        endpoint="opc.tcp://localhost:4840",
    )
    
    # Start the digital twin
    print("\n4. Starting Digital Twin...")
    twin.start()
    time.sleep(1.0)  # Wait for server to start
    
    # Connect the controller
    print("\n5. Connecting MPC Controller...")
    interface.connect()
    
    # Initialize
    print("\n6. Initializing...")
    interface.initialize(y0, u0)
    time.sleep(1.0)  # Wait for initialization to propagate
    
    # Define setpoint trajectory
    n_steps = 40
    setpoints = np.zeros((n_steps, 2))
    setpoints[5:, 0] = 1.0    # Output 1 setpoint change at step 5
    setpoints[20:, 1] = 0.5   # Output 2 setpoint change at step 20
    
    # Arrays to store results
    measurements = np.zeros((n_steps, 2))
    inputs = np.zeros((n_steps, 2))
    actual_setpoints = np.zeros((n_steps, 2))
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
        actual_setpoints[step, :] = setpoint
        results.append(result)
        
        # Print progress
        status = "OK" if result["success"] else "FALLBACK"
        if step % 5 == 0 or not result["success"]:
            print(
                f"{step:4d} "
                f"{result['measurement'][0]:8.3f} "
                f"{result['measurement'][1]:8.3f} "
                f"{result['input'][0]:8.3f} "
                f"{result['input'][1]:8.3f} "
                f"{status:>8}"
            )
        
        # Wait for next control interval
        time.sleep(1.0)  # 1 second control interval
    
    print("-" * 60)
    
    # Stop everything
    print("\n8. Shutting down...")
    interface.disconnect()
    twin.stop()
    
    # Plot results
    print("\n9. Plotting results...")
    plot_results(measurements, inputs, actual_setpoints)
    
    # Print summary
    solver_failures = sum(not r["success"] for r in results)
    mean_solve_time = np.mean([r["solve_time"] for r in results])
    
    print("\n" + "=" * 70)
    print("Integration Test Complete")
    print("=" * 70)
    print(f"Total steps: {n_steps}")
    print(f"Solver failures: {solver_failures}")
    print(f"Mean solve time: {mean_solve_time*1000:.2f} ms")
    print(f"Final outputs: {measurements[-1, :]}")
    print(f"Final setpoints: {setpoints[-1, :]}")
    print(f"Final error: {measurements[-1, :] - setpoints[-1, :]}")


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
        ax.set_title(f'Output {i+1} Tracking (via OPC UA)')
    
    # Plot inputs
    for i in range(2):
        ax = axes[2 + i]
        ax.step(time_axis, inputs[:, i], 'g-', where='post', linewidth=1.5, label=f'Input {i+1}')
        ax.set_ylabel(f'Input {i+1}')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best')
        ax.set_title(f'Input {i+1} (via OPC UA)')
    
    axes[-1].set_xlabel('Time (steps)')
    plt.suptitle('MPC Control via OPC UA Communication', fontsize=14)
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    # Save the plot
    plt.savefig('opcua_integration_results.png', dpi=150, bbox_inches='tight')
    print("Plot saved to: opcua_integration_results.png")
    
    plt.show()


def main():
    """Main entry point."""
    try:
        run_integration_test()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\n\nError: {type(e).__name__}: {e}")
        raise


if __name__ == "__main__":
    main()
