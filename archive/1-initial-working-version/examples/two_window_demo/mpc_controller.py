"""MPC Controller - Run in Window 2.

This script runs the MPC controller as an OPC UA client.
It connects to the process simulator running in another window.

Usage:
    python mpc_controller.py
"""

import time
import numpy as np

from optam_mpc.core.models import MIMOFOPDT
from optam_mpc.core.mpc import MPCController
from optam_mpc.interfaces.opcua_client import OPCUAControllerInterface


def main():
    """Run the MPC controller."""
    print("=" * 70)
    print("MPC CONTROLLER")
    print("=" * 70)
    
    # Create controller model (with slight mismatch to demonstrate bias correction)
    dt = 1.0
    controller_model = MIMOFOPDT(
        K=np.array([[1.4, 0.5], [-0.2, 0.9]]),  # Slight mismatch
        tau=np.array([[5.0, 3.0], [2.0, 4.0]]),
        theta=np.array([[2.0, 1.0], [0.0, 1.5]]),
        dt=dt,
    )
    
    # Create MPC controller
    controller = MPCController(controller_model, {
        "prediction_horizon": 20,
        "control_horizon": 5,
        "output_weights": [1.0, 1.0],
        "move_weights": [0.5, 0.5],
        "terminal_weights": [5.0, 5.0],
        "output_scale": [1.0, 0.5],
        "input_scale": [1.0, 1.0],
        "input_min": [-2.0, -2.0],
        "input_max": [2.0, 2.0],
        "move_min": [-0.3, -0.3],
        "move_max": [0.3, 0.3],
        "bias_filter": 0.3,
    })
    
    # Create OPC UA interface
    interface = OPCUAControllerInterface(
        controller=controller,
        endpoint="opc.tcp://localhost:4840",
    )
    
    # Connect to process simulator
    print("\nConnecting to process simulator...")
    print("OPC UA Endpoint: opc.tcp://localhost:4840")
    
    try:
        interface.connect()
        print("Connected successfully!")
    except Exception as e:
        print(f"Failed to connect: {e}")
        print("\nMake sure the process simulator is running first!")
        return
    
    # Initialize controller
    y0 = np.array([0.0, 0.0])
    u0 = np.array([0.0, 0.0])
    interface.initialize(y0, u0)
    print("\nController initialized")
    
    # Define setpoint trajectory
    n_steps = 100
    setpoints = np.zeros((n_steps, 2))
    setpoints[10:, 0] = 0.8    # Step change in CV1 at step 10
    setpoints[40:, 1] = 0.4    # Step change in CV2 at step 40
    setpoints[70:, 0] = 0.5    # Another change in CV1 at step 70
    
    print("\nStarting control loop...")
    print("Press Ctrl+C to stop")
    print("-" * 70)
    print(f"{'Step':>4} {'CV1':>8} {'CV2':>8} {'SP1':>8} {'SP2':>8} {'MV1':>8} {'MV2':>8} {'Status':>8}")
    
    try:
        for step in range(n_steps):
            setpoint = setpoints[step, :]
            
            # Execute control step
            result = interface.control_step(setpoint)
            
            # Display status
            status = "OK" if result["success"] else "FALLBACK"
            print(
                f"{step:4d} "
                f"{result['measurement'][0]:8.3f} "
                f"{result['measurement'][1]:8.3f} "
                f"{setpoint[0]:8.3f} "
                f"{setpoint[1]:8.3f} "
                f"{result['input'][0]:8.3f} "
                f"{result['input'][1]:8.3f} "
                f"{status:>8}"
            )
            
            # Wait for next control interval
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        print("\n\nStopping controller...")
    
    finally:
        interface.disconnect()
        print("Controller stopped")


if __name__ == "__main__":
    main()
