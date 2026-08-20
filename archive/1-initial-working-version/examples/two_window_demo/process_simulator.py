"""Process Simulator - Run in Window 1.

This script runs the process simulator (digital twin) as an OPC UA server.
The MPC controller in another window will connect to this.

Usage:
    python process_simulator.py
"""

import time
import numpy as np

from optam_mpc.core.models import MIMOFOPDT
from optam_mpc.interfaces.opcua_server import DigitalTwinServer


def main():
    """Run the process simulator."""
    print("=" * 70)
    print("PROCESS SIMULATOR (Digital Twin)")
    print("=" * 70)
    
    # Create process model
    dt = 1.0
    process_model = MIMOFOPDT(
        K=np.array([[1.5, 0.5], [-0.2, 1.0]]),
        tau=np.array([[5.0, 3.0], [2.0, 4.0]]),
        theta=np.array([[2.0, 1.0], [0.0, 1.5]]),
        dt=dt,
    )
    
    # Initial conditions
    y0 = np.array([0.0, 0.0])
    u0 = np.array([0.0, 0.0])
    process_model.reset(y0, u0)
    
    # Create digital twin
    twin = DigitalTwinServer(
        process=process_model,
        endpoint="opc.tcp://localhost:4840",
        name="Process Simulator",
        update_interval=1.0,
        noise_std=[0.02, 0.02],
    )
    
    print("\nStarting process simulator...")
    print("OPC UA Endpoint: opc.tcp://localhost:4840")
    print("\nWaiting for MPC controller to connect...")
    print("Press Ctrl+C to stop")
    print("-" * 70)
    
    try:
        twin.start()
        
        # Display status periodically
        step = 0
        while True:
            time.sleep(1.0)
            step += 1
            
            # Show current state every 10 steps
            if step % 10 == 0:
                print(f"\nSimulator running (step {step})")
                print(f"  Current outputs: {process_model.y}")
                
    except KeyboardInterrupt:
        print("\n\nStopping process simulator...")
        twin.stop()
        print("Simulator stopped")


if __name__ == "__main__":
    main()
