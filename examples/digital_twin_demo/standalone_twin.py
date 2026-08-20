"""Standalone digital twin server for SCADA connection.

This script starts a digital twin that runs continuously, allowing
SCADA systems like FUXA to connect via OPC UA.
"""

import time
import numpy as np

from optam_mpc.core.models import MIMOFOPDT
from optam_mpc.interfaces.opcua_server import DigitalTwinServer


def main():
    """Start a standalone digital twin."""
    print("=" * 70)
    print("OptAM-MPC: Standalone Digital Twin for SCADA")
    print("=" * 70)
    
    # Create a simple 2x2 process
    dt = 1.0
    process_model = MIMOFOPDT(
        K=np.array([[1.5, 0.5], [-0.2, 1.0]]),
        tau=np.array([[5.0, 3.0], [2.0, 4.0]]),
        theta=np.array([[2.0, 1.0], [0.0, 1.5]]),
        dt=dt,
    )
    
    # Initialize at steady state
    y0 = np.array([0.0, 0.0])
    u0 = np.array([0.0, 0.0])
    process_model.reset(y0, u0)
    
    # Create digital twin
    twin = DigitalTwinServer(
        process=process_model,
        endpoint="opc.tcp://localhost:4840",
        name="Process Digital Twin",
        update_interval=1.0,
        noise_std=[0.02, 0.02],
    )
    
    print("\nStarting digital twin...")
    print("Endpoint: opc.tcp://localhost:4840")
    print("Press Ctrl+C to stop")
    
    try:
        twin.start()
        
        # Keep running
        while True:
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        print("\n\nStopping digital twin...")
        twin.stop()
        print("Shutdown complete")


if __name__ == "__main__":
    main()
