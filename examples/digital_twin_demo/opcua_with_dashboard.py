"""OPC UA integration with real-time monitoring dashboard."""

import time
import numpy as np
import threading

from optam_mpc.core.models import MIMOFOPDT
from optam_mpc.core.mpc import MPCController
from optam_mpc.interfaces.opcua_server import DigitalTwinServer
from optam_mpc.interfaces.opcua_client import OPCUAControllerInterface
from optam_mpc.interfaces.dashboard import MPCMonitor


def main():
    """Run integration with dashboard."""
    print("=" * 70)
    print("OptAM-MPC: OPC UA with Dashboard")
    print("=" * 70)
    
    # Create models
    dt = 1.0
    plant_model = MIMOFOPDT(
        K=np.array([[1.5, 0.5], [-0.2, 1.0]]),
        tau=np.array([[5.0, 3.0], [2.0, 4.0]]),
        theta=np.array([[2.0, 1.0], [0.0, 1.5]]),
        dt=dt,
    )
    controller_model = MIMOFOPDT(
        K=np.array([[1.4, 0.5], [-0.2, 0.9]]),
        tau=np.array([[5.0, 3.0], [2.0, 4.0]]),
        theta=np.array([[2.0, 1.0], [0.0, 1.5]]),
        dt=dt,
    )
    
    # Initialize
    y0 = np.array([0.0, 0.0])
    u0 = np.array([0.0, 0.0])
    plant_model.reset(y0, u0)
    
    # Create controller
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
        "output_min": [-0.5, -0.5],
        "output_max": [1.5, 1.0],
        "bias_filter": 0.3,
    })
    
    # Create OPC UA interface
    interface = OPCUAControllerInterface(
        controller=controller,
        endpoint="opc.tcp://localhost:4840",
    )
    
    # Create digital twin
    twin = DigitalTwinServer(
        process=plant_model,
        endpoint="opc.tcp://localhost:4840",
        name="Digital Twin",
        update_interval=1.0,
        noise_std=[0.01, 0.01],
    )
    
    # Create monitor
    monitor = MPCMonitor(interface, port=5000)
    
    # Start everything
    print("\n1. Starting Digital Twin...")
    twin.start()
    time.sleep(2.0)
    
    print("2. Connecting Controller...")
    interface.connect()
    
    print("3. Starting Monitor...")
    monitor.start()
    time.sleep(2.0)
    
    print("4. Initializing...")
    interface.initialize(y0, u0)
    time.sleep(1.5)
    
    print("\n5. Running Control Loop...")
    print("Open your browser to http://localhost:5000")
    print("Press Ctrl+C to stop")
    
    # Setpoints with step changes
    n_steps = 60
    setpoints = np.zeros((n_steps, 2))
    setpoints[5:, 0] = 0.8
    setpoints[25:, 1] = 0.4
    setpoints[40:, 0] = 0.5
    
    try:
        for step in range(n_steps):
            setpoint = setpoints[step, :]
            result = interface.control_step(setpoint)
            
            # Update monitor
            monitor.update(
                result["measurement"],
                setpoint,
                result["input"],
                result,
            )
            
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        interface.disconnect()
        twin.stop()
        monitor.stop()
        print("Shutdown complete")


if __name__ == "__main__":
    main()
