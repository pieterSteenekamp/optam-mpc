"""OPC UA integration with data logging."""

import time
import numpy as np

from optam_mpc.core.models import MIMOFOPDT
from optam_mpc.core.mpc import MPCController
from optam_mpc.interfaces.opcua_server import DigitalTwinServer
from optam_mpc.interfaces.opcua_client import OPCUAControllerInterface
from optam_mpc.utils.data_logger import DataLogger, ControlRecord


def main():
    """Run integration with data logging."""
    print("=" * 70)
    print("OptAM-MPC: OPC UA with Data Logging")
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
    
    # Create data logger
    logger = DataLogger(
        log_dir="logs",
        controller_name="tank_controller",
        cv_names=["concentration", "temperature"],
        mv_names=["flow_rate", "coolant_temp"],
        log_format="csv",  # Use CSV for easy Excel analysis
    )
    
    # Start everything
    print("\n1. Starting Digital Twin...")
    twin.start()
    time.sleep(2.0)
    
    print("2. Connecting Controller...")
    interface.connect()
    
    print("3. Starting Data Logger...")
    logger.start()
    
    print("4. Initializing...")
    interface.initialize(y0, u0)
    time.sleep(1.5)
    
    print("\n5. Running Control Loop...")
    
    # Setpoints with step changes
    n_steps = 30
    setpoints = np.zeros((n_steps, 2))
    setpoints[5:, 0] = 0.8
    setpoints[15:, 1] = 0.4
    setpoints[25:, 0] = 0.5
    
    try:
        for step in range(n_steps):
            setpoint = setpoints[step, :]
            result = interface.control_step(setpoint)
            
            # Create control record and log it
            record = ControlRecord(
                timestamp=time.time(),
                step=step,
                cvs=result["measurement"].tolist(),
                setpoints=setpoint.tolist(),
                mvs=result["input"].tolist(),
                solver_success=result["success"],
                solve_time_ms=result["solve_time"] * 1000,
                objective=0.0,  # Would need to extract from controller result
                bias=controller._bias.tolist() if hasattr(controller, '_bias') else None,
            )
            logger.log(record)
            
            # Print progress
            if step % 5 == 0:
                print(f"Step {step}: CVs={result['measurement']}, MVs={result['input']}")
            
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        print("\n\nStopping...")
    
    finally:
        # Stop logging and show summary
        logger.stop()
        logger.print_summary()
        
        # Cleanup
        interface.disconnect()
        twin.stop()
        print("\nShutdown complete")


if __name__ == "__main__":
    main()
