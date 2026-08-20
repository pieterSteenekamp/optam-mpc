"""Controller mode management example.

This example demonstrates:
1. Starting in MANUAL mode
2. Switching to AUTO mode
3. Operator intervention (back to MANUAL)
4. Returning to AUTO
5. Bumpless transfer
"""

import time
import numpy as np

from optam_mpc.core.models import MIMOFOPDT
from optam_mpc.core.mpc import MPCController
from optam_mpc.core.controller_modes import ControllerMode, ModeManager, BumplessTransfer
from optam_mpc.interfaces.opcua_server import DigitalTwinServer
from optam_mpc.interfaces.opcua_client import OPCUAControllerInterface


def main():
    """Run controller mode management example."""
    print("=" * 70)
    print("OptAM-MPC: Controller Mode Management")
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
        "bias_filter": 0.3,
    })
    
    # Create mode manager
    mode_manager = ModeManager(initial_mode=ControllerMode.MANUAL)
    bumpless = BumplessTransfer(nu=2)
    
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
    
    # Start everything
    print("\n1. Starting Digital Twin...")
    twin.start()
    time.sleep(2.0)
    
    print("2. Connecting Controller...")
    interface.connect()
    
    print("3. Initializing...")
    interface.initialize(y0, u0)
    time.sleep(1.5)
    
    # Define setpoints
    n_steps = 60
    setpoints = np.zeros((n_steps, 2))
    setpoints[5:, 0] = 0.8
    setpoints[30:, 1] = 0.4
    
    print("\n4. Running Control Loop...")
    print("\nPhase 1: MANUAL mode (steps 0-9)")
    print("Phase 2: AUTO mode (steps 10-39)")
    print("Phase 3: MANUAL mode - operator intervention (steps 40-49)")
    print("Phase 4: AUTO mode with bumpless transfer (steps 50-59)")
    print("-" * 70)
    
    manual_mvs = np.array([0.1, 0.1])  # Operator-set MVs in manual mode
    
    try:
        for step in range(n_steps):
            # Handle mode changes at specific steps
            if step == 10:
                mode_manager.set_auto("Operator switched to AUTO")
                # Start bumpless transfer from manual MVs to controller MVs
                bumpless.start_transition(
                    current_mvs=manual_mvs,
                    target_mvs=np.array([0.0, 0.0]),
                    steps=5,
                )
            elif step == 40:
                mode_manager.set_manual("Operator intervention required")
            elif step == 50:
                mode_manager.set_auto("Operator returned to AUTO")
                bumpless.start_transition(
                    current_mvs=manual_mvs,
                    target_mvs=np.array([0.0, 0.0]),
                    steps=5,
                )
            
            # Calculate control action based on mode
            if mode_manager.is_manual():
                # Operator controls MVs directly
                mvs = manual_mvs.copy()
                result = {
                    "measurement": interface.read_outputs(),
                    "setpoint": setpoints[step, :],
                    "input": mvs,
                    "success": True,
                    "message": "MANUAL mode",
                    "solve_time": 0.0,
                }
                interface.write_inputs(mvs)
            else:
                # AUTO mode - MPC controls MVs
                setpoint = setpoints[step, :]
                result = interface.control_step(setpoint)
                
                # Apply bumpless transfer if transitioning
                if bumpless.is_transitioning():
                    smooth_mvs = bumpless.get_next_mvs()
                    interface.write_inputs(smooth_mvs)
                    result["input"] = smooth_mvs
            
            # Print status
            mode_str = mode_manager.current_mode.value.upper()
            print(f"Step {step:2d} [{mode_str:8s}] "
                  f"CVs=[{result['measurement'][0]:.3f}, {result['measurement'][1]:.3f}] "
                  f"MVs=[{result['input'][0]:.3f}, {result['input'][1]:.3f}]")
            
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        print("\n\nStopping...")
    
    finally:
        # Show mode status
        print("\n" + "=" * 70)
        mode_manager.print_status()
        
        # Cleanup
        interface.disconnect()
        twin.stop()
        print("\nShutdown complete")


if __name__ == "__main__":
    main()
