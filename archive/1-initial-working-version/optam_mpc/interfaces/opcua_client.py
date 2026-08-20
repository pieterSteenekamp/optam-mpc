"""OPC UA client for MPC controller communication.

This module provides an OPC UA client that the MPC controller uses to
communicate with a process (real or simulated) through OPC UA protocol.
"""

import time
from typing import Optional, List, Tuple

import numpy as np
from opcua import Client

from optam_mpc.core.mpc import MPCController


class OPCUAControllerInterface:
    """Interface between MPC controller and OPC UA server.

    This class handles the communication between the MPC controller
    and a process exposed through OPC UA. It reads measurements and
    writes control actions.

    Parameters
    ----------
    controller : MPCController
        MPC controller to interface.
    endpoint : str
        OPC UA endpoint URL.
    input_node_names : list of str
        Node names for process inputs.
    output_node_names : list of str
        Node names for process outputs.
    """

    def __init__(
        self,
        controller: MPCController,
        endpoint: str,
        input_node_names: Optional[List[str]] = None,
        output_node_names: Optional[List[str]] = None,
    ):
        """Initialize the OPC UA interface.

        Parameters
        ----------
        controller : MPCController
            MPC controller to interface.
        endpoint : str
            OPC UA endpoint URL.
        input_node_names : list of str
            Names of input nodes (default: Input1, Input2, ...).
        output_node_names : list of str
            Names of output nodes (default: Output1, Output2, ...).
        """
        self.controller = controller
        self.endpoint = endpoint
        self.client: Optional[Client] = None
        
        # Default node names
        if input_node_names is None:
            input_node_names = [f"Input{i+1}" for i in range(controller.nu)]
        if output_node_names is None:
            output_node_names = [f"Output{i+1}" for i in range(controller.ny)]
        
        self.input_node_names = input_node_names
        self.output_node_names = output_node_names
        
        # Node references
        self._input_nodes = []
        self._output_nodes = []
        
        # Verify dimensions
        if len(input_node_names) != controller.nu:
            raise ValueError(
                f"Number of input nodes ({len(input_node_names)}) must match "
                f"number of controller inputs ({controller.nu})"
            )
        if len(output_node_names) != controller.ny:
            raise ValueError(
                f"Number of output nodes ({len(output_node_names)}) must match "
                f"number of controller outputs ({controller.ny})"
            )
    
    def connect(self, timeout: float = 5.0):
        """Connect to the OPC UA server.

        Parameters
        ----------
        timeout : float
            Connection timeout in seconds.
        """
        self.client = Client(self.endpoint)
        self.client.connect()
        # insert here
         # Find process node
        objects = self.client.get_objects_node()
        children = objects.get_children()
        process_node = None
        
        for child in children:
            browse_name = child.get_browse_name()
            # Handle different versions of the opcua library
            if hasattr(browse_name, 'Name'):
                # Newer versions: browse_name.Name is a string
                name = browse_name.Name
            else:
                # Older versions: browse_name.Name.String
                name = browse_name.Name.String
            
            if name == "Process":
                process_node = child
                break
        
        if process_node is None:
            raise RuntimeError("Could not find 'Process' node on OPC UA server")
        
        # Get input nodes
        self._input_nodes = []
        for name in self.input_node_names:
            node = process_node.get_child([f"2:{name}"])
            self._input_nodes.append(node)
        
        # Get output nodes
        self._output_nodes = []
        for name in self.output_node_names:
            node = process_node.get_child([f"2:{name}"])
            self._output_nodes.append(node)       
        
        print(f"Connected to OPC UA server at {self.endpoint}")
        
    def disconnect(self):
        """Disconnect from the OPC UA server."""
        if self.client is not None:
            self.client.disconnect()
            self.client = None
            print("Disconnected from OPC UA server")
    
    def read_outputs(self) -> np.ndarray:
        """Read current output values from the process.

        Returns
        -------
        np.ndarray
            Current output values.
        """
        if self.client is None:
            raise RuntimeError("Not connected to OPC UA server")
        
        outputs = np.array([
            node.get_value() for node in self._output_nodes
        ])
        return outputs
    
    def write_inputs(self, inputs: np.ndarray):
        """Write control inputs to the process.

        Parameters
        ----------
        inputs : np.ndarray
            Input values to write.
        """
        if self.client is None:
            raise RuntimeError("Not connected to OPC UA server")
        
        for node, value in zip(self._input_nodes, inputs):
            node.set_value(float(value))
    
    def initialize(self, y0: np.ndarray, u0: np.ndarray):
        """Initialize the controller with current process state.

        Parameters
        ----------
        y0 : np.ndarray
            Initial outputs.
        u0 : np.ndarray
            Initial inputs.
        """
        self.controller.reset(y0, u0)
        self.write_inputs(u0)
        
    def control_step(self, setpoint: np.ndarray) -> dict:
        """Execute one control step.

        Parameters
        ----------
        setpoint : np.ndarray
            Desired setpoint values.

        Returns
        -------
        dict
            Control result with diagnostics.
        """
        # Read current outputs
        y_measured = self.read_outputs()
        
        # Calculate control action
        result = self.controller.control(y_measured, setpoint)
        
        # Write control action to process
        self.write_inputs(result.u)
        
        return {
            "measurement": y_measured,
            "setpoint": setpoint,
            "input": result.u,
            "success": result.success,
            "message": result.message,
            "solve_time": result.solve_time_seconds,
        }
    
    def run_control_loop(
        self,
        setpoints: np.ndarray,
        sample_time: float,
        n_steps: Optional[int] = None,
    ):
        """Run a control loop for multiple steps.

        Parameters
        ----------
        setpoints : np.ndarray
            Setpoint trajectory, shape (n_steps, ny).
        sample_time : float
            Time between control actions in seconds.
        n_steps : int, optional
            Number of steps to run (default: len(setpoints)).

        Yields
        ------
        dict
            Control results at each step.
        """
        if n_steps is None:
            n_steps = len(setpoints)
        
        for step in range(n_steps):
            setpoint = setpoints[step, :] if setpoints.ndim == 2 else setpoints
            result = self.control_step(setpoint)
            yield result
            time.sleep(sample_time)
