"""OPC UA server for process simulation.

This module provides an OPC UA server that hosts a process model,
allowing external clients (like the MPC controller) to interact
with the simulated process through standard OPC UA protocol.
"""

import time
import threading
from typing import Optional, List, Dict, Any

import numpy as np
from opcua import Server, ua
from opcua.common.node import Node

from optam_mpc.core.models import Process


class ProcessOPCUAServer:
    """OPC UA server hosting a process model.

    This class creates an OPC UA server that exposes process variables
    (inputs and outputs) to OPC UA clients. It simulates the process
    in real-time, updating outputs based on input changes.

    Parameters
    ----------
    process : Process
        Process model to simulate.
    endpoint : str
        OPC UA endpoint URL (e.g., "opc.tcp://localhost:4840").
    name : str
        Server name.
    update_interval : float
        Simulation update interval in seconds.
    """

    def __init__(
        self,
        process: Process,
        endpoint: str = "opc.tcp://localhost:4840",
        name: str = "Process Simulator",
        update_interval: float = 1.0,
    ):
        """Initialize the OPC UA server.

        Parameters
        ----------
        process : Process
            Process model to simulate.
        endpoint : str
            OPC UA endpoint URL.
        name : str
            Server name.
        update_interval : float
            Simulation update interval in seconds.
        """
        self.process = process
        self.endpoint = endpoint
        self.name = name
        self.update_interval = update_interval
        
        # Server state
        self.server: Optional[Server] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        
        # Node references
        self._input_nodes: List[Node] = []
        self._output_nodes: List[Node] = []
        self._setpoint_nodes: List[Node] = []
        
        # Current values
        self._inputs = np.zeros(process.nu)
        self._outputs = np.zeros(process.ny)
        self._setpoints = np.zeros(process.ny)
        
    def setup(self):
        """Set up the OPC UA server and create address space."""
        self.server = Server()
        self.server.set_endpoint(self.endpoint)
        self.server.set_server_name(self.name)
        
        # Register namespace
        uri = f"http://optam-mpc.org/{self.name.lower().replace(' ', '_')}"
        idx = self.server.register_namespace(uri)
        
        # Create object node
        objects = self.server.get_objects_node()
        process_node = objects.add_object(idx, "Process")
        
        # Create input variables
        self._input_nodes = []
        for i in range(self.process.nu):
            node = process_node.add_variable(
                idx, f"Input{i+1}", float(self._inputs[i])
            )
            node.set_writable(True)
            self._input_nodes.append(node)
        
        # Create output variables
        self._output_nodes = []
        for i in range(self.process.ny):
            node = process_node.add_variable(
                idx, f"Output{i+1}", float(self._outputs[i])
            )
            node.set_writable(False)
            self._output_nodes.append(node)
        
        # Create setpoint variables (optional, for reference)
        self._setpoint_nodes = []
        for i in range(self.process.ny):
            node = process_node.add_variable(
                idx, f"Setpoint{i+1}", float(self._setpoints[i])
            )
            node.set_writable(True)
            self._setpoint_nodes.append(node)
        
        print(f"OPC UA Server '{self.name}' configured at {self.endpoint}")
        print(f"  Inputs: {self.process.nu}")
        print(f"  Outputs: {self.process.ny}")
        
    def start(self):
        """Start the OPC UA server."""
        if self.server is None:
            self.setup()
        
        self.server.start()
        self._running = True
        self._thread = threading.Thread(target=self._run_loop)
        self._thread.daemon = True
        self._thread.start()
        print(f"OPC UA Server started at {self.endpoint}")
        
    def stop(self):
        """Stop the OPC UA server."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self.server is not None:
            self.server.stop()
        print("OPC UA Server stopped")
        
    def _run_loop(self):
        """Main simulation loop (runs in separate thread)."""
        while self._running:
            try:
                # Read inputs from OPC UA
                for i, node in enumerate(self._input_nodes):
                    self._inputs[i] = node.get_value()
                
                # Read setpoints
                for i, node in enumerate(self._setpoint_nodes):
                    self._setpoints[i] = node.get_value()
                
                # Simulate process step
                self._outputs = self.process.step(self._inputs)
                
                # Write outputs to OPC UA
                for i, node in enumerate(self._output_nodes):
                    node.set_value(float(self._outputs[i]))
                
                # Wait for next update
                time.sleep(self.update_interval)
                
            except Exception as e:
                print(f"Simulation error: {e}")
                time.sleep(self.update_interval)
                
    def reset(self, y0: np.ndarray, u0: np.ndarray):
        """Reset the simulation to initial conditions.

        Parameters
        ----------
        y0 : np.ndarray
            Initial outputs.
        u0 : np.ndarray
            Initial inputs.
        """
        self.process.reset(y0, u0)
        self._inputs = u0.copy()
        self._outputs = y0.copy()
        
        # Update OPC UA nodes if server is running
        if self.server is not None:
            for i, node in enumerate(self._input_nodes):
                node.set_value(float(u0[i]))
            for i, node in enumerate(self._output_nodes):
                node.set_value(float(y0[i]))


class DigitalTwinServer(ProcessOPCUAServer):
    """Digital twin server with additional features.

    This extends the basic OPC UA server with digital twin capabilities:
    - Model mismatch simulation
    - Disturbance injection
    - Noise simulation
    - Performance monitoring
    """

    def __init__(
        self,
        process: Process,
        endpoint: str = "opc.tcp://localhost:4840",
        name: str = "Digital Twin",
        update_interval: float = 1.0,
        noise_std: Optional[List[float]] = None,
        disturbance: Optional[np.ndarray] = None,
    ):
        """Initialize the digital twin server.

        Parameters
        ----------
        process : Process
            Process model for the digital twin.
        endpoint : str
            OPC UA endpoint URL.
        name : str
            Server name.
        update_interval : float
            Simulation update interval.
        noise_std : list of float, optional
            Measurement noise standard deviations.
        disturbance : np.ndarray, optional
            Input disturbances over time.
        """
        super().__init__(process, endpoint, name, update_interval)
        self.noise_std = noise_std or [0.0] * process.ny
        self.disturbance = disturbance
        self.step_count = 0
        self.rng = np.random.default_rng(42)
        
    def _run_loop(self):
        """Enhanced simulation loop with noise and disturbances."""
        while self._running:
            try:
                # Read inputs from OPC UA
                for i, node in enumerate(self._input_nodes):
                    self._inputs[i] = node.get_value()
                
                # Apply disturbance if configured
                if self.disturbance is not None and self.step_count < len(self.disturbance):
                    actual_inputs = self._inputs + self.disturbance[self.step_count]
                else:
                    actual_inputs = self._inputs
                
                # Simulate process step
                true_outputs = self.process.step(actual_inputs)
                
                # Add measurement noise
                noise = self.rng.normal(0.0, self.noise_std)
                measured_outputs = true_outputs + noise
                
                # Write outputs to OPC UA
                for i, node in enumerate(self._output_nodes):
                    node.set_value(float(measured_outputs[i]))
                
                self.step_count += 1
                time.sleep(self.update_interval)
                
            except Exception as e:
                print(f"Digital twin error: {e}")
                time.sleep(self.update_interval)
