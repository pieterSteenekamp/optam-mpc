"""Real-time monitoring dashboard for OptAM-MPC.

This module provides a web-based dashboard for monitoring MPC controller
performance in real-time. It uses Flask and SocketIO for live updates.
"""

import threading
import time
from typing import Optional, List, Dict, Any
from datetime import datetime

import numpy as np
from flask import Flask, render_template_string, jsonify
from flask_socketio import SocketIO

from optam_mpc.interfaces.opcua_client import OPCUAControllerInterface


class MPCMonitor:
    """Real-time monitoring for MPC controller.

    This class provides a web dashboard that displays:
    - Current output values vs. setpoints
    - Control input values
    - Solver performance metrics
    - Historical trends

    Parameters
    ----------
    interface : OPCUAControllerInterface
        The OPC UA interface connected to the controller.
    port : int
        Port for the web dashboard.
    history_length : int
        Number of data points to keep in history.
    """

    def __init__(
        self,
        interface: OPCUAControllerInterface,
        port: int = 5000,
        history_length: int = 100,
    ):
        """Initialize the monitor.

        Parameters
        ----------
        interface : OPCUAControllerInterface
            The OPC UA interface to monitor.
        port : int
            Port for the web dashboard.
        history_length : int
            Number of data points to keep in history.
        """
        self.interface = interface
        self.port = port
        self.history_length = history_length
        
        # Data storage
        self.measurements_history: List[np.ndarray] = []
        self.setpoints_history: List[np.ndarray] = []
        self.inputs_history: List[np.ndarray] = []
        self.solve_times: List[float] = []
        self.success_flags: List[bool] = []
        
        # Flask app
        self.app = Flask(__name__)
        self.socketio = SocketIO(self.app, cors_allowed_origins="*")
        
        # Setup routes
        self._setup_routes()
        
    def _setup_routes(self):
        """Set up Flask routes."""
        
        @self.app.route('/')
        def index():
            return self._get_dashboard_html()
        
        @self.app.route('/api/current')
        def current_data():
            return jsonify(self._get_current_data())
        
        @self.app.route('/api/history')
        def history_data():
            return jsonify(self._get_history_data())
    
    def _get_dashboard_html(self) -> str:
        """Get the dashboard HTML template."""
        return """
        <!DOCTYPE html>
        <html>
        <head>
            <title>OptAM-MPC Monitor</title>
            <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
            <script src="https://cdn.socket.io/4.5.0/socket.io.min.js"></script>
            <style>
                body {
                    font-family: Arial, sans-serif;
                    margin: 20px;
                    background-color: #f5f5f5;
                }
                .header {
                    text-align: center;
                    padding: 20px;
                    background-color: #2c3e50;
                    color: white;
                    border-radius: 5px;
                }
                .container {
                    display: grid;
                    grid-template-columns: repeat(2, 1fr);
                    gap: 20px;
                    margin-top: 20px;
                }
                .card {
                    background-color: white;
                    padding: 20px;
                    border-radius: 5px;
                    box-shadow: 0 2px 5px rgba(0,0,0,0.1);
                }
                .metric {
                    font-size: 2em;
                    font-weight: bold;
                    color: #2c3e50;
                    text-align: center;
                    margin: 10px 0;
                }
                .label {
                    font-size: 0.9em;
                    color: #7f8c8d;
                    text-align: center;
                }
                canvas {
                    max-height: 300px;
                }
                .status-ok {
                    color: #27ae60;
                }
                .status-fallback {
                    color: #e74c3c;
                }
            </style>
        </head>
        <body>
            <div class="header">
                <h1>OptAM-MPC Real-time Monitor</h1>
                <p>Live controller performance</p>
            </div>



            <div class="container">
                <div class="card">
                    <h3>Controlled Variables (CVs)</h3>
                    <canvas id="cvChart"></canvas>
                </div>
                <div class="card">
                    <h3>Manipulated Variables (MVs)</h3>
                    <canvas id="mvChart"></canvas>
                </div>
                <div class="card">
                    <h3>Performance Metrics</h3>
                    <div id="metrics">
                        <div class="metric" id="solveTime">0 ms</div>
                        <div class="label">Mean Solve Time</div>
                        <br>
                        <div class="metric" id="successRate">100%</div>
                        <div class="label">Success Rate</div>
                    </div>
                </div>
                <div class="card">
                    <h3>Current Status</h3>
                    <div id="status">
                        <p>CV 1: <span id="cv1">0.000</span></p>
                        <p>Setpoint 1: <span id="setpoint1">0.000</span></p>
                        <p>MV 1: <span id="mv1">0.000</span></p>
                        <br>
                        <p>CV 2: <span id="cv2">0.000</span></p>
                        <p>Setpoint 2: <span id="setpoint2">0.000</span></p>
                        <p>MV 2: <span id="mv2">0.000</span></p>
                    </div>
                </div>
            </div>




            










            
            <script>
                const socket = io();
                
                let outputChart, inputChart;
                









                let cvChart, mvChart;
                
                const cvColors = [
                    'rgb(75, 192, 192)',
                    'rgb(54, 162, 235)',
                    'rgb(255, 159, 64)',
                    'rgb(153, 102, 255)'
                ];
                const setpointColors = [
                    'rgb(255, 99, 132)',
                    'rgb(255, 159, 64)',
                    'rgb(255, 205, 86)',
                    'rgb(201, 203, 207)'
                ];
                const mvColors = [
                    'rgb(54, 162, 235)',
                    'rgb(255, 159, 64)',
                    'rgb(75, 192, 192)',
                    'rgb(153, 102, 255)'
                ];
                
                function updateCharts(data) {
                    const labels = data.time;
                    const cvs = data.outputs;      // CVs
                    const setpoints = data.setpoints;
                    const mvs = data.inputs;       // MVs
                    
                    // Create CV chart datasets
                    const cvDatasets = [];
                    for (let i = 0; i < data.n_outputs; i++) {
                        cvDatasets.push({
                            label: 'CV ' + (i + 1),
                            data: cvs[i],
                            borderColor: cvColors[i % cvColors.length],
                            backgroundColor: cvColors[i % cvColors.length],
                            tension: 0.1,
                            fill: false
                        });
                        cvDatasets.push({
                            label: 'Setpoint ' + (i + 1),
                            data: setpoints[i],
                            borderColor: setpointColors[i % setpointColors.length],
                            borderDash: [5, 5],
                            tension: 0.1,
                            fill: false
                        });
                    }
                    
                    if (!cvChart) {
                        const ctx1 = document.getElementById('cvChart').getContext('2d');
                        cvChart = new Chart(ctx1, {
                            type: 'line',
                            data: {
                                labels: labels,
                                datasets: cvDatasets
                            },
                            options: {
                                responsive: true,
                                animation: {
                                    duration: 0
                                },
                                scales: {
                                    y: {
                                        beginAtZero: true
                                    }
                                }
                            }
                        });
                    } else {
                        cvChart.data.labels = labels;
                        cvChart.data.datasets = cvDatasets;
                        cvChart.update();
                    }
                    
                    // Create MV chart datasets
                    const mvDatasets = [];
                    for (let i = 0; i < data.n_inputs; i++) {
                        mvDatasets.push({
                            label: 'MV ' + (i + 1),
                            data: mvs[i],
                            borderColor: mvColors[i % mvColors.length],
                            backgroundColor: mvColors[i % mvColors.length],
                            tension: 0.1,
                            fill: false,
                            steppedLine: 'after'
                        });
                    }
                    
                    if (!mvChart) {
                        const ctx2 = document.getElementById('mvChart').getContext('2d');
                        mvChart = new Chart(ctx2, {
                            type: 'line',
                            data: {
                                labels: labels,
                                datasets: mvDatasets
                            },
                            options: {
                                responsive: true,
                                animation: {
                                    duration: 0
                                }
                            }
                        });
                    } else {
                        mvChart.data.labels = labels;
                        mvChart.data.datasets = mvDatasets;
                        mvChart.update();
                    }
                }













                
                function updateMetrics(data) {
                    document.getElementById('solveTime').textContent = 
                        data.mean_solve_time.toFixed(2) + ' ms';
                    document.getElementById('successRate').textContent = 
                        data.success_rate.toFixed(1) + '%';
                }



                function updateStatus(data) {
                    if (data.current) {
                        document.getElementById('cv1').textContent = 
                            data.current.outputs[0].toFixed(3);
                        document.getElementById('setpoint1').textContent = 
                            data.current.setpoints[0].toFixed(3);
                        document.getElementById('mv1').textContent = 
                            data.current.inputs[0].toFixed(3);
                        
                        if (data.current.outputs.length > 1) {
                            document.getElementById('cv2').textContent = 
                                data.current.outputs[1].toFixed(3);
                            document.getElementById('setpoint2').textContent = 
                                data.current.setpoints[1].toFixed(3);
                            document.getElementById('mv2').textContent = 
                                data.current.inputs[1].toFixed(3);
                        }
                    }
                }
                
                socket.on('update', function(data) {
                    updateCharts(data);
                    updateMetrics(data);
                    updateStatus(data);
                });
                
                // Initial load
                fetch('/api/history')
                    .then(response => response.json())
                    .then(data => {
                        updateCharts(data);
                        updateMetrics(data);
                    });
            </script>
        </body>
        </html>
        """
    
    def _get_current_data(self) -> Dict[str, Any]:
        """Get current data for API."""
        if self.measurements_history:
            return {
                "outputs": self.measurements_history[-1].tolist(),
                "setpoints": self.setpoints_history[-1].tolist(),
                "inputs": self.inputs_history[-1].tolist(),
            }
        return {"outputs": [], "setpoints": [], "inputs": []}
    # add here
    def _get_history_data(self) -> Dict[str, Any]:
        """Get historical data for API.
        
        Returns data in a format suitable for Chart.js with separate
        arrays for each output and input.
        """
        n_outputs = len(self.measurements_history[0]) if self.measurements_history else 0
        n_inputs = len(self.inputs_history[0]) if self.inputs_history else 0
        
        # Organize data by output/input index
        outputs_by_index = []
        setpoints_by_index = []
        inputs_by_index = []
        
        for i in range(n_outputs):
            outputs_by_index.append([m[i] for m in self.measurements_history])
            setpoints_by_index.append([s[i] for s in self.setpoints_history])
        
        for i in range(n_inputs):
            inputs_by_index.append([u[i] for u in self.inputs_history])
        
        return {
            "time": list(range(len(self.measurements_history))),
            "outputs": outputs_by_index,
            "setpoints": setpoints_by_index,
            "inputs": inputs_by_index,
            "n_outputs": n_outputs,
            "n_inputs": n_inputs,
            "mean_solve_time": np.mean(self.solve_times) * 1000 if self.solve_times else 0,
            "success_rate": np.mean(self.success_flags) * 100 if self.success_flags else 100,
        }
    
    def update(self, measurement: np.ndarray, setpoint: np.ndarray, input_value: np.ndarray, result: Dict):
        """Update the monitor with new data.

        Parameters
        ----------
        measurement : np.ndarray
            Current measurement.
        setpoint : np.ndarray
            Current setpoint.
        input_value : np.ndarray
            Current input.
        result : dict
            Control result from interface.
        """
        self.measurements_history.append(measurement.copy())
        self.setpoints_history.append(setpoint.copy())
        self.inputs_history.append(input_value.copy())
        self.solve_times.append(result.get("solve_time", 0.0))
        self.success_flags.append(result.get("success", True))
        
        # Trim history
        if len(self.measurements_history) > self.history_length:
            self.measurements_history = self.measurements_history[-self.history_length:]
            self.setpoints_history = self.setpoints_history[-self.history_length:]
            self.inputs_history = self.inputs_history[-self.history_length:]
            self.solve_times = self.solve_times[-self.history_length:]
            self.success_flags = self.success_flags[-self.history_length:]
        
        # Emit update
        data = self._get_history_data()
        data["current"] = self._get_current_data()
        self.socketio.emit('update', data)
    
    def start(self):
        """Start the monitor in a background thread."""
        def run_server():
            try:
                self.socketio.run(
                    self.app,
                    port=self.port,
                    debug=False,
                    use_reloader=False,
                    allow_unsafe_werkzeug=True,
                )
            except Exception as e:
                print(f"Monitor server stopped: {e}")
        
        threading.Thread(target=run_server, daemon=True).start()
        print(f"Monitor started at http://localhost:{self.port}")
        
    def stop(self):
        """Stop the monitor."""
        # The SocketIO server runs in a daemon thread, so it will
        # be automatically terminated when the main program exits.
        # We just need to clean up any references.
        self.socketio = None
        print("Monitor stopped")
            
