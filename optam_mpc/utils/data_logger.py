"""Data logging utilities for OptAM-MPC.

This module provides data logging capabilities for MPC applications.
It records controller performance data for later analysis and reporting.
"""

import csv
import json
import time
from dataclasses import dataclass, field, asdict
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Dict, Any


@dataclass
class ControlRecord:
    """Represents one control cycle record.
    
    Attributes
    ----------
    timestamp : float
        Unix timestamp.
    step : int
        Control step number.
    cvs : list of float
        Controlled variable values.
    setpoints : list of float
        Setpoint values.
    mvs : list of float
        Manipulated variable values.
    solver_success : bool
        Whether the solver succeeded.
    solve_time_ms : float
        Solver computation time in milliseconds.
    objective : float
        Objective function value.
    bias : list of float, optional
        Bias correction values.
    """
    timestamp: float
    step: int
    cvs: List[float]
    setpoints: List[float]
    mvs: List[float]
    solver_success: bool
    solve_time_ms: float
    objective: float = 0.0
    bias: Optional[List[float]] = None
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        return asdict(self)
    
    def to_csv_row(self) -> List[Any]:
        """Convert to CSV row."""
        row = [
            self.timestamp,
            self.step,
            *self.cvs,
            *self.setpoints,
            *self.mvs,
            self.solver_success,
            self.solve_time_ms,
            self.objective,
        ]
        if self.bias is not None:
            row.extend(self.bias)
        return row


class DataLogger:
    """Logs MPC controller performance data.
    
    This class provides flexible data logging to multiple formats:
    - CSV files (for spreadsheet analysis)
    - JSON files (for programmatic analysis)
    
    Parameters
    ----------
    log_dir : str
        Directory for log files.
    controller_name : str
        Name of the controller (used in filenames).
    cv_names : list of str
        Names of controlled variables.
    mv_names : list of str
        Names of manipulated variables.
    log_format : str
        Format to log: 'csv', 'json', or 'both'.
    """
    
    def __init__(
        self,
        log_dir: str = "logs",
        controller_name: str = "mpc_controller",
        cv_names: Optional[List[str]] = None,
        mv_names: Optional[List[str]] = None,
        log_format: str = "csv",
    ):
        """Initialize the data logger."""
        self.log_dir = Path(log_dir)
        self.controller_name = controller_name
        self.cv_names = cv_names or []
        self.mv_names = mv_names or []
        self.log_format = log_format.lower()
        
        # Create log directory if it doesn't exist
        self.log_dir.mkdir(parents=True, exist_ok=True)
        
        # Data storage
        self.records: List[ControlRecord] = []
        
        # File handles
        self._csv_file = None
        self._csv_writer = None
        self._json_file = None
        
        # Generate unique filename based on timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.base_filename = f"{controller_name}_{timestamp}"
        
    def start(self):
        """Start logging and open files."""
        if "csv" in self.log_format:
            csv_path = self.log_dir / f"{self.base_filename}.csv"
            self._csv_file = open(csv_path, 'w', newline='')
            self._csv_writer = csv.writer(self._csv_file)
            self._write_csv_header()
            
        if "json" in self.log_format:
            json_path = self.log_dir / f"{self.base_filename}.json"
            self._json_file = open(json_path, 'w')
            self._json_file.write('{"records": [\n')
            
        print(f"Data logging started. Files in: {self.log_dir}")
        if "csv" in self.log_format:
            print(f"  CSV: {self.base_filename}.csv")
        if "json" in self.log_format:
            print(f"  JSON: {self.base_filename}.json")
            
    def _write_csv_header(self):
        """Write CSV header row."""
        header = [
            "timestamp",
            "step",
        ]
        header.extend([f"CV_{name}" for name in self.cv_names] or 
                      [f"CV_{i+1}" for i in range(len(self.records[0].cvs) if self.records else 1)])
        header.extend([f"SP_{name}" for name in self.cv_names] or 
                      [f"SP_{i+1}" for i in range(len(self.records[0].setpoints) if self.records else 1)])
        header.extend([f"MV_{name}" for name in self.mv_names] or 
                      [f"MV_{i+1}" for i in range(len(self.records[0].mvs) if self.records else 1)])
        header.extend([
            "solver_success",
            "solve_time_ms",
            "objective",
        ])
        
        if self.records and self.records[0].bias is not None:
            header.extend([f"Bias_{i+1}" for i in range(len(self.records[0].bias))])
            
        self._csv_writer.writerow(header)
        
    def log(self, record: ControlRecord):
        """Log a control record.
        
        Parameters
        ----------
        record : ControlRecord
            The control record to log.
        """
        self.records.append(record)
        
        # Write to CSV if active
        if self._csv_writer is not None:
            self._csv_writer.writerow(record.to_csv_row())
            self._csv_file.flush()
            
        # Write to JSON if active
        if self._json_file is not None:
            if len(self.records) > 1:
                self._json_file.write(',\n')
            json.dump(record.to_dict(), self._json_file)
            self._json_file.flush()
            
    def stop(self):
        """Stop logging and close files."""
        if self._csv_file is not None:
            self._csv_file.close()
            self._csv_file = None
            
        if self._json_file is not None:
            self._json_file.write('\n]}\n')
            self._json_file.close()
            self._json_file = None
            
        print(f"Data logging stopped. {len(self.records)} records saved.")
        
    def get_records(self) -> List[ControlRecord]:
        """Get all logged records."""
        return self.records.copy()
    
    def get_statistics(self) -> Dict[str, Any]:
        """Calculate statistics from logged data.
        
        Returns
        -------
        dict
            Statistics including:
            - Number of records
            - Solver success rate
            - Mean solve time
            - CV tracking errors
        """
        if not self.records:
            return {"n_records": 0}
        
        n = len(self.records)
        solver_successes = sum(1 for r in self.records if r.solver_success)
        mean_solve_time = sum(r.solve_time_ms for r in self.records) / n
        
        # Calculate tracking errors
        cv_errors = []
        for record in self.records:
            errors = [cv - sp for cv, sp in zip(record.cvs, record.setpoints)]
            cv_errors.append(errors)
        
        # Transpose to get errors per CV
        cv_errors_by_var = list(zip(*cv_errors))
        
        stats = {
            "n_records": n,
            "solver_success_rate": solver_successes / n * 100,
            "mean_solve_time_ms": mean_solve_time,
            "cv_rmse": [np.sqrt(np.mean(np.array(e)**2)) for e in cv_errors_by_var],
            "cv_max_error": [max(abs(e) for e in errors) for errors in cv_errors_by_var],
        }
        
        return stats
    
    def print_summary(self):
        """Print a summary of logged data."""
        stats = self.get_statistics()
        
        print("\n" + "=" * 60)
        print("DATA LOGGING SUMMARY")
        print("=" * 60)
        print(f"Total records: {stats['n_records']}")
        print(f"Solver success rate: {stats['solver_success_rate']:.1f}%")
        print(f"Mean solve time: {stats['mean_solve_time_ms']:.2f} ms")
        
        if 'cv_rmse' in stats:
            print("\nControlled Variable Performance:")
            for i, (rmse, max_err) in enumerate(zip(stats['cv_rmse'], stats['cv_max_error'])):
                cv_name = self.cv_names[i] if i < len(self.cv_names) else f"CV_{i+1}"
                print(f"  {cv_name}: RMSE={rmse:.4f}, Max Error={max_err:.4f}")
        
        print("=" * 60)


# Import numpy here to avoid circular import
import numpy as np
