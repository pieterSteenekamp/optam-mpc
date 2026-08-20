"""Controller mode management for OptAM-MPC.

This module provides controller mode management essential for industrial
deployment. It handles:
- MANUAL mode: Operator directly controls MVs
- AUTO mode: MPC controller controls MVs
- CASCADE mode: MPC receives setpoints from higher-level controller
- Bumpless transfer between modes
"""

from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from typing import Optional, List, Callable

import numpy as np


class ControllerMode(Enum):
    """Controller operating modes."""
    MANUAL = "manual"
    AUTO = "auto"
    CASCADE = "cascade"


@dataclass
class ModeTransition:
    """Records a controller mode transition.
    
    Attributes
    ----------
    from_mode : ControllerMode
        Previous mode.
    to_mode : ControllerMode
        New mode.
    timestamp : datetime
        When the transition occurred.
    reason : str
        Why the transition happened.
    """
    from_mode: ControllerMode
    to_mode: ControllerMode
    timestamp: datetime = field(default_factory=datetime.now)
    reason: str = ""


class ModeManager:
    """Manages controller operating modes and transitions.
    
    This class handles:
    - Current mode tracking
    - Mode transition validation
    - Bumpless transfer logic
    - Mode transition history
    
    Parameters
    ----------
    initial_mode : ControllerMode
        Initial controller mode (default: MANUAL).
    """
    
    def __init__(self, initial_mode: ControllerMode = ControllerMode.MANUAL):
        """Initialize the mode manager."""
        self.current_mode = initial_mode
        self.previous_mode = initial_mode
        self.transition_history: List[ModeTransition] = []
        
        # Callbacks for mode changes
        self._callbacks: List[Callable[[ControllerMode, ControllerMode], None]] = []
        
        # Track when mode was last changed
        self.last_change_time = datetime.now()
        self.time_in_current_mode = 0.0
        
        # Bumpless transfer variables
        self._last_manual_mvs: Optional[np.ndarray] = None
        self._last_auto_mvs: Optional[np.ndarray] = None
        
    def add_callback(self, callback: Callable[[ControllerMode, ControllerMode], None]):
        """Add a callback for mode changes.
        
        Parameters
        ----------
        callback : callable
            Function called with (old_mode, new_mode) on transition.
        """
        self._callbacks.append(callback)
        
    def _notify_callbacks(self, old_mode: ControllerMode, new_mode: ControllerMode):
        """Notify all callbacks of a mode change."""
        for callback in self._callbacks:
            try:
                callback(old_mode, new_mode)
            except Exception as e:
                print(f"Mode callback error: {e}")
                
    def can_transition(self, new_mode: ControllerMode) -> bool:
        """Check if a transition to the new mode is allowed.
        
        Parameters
        ----------
        new_mode : ControllerMode
            Desired new mode.
            
        Returns
        -------
        bool
            True if transition is allowed.
        """
        # Define allowed transitions
        # From MANUAL: can go to AUTO or stay in MANUAL
        # From AUTO: can go to MANUAL or CASCADE
        # From CASCADE: can go to AUTO or MANUAL
        
        if self.current_mode == ControllerMode.MANUAL:
            return new_mode in [ControllerMode.MANUAL, ControllerMode.AUTO]
        elif self.current_mode == ControllerMode.AUTO:
            return new_mode in [ControllerMode.MANUAL, ControllerMode.AUTO, ControllerMode.CASCADE]
        elif self.current_mode == ControllerMode.CASCADE:
            return new_mode in [ControllerMode.MANUAL, ControllerMode.AUTO, ControllerMode.CASCADE]
        return False
        
    def request_mode_change(
        self,
        new_mode: ControllerMode,
        reason: str = "",
    ) -> bool:
        """Request a controller mode change.
        
        Parameters
        ----------
        new_mode : ControllerMode
            Desired new mode.
        reason : str
            Reason for the mode change.
            
        Returns
        -------
        bool
            True if transition was successful.
        """
        if not self.can_transition(new_mode):
            print(f"Invalid mode transition: {self.current_mode.value} -> {new_mode.value}")
            return False
        
        if new_mode == self.current_mode:
            print(f"Already in {new_mode.value} mode")
            return False
        
        # Record transition
        transition = ModeTransition(
            from_mode=self.current_mode,
            to_mode=new_mode,
            reason=reason,
        )
        self.transition_history.append(transition)
        
        # Update mode
        old_mode = self.current_mode
        self.previous_mode = old_mode
        self.current_mode = new_mode
        self.last_change_time = datetime.now()
        
        # Notify callbacks
        self._notify_callbacks(old_mode, new_mode)
        
        print(f"Mode changed: {old_mode.value} -> {new_mode.value}" + 
              (f" ({reason})" if reason else ""))
        
        return True
        
    def set_manual(self, reason: str = "Operator request"):
        """Switch to MANUAL mode."""
        return self.request_mode_change(ControllerMode.MANUAL, reason)
        
    def set_auto(self, reason: str = "Operator request"):
        """Switch to AUTO mode."""
        return self.request_mode_change(ControllerMode.AUTO, reason)
        
    def set_cascade(self, reason: str = "Supervisory request"):
        """Switch to CASCADE mode."""
        return self.request_mode_change(ControllerMode.CASCADE, reason)
        
    def is_manual(self) -> bool:
        """Check if in MANUAL mode."""
        return self.current_mode == ControllerMode.MANUAL
        
    def is_auto(self) -> bool:
        """Check if in AUTO mode."""
        return self.current_mode == ControllerMode.AUTO
        
    def is_cascade(self) -> bool:
        """Check if in CASCADE mode."""
        return self.current_mode == ControllerMode.CASCADE
        
    def update_time_in_mode(self, dt: float):
        """Update time spent in current mode.
        
        Parameters
        ----------
        dt : float
            Time since last update (seconds).
        """
        self.time_in_current_mode += dt
        
    def get_transition_history(self, limit: int = 20) -> List[ModeTransition]:
        """Get recent mode transitions."""
        return self.transition_history[-limit:]
        
    def print_status(self):
        """Print current mode status."""
        print(f"Controller Mode: {self.current_mode.value.upper()}")
        print(f"Previous Mode: {self.previous_mode.value.upper()}")
        print(f"Time in current mode: {self.time_in_current_mode:.1f} seconds")
        print(f"Total transitions: {len(self.transition_history)}")
        
        if self.transition_history:
            print("\nRecent transitions:")
            for transition in self.transition_history[-5:]:
                print(f"  {transition.timestamp.strftime('%H:%M:%S')}: "
                      f"{transition.from_mode.value} -> {transition.to_mode.value}"
                      + (f" ({transition.reason})" if transition.reason else ""))


class BumplessTransfer:
    """Handles bumpless transfer between controller modes.
    
    Bumpless transfer ensures that when switching modes, there are no
    sudden changes in manipulated variables that could upset the process.
    
    Parameters
    ----------
    nu : int
        Number of manipulated variables.
    """
    
    def __init__(self, nu: int):
        """Initialize bumpless transfer."""
        self.nu = nu
        self._last_mvs = np.zeros(nu)
        self._target_mvs = np.zeros(nu)
        self._transitioning = False
        self._transition_steps = 0
        self._current_step = 0
        
    def start_transition(self, current_mvs: np.ndarray, target_mvs: np.ndarray, steps: int = 5):
        """Start a bumpless transition.
        
        Parameters
        ----------
        current_mvs : np.ndarray
            Current manipulated variable values.
        target_mvs : np.ndarray
            Target manipulated variable values.
        steps : int
            Number of steps for smooth transition.
        """
        self._last_mvs = current_mvs.copy()
        self._target_mvs = target_mvs.copy()
        self._transitioning = True
        self._transition_steps = max(1, steps)
        self._current_step = 0
        
    def get_next_mvs(self) -> np.ndarray:
        """Get the next MVs during transition.
        
        Returns
        -------
        np.ndarray
            Next MVs in the transition sequence.
        """
        if not self._transitioning:
            return self._target_mvs.copy()
        
        # Calculate interpolation factor
        alpha = (self._current_step + 1) / self._transition_steps
        alpha = min(1.0, alpha)
        
        # Interpolate between current and target
        next_mvs = self._last_mvs + alpha * (self._target_mvs - self._last_mvs)
        
        # Update step counter
        self._current_step += 1
        if self._current_step >= self._transition_steps:
            self._transitioning = False
            next_mvs = self._target_mvs.copy()
        
        return next_mvs
        
    def is_transitioning(self) -> bool:
        """Check if transition is in progress."""
        return self._transitioning
        
    def cancel_transition(self):
        """Cancel an ongoing transition."""
        self._transitioning = False
