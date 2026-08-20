"""Process models for OptAM-MPC.

This module provides the model interfaces and implementations used by the
MPC controller. Models represent the dynamic behavior of a process and are
used for prediction within the controller.

Classes
-------
Process
    Abstract base class defining the model interface.
MIMOFOPDT
    Multiple-input, multiple-output first-order-plus-dead-time model.
IntegratingTank
    SISO integrating process with dead time.
NonlinearCSTR
    Nonlinear exothermic continuous stirred-tank reactor.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

import numpy as np
from numpy.typing import NDArray

from optam_mpc.utils.validation import (
    require_finite,
    vectorize,
    maximum_bound_violation,
)


# Type alias for numpy arrays
Array = NDArray[np.float64]


class Process(ABC):
    """Abstract base class for process models.

    This class defines the minimal interface required by the MPC controller.
    Subclasses must implement the abstract methods to define their specific
    dynamic behavior.

    Parameters
    ----------
    ny : int
        Number of outputs (controlled variables).
    nu : int
        Number of inputs (manipulated variables).
    dt : float
        Sampling time in seconds.

    Attributes
    ----------
    ny : int
        Number of outputs.
    nu : int
        Number of inputs.
    dt : float
        Sampling time.
    y : Array
        Current output values.
    """

    def __init__(self, ny: int, nu: int, dt: float) -> None:
        """Initialize the process model.

        Parameters
        ----------
        ny : int
            Number of outputs.
        nu : int
            Number of inputs.
        dt : float
            Sampling time in seconds.

        Raises
        ------
        ValueError
            If ny or nu is less than 1, or if dt is not positive and finite.
        """
        if ny < 1 or nu < 1:
            raise ValueError("A process must have at least one output and one input.")
        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError("dt must be a positive finite number.")

        self.ny = int(ny)
        self.nu = int(nu)
        self.dt = float(dt)
        self.y = np.zeros(self.ny, dtype=float)

    def _checked_output(self, y0: Array) -> Array:
        """Validate and convert output values to the correct shape.

        Parameters
        ----------
        y0 : array-like
            Output values to validate.

        Returns
        -------
        Array
            Validated output array with shape (ny,).
        """
        output = vectorize(y0, self.ny, "y0")
        require_finite(output, "y0")
        return output

    def _checked_input(self, u: Array) -> Array:
        """Validate and convert input values to the correct shape.

        Parameters
        ----------
        u : array-like
            Input values to validate.

        Returns
        -------
        Array
            Validated input array with shape (nu,).
        """
        input_value = vectorize(u, self.nu, "u")
        require_finite(input_value, "u")
        return input_value

    @abstractmethod
    def reset(self, y0: Array, u0: Array) -> None:
        """Reset all dynamic state to a condition consistent with y0 and u0.

        Parameters
        ----------
        y0 : array-like
            Initial output values.
        u0 : array-like
            Initial input values.
        """

    @abstractmethod
    def step(self, u: Array) -> Array:
        """Advance the process by one sampling interval.

        Parameters
        ----------
        u : array-like
            Input values to apply.

        Returns
        -------
        Array
            Copy of the updated output values.
        """

    @abstractmethod
    def clone(self) -> "Process":
        """Return an independent deep copy of the process.

        Returns
        -------
        Process
            A new instance with the same configuration and state.
        """
# next
class MIMOFOPDT(Process):
    """Multiple-input, multiple-output first-order-plus-dead-time model.

    This model represents a matrix of independent first-order-plus-dead-time
    (FOPDT) response paths. Each path follows the differential equation::

        tau[i,j] * dx[i,j]/dt + x[i,j] = K[i,j] * u[j]

    The output is the sum of the path states plus a constant operating-point
    offset. Fractional dead time is integrated exactly for zero-order-held
    inputs.

    Parameters
    ----------
    K : array-like, shape (ny, nu)
        Steady-state gain matrix.
    tau : array-like, shape (ny, nu)
        Time constant matrix (must be positive).
    theta : array-like, shape (ny, nu)
        Dead time matrix (must be non-negative).
    dt : float
        Sampling time in seconds.

    Attributes
    ----------
    K : Array
        Gain matrix.
    tau : Array
        Time constant matrix.
    theta : Array
        Dead time matrix.

    Examples
    --------
    >>> K = np.array([[1.5, 0.5], [-0.2, 1.0]])
    >>> tau = np.array([[5.0, 3.0], [2.0, 4.0]])
    >>> theta = np.array([[2.0, 1.0], [0.0, 1.5]])
    >>> model = MIMOFOPDT(K, tau, theta, dt=1.0)
    >>> model.reset(y0=np.zeros(2), u0=np.zeros(2))
    >>> y = model.step(u=np.array([1.0, 0.5]))
    """

    def __init__(self, K: Array, tau: Array, theta: Array, dt: float) -> None:
        """Initialize the MIMO FOPDT model.

        Parameters
        ----------
        K : array-like, shape (ny, nu)
            Gain matrix.
        tau : array-like, shape (ny, nu)
            Time constant matrix (must be positive).
        theta : array-like, shape (ny, nu)
            Dead time matrix (must be non-negative).
        dt : float
            Sampling time in seconds.

        Raises
        ------
        ValueError
            If matrices have incompatible shapes, if time constants are
            not positive, or if dead times are negative.
        """
        # Convert and validate matrix shapes
        gain = np.atleast_2d(np.asarray(K, dtype=float)).copy()
        time_constant = np.atleast_2d(np.asarray(tau, dtype=float)).copy()
        dead_time = np.atleast_2d(np.asarray(theta, dtype=float)).copy()

        if gain.shape != time_constant.shape or gain.shape != dead_time.shape:
            raise ValueError("K, tau, and theta must have identical (ny, nu) shapes.")
        if gain.size == 0:
            raise ValueError("K, tau, and theta must not be empty.")

        # Validate matrix values
        require_finite(gain, "K")
        require_finite(time_constant, "tau")
        require_finite(dead_time, "theta")
        if np.any(time_constant <= 0.0):
            raise ValueError("Every time constant in tau must be positive.")
        if np.any(dead_time < 0.0):
            raise ValueError("Dead times in theta must be non-negative.")

        ny, nu = gain.shape
        super().__init__(ny=ny, nu=nu, dt=dt)

        # Store model parameters
        self.K = gain
        self.tau = time_constant
        self.theta = dead_time

        # Calculate delay discretization
        delay_samples = self.theta / self.dt
        self._whole_delay = np.floor(delay_samples + 1e-12).astype(int)
        self._fractional_delay = self.theta - self._whole_delay * self.dt
        self._fractional_delay[np.abs(self._fractional_delay) < 1e-12] = 0.0
        self._history_length = int(np.max(self._whole_delay)) + 2

        # Initialize state variables
        self._u_history = np.zeros((self._history_length, self.nu), dtype=float)
        self._path_state = np.zeros((self.ny, self.nu), dtype=float)
        self._output_offset = np.zeros(self.ny, dtype=float)

    def reset(self, y0: Array, u0: Array) -> None:
        """Reset the model to a steady-state condition matching y0 and u0.

        Parameters
        ----------
        y0 : array-like, shape (ny,)
            Initial output values.
        u0 : array-like, shape (nu,)
            Initial input values.
        """
        output = self._checked_output(y0)
        input_value = self._checked_input(u0)

        # Initialize every path at steady state for u0, then use a constant
        # operating-point offset to make the total output exactly equal y0.
        self._path_state = self.K * input_value[np.newaxis, :]
        self._output_offset = output - np.sum(self._path_state, axis=1)
        self._u_history = np.tile(input_value, (self._history_length, 1))
        self.y = output.copy()

    def clone(self) -> "MIMOFOPDT":
        """Create an independent deep copy of the model.

        Returns
        -------
        MIMOFOPDT
            A new instance with the same parameters and state.
        """
        other = MIMOFOPDT(self.K, self.tau, self.theta, self.dt)
        other.y = self.y.copy()
        other._path_state = self._path_state.copy()
        other._output_offset = self._output_offset.copy()
        other._u_history = self._u_history.copy()
        return other

    @staticmethod
    def _first_order_update(
        x: float, target: float, duration: float, tau: float
    ) -> float:
        """Update a first-order system over a specified duration.

        Parameters
        ----------
        x : float
            Current state value.
        target : float
            Target value (gain times input).
        duration : float
            Integration time.
        tau : float
            Time constant.

        Returns
        -------
        float
            Updated state value.
        """
        if duration <= 0.0:
            return x
        decay = math.exp(-duration / tau)
        return decay * x + (1.0 - decay) * target

    def step(self, u: Array) -> Array:
        """Advance the model by one sampling interval.

        Parameters
        ----------
        u : array-like, shape (nu,)
            Input values to apply.

        Returns
        -------
        Array
            Copy of the updated output values.
        """
        input_value = self._checked_input(u)

        # Update input history (shift and add new input)
        self._u_history[1:, :] = self._u_history[:-1, :].copy()
        self._u_history[0, :] = input_value

        # Update each path state
        for i in range(self.ny):
            for j in range(self.nu):
                whole = int(self._whole_delay[i, j])
                fraction = float(self._fractional_delay[i, j])
                x = float(self._path_state[i, j])
                gain = float(self.K[i, j])
                tau = float(self.tau[i, j])

                # During the fractional first part of the interval, the older
                # held input is still active. The newer delayed sample acts
                # during the remainder. This is exact under zero-order hold.
                if fraction > 0.0:
                    older_input = self._u_history[whole + 1, j]
                    x = self._first_order_update(
                        x, gain * older_input, fraction, tau
                    )

                newer_input = self._u_history[whole, j]
                x = self._first_order_update(
                    x,
                    gain * newer_input,
                    self.dt - fraction,
                    tau,
                )
                self._path_state[i, j] = x

        # Calculate outputs
        self.y = self._output_offset + np.sum(self._path_state, axis=1)
        return self.y.copy()
#next
class IntegratingTank(Process):
    """Single-input, single-output integrating process with dead time.

    This model represents a simple integrating process::

        dy/dt = K * u(t - theta)

    It is useful for modeling tank levels, inventory systems, and other
    processes where the output integrates the input over time.

    Parameters
    ----------
    K : float
        Process gain (rate of integration per unit input).
    theta : float
        Dead time in seconds (must be non-negative).
    dt : float
        Sampling time in seconds.

    Attributes
    ----------
    K : float
        Process gain.
    theta : float
        Dead time.

    Examples
    --------
    >>> model = IntegratingTank(K=0.2, theta=1.25, dt=0.5)
    >>> model.reset(y0=np.array([2.0]), u0=np.array([0.0]))
    >>> y = model.step(u=np.array([1.0]))
    """

    def __init__(self, K: float, theta: float, dt: float) -> None:
        """Initialize the integrating tank model.

        Parameters
        ----------
        K : float
            Process gain.
        theta : float
            Dead time in seconds.
        dt : float
            Sampling time in seconds.

        Raises
        ------
        ValueError
            If K is not finite, if theta is negative, or if dt is invalid.
        """
        if not np.isfinite(K):
            raise ValueError("K must be finite.")
        if not np.isfinite(theta) or theta < 0.0:
            raise ValueError("theta must be a non-negative finite number.")

        super().__init__(ny=1, nu=1, dt=dt)
        self.K = float(K)
        self.theta = float(theta)

        # Calculate delay discretization
        self._whole_delay = int(math.floor(theta / dt + 1e-12))
        self._fractional_delay = theta - self._whole_delay * dt
        if abs(self._fractional_delay) < 1e-12:
            self._fractional_delay = 0.0

        # Initialize state variables
        self._history_length = self._whole_delay + 2
        self._u_history = np.zeros(self._history_length, dtype=float)

    def reset(self, y0: Array, u0: Array) -> None:
        """Reset the model to a condition consistent with y0 and u0.

        Parameters
        ----------
        y0 : array-like, shape (1,)
            Initial output value.
        u0 : array-like, shape (1,)
            Initial input value.
        """
        self.y = self._checked_output(y0)
        input_value = self._checked_input(u0)
        self._u_history.fill(input_value[0])

    def clone(self) -> "IntegratingTank":
        """Create an independent deep copy of the model.

        Returns
        -------
        IntegratingTank
            A new instance with the same parameters and state.
        """
        other = IntegratingTank(self.K, self.theta, self.dt)
        other.y = self.y.copy()
        other._u_history = self._u_history.copy()
        return other

    def step(self, u: Array) -> Array:
        """Advance the model by one sampling interval.

        Parameters
        ----------
        u : array-like, shape (1,)
            Input value to apply.

        Returns
        -------
        Array
            Copy of the updated output value.
        """
        input_value = self._checked_input(u)

        # Update input history (shift and add new input)
        self._u_history[1:] = self._u_history[:-1].copy()
        self._u_history[0] = input_value[0]

        # Calculate delayed input integral (exact for ZOH with fractional delay)
        fraction = self._fractional_delay
        older_input = self._u_history[self._whole_delay + 1]
        newer_input = self._u_history[self._whole_delay]
        delayed_input_integral = (
            fraction * older_input + (self.dt - fraction) * newer_input
        )

        # Update output (integrate)
        self.y[0] += self.K * delayed_input_integral
        return self.y.copy()

#next
@dataclass(frozen=True)
class CSTRParameters:
    """Physical parameters for the exothermic continuous stirred-tank reactor.

    Parameters
    ----------
    volume : float
        Reactor volume in liters.
    inlet_concentration : float
        Inlet concentration of reactant in mol/L.
    inlet_temperature : float
        Inlet feed temperature in Kelvin.
    pre_exponential_factor : float
        Arrhenius pre-exponential factor in 1/min.
    activation_temperature : float
        Activation energy divided by gas constant (E/R) in Kelvin.
    heat_of_reaction : float
        Heat of reaction in J/mol (negative for exothermic).
    density_heat_capacity : float
        Density times heat capacity in J/L/K.
    heat_transfer : float
        Heat transfer coefficient times area in J/min/K.
    """

    volume: float = 100.0              # L
    inlet_concentration: float = 1.0   # mol/L
    inlet_temperature: float = 350.0   # K
    pre_exponential_factor: float = 7.2e10  # 1/min
    activation_temperature: float = 8750.0  # E/R, K
    heat_of_reaction: float = -5.0e4   # J/mol
    density_heat_capacity: float = 500.0  # J/L/K
    heat_transfer: float = 5.0e4       # J/min/K


class NonlinearCSTR(Process):
    """Nonlinear exothermic continuous stirred-tank reactor.

    This model represents a CSTR with a first-order exothermic reaction.
    Both states (concentration and temperature) are treated as measured
    outputs. The model includes nonlinear Arrhenius kinetics and heat
    transfer dynamics.

    Outputs/states:
        - Concentration [mol/L]
        - Reactor temperature [K]

    Inputs:
        - Feed flow rate [L/min]
        - Coolant temperature [K]

    Parameters
    ----------
    dt : float
        Sampling time in minutes.
    parameters : CSTRParameters, optional
        Physical parameters for the reactor.
    integration_substeps : int, optional
        Number of RK4 substeps per sampling interval (default: 4).

    Examples
    --------
    >>> model = NonlinearCSTR(dt=0.5, integration_substeps=2)
    >>> y0, u0 = model.equilibrium_for_temperature_and_flow(320.0, 10.0)
    >>> model.reset(y0, u0)
    >>> y = model.step(u0)
    """

    def __init__(
        self,
        dt: float,
        parameters: Optional[CSTRParameters] = None,
        integration_substeps: int = 4,
    ) -> None:
        """Initialize the nonlinear CSTR model.

        Parameters
        ----------
        dt : float
            Sampling time in minutes.
        parameters : CSTRParameters, optional
            Physical parameters for the reactor.
        integration_substeps : int, optional
            Number of RK4 substeps per sampling interval.

        Raises
        ------
        ValueError
            If integration_substeps is less than 1, or if parameters are invalid.
        """
        super().__init__(ny=2, nu=2, dt=dt)

        if integration_substeps < 1:
            raise ValueError("integration_substeps must be at least one.")

        self.parameters = parameters or CSTRParameters()
        self.integration_substeps = int(integration_substeps)

        # Validate parameters
        parameter_values = np.asarray(list(vars(self.parameters).values()), dtype=float)
        require_finite(parameter_values, "CSTR parameters")
        positive_indices = [0, 1, 2, 3, 4, 6, 7]
        if np.any(parameter_values[positive_indices] <= 0.0):
            raise ValueError(
                "CSTR volume, kinetic and heat-transfer parameters must be positive."
            )

    def reset(self, y0: Array, u0: Array) -> None:
        """Reset the model to specified initial conditions.

        Parameters
        ----------
        y0 : array-like, shape (2,)
            Initial concentration [mol/L] and temperature [K].
        u0 : array-like, shape (2,)
            Initial feed flow [L/min] and coolant temperature [K].

        Raises
        ------
        ValueError
            If temperature is below absolute zero.
        """
        self._checked_input(u0)
        self.y = self._checked_output(y0)
        if self.y[1] <= 1.0:
            raise ValueError("CSTR temperature must be above absolute zero.")

    def clone(self) -> "NonlinearCSTR":
        """Create an independent deep copy of the model.

        Returns
        -------
        NonlinearCSTR
            A new instance with the same parameters and state.
        """
        other = NonlinearCSTR(self.dt, self.parameters, self.integration_substeps)
        other.y = self.y.copy()
        return other

    def _rate_constant(self, temperature: float) -> float:
        """Calculate the Arrhenius rate constant.

        Parameters
        ----------
        temperature : float
            Reactor temperature in Kelvin.

        Returns
        -------
        float
            Rate constant in 1/min.

        Raises
        ------
        FloatingPointError
            If temperature is invalid or rate constant is non-finite.
        """
        if not np.isfinite(temperature) or temperature <= 1.0:
            raise FloatingPointError("The predicted CSTR temperature is invalid.")
        p = self.parameters
        return p.pre_exponential_factor * math.exp(
            -p.activation_temperature / temperature
        )

    def _derivatives(self, state: Array, u: Array) -> Array:
        """Calculate the time derivatives of the state variables.

        Parameters
        ----------
        state : Array, shape (2,)
            Current concentration [mol/L] and temperature [K].
        u : Array, shape (2,)
            Feed flow [L/min] and coolant temperature [K].

        Returns
        -------
        Array, shape (2,)
            Time derivatives [dC/dt, dT/dt].
        """
        concentration, temperature = state
        flow, coolant_temperature = u
        p = self.parameters

        rate = self._rate_constant(float(temperature)) * concentration
        dilution = flow / p.volume
        heat_release = -p.heat_of_reaction / p.density_heat_capacity
        cooling = p.heat_transfer / (p.volume * p.density_heat_capacity)

        concentration_rate = (
            dilution * (p.inlet_concentration - concentration) - rate
        )
        temperature_rate = (
            dilution * (p.inlet_temperature - temperature)
            + heat_release * rate
            - cooling * (temperature - coolant_temperature)
        )

        derivative = np.array([concentration_rate, temperature_rate], dtype=float)
        if not np.all(np.isfinite(derivative)):
            raise FloatingPointError("The CSTR derivative became non-finite.")
        return derivative

    def step(self, u: Array) -> Array:
        """Advance the model by one sampling interval using RK4 integration.

        Parameters
        ----------
        u : array-like, shape (2,)
            Feed flow [L/min] and coolant temperature [K].

        Returns
        -------
        Array
            Updated concentration [mol/L] and temperature [K].
        """
        input_value = self._checked_input(u)
        state = self.y.copy()
        h = self.dt / self.integration_substeps

        for _ in range(self.integration_substeps):
            # Fourth-order Runge-Kutta integration
            k1 = self._derivatives(state, input_value)
            k2 = self._derivatives(state + 0.5 * h * k1, input_value)
            k3 = self._derivatives(state + 0.5 * h * k2, input_value)
            k4 = self._derivatives(state + h * k3, input_value)
            state = state + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

            if not np.all(np.isfinite(state)) or state[1] <= 1.0:
                raise FloatingPointError("The integrated CSTR state became invalid.")

        self.y = state
        return self.y.copy()

    def equilibrium_for_temperature_and_flow(
        self,
        temperature: float,
        flow: float,
    ) -> tuple[Array, Array]:
        """Calculate equilibrium state and input for specified conditions.

        Parameters
        ----------
        temperature : float
            Desired reactor temperature in Kelvin.
        flow : float
            Desired feed flow rate in L/min.

        Returns
        -------
        tuple[Array, Array]
            Equilibrium state [concentration, temperature] and
            corresponding input [flow, coolant_temperature].

        Raises
        ------
        ValueError
            If flow is not positive.
        """
        if flow <= 0.0:
            raise ValueError("Equilibrium flow must be positive.")

        p = self.parameters
        kinetic_constant = self._rate_constant(temperature)
        dilution = flow / p.volume

        concentration = (
            dilution * p.inlet_concentration / (dilution + kinetic_constant)
        )
        rate = kinetic_constant * concentration
        heat_release = -p.heat_of_reaction / p.density_heat_capacity
        cooling = p.heat_transfer / (p.volume * p.density_heat_capacity)

        coolant_temperature = temperature - (
            dilution * (p.inlet_temperature - temperature) + heat_release * rate
        ) / cooling

        return (
            np.array([concentration, temperature], dtype=float),
            np.array([flow, coolant_temperature], dtype=float),
        )

# Backwards-compatible spelling for users familiar with the original code
MIMO_FOPDT = MIMOFOPDT    
    
    



