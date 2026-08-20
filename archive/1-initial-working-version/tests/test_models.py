"""Tests for process models."""

import math

import numpy as np
import pytest

from optam_mpc.core.models import (
    Process,
    MIMOFOPDT,
    MIMO_FOPDT,  # Backwards compatibility alias
    IntegratingTank,
    NonlinearCSTR,
    CSTRParameters,
)


class TestMIMOFOPDT:
    """Test suite for the MIMO FOPDT model."""

    def test_initialization(self):
        """Test basic initialization and attribute assignment."""
        K = np.array([[1.5, 0.5], [-0.2, 1.0]])
        tau = np.array([[5.0, 3.0], [2.0, 4.0]])
        theta = np.array([[2.0, 1.0], [0.0, 1.5]])
        
        model = MIMOFOPDT(K, tau, theta, dt=1.0)
        
        assert model.ny == 2
        assert model.nu == 2
        assert model.dt == 1.0
        assert np.array_equal(model.K, K)
        assert np.array_equal(model.tau, tau)
        assert np.array_equal(model.theta, theta)

    def test_invalid_shapes(self):
        """Test that mismatched shapes raise ValueError."""
        K = np.array([[1.0, 0.5]])
        tau = np.array([[5.0]])  # Wrong shape
        theta = np.array([[2.0, 1.0]])
        
        with pytest.raises(ValueError, match="identical"):
            MIMOFOPDT(K, tau, theta, dt=1.0)

    def test_invalid_time_constants(self):
        """Test that non-positive time constants raise ValueError."""
        K = np.array([[1.0]])
        tau = np.array([[0.0]])  # Must be positive
        theta = np.array([[0.0]])
        
        with pytest.raises(ValueError, match="positive"):
            MIMOFOPDT(K, tau, theta, dt=1.0)

    def test_invalid_dead_time(self):
        """Test that negative dead time raises ValueError."""
        K = np.array([[1.0]])
        tau = np.array([[1.0]])
        theta = np.array([[-1.0]])  # Must be non-negative
        
        with pytest.raises(ValueError, match="non-negative"):
            MIMOFOPDT(K, tau, theta, dt=1.0)

    def test_fractional_dead_time_response(self):
        """Test exact fractional FOPDT response.

        For K=1, tau=1, theta=0.5, dt=1.0:
        After one step with u=1, the response should be 1 - exp(-0.5)
        because the new input acts for only 0.5 time units.
        """
        model = MIMOFOPDT(
            K=np.array([[1.0]]),
            tau=np.array([[1.0]]),
            theta=np.array([[0.5]]),
            dt=1.0,
        )
        model.reset(np.array([0.0]), np.array([0.0]))
        response = model.step(np.array([1.0]))[0]
        
        expected = 1.0 - math.exp(-0.5)
        assert math.isclose(response, expected, rel_tol=0.0, abs_tol=1e-12)

    def test_steady_state_initialization(self):
        """Test that initialization at arbitrary operating point is steady."""
        model = MIMOFOPDT(
            K=np.array([[2.0, -0.5]]),
            tau=np.array([[3.0, 4.0]]),
            theta=np.array([[0.0, 0.75]]),
            dt=0.5,
        )
        
        y0 = np.array([7.0])
        u0 = np.array([1.2, -0.3])
        model.reset(y0, u0)
        
        # At steady state, applying the same input should not change output
        y1 = model.step(u0)[0]
        assert math.isclose(y1, y0[0], rel_tol=0.0, abs_tol=1e-12)

    def test_clone_independence(self):
        """Test that cloning creates an independent copy."""
        model = MIMOFOPDT(
            K=np.array([[1.0]]),
            tau=np.array([[1.0]]),
            theta=np.array([[0.0]]),
            dt=1.0,
        )
        model.reset(np.array([0.0]), np.array([0.0]))
        clone = model.clone()
        
        # Modify clone
        clone.step(np.array([1.0]))
        
        # Original should be unchanged
        assert model.y[0] == 0.0
        assert clone.y[0] != 0.0

    def test_backwards_compatibility_alias(self):
        """Test that MIMO_FOPDT alias works."""
        assert MIMO_FOPDT is MIMOFOPDT


class TestIntegratingTank:
    """Test suite for the IntegratingTank model."""

    def test_fractional_delay_integration(self):
        """Test exact fractional delay for integrator."""
        model = IntegratingTank(K=1.0, theta=0.25, dt=1.0)
        model.reset(np.array([0.0]), np.array([0.0]))
        
        # With K=1, theta=0.25, dt=1.0:
        # The input acts for 0.75 time units
        integrated = model.step(np.array([1.0]))[0]
        assert math.isclose(integrated, 0.75, rel_tol=0.0, abs_tol=1e-12)

    def test_pure_integration(self):
        """Test that integrator accumulates input over time."""
        model = IntegratingTank(K=1.0, theta=0.0, dt=1.0)
        model.reset(np.array([0.0]), np.array([0.0]))
        
        model.step(np.array([1.0]))
        model.step(np.array([1.0]))
        y = model.step(np.array([1.0]))[0]
        
        assert math.isclose(y, 3.0, rel_tol=0.0, abs_tol=1e-12)

    def test_clone_independence(self):
        """Test that cloning creates an independent copy."""
        model = IntegratingTank(K=1.0, theta=0.0, dt=1.0)
        model.reset(np.array([0.0]), np.array([0.0]))
        clone = model.clone()
        
        clone.step(np.array([1.0]))
        
        assert model.y[0] == 0.0
        assert clone.y[0] == 1.0


class TestNonlinearCSTR:
    """Test suite for the NonlinearCSTR model."""

    def test_equilibrium_calculation(self):
        """Test that equilibrium calculation gives a steady state."""
        model = NonlinearCSTR(dt=0.5, integration_substeps=2)
        y0, u0 = model.equilibrium_for_temperature_and_flow(320.0, 10.0)
        
        model.reset(y0, u0)
        y1 = model.step(u0)
        
        # At equilibrium, the state should not change
        assert np.allclose(y1, y0, rtol=0.0, atol=1e-10)

    def test_initialization_validation(self):
        """Test that invalid initial conditions raise errors."""
        model = NonlinearCSTR(dt=0.5)
        
        # Temperature below absolute zero
        with pytest.raises(ValueError, match="absolute zero"):
            model.reset(np.array([1.0, 0.0]), np.array([10.0, 300.0]))

    def test_clone_independence(self):
        """Test that cloning creates an independent copy."""
        model = NonlinearCSTR(dt=0.5, integration_substeps=2)
        y0, u0 = model.equilibrium_for_temperature_and_flow(320.0, 10.0)
        model.reset(y0, u0)
        
        clone = model.clone()
        clone.step(u0 + np.array([1.0, 0.0]))
        
        # Original should remain at equilibrium
        assert np.allclose(model.y, y0, rtol=0.0, atol=1e-10)


# Run tests if executed directly
if __name__ == "__main__":
    pytest.main([__file__, "-v"])
