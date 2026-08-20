"""Tests for the MPC controller."""

import numpy as np
import pytest

from optam_mpc.core.models import MIMOFOPDT, IntegratingTank, NonlinearCSTR
from optam_mpc.core.mpc import MPCController, MPCResult


class TestMPCControllerInitialization:
    """Test suite for MPC controller initialization."""

    def test_basic_initialization(self):
        """Test that controller initializes with valid configuration."""
        model = MIMOFOPDT(
            K=np.array([[1.0]]),
            tau=np.array([[5.0]]),
            theta=np.array([[1.0]]),
            dt=1.0,
        )
        config = {
            "prediction_horizon": 10,
            "control_horizon": 3,
        }
        
        controller = MPCController(model, config)
        
        assert controller.ny == 1
        assert controller.nu == 1
        assert controller.prediction_horizon == 10
        assert controller.control_horizon == 3

    def test_invalid_horizons(self):
        """Test that invalid horizons raise ValueError."""
        model = MIMOFOPDT(
            K=np.array([[1.0]]),
            tau=np.array([[5.0]]),
            theta=np.array([[0.0]]),
            dt=1.0,
        )
        
        # Control horizon > prediction horizon
        with pytest.raises(ValueError, match="control_horizon"):
            MPCController(model, {
                "prediction_horizon": 5,
                "control_horizon": 10,
            })
        
        # Zero prediction horizon
        with pytest.raises(ValueError, match="prediction_horizon"):
            MPCController(model, {
                "prediction_horizon": 0,
                "control_horizon": 1,
            })

    def test_vector_parameter_resolution(self):
        """Test that scalar parameters are expanded to vectors."""
        model = MIMOFOPDT(
            K=np.array([[1.0, 0.5], [0.2, 1.0]]),
            tau=np.array([[5.0, 3.0], [2.0, 4.0]]),
            theta=np.array([[0.0, 0.0], [0.0, 0.0]]),
            dt=1.0,
        )
        
        controller = MPCController(model, {
            "prediction_horizon": 10,
            "control_horizon": 3,
            "output_weights": 1.0,  # Scalar
            "move_weights": 0.1,    # Scalar
        })
        
        # Should be expanded to correct length
        assert controller.output_weights.shape == (2,)
        assert controller.move_weights.shape == (2,)
        assert np.all(controller.output_weights == 1.0)
        assert np.all(controller.move_weights == 0.1)

    def test_invalid_bounds(self):
        """Test that invalid bounds raise ValueError."""
        model = MIMOFOPDT(
            K=np.array([[1.0]]),
            tau=np.array([[5.0]]),
            theta=np.array([[0.0]]),
            dt=1.0,
        )
        
        # Lower bound > upper bound
        with pytest.raises(ValueError, match="lower bound"):
            MPCController(model, {
                "prediction_horizon": 10,
                "control_horizon": 3,
                "input_min": [2.0],
                "input_max": [1.0],
            })


class TestMPCControllerReset:
    """Test suite for controller reset functionality."""

    def test_valid_reset(self):
        """Test that reset works with valid inputs."""
        model = MIMOFOPDT(
            K=np.array([[1.0]]),
            tau=np.array([[5.0]]),
            theta=np.array([[0.0]]),
            dt=1.0,
        )
        controller = MPCController(model, {
            "prediction_horizon": 10,
            "control_horizon": 3,
        })
        
        y0 = np.array([0.0])
        u0 = np.array([0.0])
        controller.reset(y0, u0)
        
        assert controller._initialized
        assert np.all(controller._previous_u == u0)
        assert np.all(controller._bias == 0.0)

    def test_reset_violates_input_bounds(self):
        """Test that reset fails if u0 violates input bounds."""
        model = MIMOFOPDT(
            K=np.array([[1.0]]),
            tau=np.array([[5.0]]),
            theta=np.array([[0.0]]),
            dt=1.0,
        )
        controller = MPCController(model, {
            "prediction_horizon": 10,
            "control_horizon": 3,
            "input_min": [-1.0],
            "input_max": [1.0],
        })
        
        with pytest.raises(ValueError, match="input bounds"):
            controller.reset(
                y0=np.array([0.0]),
                u0=np.array([2.0]),  # Violates upper bound
            )

    def test_control_before_reset(self):
        """Test that control fails if called before reset."""
        model = MIMOFOPDT(
            K=np.array([[1.0]]),
            tau=np.array([[5.0]]),
            theta=np.array([[0.0]]),
            dt=1.0,
        )
        controller = MPCController(model, {
            "prediction_horizon": 10,
            "control_horizon": 3,
        })
        
        with pytest.raises(RuntimeError, match="reset"):
            controller.control(
                y_measured=np.array([0.0]),
                reference=np.array([1.0]),
            )


class TestMPCControllerControl:
    """Test suite for control calculations."""

    def test_simple_tracking(self):
        """Test that controller can track a simple setpoint."""
        model = MIMOFOPDT(
            K=np.array([[1.0]]),
            tau=np.array([[2.0]]),
            theta=np.array([[0.0]]),
            dt=1.0,
        )
        controller = MPCController(model, {
            "prediction_horizon": 10,
            "control_horizon": 3,
            "output_weights": [10.0],
            "move_weights": [0.1],
            "input_min": [-1.0],
            "input_max": [1.0],
        })
        
        controller.reset(y0=np.array([0.0]), u0=np.array([0.0]))
        
        # Request a setpoint change from 0 to 1
        result = controller.control(
            y_measured=np.array([0.0]),
            reference=np.array([1.0]),
        )
        
        assert result.success
        assert not result.fallback_used
        assert result.u.shape == (1,)
        assert np.all(np.isfinite(result.u))
        # Should move input in positive direction to reach setpoint
        assert result.u[0] > 0.0

    def test_constraint_respect(self):
        """Test that controller respects input constraints."""
        model = MIMOFOPDT(
            K=np.array([[1.0]]),
            tau=np.array([[2.0]]),
            theta=np.array([[0.0]]),
            dt=1.0,
        )
        controller = MPCController(model, {
            "prediction_horizon": 10,
            "control_horizon": 3,
            "output_weights": [10.0],
            "move_weights": [0.1],
            "input_min": [-0.5],
            "input_max": [0.5],
            "move_min": [-0.2],
            "move_max": [0.2],
        })
        
        controller.reset(y0=np.array([0.0]), u0=np.array([0.0]))
        
        # Large setpoint change that would require large input
        result = controller.control(
            y_measured=np.array([0.0]),
            reference=np.array([10.0]),
        )
        
        assert result.success
        # Input should be within bounds
        assert -0.5 - 1e-6 <= result.u[0] <= 0.5 + 1e-6
        # Move should be within limits
        assert -0.2 - 1e-6 <= result.u[0] <= 0.2 + 1e-6

    def test_multiple_steps(self):
        """Test that controller can run multiple control cycles."""
        model = MIMOFOPDT(
            K=np.array([[1.0]]),
            tau=np.array([[2.0]]),
            theta=np.array([[0.0]]),
            dt=1.0,
        )
        controller = MPCController(model, {
            "prediction_horizon": 10,
            "control_horizon": 3,
            "output_weights": [10.0],
            "move_weights": [0.1],
            "input_min": [-1.0],
            "input_max": [1.0],
        })
        
        # Simulate a simple control loop
        y = np.array([0.0])
        u = np.array([0.0])
        setpoint = np.array([1.0])
        
        controller.reset(y0=y, u0=u)
        
        # Run 10 control cycles
        for _ in range(10):
            result = controller.control(y_measured=y, reference=setpoint)
            assert result.success
            u = result.u
            # Update plant (simple integration for testing)
            y = y + 0.1 * u
        
        # After 10 steps, should be moving toward setpoint
        assert y[0] > 0.0

    def test_soft_constraints(self):
        """Test that soft output constraints work."""
        model = MIMOFOPDT(
            K=np.array([[0.0]]),  # Zero gain - cannot reach setpoint
            tau=np.array([[1.0]]),
            theta=np.array([[0.0]]),
            dt=1.0,
        )
        controller = MPCController(model, {
            "prediction_horizon": 3,
            "control_horizon": 1,
            "output_weights": [1.0],
            "move_weights": [0.1],
            "output_max": [1.0],
            "soft_output_weights": [100.0],
        })
        
        controller.reset(y0=np.array([2.0]), u0=np.array([0.0]))
        
        # Output starts above maximum - should use slack
        result = controller.control(
            y_measured=np.array([2.0]),
            reference=np.array([2.0]),
        )
        
        assert result.success
        # Slack should be approximately 1.0 (2.0 - 1.0)
        assert 0.9 <= result.predicted_outputs.max() - 1.0 <= 1.1

    def test_bias_correction(self):
        """Test that bias correction tracks model mismatch."""
        # Model has gain 1.0
        model = MIMOFOPDT(
            K=np.array([[1.0]]),
            tau=np.array([[2.0]]),
            theta=np.array([[0.0]]),
            dt=1.0,
        )
        controller = MPCController(model, {
            "prediction_horizon": 10,
            "control_horizon": 3,
            "bias_filter": 0.5,
        })
        
        controller.reset(y0=np.array([0.0]), u0=np.array([0.0]))
        
        # First measurement shows mismatch (actual plant has gain 1.5)
        result = controller.control(
            y_measured=np.array([1.5]),  # Model predicts 1.0
            reference=np.array([1.5]),
        )
        
        # Bias should be updated
        assert abs(result.bias_estimate[0] - 0.5) < 0.1


class TestMPCResult:
    """Test suite for MPC result object."""

    def test_result_attributes(self):
        """Test that result has all expected attributes."""
        model = MIMOFOPDT(
            K=np.array([[1.0]]),
            tau=np.array([[2.0]]),
            theta=np.array([[0.0]]),
            dt=1.0,
        )
        controller = MPCController(model, {
            "prediction_horizon": 10,
            "control_horizon": 3,
        })
        controller.reset(y0=np.array([0.0]), u0=np.array([0.0]))
        
        result = controller.control(
            y_measured=np.array([0.0]),
            reference=np.array([1.0]),
        )
        
        assert isinstance(result, MPCResult)
        assert hasattr(result, "u")
        assert hasattr(result, "success")
        assert hasattr(result, "fallback_used")
        assert hasattr(result, "status")
        assert hasattr(result, "message")
        assert hasattr(result, "objective")
        assert hasattr(result, "iterations")
        assert hasattr(result, "solve_time_seconds")
        assert hasattr(result, "predicted_outputs")
        assert hasattr(result, "planned_inputs")
        assert hasattr(result, "bias_estimate")


# Run tests if executed directly
if __name__ == "__main__":
    pytest.main([__file__, "-v"])
