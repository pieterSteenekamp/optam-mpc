from pathlib import Path
import unittest

from optam_mpc_linear import load_definition
from optam_mpc_linear.controller_adapter import LinearVelocityStateModel


ROOT = Path(__file__).resolve().parents[1]
APPLICATION = ROOT / "examples" / "01_surge_vessel" / "application.toml"


class ControllerAdapterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.definition = load_definition(APPLICATION)
        cls.model = LinearVelocityStateModel(cls.definition)

    def test_dimensions_include_derived_cv_state(self):
        self.assertEqual(self.model.state_dimension, 2)
        self.assertEqual(self.model.output_dimension, 2)
        self.assertEqual(self.model.manipulated_input_dimension, 1)
        self.assertEqual(self.model.disturbance_dimension, 0)

    def test_nominal_transition_is_steady(self):
        self.assertEqual(
            self.model.transition_values((50.0, 0.0), (60.0,), ()),
            (50.0, 0.0),
        )

    def test_positive_outlet_step_predicts_falling_level(self):
        next_state = self.model.transition_values((50.0, 0.0), (65.0,), ())
        self.assertLess(next_state[0], 50.0)
        self.assertAlmostEqual(next_state[1], -5.0 / 240.0, places=14)


if __name__ == "__main__":
    unittest.main(verbosity=2)
