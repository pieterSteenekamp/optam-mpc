from __future__ import annotations

from pathlib import Path
import tempfile
import unittest

from optam_mpc_linear import (
    ConfigurationError,
    DerivedRateCalculator,
    LinearIntegratorModel,
    load_definition,
)


ROOT = Path(__file__).resolve().parents[1]
APPLICATION = ROOT / "examples" / "01_surge_vessel" / "application.toml"


class SurgeVesselBuilderTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.definition = load_definition(APPLICATION)
        cls.model = LinearIntegratorModel(cls.definition)

    def test_no_measured_wild_inlet_flow(self):
        self.assertEqual(
            [item for item in self.definition.sources if item.kind == "dv"], []
        )

    def test_nominal_source_values_give_zero_rate(self):
        rates = self.model.rates_per_minute(self.model.source_initial)
        self.assertEqual(rates["level"], 0.0)

    def test_nominal_model_step_is_steady(self):
        result = self.model.step(self.model.primary_initial, self.model.source_initial)
        self.assertEqual(result, self.model.primary_initial)

    def test_positive_outlet_step_matches_first_principles(self):
        sources = dict(self.model.source_initial)
        sources["outlet_flow"] += 5.0
        self.assertAlmostEqual(
            self.model.rates_per_minute(sources)["level"],
            -5.0 / (60.0 * 4.0), places=14,
        )

    def test_negative_outlet_step_matches_first_principles(self):
        sources = dict(self.model.source_initial)
        sources["outlet_flow"] -= 5.0
        self.assertAlmostEqual(
            self.model.rates_per_minute(sources)["level"],
            5.0 / (60.0 * 4.0), places=14,
        )

    def test_one_sample_level_change_has_correct_units(self):
        sources = dict(self.model.source_initial)
        sources["outlet_flow"] += 5.0
        result = self.model.step(self.model.primary_initial, sources)
        expected = 50.0 - 5.0 / (60.0 * 4.0) * (5.0 / 60.0)
        self.assertAlmostEqual(result["level"], expected, places=14)

    def test_predicted_derived_rate_comes_from_level_trajectory(self):
        sources = dict(self.model.source_initial)
        sources["outlet_flow"] += 5.0
        current = self.model.step(self.model.primary_initial, sources)
        derived = self.model.predicted_derived_values(self.model.primary_initial, current)
        self.assertAlmostEqual(derived["level_rate"], -5.0 / 240.0, places=12)


class DerivedMeasurementTests(unittest.TestCase):
    def test_first_valid_update_has_zero_rate(self):
        calculator = DerivedRateCalculator(0.30, 5.0, 50.0)
        filtered, rate = calculator.update(50.2)
        self.assertAlmostEqual(filtered, 50.06)
        self.assertEqual(rate, 0.0)

    def test_second_rising_update_has_positive_rate(self):
        calculator = DerivedRateCalculator(0.30, 5.0, 50.0)
        calculator.update(50.0)
        _filtered, rate = calculator.update(50.1)
        self.assertGreater(rate, 0.0)


class ValidationMessageTests(unittest.TestCase):
    def test_bad_destination_has_engineering_error(self):
        original = APPLICATION.read_text(encoding="utf-8")
        changed = original.replace('destination = "level"', 'destination = "missing"')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.toml"
            path.write_text(changed, encoding="utf-8")
            with self.assertRaises(ConfigurationError) as caught:
                load_definition(path)
        self.assertIn(
            "Model path outlet_flow -> missing: destination does not identify a primary CV.",
            caught.exception.errors,
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
