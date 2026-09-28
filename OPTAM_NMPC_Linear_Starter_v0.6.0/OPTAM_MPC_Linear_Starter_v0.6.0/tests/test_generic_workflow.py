"""Prove the runtime can use edited generic templates without examples/."""
from copy import deepcopy
from dataclasses import replace
from pathlib import Path
import os
import shutil
import subprocess
import sys
import tempfile
import tomllib
import unittest

from optam_mpc_linear import ConfigurationError, load_definition
from optam_mpc_linear.definition import DerivedCV
from optam_mpc_linear.builder import verify_model_checks
from optam_mpc_linear.simulation import (
    LinearPlant, Measurements, evaluate_checks, validate_simulation,
)

ROOT = Path(__file__).resolve().parents[1]


def filled_templates():
    app = (ROOT / "application_template.toml").read_text()
    app = app.replace("sample_time_seconds = -1.0", "sample_time_seconds = 5.0")
    app = app.replace("prediction_horizon_samples = 0", "prediction_horizon_samples = 4")
    app = app.replace("control_horizon_samples = 0", "control_horizon_samples = 2")
    app = app.replace("= -1.0", "= 1.0")
    app = app.replace("protected_low = 0.0", "protected_low = -100.0")
    app = app.replace("protected_high = 0.0", "protected_high = 100.0")
    app = app.replace("lower = 0.0", "lower = -10.0").replace("upper = 0.0", "upper = 10.0")
    app = app.replace("integrating_gain = 0.0", "integrating_gain = -0.1")
    sim = (ROOT / "simulation_template.toml").read_text()
    sim = sim.replace("samples = 0", "samples = 4").replace("value = 0.0", "value = 2.0")
    sim = sim.replace("integrating_gain = 0.0", "integrating_gain = -0.1")
    sim = sim.replace("limit = 0.0 # REPLACE: minimum", "limit = -100.0 # REPLACE: minimum")
    sim = sim.replace("limit = 0.0", "limit = 100.0")
    for old, new in {"cv_1": "inventory_a", "cv_2": "inventory_b",
                     "mv_1": "drive_a", "mv_2": "drive_b",
                     "dv_1": "feed_a", "dv_2": "feed_b"}.items():
        app, sim = app.replace(old, new), sim.replace(old, new)
    return app, sim


class GenericWorkflowTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="mpc generic ")
        self.addCleanup(self.temp.cleanup)
        self.folder = Path(self.temp.name)
        app, sim = filled_templates()
        self.app = self.folder / "application.toml"
        self.app.write_text(app)
        self.sim = self.folder / "simulation.toml"
        self.sim.write_text(sim)
        self.definition = load_definition(self.app)
        self.raw = tomllib.loads(sim)
        validate_simulation(self.raw, self.definition)

    def test_two_cvs_mvs_dvs_without_derived(self):
        self.assertEqual(len(self.definition.primary_cvs), 2)
        self.assertEqual(len(self.definition.sources), 4)
        self.assertEqual(len(self.definition.derived_cvs), 0)

    def test_generic_plant_integrates_measured_disturbance(self):
        plant = LinearPlant(self.raw, 5)
        plant.events(0)
        self.assertEqual(plant.measured_dvs()["feed_a"], 2)
        plant.step()
        self.assertAlmostEqual(plant.outputs["y1"], -0.1*2*5/60)
        self.assertEqual(plant.outputs["y2"], 0)

    def test_hidden_input_is_not_exposed_as_measured_dv(self):
        self.raw["plant_inputs"].append(dict(id="unknown_feed", role="hidden", unit="REPLACE", initial=7))
        self.raw["plant_paths"].append(dict(source="unknown_feed", destination="y2",
                                          type="integrator", integrating_gain=1, dead_time_seconds=0))
        self.raw["events"].append(dict(sample=0, input="unknown_feed", value=8))
        validate_simulation(self.raw, self.definition)
        plant = LinearPlant(self.raw, 5)
        plant.events(0)
        self.assertNotIn("unknown_feed", plant.measured_dvs())
        plant.step()
        self.assertAlmostEqual(plant.outputs["y2"], 5/60)

    def test_missing_mapping_rejected(self):
        self.raw["plant_outputs"].pop()
        with self.assertRaises(ConfigurationError):
            validate_simulation(self.raw, self.definition)

    def test_mv_event_rejected(self):
        self.raw["events"][0]["input"] = "u1"
        with self.assertRaises(ConfigurationError):
            validate_simulation(self.raw, self.definition)

    def test_event_at_end_rejected(self):
        self.raw["events"][0]["sample"] = 4
        with self.assertRaises(ConfigurationError):
            validate_simulation(self.raw, self.definition)

    def test_duplicate_event_rejected(self):
        self.raw["events"].append(deepcopy(self.raw["events"][0]))
        with self.assertRaises(ConfigurationError):
            validate_simulation(self.raw, self.definition)

    def test_unknown_plot_signal_rejected(self):
        self.raw["plots"][0]["signals"] = ["missing"]
        with self.assertRaises(ConfigurationError):
            validate_simulation(self.raw, self.definition)

    def test_nonfinite_gain_rejected(self):
        self.raw["plant_paths"][0]["integrating_gain"] = float("nan")
        with self.assertRaises(ConfigurationError):
            validate_simulation(self.raw, self.definition)

    def test_controller_input_initial_mismatch_rejected(self):
        self.raw["plant_inputs"][0]["initial"] = 8
        with self.assertRaises(ConfigurationError):
            validate_simulation(self.raw, self.definition)

    def test_nominal_drift_and_model_mismatch_are_independent(self):
        self.raw["plant_outputs"][0]["nominal_rate_per_minute"] = 2
        self.raw["plant_paths"][0]["integrating_gain"] = 3
        plant = LinearPlant(self.raw, 60)
        plant.set_mvs({"drive_a": 1, "drive_b": 0})
        plant.step()
        self.assertEqual(plant.outputs["y1"], 5)
        self.assertEqual(self.definition.paths[0].integrating_gain, -0.1)

    def test_initial_move_and_final_target_checks(self):
        records = [dict(x=1), dict(x=4), dict(x=2)]
        checks = [
            dict(signal="x", metric="minimum", limit=1),
            dict(signal="x", metric="maximum", limit=4),
            dict(signal="x", metric="max_abs_move", limit=3),
            dict(signal="x", metric="final_error", target=2, limit=0),
        ]
        self.assertTrue(all(x["passed"] for x in evaluate_checks(records, checks)))
        checks[2]["limit"] = 2.9
        self.assertFalse(evaluate_checks(records, checks)[2]["passed"])

    def test_multiple_derived_sources_are_processed_by_id(self):
        derived = tuple(DerivedCV(f"rate_{cv.id}", "Rate", "unit/min", cv.id, 0.5, 0, 0)
                        for cv in self.definition.primary_cvs)
        definition = replace(self.definition, derived_cvs=derived)
        validate_simulation(self.raw, definition)
        measurements = Measurements(definition, {"inventory_a": 0, "inventory_b": 10})
        measurements.update({"inventory_a": 0, "inventory_b": 10})
        state, _ = measurements.update({"inventory_a": 2, "inventory_b": 6})
        self.assertEqual(state, (1, 8, 12, -24))

    def test_model_check_uses_named_second_destination(self):
        self.definition.raw["model_checks"] = [dict(source="drive_b", destination="inventory_b",
                                                    step=2, expected_rate_per_minute=-0.2, tolerance=1e-9)]
        verify_model_checks(self.definition)
        self.definition.raw["model_checks"][0]["expected_rate_per_minute"] = 0.2
        with self.assertRaises(ConfigurationError):
            verify_model_checks(self.definition)

    def test_solver_failure_is_reported_and_mv_held(self):
        from optam_mpc.controller import NMPCOptimizationError
        from run_simulation import simulate

        class FailingController:
            def calculate_move(self, *args):
                raise NMPCOptimizationError("Deliberate test failure")

        def factory(definition):
            return FailingController(), (), (0, 0), ()

        records, report = simulate(self.definition, self.raw, factory)
        self.assertFalse(report["passed"])
        self.assertEqual(len(report["failures"]), 4)
        self.assertTrue(all(row["u1"] == 0 and row["u2"] == 0 for row in records))

    def test_programming_errors_are_not_treated_as_solver_failures(self):
        from run_simulation import simulate

        class BrokenController:
            def calculate_move(self, *args):
                raise TypeError("Deliberate bug")

        with self.assertRaises(TypeError):
            simulate(self.definition, self.raw, lambda _: (BrokenController(), (), (0, 0), ()))

    def test_real_solver_in_isolated_runtime_without_examples(self):
        runtime = self.folder / "generic runtime"
        runtime.mkdir()
        for name in ("optam_mpc", "optam_mpc_linear"):
            shutil.copytree(ROOT / name, runtime / name, ignore=shutil.ignore_patterns("__pycache__"))
        for name in ("run_simulation.py", "validate_and_report.py"):
            shutil.copy2(ROOT / name, runtime / name)
        self.assertFalse((runtime / "examples").exists())
        env = dict(os.environ, OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1")
        for name in ("validate_and_report.py", "run_simulation.py"):
            result = subprocess.run([sys.executable, str(runtime / name), str(self.app), str(self.sim)],
                                    cwd=self.folder, env=env, capture_output=True, text=True, timeout=60)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertTrue((self.folder / "results" / "results.csv").exists())
        self.assertTrue((self.folder / "results" / "report.json").exists())
        self.assertTrue((self.folder / "results" / "results.png").exists())

    def test_failed_acceptance_exit_code(self):
        text = self.sim.read_text().replace("limit = -100.0", "limit = 100.0")
        self.sim.write_text(text)
        result = subprocess.run([sys.executable, str(ROOT / "run_simulation.py"),
                                 str(self.app), str(self.sim)], cwd=self.folder,
                                capture_output=True, text=True, timeout=60)
        self.assertEqual(result.returncode, 2, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main(verbosity=2)
