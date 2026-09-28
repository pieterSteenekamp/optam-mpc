"""Engineer a different synthetic 2x2 from templates only, with no example files."""
from pathlib import Path
import shutil
import tempfile
import unittest

from nonlinear_runtime import load_definition, load_simulation
from run_simulation import simulate

ROOT = Path(__file__).resolve().parents[1]


class GenericEngineeringTests(unittest.TestCase):
    def test_filled_generic_templates_without_example_model(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp)
            app=(ROOT/"application_template.toml").read_text()
            replacements={
                'name = "REPLACE WITH APPLICATION NAME"':'name = "Independent synthetic application"',
                'sample_time_seconds = -1.0':'sample_time_seconds = 5.0',
                'prediction_horizon_samples = 0':'prediction_horizon_samples = 10',
                'control_horizon_samples = 0':'control_horizon_samples = 3',
                'protected_low = 0.0':'protected_low = -5.0',
                'protected_high = 0.0':'protected_high = 5.0',
                'lower = 0.0':'lower = -1.0',
                'upper = 0.0':'upper = 1.0',
                'maximum_increase_per_sample = -1.0':'maximum_increase_per_sample = 0.2',
                'maximum_decrease_per_sample = -1.0':'maximum_decrease_per_sample = 0.2',
                'tau_a_seconds = -1.0':'tau_a_seconds = 10.0',
                'tau_b_seconds = -1.0':'tau_b_seconds = 20.0',
                'weight = -1.0':'weight = 1.0',
                'normalization = -1.0':'normalization = 1.0',
            }
            for before,after in replacements.items(): app=app.replace(before,after)
            # Independent calculation: dt/tau=0.5, first equilibrium=0.1.
            app+='\n[[nonlinear_checks]]\nname="DV unit check"\nstate=[0,0]\nmvs=[0,0]\ndvs=[0.1,0]\nexpected_next=[0.03934693402873666,0]\ntolerance=1e-10\n'
            sim=(ROOT/"simulation_template.toml").read_text()
            sim=sim.replace('samples = 0','samples = 60').replace('sample = 0','sample = 5')
            sim=sim.replace('value = 0.0 # REPLACE: absolute','value = 0.1 # REPLACE: absolute')
            sim=sim.replace('metric = "minimum"\nlimit = 0.0','metric = "minimum"\nlimit = -1.0')
            sim=sim.replace('metric = "maximum"\nlimit = 0.0','metric = "maximum"\nlimit = 1.0')
            sim=sim.replace('limit = 0.0 # REPLACE: maximum absolute final error','limit = 0.01 # REPLACE: maximum absolute final error')
            sim=sim.replace('limit = 0.0 # REPLACE: absolute move per sample','limit = 0.2 # REPLACE: absolute move per sample')
            sim=sim.replace('tau_a_seconds = -1.0','tau_a_seconds = 10.0').replace('tau_b_seconds = -1.0','tau_b_seconds = 20.0')
            (folder/"application.toml").write_text(app)
            (folder/"simulation.toml").write_text(sim)
            shutil.copy2(ROOT/"model_template.py",folder/"model.py")
            definition=load_definition(folder/"application.toml")
            settings=load_simulation(folder/"simulation.toml",definition)
            records,report=simulate(definition,settings)
            self.assertTrue(report["passed"],report)
            self.assertFalse(report["failures"])
            self.assertEqual(len(records),61)
            self.assertNotIn("flow",definition.model.cv_ids)


if __name__=="__main__": unittest.main()
