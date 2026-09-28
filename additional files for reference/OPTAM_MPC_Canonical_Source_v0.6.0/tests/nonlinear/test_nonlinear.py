import copy
import math
from pathlib import Path
import shutil
import tempfile
import unittest

from nonlinear_runtime import load_definition, load_simulation, build_controller, ConfigurationError
from optam_mpc_linear.opc import fingerprint, validate_sample, validate_command, PROTOCOL, load_opc
from optam_mpc_linear.simulation import evaluate_checks
from optam_mpc.prediction import nonlinear_velocity_step_values

ROOT = Path(__file__).resolve().parents[1]
EXAMPLE = ROOT / "examples" / "01_heat_exchanger"


class NonlinearTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.folder = Path(self.tmp.name)
        for name in ("application.toml", "model.py", "simulation.toml", "opc.toml"):
            shutil.copy2(EXAMPLE/name, self.folder/name)
        self.app = self.folder/"application.toml"

    def edit(self, before, after, file=None):
        file = file or self.app
        text = file.read_text()
        self.assertIn(before,text)
        file.write_text(text.replace(before,after))

    def test_reference_loads_and_checks(self):
        d = load_definition(self.app)
        self.assertEqual(d.model.state_dimension, 2)
        self.assertGreater(d.model.probe_count, 60)

    def test_model_nominal_is_stationary(self):
        m = load_definition(self.app).model
        self.assertEqual(tuple(float(v) for v in m.transition_values((60,64),(50,50),(40,5))), (60,64))

    def test_nonlinear_response_is_not_additive(self):
        m = load_definition(self.app).model
        base = float(m.transition_values((60,64),(50,50),(40,5))[0])
        one = float(m.transition_values((60,64),(55,50),(40,5))[0])-base
        two = float(m.transition_values((60,64),(60,50),(40,5))[0])-base
        self.assertGreater(abs(two-2*one), .01)

    def test_pressure_gain_changes_with_pressure(self):
        m = load_definition(self.app).model
        def slope(p):
            return (float(m.transition_values((60,64),(50,50),(40,p+.0001))[0])-
                    float(m.transition_values((60,64),(50,50),(40,p-.0001))[0]))/.0002
        self.assertGreater(slope(4.7), slope(5.3))

    def test_independent_cold_inlet_calculation(self):
        m = load_definition(self.app).model
        result = m.transition_values((60,64),(50,50),(37,5))
        self.assertAlmostEqual(float(result[1]), 64+(1-math.exp(-5/40))*(-1.8),12)

    def test_nominal_move(self):
        c,x,u,d = build_controller(load_definition(self.app))
        r = c.calculate_move(x,d,u)
        self.assertTrue(all(abs(v-50)<1e-5 for v in r.first_input))

    def test_velocity_stationary_anchor_with_model_bias(self):
        m = load_definition(self.app).model
        x,u,d = (60,65),(50,50),(40,5)
        f = m.transition_values(x,u,d)
        nxt,_ = nonlinear_velocity_step_values(x,u,d,f,m.transition_values)
        self.assertTrue(all(abs(float(a)-b)<1e-12 for a,b in zip(nxt,x)))

    def test_changed_python_changes_fingerprint(self):
        first = fingerprint(load_definition(self.app))
        file = self.folder/"model.py"
        file.write_text(file.read_text()+"\n# revision\n")
        self.assertNotEqual(first, fingerprint(load_definition(self.app)))

    def test_fingerprint_is_location_independent(self):
        self.assertEqual(fingerprint(load_definition(self.app)), fingerprint(load_definition(EXAMPLE/"application.toml")))

    def test_mismatch_is_plant_only(self):
        d = load_definition(self.app)
        s = load_simulation(EXAMPLE/"mismatch.toml",d)
        self.assertEqual(d.model.parameters["flow_tau_seconds"],10)
        self.assertEqual(s["_nonlinear_model"].parameters["flow_tau_seconds"],12)

    def test_invalid_tau_rejected(self):
        self.edit("flow_tau_seconds = 10", "flow_tau_seconds = 0")
        with self.assertRaises(ValueError): load_definition(self.app)

    def test_unknown_parameter_rejected(self):
        self.edit("flow_tau_seconds = 10", "flow_tau_seconds = 10\nmisspelled = 1")
        with self.assertRaises(ValueError): load_definition(self.app)

    def test_wrong_state_ids_rejected(self):
        self.edit('STATE_IDS = ("flow", "temperature")','STATE_IDS = ("temperature", "flow")',self.folder/"model.py")
        with self.assertRaises(ConfigurationError): load_definition(self.app)

    def test_bad_expected_result_rejected(self):
        self.edit("expected_next = [60,64]", "expected_next = [61,64]")
        with self.assertRaises(ConfigurationError): load_definition(self.app)

    def test_no_independent_checks_rejected(self):
        self.app.write_text(self.app.read_text().split("[[nonlinear_checks]]")[0])
        with self.assertRaises(ConfigurationError): load_definition(self.app)

    def test_wrong_output_keys_rejected(self):
        self.edit('return {"flow":','return {"wrong":',self.folder/"model.py")
        with self.assertRaises(ConfigurationError): load_definition(self.app)

    def test_unsafe_pressure_domain_rejected(self):
        self.edit("upstream_pressure = [4.6, 5.4]", "upstream_pressure = [2, 5.4]")
        with self.assertRaises(ConfigurationError): load_definition(self.app)

    def test_mv_bounds_outside_domain_rejected(self):
        self.edit("lower = 35", "lower = 25")
        with self.assertRaises(ConfigurationError): load_definition(self.app)

    def test_missing_domain_rejected(self):
        self.edit("flow = [30, 90]", "")
        with self.assertRaises(ConfigurationError): load_definition(self.app)

    def test_invalid_runtime_dv_rejected(self):
        c,x,u,d = build_controller(load_definition(self.app))
        with self.assertRaises(ConfigurationError): c.calculate_move(x,(40,3),u)

    def test_feedforward_off_still_rejects_invalid_dv(self):
        self.edit("use_measured_disturbances = true", "use_measured_disturbances = false")
        c,x,u,d = build_controller(load_definition(self.app))
        with self.assertRaises(ConfigurationError): c.calculate_move(x,(40,math.nan),u)

    def test_python_parent_path_rejected(self):
        self.edit('file = "model.py"','file = "../model.py"')
        with self.assertRaises(ConfigurationError): load_definition(self.app)

    def test_nonstationary_start_rejected(self):
        self.edit("initial = 64", "initial = 65")
        with self.assertRaises(ConfigurationError): load_definition(self.app)

    def test_scenario_dv_outside_domain_rejected(self):
        file = self.folder/"simulation.toml"
        self.edit("value = 37", "value = 30", file)
        with self.assertRaises(ConfigurationError): load_simulation(file,load_definition(self.app))

    def test_template_is_intentionally_incomplete(self):
        with self.assertRaises(ConfigurationError): load_definition(ROOT/"application_template.toml")

    def test_window_detects_oscillation(self):
        c = dict(signal="flow",metric="max_abs_error",target=60,limit=.1,start_sample=1)
        self.assertFalse(evaluate_checks([{"flow":60},{"flow":61},{"flow":60}],[c])[0]["passed"])

    def test_non_loopback_endpoint_rejected(self):
        file=self.folder/"opc.toml"
        self.edit("127.0.0.1", "0.0.0.0", file)
        with self.assertRaises(ValueError): load_opc(file)

    def test_protocol_checks(self):
        d = load_definition(self.app)
        frame=dict(protocol=PROTOCOL,application_hash=fingerprint(d),run_id="run",sample=0,
                   sample_time_seconds=5,cvs={"flow":60,"temperature":64},
                   dvs={"inlet_temperature":40,"upstream_pressure":5},mvs={"hx_valve":50,"bypass_valve":50})
        validate_sample(frame,d,0,"run")
        for key,value in (("application_hash","wrong"),("sample",2),("run_id","other")):
            bad=copy.deepcopy(frame);bad[key]=value
            with self.assertRaises(ValueError): validate_sample(bad,d,0,"run")
        cmd=dict(application_hash=fingerprint(d),run_id="run",sample=0,controller_id="one",
                 status="ok",solver_seconds=.1,mvs={"hx_valve":50,"bypass_valve":50})
        validate_command(cmd,frame,d)
        cmd["mvs"]["hx_valve"]=55
        with self.assertRaises(ValueError): validate_command(cmd,frame,d)


if __name__ == "__main__": unittest.main()
