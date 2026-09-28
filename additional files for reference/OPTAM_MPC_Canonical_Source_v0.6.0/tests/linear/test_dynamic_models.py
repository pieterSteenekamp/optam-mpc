"""Dynamic path and HX engineering checks independent of closed-loop success."""
from copy import deepcopy
from dataclasses import replace
import math
from pathlib import Path
import tempfile
import unittest

from optam_mpc_linear import load_definition, ConfigurationError
from optam_mpc_linear.definition import IntegratorPath, parse_path
from optam_mpc_linear.dynamics import PathBank, model_check_value
from optam_mpc_linear.controller_adapter import LinearVelocityStateModel, build_controller
from optam_mpc_linear.simulation import load_simulation, LinearPlant, evaluate_checks
from optam_mpc.prediction import nonlinear_velocity_step_values

ROOT = Path(__file__).resolve().parents[1]
HX = ROOT / "examples" / "02_heat_exchanger"


def path(kind="first_order", gain=2, tau=10, delay=0):
    return IntegratorPath("u", "y", gain if kind=="integrator" else 0, delay, "test",
                          kind, gain if kind!="integrator" else 0, tau)


class DynamicPathTests(unittest.TestCase):
    def response(self, p, count=10):
        bank = PathBank([p], 5, {"u": 0}, ["y"])
        state, y, values = bank.initial, 0, []
        for _ in range(count):
            state, change = bank.advance(state, {"u": 1})
            y += change["y"]
            values.append(y)
        return values

    def test_exact_first_order_response(self):
        values = self.response(path())
        for k,y in enumerate(values,1):
            self.assertAlmostEqual(y, 2*(1-math.exp(-5*k/10)), places=13)

    def test_two_sample_delay(self):
        values = self.response(path(delay=10))
        self.assertEqual(values[:2], [0,0])
        self.assertAlmostEqual(values[2], 2*(1-math.exp(-.5)))

    def test_negative_gain(self):
        self.assertLess(self.response(path(gain=-2))[0], 0)

    def test_delayed_integrator(self):
        values = self.response(path(kind="integrator", gain=3, delay=10))
        self.assertEqual(values[:2], [0,0])
        self.assertAlmostEqual(values[-1], 3*8*5/60)

    def test_static_gain_sample_convention(self):
        self.assertEqual(self.response(path(kind="gain", gain=4, delay=5), 4), [0,4,4,4])

    def test_first_order_settles_without_drift(self):
        values = self.response(path(), 300)
        self.assertAlmostEqual(values[-1], 2)
        self.assertAlmostEqual(values[-1], values[-2])

    def test_nonzero_nominal_input_is_subtracted(self):
        bank = PathBank([path()], 5, {"u": 50}, ["y"])
        _, change = bank.advance(bank.initial, {"u": 50})
        self.assertEqual(change["y"], 0)

    def test_parallel_paths_add(self):
        bank = PathBank([path(), path(gain=3)], 5, {"u":0}, ["y"])
        _, change = bank.advance(bank.initial, {"u":1})
        self.assertAlmostEqual(change["y"], 5*(1-math.exp(-.5)))

    def test_mixed_integrator_and_lag(self):
        bank = PathBank([path(), path(kind="integrator", gain=3)], 5, {"u":0}, ["y"])
        _, change = bank.advance(bank.initial, {"u":1})
        self.assertAlmostEqual(change["y"], 2*(1-math.exp(-.5))+.25)

    def test_fractional_delay_rejected_not_rounded(self):
        errors = []
        parse_path(dict(source="u",destination="y",type="first_order",gain=1,
                        time_constant_seconds=10,dead_time_seconds=7), 5, "test", errors)
        self.assertTrue(errors)

    def test_zero_lag_rejected(self):
        errors = []
        parse_path(dict(source="u",destination="y",type="first_order",gain=1,
                        time_constant_seconds=0,dead_time_seconds=0), 5, "test", errors)
        self.assertTrue(errors)


class HeatExchangerTests(unittest.TestCase):
    def setUp(self):
        self.definition = load_definition(HX/"application.toml")

    def test_two_by_two_with_two_measured_disturbances(self):
        self.assertEqual(len(self.definition.primary_cvs), 2)
        self.assertEqual(len(self.definition.sources), 4)
        self.assertEqual(len(self.definition.derived_cvs), 0)

    def test_no_integrators(self):
        self.assertTrue(all(p.type=="first_order" for p in self.definition.paths))

    @staticmethod
    def physical(h=50, b=50, tin=40, p=5):
        qh = 30*h/50*math.sqrt((p-3)/2)
        qb = 30*b/50*math.sqrt((p-3)/2)
        th = 100-(100-tin)*math.exp(-30*math.log(5)/qh)
        return qh+qb, (qh*th+qb*tin)/(qh+qb)

    def test_nominal_heat_and_mass_balance(self):
        self.assertEqual(self.physical(), (60,64))

    def test_all_gains_match_independent_physical_derivatives(self):
        mapping = {"hx_valve":"h","bypass_valve":"b","inlet_temperature":"tin","upstream_pressure":"p"}
        nominal = dict(h=50,b=50,tin=40,p=5)
        for p in self.definition.paths:
            plus, minus = dict(nominal), dict(nominal)
            plus[mapping[p.source]] += 1e-4
            minus[mapping[p.source]] -= 1e-4
            index = 0 if p.destination=="flow" else 1
            actual = (self.physical(**plus)[index]-self.physical(**minus)[index])/2e-4
            self.assertAlmostEqual(actual, p.gain, places=6)

    def test_temperature_to_flow_path_is_physically_zero(self):
        self.assertEqual(self.physical(tin=43)[0], 60)
        self.assertEqual(model_check_value(self.definition, dict(source="inlet_temperature",
                         destination="flow", metric="steady_change",step=1)), 0)

    def test_augmented_velocity_step_matches_absolute_model_on_nominal_history(self):
        model = LinearVelocityStateModel(self.definition)
        old = (60,64,*model.bank.initial)
        u0, d0 = (50,50), (40,5)
        u1, d1 = (51,49), (39,5.2)
        current = model.transition_values(old, u0, d0)
        previous_f = model.transition_values(old, u0, d0)
        next_v, _ = nonlinear_velocity_step_values(current,u1,d1,previous_f,model.transition_values)
        direct = model.transition_values(current,u1,d1)
        for a,b in zip(next_v,direct):
            self.assertAlmostEqual(a,b,places=12)

    def test_plant_step_gain_and_steady_state(self):
        settings = load_simulation(HX/"simulation.toml", self.definition)
        plant = LinearPlant(settings,5)
        plant.set_mvs({"hx_valve":51,"bypass_valve":50})
        for _ in range(300):
            plant.step()
        self.assertAlmostEqual(plant.outputs["outlet_flow"],60.6,places=10)
        self.assertAlmostEqual(plant.outputs["outlet_temperature"],64+self.definition.paths[2].gain,places=10)

    def test_target_update_preserves_controller_history(self):
        controller, state, mvs, dvs = build_controller(self.definition)
        controller.calculate_move(state,dvs,mvs)
        previous = controller.core._previous_state
        controller.update_tuning(cv_targets={0:64})
        self.assertIs(controller.core._previous_state, previous)
        self.assertEqual(controller.configuration.controlled_variables[0].target,64)

    def test_windowed_acceptance_uses_requested_endpoint(self):
        values = [{"x":x} for x in (0,1,2,3,4)]
        result = evaluate_checks(values,[dict(signal="x",metric="final_error",
                                  target=2,limit=0,end_sample=2)])
        self.assertTrue(result[0]["passed"])

    def test_settling_window_catches_oscillation_missed_by_final_value(self):
        values = [{"x":x} for x in (0,1,-1,1,0)]
        checks = [dict(signal="x",metric="final_error",target=0,limit=.1),
                  dict(signal="x",metric="max_abs_error",target=0,limit=.1,start_sample=1)]
        results = evaluate_checks(values,checks)
        self.assertTrue(results[0]["passed"])
        self.assertFalse(results[1]["passed"])

    def test_independent_model_mismatch_is_declared(self):
        raw = load_simulation(HX/"mismatch.toml", self.definition)
        for original, changed in zip(self.definition.paths,raw["plant_paths"]):
            self.assertAlmostEqual(changed["gain"], original.gain*1.15)
            self.assertAlmostEqual(changed["time_constant_seconds"], original.time_constant_seconds*1.2)
            self.assertEqual(changed["dead_time_seconds"], original.dead_time_seconds+5)


if __name__ == "__main__":
    unittest.main(verbosity=2)
