"""Generic measured-state nonlinear model contract; no exchanger-specific code."""
from __future__ import annotations

from dataclasses import replace
import hashlib
import itertools
import math
from pathlib import Path
import types
import tomllib

import casadi as ca

from optam_mpc_linear.definition import ConfigurationError, validate_definition
from optam_mpc_linear.controller_adapter import build_controller as core_factory
from optam_mpc_linear.simulation import LinearPlant, validate_simulation


def require(condition, message):
    if not condition:
        raise ConfigurationError([message])


def numbers(values, count, label):
    require(isinstance(values, (list, tuple)) and len(values) == count,
            f"{label}: exactly {count} numbers required.")
    require(all(type(v) in (int, float) and math.isfinite(v) for v in values),
            f"{label}: finite numeric values required.")
    return tuple(float(v) for v in values)


class Model:
    def __init__(self, cfg, parent, definition):
        require(isinstance(cfg, dict), "Use a [model] or [plant_model] table.")
        require(set(cfg) <= {"file", "parameters", "domain"}, "Unknown nonlinear model setting.")
        filename = cfg.get("file")
        require(isinstance(filename, str), "Model file is required.")
        file = Path(filename)
        require(not file.is_absolute() and file.suffix == ".py" and ".." not in file.parts,
                "Model file must be a relative .py path below the configuration folder.")
        file = (parent / file).resolve()
        require(file.is_relative_to(parent.resolve()), "Model file must stay inside configuration folder.")
        content = file.read_bytes()
        self.sha256 = hashlib.sha256(content).hexdigest()
        # This executes trusted engineer-authored Python; it is NOT a sandbox.
        module = types.ModuleType("engineered_model_" + self.sha256[:12])
        module.__file__ = str(file)
        exec(compile(content, str(file), "exec"), module.__dict__)
        self.module, self.definition = module, definition
        self.cv_ids = tuple(x.id for x in definition.primary_cvs)
        self.mv_sources = tuple(x for x in definition.sources if x.kind == "mv")
        self.dv_sources = tuple(x for x in definition.sources if x.kind == "dv")
        self.mv_ids = tuple(x.id for x in self.mv_sources)
        self.dv_ids = tuple(x.id for x in self.dv_sources)
        for name, ids in (("STATE_IDS", self.cv_ids), ("MV_IDS", self.mv_ids), ("DV_IDS", self.dv_ids)):
            require(tuple(getattr(module, name, ())) == ids,
                    f"{name} must exactly match TOML IDs in their declared order: {ids}.")
        self.parameters = cfg.get("parameters", {})
        require(isinstance(self.parameters, dict), "Model parameters must be a table.")
        require(all(type(v) in (int, float) and math.isfinite(v) for v in self.parameters.values()),
                "Model parameters must be finite numbers.")
        require(callable(getattr(module, "validate_parameters", None)), "Define validate_parameters(p).")
        require(callable(getattr(module, "transition", None)), "Define transition(x, u, d, p, dt).")
        module.validate_parameters(self.parameters)
        self.domain = cfg.get("domain", {})
        self.ids = (*self.cv_ids, *self.mv_ids, *self.dv_ids)
        require(set(self.domain) == set(self.ids), "Model domain must cover every CV, MV and DV exactly.")
        for key in self.ids:
            low, high = numbers(self.domain[key], 2, f"Domain {key}")
            require(low < high, f"Domain {key}: lower must be less than upper.")
        for item in definition.raw["mvs"]:
            low, high = self.domain[item["id"]]
            require(low <= item["lower"] < item["upper"] <= high,
                    f"MV {item['id']}: controller bounds must lie inside model domain.")
        self.state_dimension = self.output_dimension = len(self.cv_ids)
        self.manipulated_input_dimension = len(self.mv_ids)
        self.disturbance_dimension = len(self.dv_ids)
        self.dt = definition.sample_time_seconds
        self.initial_state = tuple(x.initial for x in definition.primary_cvs)
        self.initial_mvs = tuple(x.initial for x in self.mv_sources)
        self.initial_dvs = tuple(x.initial for x in self.dv_sources)
        self.check_point(self.initial_state, self.initial_mvs, self.initial_dvs)
        self.preflight()

    def check_point(self, x, u, d):
        values = (*numbers(x, self.state_dimension, "States"),
                  *numbers(u, self.manipulated_input_dimension, "MVs"),
                  *numbers(d, self.disturbance_dimension, "DVs"))
        for key, value in zip(self.ids, values):
            low, high = self.domain[key]
            require(low - 1e-8 <= value <= high + 1e-8,
                    f"{key}={value:g} outside model domain [{low:g}, {high:g}].")

    def transition_values(self, state, inputs, disturbances):
        result = self.module.transition(dict(zip(self.cv_ids, state)),
                                        dict(zip(self.mv_ids, inputs)),
                                        dict(zip(self.dv_ids, disturbances)), self.parameters, self.dt)
        require(isinstance(result, dict) and set(result) == set(self.cv_ids),
                "transition must return a dictionary containing exactly STATE_IDS.")
        return tuple(result[k] for k in self.cv_ids)

    def output_values(self, state, inputs, disturbances):
        return tuple(state)

    def preflight(self):
        n, m, q = self.state_dimension, self.manipulated_input_dimension, self.disturbance_dimension
        z = ca.SX.sym("z", n + m + q)
        f = ca.vertcat(*self.transition_values(tuple(z[:n].elements()),
                                             tuple(z[n:n+m].elements()), tuple(z[n+m:].elements())))
        fun = ca.Function("model_check", [z], [f, ca.jacobian(f, z)])
        nominal = (*self.initial_state, *self.initial_mvs, *self.initial_dvs)
        # Axis end points for all dimensions; complete box corners for modest models.
        probes = [nominal]
        for i, key in enumerate(self.ids):
            for value in self.domain[key]:
                point = list(nominal)
                point[i] = value
                probes.append(point)
        if len(self.ids) <= 8:
            probes.extend(itertools.product(*(self.domain[k] for k in self.ids)))
        for point in probes:
            fx, jac = fun(point)
            require(all(math.isfinite(float(v)) for a in (fx, jac) for v in a.full().flat),
                    "Model output or derivative is non-finite in the declared domain.")
            direct = self.transition_values(point[:n], point[n:n+m], point[n+m:])
            require(all(abs(float(a)-float(b)) < 1e-8 for a,b in zip(direct, fx.full().flat)),
                    "Numeric and symbolic model evaluations disagree.")
        self.probe_count = len(probes)


def load_definition(path):
    path = Path(path).resolve()
    with path.open("rb") as stream:
        raw = tomllib.load(stream)
    require(raw.get("application", {}).get("model_type") == "nonlinear_python",
            'Use application.model_type = "nonlinear_python" in this package.')
    require(set(raw) <= {"application", "controller", "cvs", "mvs", "measured_disturbances",
                         "model", "nonlinear_checks"}, "Unknown application section; no linear paths or derived CVs here.")
    definition = validate_definition(raw)
    model = Model(raw.get("model"), path.parent, definition)
    raw["_model_sha256"] = model.sha256
    definition = replace(definition, model=model)
    nominal = model.transition_values(model.initial_state, model.initial_mvs, model.initial_dvs)
    require(all(abs(float(a)-b) <= 1e-8 for a,b in zip(nominal,model.initial_state)),
            "Initial CV/MV/DV values must be a steady state of the controller model.")
    verify_model_checks(definition)
    return definition


def verify_model_checks(definition):
    checks = definition.raw.get("nonlinear_checks", [])
    require(isinstance(checks, list) and len(checks) > 0, "At least one [[nonlinear_checks]] is required.")
    model = definition.model
    results = []
    for c in checks:
        require(isinstance(c, dict) and set(c) == {"name", "state", "mvs", "dvs", "expected_next", "tolerance"},
                "Each nonlinear check needs name, state, mvs, dvs, expected_next, tolerance.")
        model.check_point(c["state"], c["mvs"], c["dvs"])
        expected = numbers(c["expected_next"], model.state_dimension, "expected_next")
        tol = c["tolerance"]
        require(type(tol) in (int,float) and math.isfinite(tol) and tol > 0, "Check tolerance must be positive.")
        actual = tuple(float(v) for v in model.transition_values(c["state"],c["mvs"],c["dvs"]))
        require(all(math.isfinite(v) and abs(v-e) <= tol for v,e in zip(actual,expected)),
                f"Nonlinear check {c['name']!r} failed: actual={actual}, expected={expected}.")
        results.append((c["name"], actual))
    return results


class Controller:
    def __init__(self, core, model):
        self.core, self.model = core, model

    def __getattr__(self, name):
        return getattr(self.core, name)

    def calculate_move(self, state_values, disturbance_values, previous_input_values):
        # Check even disabled feedforward measurements: disabling it must not hide bad data.
        self.model.check_point(state_values, previous_input_values, disturbance_values)
        if not self.model.definition.raw["controller"].get("use_measured_disturbances", True):
            disturbance_values = self.model.initial_dvs
        return self.core.calculate_move(state_values, disturbance_values, previous_input_values)


def build_controller(definition):
    core, x, u, d = core_factory(definition, model=definition.model)
    return Controller(core, definition.model), x, u, d


def load_simulation(path, definition):
    path = Path(path).resolve()
    with path.open("rb") as stream:
        raw = tomllib.load(stream)
    require("plant_model" in raw and "plant_paths" not in raw, "Use [plant_model], not linear plant_paths.")
    validate_simulation(raw, definition)
    require(all(x["nominal_rate_per_minute"] == 0 for x in raw["plant_outputs"]),
            "Nonlinear plant: nominal_rate_per_minute must be zero; equations own all dynamics.")
    require(all(x["role"] != "hidden" for x in raw["plant_inputs"]),
            "This measured-state starter supports MVs and measured DVs only.")
    model = Model(raw["plant_model"], path.parent, definition)
    initial_x = {x["cv"]: x["initial"] for x in raw["plant_outputs"]}
    x = tuple(initial_x[k] for k in model.cv_ids)
    require(x == model.initial_state, "Plant initial outputs must equal the controller initial CVs.")
    nominal = model.transition_values(x, model.initial_mvs, model.initial_dvs)
    require(all(abs(float(a)-b) <= 1e-8 for a,b in zip(nominal,x)),
            "Plant initial point must also be stationary; adjust plant parameters or initial values.")
    input_map = {x["id"]: x["controller_id"] for x in raw["plant_inputs"]}
    for e in raw.get("events", []):
        key = input_map[e["input"]]
        for domain in (model.domain, definition.model.domain):
            require(domain[key][0] <= e["value"] <= domain[key][1], "DV event outside model domain.")
    raw["_nonlinear_model"] = model
    return raw


class NonlinearPlant(LinearPlant):
    # Reuse generic event/mapping methods; this class does NOT use PathBank.
    def __init__(self, raw, sample_time_seconds):
        self.raw, self.model = raw, raw["_nonlinear_model"]
        self.outputs = {x["id"]: x["initial"] for x in raw["plant_outputs"]}
        self.inputs = {x["id"]: x["initial"] for x in raw["plant_inputs"]}

    def step(self):
        state = self.measurements()
        mvs = {x["controller_id"]: self.inputs[x["id"]] for x in self.raw["plant_inputs"] if x["role"] == "mv"}
        dvs = self.measured_dvs()
        x, u, d = tuple(state[k] for k in self.model.cv_ids), tuple(mvs[k] for k in self.model.mv_ids), tuple(dvs[k] for k in self.model.dv_ids)
        self.model.check_point(x,u,d)
        nxt = tuple(float(v) for v in self.model.transition_values(x,u,d))
        self.model.check_point(nxt,u,d)
        values = dict(zip(self.model.cv_ids,nxt))
        self.outputs = {x["id"]: values[x["cv"]] for x in self.raw["plant_outputs"]}
