"""Generic, configuration-only linear plant and scenario support.

This module has no controller/solver dependency.
"""
from __future__ import annotations

import math
from pathlib import Path
import tomllib

from .builder import DerivedRateCalculator
from .definition import ConfigurationError, ID_PATTERN, parse_path
from .dynamics import PathBank


def load_simulation(path, definition):
    with Path(path).open("rb") as stream:
        raw = tomllib.load(stream)
    validate_simulation(raw, definition)
    return raw


def validate_simulation(raw, definition):
    errors = []

    def check(condition, message):
        if not condition:
            errors.append(message)

    def number(item, key, label, minimum=None):
        value = item.get(key)
        valid = (isinstance(value, (int, float)) and not isinstance(value, bool)
                 and math.isfinite(value))
        check(valid, f"{label}.{key}: finite number required.")
        if valid and minimum is not None:
            check(value >= minimum, f"{label}.{key}: must be >= {minimum}.")
        return value if valid else 0.0

    def tables(key):
        value = raw.get(key, [])
        if not isinstance(value, list) or not all(isinstance(x, dict) for x in value):
            errors.append(f"Use repeated [[{key}]] tables.")
            return []
        return value

    allowed = {"simulation", "plant_outputs", "plant_inputs", "plant_paths", "plant_model",
               "events", "target_events", "checks", "plots"}
    check(not set(raw) - allowed, f"Unknown simulation sections: {set(raw) - allowed}")
    settings = raw.get("simulation", {})
    if not isinstance(settings, dict):
        raise ConfigurationError(["Use [simulation] table."])
    samples = settings.get("samples")
    check(type(samples) is int and samples > 0, "[simulation].samples: positive integer required.")
    max_failures = settings.get("maximum_solver_failures", 0)
    check(type(max_failures) is int and max_failures >= 0,
          "[simulation].maximum_solver_failures: non-negative integer required.")
    outputs, inputs, paths = tables("plant_outputs"), tables("plant_inputs"), tables("plant_paths")
    check(bool(outputs) and bool(inputs) and (bool(paths) or "plant_model" in raw),
          "Plant requires outputs, inputs and paths or plant_model.")
    cv_ids = {x.id for x in definition.primary_cvs}
    mv_ids = {x.id for x in definition.sources if x.kind == "mv"}
    dv_ids = {x.id for x in definition.sources if x.kind == "dv"}
    output_ids, input_ids, signals, mapped_cvs, mapped_mvs, mapped_dvs = set(), set(), set(), [], [], []
    for group, items, ids in (("plant_outputs", outputs, output_ids), ("plant_inputs", inputs, input_ids)):
        for item in items:
            ident = item.get("id")
            check(isinstance(ident, str) and bool(ID_PATTERN.fullmatch(ident)),
                  f"{group}: valid lower-case id required.")
            if not isinstance(ident, str):
                continue
            check(ident not in signals, f"Duplicate plant signal: {ident}.")
            ids.add(ident)
            signals.add(ident)
            number(item, "initial", ident)
            check(isinstance(item.get("unit"), str), f"{ident}: unit required.")
            if group == "plant_outputs":
                cv = item.get("cv")
                check(cv in cv_ids, f"{ident}: cv must identify a primary CV.")
                mapped_cvs.append(cv)
                number(item, "nominal_rate_per_minute", ident)
            else:
                role = item.get("role")
                check(role in ("mv", "measured_dv", "hidden"), f"{ident}: invalid input role.")
                if role == "hidden":
                    check("controller_id" not in item, f"{ident}: hidden inputs cannot have controller_id.")
                else:
                    cid = item.get("controller_id")
                    valid_ids = mv_ids if role == "mv" else dv_ids
                    check(cid in valid_ids, f"{ident}: unknown controller_id for role {role}.")
                    (mapped_mvs if role == "mv" else mapped_dvs).append(cid)
                    match = next((s for s in definition.sources if s.id == cid), None)
                    if match:
                        check(item.get("initial") == match.initial,
                              f"{ident}: initial must equal controller source initial.")
                        check(item.get("unit") == match.unit, f"{ident}: unit must match controller source.")
            if group == "plant_outputs":
                match = next((s for s in definition.primary_cvs if s.id == item.get("cv")), None)
                if match:
                    check(item.get("unit") == match.unit, f"{ident}: unit must match mapped CV.")
    for mapped, expected, label in ((mapped_cvs, cv_ids, "CV"), (mapped_mvs, mv_ids, "MV"),
                                    (mapped_dvs, dv_ids, "measured DV")):
        check(set(mapped) == expected and len(mapped) == len(expected),
              f"Each controller {label} must have exactly one plant mapping.")
    for p in paths:
        check(p.get("source") in input_ids, "Plant path: unknown input source.")
        check(p.get("destination") in output_ids, "Plant path: unknown output destination.")
        parse_path(p, definition.sample_time_seconds, "Plant path", errors)
    event_keys = set()
    external_ids = {x.get("id") for x in inputs if x.get("role") in ("hidden", "measured_dv")}
    for e in tables("events"):
        sample = e.get("sample")
        check(type(sample) is int and sample >= 0 and type(samples) is int and sample < samples,
              "Event sample must be an integer in 0..samples-1.")
        check(e.get("input") in external_ids, "Events may set hidden or measured-DV inputs only.")
        number(e, "value", "event")
        key = (str(sample), str(e.get("input")))
        check(key not in event_keys, "Duplicate event for same input and sample.")
        event_keys.add(key)
    target_ids = {x.id for x in definition.primary_cvs if x.target is not None}
    for e in tables("target_events"):
        sample = e.get("sample")
        check(type(sample) is int and 0 <= sample < samples, "Target event sample outside run.")
        check(e.get("cv") in target_ids, "Target event must identify a target-based primary CV.")
        number(e, "value", "target event")
        key = (str(sample), str(e.get("cv")))
        check(key not in event_keys, "Duplicate target event.")
        event_keys.add(key)
    record_ids = signals | {f"cv.{x.id}" for x in definition.primary_cvs} | {
        f"derived.{x.id}" for x in definition.derived_cvs}
    record_ids |= {f"target.{x}" for x in target_ids}
    checks = tables("checks")
    check(bool(checks), "At least one [[checks]] acceptance check is required.")
    for c in checks:
        check(c.get("signal") in record_ids, "Check references unknown signal.")
        metric = c.get("metric")
        check(metric in ("minimum", "maximum", "max_abs_move", "final_error", "max_abs_error"),
              "Check metric must be minimum, maximum, max_abs_move, final_error or max_abs_error.")
        number(c, "limit", "check", 0 if metric in ("max_abs_move", "final_error", "max_abs_error") else None)
        if metric in ("final_error", "max_abs_error"):
            number(c, "target", "check")
        for key in ("start_sample", "end_sample"):
            if key in c:
                check(type(c[key]) is int and 0 <= c[key] <= samples, f"Check {key} outside run.")
        check(c.get("start_sample", 0) <= c.get("end_sample", samples), "Check window reversed.")
    plots = tables("plots")
    check(bool(plots), "At least one [[plots]] panel is required.")
    for p in plots:
        selected = p.get("signals")
        check(isinstance(selected, list) and bool(selected)
              and all(isinstance(s, str) and s in record_ids for s in selected),
              "Plot signals must be a non-empty list of known signal IDs.")
        check(isinstance(p.get("ylabel"), str), "Plot ylabel required (include units).")
    # One filter per source prevents ambiguous primary feedback filtering.
    derived_sources = [x.source_cv for x in definition.derived_cvs]
    check(len(set(derived_sources)) == len(derived_sources),
          "Simulation supports one derived-rate CV per primary CV.")
    check(all(x.initial == 0 for x in definition.derived_cvs),
          "Derived rates must initialise at zero (no pre-start measurement history).")
    if errors:
        raise ConfigurationError(errors)


class LinearPlant:
    """Independent deviation-form integrators, with explicit nominal drift."""
    def __init__(self, raw, sample_time_seconds):
        self.raw = raw
        self.dt_minutes = sample_time_seconds / 60.0
        self.outputs = {x["id"]: x["initial"] for x in raw["plant_outputs"]}
        self.inputs = {x["id"]: x["initial"] for x in raw["plant_inputs"]}
        self.nominal_inputs = dict(self.inputs)
        errors = []
        paths = [parse_path(p, sample_time_seconds, "Plant path", errors) for p in raw["plant_paths"]]
        if errors:
            raise ConfigurationError(errors)
        self.bank = PathBank(paths, sample_time_seconds, self.nominal_inputs, self.outputs)
        self.internal = self.bank.initial

    def events(self, sample):
        for event in self.raw.get("events", []):
            if event["sample"] == sample:
                self.inputs[event["input"]] = event["value"]

    def set_mvs(self, values):
        for item in self.raw["plant_inputs"]:
            if item["role"] == "mv":
                self.inputs[item["id"]] = values[item["controller_id"]]

    def measured_dvs(self):
        return {x["controller_id"]: self.inputs[x["id"]]
                for x in self.raw["plant_inputs"] if x["role"] == "measured_dv"}

    def measurements(self):
        return {x["cv"]: self.outputs[x["id"]] for x in self.raw["plant_outputs"]}

    def step(self):
        rates = {x["id"]: x["nominal_rate_per_minute"] for x in self.raw["plant_outputs"]}
        self.internal, increments = self.bank.advance(self.internal, self.inputs)
        self.outputs = {k: v + increments[k] + self.dt_minutes * rates[k] for k, v in self.outputs.items()}


def target_values(definition):
    return {x.id: x.target for x in definition.primary_cvs if x.target is not None}


def targets_requested(settings):
    return bool(settings.get("target_events")) or any(
        s.startswith("target.") for p in settings.get("plots",[]) for s in p["signals"]
    ) or any(c["signal"].startswith("target.") for c in settings.get("checks",[]))


def apply_target_events(settings, sample, targets):
    for e in settings.get("target_events", []):
        if e["sample"] == sample:
            targets[e["cv"]] = e["value"]


def update_targets(controller, definition, previous, targets):
    changes = {i: targets[x.id] for i, x in enumerate(definition.primary_cvs)
               if x.id in targets and previous.get(x.id) != targets[x.id]}
    if changes:
        controller.update_tuning(cv_targets=changes)


class Measurements:
    def __init__(self, definition, initial):
        self.definition = definition
        self.filters = {x.id: DerivedRateCalculator(x.filter_alpha,
                        definition.sample_time_seconds, initial[x.source_cv])
                        for x in definition.derived_cvs}

    def update(self, measured):
        primary, derived = dict(measured), {}
        for cv in self.definition.derived_cvs:
            primary[cv.source_cv], derived[cv.id] = self.filters[cv.id].update(measured[cv.source_cv])
        state = tuple(primary[x.id] for x in self.definition.primary_cvs) + tuple(
            derived[x.id] for x in self.definition.derived_cvs)
        signals = {f"cv.{k}": v for k, v in primary.items()}
        signals.update({f"derived.{k}": v for k, v in derived.items()})
        return state, signals


def evaluate_checks(records, checks):
    results = []
    for c in checks:
        values = [row[c["signal"]] for i, row in enumerate(records)
                  if c.get("start_sample", 0) <= i <= c.get("end_sample", len(records)-1)]
        if not values:
            results.append(dict(signal=c["signal"], metric=c["metric"], actual=None,
                                limit=c["limit"], target=c.get("target"), passed=False))
            continue
        metric = c["metric"]
        if metric == "minimum":
            actual = min(values)
            passed = actual >= c["limit"]
        elif metric == "maximum":
            actual = max(values)
            passed = actual <= c["limit"]
        elif metric == "max_abs_move":
            actual = max((abs(b-a) for a, b in zip(values, values[1:])), default=0)
            passed = actual <= c["limit"]
        elif metric == "max_abs_error":
            actual = max(abs(v-c["target"]) for v in values)
            passed = actual <= c["limit"]
        else:
            actual = abs(values[-1] - c["target"])
            passed = actual <= c["limit"]
        passed = passed and all(math.isfinite(x) for x in values)
        results.append(dict(signal=c["signal"], metric=metric, actual=actual,
                            limit=c["limit"], target=c.get("target"), passed=passed,
                            start_sample=c.get("start_sample",0),
                            end_sample=c.get("end_sample",len(records)-1)))
    return results
