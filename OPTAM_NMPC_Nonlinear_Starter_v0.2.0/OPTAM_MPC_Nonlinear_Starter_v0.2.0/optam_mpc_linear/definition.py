"""Read and validate the configuration-only linear application definition."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import re
import math
import tomllib


ID_PATTERN = re.compile(r"^[a-z][a-z0-9_]*$")


class ConfigurationError(ValueError):
    """One or more engineering configuration items are invalid."""

    def __init__(self, errors):
        self.errors = tuple(errors)
        super().__init__("\n".join(self.errors))


@dataclass(frozen=True)
class PrimaryCV:
    id: str
    name: str
    unit: str
    initial: float
    target: float | None
    zone_low: float | None
    zone_high: float | None


@dataclass(frozen=True)
class DerivedCV:
    id: str
    name: str
    unit: str
    source_cv: str
    filter_alpha: float
    initial: float
    target: float


@dataclass(frozen=True)
class SourceVariable:
    id: str
    name: str
    unit: str
    kind: str
    initial: float


@dataclass(frozen=True)
class IntegratorPath:
    source: str
    destination: str
    integrating_gain: float
    dead_time_seconds: float
    engineering_basis: str
    type: str = "integrator"
    gain: float = 0.0
    time_constant_seconds: float = 0.0


def parse_path(item, sample_time, label, errors):
    kind = item.get("type")
    if kind not in ("integrator", "first_order", "gain"):
        errors.append(f"{label}: type must be integrator, first_order or gain.")
    gain = _number(item, "integrating_gain" if kind == "integrator" else "gain", label, errors)
    tau = _number(item, "time_constant_seconds", label, errors, positive=True) if kind == "first_order" else 0.0
    delay = _number(item, "dead_time_seconds", label, errors)
    if delay < 0 or (sample_time > 0 and abs(delay/sample_time-round(delay/sample_time)) > 1e-9):
        errors.append(f"{label}: dead time must be a non-negative whole multiple of sample time.")
    return IntegratorPath(str(item.get("source")), str(item.get("destination")),
                          gain if kind == "integrator" else 0.0, delay,
                          str(item.get("engineering_basis", "")), str(kind),
                          gain if kind != "integrator" else 0.0, tau)


@dataclass(frozen=True)
class LinearApplicationDefinition:
    name: str
    sample_time_seconds: float
    primary_cvs: tuple[PrimaryCV, ...]
    derived_cvs: tuple[DerivedCV, ...]
    sources: tuple[SourceVariable, ...]
    paths: tuple[IntegratorPath, ...]
    raw: dict
    model: object = None


def _number(item, key, label, errors, *, positive=False):
    value = item.get(key)
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value):
        errors.append(f"{label}: {key} must be a number.")
        return 0.0
    value = float(value)
    if positive and value <= 0.0:
        errors.append(f"{label}: {key} must be positive.")
    return value


def _identifier(item, label, errors):
    value = item.get("id")
    if not isinstance(value, str) or not ID_PATTERN.fullmatch(value):
        errors.append(
            f"{label}: id must start with a lower-case letter and contain only "
            "lower-case letters, digits and underscores."
        )
        return "invalid"
    return value


def load_definition(path: str | Path) -> LinearApplicationDefinition:
    path = Path(path)
    with path.open("rb") as stream:
        raw = tomllib.load(stream)
    return validate_definition(raw)


def validate_definition(raw) -> LinearApplicationDefinition:
    """Validate an already parsed linear or nonlinear application mapping."""

    errors = []
    for key in ("application", "controller"):
        if not isinstance(raw.get(key, {}), dict):
            errors.append(f"Use [{key}] table.")
    for key in ("cvs", "derived_cvs", "mvs", "measured_disturbances", "move_combinations", "model_paths", "model_checks"):
        if not isinstance(raw.get(key, []), list) or not all(
            isinstance(item, dict) for item in raw.get(key, [])
        ):
            errors.append(f"Use repeated [[{key}]] tables.")
    if set(raw) & {"plant", "scenario", "acceptance", "verification"}:
        errors.append("Legacy simulation/verification sections detected. Use simulation.toml and [[model_checks]].")
    if errors:
        raise ConfigurationError(errors)
    app = raw.get("application", {})
    name = app.get("name")
    if not isinstance(name, str) or not name.strip():
        errors.append("[application].name must be non-empty text.")
        name = "Invalid application"
    sample_time = _number(
        app, "sample_time_seconds", "[application]", errors, positive=True
    )
    nonlinear = app.get("model_type") == "nonlinear_python"
    if app.get("model_type") not in ("linear_paths", "nonlinear_python"):
        errors.append("Unknown application.model_type.")
    controller = raw.get("controller", {})
    if type(controller.get("use_measured_disturbances", True)) is not bool:
        errors.append("use_measured_disturbances must be true or false.")
    if controller.get("prediction_form") != "velocity":
        errors.append('[controller].prediction_form must be "velocity".')
    for key in ("prediction_horizon_samples", "control_horizon_samples"):
        value = controller.get(key)
        if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
            errors.append(f"[controller].{key} must be a positive whole number.")
    if (type(controller.get("control_horizon_samples")) is int
            and type(controller.get("prediction_horizon_samples")) is int
            and controller["control_horizon_samples"] > controller["prediction_horizon_samples"]):
        errors.append("Control horizon must not exceed prediction horizon.")
    if "maximum_solver_iterations" in controller:
        value = controller["maximum_solver_iterations"]
        if type(value) is not int or value <= 0:
            errors.append("maximum_solver_iterations must be a positive integer.")
    if "maximum_solver_cpu_seconds" in controller:
        _number(controller, "maximum_solver_cpu_seconds", "[controller]", errors, positive=True)

    primary_cvs = []
    identifiers = set()
    for index, item in enumerate(raw.get("cvs", []), start=1):
        label = f"CV #{index}"
        item_id = _identifier(item, label, errors)
        if item_id in identifiers:
            errors.append(f"{label}: duplicate id {item_id}.")
        identifiers.add(item_id)
        if item.get("source_type") != "instrumented_process":
            errors.append(f'{label}: source_type must be "instrumented_process".')
        target = item.get("target")
        zone_low, zone_high = item.get("zone_low"), item.get("zone_high")
        for key in ("target", "zone_low", "zone_high", "protected_low", "protected_high"):
            if key in item:
                item[key] = _number(item, key, label, errors)
        target = item.get("target")
        zone_low, zone_high = item.get("zone_low"), item.get("zone_high")
        if target is not None and (zone_low is not None or zone_high is not None):
            errors.append(f"{label}: use target OR zone, not both.")
        if zone_low is not None and zone_high is not None and zone_low >= zone_high:
            errors.append(f"{label}: zone_low must be below zone_high.")
        if target is None and (zone_low is None or zone_high is None):
            errors.append(f"{label}: define target or both zone_low and zone_high.")
        protected_low = item.get("protected_low")
        protected_high = item.get("protected_high")
        if protected_low is not None and protected_high is not None:
            if float(protected_low) >= float(protected_high):
                errors.append(f"{label}: protected low limit must be below high limit.")
        _number(item, "weight", label, errors, positive=True)
        _number(item, "normalization", label, errors, positive=True)
        if protected_low is not None or protected_high is not None:
            _number(item, "protected_limit_weight", label, errors, positive=True)
            _number(item, "protected_limit_normalization", label, errors, positive=True)
        primary_cvs.append(PrimaryCV(
            item_id, str(item.get("name", item_id)), str(item.get("unit", "")),
            _number(item, "initial", label, errors),
            None if target is None else float(target),
            None if zone_low is None else float(zone_low),
            None if zone_high is None else float(zone_high),
        ))
    if not primary_cvs:
        errors.append("At least one [[cvs]] entry is required.")

    derived_cvs = []
    primary_ids = {item.id for item in primary_cvs}
    for index, item in enumerate(raw.get("derived_cvs", []), start=1):
        label = f"Derived CV #{index}"
        item_id = _identifier(item, label, errors)
        if item_id in identifiers:
            errors.append(f"{label}: duplicate id {item_id}.")
        identifiers.add(item_id)
        if item.get("source_type") != "derived_auxiliary":
            errors.append(f'{label}: source_type must be "derived_auxiliary".')
        if item.get("calculation") != "rate_of_change":
            errors.append(f'{label}: this version supports calculation "rate_of_change".')
        source_cv = item.get("source_cv")
        if source_cv not in primary_ids:
            errors.append(f"Derived CV {item_id}: unknown source_cv {source_cv}.")
        alpha = _number(item, "filter_alpha", label, errors, positive=True)
        if alpha > 1.0:
            errors.append(f"{label}: filter_alpha must not exceed 1.0.")
        _number(item, "weight", label, errors, positive=True)
        _number(item, "normalization", label, errors, positive=True)
        derived_cvs.append(DerivedCV(
            item_id, str(item.get("name", item_id)), str(item.get("unit", "")),
            str(source_cv), alpha, _number(item, "initial", label, errors),
            _number(item, "target", label, errors),
        ))

    sources = []
    for kind, table in (("mv", "mvs"), ("dv", "measured_disturbances")):
        for index, item in enumerate(raw.get(table, []), start=1):
            label = f"{kind.upper()} #{index}"
            item_id = _identifier(item, label, errors)
            if item_id in identifiers:
                errors.append(f"{label}: duplicate id {item_id}.")
            identifiers.add(item_id)
            initial = _number(item, "initial", label, errors)
            if kind == "mv":
                lower = _number(item, "lower", label, errors)
                upper = _number(item, "upper", label, errors)
                if lower >= upper:
                    errors.append(f"MV {item_id}: lower limit must be below upper limit.")
                if not lower <= initial <= upper:
                    errors.append(
                        f"MV {item_id}: initial value {initial:g} is outside "
                        f"limits {lower:g} to {upper:g} {item.get('unit', '')}."
                    )
                _number(
                    item, "maximum_increase_per_sample", label, errors,
                    positive=True,
                )
                _number(
                    item, "maximum_decrease_per_sample", label, errors,
                    positive=True,
                )
                _number(item, "move_weight", label, errors, positive=True)
                _number(item, "move_normalization", label, errors, positive=True)
            sources.append(SourceVariable(
                item_id, str(item.get("name", item_id)),
                str(item.get("unit", "")), kind, initial,
            ))
    if not any(item.kind == "mv" for item in sources):
        errors.append("At least one [[mvs]] entry is required.")

    mv_count = sum(item.kind == "mv" for item in sources)
    combination_names = set()
    for index, item in enumerate(raw.get("move_combinations", []), start=1):
        label = f"Move combination #{index}"
        name = item.get("name")
        if not isinstance(name, str) or not name.strip():
            errors.append(f"{label}: name must be non-empty text.")
        elif name in combination_names:
            errors.append(f"{label}: duplicate name {name!r}.")
        else:
            combination_names.add(name)
        coefficients = item.get("coefficients")
        if not isinstance(coefficients, list) or len(coefficients) != mv_count:
            errors.append(f"{label}: coefficients must contain exactly {mv_count} numbers, in MV order.")
        elif any(
            not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value)
            for value in coefficients
        ):
            errors.append(f"{label}: coefficients must contain only finite numbers.")
        elif not any(float(value) != 0.0 for value in coefficients):
            errors.append(f"{label}: coefficients cannot all be zero.")
        _number(item, "weight", label, errors, positive=True)
        _number(item, "normalization", label, errors, positive=True)

    source_ids = {item.id for item in sources}
    paths = []
    for index, item in enumerate(raw.get("model_paths", []), start=1):
        source = item.get("source")
        destination = item.get("destination")
        label = f"Model path {source} -> {destination}"
        if source not in source_ids:
            errors.append(f"{label}: source does not identify an MV or measured DV.")
        if destination not in primary_ids:
            errors.append(f"{label}: destination does not identify a primary CV.")
        paths.append(parse_path(item, sample_time, label, errors))
    if not paths and not nonlinear:
        errors.append("At least one [[model_paths]] entry is required.")

    for item in raw.get("model_checks", []):
        if item.get("source") not in source_ids or item.get("destination") not in primary_ids:
            errors.append("Model check: source/destination must identify a controller source and primary CV.")
        kind = item.get("metric", "rate")
        if kind not in ("rate", "steady_change", "step_response"):
            errors.append("Model check metric must be rate, steady_change or step_response.")
        expected_key = "expected_rate_per_minute" if kind == "rate" else "expected_change"
        for key in ("step", expected_key, "tolerance"):
            _number(item, key, "Model check", errors, positive=(key == "tolerance"))
        if kind == "step_response":
            seconds = _number(item, "time_seconds", "Model check", errors)
            if seconds < 0:
                errors.append("Model check time_seconds must not be negative.")

    for variables, label in (([*primary_cvs, *derived_cvs], "CV"),
                             ([s for s in sources if s.kind == "mv"], "MV")):
        names = [v.name for v in variables]
        if any(not n.strip() for n in names) or len(names) != len(set(names)):
            errors.append(f"{label} display names must be non-empty and unique.")

    if errors:
        raise ConfigurationError(errors)
    return LinearApplicationDefinition(
        name.strip(), sample_time, tuple(primary_cvs), tuple(derived_cvs),
        tuple(sources), tuple(paths), raw,
    )
