"""Build and evaluate the configured linear integrator model."""

from __future__ import annotations

from .definition import LinearApplicationDefinition
from .definition import ConfigurationError
from .dynamics import model_check_value


def verify_model_checks(definition):
    """Reject failed independent numerical checks before simulation."""
    errors = []
    for check in definition.raw.get("model_checks", []):
        actual = model_check_value(definition, check)
        expected = check.get("expected_change", check.get("expected_rate_per_minute"))
        if abs(actual - expected) > check["tolerance"]:
            errors.append(f"Model check {check['source']} -> {check['destination']}: "
                          f"response {actual:g}, expected {expected:g}.")
    if errors:
        raise ConfigurationError(errors)


class LinearIntegratorModel:
    """Deviation-form linear model constructed entirely from configuration."""

    def __init__(self, definition: LinearApplicationDefinition):
        self.definition = definition
        self.sample_time_minutes = definition.sample_time_seconds / 60.0
        self.primary_initial = {
            item.id: item.initial for item in definition.primary_cvs
        }
        self.source_initial = {item.id: item.initial for item in definition.sources}

    def rates_per_minute(self, source_values):
        if any(p.type != "integrator" or p.dead_time_seconds != 0 for p in self.definition.paths):
            raise ValueError("This legacy rate helper requires zero-delay integrators; use dynamic path evaluation.")
        rates = {item.id: 0.0 for item in self.definition.primary_cvs}
        for path in self.definition.paths:
            deviation = source_values[path.source] - self.source_initial[path.source]
            rates[path.destination] += path.integrating_gain * deviation
        return rates

    def step(self, primary_values, source_values):
        rates = self.rates_per_minute(source_values)
        return {
            cv_id: primary_values[cv_id] + self.sample_time_minutes * rates[cv_id]
            for cv_id in primary_values
        }

    def predicted_derived_values(self, previous_primary, current_primary):
        values = {}
        for item in self.definition.derived_cvs:
            values[item.id] = (
                current_primary[item.source_cv] - previous_primary[item.source_cv]
            ) / self.sample_time_minutes
        return values


class DerivedRateCalculator:
    """Measurement filter and rate calculator configured as a derived CV."""

    def __init__(self, alpha, sample_time_seconds, initial_measurement):
        self.alpha = float(alpha)
        self.sample_time_minutes = float(sample_time_seconds) / 60.0
        self.filtered = float(initial_measurement)
        self.has_two_valid_samples = False

    def update(self, measurement):
        previous = self.filtered
        self.filtered = self.alpha * float(measurement) + (1.0 - self.alpha) * previous
        if not self.has_two_valid_samples:
            self.has_two_valid_samples = True
            return self.filtered, 0.0
        return self.filtered, (self.filtered - previous) / self.sample_time_minutes
