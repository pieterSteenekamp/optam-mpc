"""Adapt a configuration-generated linear model to the generic controller."""

from __future__ import annotations
from .dynamics import PathBank

from optam_mpc.controller import (
    ControlledVariableConfiguration,
    GenericNMPCConfiguration,
    GenericNonlinearMPC,
    ManipulatedMoveCombinationConfiguration,
    ManipulatedVariableConfiguration,
)


class LinearVelocityStateModel:
    """Primary CV states followed by configured derived-rate CV states."""

    def __init__(self, definition):
        self.definition = definition
        self.primary_count = len(definition.primary_cvs)
        self.derived_count = len(definition.derived_cvs)
        self.mv_sources = tuple(item for item in definition.sources if item.kind == "mv")
        self.dv_sources = tuple(item for item in definition.sources if item.kind == "dv")
        self.primary_index = {
            item.id: index for index, item in enumerate(definition.primary_cvs)
        }
        self.source_initial = {item.id: item.initial for item in definition.sources}
        self.state_dimension = self.primary_count + self.derived_count
        self.manipulated_input_dimension = len(self.mv_sources)
        self.disturbance_dimension = len(self.dv_sources)
        self.output_dimension = self.state_dimension
        self.sample_time_minutes = definition.sample_time_seconds / 60.0
        self.bank = PathBank(definition.paths, definition.sample_time_seconds,
                             self.source_initial, self.primary_index)
        self.state_dimension += self.bank.dimension

    def transition_values(self, state, inputs, disturbances):
        source_values = {}
        for item, value in zip(self.mv_sources, inputs):
            source_values[item.id] = value
        for item, value in zip(self.dv_sources, disturbances):
            source_values[item.id] = value
        internal, increments = self.bank.advance(state[self.output_dimension:], source_values)
        next_primary = tuple(
            state[index] + increments[item.id]
            for index, item in enumerate(self.definition.primary_cvs)
        )
        next_derived = tuple(
            increments[item.source_cv] / self.sample_time_minutes
            for item in self.definition.derived_cvs
        )
        return (*next_primary, *next_derived, *internal)

    def output_values(self, state, inputs, disturbances):
        del inputs, disturbances
        return tuple(state[:self.output_dimension])


class LinearController:
    """Maintain unmeasured path/queue states from applied MVs and prior DVs."""
    def __init__(self, core, model):
        self.core, self.model = core, model
        self.internal = model.bank.initial
        self.previous_dvs = None

    def __getattr__(self, name):
        return getattr(self.core, name)

    def calculate_move(self, state_values, disturbance_values, previous_input_values):
        dvs = tuple(disturbance_values)
        if not self.model.definition.raw["controller"].get("use_measured_disturbances", True):
            dvs = tuple(x.initial for x in self.model.dv_sources)
        if self.previous_dvs is not None:
            sources = {x.id: v for x, v in zip(self.model.mv_sources, previous_input_values)}
            sources.update({x.id: v for x, v in zip(self.model.dv_sources, self.previous_dvs)})
            self.internal, _ = self.model.bank.advance(self.internal, sources)
        self.previous_dvs = dvs
        try:
            return self.core.calculate_move((*state_values, *self.internal), dvs, previous_input_values)
        except Exception:
            self.core.reset_measurement_history()
            raise


def _primary_cv(item, output_index, raw):
    common = dict(
        name=item.name,
        output_index=output_index,
        weight=float(raw["weight"]),
        normalization=float(raw["normalization"]),
        constraint_low=raw.get("protected_low"),
        constraint_high=raw.get("protected_high"),
        constraint_weight=raw.get("protected_limit_weight"),
        constraint_normalization=raw.get("protected_limit_normalization"),
    )
    if item.target is not None:
        return ControlledVariableConfiguration.for_target(target=item.target, **common)
    return ControlledVariableConfiguration.for_zone(
        zone_low=item.zone_low, zone_high=item.zone_high, **common
    )


def build_controller(definition, model=None):
    linear = model is None
    model = LinearVelocityStateModel(definition) if linear else model
    cvs = [
        _primary_cv(item, index, definition.raw["cvs"][index])
        for index, item in enumerate(definition.primary_cvs)
    ]
    for offset, item in enumerate(definition.derived_cvs):
        raw = definition.raw["derived_cvs"][offset]
        cvs.append(ControlledVariableConfiguration.for_target(
            name=item.name,
            output_index=len(definition.primary_cvs) + offset,
            target=item.target,
            weight=float(raw["weight"]),
            normalization=float(raw["normalization"]),
        ))
    mvs = tuple(
        ManipulatedVariableConfiguration(
            name=item.name,
            lower_bound=float(raw["lower"]),
            upper_bound=float(raw["upper"]),
            maximum_change_per_sample=max(
                float(raw["maximum_increase_per_sample"]),
                float(raw["maximum_decrease_per_sample"]),
            ),
            maximum_increase_per_sample=float(raw["maximum_increase_per_sample"]),
            maximum_decrease_per_sample=float(raw["maximum_decrease_per_sample"]),
            move_weight=float(raw["move_weight"]),
            move_normalization=float(raw["move_normalization"]),
        )
        for item, raw in zip(model.mv_sources, definition.raw["mvs"])
    )
    settings = definition.raw["controller"]
    combinations = tuple(
        ManipulatedMoveCombinationConfiguration(
            name=item["name"],
            coefficients=tuple(float(value) for value in item["coefficients"]),
            weight=float(item["weight"]),
            normalization=float(item["normalization"]),
        )
        for item in definition.raw.get("move_combinations", [])
    )
    controller = GenericNonlinearMPC(
        model,
        GenericNMPCConfiguration(
            prediction_horizon_samples=int(settings["prediction_horizon_samples"]),
            control_horizon_samples=int(settings["control_horizon_samples"]),
            controlled_variables=tuple(cvs),
            manipulated_variables=mvs,
            move_combinations=combinations,
            maximum_solver_iterations=int(settings.get("maximum_solver_iterations", 300)),
            maximum_solver_cpu_seconds=settings.get("maximum_solver_cpu_seconds"),
        ),
        prediction_form=settings.get("prediction_form", "velocity"),
    )
    initial_state = tuple(item.initial for item in definition.primary_cvs) + tuple(
        item.initial for item in definition.derived_cvs
    )
    initial_mvs = tuple(item.initial for item in model.mv_sources)
    initial_dvs = tuple(item.initial for item in model.dv_sources)
    if linear and (model.bank.dimension or not settings.get("use_measured_disturbances", True)):
        controller = LinearController(controller, model)
    return controller, initial_state, initial_mvs, initial_dvs
