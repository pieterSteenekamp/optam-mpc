"""Translate a compact engineering configuration into the generic NMPC."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import tomllib

from .controller import (
    ControlledVariableConfiguration,
    GenericNMPCConfiguration,
    GenericNonlinearMPC,
    ManipulatedMoveCombinationConfiguration,
    ManipulatedVariableConfiguration,
)


def read_config(path: str | Path) -> dict:
    with Path(path).open("rb") as stream:
        return tomllib.load(stream)


def load_model(model_path: str | Path, parameters: dict):
    path = Path(model_path).resolve()
    spec = importlib.util.spec_from_file_location("optam_user_model", path)
    if spec is None or spec.loader is None:
        raise ValueError(f"Cannot import application model: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module.create_model(parameters), module


def load_controller(config_path: str | Path):
    """Return `(controller, model_module, config)` for one application."""

    config_path = Path(config_path).resolve()
    config = read_config(config_path)
    model, module = load_model(
        config_path.parent / config["application"]["model_file"],
        config.get("model_parameters", {}),
    )

    cvs = []
    for item in config["controlled_variables"]:
        common = dict(
            name=item["name"],
            output_index=item["output_index"],
            weight=item["weight"],
            normalization=item["normalization"],
            rate_weight=item.get("rate_weight"),
            rate_normalization=item.get("rate_normalization"),
            constraint_low=item.get("constraint_low"),
            constraint_high=item.get("constraint_high"),
            constraint_weight=item.get("constraint_weight"),
            constraint_normalization=item.get("constraint_normalization"),
        )
        if "target" in item:
            cvs.append(ControlledVariableConfiguration.for_target(
                target=item["target"], **common
            ))
        else:
            cvs.append(ControlledVariableConfiguration.for_zone(
                zone_low=item["zone_low"], zone_high=item["zone_high"], **common
            ))

    mvs = tuple(
        ManipulatedVariableConfiguration(
            name=item["name"],
            lower_bound=item["lower"],
            upper_bound=item["upper"],
            maximum_change_per_sample=max(
                item["maximum_increase_per_sample"],
                item["maximum_decrease_per_sample"],
            ),
            maximum_increase_per_sample=item["maximum_increase_per_sample"],
            maximum_decrease_per_sample=item["maximum_decrease_per_sample"],
            move_weight=item["move_weight"],
            move_normalization=item["move_normalization"],
        )
        for item in config["manipulated_variables"]
    )
    tuning = config["controller"]
    combinations = tuple(
        ManipulatedMoveCombinationConfiguration(
            name=item["name"],
            coefficients=tuple(float(value) for value in item["coefficients"]),
            weight=float(item["weight"]),
            normalization=float(item["normalization"]),
        )
        for item in config.get("move_combinations", [])
    )
    controller = GenericNonlinearMPC(
        model,
        GenericNMPCConfiguration(
            prediction_horizon_samples=tuning["prediction_horizon_samples"],
            control_horizon_samples=tuning["control_horizon_samples"],
            controlled_variables=tuple(cvs),
            manipulated_variables=mvs,
            move_combinations=combinations,
            maximum_solver_iterations=tuning.get("maximum_solver_iterations", 300),
            maximum_solver_cpu_seconds=tuning.get("maximum_solver_cpu_seconds"),
        ),
        prediction_form=tuning.get("prediction_form", "velocity"),
    )
    return controller, module, config
