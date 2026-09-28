"""Validate a linear application and print an engineering model report."""

from __future__ import annotations

import argparse
from pathlib import Path

from optam_mpc_linear import ConfigurationError, LinearIntegratorModel, load_definition
from optam_mpc_linear.simulation import load_simulation
from optam_mpc_linear.opc import load_opc
from optam_mpc_linear.dynamics import model_check_value


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("configuration", type=Path)
    parser.add_argument("simulation", type=Path, nargs="?")
    parser.add_argument("opc", type=Path, nargs="?")
    args = parser.parse_args()
    try:
        definition = load_definition(args.configuration)
        responses = [model_check_value(definition,c) for c in definition.raw.get("model_checks",[])]
        if args.simulation:
            load_simulation(args.simulation, definition)
        if args.opc:
            opc = load_opc(args.opc)
            if definition.sample_time_seconds / opc["speed"] + opc["command_timeout_seconds"] >= opc["stale_timeout_seconds"]:
                raise ValueError("stale timeout must exceed sample_time/speed + command timeout.")
    except (ConfigurationError, OSError, ValueError, KeyError, TypeError, AttributeError) as exc:
        print("Linear application validation: FAIL")
        for error in getattr(exc, "errors", [str(exc)]):
            print(f"  - {error}")
        return 1

    model = LinearIntegratorModel(definition)
    print("Linear application validation: PASS")
    print(f"  Application: {definition.name}")
    print(f"  Sample time: {definition.sample_time_seconds:g} s")
    print(f"  Primary CVs: {len(definition.primary_cvs)}")
    print(f"  Derived CVs: {len(definition.derived_cvs)}")
    print(f"  MVs: {sum(item.kind == 'mv' for item in definition.sources)}")
    print(f"  Measured DVs: {sum(item.kind == 'dv' for item in definition.sources)}")
    print("  Dynamic paths:")
    for path in definition.paths:
        print(
            f"    {path.source} -> {path.destination}: {path.type}, "
            f"gain={path.integrating_gain if path.type == 'integrator' else path.gain:.10g}, "
            f"tau={path.time_constant_seconds:g} s, delay={path.dead_time_seconds:g} s"
        )

    failed = False
    for check, rate in zip(definition.raw.get("model_checks", []), responses):
        expected = check.get("expected_change", check.get("expected_rate_per_minute"))
        status = "PASS" if abs(rate - expected) <= check["tolerance"] else "FAIL"
        failed |= status == "FAIL"
        print(
            f"  {check['source']} -> {check['destination']}, step {check['step']:g}: "
            f"{check.get('metric', 'rate')} response {rate:.10g}; expected {expected:.10g}: {status}"
        )
    if not definition.raw.get("model_checks"):
        print("  Model response checks: NOT CONFIGURED (no independent numerical verification).")
    if args.simulation:
        print("Simulation configuration validation: PASS")
    if args.opc:
        print("OPC configuration validation: PASS (local supervised twin protocol)")
        print(f"  Endpoint: {opc['endpoint']}")
        print(f"  Namespace URI: {opc['namespace_uri']}")
        simulation = load_simulation(args.simulation, definition)
        for item in simulation["plant_outputs"]:
            print(f"  CV mapping: plant {item['id']} -> controller {item['cv']}")
        for item in simulation["plant_inputs"]:
            print(f"  Input mapping: {item['id']} -> {item.get('controller_id', 'HIDDEN FROM CONTROLLER')}")
    print(f"Validation and model checks: {'FAIL' if failed else 'PASS'}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
