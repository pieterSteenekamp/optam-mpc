"""Validate trusted nonlinear model Python and its engineering configuration."""
import argparse
from pathlib import Path
from nonlinear_runtime import load_definition, load_simulation, verify_model_checks, build_controller
from optam_mpc_linear.opc import load_opc


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("application", type=Path)
    parser.add_argument("simulation", nargs="?", type=Path)
    parser.add_argument("opc", nargs="?", type=Path)
    args = parser.parse_args()
    try:
        definition = load_definition(args.application)
        print(f"Application: {definition.name}")
        print(f"Model SHA-256: {definition.model.sha256}")
        print(f"Numeric/symbolic domain probes: {definition.model.probe_count}")
        for name, actual in verify_model_checks(definition):
            print(f"Model check {name}: {actual}: PASS")
        if args.simulation:
            settings = load_simulation(args.simulation, definition)
            print(f"Independent plant parameters: loaded; {settings['simulation']['samples']} samples")
        if args.opc:
            cfg = load_opc(args.opc)
            if definition.sample_time_seconds/cfg["speed"]+cfg["command_timeout_seconds"] >= cfg["stale_timeout_seconds"]:
                raise ValueError("Stale timeout must exceed sample_time/speed + command timeout.")
            print(f"Local OPC endpoint: {cfg['endpoint']}")
        controller, x, u, d = build_controller(definition)
        result = controller.calculate_move(x, d, u)
        print(f"Nominal solver smoke test: PASS; first MV values {result.first_input}")
        print("Nonlinear application validation: PASS")
        print("This verifies configuration/model consistency, not plant suitability or safety.")
        return 0
    except Exception as exc:
        print(f"Nonlinear application validation: FAIL\n{type(exc).__name__}: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
