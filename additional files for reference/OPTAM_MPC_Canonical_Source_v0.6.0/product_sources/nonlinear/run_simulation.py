"""Run any supported application and separately configured simulated plant."""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import time
import tomllib

from optam_mpc_linear import ConfigurationError
from nonlinear_runtime import load_definition, verify_model_checks, load_simulation, NonlinearPlant
from optam_mpc_linear.simulation import Measurements, evaluate_checks
from optam_mpc_linear.simulation import target_values, apply_target_events, update_targets, targets_requested
from optam_mpc.provenance import report_provenance


def simulate(definition, settings, controller_factory=None):
    verify_model_checks(definition)
    if controller_factory is None:
        from nonlinear_runtime import build_controller
        controller_factory = build_controller
    from optam_mpc.controller import NMPCOptimizationError
    controller, _, mvs, _ = controller_factory(definition)
    mv_ids = [x.id for x in definition.sources if x.kind == "mv"]
    dv_ids = [x.id for x in definition.sources if x.kind == "dv"]
    plant = NonlinearPlant(settings, definition.sample_time_seconds)
    measurements = Measurements(definition, plant.measurements())
    state, feedback = measurements.update(plant.measurements())
    targets = target_values(definition)
    records = []
    failures, solve_times = [], []

    def record(sample):
        row = dict(sample=sample, time_minutes=sample*definition.sample_time_seconds/60)
        row.update(plant.outputs)
        row.update(plant.inputs)
        row.update(feedback)
        if targets_requested(settings):
            row.update({f"target.{k}": v for k, v in targets.items()})
        if not all(math.isfinite(v) for v in row.values()):
            raise RuntimeError(f"Non-finite simulation value at sample {sample}.")
        records.append(row)

    record(0)
    started = time.perf_counter()
    for sample in range(settings["simulation"]["samples"]):
        plant.events(sample)
        previous_targets = dict(targets)
        apply_target_events(settings, sample, targets)
        update_targets(controller, definition, previous_targets, targets)
        dvs = plant.measured_dvs()
        try:
            result = controller.calculate_move(state, tuple(dvs[k] for k in dv_ids), mvs)
            mvs = result.first_input
            solve_times.append(result.solver_time_seconds)
        except NMPCOptimizationError as exc:
            failures.append(dict(sample=sample, message=str(exc)))
            # Development simulation fallback: hold last move; count the failure.
        plant.set_mvs(dict(zip(mv_ids, mvs)))
        plant.step()
        state, feedback = measurements.update(plant.measurements())
        record(sample+1)
    checks = evaluate_checks(records, settings["checks"])
    passed = (len(failures) <= settings["simulation"].get("maximum_solver_failures", 0)
              and all(c["passed"] for c in checks))
    report = dict(application=definition.name, samples=settings["simulation"]["samples"],
                  elapsed_seconds=time.perf_counter()-started, failures=failures, checks=checks,
                  mean_solver_seconds=sum(solve_times)/len(solve_times) if solve_times else None,
                  passed=passed)
    return records, report


def save_results(records, report, settings, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    output.mkdir(parents=True, exist_ok=True)
    with (output / "results.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    (output / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    panels = settings["plots"]
    figure, axes = plt.subplots(len(panels), 1, figsize=(10, 3*len(panels)),
                                sharex=True, squeeze=False)
    for ax, panel in zip(axes[:, 0], panels):
        for signal in panel["signals"]:
            ax.plot([r["time_minutes"] for r in records], [r[signal] for r in records], label=signal)
        ax.set_ylabel(panel["ylabel"])
        ax.set_title(panel.get("title", ""))
        ax.legend(loc="best")
        ax.grid(alpha=0.25)
    axes[-1, 0].set_xlabel("Time (minutes)")
    figure.suptitle(report["application"])
    figure.tight_layout()
    figure.savefig(output / "results.png", dpi=150)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("application", type=Path)
    parser.add_argument("simulation", type=Path)
    parser.add_argument("--output", type=Path, help="Optional separate results folder.")
    args = parser.parse_args()
    try:
        definition = load_definition(args.application)
        settings = load_simulation(args.simulation, definition)
        records, report = simulate(definition, settings)
        report["application_file"] = str(args.application.resolve())
        report["simulation_file"] = str(args.simulation.resolve())
        report["provenance"] = report_provenance(args.application.resolve(), args.simulation.resolve())
        output = args.output.resolve() if args.output else args.simulation.resolve().parent / "results"
        save_results(records, report, settings, output)
    except (ConfigurationError, OSError, ValueError, KeyError, TypeError, AttributeError) as exc:
        print(f"Simulation: FAIL\n{exc}")
        return 1
    print(f"Completed: {report['samples']} cycles in {report['elapsed_seconds']:.2f} s")
    print(f"Solver failures: {len(report['failures'])}")
    if report["mean_solver_seconds"] is not None:
        print(f"Mean solver time: {report['mean_solver_seconds']:.4f} s")
    for check in report["checks"]:
        print(f"{check['signal']} {check['metric']}: {check['actual']:.8g}; "
              f"limit {check['limit']:g}: {'PASS' if check['passed'] else 'FAIL'}")
    print(f"Results: {output}")
    print(f"Closed-loop acceptance: {'PASS' if report['passed'] else 'FAIL'}")
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
