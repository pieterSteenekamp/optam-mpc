"""Optional slow reference test: isolated runtime, full OPC/offline comparison.

Run: python tests/verify_full_opc.py
"""
import json
import argparse
import tomllib
import os
from pathlib import Path
import shutil
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from test_opc import OPCIntegrationTests, ROOT


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--example", default="01_surge_vessel",
                        choices=["01_surge_vessel", "02_heat_exchanger"])
    args = parser.parse_args()
    reference = ROOT / "examples" / args.example
    trial = OPCIntegrationTests()
    trial.setUp()
    try:
        # Use the complete supplied engineering values; no example code is copied.
        for name, target in (("application.toml", trial.app), ("simulation.toml", trial.sim)):
            shutil.copy2(reference / name, target)
        with trial.sim.open("rb") as stream:
            sample_count = tomllib.load(stream)["simulation"]["samples"]
        trial.opc.write_text(trial.opc.read_text().replace("startup_timeout_seconds = 3.0",
                                                        "startup_timeout_seconds = 30.0")
                             .replace("command_timeout_seconds = 2.0", "command_timeout_seconds = 15.0")
                             .replace("stale_timeout_seconds = 5.0", "stale_timeout_seconds = 30.0"))
        from optam_mpc_linear.opc import load_opc
        trial.cfg = load_opc(trial.opc)
        runtime = trial.folder / "runtime"
        runtime.mkdir()
        for name in ("optam_mpc", "optam_mpc_linear"):
            shutil.copytree(ROOT / name, runtime / name, ignore=shutil.ignore_patterns("__pycache__"))
        for name in ("run_opc_twin.py", "run_opc_controller.py", "run_simulation.py"):
            shutil.copy2(ROOT / name, runtime / name)
        assert not (runtime / "examples").exists()
        import test_opc
        original_root = test_opc.ROOT
        test_opc.ROOT = runtime
        try:
            twin = trial.start("run_opc_twin.py", trial.app, trial.sim, trial.opc)
            trial.ready()
            controller = trial.start("run_opc_controller.py", trial.app, trial.opc)
            code = controller.wait(timeout=600)
            if code:
                raise RuntimeError((trial.folder / "run_opc_controller.py.log").read_text())
            report = trial.wait_report()
            assert report["passed"] and report["samples"] == sample_count, report
            print("Full OPC run: PASS", flush=True)
            offline = trial.start("run_simulation.py", trial.app, trial.sim)
            assert offline.wait(timeout=600) == 0
            assert (trial.folder/"results"/"results.csv").read_text() == (trial.folder/"opc_results"/"results.csv").read_text()
            print(f"All {sample_count+1} recorded rows exactly match offline simulation: PASS", flush=True)
            result_dir = reference / "verified_opc"
            result_dir.mkdir(exist_ok=True)
            for filename in ("report.json", "results.png", "results.csv"):
                shutil.copy2(trial.folder / "opc_results" / filename, result_dir / filename)
            print(json.dumps(report, indent=2), flush=True)
        finally:
            test_opc.ROOT = original_root
    finally:
        trial.doCleanups()


if __name__ == "__main__":
    main()
