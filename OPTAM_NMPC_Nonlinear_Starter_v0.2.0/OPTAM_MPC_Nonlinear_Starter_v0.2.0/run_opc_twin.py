"""Generic OPC UA server + independently configured digital twin."""
from __future__ import annotations

import argparse
import asyncio
import csv
from pathlib import Path
import time
import uuid

from asyncua import Server, ua

from nonlinear_runtime import load_definition, verify_model_checks, load_simulation, NonlinearPlant
from optam_mpc_linear.simulation import Measurements, evaluate_checks
from optam_mpc_linear.simulation import target_values, apply_target_events, targets_requested
from optam_mpc_linear.opc import (
    PROTOCOL, NODE_KEYS, encode, fingerprint, load_opc, read_frame, validate_command,
)
from run_simulation import save_results


async def run(application, simulation, opc_path):
    definition = load_definition(application)
    verify_model_checks(definition)
    settings, cfg = load_simulation(simulation, definition), load_opc(opc_path)
    if definition.sample_time_seconds / cfg["speed"] + cfg["command_timeout_seconds"] >= cfg["stale_timeout_seconds"]:
        raise ValueError("stale_timeout_seconds must exceed sample_time/speed + command_timeout_seconds.")
    # Validate the same startup measurement convention used in offline simulation.
    plant = NonlinearPlant(settings, definition.sample_time_seconds)
    measurements = Measurements(definition, plant.measurements())
    _, feedback = measurements.update(plant.measurements())
    targets = target_values(definition)
    mvs = {x.id: x.initial for x in definition.sources if x.kind == "mv"}
    run_id, owner = uuid.uuid4().hex, None
    records, solve_times = [], []
    output = Path(simulation).resolve().parent / "opc_results"
    output.mkdir(parents=True, exist_ok=True)
    (output / "current_run.json").write_text(encode(dict(
        run_id=run_id, application_hash=fingerprint(definition), state="starting")),
        encoding="utf-8")
    server = Server()
    await server.init()
    server.set_endpoint(cfg["endpoint"])
    server.set_server_name("OPTAM local supervised nonlinear twin")
    server.set_security_policy([ua.SecurityPolicyType.NoSecurity])
    ns = await server.register_namespace(cfg["namespace_uri"])
    folder = await server.nodes.objects.add_object(ua.NodeId("OPTAM", ns), "OPTAM")
    nodes = {}
    for key in NODE_KEYS:
        initial = False if key == "stop_node" else encode({})
        nodes[key] = await folder.add_variable(ua.NodeId(cfg[key], ns), cfg[key], initial)
        if key in ("command_node", "stop_node"):
            await nodes[key].set_writable()
    state, reason, interrupted = "waiting", "Waiting for first controller command.", False
    started = time.monotonic()
    csv_file = (output / "live.csv").open("w", newline="", encoding="utf-8")
    writer = None

    async def status(new_state, message, **extra):
        await nodes["status_node"].write_value(encode(dict(
            run_id=run_id, state=new_state, message=message, completed_samples=len(records)-1, **extra)))

    async def record(sample):
        nonlocal writer
        row = dict(sample=sample, time_minutes=sample * definition.sample_time_seconds / 60)
        row.update(plant.outputs)
        row.update(plant.inputs)
        row.update(feedback)
        if targets_requested(settings):
            row.update({f"target.{k}": v for k, v in targets.items()})
        encode(row)  # Reject NaN/Inf before publishing or saving.
        records.append(row)
        if writer is None:
            writer = csv.DictWriter(csv_file, fieldnames=list(row))
            writer.writeheader()
        writer.writerow(row)
        csv_file.flush()
        await nodes["telemetry_node"].write_value(encode(dict(run_id=run_id, values=row)))

    async with server:
        try:
            await record(0)
            await status(state, reason)
            print(f"Twin READY at {cfg['endpoint']}", flush=True)
            print("Start the controller in another window. Ctrl+C stops; no plant connection.", flush=True)
            for sample in range(settings["simulation"]["samples"]):
                cycle_start = time.monotonic()
                plant.events(sample)
                apply_target_events(settings, sample, targets)
                frame = dict(protocol=PROTOCOL, application_hash=fingerprint(definition),
                             run_id=run_id, sample=sample, sample_time_seconds=definition.sample_time_seconds,
                             cvs=plant.measurements(), dvs=plant.measured_dvs(), mvs=dict(mvs),
                             targets=dict(targets))
                await nodes["sample_node"].write_value(encode(frame))
                timeout = cfg["startup_timeout_seconds"] if sample == 0 else cfg["command_timeout_seconds"]
                deadline = time.monotonic() + timeout
                refreshed = time.monotonic()
                while True:
                    if await nodes["stop_node"].read_value():
                        state, reason = "stopped", "Operator requested stop; simulated state frozen."
                        break
                    command = await read_frame(nodes["command_node"])
                    if command:
                        # Previous accepted command remains in its node until replaced.
                        previous = command.get("run_id") == run_id and command.get("sample") == sample-1
                        if not previous:
                            mvs = validate_command(command, frame, definition, owner)
                            owner = command["controller_id"]
                            solve_times.append(command["solver_seconds"])
                            break
                    if time.monotonic() >= deadline:
                        raise TimeoutError(f"No valid command for sample {sample} within {timeout:g} s.")
                    if sample == 0 and time.monotonic() - refreshed >= 1:
                        # The initial steady sample remains valid while waiting for startup.
                        await nodes["sample_node"].write_value(encode(frame))
                        refreshed = time.monotonic()
                    await asyncio.sleep(cfg["poll_seconds"])
                if state == "stopped":
                    break
                plant.set_mvs(mvs)
                plant.step()
                _, feedback = measurements.update(plant.measurements())
                await record(sample+1)
                state, reason = "running", "Last command accepted; plant advanced one sample."
                await status(state, reason)
                if sample % 20 == 0:
                    print(f"sample={sample+1} outputs={plant.outputs} inputs={plant.inputs}", flush=True)
                # Pacing is a minimum wall interval, never a real-time deadline guarantee.
                until = cycle_start + definition.sample_time_seconds / cfg["speed"]
                while time.monotonic() < until:
                    if await nodes["stop_node"].read_value():
                        state, reason = "stopped", "Operator requested stop; simulated state frozen."
                        break
                    await asyncio.sleep(min(cfg["poll_seconds"], max(0, until-time.monotonic())))
                if state == "stopped":
                    break
            else:
                state, reason = "completed", "All configured samples completed."
        except asyncio.CancelledError:
            interrupted = True
            state, reason = "stopped", "Keyboard interrupt; simulated state frozen."
        except Exception as exc:
            state, reason = "fault", f"{type(exc).__name__}: {exc}"
        finally:
            csv_file.close()
            checks = evaluate_checks(records, settings["checks"]) if records else []
            passed = state == "completed" and all(c["passed"] for c in checks)
            report = dict(application=definition.name, run_id=run_id, mode="supervised_opc",
                          terminal_state=state, message=reason, samples=max(0, len(records)-1),
                          requested_samples=settings["simulation"]["samples"],
                          elapsed_seconds=time.monotonic()-started,
                          failures=[] if state == "completed" else [dict(message=reason)],
                          mean_solver_seconds=sum(solve_times)/len(solve_times) if solve_times else None,
                          checks=checks, passed=passed)
            if records:
                await status("saving", "Writing result files.")
                await asyncio.to_thread(save_results, records, report, settings, output)
            await status(state, reason, passed=passed)
            print(f"Twin {state.upper()}: {reason}", flush=True)
            print(f"OPC closed-loop acceptance: {'PASS' if passed else 'FAIL'}; results: {output}", flush=True)
        if not interrupted:
            print("Server remains available for inspection. Use opc_tools.bat ... stop or Ctrl+C.", flush=True)
            try:
                while not await nodes["stop_node"].read_value():
                    await asyncio.sleep(cfg["poll_seconds"])
            except asyncio.CancelledError:
                pass
    return 0 if passed else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("application", type=Path)
    parser.add_argument("simulation", type=Path)
    parser.add_argument("opc", type=Path)
    args = parser.parse_args()
    try:
        return asyncio.run(run(args.application, args.simulation, args.opc))
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f"Twin FAULT: {type(exc).__name__}: {exc}", flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
