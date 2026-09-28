"""Generic controller client. Reads only measured data, never twin telemetry."""
from __future__ import annotations

import argparse
import asyncio
from pathlib import Path
import time
import uuid

from asyncua import Client

from optam_mpc_linear import load_definition
from optam_mpc_linear.builder import verify_model_checks
from optam_mpc_linear.controller_adapter import build_controller
from optam_mpc_linear.simulation import Measurements
from optam_mpc_linear.simulation import target_values, update_targets
from optam_mpc_linear.opc import (
    client_nodes, encode, fingerprint, load_opc, read_frame, validate_sample,
)


async def run(application, opc_path):
    definition = load_definition(application)
    verify_model_checks(definition)
    cfg = load_opc(opc_path)
    controller, _, _, _ = build_controller(definition)
    mv_ids = [x.id for x in definition.sources if x.kind == "mv"]
    dv_ids = [x.id for x in definition.sources if x.kind == "dv"]
    owner = uuid.uuid4().hex
    targets = target_values(definition)
    expected, run_id, measurements = 0, None, None
    print(f"Connecting to {cfg['endpoint']}; one controller only.", flush=True)
    # No automatic reconnect: explicitly restart both processes after a fault.
    async with Client(cfg["endpoint"], timeout=cfg["request_timeout_seconds"]) as client:
        nodes = await client_nodes(client, cfg)
        last_progress = time.monotonic()
        while True:
            status = await read_frame(nodes["status_node"])
            state = status.get("state")
            if state in ("completed", "fault", "stopped"):
                print(f"Twin {state}: {status.get('message', '')}", flush=True)
                return 0 if state == "completed" and status.get("passed") else 2
            frame = await read_frame(nodes["sample_node"])
            if frame.get("sample") == expected:
                frame = await read_frame(nodes["sample_node"], cfg["stale_timeout_seconds"])
                validate_sample(frame, definition, expected, run_id)
                new_targets = frame.get("targets", targets)
                update_targets(controller, definition, targets, new_targets)
                targets = dict(new_targets)
                run_id = frame["run_id"]
                if measurements is None:
                    measurements = Measurements(definition, frame["cvs"])
                state_values, _ = measurements.update(frame["cvs"])
                command = dict(run_id=run_id, sample=expected, controller_id=owner,
                               application_hash=fingerprint(definition))
                try:
                    result = await asyncio.to_thread(
                        controller.calculate_move, state_values,
                        tuple(frame["dvs"][k] for k in dv_ids), tuple(frame["mvs"][k] for k in mv_ids))
                    command.update(status="ok", mvs=dict(zip(mv_ids, result.first_input)),
                                   solver_seconds=result.solver_time_seconds)
                except Exception as exc:
                    command.update(status="fault", message=f"{type(exc).__name__}: {exc}")
                    await nodes["command_node"].write_value(encode(command))
                    raise
                # If the twin timed out while solving, do not send the late move.
                current = await read_frame(nodes["status_node"])
                if current.get("state") in ("fault", "stopped", "completed"):
                    raise RuntimeError("Twin stopped while solving; command not sent.")
                await nodes["command_node"].write_value(encode(command))
                if expected % 20 == 0:
                    print(f"sample={expected} CVs={frame['cvs']} MVs={command['mvs']} "
                          f"solve={result.solver_time_seconds:.3f}s", flush=True)
                expected += 1
                last_progress = time.monotonic()
            else:
                if frame.get("run_id") not in (None, run_id) and run_id is not None:
                    raise ValueError("Twin run changed; restart explicitly.")
                if type(frame.get("sample")) is int and frame["sample"] > expected:
                    raise ValueError("Controller missed a sample; refusing to resynchronise silently.")
                if time.monotonic() - last_progress > cfg["stale_timeout_seconds"]:
                    raise TimeoutError("No fresh plant sample; controller stopped without writing new MVs.")
            await asyncio.sleep(cfg["poll_seconds"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("application", type=Path)
    parser.add_argument("opc", type=Path)
    args = parser.parse_args()
    try:
        return asyncio.run(run(args.application, args.opc))
    except KeyboardInterrupt:
        print("Controller stopped. Twin will time out and freeze; restart both for a fresh run.")
        return 130
    except Exception as exc:
        print(f"Controller FAULT: {type(exc).__name__}: {exc}", flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
