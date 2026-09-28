"""Generic read-only status/live trends and explicit simulated-test stop."""
import argparse
import asyncio
from pathlib import Path
import json

from asyncua import Client
from optam_mpc_linear.opc import load_opc, client_nodes, read_frame


async def run(args):
    cfg = load_opc(args.opc)
    async with Client(cfg["endpoint"], timeout=cfg["request_timeout_seconds"]) as client:
        nodes = await client_nodes(client, cfg)
        if args.action == "stop":
            await nodes["stop_node"].write_value(True)
            print("Stop requested. The simulated plant will freeze; restart both processes for another run.")
            return
        if args.action == "status":
            print(json.dumps(await read_frame(nodes["status_node"]), indent=2))
            print(json.dumps(await read_frame(nodes["telemetry_node"]), indent=2))
            return
        import matplotlib.pyplot as plt
        import tomllib
        if args.simulation is None:
            raise ValueError("trend requires the simulation.toml path.")
        with args.simulation.open("rb") as stream:
            panels = tomllib.load(stream)["plots"]
        plt.ion()
        fig, axes = plt.subplots(len(panels), 1, figsize=(10, 3*len(panels)), sharex=True, squeeze=False)
        traces = []
        for ax, panel in zip(axes[:, 0], panels):
            ax.set_title(panel.get("title", ""))
            ax.set_ylabel(panel["ylabel"])
            ax.grid(alpha=.25)
            for key in panel["signals"]:
                line, = ax.plot([], [], label=key)
                traces.append((line, key))
            ax.legend(loc="best")
        axes[-1, 0].set_xlabel("Simulated time (minutes)")
        fig.tight_layout()
        plt.show(block=False)
        records, seen, run_id = [], None, None
        while plt.fignum_exists(fig.number):
            frame = await read_frame(nodes["telemetry_node"])
            if frame.get("values"):
                if frame["run_id"] != run_id:
                    records, seen, run_id = [], None, frame["run_id"]
                row = frame["values"]
                if row["sample"] != seen:
                    records.append(row)
                    records = records[-5000:]
                    seen = row["sample"]
                    for line, key in traces:
                        line.set_data([r["time_minutes"] for r in records], [r[key] for r in records])
                    for ax in axes[:, 0]:
                        ax.relim()
                        ax.autoscale_view()
                    fig.canvas.draw_idle()
            fig.canvas.flush_events()
            await asyncio.sleep(max(.1, cfg["poll_seconds"]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("opc", type=Path)
    parser.add_argument("action", choices=("status", "stop", "trend"))
    parser.add_argument("simulation", type=Path, nargs="?")
    args = parser.parse_args()
    try:
        asyncio.run(run(args))
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f"OPC tool: {type(exc).__name__}: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
