"""Protocol checks and real two-process local OPC integration."""
import asyncio
from copy import deepcopy
from datetime import datetime, timedelta, timezone
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import unittest

from asyncua import Client, ua
from test_generic_workflow import filled_templates
from optam_mpc_linear import load_definition
from optam_mpc_linear.opc import (
    PROTOCOL, client_nodes, encode, fingerprint, load_opc, read_frame,
    validate_command, validate_sample,
)

ROOT = Path(__file__).resolve().parents[1]


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        p = Path(self.tmp.name) / "app.toml"
        p.write_text(filled_templates()[0])
        self.definition = load_definition(p)
        self.frame = dict(protocol=PROTOCOL, application_hash=fingerprint(self.definition),
                          run_id="run", sample=0, sample_time_seconds=5,
                          cvs={"inventory_a": 0, "inventory_b": 0},
                          dvs={"feed_a": 0, "feed_b": 0}, mvs={"drive_a": 0, "drive_b": 0})
        self.command = dict(run_id="run", sample=0, application_hash=fingerprint(self.definition),
                            controller_id="controller", status="ok",
                            solver_seconds=.01, mvs={"drive_a": .1, "drive_b": .1})

    def test_valid_sample_and_command(self):
        validate_sample(self.frame, self.definition, 0, "run")
        validate_command(self.command, self.frame, self.definition)

    def test_wrong_application_rejected(self):
        self.frame["application_hash"] = "different"
        with self.assertRaises(ValueError):
            validate_sample(self.frame, self.definition)

    def test_nonfinite_measurement_rejected(self):
        self.frame["cvs"]["inventory_a"] = float("nan")
        with self.assertRaises(ValueError):
            validate_sample(self.frame, self.definition)

    def test_missing_measurement_rejected(self):
        self.frame["cvs"].pop("inventory_a")
        with self.assertRaises(ValueError):
            validate_sample(self.frame, self.definition)

    def test_skipped_sample_rejected(self):
        with self.assertRaises(ValueError):
            validate_sample(self.frame, self.definition, 1)

    def test_restart_rejected(self):
        with self.assertRaises(ValueError):
            validate_sample(self.frame, self.definition, run_id="other")

    def test_stale_command_rejected(self):
        self.command["sample"] = -1
        with self.assertRaises(ValueError):
            validate_command(self.command, self.frame, self.definition)

    def test_second_controller_rejected(self):
        with self.assertRaises(ValueError):
            validate_command(self.command, self.frame, self.definition, owner="first")

    def test_large_move_rejected(self):
        self.command["mvs"]["drive_a"] = 2
        with self.assertRaises(ValueError):
            validate_command(self.command, self.frame, self.definition)

    def test_position_limit_rejected(self):
        self.frame["mvs"]["drive_a"] = 10
        self.command["mvs"]["drive_a"] = 10.5
        with self.assertRaises(ValueError):
            validate_command(self.command, self.frame, self.definition)

    def test_solver_fault_rejected(self):
        self.command["status"] = "fault"
        with self.assertRaises(ValueError):
            validate_command(self.command, self.frame, self.definition)

    def test_bad_opc_quality_and_timestamp_rejected(self):
        class Node:
            async def read_data_value(inner):
                return inner.data
        node = Node()
        node.data = ua.DataValue(ua.Variant(encode(self.frame)), ua.StatusCode(ua.StatusCodes.BadNoCommunication))
        with self.assertRaises(ua.UaStatusCodeError):
            asyncio.run(read_frame(node))
        node.data = ua.DataValue(ua.Variant(encode(self.frame)),
                                SourceTimestamp=datetime.now(timezone.utc)-timedelta(seconds=10))
        with self.assertRaises(ValueError):
            asyncio.run(read_frame(node, 1))

    def test_loopback_only(self):
        path = Path(self.tmp.name) / "opc.toml"
        path.write_text((ROOT / "opc_template.toml").read_text().replace("127.0.0.1", "0.0.0.0"))
        with self.assertRaises(ValueError):
            load_opc(path)


class OPCIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix="opc generic ")
        self.addCleanup(self.tmp.cleanup)
        self.folder = Path(self.tmp.name)
        app, sim = filled_templates()
        self.app, self.sim, self.opc = [self.folder / x for x in ("application.toml", "simulation.toml", "opc.toml")]
        self.app.write_text(app)
        self.sim.write_text(sim)
        with socket.socket() as sock:
            sock.bind(("127.0.0.1", 0))
            port = sock.getsockname()[1]
        self.opc.write_text((ROOT / "opc_template.toml").read_text()
                           .replace(":4840/", f":{port}/").replace("speed = 20.0", "speed = 1000.0")
                           .replace("startup_timeout_seconds = 120.0", "startup_timeout_seconds = 3.0")
                           .replace("command_timeout_seconds = 15.0", "command_timeout_seconds = 2.0")
                           .replace("stale_timeout_seconds = 30.0", "stale_timeout_seconds = 5.0"))
        self.cfg = load_opc(self.opc)
        self.processes = []
        self.logs = []
        self.addCleanup(self.clean_processes)

    def clean_processes(self):
        for p in self.processes:
            if p.poll() is None:
                p.terminate()
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait(timeout=5)
        for stream in self.logs:
            stream.close()

    def start(self, script, *args):
        log = (self.folder / (script + ".log")).open("w")
        self.logs.append(log)
        p = subprocess.Popen([sys.executable, str(ROOT / script), *map(str, args)],
                             cwd=self.folder, stdout=log, stderr=subprocess.STDOUT,
                             env=dict(os.environ, OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1"))
        self.processes.append(p)
        return p

    def ready(self):
        async def check():
            for _ in range(100):
                try:
                    async with Client(self.cfg["endpoint"], timeout=1) as client:
                        nodes = await client_nodes(client, self.cfg)
                        return await read_frame(nodes["sample_node"])
                except Exception:
                    await asyncio.sleep(.1)
            raise RuntimeError("Twin did not become ready.")
        return asyncio.run(check())

    def wait_report(self):
        path = self.folder / "opc_results" / "report.json"
        for _ in range(150):
            if path.exists():
                try:
                    return json.loads(path.read_text())
                except json.JSONDecodeError:
                    pass
            time.sleep(.1)
        logs = "".join(p.read_text() for p in self.folder.glob("*.log"))
        self.fail("No report: " + logs)

    def test_two_process_success_matches_offline(self):
        self.start("run_opc_twin.py", self.app, self.sim, self.opc)
        self.ready()
        controller = self.start("run_opc_controller.py", self.app, self.opc)
        self.assertEqual(controller.wait(timeout=20), 0)
        report = self.wait_report()
        self.assertTrue(report["passed"])
        self.assertEqual(report["samples"], 4)
        offline = self.start("run_simulation.py", self.app, self.sim)
        self.assertEqual(offline.wait(timeout=20), 0)
        self.assertEqual((self.folder/"results"/"results.csv").read_text(),
                         (self.folder/"opc_results"/"results.csv").read_text())

    def test_no_controller_times_out_without_advancing(self):
        self.start("run_opc_twin.py", self.app, self.sim, self.opc)
        self.ready()
        report = self.wait_report()
        self.assertEqual(report["terminal_state"], "fault")
        self.assertEqual(report["samples"], 0)
        self.assertFalse(report["passed"])

    def test_controller_disconnect_after_one_command_freezes(self):
        self.start("run_opc_twin.py", self.app, self.sim, self.opc)
        self.ready()
        async def one_command():
            async with Client(self.cfg["endpoint"], timeout=1) as client:
                nodes = await client_nodes(client, self.cfg)
                frame = await read_frame(nodes["sample_node"])
                command = dict(run_id=frame["run_id"], sample=0,
                               application_hash=frame["application_hash"], controller_id="test",
                               status="ok", solver_seconds=0, mvs=frame["mvs"])
                await nodes["command_node"].write_value(encode(command))
        asyncio.run(one_command())
        report = self.wait_report()
        self.assertEqual(report["samples"], 1)
        self.assertEqual(report["terminal_state"], "fault")

    def test_bad_command_is_not_applied(self):
        self.start("run_opc_twin.py", self.app, self.sim, self.opc)
        self.ready()
        async def bad_command():
            async with Client(self.cfg["endpoint"], timeout=1) as client:
                nodes = await client_nodes(client, self.cfg)
                await nodes["command_node"].write_value('{"run_id":"wrong","sample":0}')
        asyncio.run(bad_command())
        report = self.wait_report()
        self.assertFalse(report["passed"])
        self.assertEqual(report["samples"], 0)

    def test_operator_stop(self):
        self.start("run_opc_twin.py", self.app, self.sim, self.opc)
        self.ready()
        tool = self.start("opc_tools.py", self.opc, "stop")
        self.assertEqual(tool.wait(timeout=10), 0)
        report = self.wait_report()
        self.assertEqual(report["terminal_state"], "stopped")
        self.assertFalse(report["passed"])

    def test_wrong_application_client_rejected(self):
        self.start("run_opc_twin.py", self.app, self.sim, self.opc)
        self.ready()
        other = self.folder / "other.toml"
        other.write_text(self.app.read_text().replace("integrating_gain = -0.1",
                                                     "integrating_gain = -0.2"))
        controller = self.start("run_opc_controller.py", other, self.opc)
        self.assertNotEqual(controller.wait(timeout=10), 0)
        report = self.wait_report()
        self.assertFalse(report["passed"])
        self.assertEqual(report["samples"], 0)

    def test_hidden_input_visible_only_in_observer_frame(self):
        self.sim.write_text(self.sim.read_text() + """
[[plant_inputs]]
id = "secret_flow"
role = "hidden"
unit = "unit"
initial = 123.0
[[plant_paths]]
source = "secret_flow"
destination = "y1"
type = "integrator"
integrating_gain = 1.0
dead_time_seconds = 0.0
""")
        self.start("run_opc_twin.py", self.app, self.sim, self.opc)
        self.ready()
        async def inspect():
            async with Client(self.cfg["endpoint"], timeout=1) as client:
                nodes = await client_nodes(client, self.cfg)
                sample = await read_frame(nodes["sample_node"])
                telemetry = await read_frame(nodes["telemetry_node"])
                self.assertNotIn("secret_flow", json.dumps(sample))
                self.assertEqual(telemetry["values"]["secret_flow"], 123)
                await nodes["stop_node"].write_value(True)
        asyncio.run(inspect())

    def test_report_belongs_to_current_run(self):
        self.start("run_opc_twin.py", self.app, self.sim, self.opc)
        self.ready()
        report = self.wait_report()
        marker = json.loads((self.folder / "opc_results" / "current_run.json").read_text())
        self.assertEqual(marker["run_id"], report["run_id"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
