"""Real local OPC startup faults must not advance the nonlinear plant."""
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import tempfile
import time
import unittest

ROOT=Path(__file__).resolve().parents[1]
EXAMPLE=ROOT/"examples"/"01_heat_exchanger"


class OPCFaultTests(unittest.TestCase):
    def fault_case(self, wrong_model):
        with tempfile.TemporaryDirectory(prefix="nonlinear fault ") as temp:
            folder=Path(temp)
            for name in ("application.toml","simulation.toml","model.py"):
                shutil.copy2(EXAMPLE/name,folder/name)
            with socket.socket() as sock:
                sock.bind(("127.0.0.1",0));port=sock.getsockname()[1]
            cfg=(EXAMPLE/"opc.toml").read_text().replace(":4842/",f":{port}/")
            cfg=cfg.replace("startup_timeout_seconds = 120.0","startup_timeout_seconds = 3.0")
            (folder/"opc.toml").write_text(cfg)
            handles=[];processes=[]
            def start(script,*args):
                f=(folder/(script+".log")).open("w");handles.append(f)
                p=subprocess.Popen([sys.executable,"-u",str(ROOT/script),*map(str,args)],cwd=ROOT,
                    env=dict(os.environ,OPENBLAS_NUM_THREADS="1",OMP_NUM_THREADS="1"),stdout=f,stderr=subprocess.STDOUT)
                processes.append(p);return p
            try:
                twin=start("run_opc_twin.py",folder/"application.toml",folder/"simulation.toml",folder/"opc.toml")
                until=time.monotonic()+20
                while "Twin READY" not in (folder/"run_opc_twin.py.log").read_text():
                    self.assertIsNone(twin.poll(),(folder/"run_opc_twin.py.log").read_text())
                    self.assertLess(time.monotonic(),until)
                    time.sleep(.1)
                if wrong_model:
                    client_folder=folder/"client";client_folder.mkdir()
                    shutil.copy2(folder/"application.toml",client_folder/"application.toml")
                    (client_folder/"model.py").write_text((folder/"model.py").read_text()+"\n# deliberately different version\n")
                    client=start("run_opc_controller.py",client_folder/"application.toml",folder/"opc.toml")
                    self.assertNotEqual(client.wait(timeout=20),0)
                    self.assertIn("application",(folder/"run_opc_controller.py.log").read_text().lower())
                report_file=folder/"opc_results"/"report.json"
                until=time.monotonic()+20
                while not report_file.exists():
                    self.assertLess(time.monotonic(),until)
                    time.sleep(.1)
                # Writer creates a small JSON file in one write; retry briefly if still open.
                for attempt in range(20):
                    try: report=json.loads(report_file.read_text());break
                    except json.JSONDecodeError: time.sleep(.05)
                else: self.fail("Incomplete report")
                self.assertEqual(report["samples"],0)
                self.assertEqual(report["terminal_state"],"fault")
                self.assertFalse(report["passed"])
            finally:
                for p in processes:
                    if p.poll() is None:p.terminate()
                for p in processes:
                    try:p.wait(timeout=10)
                    except subprocess.TimeoutExpired:p.kill();p.wait()
                for f in handles:f.close()

    def test_missing_controller_freezes_plant(self):self.fault_case(False)
    def test_wrong_model_version_rejected_without_advancement(self):self.fault_case(True)


if __name__=="__main__":unittest.main()
