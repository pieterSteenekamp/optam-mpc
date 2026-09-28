"""Full real two-process OPC/offline comparison, in an isolated copied runtime.

Run from package root: python tests/verify_full_opc.py
Writes verified_opc reference results only after complete equality and PASS.
"""
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import tempfile
import time

ROOT=Path(__file__).resolve().parents[1]


def main():
    reference=ROOT/"examples"/"01_heat_exchanger"
    with tempfile.TemporaryDirectory(prefix="optam nonlinear verification ") as temp:
        folder=Path(temp)
        runtime=folder/"runtime";runtime.mkdir()
        for name in ("optam_mpc","optam_mpc_linear"):
            shutil.copytree(ROOT/name,runtime/name,ignore=shutil.ignore_patterns("__pycache__"))
        for name in ("nonlinear_runtime.py","run_simulation.py","run_opc_twin.py","run_opc_controller.py","opc_tools.py"):
            shutil.copy2(ROOT/name,runtime/name)
        for name in ("application.toml","model.py","simulation.toml"):
            shutil.copy2(reference/name,folder/name)
        assert not (runtime/"examples").exists()
        with socket.socket() as sock:
            sock.bind(("127.0.0.1",0));port=sock.getsockname()[1]
        opc=(reference/"opc.toml").read_text().replace(":4842/",f":{port}/").replace("speed = 20.0","speed = 1000.0")
        (folder/"opc.toml").write_text(opc)
        env=dict(os.environ,OPENBLAS_NUM_THREADS="1",OMP_NUM_THREADS="1")
        handles=[];processes=[]
        def start(script,*args):
            handle=(folder/(script+".log")).open("w");handles.append(handle)
            proc=subprocess.Popen([sys.executable,"-u",str(runtime/script),*[str(folder/a) for a in args]],
                                  cwd=runtime,env=env,stdout=handle,stderr=subprocess.STDOUT)
            processes.append(proc);return proc
        try:
            twin=start("run_opc_twin.py","application.toml","simulation.toml","opc.toml")
            deadline=time.monotonic()+30
            while "Twin READY" not in (folder/"run_opc_twin.py.log").read_text():
                if twin.poll() is not None or time.monotonic()>deadline:
                    raise RuntimeError((folder/"run_opc_twin.py.log").read_text())
                time.sleep(.1)
            controller=start("run_opc_controller.py","application.toml","opc.toml")
            if controller.wait(timeout=600):
                raise RuntimeError((folder/"run_opc_controller.py.log").read_text()+
                                   (folder/"run_opc_twin.py.log").read_text())
            print("Full nonlinear OPC run: PASS",flush=True)
            offline=start("run_simulation.py","application.toml","simulation.toml")
            if offline.wait(timeout=600): raise RuntimeError((folder/"run_simulation.py.log").read_text())
            report=json.loads((folder/"opc_results"/"report.json").read_text())
            assert report["passed"] and report["samples"]==660,report
            assert (folder/"results"/"results.csv").read_bytes()==(folder/"opc_results"/"results.csv").read_bytes()
            output=reference/"verified_opc";output.mkdir(exist_ok=True)
            for name in ("report.json","results.csv","results.png"):
                shutil.copy2(folder/"opc_results"/name,output/name)
            print("All 661 OPC/offline rows exactly equal: PASS",flush=True)
        finally:
            for proc in processes:
                if proc.poll() is None: proc.terminate()
            for proc in processes:
                try: proc.wait(timeout=10)
                except subprocess.TimeoutExpired: proc.kill();proc.wait()
            for handle in handles: handle.close()


if __name__=="__main__": main()
