#!/usr/bin/env python3
"""
APC Controller – OPC UA client with dynamic MPC tuning from OPC.

MODIFIED:
- Uses bulk read (read_values) to minimise OPC calls.
- All OPC operations have a timeout (2–5 s).
- Base loop sleep increased to 0.1 s; handshake polling sleep to 0.05 s.
- More robust error handling.
"""
import asyncio
import logging
import numpy as np
from asyncua import Client, ua

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("APC")

from models import FourTankPlant, simulate_interval, generic_mpc_step

OPC_URL = "opc.tcp://localhost:4840/freeopcua/server/"

class APCController:
    def __init__(self):
        self.client = Client(OPC_URL)
        self.nodes = {}
        self.plant_model = FourTankPlant()
        self.X_current = None
        self.u_prev = np.array([2.0, 1.8, 1.8, 2.0, 0.2])
        self.connected = False

    async def connect(self):
        try:
            if self.connected: await self.client.disconnect()
            await self.client.connect()
            idx = await self.client.get_namespace_index("http://four-tank.simulator")
            plant_node = await self.client.nodes.objects.get_child([f"{idx}:Plant"])

            self.nodes["levels"]      = [await plant_node.get_child(f"{idx}:Tank{i}_Level") for i in range(1,5)]
            self.nodes["flows"]       = [await plant_node.get_child(f"{idx}:F{i}") for i in ["1","2","3","4","m"]]
            self.nodes["enable"]      = await plant_node.get_child(f"{idx}:APC_Enable")
            self.nodes["apc_sps"]     = [await plant_node.get_child(f"{idx}:APC_FlowSP_F{i}") for i in range(1,6)]
            self.nodes["max_f2"]      = await plant_node.get_child(f"{idx}:Max_F2")
            self.nodes["leak"]        = await plant_node.get_child(f"{idx}:Leak")
            self.nodes["apc_new_sp"]  = await plant_node.get_child(f"{idx}:APC_NewSP")
            self.nodes["apc_applied"] = await plant_node.get_child(f"{idx}:APC_SP_Applied")
            self.nodes["econ_w"]      = await plant_node.get_child(f"{idx}:MPC_EconWeight")
            self.nodes["level_w"]     = await plant_node.get_child(f"{idx}:MPC_LevelWeight")
            self.nodes["move_w"]      = await plant_node.get_child(f"{idx}:MPC_MoveWeight")
            self.nodes["horizon"]     = await plant_node.get_child(f"{idx}:MPC_Horizon")

            self.connected = True
            logger.info("✅ Connected to OPC server.")
        except Exception as e:
            self.connected = False
            logger.error(f"Connection failed: {e}")

    async def disconnect(self):
        if self.connected: await self.client.disconnect(); self.connected = False

    async def run(self):
        base_sleep = 0.1        # main loop delay when APC disabled
        poll_sleep = 0.05       # delay while waiting for handshake signals

        # Pre‑build the list of nodes for bulk reading (order must be preserved)
        read_node_list = (self.nodes["levels"] +
                          self.nodes["flows"] +
                          [self.nodes["max_f2"], self.nodes["leak"],
                           self.nodes["econ_w"], self.nodes["level_w"],
                           self.nodes["move_w"], self.nodes["horizon"]])

        while True:
            if not self.connected:
                await self.connect()
                if not self.connected:
                    await asyncio.sleep(2)
                    continue
            try:
                # Read enable flag with timeout
                enabled = await asyncio.wait_for(
                    self.nodes["enable"].read_value(), timeout=2.0
                )

                if enabled:
                    # Wait for simulator to be ready (apc_applied == True)
                    applied = False
                    while not applied:
                        applied = await asyncio.wait_for(
                            self.nodes["apc_applied"].read_value(), timeout=2.0
                        )
                        if not applied:
                            await asyncio.sleep(poll_sleep)

                    # Bulk read all necessary process & tuning values
                    values = await asyncio.wait_for(
                        self.client.read_values(read_node_list), timeout=5.0
                    )

                    L = np.array(values[0:4])
                    flows = np.array(values[4:9])
                    max_f2 = values[9]
                    leak = values[10]
                    econ_w = values[11]
                    level_w = values[12]
                    move_w = values[13]
                    horizon = int(values[14])

                    # Update internal model state
                    if self.X_current is None:
                        self.u_prev = np.clip(flows, 0, [5.0, max_f2, 5.0, 5.5, 5.0])
                        self.X_current = np.hstack([L, flows])
                    else:
                        self.X_current[:4] = L
                        self.X_current[4:9] = flows

                    self.plant_model.max_flows[2] = max_f2
                    self.plant_model.leak = leak

                    u_opt = generic_mpc_step(
                        self.plant_model, L, self.X_current, self.u_prev,
                        Np=horizon, econ_w=econ_w,
                        level_w=level_w, move_w=move_w
                    )
                    u_opt = np.clip(u_opt, 0, [5.0, max_f2, 5.0, 5.5, 5.0])

                    # Deadband
                    if np.max(np.abs(u_opt - self.u_prev)) < 0.02:
                        u_opt = self.u_prev

                    print(f"L={np.round(L,3)}  u={np.round(u_opt,3)}  du={np.round(u_opt-self.u_prev,3)}")

                    # Write new setpoints to OPC
                    for i, node in enumerate(self.nodes["apc_sps"]):
                        await asyncio.wait_for(
                            node.write_value(float(u_opt[i])), timeout=2.0
                        )

                    # Handshake: tell simulator new SPs are ready
                    await asyncio.wait_for(
                        self.nodes["apc_applied"].write_value(False), timeout=2.0
                    )
                    await asyncio.wait_for(
                        self.nodes["apc_new_sp"].write_value(True), timeout=2.0
                    )

                    # Advance internal prediction model
                    self.X_current = simulate_interval(
                        self.plant_model, self.X_current, u_opt, leak
                    )
                    self.u_prev = u_opt

                else:
                    # APC disabled: clear handshake and reset internal state
                    self.X_current = None
                    await asyncio.wait_for(
                        self.nodes["apc_new_sp"].write_value(False), timeout=2.0
                    )
                    await asyncio.wait_for(
                        self.nodes["apc_applied"].write_value(False), timeout=2.0
                    )

            except (asyncio.TimeoutError, ua.UaError, ConnectionError) as e:
                logger.error(f"OPC error: {e}. Disconnecting.")
                self.connected = False
            except Exception as e:
                logger.error(f"Unexpected error: {e}")

            await asyncio.sleep(base_sleep)

async def main():
    apc = APCController()
    await apc.connect()
    try:
        await apc.run()
    finally:
        await apc.disconnect()

if __name__ == "__main__":
    asyncio.run(main())