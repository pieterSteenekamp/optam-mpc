#!/usr/bin/env python3
"""
APC for lime kiln – OPC UA client, calcination target control with sintering
& temperature constraints.  PI integral is synchronised from the real plant
via back‑calculation, eliminating model mismatch.
"""
import asyncio
import logging
import numpy as np
from asyncua import Client, ua
from kiln_models import LimeKilnPlant, PI, kiln_mpc_step, nominal_feed, dt_ctrl

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("KilnAPC")

OPC_URL = "opc.tcp://localhost:4841/freeopcua/server/"

class KilnAPC:
    def __init__(self):
        self.client = Client(OPC_URL)
        self.nodes = {}
        self.plant_model = LimeKilnPlant()
        self.pi_model = PI(0.01, 200.0, dt_ctrl, 0.0, 1.0)
        self.T_current = 1200.0
        self.u_prev = 1200.0
        self.connected = False

    async def connect(self):
        try:
            if self.connected: await self.client.disconnect()
            await self.client.connect()
            idx = await self.client.get_namespace_index("http://lime-kiln.simulator")
            kiln_node = await self.client.nodes.objects.get_child([f"{idx}:Kiln"])

            self.nodes["feed"]      = await kiln_node.get_child(f"{idx}:FeedRate")
            self.nodes["temp"]      = await kiln_node.get_child(f"{idx}:Temperature")
            self.nodes["t_wall"]    = await kiln_node.get_child(f"{idx}:T_Wall")
            self.nodes["xc"]        = await kiln_node.get_child(f"{idx}:Calcination")
            self.nodes["xs"]        = await kiln_node.get_child(f"{idx}:Sintering")
            self.nodes["firing"]    = await kiln_node.get_child(f"{idx}:FiringRate")   # NEW
            self.nodes["tsp"]       = await kiln_node.get_child(f"{idx}:T_SP")
            self.nodes["apc_en"]    = await kiln_node.get_child(f"{idx}:APC_Enable")
            self.nodes["new_sp"]    = await kiln_node.get_child(f"{idx}:APC_NewSP")
            self.nodes["applied"]   = await kiln_node.get_child(f"{idx}:APC_SP_Applied")
            self.nodes["pi_kc"]     = await kiln_node.get_child(f"{idx}:PI_Kc")
            self.nodes["pi_ti"]     = await kiln_node.get_child(f"{idx}:PI_Ti")

            self.nodes["mpc_xc_sp"]   = await kiln_node.get_child(f"{idx}:MPC_XcSP")
            self.nodes["mpc_xs_max"]  = await kiln_node.get_child(f"{idx}:MPC_XsMax")
            self.nodes["mpc_temp_max"]= await kiln_node.get_child(f"{idx}:MPC_TempMax")
            self.nodes["mpc_xc_w"]    = await kiln_node.get_child(f"{idx}:MPC_XcWeight")
            self.nodes["mpc_move_w"]  = await kiln_node.get_child(f"{idx}:MPC_MoveWeight")
            self.nodes["mpc_horizon"] = await kiln_node.get_child(f"{idx}:MPC_Horizon")

            self.connected = True
            logger.info("✅ Connected to kiln OPC server.")
        except Exception as e:
            self.connected = False
            logger.error(f"Connection failed: {e}")

    async def disconnect(self):
        if self.connected:
            await self.client.disconnect()
            self.connected = False

    async def run(self):
        base_sleep = 0.1
        poll_sleep = 0.05

        # Bulk read list – order must match unpacking below
        read_list = [self.nodes["feed"], self.nodes["temp"], self.nodes["t_wall"],
                     self.nodes["xc"], self.nodes["xs"], self.nodes["firing"],
                     self.nodes["tsp"],
                     self.nodes["pi_kc"], self.nodes["pi_ti"],
                     self.nodes["mpc_xc_sp"], self.nodes["mpc_xs_max"],
                     self.nodes["mpc_temp_max"], self.nodes["mpc_xc_w"],
                     self.nodes["mpc_move_w"], self.nodes["mpc_horizon"]]

        while True:
            if not self.connected:
                await self.connect()
                if not self.connected:
                    await asyncio.sleep(2)
                    continue
            try:
                enabled = await asyncio.wait_for(self.nodes["apc_en"].read_value(), timeout=2.0)
                if enabled:
                    applied = False
                    while not applied:
                        applied = await asyncio.wait_for(self.nodes["applied"].read_value(), timeout=2.0)
                        if not applied: await asyncio.sleep(poll_sleep)

                    vals = await asyncio.wait_for(self.client.read_values(read_list), timeout=5.0)
                    (F, T, T_wall, Xc_real, Xs_real, u_fire, tsp_prev,
                     kc, ti, xc_sp, xs_max, T_max, xc_w, move_w, horizon) = vals
                    horizon = int(horizon)

                    # --- Synchronise PI gains and INTEGRAL from real plant ---
                    self.pi_model.Kc = kc
                    self.pi_model.Ti = ti
                    # Back‑calculate real integral: I = u_fire - Kc*(T_sp - T)
                    I_real = u_fire - kc * (tsp_prev - T)
                    self.pi_model.integral = np.clip(I_real, 0.0, 1.0)   # PI output limits

                    # Synchronise plant model state
                    self.plant_model.set_feed(F)
                    self.plant_model.T_solid = T
                    self.plant_model.T_wall = T_wall
                    self.plant_model.Xc = Xc_real
                    self.plant_model.Xs = Xs_real

                    self.T_current = T
                    self.u_prev = tsp_prev

                    T_sp_opt = kiln_mpc_step(
                        self.plant_model, self.pi_model,
                        T, tsp_prev, F,
                        Np=horizon, move_w=move_w,
                        xc_setpoint=xc_sp, xc_weight=xc_w,
                        xs_max=xs_max, temp_max=T_max
                    )
                    T_sp_opt = np.clip(T_sp_opt, 900.0, 1500.0)

                    if abs(T_sp_opt - tsp_prev) < 0.5:
                        T_sp_opt = tsp_prev

                    await self.nodes["tsp"].write_value(float(T_sp_opt))
                    await self.nodes["applied"].write_value(False)
                    await self.nodes["new_sp"].write_value(True)

                    logger.info(f"F={F:.2f} T={T:.0f} Xc={Xc_real:.3f} Xs={Xs_real:.3f} "
                                f"u_fire={u_fire:.3f} I≈{I_real:.3f} → Tsp={T_sp_opt:.1f}")

                else:
                    await self.nodes["new_sp"].write_value(False)
                    await self.nodes["applied"].write_value(False)

            except (asyncio.TimeoutError, ua.UaError, ConnectionError) as e:
                logger.error(f"OPC error: {e}. Disconnecting.")
                self.connected = False
            except Exception as e:
                logger.error(f"Unexpected error: {e}")

            await asyncio.sleep(base_sleep)


async def main():
    apc = KilnAPC()
    await apc.connect()
    try:
        await apc.run()
    finally:
        await apc.disconnect()

if __name__ == "__main__":
    asyncio.run(main())