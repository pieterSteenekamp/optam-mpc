#!/usr/bin/env python3
"""
Lime kiln digital twin – OPC UA server, Pygame GUI with live kiln cartoon,
CSV logger, tuning panel (calcination setpoint, sintering & temp constraints).
Starts from steady state with very weak PI tuning.
"""
import asyncio
import math
import random
import time
import numpy as np
import pygame
import csv
from asyncua import Server, ua
from kiln_models import (LimeKilnPlant, PI, nominal_feed,
                         dt_ctrl, max_firing_power)

# ---------- OPC server ----------
async def create_opc_server():
    server = Server()
    await server.init()
    server.set_endpoint("opc.tcp://0.0.0.0:4841/freeopcua/server/")
    uri = "http://lime-kiln.simulator"
    idx = await server.register_namespace(uri)
    kiln_node = await server.nodes.objects.add_object(idx, "Kiln")

    feed_node   = await kiln_node.add_variable(idx, "FeedRate", nominal_feed); await feed_node.set_writable()
    temp_node   = await kiln_node.add_variable(idx, "Temperature", 1200.0); await temp_node.set_writable()
    fire_node   = await kiln_node.add_variable(idx, "FiringRate", 0.5); await fire_node.set_writable()
    xc_node     = await kiln_node.add_variable(idx, "Calcination", 0.0); await xc_node.set_writable()
    xs_node     = await kiln_node.add_variable(idx, "Sintering", 0.0); await xs_node.set_writable()
    qual_node   = await kiln_node.add_variable(idx, "Quality", 0.0); await qual_node.set_writable()
    tsp_node    = await kiln_node.add_variable(idx, "T_SP", 1200.0); await tsp_node.set_writable()
    twall_node  = await kiln_node.add_variable(idx, "T_Wall", 1200.0); await twall_node.set_writable()

    apc_enable  = await kiln_node.add_variable(idx, "APC_Enable", False); await apc_enable.set_writable()
    apc_new_sp  = await kiln_node.add_variable(idx, "APC_NewSP", False); await apc_new_sp.set_writable()
    apc_applied = await kiln_node.add_variable(idx, "APC_SP_Applied", False); await apc_applied.set_writable()
    manual_tsp  = await kiln_node.add_variable(idx, "Manual_T_SP", 1200.0); await manual_tsp.set_writable()

    pi_kc       = await kiln_node.add_variable(idx, "PI_Kc", 0.01); await pi_kc.set_writable()
    pi_ti       = await kiln_node.add_variable(idx, "PI_Ti", 200.0); await pi_ti.set_writable()

    mpc_xc_sp   = await kiln_node.add_variable(idx, "MPC_XcSP", 0.92); await mpc_xc_sp.set_writable()
    mpc_xs_max  = await kiln_node.add_variable(idx, "MPC_XsMax", 0.15); await mpc_xs_max.set_writable()
    mpc_temp_max= await kiln_node.add_variable(idx, "MPC_TempMax", 1450.0); await mpc_temp_max.set_writable()
    mpc_xc_w    = await kiln_node.add_variable(idx, "MPC_XcWeight", 100.0); await mpc_xc_w.set_writable()
    mpc_move_w  = await kiln_node.add_variable(idx, "MPC_MoveWeight", 0.5); await mpc_move_w.set_writable()
    mpc_horizon = await kiln_node.add_variable(idx, "MPC_Horizon", 6); await mpc_horizon.set_writable()

    sq_enable   = await kiln_node.add_variable(idx, "SquareWave_Enable", False); await sq_enable.set_writable()
    sq_low      = await kiln_node.add_variable(idx, "SquareWave_Low", 7.0); await sq_low.set_writable()
    sq_high     = await kiln_node.add_variable(idx, "SquareWave_High", 9.0); await sq_high.set_writable()
    sq_period   = await kiln_node.add_variable(idx, "SquareWave_Period", 30); await sq_period.set_writable()

    await server.start()
    return server, (feed_node, temp_node, fire_node, xc_node, xs_node, qual_node,
                    tsp_node, twall_node, apc_enable, apc_new_sp, apc_applied, manual_tsp,
                    pi_kc, pi_ti,
                    mpc_xc_sp, mpc_xs_max, mpc_temp_max, mpc_xc_w, mpc_move_w, mpc_horizon,
                    sq_enable, sq_low, sq_high, sq_period)


# ---------- Pygame GUI (unchanged cartoon drawing, updated tuning) ----------
class KilnGUI:
    def __init__(self, plant, pi_temp):
        pygame.init()
        self.width, self.height = 1000, 600
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("Lime Kiln Digital Twin – Calcination MPC")
        self.font = pygame.font.SysFont("Arial", 16)
        self.big_font = pygame.font.SysFont("Arial", 24)
        self.plant = plant
        self.pi_temp = pi_temp

        self.speed = 1
        self.apc_enabled = False
        self.manual_override = False
        self.square_wave = False
        self.feed_low = 7.0
        self.feed_high = 9.0
        self.feed_period = 30
        self.feed_counter = 0
        self.feed_state = False
        self.current_feed = nominal_feed
        self.tsp_manual = 1200.0

        self.tuning_mode = False
        self.tuning_selection = 0
        # (name, init_val, step, min, max, tag)

        self.tuning_params = [
            ("PI Kc",             0.01, 0.005, 0.001, 2.0,    "pi_kc"),
            ("PI Ti",           200.0,  10.0,   5.0,  1000.0, "pi_ti"),
            ("Xc Setpoint",      0.92,  0.01,  0.8,   1.0,    "mpc_xc_sp"),
            ("Xs Max",           0.15,  0.01,  0.05,  0.5,    "mpc_xs_max"),
            ("T Max [K]",      1450.0,  10.0, 1200.0, 1600.0, "mpc_temp_max"),
            ("Xc Weight",       200.0,  10.0,   0.0, 1000.0,  "mpc_xc_w"),   # increased
            ("Move Weight",      0.002,  0.001,   0.0,   0.5,   "mpc_move_w"), # strongly reduced
            ("Horizon",             6,     1,     1,    10,    "mpc_horizon"),
        ]

#        self.tuning_params = [
#            ("PI Kc",             0.01, 0.005, 0.001, 2.0,    "pi_kc"),
#            ("PI Ti",           200.0,  10.0,   5.0,  1000.0, "pi_ti"),
#            ("Xc Setpoint",      0.92,  0.01,  0.8,   1.0,    "mpc_xc_sp"),
#            ("Xs Max",           0.15,  0.01,  0.05,  0.5,    "mpc_xs_max"),
#            ("T Max [K]",      1450.0,  10.0, 1200.0, 1600.0, "mpc_temp_max"),
#            ("Xc Weight",       200.0,  10.0,   0.0, 1000.0,  "mpc_xc_w"),   # increased
#            ("Move Weight",      0.02,  0.01,   0.0,   5.0,   "mpc_move_w"), # strongly reduced
#            ("Horizon",             6,     1,     1,    10,    "mpc_horizon"),
#        ]


        self.tuning_values = [p[1] for p in self.tuning_params]

        # --- kiln cartoon state (unchanged) ---
        self.flow_particles = []
        self._particle_spawn_acc = 0.0
        self._last_tick_ms = pygame.time.get_ticks()

    # ---------- cartoon drawing (exactly as in your original) ----------
    TEMP_COLOR_STOPS = [
        (900.0,  (55, 15, 15)),
        (1050.0, (150, 30, 15)),
        (1200.0, (225, 95, 15)),
        (1350.0, (255, 170, 45)),
        (1500.0, (255, 235, 160)),
    ]

    @staticmethod
    def _lerp(a, b, t):
        return a + (b - a) * t

    @staticmethod
    def _lerp_color(c1, c2, t):
        return tuple(int(KilnGUI._lerp(c1[i], c2[i], t)) for i in range(3))

    def temp_to_color(self, T):
        stops = self.TEMP_COLOR_STOPS
        if T <= stops[0][0]: return stops[0][1]
        if T >= stops[-1][0]: return stops[-1][1]
        for (t0, c0), (t1, c1) in zip(stops, stops[1:]):
            if t0 <= T <= t1:
                return self._lerp_color(c0, c1, (T - t0) / (t1 - t0))
        return stops[-1][1]

    def quality_to_color(self, Q, target, tol_good=0.02, tol_ok=0.06):
        err = abs(Q - target)
        if err <= tol_good: return (40, 170, 65)
        if err <= tol_ok:
            return self._lerp_color((235, 200, 40), (210, 55, 40), (err - tol_good) / (tol_ok - tol_good))
        return (200, 40, 40)

    def flame_colors(self, u_fire):
        u = max(0.0, min(1.0, u_fire))
        outer = self._lerp_color((100, 25, 10), (255, 150, 30), u)
        inner = self._lerp_color((160, 60, 10), (255, 250, 205), u)
        return outer, inner

    def update_animation(self):
        now = pygame.time.get_ticks()
        dt = min((now - self._last_tick_ms) / 1000.0, 0.25)
        self._last_tick_ms = now

        tau_min = max(self.plant.get_residence_time(), 60.0) / 60.0
        real_seconds_per_pass = max(tau_min / max(self.speed, 1), 0.5)
        progress_rate = 1.0 / real_seconds_per_pass

        for part in self.flow_particles:
            part['progress'] += progress_rate * dt
        self.flow_particles = [pt for pt in self.flow_particles if pt['progress'] < 1.0]

        feed_ratio = self.current_feed / max(nominal_feed, 0.1)
        self._particle_spawn_acc += dt * (0.4 + 0.5 * feed_ratio)
        while self._particle_spawn_acc >= 1.0:
            self._particle_spawn_acc -= 1.0
            self.flow_particles.append({'progress': 0.0, 'phase': random.uniform(0, 6.28)})
        if len(self.flow_particles) > 14:
            self.flow_particles = self.flow_particles[-14:]

    def draw_kiln_cartoon(self, surface):
        p = self.plant
        panel = pygame.Rect(280, 250, 450, 210)
        pygame.draw.rect(surface, (222, 227, 236), panel, border_radius=12)
        pygame.draw.rect(surface, (170, 170, 185), panel, width=2, border_radius=12)
        surface.blit(self.big_font.render("Live kiln view", True, (50, 70, 110)),
                    (panel.x + 14, panel.y + 8))

        drum_rect = pygame.Rect(330, 335, 230, 70)
        drum_color = self.temp_to_color(p.T_solid)
        pygame.draw.rect(surface, drum_color, drum_rect, border_radius=30)
        pygame.draw.rect(surface, (50, 50, 55), drum_rect, width=3, border_radius=30)
        hi_rect = pygame.Rect(drum_rect.x + 14, drum_rect.y + 8, drum_rect.w - 28, 14)
        pygame.draw.ellipse(surface, self._lerp_color(drum_color, (255,255,255), 0.35), hi_rect)
        surface.blit(self.font.render(f"Kiln solids: {p.T_solid:.0f} K", True, (40,40,45)),
                    (drum_rect.x, drum_rect.y - 20))
        surface.blit(self.font.render(f"Feed: {p.F:.1f} kg/s", True, (40,40,45)),
                    (drum_rect.x - 6, drum_rect.bottom + 8))

        for part in self.flow_particles:
            x = drum_rect.x + part['progress'] * drum_rect.w
            y = drum_rect.centery + 14 * math.sin(part['progress'] * 6.0 + part['phase'])
            col = self._lerp_color((170,150,120), (255,240,210), part['progress'])
            pygame.draw.circle(surface, col, (int(x), int(y)), 6)
            pygame.draw.circle(surface, (60,50,40), (int(x), int(y)), 6, width=1)

        base_x = drum_rect.right
        mid_y = drum_rect.centery
        flicker = random.uniform(-6, 6)
        flame_h = 18 + 55 * max(0.0, min(1.0, p.u_fire)) + flicker
        outer_col, inner_col = self.flame_colors(p.u_fire)
        outer_pts = [
            (base_x, mid_y),
            (base_x + 50, int(mid_y - flame_h / 2)),
            (base_x + 50, int(mid_y + flame_h / 2)),
        ]
        inner_pts = [
            (base_x + 8, mid_y),
            (base_x + 32, int(mid_y - flame_h / 3)),
            (base_x + 32, int(mid_y + flame_h / 3)),
        ]
        pygame.draw.polygon(surface, outer_col, outer_pts)
        pygame.draw.polygon(surface, inner_col, inner_pts)
        surface.blit(self.font.render(f"Firing: {p.u_fire*100:.0f}%", True, (40,40,45)),
                    (base_x, mid_y - 60))

        prod_x, prod_y = base_x + 95, mid_y
        q_col = self.quality_to_color(p.Q, self.tuning_values[2])  # uses Xc sp for color ref
        pygame.draw.circle(surface, q_col, (int(prod_x), int(prod_y)), 22)
        pygame.draw.circle(surface, (50,50,55), (int(prod_x), int(prod_y)), 22, width=2)
        surface.blit(self.font.render("Product", True, (40,40,45)), (prod_x-26, prod_y+28))
        surface.blit(self.font.render(f"Q {p.Q:.2f}", True, (40,40,45)), (prod_x-20, prod_y+46))

    def draw(self):
        self.update_animation()
        self.screen.fill((240, 240, 255))
        p = self.plant
        vals = [
            f"Feed rate: {p.F:.2f} kg/s",
            f"Temperature: {p.T_solid:.1f} K",
            f"Firing rate: {p.u_fire:.3f}",
            f"Calcination Xc: {p.Xc:.3f}",
            f"Sintering Xs: {p.Xs:.3f}",
            f"Quality Q: {p.Q:.3f}",
        ]
        mode = "APC" if self.apc_enabled else "PI base"
        if self.manual_override: mode = "MANUAL"
        self.screen.blit(self.big_font.render(f"Mode: {mode}", True, (200,0,0)), (30, 10))
        y = 50
        for v in vals:
            img = self.font.render(v, True, (0,0,0))
            self.screen.blit(img, (30, y))
            y += 22

        self.screen.blit(self.font.render(f"Speed: {self.speed}x (↑↓)", True, (0,0,0)), (30, y+10))
        sq_str = "ON" if self.square_wave else "OFF"
        self.screen.blit(self.font.render(f"Square wave: {sq_str} (W)", True, (0,0,0)), (30, y+30))
        self.screen.blit(self.font.render("A: toggle APC   SPACE: manual   T: tuning", True, (0,0,200)), (30, y+55))

        self.draw_kiln_cartoon(self.screen)

        if self.tuning_mode:
            panel_x = 750
            pygame.draw.rect(self.screen, (220,220,200), (panel_x-10, 50, 230, 350))
            self.screen.blit(self.big_font.render("TUNING PANEL", True, (255,0,0)), (panel_x, 60))
            yy = 100
            for i, (name, val, step, lo, hi, tag) in enumerate(self.tuning_params):
                color = (0,0,0) if i != self.tuning_selection else (255,0,0)
                if "Horizon" in name:
                    txt = f"{name}: {int(self.tuning_values[i])}"
                else:
                    txt = f"{name}: {self.tuning_values[i]:.3f}"
                self.screen.blit(self.font.render(txt, True, color), (panel_x, yy))
                yy += 28
            self.screen.blit(self.font.render("↑↓ select   ←→ adjust", True, (0,0,150)), (panel_x, yy+10))

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT: return False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE: return False
                if event.key == pygame.K_t:
                    self.tuning_mode = not self.tuning_mode

                if self.tuning_mode:
                    if event.key == pygame.K_UP:
                        self.tuning_selection = (self.tuning_selection - 1) % len(self.tuning_params)
                    elif event.key == pygame.K_DOWN:
                        self.tuning_selection = (self.tuning_selection + 1) % len(self.tuning_params)
                    elif event.key == pygame.K_LEFT:
                        self._adjust_tuning(-1)
                    elif event.key == pygame.K_RIGHT:
                        self._adjust_tuning(1)
                else:
                    if event.key == pygame.K_UP: self.speed = min(300, self.speed+1)
                    elif event.key == pygame.K_DOWN: self.speed = max(1, self.speed-1)
                    elif event.key == pygame.K_a: self.apc_enabled = not self.apc_enabled
                    elif event.key == pygame.K_SPACE: self.manual_override = True
                    elif event.key == pygame.K_w: self.square_wave = not self.square_wave
                    elif event.key == pygame.K_m:
                        if self.manual_override: self.tsp_manual += 10
                    elif event.key == pygame.K_n:
                        if self.manual_override: self.tsp_manual -= 10
            if event.type == pygame.KEYUP:
                if event.key == pygame.K_SPACE: self.manual_override = False
        return True

    def _adjust_tuning(self, direction):
        idx = self.tuning_selection
        name, val, step, lo, hi, tag = self.tuning_params[idx]
        new_val = self.tuning_values[idx] + direction * step
        new_val = max(lo, min(hi, new_val))
        if "Horizon" in name:
            new_val = int(new_val)
        self.tuning_values[idx] = new_val
        if tag == "pi_kc":
            self.pi_temp.Kc = new_val
        elif tag == "pi_ti":
            self.pi_temp.Ti = new_val


# ---------- Simulation task (with pacing cap) ----------
async def simulation_task(server, plant, pi_temp, gui, nodes):
    (feed_node, temp_node, fire_node, xc_node, xs_node, qual_node,
     tsp_node, twall_node, apc_enable_node, apc_new_sp_node, apc_applied_node, manual_tsp_node,
     pi_kc_node, pi_ti_node,
     mpc_xc_sp_node, mpc_xs_max_node, mpc_temp_max_node, mpc_xc_w_node, mpc_move_w_node, mpc_horizon_node,
     sq_enable_node, sq_low_node, sq_high_node, sq_period_node) = nodes

    apc_was_on = False
    tsp = plant.T_solid

    with open('kiln_data.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time_min', 'feed_kg_s', 'T_K', 'T_sp_K', 'u_fire',
                         'Xc', 'Xs', 'Q', 'apc_on'])

    sim_time = 0.0
    square_timer = 0
    MIN_INTERVAL = 0.05   # cap at 20 Hz

    while True:
        loop_start = time.perf_counter()

        await feed_node.write_value(float(gui.current_feed))
        await sq_enable_node.write_value(gui.square_wave)
        await sq_low_node.write_value(gui.feed_low)
        await sq_high_node.write_value(gui.feed_high)
        await sq_period_node.write_value(gui.feed_period)
        await apc_enable_node.write_value(gui.apc_enabled)
        await manual_tsp_node.write_value(gui.tsp_manual)

        await pi_kc_node.write_value(float(gui.tuning_values[0]))
        await pi_ti_node.write_value(float(gui.tuning_values[1]))
        await mpc_xc_sp_node.write_value(float(gui.tuning_values[2]))
        await mpc_xs_max_node.write_value(float(gui.tuning_values[3]))
        await mpc_temp_max_node.write_value(float(gui.tuning_values[4]))
        await mpc_xc_w_node.write_value(float(gui.tuning_values[5]))
        await mpc_move_w_node.write_value(float(gui.tuning_values[6]))
        await mpc_horizon_node.write_value(int(gui.tuning_values[7]))

        if gui.square_wave:
            square_timer += 1
            if square_timer >= gui.feed_period:
                square_timer = 0
                gui.feed_state = not gui.feed_state
            gui.current_feed = gui.feed_low if not gui.feed_state else gui.feed_high
        else:
            gui.current_feed = nominal_feed
        plant.set_feed(gui.current_feed)

        if gui.manual_override:
            tsp = gui.tsp_manual
            await apc_new_sp_node.write_value(False)
            await apc_applied_node.write_value(False)
        elif gui.apc_enabled:
            if not apc_was_on:
                tsp = plant.T_solid
                await tsp_node.write_value(float(tsp))
                await apc_applied_node.write_value(True)
                await apc_new_sp_node.write_value(False)
                apc_was_on = True
            else:
                new_sp_ready = False
                while not new_sp_ready:
                    new_sp_ready = await apc_new_sp_node.read_value()
                    if not new_sp_ready: await asyncio.sleep(0.01)
                tsp = await tsp_node.read_value()
                await apc_new_sp_node.write_value(False)
                await apc_applied_node.write_value(True)
        else:
            tsp = 1200.0
            apc_was_on = False
            await apc_new_sp_node.write_value(False)
            await apc_applied_node.write_value(False)

        T, Xc, Xs, Q, u_fire = plant.step(pi_temp, tsp)
        sim_time += dt_ctrl

        await temp_node.write_value(float(T))
        await fire_node.write_value(float(u_fire))
        await xc_node.write_value(float(Xc))
        await xs_node.write_value(float(Xs))
        await qual_node.write_value(float(Q))
        await tsp_node.write_value(float(tsp))
        await twall_node.write_value(float(plant.T_wall))

        with open('kiln_data.csv', 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([sim_time, gui.current_feed, T, tsp, u_fire,
                             Xc, Xs, Q, int(gui.apc_enabled)])

        elapsed = time.perf_counter() - loop_start
        sleep_time = max(1.0 / gui.speed, MIN_INTERVAL) - elapsed
        if sleep_time > 0:
            await asyncio.sleep(sleep_time)


async def main():
    T_w_ss, T_s_ss, u_fire_ss, Xc_ss, Xs_ss, Q_ss = LimeKilnPlant.compute_steady_state(
        nominal_feed, 1200.0
    )

    plant = LimeKilnPlant()
    plant.T_wall = T_w_ss
    plant.T_solid = T_s_ss
    plant.F = nominal_feed
    plant.Xc = Xc_ss
    plant.Xs = Xs_ss
    plant.Q = Q_ss
    plant.u_fire = u_fire_ss

    pi_temp = PI(0.01, 200.0, dt_ctrl, 0.0, 1.0)
    pi_temp.reset(u_fire_ss)

    server, nodes_tuple = await create_opc_server()
    gui = KilnGUI(plant, pi_temp)

    async def gui_loop():
        running = True
        while running:
            running = gui.handle_events()
            gui.draw()
            pygame.display.flip()
            await asyncio.sleep(0.01)

    await asyncio.gather(
        simulation_task(server, plant, pi_temp, gui, nodes_tuple),
        gui_loop()
    )

if __name__ == "__main__":
    asyncio.run(main())
