#!/usr/bin/env python3
"""
Four‑tank ring process – real‑time digital twin with OPC UA server, Pygame GUI,
tuning panel, square‑wave leak, and stable startup.

FIXED: Full proportional scaling for all GUI elements. All positions and sizes
are derived from a single scale factor based on window dimensions.
"""
import asyncio
import time
import os
import subprocess
import numpy as np
import pygame
from asyncua import Server, ua
from models import FourTankPlant, PIBaseLayer, max_flow_map, L_min_tank, L_max_tank

# ---------- OPC UA Server ----------
async def create_opc_server():
    server = Server()
    await server.init()
    server.set_endpoint("opc.tcp://0.0.0.0:4840/freeopcua/server/")
    uri = "http://four-tank.simulator"
    idx = await server.register_namespace(uri)

    plant_node = await server.nodes.objects.add_object(idx, "Plant")

    level_nodes = [await plant_node.add_variable(idx, f"Tank{i+1}_Level", 2.0) for i in range(4)]
    for lv in level_nodes: await lv.set_writable()

    flow_names = ["F1", "F2", "F3", "F4", "Fm"]
    flow_nodes = [await plant_node.add_variable(idx, name, 2.0) for name in flow_names]
    for fv in flow_nodes: await fv.set_writable()

    apc_enable = await plant_node.add_variable(idx, "APC_Enable", False); await apc_enable.set_writable()
    apc_sps = [await plant_node.add_variable(idx, f"APC_FlowSP_F{i+1}", 0.0) for i in range(5)]
    for sp in apc_sps: await sp.set_writable()

    apc_new_sp = await plant_node.add_variable(idx, "APC_NewSP", False); await apc_new_sp.set_writable()
    apc_sp_applied = await plant_node.add_variable(idx, "APC_SP_Applied", False); await apc_sp_applied.set_writable()

    max_f2_node = await plant_node.add_variable(idx, "Max_F2", 2.5); await max_f2_node.set_writable()
    leak_node = await plant_node.add_variable(idx, "Leak", 0.2); await leak_node.set_writable()

    mpc_econ_w = await plant_node.add_variable(idx, "MPC_EconWeight", 0.0); await mpc_econ_w.set_writable()
    mpc_level_w = await plant_node.add_variable(idx, "MPC_LevelWeight", 5.0); await mpc_level_w.set_writable()
    mpc_move_w = await plant_node.add_variable(idx, "MPC_MoveWeight", 1.0); await mpc_move_w.set_writable()
    mpc_horizon = await plant_node.add_variable(idx, "MPC_Horizon", 6); await mpc_horizon.set_writable()

    manual_override = await plant_node.add_variable(idx, "Manual_Override", False); await manual_override.set_writable()
    manual_sps = [await plant_node.add_variable(idx, f"Manual_FlowSP_F{i+1}", 0.0) for i in range(5)]
    for sp in manual_sps: await sp.set_writable()

    await server.start()
    return server, (level_nodes, flow_nodes, apc_enable, apc_sps,
                    max_f2_node, leak_node, manual_override, manual_sps,
                    apc_new_sp, apc_sp_applied,
                    mpc_econ_w, mpc_level_w, mpc_move_w, mpc_horizon)


# ---------- Pygame GUI with tuning panel ----------
class PlantGUI:
    def __init__(self, plant, pi_layer):
        pygame.init()
        
        # Base reference size
        self.BASE_WIDTH = 1400
        self.BASE_HEIGHT = 900
        
        # Get display info for scalable window
        display_info = pygame.display.Info()
        self.width = min(self.BASE_WIDTH, display_info.current_w - 100)
        self.height = min(self.BASE_HEIGHT, display_info.current_h - 100)
        
        # Set window to be resizable
        self.screen = pygame.display.set_mode((self.width, self.height), pygame.RESIZABLE)
        pygame.display.set_caption("Four‑Tank Ring – Digital Twin")
        
        # Initialize fonts - will be updated in _update_fonts
        self.font = None
        self.big_font = None
        self.small_font = None
        
        self.plant = plant
        self.pi_layer = pi_layer

        self.speed = 1
        self.apc_enabled = False
        self.manual_override = False
        self.max_f2 = 2.5
        self.leak = 0.2

        # Square wave leak
        self.square_wave = False
        self.leak_low = 0.2
        self.leak_high = 0.5
        self.leak_period = 30
        self.leak_counter = 0
        self.leak_state = False

        # Tuning mode
        self.tuning_mode = False
        self.tuning_selection = 0
        self.tuning_params = [
            ("PI1 Kc", -0.8, -0.1, -5.0, 0.0, "pi1_Kc"),
            ("PI1 Ti", 4.0, 0.5, 0.5, 20.0, "pi1_Ti"),
            ("PI2 Kc", -0.8, -0.1, -5.0, 0.0, "pi2_Kc"),
            ("PI2 Ti", 4.0, 0.5, 0.5, 20.0, "pi2_Ti"),
            ("PI3 Kc", -0.8, -0.1, -5.0, 0.0, "pi3_Kc"),
            ("PI3 Ti", 4.0, 0.5, 0.5, 20.0, "pi3_Ti"),
            ("PI4 Kc", 0.8, 0.1, 0.0, 5.0, "pi4_Kc"),
            ("PI4 Ti", 4.0, 0.5, 0.5, 20.0, "pi4_Ti"),
            ("MPC econ w", 10.0, 0.5, 0.0, 20.0, "mpc_econ_w"),
            ("MPC level w", 5.0, 1.0, 0.0, 50.0, "mpc_level_w"),
            ("MPC move w", 1.0, 0.5, 0.0, 10.0, "mpc_move_w"),
            ("MPC horizon", 6.0, 1.0, 1.0, 10.0, "mpc_horizon"),
        ]
        self.tuning_values = [p[1] for p in self.tuning_params]

        # Store layout data
        self.scale = 1.0
        self.tank_positions = []
        self.flow_arrows = []
        self.margins = {}
        
        # PFD icon state
        self.pfd_icon_rect = None
        self.pfd_hover = False
        self.pfd_error = False
        self.pfd_error_timer = 0
        self.pfd_error_duration = 5  # seconds
        
        # Calculate initial layout
        self._calculate_scale()
        self._calculate_layout()

    def _calculate_scale(self):
        """Calculate the overall scale factor based on current window size"""
        # Base on both width and height, with minimum and maximum clamping
        scale_w = self.width / self.BASE_WIDTH
        scale_h = self.height / self.BASE_HEIGHT
        self.scale = min(scale_w, scale_h)
        # Allow scaling from 0.3 to 1.8
        self.scale = max(0.3, min(1.8, self.scale))
        self._update_fonts()

    def _update_fonts(self):
        """Update all fonts based on current scale factor"""
        base_size = max(10, min(24, int(16 * self.scale)))
        self.font = pygame.font.SysFont("Arial", int(base_size * 1.3))
        self.big_font = pygame.font.SysFont("Arial", int(base_size * 1.8))
        self.small_font = pygame.font.SysFont("Arial", max(8, int(12 * self.scale)))

    def _calculate_layout(self):
        """Calculate all positions based on current scale factor"""
        # Define margins based on scale
        self.margins = {
            'top': max(10, int(20 * self.scale)),
            'bottom': max(10, int(20 * self.scale)),
            'left': max(10, int(20 * self.scale)),
            'right': max(10, int(20 * self.scale)),
        }
        
        # Reserve space for tuning panel if visible
        tuning_width = 0
        if self.tuning_mode:
            tuning_width = max(160, int(280 * self.scale))
            tuning_width = min(tuning_width, self.width * 0.3)  # Max 30% of width
        
        # Reserve space for info panel (top-left)
        info_height = int(120 * self.scale)  # Approximate height of info text
        
        # Available space for the ring
        available_width = self.width - self.margins['left'] - self.margins['right'] - tuning_width
        available_height = self.height - self.margins['top'] - self.margins['bottom'] - info_height * 0.5
        
        # Calculate ring center (shifted slightly right if tuning panel is visible)
        center_x = self.margins['left'] + available_width // 2
        if tuning_width > 0:
            center_x = self.margins['left'] + (available_width - tuning_width) // 2 + tuning_width // 2
        
        center_y = self.margins['top'] + info_height * 0.5 + available_height // 2
        
        # Calculate ring radius based on available space
        max_radius = min(available_width, available_height) * 0.38
        radius = max_radius * 0.95  # Slightly smaller for padding
        
        # Ensure minimum radius
        min_radius = max(60, 120 * self.scale)
        radius = max(min_radius, radius)
        
        # Calculate tank size
        tank_size = radius * 0.42
        tank_size = max(40 * self.scale, min(120 * self.scale, tank_size))
        
        # Tank positions in a ring layout
        positions = [
            (center_x - radius * 0.7, center_y - radius * 0.7),  # Tank 1 (top-left)
            (center_x + radius * 0.7, center_y - radius * 0.7),  # Tank 2 (top-right)
            (center_x + radius * 0.7, center_y + radius * 0.7),  # Tank 3 (bottom-right)
            (center_x - radius * 0.7, center_y + radius * 0.7),  # Tank 4 (bottom-left)
        ]
        
        # Adjust positions to ensure tanks are within bounds
        self.tank_positions = []
        for x, y in positions:
            # Calculate tank top-left corner
            tx = x - tank_size / 2
            ty = y - tank_size / 2
            
            # Clamp to ensure tank stays within window
            min_x = self.margins['left']
            max_x = self.width - self.margins['right'] - tank_size
            min_y = self.margins['top']
            max_y = self.height - self.margins['bottom'] - tank_size
            
            tx = max(min_x, min(max_x, tx))
            ty = max(min_y, min(max_y, ty))
            
            self.tank_positions.append((tx, ty, tank_size, tank_size))
        
        # Calculate flow arrows
        self._calculate_flow_arrows()
        
        # Calculate PFD icon position (bottom-left) - 75% of original size
        icon_size = max(38, int(52 * self.scale))  # 75% of previous (70 * 0.75 = 52.5)
        icon_margin = max(10, int(20 * self.scale))
        icon_x = icon_margin
        icon_y = self.height - icon_size - icon_margin
        self.pfd_icon_rect = pygame.Rect(icon_x, icon_y, icon_size, icon_size)

    def _calculate_flow_arrows(self):
        """Calculate flow arrow positions between tanks"""
        self.flow_arrows = []
        
        # Gap between arrow ends and tank edges (small visible gap)
        gap_ratio = 0.15  # 15% of tank size as gap
        
        # Internal flows: F1 (T1->T2), F2 (T2->T3), F3 (T3->T4), F4 (T4->T1)
        for i in range(4):
            start_idx = i
            end_idx = (i + 1) % 4
            x1, y1, s1, _ = self.tank_positions[start_idx]
            x2, y2, s2, _ = self.tank_positions[end_idx]
            
            # Tank centers
            cx1 = x1 + s1/2
            cy1 = y1 + s1/2
            cx2 = x2 + s2/2
            cy2 = y2 + s2/2
            
            # Direction vector
            dx = cx2 - cx1
            dy = cy2 - cy1
            length = np.sqrt(dx*dx + dy*dy)
            if length > 0:
                dx, dy = dx/length, dy/length
            
            # Calculate gap based on tank size
            gap = s1 * gap_ratio
            
            # Start and end points with gap from tank edges
            start_x = cx1 + dx * (s1/2 + gap)
            start_y = cy1 + dy * (s1/2 + gap)
            end_x = cx2 - dx * (s2/2 + gap)
            end_y = cy2 - dy * (s2/2 + gap)
            
            # Perpendicular vector for label positioning
            perp_x = -dy
            perp_y = dx
            
            # Store arrow data with start and end points
            self.flow_arrows.append((start_x, start_y, end_x, end_y, dx, dy, perp_x, perp_y, i))
        
        # Make-up arrow (entering Tank4 from the LEFT) - horizontal
        if len(self.tank_positions) > 3:
            x4, y4, s4, _ = self.tank_positions[3]  # Tank 4
            cx4 = x4  # Left edge of Tank 4
            cy4 = y4 + s4/2  # Vertical center of Tank 4
            gap = s4 * gap_ratio
            arrow_len = max(30, 60 * self.scale)
            start_x = x4 - arrow_len
            start_y = cy4
            end_x = x4 - gap
            end_y = cy4
            self.flow_arrows.append((start_x, start_y, end_x, end_y, 1, 0, 0, 1, 4))
        
        # Loss arrow (exiting Tank2 to the right) - horizontal
        if len(self.tank_positions) > 1:
            x2, y2, s2, _ = self.tank_positions[1]
            cx2 = x2 + s2
            cy2 = y2 + s2/2
            gap = s2 * gap_ratio
            arrow_len = max(20, 30 * self.scale)
            start_x = x2 + s2 + gap
            start_y = cy2
            end_x = x2 + s2 + gap + arrow_len
            end_y = cy2
            self.flow_arrows.append((start_x, start_y, end_x, end_y, 1, 0, 0, 1, 5))

    def _draw_arrow(self, screen, start_x, start_y, end_x, end_y, color, width=None):
        """Draw a professional arrow between two points with scaled sizes"""
        if width is None:
            width = max(2, int(3 * self.scale))
        
        dx = end_x - start_x
        dy = end_y - start_y
        length = np.sqrt(dx*dx + dy*dy)
        min_length = 10 * self.scale
        if length < min_length:
            return
        
        # Normalize direction
        dx, dy = dx/length, dy/length
        
        # Draw the line
        pygame.draw.line(screen, color, (start_x, start_y), (end_x, end_y), width)
        
        # Draw arrow head
        head_length = max(8, min(15 * self.scale, length * 0.2))
        head_angle = np.radians(25)
        end_x_arrow = end_x - dx * head_length * 0.3
        end_y_arrow = end_y - dy * head_length * 0.3
        
        # Left wing
        left_x = end_x_arrow - head_length * np.cos(np.arctan2(dy, dx) - head_angle)
        left_y = end_y_arrow - head_length * np.sin(np.arctan2(dy, dx) - head_angle)
        # Right wing
        right_x = end_x_arrow - head_length * np.cos(np.arctan2(dy, dx) + head_angle)
        right_y = end_y_arrow - head_length * np.sin(np.arctan2(dy, dx) + head_angle)
        
        pygame.draw.polygon(screen, color, [(end_x, end_y), (left_x, left_y), (right_x, right_y)])

    def draw(self):
        """Main draw method - all elements scaled proportionally"""
        # Handle window resize
        current_size = self.screen.get_size()
        if current_size[0] != self.width or current_size[1] != self.height:
            self.width, self.height = current_size
            self._calculate_scale()
            self._calculate_layout()

        # Background
        self.screen.fill((248, 248, 248))
        
        levels = self.plant.levels
        flows = self.plant.flows

        # ----- Draw Flow Lines (arrows) -----
        flow_color = (0, 60, 120)
        
        # Internal flows (F1-F4)
        for i in range(4):
            start_x, start_y, end_x, end_y, dx, dy, perp_x, perp_y, idx = self.flow_arrows[i]
            
            # For F2 (vertical, right side) and F4 (vertical, left side), adjust gaps
            if i == 1:  # F2: Tank 2 -> Tank 3 (vertical, flowing down)
                x2, y2, s2, _ = self.tank_positions[1]  # Tank 2
                x3, y3, s3, _ = self.tank_positions[2]  # Tank 3
                
                # Start gap: below Tank 2 level reading
                start_y = y2 + s2 + max(20, 40 * self.scale)  # Below level reading
                # End gap: above Tank 3 label
                end_y = y3 - max(25, 45 * self.scale)  # Above TANK 3 label
                
                # Horizontal center of the arrow (slightly to the right of tank center)
                mid_x = (x2 + s2/2 + x3 + s3/2) / 2
                start_x = mid_x
                end_x = mid_x
                
                # Update the arrow points
                self.flow_arrows[i] = (start_x, start_y, end_x, end_y, dx, dy, perp_x, perp_y, idx)
                
            elif i == 3:  # F4: Tank 4 -> Tank 1 (vertical, flowing up)
                x4, y4, s4, _ = self.tank_positions[3]  # Tank 4
                x1, y1, s1, _ = self.tank_positions[0]  # Tank 1
                
                # Start gap: below TANK 4 label (label is above tank, so start below label)
                start_y = y4 - max(20, 35 * self.scale)  # Below TANK 4 label
                # End gap: above Tank 1 level reading (level reading is below tank)
                end_y = y1 + s1 + max(20, 40 * self.scale)  # Above level reading
                
                # Horizontal center of the arrow (slightly to the left of tank center)
                mid_x = (x4 + s4/2 + x1 + s1/2) / 2
                start_x = mid_x
                end_x = mid_x
                
                # Update the arrow points
                self.flow_arrows[i] = (start_x, start_y, end_x, end_y, dx, dy, perp_x, perp_y, idx)
            
            # Draw the arrow
            self._draw_arrow(self.screen, start_x, start_y, end_x, end_y, flow_color)
            
            # Flow label
            flow_label = f"{flows[i]:.2f} m³/h"
            label = self.font.render(flow_label, True, (0, 0, 0))
            
            # Position label at midpoint with offset
            mid_x = (start_x + end_x) / 2
            mid_y = (start_y + end_y) / 2
            offset = max(25, 45 * self.scale)
            
            # Different positioning for each flow to avoid overlap
            if i == 0:  # F1: T1->T2 (top)
                label_x = mid_x + perp_x * offset - label.get_width() // 2
                label_y = mid_y + perp_y * offset - label.get_height() // 2
            elif i == 1:  # F2: T2->T3 (right side)
                label_x = mid_x + perp_x * offset - label.get_width() // 2
                label_y = mid_y + perp_y * offset - label.get_height() // 2
            elif i == 2:  # F3: T3->T4 (bottom)
                label_x = mid_x + perp_x * offset - label.get_width() // 2
                label_y = mid_y + perp_y * offset - label.get_height() // 2
            else:  # F4: T4->T1 (left side)
                label_x = mid_x - perp_x * offset - label.get_width() // 2
                label_y = mid_y - perp_y * offset - label.get_height() // 2
            
            # Ensure label is on screen
            label_x = max(5, min(self.width - label.get_width() - 5, label_x))
            label_y = max(5, min(self.height - label.get_height() - 5, label_y))
            self.screen.blit(label, (label_x, label_y))
            
            # ----- Add description boxes for F2 and F4 -----
            if i == 1:  # F2: Tank 2 -> Tank 3 (right side)
                # Position on the RIGHT side of the arrow (outside the ring)
                desc_offset = max(80, 100 * self.scale)
                desc_x = mid_x + desc_offset
                desc_y = mid_y - 30 * self.scale
                
                # Three lines of text
                line1 = self.font.render("Recovery Boiler", True, (80, 80, 80))
                line2 = self.font.render("Flow Limit", True, (80, 80, 80))
                line3 = self.font.render("2.50 m³/h", True, (80, 80, 80))
                
                # Center the text block
                max_width = max(line1.get_width(), line2.get_width(), line3.get_width())
                line_x = desc_x - max_width // 2
                
                self.screen.blit(line1, (line_x, desc_y))
                self.screen.blit(line2, (line_x, desc_y + line1.get_height() + 2 * self.scale))
                self.screen.blit(line3, (line_x, desc_y + line1.get_height() + line2.get_height() + 4 * self.scale))
                
            elif i == 3:  # F4: Tank 4 -> Tank 1 (left side)
                # Position on the inside of the ring (towards center) - this is the "right side"
                desc_offset = max(50, 70 * self.scale)
                desc_x = mid_x + desc_offset  # Inside the ring
                desc_y = mid_y - 20 * self.scale
                
                # Two lines of text
                line1 = self.font.render("To Be", True, (80, 80, 80))
                line2 = self.font.render("Optimized", True, (80, 80, 80))
                
                # Center the text block
                max_width = max(line1.get_width(), line2.get_width())
                line_x = desc_x - max_width // 2
                
                self.screen.blit(line1, (line_x, desc_y))
                self.screen.blit(line2, (line_x, desc_y + line1.get_height() + 2 * self.scale))

        # Make-up stream (Fm) - NOW ENTERING TANK 4 FROM THE LEFT
        if len(self.flow_arrows) > 4:
            start_x, start_y, end_x, end_y, dx, dy, perp_x, perp_y, idx = self.flow_arrows[4]
            self._draw_arrow(self.screen, start_x, start_y, end_x, end_y, (200, 160, 50), max(3, int(4 * self.scale)))
            
            # Fm flow reading - ABOVE the arrow
            mid_x = (start_x + end_x) / 2
            mid_y = (start_y + end_y) / 2
            
            # Flow reading ABOVE the arrow
            flow_label = f"{flows[4]:.2f} m³/h"
            label = self.font.render(flow_label, True, (0, 0, 0))
            label_x = mid_x - label.get_width() // 2
            label_y = mid_y - max(30, 50 * self.scale)  # ABOVE arrow with visible gap
            self.screen.blit(label, (label_x, label_y))
            
            # Fm description - BELOW the arrow
            desc1 = self.font.render("Make-up", True, (80, 80, 80))
            desc2 = self.font.render("Stream", True, (80, 80, 80))
            desc_x = mid_x - max(desc1.get_width(), desc2.get_width()) // 2
            
            # Position BELOW arrow with visible gap
            desc_y1 = mid_y + max(10, 15 * self.scale)  # Gap from arrow
            desc_y2 = desc_y1 + desc1.get_height() + 2 * self.scale
            
            self.screen.blit(desc1, (desc_x, desc_y1))
            self.screen.blit(desc2, (desc_x, desc_y2))

        # Loss stream (exiting Tank2)
        if len(self.flow_arrows) > 5:
            start_x, start_y, end_x, end_y, dx, dy, perp_x, perp_y, idx = self.flow_arrows[5]
            self._draw_arrow(self.screen, start_x, start_y, end_x, end_y, (200, 80, 30))
            
            # Loss flow reading
            mid_x = (start_x + end_x) / 2
            mid_y = (start_y + end_y) / 2
            loss_label = f"{self.leak:.2f} m³/h"
            label = self.font.render(loss_label, True, (0, 0, 0))
            label_x = mid_x + 35 * self.scale - label.get_width() // 2
            label_y = mid_y - 25 * self.scale
            self.screen.blit(label, (label_x, label_y))
            
            # Loss description
            desc = self.font.render("Loss", True, (80, 80, 80))
            desc_x = mid_x + 35 * self.scale - desc.get_width() // 2
            desc_y = mid_y + 10 * self.scale
            self.screen.blit(desc, (desc_x, desc_y))

        # ----- Draw Tanks -----
        for i, (x, y, w, h) in enumerate(self.tank_positions):
            # Tank outline
            outline_width = max(2, int(3 * self.scale))
            pygame.draw.rect(self.screen, (60, 60, 80), (x, y, w, h), outline_width)
            
            # Liquid level
            frac = (levels[i] - L_min_tank) / (L_max_tank - L_min_tank)
            frac = max(0.0, min(1.0, frac))
            liquid_top = y + h - int(frac * h)
            liquid_height = int(h - (liquid_top - y))
            
            if liquid_height > 0:
                for j in range(liquid_height):
                    alpha = 0.5 + 0.5 * (j / liquid_height) if liquid_height > 0 else 0.5
                    color = (int(30 * (1 - alpha) + 60 * alpha), 
                            int(60 * (1 - alpha) + 120 * alpha), 
                            int(140 * (1 - alpha) + 200 * alpha))
                    pygame.draw.line(self.screen, color, 
                                   (x+3, liquid_top + j), 
                                   (x+w-3, liquid_top + j), 1)
            
            # Tank label
            label = self.big_font.render(f"TANK {i+1}", True, (0, 0, 80))
            label_x = x + w//2 - label.get_width()//2
            label_y = y - max(15, 30 * self.scale)
            label_y = max(self.margins['top'], label_y)
            self.screen.blit(label, (label_x, label_y))
            
            # Level reading
            level_text = self.font.render(f"{levels[i]:.2f} m", True, (0, 0, 0))
            level_x = x + w//2 - level_text.get_width()//2
            level_y = y + h + max(5, 10 * self.scale)
            if level_y + level_text.get_height() > self.height - self.margins['bottom']:
                level_y = self.height - self.margins['bottom'] - level_text.get_height()
            self.screen.blit(level_text, (level_x, level_y))

        # ----- Info Panel (top-left) -----
        margin = max(10, int(20 * self.scale))
        x_pos = margin
        y_pos = margin
        
        mode = "APC" if self.apc_enabled else "PI (base)"
        if self.manual_override: mode = "MANUAL"
        mode_surface = self.big_font.render(f"Mode: {mode}", True, (180, 0, 0))
        self.screen.blit(mode_surface, (x_pos, y_pos))
        y_pos += mode_surface.get_height() + max(5, 10 * self.scale)
        
        speed_text = self.font.render(f"Speed: {self.speed}x  (↑/↓)", True, (0, 0, 0))
        self.screen.blit(speed_text, (x_pos, y_pos))
        y_pos += speed_text.get_height() + max(2, 5 * self.scale)
        
        leak_str = f"Loss: {self.leak:.2f} m³/h"
        if self.square_wave:
            leak_str += " [SQUARE]"
        leak_text = self.font.render(leak_str + "  (K/L)", True, (0, 0, 0))
        self.screen.blit(leak_text, (x_pos, y_pos))
        y_pos += leak_text.get_height() + max(2, 5 * self.scale)
        
        maxf2_text = self.font.render(f"Max F2: {self.max_f2:.2f} m³/h  (J/I)", True, (0, 0, 0))
        self.screen.blit(maxf2_text, (x_pos, y_pos))
        y_pos += maxf2_text.get_height() + max(5, 10 * self.scale)
        
        controls_text = self.font.render("SPACE: Manual   A: APC   W: square loss   T: tune", True, (0, 0, 150))
        self.screen.blit(controls_text, (x_pos, y_pos))

        # ----- PFD Icon (bottom-left) -----
        self._draw_pfd_icon()

        # ----- Tuning Panel (right side) -----
        if self.tuning_mode:
            self._draw_tuning_panel()

    def _draw_pfd_icon(self):
        """Draw the PFD icon in the bottom-left corner"""
        if self.pfd_icon_rect is None:
            return
        
        x, y, w, h = self.pfd_icon_rect
        
        # Determine colors
        if self.pfd_error:
            # Error state - RED
            bg_color = (200, 50, 50)
            border_color = (150, 0, 0)
            text_color = (255, 255, 255)
        elif self.pfd_hover:
            # Hover state - lighter blue
            bg_color = (70, 130, 200)
            border_color = (30, 80, 150)
            text_color = (255, 255, 255)
        else:
            # Normal state - blue matching flow arrows
            bg_color = (40, 80, 150)
            border_color = (20, 50, 100)
            text_color = (255, 255, 255)
        
        # Draw rounded rectangle
        pygame.draw.rect(self.screen, bg_color, (x, y, w, h), border_radius=int(6 * self.scale))
        pygame.draw.rect(self.screen, border_color, (x, y, w, h), max(1, int(2 * self.scale)), border_radius=int(6 * self.scale))
        
        # Draw "PFD" text inside
        font_size = max(14, int(20 * self.scale))
        pfd_font = pygame.font.SysFont("Arial", font_size, bold=True)
        pfd_text = pfd_font.render("PFD", True, text_color)
        text_x = x + w//2 - pfd_text.get_width()//2
        text_y = y + h//2 - pfd_text.get_height()//2
        self.screen.blit(pfd_text, (text_x, text_y))

    def _draw_tuning_panel(self):
        """Draw the tuning panel with full scaling"""
        # Panel dimensions
        panel_width = max(160, int(280 * self.scale))
        panel_width = min(panel_width, self.width * 0.3)
        margin = max(10, int(20 * self.scale))
        panel_x = self.width - panel_width - margin
        panel_y = margin
        panel_height = self.height - margin * 2
        
        # Background
        pygame.draw.rect(self.screen, (255, 248, 220), (panel_x, panel_y, panel_width, panel_height))
        pygame.draw.rect(self.screen, (180, 180, 160), (panel_x, panel_y, panel_width, panel_height), max(1, int(2 * self.scale)))
        
        # Title
        title = self.big_font.render("TUNING PANEL", True, (180, 0, 0))
        title_x = panel_x + panel_width//2 - title.get_width()//2
        title_y = panel_y + max(5, 10 * self.scale)
        self.screen.blit(title, (title_x, title_y))
        
        # Parameter list
        param_start_y = title_y + title.get_height() + max(5, 15 * self.scale)
        line_height = max(16, int(22 * self.scale))
        param_spacing = max(2, 5 * self.scale)
        
        # Calculate how many parameters fit
        available_height = panel_height - param_start_y - panel_y - 130 * self.scale
        max_params = int(available_height / (line_height + param_spacing))
        max_params = min(max_params, len(self.tuning_params))
        
        # If not all fit, reduce font size and try again
        current_font = self.font
        if max_params < len(self.tuning_params):
            temp_font = pygame.font.SysFont("Arial", int(10 * self.scale))
            for i in range(min(max_params, len(self.tuning_params))):
                y_pos = param_start_y + i * (line_height + param_spacing)
                if y_pos + line_height > panel_y + panel_height - 130 * self.scale:
                    break
                name, val, step, lo, hi, tag = self.tuning_params[i]
                color = (0, 0, 0) if i != self.tuning_selection else (200, 0, 0)
                if "horizon" in name:
                    txt = f"{name}: {int(self.tuning_values[i])}"
                else:
                    txt = f"{name}: {self.tuning_values[i]:.2f}"
                txt_surface = temp_font.render(txt, True, color)
                self.screen.blit(txt_surface, (panel_x + 10 * self.scale, y_pos))
        else:
            for i in range(len(self.tuning_params)):
                y_pos = param_start_y + i * (line_height + param_spacing)
                if y_pos + line_height > panel_y + panel_height - 130 * self.scale:
                    break
                name, val, step, lo, hi, tag = self.tuning_params[i]
                color = (0, 0, 0) if i != self.tuning_selection else (200, 0, 0)
                if "horizon" in name:
                    txt = f"{name}: {int(self.tuning_values[i])}"
                else:
                    txt = f"{name}: {self.tuning_values[i]:.2f}"
                txt_surface = current_font.render(txt, True, color)
                self.screen.blit(txt_surface, (panel_x + 10 * self.scale, y_pos))
        
        # Description section (at bottom of panel)
        desc_y = panel_y + panel_height - 120 * self.scale
        if desc_y < panel_y + 200 * self.scale:
            desc_y = panel_y + 200 * self.scale
        
        # Draw separator line
        line_y = desc_y - max(3, 5 * self.scale)
        if line_y > panel_y:
            pygame.draw.line(self.screen, (150, 150, 150), 
                           (panel_x + 10 * self.scale, line_y),
                           (panel_x + panel_width - 10 * self.scale, line_y), 
                           max(1, int(2 * self.scale)))
        
        # PI Parameters
        desc_text = self.small_font.render("PI Parameters:", True, (0, 0, 120))
        self.screen.blit(desc_text, (panel_x + 10 * self.scale, desc_y))
        desc_y += self.small_font.get_linesize() + 2 * self.scale
        
        desc_text = self.small_font.render("  Kc: P (Proportional Gain)", True, (60, 60, 60))
        self.screen.blit(desc_text, (panel_x + 10 * self.scale, desc_y))
        desc_y += self.small_font.get_linesize() + self.scale
        
        desc_text = self.small_font.render("  Ti: I (Integral Time)", True, (60, 60, 60))
        self.screen.blit(desc_text, (panel_x + 10 * self.scale, desc_y))
        desc_y += self.small_font.get_linesize() + 5 * self.scale
        
        # MPC Parameters
        desc_text = self.small_font.render("MPC Parameters:", True, (0, 0, 120))
        self.screen.blit(desc_text, (panel_x + 10 * self.scale, desc_y))
        desc_y += self.small_font.get_linesize() + 2 * self.scale
        
        desc_text = self.small_font.render("  econ w: Production priority", True, (60, 60, 60))
        self.screen.blit(desc_text, (panel_x + 10 * self.scale, desc_y))
        desc_y += self.small_font.get_linesize() + self.scale
        
        desc_text = self.small_font.render("  level w: Level control priority", True, (60, 60, 60))
        self.screen.blit(desc_text, (panel_x + 10 * self.scale, desc_y))
        desc_y += self.small_font.get_linesize() + self.scale
        
        desc_text = self.small_font.render("  move w: Smooth operation priority", True, (60, 60, 60))
        self.screen.blit(desc_text, (panel_x + 10 * self.scale, desc_y))
        desc_y += self.small_font.get_linesize() + self.scale
        
        desc_text = self.small_font.render("  horizon: Look-ahead time", True, (60, 60, 60))
        self.screen.blit(desc_text, (panel_x + 10 * self.scale, desc_y))
        desc_y += self.small_font.get_linesize() + 5 * self.scale
        
        # Controls hint
        desc_text = self.small_font.render("↑↓ select  ←→ adjust", True, (0, 0, 120))
        self.screen.blit(desc_text, (panel_x + 10 * self.scale, desc_y))

    def handle_events(self):
        # Update error timer
        if self.pfd_error:
            self.pfd_error_timer += 0.01  # Increment each frame
            if self.pfd_error_timer >= self.pfd_error_duration:
                self.pfd_error = False
                self.pfd_error_timer = 0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            if event.type == pygame.VIDEORESIZE:
                self.width, self.height = event.w, event.h
                self.screen = pygame.display.set_mode((self.width, self.height), pygame.RESIZABLE)
                self._calculate_scale()
                self._calculate_layout()
            
            # Mouse events for PFD icon
            if event.type == pygame.MOUSEMOTION:
                if self.pfd_icon_rect:
                    self.pfd_hover = self.pfd_icon_rect.collidepoint(event.pos)
            
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left click
                    if self.pfd_icon_rect and self.pfd_icon_rect.collidepoint(event.pos):
                        self._open_pfd()
            
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    return False
                if event.key == pygame.K_t:
                    self.tuning_mode = not self.tuning_mode
                    self._calculate_layout()

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
                    if event.key == pygame.K_UP:
                        self.speed = min(300, self.speed + 1)
                    elif event.key == pygame.K_DOWN:
                        self.speed = max(1, self.speed - 1)
                    elif event.key == pygame.K_a:
                        self.apc_enabled = not self.apc_enabled
                    elif event.key == pygame.K_SPACE:
                        self.manual_override = True
                    elif event.key == pygame.K_w:
                        self.square_wave = not self.square_wave
                    elif event.key == pygame.K_j:
                        self.max_f2 = max(0.1, self.max_f2 - 0.1)
                    elif event.key == pygame.K_i:
                        self.max_f2 = min(5.0, self.max_f2 + 0.1)
                    elif event.key == pygame.K_k:
                        self.leak = max(0.0, self.leak - 0.05)
                    elif event.key == pygame.K_l:
                        self.leak = min(1.0, self.leak + 0.05)
            if event.type == pygame.KEYUP:
                if event.key == pygame.K_SPACE:
                    self.manual_override = False
        return True

    def _open_pfd(self):
        """Open the PFD image in the default viewer"""
        filename = "PFD.jpg"
        
        # Check if file exists
        if os.path.exists(filename):
            try:
                # Open with default Windows image viewer
                os.startfile(filename)
                # Reset error state if it was in error
                self.pfd_error = False
                self.pfd_error_timer = 0
            except Exception as e:
                print(f"Error opening file: {e}")
                self.pfd_error = True
                self.pfd_error_timer = 0
        else:
            # File not found - show error
            self.pfd_error = True
            self.pfd_error_timer = 0
            print(f"File not found: {filename}")

    def _adjust_tuning(self, direction):
        idx = self.tuning_selection
        name, val, step, lo, hi, tag = self.tuning_params[idx]
        new_val = self.tuning_values[idx] + direction * step
        new_val = max(lo, min(hi, new_val))
        if "horizon" in name:
            new_val = int(new_val)
        self.tuning_values[idx] = new_val
        if tag.startswith("pi"):
            pi_num = int(tag[2]) - 1
            if "Kc" in tag:
                self.pi_layer.update_pi(pi_num, Kc=new_val)
            else:
                self.pi_layer.update_pi(pi_num, Ti=new_val)
        elif tag == "mpc_econ_w":
            pass
        elif tag == "mpc_level_w":
            pass
        elif tag == "mpc_move_w":
            pass
        elif tag == "mpc_horizon":
            pass


# ---------- Simulation task ----------
async def simulation_task(server, plant, pi_layer, gui, nodes):
    (level_nodes, flow_nodes, apc_enable_node, apc_sps,
     max_f2_node, leak_node, manual_override_node, manual_sps,
     apc_new_sp_node, apc_sp_applied_node,
     mpc_econ_w_node, mpc_level_w_node, mpc_move_w_node, mpc_horizon_node) = nodes

    apc_was_on = False
    u_sp = np.array([2.0, 1.8, 1.8, 2.0, 0.2])
    square_timer = 0
    MIN_INTERVAL = 0.05

    while True:
        loop_start = time.perf_counter()

        await max_f2_node.write_value(float(gui.max_f2))
        await apc_enable_node.write_value(gui.apc_enabled)
        await manual_override_node.write_value(gui.manual_override)

        if gui.square_wave:
            square_timer += 1
            if square_timer >= gui.leak_period:
                square_timer = 0
                gui.leak_state = not gui.leak_state
            gui.leak = gui.leak_low if not gui.leak_state else gui.leak_high
        await leak_node.write_value(float(gui.leak))

        await mpc_econ_w_node.write_value(float(gui.tuning_values[8]))
        await mpc_level_w_node.write_value(float(gui.tuning_values[9]))
        await mpc_move_w_node.write_value(float(gui.tuning_values[10]))
        await mpc_horizon_node.write_value(int(gui.tuning_values[11]))

        plant.max_flows[2] = gui.max_f2
        plant.leak = gui.leak
        levels = plant.levels

        if gui.manual_override:
            for i in range(5):
                u_sp[i] = await manual_sps[i].read_value()
            await apc_new_sp_node.write_value(False)
            await apc_sp_applied_node.write_value(False)
        elif gui.apc_enabled:
            if not apc_was_on:
                pi_sp = pi_layer.compute_setpoints(levels, gui.max_f2, gui.leak, max_flow_map[4])
                for i, val in enumerate(pi_sp):
                    await apc_sps[i].write_value(float(val))
                u_sp = pi_sp
                await apc_sp_applied_node.write_value(True)
                await apc_new_sp_node.write_value(False)
                apc_was_on = True
            else:
                new_sp_ready = False
                while not new_sp_ready:
                    new_sp_ready = await apc_new_sp_node.read_value()
                    if not new_sp_ready:
                        await asyncio.sleep(0.01)
                for i in range(5):
                    u_sp[i] = await apc_sps[i].read_value()
                await apc_new_sp_node.write_value(False)
                await apc_sp_applied_node.write_value(True)
        else:
            u_sp = pi_layer.compute_setpoints(levels, gui.max_f2, gui.leak, max_flow_map[4])
            apc_was_on = False
            await apc_new_sp_node.write_value(False)
            await apc_sp_applied_node.write_value(False)

        plant.simulate_interval(u_sp, gui.leak)

        for i in range(4):
            await level_nodes[i].write_value(float(plant.levels[i]))
        for i in range(5):
            await flow_nodes[i].write_value(float(plant.flows[i]))

        elapsed = time.perf_counter() - loop_start
        desired = 1.0 / gui.speed
        sleep_time = max(desired, MIN_INTERVAL) - elapsed
        if sleep_time > 0:
            await asyncio.sleep(sleep_time)


async def main():
    plant = FourTankPlant()
    pi_layer = PIBaseLayer(dt=1.0)
    pi_layer.initialize_steady_state([2.0, 1.8, 1.8, 2.0, 0.2])

    server, nodes_tuple = await create_opc_server()
    gui = PlantGUI(plant, pi_layer)

    async def gui_loop():
        running = True
        while running:
            running = gui.handle_events()
            gui.draw()
            pygame.display.flip()
            await asyncio.sleep(0.01)

    await asyncio.gather(
        simulation_task(server, plant, pi_layer, gui, nodes_tuple),
        gui_loop()
    )

if __name__ == "__main__":
    asyncio.run(main())