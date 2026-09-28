"""Illustrative first-principles steady map with two empirical output lags.

Engineering units: valve percentage points, flow m3/h, temperature degC,
pressure barg; dt and tau in seconds. See DESIGN_BASIS.md.
"""
import casadi as ca

STATE_IDS = ("flow", "temperature")
MV_IDS = ("hx_valve", "bypass_valve")
DV_IDS = ("inlet_temperature", "upstream_pressure")


def validate_parameters(p):
    required = {"branch_flow_nominal", "valve_nominal", "pressure_nominal", "downstream_pressure",
                "inlet_temperature_nominal", "hot_temperature", "heat_transfer_flow",
                "valve_curvature", "flow_tau_seconds", "temperature_tau_seconds"}
    if set(p) != required:
        raise ValueError(f"Parameters must be exactly {sorted(required)}")
    for key in ("branch_flow_nominal", "valve_nominal", "heat_transfer_flow",
                "flow_tau_seconds", "temperature_tau_seconds"):
        if p[key] <= 0:
            raise ValueError(f"{key} must be positive.")
    if p["pressure_nominal"] <= p["downstream_pressure"]:
        raise ValueError("Nominal upstream pressure must exceed downstream pressure.")
    if not 0 <= p["valve_curvature"] <= 0.2:
        raise ValueError("This illustrative valve law requires curvature in [0, 0.2].")
    if p["hot_temperature"] <= p["inlet_temperature_nominal"]:
        raise ValueError("Heating example requires a hotter utility.")


def transition(x, u, d, p, dt):
    pressure_factor = ca.sqrt((d["upstream_pressure"]-p["downstream_pressure"])
                              /(p["pressure_nominal"]-p["downstream_pressure"]))
    h = u["hx_valve"]/p["valve_nominal"]
    b = u["bypass_valve"]/p["valve_nominal"]
    qh = p["branch_flow_nominal"]*(h+p["valve_curvature"]*(h-1)**2)*pressure_factor
    qb = p["branch_flow_nominal"]*(b+p["valve_curvature"]*(b-1)**2)*pressure_factor
    total = qh+qb
    th = p["hot_temperature"]-(p["hot_temperature"]-d["inlet_temperature"])*ca.exp(-p["heat_transfer_flow"]/qh)
    mixed = (qh*th+qb*d["inlet_temperature"])/total
    af = ca.exp(-dt/p["flow_tau_seconds"])
    at = ca.exp(-dt/p["temperature_tau_seconds"])
    return {"flow": af*x["flow"]+(1-af)*total,
            "temperature": at*x["temperature"]+(1-at)*mixed}
