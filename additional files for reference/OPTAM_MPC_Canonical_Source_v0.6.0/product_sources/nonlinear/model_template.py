"""Copy beside application.toml; replace the marked engineering sections.

Only trusted Python: loading/validation executes this file.
Use CasADi maths for expressions that involve x/u/d. Do not import local helper files.
"""
import casadi as ca

# EDIT: must match TOML order exactly. Every state is an instrumented primary CV.
STATE_IDS = ("cv_a", "cv_b")
MV_IDS = ("mv_a", "mv_b")
DV_IDS = ("dv_a", "dv_b")


def validate_parameters(p):
    # EDIT: parameter names, units and allowable ranges for your own equations.
    required = {"tau_a_seconds", "tau_b_seconds", "gain_a", "gain_b", "curve_a", "curve_b"}
    if set(p) != required:
        raise ValueError(f"Parameters must be exactly {sorted(required)}")
    if p["tau_a_seconds"] <= 0 or p["tau_b_seconds"] <= 0:
        raise ValueError("Time constants must be positive seconds.")


def transition(x, u, d, p, dt):
    # EDIT: illustrative equations only, not a heat exchanger model.
    # x/u/d are dictionaries: values may be floats OR symbolic solver expressions.
    # Return the ABSOLUTE state one dt-second interval later, never an increment.
    target_a = p["gain_a"]*u["mv_a"] + d["dv_a"] + p["curve_a"]*u["mv_a"]**2
    target_b = p["gain_b"]*u["mv_b"] + d["dv_b"] + p["curve_b"]*u["mv_b"]**2
    a = ca.exp(-dt/p["tau_a_seconds"])
    b = ca.exp(-dt/p["tau_b_seconds"])
    return {"cv_a": a*x["cv_a"]+(1-a)*target_a,
            "cv_b": b*x["cv_b"]+(1-b)*target_b}
