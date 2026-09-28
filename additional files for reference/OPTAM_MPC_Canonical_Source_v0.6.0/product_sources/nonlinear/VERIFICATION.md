# Nonlinear Starter 0.2.0 — consolidated-line verification

The generated package passes all 31 nonlinear product tests plus the four
canonical core tests described in the canonical source release. The 660-cycle
offline exchanger run passes with zero solver failures. Its real two-process
OPC run also passes, with all 661 OPC rows exactly equal to the independent
offline run.

Verified 2026-09-02 on Linux, Python 3.12 with CasADi 3.7.2, Matplotlib 3.10.6
and asyncua 2.0.1. Windows installation and desktop live trends remain for local
acceptance; no live industrial plant or external OPC system was connected.

## Automated checks

**31 tests PASS.** Reproduce from the package root:

```bat
.venv\Scripts\python -m unittest discover -s tests
```

Coverage includes equilibrium, independent next-sample arithmetic, nonlinear
non-additivity, pressure-dependent gain, nominal solver move, stationary
velocity-form anchoring with model bias, model-source fingerprint changes,
location-independent fingerprints, independently configured plant mismatch,
invalid parameters/IDs/return values/domains/initial states/model checks,
runtime domain and non-finite-data rejection, incomplete templates, and
settled-window checks. Protocol checks exercise mismatched run/application,
sample sequence and excessive MV commands. Real two-process fault tests confirm
no plant advance when a controller is absent or its model version differs.

A distinct synthetic 2-CV/2-MV/2-DV application is built **from the generic
templates only**, without reading the exchanger reference files. Its 60-cycle
closed-loop run passes with no solver failures. This verifies the engineering
workflow is not hard-wired to the supplied heat exchanger.

Every model load also probes numerical and symbolic outputs/Jacobians at the
nominal point, axis end points and, for small models, all domain box corners.
The supplied model has 77 probes and five independent numerical check entries.
These are consistency checks, not model identification or proof of validity.

## Complete reference simulations

| Case | Cycles | Solver failures | Acceptance |
|---|---:|---:|---|
| Combined targets and measured DVs | 660 | 0 | PASS |
| Fixed-target disturbances, feedforward on | 360 | 0 | PASS |
| Same disturbances, feedforward off | 360 | 0 | PASS |
| Modest plant mismatch, unchanged controller | 360 | 0 | PASS |
| Combined scenario through real local OPC | 660 | 0 | PASS |

Combined case: flow 57.5168–64.0001 m3/h; temperature 63.4056–66.0027 degC;
maximum MV moves 0.9316 and 1.0000 percentage points/sample (unrounded checks
pass the configured 1-point limits). Final flow/temperature errors are about
0.0000146 m3/h and 0.0000260 degC. Offline mean solver time was about 0.052 s
here; this is environment-dependent, not an industrial timing guarantee.

For the modest mismatch case, maximum errors over the final five simulated
minutes are 0.00542 m3/h and 0.01110 degC, both below the 0.1 limits.

The fixed-target comparison gives these integral absolute errors:

| CV | Feedforward on | Feedforward off | Unit |
|---|---:|---:|---|
| Flow | 4.3838 | 5.1463 | (m3/h)*min |
| Temperature | 2.1728 | 2.4866 | degC*min |

These cases demonstrate this particular configuration. They do not establish
that nonlinear MPC is better than linear MPC, or that the model is robust to
larger mismatch. The earlier linear example used different lag/delay assumptions,
bounds and tuning, so its performance numbers are not a controlled comparison.

## Full OPC/offline equivalence

```bat
.venv\Scripts\python tests\verify_full_opc.py
```

The test copies the generic runtime into an isolated temporary folder **without
an examples directory**, copies the four engineered application files separately,
and starts real twin and controller processes. All **661 complete CSV records
match the offline run byte-for-byte**. Successful results are saved under
examples/01_heat_exchanger/verified_opc. The test uses accelerated speed=1000
and a temporary loopback port; normal reference settings use speed=20 and port
4842. It may take a few minutes on another computer.

## Code lineage and review boundary

The optimisation core and velocity predictor are byte-identical to Linear
Starter v0.5.0. New code supplies the nonlinear model contract and generic
model/plant loading; shared engineering/OPC helpers are reused. The internal
folder name optam_mpc_linear is retained for compatibility; it does not mean
the new model is linear. The frontend runners call nonlinear_runtime.py, and
the nonlinear transition is evaluated symbolically inside the optimisation.

This is a **separate package**. The linear release and frozen nonlinear baseline
have not been modified. No existing deployment is upgraded automatically.

Remaining work before wider use: clean Windows install/launcher checks, desktop
live-trend check, independent engineer review of the manual, application-specific
model validation, noise and broader robustness testing where needed. Hidden
states, explicit delay modelling/state estimation and industrial OPC integration
remain outside this deliberately small measured-state interface.
