# v0.6.0 consolidated-line verification

The generated package passes all 74 product tests. Four additional canonical
core tests cover declared move combinations for 1, 2 and 3 MVs, coefficient
validation, runtime tuning without solver rebuild and rollback on invalid
tuning. The 660-cycle 2x2 exchanger passes offline with zero solver failures.
The 1050-cycle surge-vessel real OPC run passes, and all 1051 recorded OPC rows
exactly match its independent offline run.

## Linear heat-exchanger extension — 2026-09-02

This release adds generic first-order and static-gain paths, whole-sample
dead times, target events, measured-disturbance feedforward selection and
windowed acceptance checks. Application-specific engineering is in TOML;
both examples use the same runtime and launchers.

Verification uses Python 3.12, CasADi 3.7.2, Matplotlib 3.10.6 and asyncua 2.0.1.
All 74 automated tests pass. The suite includes analytical dynamics, physical gain calculations,
invalid definitions, target updates, settling-window checks and OPC fault cases.

The 660-cycle heat-exchanger combined scenario passes with zero solver failures.
Its complete OPC and offline CSVs match exactly (661 records). The 1050-cycle
surge-vessel OPC/offline comparison also passes (1051 matching records); its
CSV is byte-identical to the v0.4.0 OPC reference. The full OPC tests use an
isolated runtime without the examples directory, confirming that no
example-specific Python is required.

Heat-exchanger reference results are in `examples/02_heat_exchanger/verified/`
and `verified_opc/`. The combined case covers flow and temperature target
changes, inlet-temperature disturbance and upstream-pressure disturbance.
For the fixed-target disturbance case, measured feedforward reduces flow
integral absolute error from 3.157 to 2.244 (m3/h)*min and temperature error
from 2.063 to 0.757 degC*min. These are illustrative simulation results, not
plant performance guarantees.

The mismatch case increases plant gains by 15%, time constants by 20% and
dead times by 5 seconds without changing the controller model. Nominal tuning
produces sustained flow oscillation: final-point checks alone miss this, but
the final-five-minute check correctly FAILS (maximum flow error 0.578 m3/h,
limit 0.1). Increasing both move weights from 0.05 to 0.5 PASSES the same
checks (maximum settled flow error 0.0974 m3/h and temperature error 0.0360
degC). Both cases have zero solver failures. The failed case is retained
deliberately; nominal-model success is not a robustness demonstration.

To reproduce the tests from the extracted package on Windows:

```bat
.venv\Scripts\python -m unittest discover -s tests
.venv\Scripts\python tests\verify_full_opc.py
.venv\Scripts\python tests\verify_full_opc.py --example 02_heat_exchanger
```

The long tests refresh the corresponding `verified_opc` results. The heat
exchanger README supplies commands for every offline comparison case.

Windows batch launchers, a clean Windows installation and the desktop trend
window remain to be checked on the user's computer. Python entry points,
real loopback OPC communication and generated static plots were checked on
Linux. This package is a supervised local digital twin, not live-plant OPC
integration or a safety system. The frozen nonlinear baseline is unchanged.

## Historical v0.4.0 verification

## OPC extension — 2026-09-02

52 automated tests pass with Python 3.12, CasADi 3.7.2, Matplotlib 3.10.6 and
asyncua 2.0.1. The underlying controller core is unchanged.

The added protocol tests cover application fingerprints, restart/sequence
checks, missing/non-finite data, bad OPC quality and timestamps, command limits,
controller ownership, solver-fault responses and loopback endpoint restriction.

Real OPC integration tests use separate server/twin and controller processes:

- Successful 2-CV/2-MV/2-DV run, with CSV exactly matching offline execution.
- No-controller timeout without any plant advance.
- Client disconnect after one accepted command: exactly one plant step.
- Invalid command rejected without a plant step.
- Wrong application rejected by the controller.
- Operator stop and partial-run reporting.
- Hidden disturbance present in observer telemetry but absent from the
  controller's measurement frame.
- Report identity matched against the current-run marker.

The full 1050-cycle vessel reference was then run in an isolated copied runtime
with no examples directory. Its OPC and offline runs produced exactly identical
CSV files: all 1051 records, including the initial point. The OPC run had no
failures and all five engineering checks passed. Reference results are under
`examples/01_surge_vessel/verified_opc/`.

This long verification used speed=1000 (unpaced relative to normal solving) and
completed its OPC phase in about 116 seconds in this environment. The supplied
OPC template uses speed=20 for easier observation. Neither number is an
industrial deadline guarantee.

To rerun the optional long comparison:

```bat
.venv\Scripts\python tests\verify_full_opc.py
```

It creates temporary test processes/folders, compares the complete CSV records
and replaces `verified_opc` reference outputs on success. The normal test suite
does not include this long comparison.

Windows batch launchers and the desktop live-trend window have not been
executed on Windows here. Python entry points, local OPC networking and static
result plots were checked on Linux; the static plot was visually inspected.
The installer download was interrupted in this workspace; verification used
already available packages at the pinned versions. A clean Windows installation
remains part of Pieter's acceptance exercise.

This is a supervised local twin protocol, not an adapter for arbitrary plant
OPC tags, a free-running plant simulator, or an industrial deployment/safety
system. The manual explicitly describes these boundaries.

## Previous offline foundation

- Generic simulation runner replaces vessel-specific runner and launcher.
- Separate generic application and simulation templates.
- Independent plant paths and explicit MV/CV/measured-DV mappings.
- Hidden disturbance events remain invisible to controller DV inputs.
- Named checks, generic plots, CSV records and JSON reports.
- Manual covers the complete supported offline engineering workflow.
- Old example-only launcher and outdated tutorial draft are omitted from this
  version; earlier packages and the frozen nonlinear baseline remain unchanged.
- Generic nonlinear controller core is unchanged from the preceding starter.

### Offline checks retained

31 automated tests pass under Python 3.12, CasADi 3.7.2 and Matplotlib 3.10.6.
Tests include the original 13 regressions plus:

- templates filled for two primary CVs, two MVs and two measured DVs with
  renamed controller variables and no derived CV requirement;
- a real solver run in a temporary generic runtime folder with no examples;
- configuration paths containing spaces and an external working directory;
- hidden/measured disturbance separation, independent model gains and drift;
- multiple derived CV sources, mapping/event/plot validation;
- non-finite gain rejection, named destination model checks;
- all four acceptance metrics and failed-acceptance exit code;
- solver-failure hold-last-MV behaviour and propagation of programming errors.

The full 1050-cycle vessel test passes, including a separate run using an
isolated runtime without an examples directory. That isolated vessel run uses
the reference TOML values; it demonstrates absence of runtime dependencies,
not independent human usability validation.

| Quantity | Result |
|---|---:|
| Solver failures | 0 |
| Minimum raw level | 49.999479 % |
| Maximum raw level | 50.590742 % |
| Final raw level | 49.999479 % |
| Largest absolute outlet move | 0.057948496 m3/h per sample |
| Final outlet | 60.217877 m3/h |
| Declared closed-loop acceptance | PASS |

The generated reference plot was visually inspected. Small numerical/timing
differences across machines are expected.

### Remaining limits of the offline foundation

- Windows batch launchers were inspected, not executed on Windows. Python
  entry points and external-path behaviour were executed on Linux.
- Pieter's independent clean-start engineering exercise remains the user
  acceptance test of the manual and templates.
- No claim of arbitrary-disturbance safety, complete scenario coverage or
  optimal tuning follows from this one vessel test.
- Gain/lag/dead-time paths, nonlinear user equations and live-plant deployment
  are not implemented in this development step.
