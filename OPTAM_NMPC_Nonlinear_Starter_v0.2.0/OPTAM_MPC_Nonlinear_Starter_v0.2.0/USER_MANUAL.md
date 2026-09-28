# OPTAM-MPC nonlinear application user manual

Version 0.1.0-development — engineering reference, not a comprehensive tutorial.

## 1. Purpose and responsibilities

Use this package to engineer, validate and exercise a small nonlinear MPC
application offline and through a local OPC UA digital twin. The engineer
supplies a working process model from first principles or an external source.
This package does not identify models from plant tests.

| Owner | Responsibility |
|---|---|
| APC engineer | Variable selection, units, model equations and parameters, operating domain, initial equilibrium, tuning, scenarios and acceptance criteria |
| Package | Nonlinear optimisation, velocity-form feedback, input limits, generic simulation, OPC exchange, plots and reports |
| Site engineering team | Instrument validation, plant interfaces, operating procedures, independent protection and commissioning approval; outside this release |

Linear applications can be configuration-only. Nonlinear applications require
some Python: named variables, dictionaries, arithmetic, function calls and
return values. The engineer does **not** write solver, OPC or plotting code.
AI assistance can help transcribe equations, but it cannot certify their
physical correctness. Review the equations, units and independent checks.

## 2. Supported model contract and deliberate limits

The engineer provides an absolute discrete transition:

`x_next = F(x, u, d, parameters, dt_seconds)`.

In this first interface every state `x` is an instrumented primary CV. All
states are measured each sample. `u` contains the manipulated inputs, and `d`
contains measured disturbances. The number of variables is not fixed at 2x2.
The runtime reorders returned dictionaries according to declared TOML IDs.

Supported: smooth nonlinear equations, CV targets or zones, soft protected CV
limits, hard MV position and move limits, measured-DV feedforward, target
events, separate plant parameters, local OPC simulation and result checks.

Not supported by this small interface: hidden dynamic states, state estimation,
explicit transport-delay queues, derived auxiliary CVs, plant-only hidden
inputs, model identification or arbitrary live-plant OPC tags. Do not label an
unmeasured state as an instrumented CV to make it fit. Such an application needs
an explicit state-estimation/model-interface extension, not a documentation trick.
The earlier surge-vessel derived-rate feature remains in the separate linear
package; it is not part of this release's nonlinear contract.

The heat exchanger uses algebraic branch-flow/heat-transfer calculations and
two output lags. Branch flows are **not** assumed measured. Its timing model
is deliberately simpler than the earlier seven-path linear example; it is not
an exact dynamic conversion of that example.

## 3. Install and create an application

Use Windows with Python 3.11 or later. Open Command Prompt in the extracted
package directory; commands below assume that working directory. Run:

```bat
setup_windows.bat
mkdir applications\my_hx
copy application_template.toml applications\my_hx\application.toml
copy model_template.py applications\my_hx\model.py
copy simulation_template.toml applications\my_hx\simulation.toml
copy opc_template.toml applications\my_hx\opc.toml
```

Setup creates `.venv` and installs the pinned packages in `requirements.txt`.
An internet connection is required for installation. The batch launchers use
that environment without requiring manual activation. Quote paths containing
spaces. Run from an open Command Prompt so error messages remain visible.

The generic templates contain deliberately invalid placeholder values. Replace
every REPLACE entry; add/delete complete variable blocks to match your process.
Do not modify the generic runners or copy an example-specific batch file.

## 4. Configure application.toml

| Section | Required engineering content |
|---|---|
| application | Name, positive sample_time_seconds, model_type="nonlinear_python" |
| controller | prediction_form="velocity", positive prediction/control horizons; control horizon no longer than prediction horizon |
| cvs | Unique id/name, unit, source_type="instrumented_process", initial value, target OR zone_low/zone_high, positive weight and normalization |
| mvs | Unique id/name, unit, initial, lower/upper, positive per-sample increase/decrease limits, move_weight and move_normalization |
| measured_disturbances | Unique id/name, unit and initial value; omit blocks if none |
| model | Relative Python file name, numerical parameters and domain |
| nonlinear_checks | Independent numerical transition checks; at least one is required |

IDs are lower-case letters/digits/underscores, starting with a letter, unique
across CVs, MVs and DVs. Declare CVs, MVs and DVs in the same order as the Python
ID tuples. Units are not automatically converted: consistency is your responsibility.

For a target CV provide `target`. For a zone CV provide both `zone_low` and
`zone_high` and remove `target`. Target events apply only to target-based CVs.
Optional `protected_low`/`protected_high` require positive
`protected_limit_weight` and `protected_limit_normalization`. These are soft
penalties, not inviolate safety limits. Conflicting objectives or insufficient
actuation can cause violations; report checks detect them afterwards.

CV error penalties are scaled by their normalization. For example, flow
normalization 5 m3/h and temperature normalization 2 degC put those error sizes
on comparable footing before applying weights. MV move penalties use
`move_weight*(move/move_normalization)^2`. Increase move weights to discourage
movement; this generally trades response speed for smoothness/robustness.

Horizon lengths are samples, not seconds. At dt=5 seconds, prediction horizon
30 covers 150 seconds and control horizon 10 covers 50 seconds. Do not shorten
them solely to obtain a faster solver without checking control performance.
`maximum_solver_cpu_seconds` is a solver setting, not a guaranteed end-to-end
cycle deadline. `use_measured_disturbances=false` is for an explicit comparison:
the controller then predicts with nominal DVs while the actual plant still
experiences every disturbance. Bad/out-of-domain DVs are still rejected.

## 5. Write model.py — the engineer's Python boundary

Edit only the engineering sections in your copy of `model_template.py`:

1. `STATE_IDS`, `MV_IDS`, `DV_IDS`: tuples matching the TOML IDs/order.
2. `validate_parameters(p)`: exact required parameter names and physical ranges.
3. `transition(x, u, d, p, dt)`: the process equations and next-state return.

The model loader executes Python. It is not a security sandbox. Model files must
be trusted, self-contained files below their configuration directory. Import
CasADi, but do not import application-local helper files: only the named file's
bytes are included in the model identity. Dependencies must be managed through
the package requirements, not installed during a control calculation.

Example of a single first-order output update:

```python
import casadi as ca

def transition(x, u, d, p, dt):
    equilibrium = p["gain"]*u["valve"] + d["inlet"]
    a = ca.exp(-dt/p["tau_seconds"])
    return {"outlet": a*x["outlet"] + (1-a)*equilibrium}
```

This snippet illustrates the calculation; it is not a complete replacement for
the two-output template. Return one value for **every** STATE_IDS key, with no
extra keys. Return absolute values at the next sample, **not increments, rates
or a derivative**. The core supplies velocity/incremental prediction internally.
Do not implement integral-action correction or subtract a previous prediction
inside the engineer's function.

The same function is called with ordinary numeric values and with symbolic
solver expressions. Use ordinary `+`, `-`, `*`, `/`, `**`, and CasADi functions
such as `ca.exp`, `ca.sqrt`, `ca.log`, `ca.sin`. Do not use `float()`, `math.exp`,
NumPy conversions or Python `if` comparisons on `x`, `u` or `d`. Python `if`
is appropriate in `validate_parameters`, where values are ordinary numbers.
Do not clip invalid expressions to hide a bad physical domain. Keep the function
deterministic and free of file access, networking, random numbers, mutable
history or calls to the optimiser. Fixed-length loops over model structure are
acceptable; decisions based on symbolic process values are not Python branches.

For first-order lags the exponential update is exact under held inputs. If an
external model supplies continuous derivatives instead, the engineer must supply
a suitable discrete transition and independently check integration accuracy.
This release does not choose a numerical integrator automatically.

## 6. Parameters, operating domain and initialisation

All `[model.parameters]` entries must be finite numbers; units belong in the
model/design documentation. Unknown/missing parameter names should be rejected
by `validate_parameters`. The completed exchanger model demonstrates this.

`[model.domain]` maps every CV/MV/DV ID to `[lower, upper]`. Use the actual range
over which the equations are justified, not unnecessarily wide bounds. MV
operating limits must lie inside their model domain. Pressure must stay above
downstream pressure in a square-root valve law, and denominators must not
approach zero. The exchanger's 35–65% valve domain explicitly excludes shutoff.

Validation checks finite values and derivatives at the nominal point, each axis
end point, and all box corners when there are at most eight variables. This is
a useful screen, **not a proof** over the whole domain. Runtime checks reject
incoming measurements/DVs outside the domain. Domain ranges are not extra hard
constraints on every predicted CV or every intermediate solver trial. Your
equations must remain numerically well behaved during prediction; models with
singular state expressions may need a different formulation. Do not widen a
domain just to suppress a legitimate validation failure.

Initial CV/MV/DV values must form a stationary point of both the controller
model and the configured plant. Nominal steady-state tolerance is 1e-8 in the
state engineering units. This avoids an unexplained startup transient. The
starter has no warm restart from arbitrary plant conditions. Stop and restart
both processes for a new local simulation.

## 7. Independent model checks and validate_and_report.py

Each `[[nonlinear_checks]]` contains:

```toml
[[nonlinear_checks]]
name = "Cold inlet: hand-calculated next sample"
state = [60, 64]
mvs = [50, 50]
dvs = [37, 5]
expected_next = [60, 63.78849442465227]
tolerance = 1e-8
```

Vector order is the Python/TOML order. Tolerance is an absolute tolerance applied
to each output in its own units; select it accordingly. Add checks for equilibrium,
each MV, each DV, and relevant combinations/operating points. Values must come
from a separate calculation or trusted external model, not be copied blindly
from the function you are checking. These are model implementation checks, not
open-loop identification experiments.

```bat
validate_application.bat applications\my_hx\application.toml applications\my_hx\simulation.toml applications\my_hx\opc.toml
```

The script loads and validates TOML, executes the trusted model file, checks
IDs/parameters/domains, builds symbolic outputs/Jacobians, verifies equilibrium
and numerical checks, validates the separate plant/scenario and OPC settings,
then builds the controller and solves one nominal move. It prints the model
SHA-256 and either PASS or a diagnostic FAIL; exit code is 0 or 1. Validation
does not connect to a live plant or prove stability, robustness or safety.

## 8. Configure simulation.toml

The simulator is an independent state instance with separately configured
parameters. A reference run may use the same Python equations as the controller;
this checks implementation consistency, not independent physical accuracy.
Use a separate `plant_model.file` where an independently developed compatible
plant model is available. Both model files follow the same ID/state contract.

| Section | Meaning |
|---|---|
| simulation | Positive sample count, maximum_solver_failures (normally zero) |
| plant_outputs | Plant signal id, matching controller cv, unit and initial; nominal_rate_per_minute must remain zero |
| plant_inputs | Plant signal id, role="mv" or "measured_dv", controller_id, unit and initial |
| plant_model | Python file, independent parameters and domain, same format as model |
| events | Zero-based sample, measured-DV plant input ID and absolute value |
| target_events | Zero-based sample, target-based controller CV ID and absolute target |
| checks | Signal, metric, limit and optional target/window |
| plots | Signals, ylabel including units, optional title |

Map every controller CV/MV/DV exactly once. Input units/initials must match the
application. Plant outputs must start at the same stationary CV values. All
dynamics belong in the nonlinear function: do not add linear `plant_paths` or
nominal drift on top of it.

Events are applied at the beginning of their sample, before the controller move.
The resulting next-sample measurements are recorded after the plant advance.
There is an initial record plus one per completed sample. The controller sees
current measured disturbances, not advance knowledge of the scenario schedule.

Check metrics: `minimum`, `maximum`, `max_abs_move`, `final_error`,
`max_abs_error`. Error checks also require `target`. Optional `start_sample` and
`end_sample` bound an **inclusive** window. `final_error` checks only the last
point in that window; `max_abs_error` checks all points. Use a settled window
to avoid accepting an oscillation merely because its last point crosses target.
Move checks compare adjacent records within the selected window; use the full
run to cover every move. Check every CV limit, MV bound and MV move limit, not
only terminal tracking error.

Plot/check signals are plant IDs, `cv.<controller_cv_id>` and, when requested,
`target.<controller_cv_id>`. Target traces are included when a scenario has
target events or explicitly requests those traces.

## 9. Run offline and inspect the result

```bat
run_simulation.bat applications\my_hx\application.toml applications\my_hx\simulation.toml
```

Optional third argument selects a separate results folder:

```bat
run_simulation.bat applications\my_hx\application.toml applications\my_hx\mismatch.toml applications\my_hx\results_mismatch
```

Default output is `results` beside the simulation TOML. It contains `results.csv`,
`results.png` and `report.json`. Reusing a results folder overwrites its files:
use separate destinations when comparing runs. The generic plotter auto-scales
axes; review the full transients, not only the printed PASS.

Exit 0 means all acceptance checks pass; 2 means the run completed but acceptance
failed; 1 indicates a configuration/runtime error. Solver failure in offline
development simulation holds the previous MV and counts a failure; the supplied
scenarios permit zero failures. Domain violations stop the run. Holding the
previous input is a simulation convention, not a certified plant fallback.

## 10. Run through OPC in separate windows

Use the **same application and OPC file** in both processes. The local protocol
uses OPC UA NoSecurity on loopback only. Do not change it to a remote/all-interface
address; this package is not a secure industrial OPC adapter.

Window 1, from the package root:

```bat
run_digital_twin.bat applications\my_hx\application.toml applications\my_hx\simulation.toml applications\my_hx\opc.toml
```

Wait for `Twin READY`. Window 2:

```bat
run_controller.bat applications\my_hx\application.toml applications\my_hx\opc.toml
```

Optional window 3:

```bat
opc_tools.bat applications\my_hx\opc.toml trend applications\my_hx\simulation.toml
opc_tools.bat applications\my_hx\opc.toml status
```

Run the trend command while the test is running; it begins collecting at connection,
does not load earlier history, and is for observation rather than complete logging.
Close its plot window to exit. The twin writes complete results under `opc_results`
beside the simulation file, including `live.csv`, final CSV/PNG/report and a
`current_run.json` marker. Check the run ID, terminal state and sample count so
you do not mistake an older report for the current run. Starting another run in
the same folder overwrites these outputs; preserve wanted results first.

The server stays available after completion for inspection. To stop:

```bat
opc_tools.bat applications\my_hx\opc.toml stop
```

Alternatively use Ctrl+C in the twin window. Stopping only the controller makes
the twin time out and freeze. Explicitly restart both processes afterwards;
there is no automatic reconnect or transfer to another controller.

## 11. OPC settings, messages and failure behaviour

The generic OPC file defines endpoint, namespace URI, five node identifiers,
poll/request/startup/command/stale timeouts and speed. The nonlinear example
uses localhost port 4842 to distinguish it from the linear examples. Both
processes must use exactly the same values. `speed=20` requests twenty simulated
seconds per wall second; `speed=1` requests normal pace. Slow solving extends
the run. Neither setting creates a real-time deadline guarantee.

The stale timeout must exceed `sample_time/speed + command_timeout`. Avoid
reducing timeouts to values that reject normal solves on your computer. Changing
the speed requires re-validating this relationship.

| Node | Content / ownership |
|---|---|
| MPC.Sample | Atomic JSON frame: run ID, sample, application/model fingerprint, CVs, DVs, applied MVs and current targets; twin writes |
| MPC.Command | Matching run/sample/fingerprint, controller owner, solve status and MV command; controller writes |
| Twin.Status | Startup, running, saving, completed, stopped or fault status |
| Twin.Telemetry | Read-only observer values for trends, not an extra controller measurement source |
| Twin.Stop | Operator stop request |

Application identity includes all application TOML content and the bytes of the
controller model Python file. It excludes the file's absolute directory so
copying the same application to a new folder does not change its identity.
Editing model code or parameters requires stopping and restarting both sides.
Do not edit files during a run. Targets change only through configured scenario
events in this release; arbitrary online tuning sliders are not included.

The twin advances exactly once per accepted command. Stale, mismatched, invalid,
non-finite or excessive commands, loss of communication and controller faults
stop advancement. This is a **supervised, synchronised simulation**, not a
free-running physical plant: the real world cannot freeze on communications loss.
Its fault behaviour must not be presented as an industrial safety strategy.

## 12. Prediction, tuning and robustness

The core evaluates the nonlinear transition throughout the horizon. It uses
successive differences of model transitions, anchored to measured states, for
velocity-form prediction. At a stationary measured point with unchanged inputs
and DVs, equal model transitions cancel. This provides the feedback/integral-action
structure; it is not a blanket guarantee of zero error under saturation,
unmeasured dynamics, noise or every kind of model mismatch. Check your cases.

Begin with conservative positive move weights and sensible unit scales. Verify
stationary startup, individual MV/DV signs, target changes and constraints.
Then change **plant parameters only** to test mismatch. Do not silently alter
the controller model too, or the mismatch test disappears. Compare the same
scenarios and the same acceptance limits. Tune deliberately and retain failures
that reveal limitations rather than loosening checks until they pass.

The generic `compare_results.py` compares fixed-target runs with identical time
vectors. It produces an overlay plot and integral/peak/final absolute-error
metrics. Its `--signal ID TARGET` assumes a constant target; do not use it to
claim tracking metrics for changing-target scenarios.

## 13. Heat-exchanger engineering reference

Read `examples/01_heat_exchanger/DESIGN_BASIS.md` for equations and assumptions,
then compare your four engineered files against the supplied reference. The
reference README gives exact commands for nominal, mismatch, feedforward-on,
feedforward-off and OPC runs. `verified` and `verified_opc` contain saved results.

No runtime imports anything from the examples folder. For a clean engineering
exercise, copy only the generic templates into your own application folder,
enter the design-basis values/equations, and validate. Use the complete solution
as a diagnostic reference when needed—not as a hidden prerequisite.

## 14. Troubleshooting and acceptance before a pause/release

| Symptom | Check first |
|---|---|
| Python not found / missing package | Python installation, run setup_windows.bat, verify the package's .venv |
| TOML cannot overwrite value | Duplicate key or wrongly placed table header; each repeated variable uses double brackets |
| ID mismatch | Exact spelling and order in TOML and Python tuples |
| Non-finite expression/derivative | Pressure differential, denominator, log/sqrt domain, parameter units |
| Symbolic conversion/boolean error | float(), math/NumPy operations or Python if applied to symbolic values |
| Initial point not stationary | Physical equilibrium, units and independently configured plant parameters |
| Model check fails | Absolute next-state vs rate/increment; dt units; independently calculated expected numbers |
| OPC cannot connect | Twin READY, same endpoint/port, another process already using the port |
| Application fingerprint differs | Same TOML and model source in both processes; restart after edits |
| Solver succeeds but controls poorly | Scaling, constraints, horizon, model mismatch and move weights; numerical success is not engineering success |
| Test PASS but oscillation visible | Require max_abs_error over a settled window, not only final_error |

Before relying on your engineered application: retain its design basis, all
four application files, pinned package/version, numerical checks, scenario
criteria and plotted reference results. Run the automated tests and full local
OPC comparison described in VERIFICATION.md. Test Windows installation/launchers
and your desktop trend window locally. Keep this development release separate
from any approved plant controller. Live-plant commissioning remains a distinct,
site-reviewed project.
