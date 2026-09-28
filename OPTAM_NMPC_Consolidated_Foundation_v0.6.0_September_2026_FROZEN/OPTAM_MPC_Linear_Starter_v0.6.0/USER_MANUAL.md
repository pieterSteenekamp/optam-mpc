# OPTAM-MPC Linear Application Starter — User manual

## 1. Scope

This manual is a configuration and operating reference. It assumes that the
APC engineer can define the control problem and supply a working process model.
It does not teach MPC theory, process modelling or Python programming.

This development version supports:

- configurable numbers of primary CVs, derived rate CVs, MVs and measured DVs;
- linear superposition of configured model paths;
- integrator, first-order lag and static-gain paths with sampled dead time;
- velocity-form prediction;
- CV targets or zones;
- strongly protected predicted CV operating limits;
- MV position and asymmetric move limits;
- model implementation verification; and
- generic configuration-driven linear plant simulation, events, plots and checks.

It also supports a local OPC UA digital-twin server, separate controller client,
read-only live trends, status inspection and controlled test shutdown.
This is a supervised simulated-process workflow, not a live-plant OPC driver.

Dead time must be a whole multiple of the sample interval; fractional delays
are rejected rather than silently rounded.

## 2. Installation

Python 3.11 or later is required. Extract the package into a writable folder
and run once:

```bat
setup_windows.bat
```

This creates `.venv` and installs the pinned runtime dependencies.

Use an open Windows Command Prompt for all commands in this manual. The batch
files return an exit code without pausing. Double-clicking them can hide errors
when the window closes. Internet access is needed for dependency installation.
No OPC server or additional modelling package is needed for offline simulation.

## 3. Starting a new application

Create a new application folder and copy the three generic templates into it.
For example, from the extracted package root:

```bat
mkdir applications\my_surge
copy application_template.toml applications\my_surge\application.toml
copy simulation_template.toml applications\my_surge\simulation.toml
copy opc_template.toml applications\my_surge\opc.toml
```

The APC engineer edits these three TOML files in a plain-text editor. The OPC
file is needed only for the two-process OPC run. No Python
file or batch file is created or edited. The shared runtime remains in the
package root; the application folder contains engineering configuration only.

The template illustrates two CVs, two MVs and two measured DVs. These counts
are examples only:

- delete blocks that are not required;
- copy complete `[[...]]` blocks when more variables or paths are required;
- give every variable a unique stable `id`;
- replace all placeholder values and comments marked `REPLACE`.

The completed files under `examples/01_surge_vessel/` show one reference result.
They are not loaded by the generic runtime unless explicitly named on the
command line. They can be removed from a working installation without affecting
your own application.

## 4. TOML rules used by OPTAM-MPC

- `[section]` begins a single table.
- `[[section]]` begins one item in a repeated list.
- Text values use quotation marks.
- Numbers do not use quotation marks.
- `#` begins a comment.
- Identifiers begin with a lower-case letter and contain only lower-case
  letters, digits and underscores.

Variable `id` values are used in model paths and should remain stable. Display
`name` values may contain spaces and may be changed without altering paths.
CV display names must be unique across primary and derived CVs; MV display
names must be unique among MVs.

## 5. Application and controller settings

### `[application]`

| Field | Meaning |
|---|---|
| `name` | Application display name |
| `sample_time_seconds` | Controller execution interval |
| `model_type` | Currently `linear_paths` |

### `[controller]`

| Field | Meaning |
|---|---|
| `prediction_form` | Use `velocity` for the current feedback structure |
| `prediction_horizon_samples` | Number of future samples predicted |
| `control_horizon_samples` | Number of independently optimised future moves |
| `maximum_solver_cpu_seconds` | Per-cycle solver limit |

Horizon values must be positive whole numbers. Their duration equals the sample
count multiplied by the sample time. Control horizon must not exceed prediction
horizon. Optional `maximum_solver_iterations` is a positive integer (default 300).
All objective weights and normalizations in this version must be strictly
positive; zero is not a supported way to disable an objective.

## 6. Primary controlled variables

Each `[[cvs]]` block defines one instrumented process CV.

| Field | Meaning |
|---|---|
| `id`, `name`, `unit` | Identity and engineering presentation |
| `source_type` | `instrumented_process` |
| `initial` | Nominal initial measurement |
| `target` | Preferred value |
| `zone_low`, `zone_high` | Alternative permitted zone; use instead of `target` |
| `weight` | Relative target or zone priority |
| `normalization` | Meaningful error magnitude used for scaling |
| `protected_low`, `protected_high` | Strongly protected predicted operating limits |
| `protected_limit_weight` | Violation penalty |
| `protected_limit_normalization` | Meaningful violation magnitude |
| `display` | Operator and engineering display role |

Protected operating limits are soft optimisation constraints. They are not
safety trips and do not replace alarms, interlocks or basic regulatory control.

## 7. Derived auxiliary CVs

A `[[derived_cvs]]` block can define a filtered rate of change of a primary CV.

Required fields include:

- `calculation = "rate_of_change"`;
- `source_cv`, referring to a primary CV identifier;
- `filter_alpha`, greater than zero and no greater than one;
- `target`, normally zero;
- `weight` and `normalization`.

OPTAM-MPC calculates the measurement and predicted trajectory. No additional
plant instrument or user-written Python preprocessor is required.

Current measurement convention: the source primary CV is exponentially
filtered, and its filtered value is also passed to the controller as primary
feedback. The derived rate is the difference between consecutive filtered
values divided by the sample interval in minutes. Raw simulated measurements
remain available for acceptance checks. The first rate is zero; configure
`initial = 0.0`. Only one derived-rate CV per source primary CV is supported.
The prediction model returns the nominal model rate, not a separate simulated
filter state. This retains the tested velocity-form feedback arrangement.

## 8. Manipulated variables

Each `[[mvs]]` block defines one MV.

| Field | Meaning |
|---|---|
| `initial` | Nominal initial MV value |
| `lower`, `upper` | MV operating range |
| `maximum_increase_per_sample` | Positive move limit |
| `maximum_decrease_per_sample` | Magnitude of negative move limit |
| `move_weight` | Penalty on MV movement |
| `move_normalization` | Meaningful move magnitude |

The initial value must lie within the configured range.

## 9. Measured disturbances

Each `[[measured_disturbances]]` block defines a measured DV with `id`, `name`,
`unit` and `initial`.

A measured DV affects a CV only through a configured model path. Do not declare
an unmeasured disturbance as a measured DV merely because the digital twin or
plant model knows its value.

## 10. Linear model paths

Each `[[model_paths]]` block connects one MV or measured DV to one primary CV.
Multiple paths affecting the same CV are added by linear superposition.

An integrating path is:

```toml
[[model_paths]]
source = "mv_id"
destination = "cv_id"
type = "integrator"
integrating_gain = -0.0041666667
dead_time_seconds = 0.0
engineering_basis = "Source of model or calculation"
```

The integrator is defined as:

$$
\frac{d\Delta y}{dt}=K_I\Delta u
$$

`integrating_gain` is entered in destination units per minute per source unit.
The explicit per-minute convention avoids hidden time-base conversions.

For a non-integrating first-order path use:

```toml
[[model_paths]]
source = "mv_id"
destination = "cv_id"
type = "first_order"
gain = 0.6
time_constant_seconds = 10.0
dead_time_seconds = 5.0
engineering_basis = "Source of local gain and dynamic assumptions"
```

Here `gain` is steady output change per input unit. Unlike integrating gain,
it has no per-minute factor. The form is G(s)=K exp(-theta s)/(tau s+1).
A static path uses `type = "gain"`, `gain` and `dead_time_seconds`; omit tau.
Delete `integrating_gain` when changing a path to a non-integrating type.

First-order paths use exact zero-order-hold discretisation:
z_next = a*z + (1-a)*K*delayed_input_deviation, a=exp(-Ts/tau).
Each path starts at zero deviation; pre-start input histories are at nominal
initial values. Software creates and maintains the path and delay states from
applied MVs and measured DV history. They are not additional instrumented CVs.

At an event on sample k, zero-delay paths affect the next recorded point
(k+1). A dead time of m sample intervals suppresses the first m post-event
points. For all types, dead time must be non-negative and an integer multiple
of `sample_time_seconds`. Time constants are positive seconds.
Paths may have different time constants and delays. Missing paths mean zero
effect; variable count does not require a fully populated gain matrix.

## 11. Model implementation checks

Optional repeated `[[model_checks]]` blocks in `application.toml` contain
`source`, `destination`, `step`, `expected_rate_per_minute` and positive
`tolerance`. Each check changes one controller source from its nominal initial
value, sums all paths into the named destination, and compares its calculated
rate with your independently supplied expected rate. Both positive and negative
steps are useful for checking signs and units.

These checks verify implementation of a supplied model. They are not plant
step testing, model identification or proof that the model represents reality.
An omitted check is reported as NOT CONFIGURED, not as a verified response.

For non-integrating paths use `metric = "steady_change"` with `step`,
`expected_change` and `tolerance`. This checks the sum of the relevant steady
gains times the step. For a time-specific response use
`metric = "step_response"` and add `time_seconds` and `expected_change`.
The analytical path response is compared with the independently supplied
expected change. The default `metric = "rate"` remains for zero-delay
integrators with `expected_rate_per_minute`.

The automated regression suite separately tests discrete trajectories against
analytical responses. A steady-gain check alone does not verify a time constant
or dead time.

## 12. Independent simulated plant: simulation.toml

Copy `simulation_template.toml`; replace or delete its illustrative blocks.
This file defines the test plant independently of the controller model.
It can have different gains and additional unmeasured inputs. No automatic
copying of the controller paths takes place.

| Section | Fields and meaning |
|---|---|
| `[simulation]` | `samples`: positive run length; `maximum_solver_failures`: non-negative allowed count, normally zero |
| `[[plant_outputs]]` | Unique plant `id`; `cv`: mapped controller primary CV ID; `unit`; `initial`; `nominal_rate_per_minute`: drift at nominal inputs |
| `[[plant_inputs]]` | Unique plant `id`; `role`; `unit`; `initial`; `controller_id` only for an MV or measured DV |
| `[[plant_paths]]` | Plant source/destination IDs and the same path fields/types defined in section 10 |
| `[[events]]` | `sample`, plant `input` ID and new absolute `value` |
| `[[target_events]]` | `sample`, controller primary `cv` ID and new target `value` |
| `[[checks]]` | Recorded `signal`, `metric`, `limit`, plus `target` for final-error checks |
| `[[plots]]` | `signals`: list of recorded signal IDs; `ylabel`: label including units; optional `title` |

Input roles are:

- `mv`: driven by a controller MV; `controller_id` is required.
- `measured_dv`: simulated externally and passed to the named controller DV.
- `hidden`: simulated externally but never passed as a measured controller DV;
  omit `controller_id` and do not add it to `measured_disturbances`.

Every controller primary CV, MV and measured DV must have exactly one plant
mapping. Mapped units must match; there is no automatic unit conversion. Mapped
MV/DV initial values must equal their controller nominal values. Output initial
values may differ from controller nominal values to test an initial offset.

For integrating contributions the plant uses:

$$
y_{k+1}=y_k+\frac{T_s}{60}\left[r_0+\sum_j K_{I,j}(u_{j,k}-u_{j,0})\right]
$$

`T_s` is the application sample time in seconds. Gains use output units/minute
per input unit. `r_0` is `nominal_rate_per_minute`. A zero nominal drift assumes
the initial flows are balanced; it is not automatically calculated from
absolute flow values. The APC engineer supplies the gains and nominal drift.
For a non-steady initial condition, explicitly calculate the correct drift.

First-order and gain contributions are added as deviations from the initial
output using the path dynamics in section 10. Set nominal drift to zero for
the non-integrating heat-exchanger example. Dynamics such as actuator lag can
be represented by the supplied path lags, but there is no separate nonlinear
actuator, clipping, noise or physical-boundary model. Do not treat this
simulator as a physical safety system.

## 13. Scenario, records and acceptance

An event changes an external input at the start of a zero-based sample.
At sample `k`: apply events, read measured DVs, solve the controller, apply MVs,
advance the plant one interval, update measurements and record the result.
An event at sample 30 with a 5-second interval starts at 150 seconds.
Its first post-step record is at 155 seconds. Values are held until another
event changes them. Events may not override MVs. Future DV events are not
provided to the controller as a preview trajectory.

Target events update target-based primary CVs at the start of the specified
sample, without resetting history or the warm start. Zone CVs cannot use these
events. New targets are held for prediction; future target events are not
previewed. The same target schedule is carried over OPC.

Records contain the initial point and every completed interval. Signal names:

- Plant input/output IDs: actual simulated values.
- `cv.<controller_cv_id>`: primary feedback (filtered if a rate CV uses it).
- `derived.<derived_cv_id>`: derived feedback rate.
- `target.<cv_id>`: active target, recorded when events, plots or checks request it.

Use the raw plant output signal for physical operating-limit checks.

| Check metric | Pass condition |
|---|---|
| `minimum` | All recorded values >= `limit` |
| `maximum` | All recorded values <= `limit` |
| `max_abs_move` | Largest absolute consecutive change <= `limit`, including the first move from the initial value |
| `final_error` | Absolute final value minus `target` <= `limit` |
| `max_abs_error` | Largest absolute error from `target` over the check window <= `limit` |

Optional `start_sample` and `end_sample` select an inclusive recorded-index
window for any check. Record 0 is the initial state. A windowed final-error
check uses the last record in that window. Use max_abs_error over a settling
window to detect oscillation that a single final-value check can miss.

At least one acceptance check and plot panel are required. Add checks for every
important CV and MV; the generic template's illustrative checks are not a
complete engineering acceptance specification. Group only compatible units in
one plot panel. Plots auto-scale and do not automatically draw limit or target
lines. To display a changing target, include its `target.<cv_id>` signal.
Use the numerical report to judge limits, not the plot's axis range.

PASS means only that the specified finite scenario passed its declared checks.
It does not prove limit protection for every disturbance or establish optimal
tuning. The MPC minimises a weighted objective: it does not independently
guarantee the smallest physically possible outlet-flow movement.

## 14. Validation and execution

From the package root, run:

```bat
validate_application.bat applications\my_surge\application.toml applications\my_surge\simulation.toml
```

Validation checks the implemented configuration rules, mappings and declared
model responses. It does not check the physical correctness of supplied gains
or guarantee controllability. Correct reported errors before execution.
Omit the second argument to validate only the controller definition.

Examples of errors are:

```text
MV mv_1: lower limit must be below upper limit.
Derived CV cv_1_rate: unknown source_cv cv_3.
Model path mv_1 -> cv_1: integrating_gain must be a number.
```

Run your own application with the unchanged generic launcher:

```bat
run_simulation.bat applications\my_surge\application.toml applications\my_surge\simulation.toml
```

Paths containing spaces must be enclosed in double quotes. Relative arguments
are resolved from the current Command Prompt folder, not the package folder.
The launchers locate the shared runtime relative to their own location, so
absolute application paths outside the package work too.

Results are written beside the selected simulation file:

```text
applications\my_surge\results\results.png
applications\my_surge\results\results.csv
applications\my_surge\results\report.json
```

Repeating a run replaces these three result files. Copy the application folder
or rename its results folder before running if an earlier result must be kept.
JSON includes numerical check results and solver failure messages.

An optional third argument to `run_simulation.bat` chooses a separate results
folder, useful for comparisons. The Python equivalent is `--output FOLDER`.

Expected final console line:
```text
Closed-loop acceptance: PASS
```

Exit codes: 0 = pass; 1 = configuration/file error; 2 = failed closed-loop
acceptance. Unexpected runtime/programming errors stop the run and return a
nonzero code. On a solver failure the simulation holds the last MV command and
records the failure. This is a development fallback, not a plant deployment
policy. A run uses simulated time and is not real-time control.

For non-Windows operation, use Python 3.11+ with `requirements.txt` installed:

```text
python validate_and_report.py path/to/application.toml path/to/simulation.toml
python run_simulation.py path/to/application.toml path/to/simulation.toml
```

## 15. Surge-vessel reference and engineering responsibility

The completed example contains three TOML configuration files, explanatory
notes and reference results. It has no dedicated Python model or launcher.
Its README states the dimensions, gain calculations, disturbance and pass
criteria so you can compare your independently edited files with the reference.

For your clean-start exercise, use only the generic templates and this manual
to create your own files. Consult the completed reference if you need to check
an engineering value or the resulting TOML structure. This is configuration
work, not a requirement to reverse-engineer Python.

The engineer supplies the variable list, working model, limits, tuning, initial
conditions and simulation acceptance criteria. The package supplies parsing,
model construction, controller, simulation, reporting and generic commands.

## 16. Template, example and tutorial

- Use the three generic TOML templates as the neutral structural starting point.
- Use `examples/` to inspect completed, runnable applications.
- Use this manual for field definitions and operating commands.
- A full teaching tutorial is deferred. This reference manual does not require
  one to operate the supported workflow.

## 17. Troubleshooting and software checks

- Python not found: install Python 3.11+ and ensure `python --version` works
  in Command Prompt, then run setup again.
- Missing module or virtual environment: run `setup_windows.bat` successfully
  before validation or simulation.
- File not found: check both path arguments and quote paths with spaces.
- TOML error: check table brackets, quoted text and duplicate field definitions.
- Unknown mapping: use controller IDs in `cv` / `controller_id` and plant IDs
  in plant paths and events.
- Legacy configuration rejected: this version moves plant/scenario/checks to
  `simulation.toml` and uses `[[model_checks]]` instead of `[verification]`.
- Failed acceptance: inspect the report and trends, then review the engineering
  model, feasibility, tuning and scenario. Do not relax a limit just to get PASS.

Package regression tests are a developer check, not a per-application setup step:

```bat
.venv\Scripts\python -m unittest discover -s tests -v
```

The tests use reference fixtures; the runtime and your own application do not.

## 18. Current development limitations

- Only linear integrator, first-order and gain paths are implemented.
- Fractional dead times and nonlinear user equations are not yet implemented.
- Only the stated linear plant and measurement forms are supported.
- OPC uses the supplied local twin protocol, not arbitrary external OPC tags.
- Individual MV move weights are not yet exposed for online tuning.
- This is an engineering development package, not a certified plant control or
  safety system.

## 19. OPC operating arrangement

The digital twin and OPC UA server run together in one Command Prompt window.
The controller runs in another. An optional third window provides live trends
or status commands. All three use the same local endpoint configuration.

The twin owns simulated time. For each sample it applies scenario events,
publishes a coherent measurement snapshot, waits for a validated controller
command, applies the complete MV vector and advances the plant one interval.
The controller receives only mapped CVs, measured DVs and previously applied
MVs. Hidden disturbances remain absent from its sample.

This synchronised arrangement makes OPC/offline comparisons reproducible.
It is not a free-running real plant: on a controller or communication fault,
the simulation freezes rather than continuing to integrate at held outputs.
This deliberate test behaviour must never be interpreted as a plant fallback.

| File | Owner of engineering content | Used by |
|---|---|---|
| `application.toml` | APC engineer: model, variables, tuning and limits | Twin for checks; controller for control |
| `simulation.toml` | APC engineer: independent plant, mapping, events, acceptance and plots | Twin; optional trend viewer |
| `opc.toml` | APC engineer: local endpoint, node identifiers and timing | Twin, controller and monitoring tools |

Both processes must use identical application configuration contents. The
software checks a fingerprint; different tuning or model files are rejected.
Comments and file paths do not affect the fingerprint. Do not edit files during
a run; stop and restart both processes after changes.

## 20. OPC configuration and mapping

Copy `opc_template.toml` to your application folder as `opc.toml`.
Its defaults are usable for any supported application on this computer.
If another program uses port 4840, choose a different unused port in this file.
Do not open firewall ports for this demonstration.

| Field in `[opc]` | Meaning |
|---|---|
| `endpoint` | Local OPC UA URL, e.g. `opc.tcp://127.0.0.1:4840/optam/linear/`; loopback is enforced |
| `namespace_uri` | Stable namespace identity; index is discovered, not hard-coded |
| `sample_node` | String NodeId for the atomic controller measurement frame |
| `command_node` | String NodeId for the atomic controller MV response |
| `status_node` | String NodeId for twin state and result |
| `telemetry_node` | String NodeId for read-only observer data, including hidden inputs |
| `stop_node` | Boolean NodeId; write true to request test stop |
| `poll_seconds` | Client/server polling interval |
| `request_timeout_seconds` | OPC request timeout |
| `startup_timeout_seconds` | Time allowed for the first controller command after Twin READY |
| `command_timeout_seconds` | Time allowed for subsequent commands |
| `stale_timeout_seconds` | Controller limit on waiting for a new sample; also checks new-frame timestamp age |
| `speed` | Requested simulation-time/wall-time ratio; 1 is nominal real-time pacing, 20 is accelerated |

Speed changes pacing, not the model sample interval. Solver and communication
time may make a run slower than requested. The stale timeout must exceed
`sample_time_seconds / speed + command_timeout_seconds`. Increase the timeouts
if legitimately necessary for a slower computer; do not use them to hide
unexplained hangs. All timeouts are wall-clock seconds.

This package uses five OPC nodes with structured JSON text frames, except the
Boolean stop node. Values are keyed by the configured controller IDs. This
avoids mixing measurements from different samples or partially applying a
multi-MV command. The frames are constructed by the software; the engineer
does not write JSON or Python.

The plant-to-controller mappings remain in `simulation.toml`: output `cv`
and input `controller_id` fields. There is no second mapping to maintain.
For the vessel reference:

| Plant signal | Role | Controller signal |
|---|---|---|
| `vessel_level` | Measured CV | `level` |
| `outlet` | MV applied by twin | `outlet_flow` |
| `inlet` | Hidden disturbance | None |
| No plant signal | Internal derived CV | `level_rate`, calculated inside controller |

These OPC nodes are not individually writable level/flow tags. Connecting an
existing plant server with arbitrary scalar tags needs a separate future
adapter. Merely changing this endpoint cannot provide that functionality.
Namespace URI and string NodeIds can be browsed with an OPC client, but no
external client is required to engineer or operate this example.

Security is intentionally local-only: anonymous, unencrypted OPC UA on
127.0.0.1, with no certificate setup. Other local processes can access it.
Hidden-input exclusion is a controller software rule, not an access-control
boundary. Never use this configuration for a plant or expose it on a network.

## 21. Validation and startup

Use separate open Command Prompt windows. In each, first change directory to
the extracted package folder. Use `cd /d C:\your\package\folder` when necessary.

Validate all three files:

```bat
validate_application.bat applications\my_surge\application.toml applications\my_surge\simulation.toml applications\my_surge\opc.toml
```

Check the printed CV and input mappings. In particular, confirm that the wild
inlet is reported as HIDDEN FROM CONTROLLER. Validation is a local configuration
check, not an OPC connectivity test.

First run the offline simulation using section 14. Once it passes, in window 1:

```bat
run_digital_twin.bat applications\my_surge\application.toml applications\my_surge\simulation.toml applications\my_surge\opc.toml
```

Wait for `Twin READY`. The initial state is held while waiting. Within the
startup timeout (120 seconds by default), run in window 2:

```bat
run_controller.bat applications\my_surge\application.toml applications\my_surge\opc.toml
```

Start only one controller. Console progress appears every 20 samples.
At default vessel settings, 1050 samples represent 87.5 simulated minutes;
20x pacing means at least 4.375 wall minutes plus startup/reporting overhead.
Slow solving or communication can extend this time.

On normal completion the controller exits. The twin writes its results and
remains available for inspection until stopped. A completed run can still FAIL
its engineering acceptance checks; look for `OPC closed-loop acceptance: PASS`.

## 22. Trends, status and output files

Optional live trends in a third Command Prompt:

```bat
opc_tools.bat applications\my_surge\opc.toml trend applications\my_surge\simulation.toml
```

The plot panels come from your `[[plots]]` configuration. The observer can show
the hidden inlet without feeding it to the controller. Axes auto-scale.
Closing the trend window does not stop the twin or controller. Live display
starts when connected and can skip samples at high speed; it is not the
authoritative record. It retains up to 5000 observed points.

Read the current state and latest plant values:

```bat
opc_tools.bat applications\my_surge\opc.toml status
```

The twin writes `opc_results` beside `simulation.toml`:

- `live.csv`: flushed after each accepted plant step, useful if a process dies.
- `results.csv`: complete or partial recorded trajectory at orderly termination.
- `results.png`: configured plots of that trajectory.
- `report.json`: run identifier, terminal state, reason, sample count and checks.
- `current_run.json`: identity of the latest attempted run, written at startup.

Offline results remain separately in `results`. Each mode replaces its own
output files on subsequent runs. Save a copy before restarting if you need
comparison evidence. A force-killed process may not create a final report;
`live.csv` and console errors then describe the interrupted run. Always check
the report's run ID (it must match `current_run.json`), sample count and terminal
state, not merely an old PASS.

## 23. Shutdown, faults and recovery

For an orderly stop, in another Command Prompt:

```bat
opc_tools.bat applications\my_surge\opc.toml stop
```

Or press Ctrl+C in the twin window. Early termination is recorded as an
incomplete/failed test, not a successful acceptance run.
If stopping causes the server to close before the controller reads its final
status, the controller may report connection loss; this is expected during stop.

| Condition | Behaviour |
|---|---|
| No controller starts | Startup timeout; no plant steps; failed test |
| Controller stopped/disconnected | Twin waits up to command timeout, holds the last accepted state/MVs, then faults |
| Twin stopped/disconnected | Controller stops on OPC request failure or stale data; no automatic reconnect |
| Bad OPC quality, stale or non-finite measurement | Controller rejects it; no new command; twin subsequently times out |
| Solver failure | Controller reports a fault and exits; twin rejects the response and freezes |
| Wrong application, old run/sample or invalid MV command | Rejected; no invalid move applied |
| Duplicate controller | Unsupported; owner/sequence checks can fault the test; stop and restart with one client |
| Acceptance limit exceeded | Report FAIL; physical operating-limit checks remain separate from soft MPC penalties |

OPC mode treats every solver/communication fault as a failed test, even if the
offline `maximum_solver_failures` setting would allow a held move.
It never resumes partway through a run or automatically replays old commands.
After a fault: stop both processes, preserve evidence if needed, correct the
cause, then restart the twin and controller from the configured initial state.
Filtering and velocity-model history are reset on the new controller process.

Common troubleshooting:

- Connection refused: start the twin first and wait for READY; check the port
  and that both processes use the same `opc.toml`.
- Port already in use: stop your previous twin or select an unused local port.
- Unknown namespace/node: confirm identical OPC configuration on both sides.
- Application mismatch: use the same `application.toml` in both commands.
- Startup timeout: start the controller sooner, or increase startup timeout.
- Command timeout: inspect controller errors and solver timing before changing
  the limit. The simulation has stopped; it is not silently running open loop.
- OPC security warnings: signing/encryption are intentionally absent for this
  loopback demo. Do not enter credentials or expose the endpoint remotely.
- Session-timeout negotiation warning: the server may negotiate a shorter OPC
  session lifetime; this differs from the configured per-sample timeout.
- Plot fails to open: use the final PNG or status command; check that Python
  has a working desktop Matplotlib backend.

## 24. Independent engineering acceptance and reference comparison

Your clean-start exercise is complete when you can:

1. Create the application folder from the three generic templates.
2. Enter the engineering values and validate all mappings without changing code.
3. Pass the offline vessel scenario.
4. Start the same application over OPC in two separate windows.
5. Inspect live status/trends and obtain a complete OPC PASS report.
6. Stop and restart predictably; recognise an incomplete or failed test.
7. Explain any difference from the supplied reference using configuration,
   model, tuning or scenario differences.

The completed solution is under `examples/01_surge_vessel`. It uses the same
templates' schema and generic commands as your own folder. Its README contains
the full engineering data; its three TOML files and reference results show the
finished solution. There is no example-specific launcher or Python model.

To investigate a discrepancy, compare in this order: variable IDs and mappings,
units and sample time, gain signs and magnitudes, nominal values/drift, derived
filter settings, limits and weights, scenario events, then acceptance checks.
Do not copy a reference value blindly if your physical vessel differs.

Record any place where the manual makes you guess, any confusing error message,
and any undocumented action you had to take. Those are package usability
findings—not automatically engineering mistakes. The full tutorial remains a
separate future deliverable.

## 25. Engineering the 2x2 heat-exchanger example

Use the same three generic templates and tools. Start a new application folder;
retain two primary CVs, two MVs and two measured DVs, but no derived CV is needed.

| Category | Controller IDs and meaning |
|---|---|
| CVs | `flow`: combined outlet flow; `temperature`: mixed outlet temperature |
| MVs | `hx_valve`, `bypass_valve`: the two branch valve positions |
| Measured DVs | `inlet_temperature`, `upstream_pressure`: measured before the split |

Read `examples/02_heat_exchanger/DESIGN_BASIS.md` for the assumed operating
point, gain derivation, seven nonzero model paths and dynamic assumptions.
The completed TOML files provide all limits, normalizations, weights and
simulation mappings. They are reference results, not required runtime code.

Change the template's illustrative integrator paths to first_order paths,
using `gain`, `time_constant_seconds` and `dead_time_seconds`. Duplicate
blocks for additional cross-effects. Do the same independently for plant
paths in simulation.toml. Preserve zero nominal plant drift. Configure the
setpoint and disturbance schedule, then the checks and plot panels.

The inlet-temperature-to-flow path is deliberately absent. Both valves affect
both CVs; upstream pressure affects both outputs. Upstream pressure can serve
as a flow disturbance here only because downstream pressure is assumed fixed.
For a plant with varying backpressure, reconsider the model and measurements.

Validate, run offline, then start the twin and controller using sections 14
and 21 with your new file paths. There is no new heat-exchanger launcher.
The reference OPC file uses port 4841 to distinguish it from the vessel's
4840; your own endpoint can use either unused local port.

## 26. Feedforward, mismatch and reference comparisons

`[controller].use_measured_disturbances` defaults to true. Setting it false
holds the controller's DVs at their nominal values; plant disturbances still
occur and remain visible to the observer. This is a controlled feedback-only
comparison, not removal of the actual disturbance. Change this setting only
between runs and use the same application file in both OPC processes.

The exchanger reference supplies:

- `application.toml`: nominal-model tuning, valve move weights 0.05.
- `application_no_feedforward.toml`: identical except feedforward disabled.
- `application_detuned.toml`: move weights 0.5 for the mismatch challenge.
- `simulation.toml`: combined setpoint and measured-disturbance test.
- `disturbances.toml`: fixed-target feedforward comparison.
- `mismatch.toml`: same disturbances with larger plant gains/lags/delays.

The more aggressive nominal tuning is not claimed to be robust to that
mismatch. Its mismatch run deliberately fails the settled-window check;
the detuned configuration is the supplied passing solution for that case.
This exposes an important engineering tradeoff rather than hiding the failure.

Use separate results folders. `compare_results.py` can overlay two CSV runs
and calculate peak error and integral absolute error (IAE). It requires matching
recorded times. Use it only with comparable scenarios and fixed targets.
IAE uses the trapezoidal integral in engineering-unit minutes; lower is better
for that stated test, not proof of general superiority.

The exchanger README contains exact commands for every reference case and the
comparison. Its `verified/` and `verified_opc/` folders contain result
plots/reports/CSV data, including the explicitly labelled failed mismatch case.
Use those to investigate configuration differences, and inspect trends as well
as numerical PASS/FAIL. This example does not validate a nonlinear exchanger
over its full operating range or replace plant commissioning.
