# Example 01 — single surge vessel

This is a completed configuration-only linear MPC application.

- Primary CV: vessel level
- Derived auxiliary CV: filtered level rate, target zero
- MV: outlet-flow setpoint
- Hidden disturbance: wild inlet flow
- Model: first-principles outlet-flow-to-level integrator

The vessel volume represented by one percentage point is 4.0 m3/%. Therefore:

$$
K_I=-\frac{1}{60\times4}=-0.0041666667
$$

in (%/min) per (m3/h). Wild inlet flow is present only in the digital twin and
is not a controller DV.

For a constant-cross-section vessel, 4 m3/% corresponds to 400 m3 over the
0–100% level range. Obtain usable volume versus indicated level from the actual
geometry/calibration; this linear model assumes the constant-area relationship.
At balanced initial inlet/outlet flows of 60 m3/h, nominal drift is zero.
The plant inlet gain is +1/240 and outlet gain is -1/240 in (%/min)/(m3/h).
For different initial flows, drift is (inlet - outlet)/(60*4) %/min.

## Engineering specification for independent configuration

Use these data to fill copies of the three generic templates. The supplied TOML
files are the completed reference, not runtime dependencies.

| Item | Reference design |
|---|---|
| Sample time | 5 s |
| Primary CV | Level, initially 50%, target 50%, protected limits 20–80% |
| Level objective | Weight 0.20, normalization 10 percentage points |
| Limit penalty | Weight 1000, normalization 5 percentage points |
| Internal CV | Filtered level rate, alpha 0.30, initial/target 0 %/min |
| Rate objective | Weight 0.01, normalization 0.05 %/min |
| MV | Outlet flow, initially 60 m3/h, limits 20–100 m3/h |
| MV move limit | Increase/decrease 0.25 m3/h per sample |
| MV move penalty | Weight 1, normalization 1 m3/h |
| Measured DVs | None; inlet is hidden |
| Horizons | Prediction 60 samples; control 12 samples |
| Solver time limit | 4 s per solve |
| Simulation length | 1050 samples = 87.5 min |
| Inlet event | Sample 30: set 75 m3/h; sample 180: return to 60 m3/h |
| Model checks | Outlet step +5 gives -0.0208333333333 %/min; step -5 gives +0.0208333333333 %/min; tolerance 1e-12 |
| Acceptance | Zero solver failures; raw level 20–80%; largest outlet move <=0.2500001 m3/h; final level error <=0.50 percentage points; final outlet error from 60 <=0.25 m3/h |

In your controller template, retain one primary CV and one MV; delete both
measured-DV blocks and their unused paths. Enable one derived rate CV. In your
simulation template retain one output and one MV input, and replace measured
DV inputs with a hidden inlet input. The plant needs both inlet and outlet
paths; the controller needs only the outlet path. Configure two inlet events
and the five checks above. There are no additional hidden model files.

These tuning values define the reference test, not a universal tuning rule.
They permit a slow return towards 50% while penalising outlet movement.

## Run using the generic commands

```bat
validate_application.bat examples\01_surge_vessel\application.toml examples\01_surge_vessel\simulation.toml
run_simulation.bat examples\01_surge_vessel\application.toml examples\01_surge_vessel\simulation.toml
```

Or substitute the paths to your own two files. Results appear in `results/`
beside the simulation configuration. `expected_results.png` is the reference
plot; `expected_report.json` contains the numerical reference checks.

Reference: level 49.999479–50.590742%, final level 49.999479%, maximum move
0.057948496 m3/h/sample, final outlet 60.217877 m3/h; zero solver failures.
Small numerical and timing variations across machines are expected.

## Complete OPC reference solution

The solution consists of `application.toml`, `simulation.toml` and `opc.toml`.
The OPC file uses the generic template unchanged. No vessel-specific Python,
batch file, external OPC server installation or SCADA package is needed.
The twin embeds its own local OPC UA server.

After setup, validate:

```bat
validate_application.bat examples\01_surge_vessel\application.toml examples\01_surge_vessel\simulation.toml examples\01_surge_vessel\opc.toml
```

Window 1, from the package folder:

```bat
run_digital_twin.bat examples\01_surge_vessel\application.toml examples\01_surge_vessel\simulation.toml examples\01_surge_vessel\opc.toml
```

Wait for `Twin READY`. Window 2, from the package folder:

```bat
run_controller.bat examples\01_surge_vessel\application.toml examples\01_surge_vessel\opc.toml
```

Optional window 3:

```bat
opc_tools.bat examples\01_surge_vessel\opc.toml trend examples\01_surge_vessel\simulation.toml
```

Expected result: 1050 cycles, OPC closed-loop acceptance PASS, and the same
engineering response as offline. OPC results are in `opc_results/`.
The `verified_opc/` folder contains the checked OPC reference report, plot and
CSV trajectory. Its elapsed time reflects an accelerated verification run,
not a timing requirement for your computer.

At normal completion the controller exits and the server stays available.
Stop it with Ctrl+C in window 1, or:

```bat
opc_tools.bat examples\01_surge_vessel\opc.toml stop
```

For your independent engineering exercise substitute your own three file paths
in these same generic commands. The manual sections 19–24 explain the protocol,
fault policy, startup and recovery; there are no additional setup steps hidden
in this example.
