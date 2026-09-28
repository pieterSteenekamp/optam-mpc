# Completed nonlinear heat-exchanger solution

This is the optional reference against which to compare your own engineering.
Read DESIGN_BASIS.md for the equations, units and deliberately modest operating
range. USER_MANUAL.md in the package root defines the generic interface.

Engineer-owned reference files: application.toml, model.py, simulation.toml and
opc.toml. Additional TOML files define comparison cases; no custom runners exist.

## Combined nominal scenario

Run setup_windows.bat once, then use these commands from the package root:

```bat
validate_application.bat examples\01_heat_exchanger\application.toml examples\01_heat_exchanger\simulation.toml examples\01_heat_exchanger\opc.toml
run_simulation.bat examples\01_heat_exchanger\application.toml examples\01_heat_exchanger\simulation.toml
```

660 samples of 5 seconds cover 55 simulated minutes:

| Sample | Change |
|---:|---|
| 20 | Flow target 60→64 m3/h |
| 100 | Flow target returns to 60 |
| 180 | Temperature target 64→66 degC |
| 260 | Temperature target returns to 64 |
| 340 | Inlet temperature 40→37 degC |
| 420 | Inlet temperature returns to 40 |
| 500 | Upstream pressure 5→5.4 barg |
| 580 | Pressure returns to 5 |

Expected: PASS, no solver failures. Saved reference CSV/report/plot are under
verified/combined. Your new run writes results/ unless an output folder is supplied.

## Local OPC digital twin

Window 1:

```bat
run_digital_twin.bat examples\01_heat_exchanger\application.toml examples\01_heat_exchanger\simulation.toml examples\01_heat_exchanger\opc.toml
```

After Twin READY, window 2:

```bat
run_controller.bat examples\01_heat_exchanger\application.toml examples\01_heat_exchanger\opc.toml
```

Optional third window:

```bat
opc_tools.bat examples\01_heat_exchanger\opc.toml trend examples\01_heat_exchanger\simulation.toml
```

The default 20x pace takes at least about 2.75 wall minutes plus startup/reporting;
slower solving extends it. Results are under opc_results/. Reference results
are under verified_opc/. No separate OPC server installation or SCADA is needed.

```bat
opc_tools.bat examples\01_heat_exchanger\opc.toml status
opc_tools.bat examples\01_heat_exchanger\opc.toml stop
```

## Feedforward on/off comparison

disturbances.toml keeps targets at 60 m3/h and 64 degC. Inlet temperature changes
at samples 30/110; pressure changes at 190/270. Total duration is 360 samples,
or 30 simulated minutes. Only feedforward selection changes between controllers.

```bat
run_simulation.bat examples\01_heat_exchanger\application.toml examples\01_heat_exchanger\disturbances.toml examples\01_heat_exchanger\results_ff_on
run_simulation.bat examples\01_heat_exchanger\application_no_feedforward.toml examples\01_heat_exchanger\disturbances.toml examples\01_heat_exchanger\results_ff_off
.venv\Scripts\python compare_results.py examples\01_heat_exchanger\results_ff_on\results.csv examples\01_heat_exchanger\results_ff_off\results.csv --signal outlet_flow 60 --signal outlet_temperature 64 --labels "Feedforward on" "Feedforward off" --output examples\01_heat_exchanger\comparison
```

Both runs should PASS; inspect peak/accumulated errors and the overlay rather
than assume feedforward improves every measure. Supplied results are in
verified/feedforward_on, verified/feedforward_off and verified/comparison.

## Modest plant mismatch

```bat
run_simulation.bat examples\01_heat_exchanger\application.toml examples\01_heat_exchanger\mismatch.toml examples\01_heat_exchanger\results_mismatch
```

Expected PASS with nominal controller tuning. Plant time constants increase by
20%, valve curvature doubles from 0.1 to 0.2; the controller is unchanged.
The final-five-minute checks use the same tolerances as the disturbance runs.
See verified/mismatch for the saved result. This scenario is not intended to
demonstrate severe mismatch or global stability.

## Independent engineering exercise

Create your own application directory using the four generic templates, as
described in USER_MANUAL.md section 3. Fill the TOML definitions and model Python
from DESIGN_BASIS.md. Validate before running. If results differ, compare units,
ID/order, absolute transition vs increments, parameters, initial equilibrium,
dt and scenario timing before changing tuning. Do not edit generic runtime files
to make this specific example work.
