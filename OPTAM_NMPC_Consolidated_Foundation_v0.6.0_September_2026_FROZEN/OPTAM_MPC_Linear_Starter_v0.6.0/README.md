# OPTAM-MPC Linear Application Starter

Version 0.6.0 is built from the consolidated canonical source line.
The APC engineer edits three TOML files; no application-specific Python or batch
file is needed for the supported linear applications.

- `USER_MANUAL.md` — concise software and configuration reference;
- `application_template.toml` — neutral 2-CV/2-MV/2-DV structural template;
- `simulation_template.toml` — neutral plant, event, plot and acceptance template;
- `opc_template.toml` — generic local OPC connection and timing template;
- `examples/01_surge_vessel/` — completed, verified application;
- `examples/02_heat_exchanger/` — 2 CVs, 2 MVs, 2 measured DVs and comparison cases;
- `VERIFICATION.md` — release checks and remaining limitations.

Extract to a NEW folder, open Command Prompt there and run `setup_windows.bat`
once. Follow USER_MANUAL.md section 3 to create your own application folder
from the generic templates, then sections 11–14 for offline operation and
sections 19–24 for OPC operation, monitoring and fault handling.
Keep previous project versions unchanged. No live plant connection is made.

To run the reference using the same generic commands:

```bat
validate_application.bat examples\01_surge_vessel\application.toml examples\01_surge_vessel\simulation.toml
run_simulation.bat examples\01_surge_vessel\application.toml examples\01_surge_vessel\simulation.toml
```

Results go into the selected simulation file's adjacent `results` folder.
These commands do not pause; use an open Command Prompt rather than double-click.
No example-specific launcher is included. A teaching tutorial is deferred.

This version supports integrators, first-order lags and static gains, with dead
times in whole sample intervals. Both examples use the same generic runtime.
SISO FOPDT and multivariable models are configurations of this one engine.
Optional `[[move_combinations]]` support any MV dimension; online objective
tuning is applied as runtime parameters without rebuilding the NLP.
This is a supervised local digital-twin package, not plant-ready deployment.

## Reference OPC run (same generic tools)

In Command Prompt window 1, from the extracted package folder:

```bat
run_digital_twin.bat examples\01_surge_vessel\application.toml examples\01_surge_vessel\simulation.toml examples\01_surge_vessel\opc.toml
```

Wait for `Twin READY`. In window 2, from the same package folder:

```bat
run_controller.bat examples\01_surge_vessel\application.toml examples\01_surge_vessel\opc.toml
```

The two processes communicate through a real local OPC UA server. The simulated
plant advances once per accepted controller command; it freezes on a fault.
An 87.5-minute scenario is paced at up to 20x speed by default (at least about
4.4 minutes wall time). Final files are under `examples\01_surge_vessel\opc_results`.

For live trends, optionally use a third Command Prompt:

```bat
opc_tools.bat examples\01_surge_vessel\opc.toml trend examples\01_surge_vessel\simulation.toml
```

No extra SCADA software is needed. Stop the twin using Ctrl+C, or the generic
`opc_tools.bat ... stop` command. See the manual before testing fault cases.

## Start the heat-exchanger exercise

Read examples/02_heat_exchanger/README.md for commands and expected outcomes;
DESIGN_BASIS.md in that folder gives the physical assumptions and local gains.
Manual sections 25–26 explain configuring it from the same generic templates.
The comparisons include a deliberately failed aggressive-tuning mismatch case
and its passing detuned solution; that expected FAIL is not an installation fault.
