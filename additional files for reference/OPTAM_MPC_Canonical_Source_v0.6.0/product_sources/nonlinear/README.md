# OPTAM-MPC Nonlinear Application Starter

Version **0.2.0** — built from the same canonical core as Linear Starter v0.6.0.

The APC engineer configures three TOML files and supplies one small nonlinear
model Python file. The package supplies the solver, runners, plots and local
OPC UA digital twin. It uses the existing nonlinear velocity-form controller
core, not a linear controller acting on a nonlinear simulator.
Optional declared move combinations work with any MV count, and online
objective tuning no longer rebuilds the nonlinear program.

## Start here

1. Extract into a **new folder**. Keep the linear and frozen projects unchanged.
2. Open Command Prompt in this folder and run `setup_windows.bat` once.
3. Read `USER_MANUAL.md`, especially sections 2–6 defining the model contract.
4. Engineer your own files from the four generic templates.
5. Refer to `examples/01_heat_exchanger/` for the completed solution and results.

To check the supplied solution first:

```bat
validate_application.bat examples\01_heat_exchanger\application.toml examples\01_heat_exchanger\simulation.toml examples\01_heat_exchanger\opc.toml
run_simulation.bat examples\01_heat_exchanger\application.toml examples\01_heat_exchanger\simulation.toml
```

## Contents

| File | Purpose |
|---|---|
| USER_MANUAL.md | Self-contained engineering and operating reference |
| application_template.toml | Generic CV/MV/DV, tuning, model and check configuration |
| model_template.py | Small engineer-owned nonlinear model interface |
| simulation_template.toml | Generic plant, disturbances, targets, checks and plots |
| opc_template.toml | Generic supervised local OPC settings |
| examples/01_heat_exchanger/README.md | Completed solution, commands and comparisons |
| examples/01_heat_exchanger/DESIGN_BASIS.md | Assumptions, equations, units and model limits |
| VERIFICATION.md | Test results and remaining acceptance work |

The reference has 2 CVs, 2 MVs and 2 measured DVs, mild valve curvature,
pressure-dependent flow, nonlinear heat transfer and mixing. It intentionally
uses two output lags without explicit transport delays. There are no hidden
dynamic states to estimate in this starter interface.

The templates are **not pre-filled for the exchanger** and intentionally fail
validation until their placeholders are engineered. The reference is optional;
your own files do not depend on the examples folder.

This is a local development/engineering package, **not a plant-ready deployment
or a safety system**. Loading model Python executes trusted code: inspect it
before validation or running. Never use model files from untrusted sources.
