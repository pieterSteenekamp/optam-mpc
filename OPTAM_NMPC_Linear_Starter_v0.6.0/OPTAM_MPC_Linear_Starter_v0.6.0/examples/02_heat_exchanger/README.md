# Example 02 — linear 2x2 heat exchanger with bypass and two measured DVs

This is the completed reference solution. All runtime models here are linear,
non-integrating paths. Read DESIGN_BASIS.md for the operating point, physical
gain derivation, seven nonzero paths and assumed lags/delays.

## Files and expected outcomes

| Controller | Simulation | Purpose | Expected |
|---|---|---|---|
| application.toml | simulation.toml | Setpoints and both measured DVs | PASS |
| application.toml | disturbances.toml | Feedforward enabled | PASS |
| application_no_feedforward.toml | disturbances.toml | Feedback only | PASS, larger errors |
| application.toml | mismatch.toml | Nominal tuning against mismatch | FAIL: flow oscillation |
| application_detuned.toml | mismatch.toml | Larger move penalties | PASS |

opc.toml uses the generic protocol on port 4841. No example-specific Python
model, launcher, external OPC installation or SCADA package is required.

## Engineer it independently

From the extracted package root:

```bat
mkdir applications\my_hx
copy application_template.toml applications\my_hx\application.toml
copy simulation_template.toml applications\my_hx\simulation.toml
copy opc_template.toml applications\my_hx\opc.toml
```

Retain two CVs, two MVs and two measured DVs. No derived CV is needed. Change
the illustrative integrator paths to first_order, enter gain/tau/dead time,
and duplicate blocks to obtain the seven nonzero relationships. Fill in
initial values, targets, limits, normalizations and weights from the design
basis and completed application.toml.

Define two plant outputs and four inputs. Map both valve inputs as MVs, both
inlet measurements as measured_dv, and enter the independent plant paths.
Set nominal drift to zero. Configure events, checks and plots. The completed
simulation.toml shows the result; it is not a runtime dependency for your files.

## Combined scenario

Sample time 5 seconds; 660 samples represent 55 minutes.

| Sample | Change |
|---:|---|
| 20 | Flow target 60 -> 64 m3/h |
| 100 | Flow target -> 60 m3/h |
| 180 | Temperature target 64 -> 67 degC |
| 260 | Temperature target -> 64 degC |
| 340 | Inlet temperature 40 -> 37 degC |
| 420 | Inlet temperature -> 40 degC |
| 500 | Upstream pressure 5 -> 5.4 barg |
| 580 | Upstream pressure -> 5 barg |

The MPC sees changes when they occur, not a preview of the event schedule.

## Validate and run the nominal reference

Run setup_windows.bat once. Use open Command Prompt windows, each in the
package folder. Substitute your own paths when testing your engineering work.

```bat
validate_application.bat examples\02_heat_exchanger\application.toml examples\02_heat_exchanger\simulation.toml examples\02_heat_exchanger\opc.toml
run_simulation.bat examples\02_heat_exchanger\application.toml examples\02_heat_exchanger\simulation.toml
```

For OPC, window 1:

```bat
run_digital_twin.bat examples\02_heat_exchanger\application.toml examples\02_heat_exchanger\simulation.toml examples\02_heat_exchanger\opc.toml
```

Wait for Twin READY. Window 2:

```bat
run_controller.bat examples\02_heat_exchanger\application.toml examples\02_heat_exchanger\opc.toml
```

Optional window 3:

```bat
opc_tools.bat examples\02_heat_exchanger\opc.toml trend examples\02_heat_exchanger\simulation.toml
```

The default 20x pace needs at least 2.75 wall minutes plus startup/reporting;
slower solving can extend the run. Offline results are under results/ and OPC
results under opc_results/, beside the selected simulation file. Stop with
Ctrl+C in the twin window or:

```bat
opc_tools.bat examples\02_heat_exchanger\opc.toml stop
```

## Feedforward comparison

disturbances.toml keeps both targets fixed for 360 samples (30 minutes).
Inlet temperature drops at sample 30 and returns at 110. Pressure increases
at 190 and returns at 270. Both runs use the same plant and disturbances.

```bat
run_simulation.bat examples\02_heat_exchanger\application.toml examples\02_heat_exchanger\disturbances.toml results\hx_ff_on
run_simulation.bat examples\02_heat_exchanger\application_no_feedforward.toml examples\02_heat_exchanger\disturbances.toml results\hx_ff_off
.venv\Scripts\python compare_results.py results\hx_ff_on\results.csv results\hx_ff_off\results.csv --signal outlet_flow 60 --signal outlet_temperature 64 --labels "Feedforward on" "Feedforward off" --output results\hx_comparison
```

Disabling feedforward holds only the controller DVs at nominal values. The
actual simulated disturbances still occur. The comparison produces
comparison.png and comparison.json with peak errors and IAE.

## Mismatch and detuning

Mismatch multiplies plant gains by 1.15, lags by 1.20 and adds 5 seconds to
every delay. The controller model remains unchanged.

```bat
run_simulation.bat examples\02_heat_exchanger\application.toml examples\02_heat_exchanger\mismatch.toml results\hx_mismatch_fast
run_simulation.bat examples\02_heat_exchanger\application_detuned.toml examples\02_heat_exchanger\mismatch.toml results\hx_mismatch_detuned
```

The first intentionally fails: move weights 0.05 allow sustained flow
oscillation. The second increases both weights to 0.5 and meets the unchanged
acceptance checks. The final five-minute window requires flow error <=0.1
m3/h and temperature error <=0.1 degC at every point, not only the final point.

Detuning removes sustained oscillation in this case but can increase transient
peak errors. It does not prove robustness to every possible mismatch.

## Reference outputs

verified/ contains plots, reports and CSVs for the offline cases and feedforward
comparison, including the labelled failed mismatch case. verified_opc/ contains
the complete combined OPC reference. These files are outputs, not required
application inputs. Small floating-point and timing differences are expected.

Manual sections 10–14 define the model/scenario fields; 19–24 describe OPC;
25–26 describe engineering this application and comparing its cases.
