# Enhanced SISO MPC Test Harness

## User Guide

This document explains how to install, run, configure and interpret the enhanced Model Predictive Control (MPC) test harness supplied in `test_mpc(1).py`.

The program simulates a single-input, single-output (SISO) first-order-plus-dead-time (FOPDT) process and controls it with an MPC algorithm. It is intended for:

- learning and demonstrating MPC principles;
- testing controller tuning and constraint handling;
- investigating process-model mismatch;
- testing disturbance rejection and measurement noise;
- demonstrating economic MPC; and
- providing a foundation for more advanced MPC development.

> **Important:** This is a simulation and teaching program. It is not, by itself, an industrial controller suitable for direct connection to an operating plant.

---

## 1. Main capabilities

The program includes:

- exact discrete-time first-order process dynamics;
- zero, integer and fractional-sample dead time;
- correct preservation of past inputs travelling through the dead-time pipeline;
- filtered output-bias estimation for offset-free control;
- separate representations of:
  - the true process output;
  - a load disturbance acting on the plant input;
  - an additive output disturbance;
  - measurement noise; and
  - the measured output supplied to the controller;
- separate prediction and control horizons;
- manipulated-variable limits;
- manipulated-variable rate-of-change limits;
- hard output constraints;
- soft output constraints using explicit slack variables;
- shifted-solution warm starting;
- a terminal tracking penalty;
- an optional economic objective;
- solver diagnostics;
- predicted-trajectory snapshots;
- closed-loop performance metrics;
- five demonstration scenarios; and
- an automated validation suite.

---

## 2. Process model

The simulated process has the continuous transfer function:

$$
G(s)=\frac{K}{\tau s+1}e^{-\theta s}
$$

where:

- $K$ is the process gain;
- $\tau$ is the process time constant;
- $\theta$ is the process dead time; and
- $s$ is the Laplace variable.

The first-order dynamics are discretised exactly for a zero-order-held input:

$$
y_{k+1}=a y_k+(1-a)K u_{\text{delayed},k}
$$

with:

$$
a=e^{-\Delta t/\tau}
$$

Fractional dead time is represented by linearly interpolating between the two surrounding stored input samples.

### 2.1 Sample-time convention

The program uses the following sequence at sample $k$:

1. Measure `y_measured[k]`.
2. Calculate the new command `u[k]`.
3. Apply `u[k]` to the plant over the next interval.
4. Calculate `y_true[k+1]`.
5. Add the output disturbance and measurement noise to form the next measurement.

This convention ensures that the recorded plant, model, measurement and input signals are correctly aligned in time.

### 2.2 Signal definitions

The program distinguishes between the following signals:

| Signal | Meaning |
|---|---|
| `u_applied` | Manipulated-variable command calculated by the MPC |
| `load_disturbance` | Unmeasured disturbance added to the plant input |
| `u_effective` | `u_applied + load_disturbance` |
| `y_true` | Dynamic output of the simulated plant |
| `output_disturbance` | Additive disturbance applied after the plant dynamics |
| `y_controlled` | `y_true + output_disturbance` |
| `measurement_noise` | Random noise added to the controlled output |
| `y_measured` | `y_controlled + measurement_noise`; the value supplied to the MPC |
| `y_model` | Output of the controller's internal process model |

---

## 3. Software requirements

The recommended environment is:

- Python 3.10 or later;
- NumPy;
- SciPy; and
- Matplotlib.

The program does not require a separate optimisation package. It uses the SLSQP optimiser supplied by SciPy.

---

## 4. Installation

### 4.1 Recommended filename

The supplied filename is `test_mpc(1).py`. It can be run with that name as long as it is placed in quotation marks on the command line.

For easier importing into other Python programs, it is recommended that you rename it to:

```text
test_mpc.py
```

The examples in this guide use `test_mpc.py`. If you retain the original filename, replace `test_mpc.py` with `"test_mpc(1).py"` in command-line examples.

### 4.2 Create a virtual environment on Windows

Open Command Prompt or PowerShell in the directory containing the program and run:

```bash
py -m venv .venv
.venv\Scripts\activate
python -m pip install --upgrade pip
python -m pip install numpy scipy matplotlib
```

### 4.3 Create a virtual environment on macOS or Linux

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install numpy scipy matplotlib
```

You normally need to create the virtual environment only once. Activate it again whenever you return to the project.

---

## 5. Quick start

### 5.1 Run all demonstrations and display the graphs

```bash
python test_mpc.py
```

The program runs five demonstrations. Close the current graph window to continue to the next demonstration.

### 5.2 Run the demonstrations without opening graphs

```bash
python test_mpc.py --no-plots
```

This is useful when running the program from a terminal, automated process or computer without a graphical display.

### 5.3 Run the automated validation suite

```bash
python test_mpc.py --self-test
```

A successful run ends with:

```text
ALL VALIDATION CHECKS PASSED
```

The program returns a non-zero operating-system exit code if any validation check fails. This allows it to be used in an automated test or continuous-integration process.

### 5.4 Run one demonstration

```bash
python test_mpc.py --demo constraint
```

Available demonstration names are:

- `baseline`
- `mismatch`
- `disturbance`
- `constraint`
- `economic`
- `all`

### 5.5 Save graphs as PNG files

```bash
python test_mpc.py --no-plots --save-plots plots
```

The program creates the specified directory if it does not exist. When all demonstrations are selected, it writes:

```text
baseline.png
mismatch.png
disturbance.png
constraint.png
economic.png
```

To save and display a single demonstration:

```bash
python test_mpc.py --demo economic --save-plots plots
```

---

## 6. Command-line reference

| Option | Description |
|---|---|
| No option | Run all demonstrations and display each graph |
| `--self-test` | Run the automated validation suite and exit |
| `--no-plots` | Run demonstrations without displaying graph windows |
| `--demo NAME` | Run `all` or one named demonstration |
| `--save-plots DIRECTORY` | Save demonstration graphs as PNG files |
| `-h` or `--help` | Display the program's command-line help |

When `--self-test` is supplied, the program runs the validation suite rather than the demonstrations.

---

## 7. Supplied demonstrations

### 7.1 Baseline

Run with:

```bash
python test_mpc.py --demo baseline
```

This demonstration uses an exact controller model and applies two setpoint changes. It provides a reference for normal setpoint tracking and illustrates:

- finite dead time;
- input rate limits;
- the relationship between the prediction and control horizons; and
- prediction snapshots at selected setpoint changes.

Expected behaviour: the output should settle at each setpoint without sustained oscillation or steady-state offset.

### 7.2 Model mismatch

Run with:

```bash
python test_mpc.py --demo mismatch
```

The real plant and controller model have different:

- gains;
- time constants; and
- dead times.

The real process also has a fractional-sample dead time. This example demonstrates how the filtered output-bias estimate helps the controller recover from process-model mismatch.

Expected behaviour: the transient response is less accurate than the baseline response, but the final offset should be small and no sustained alternating oscillation should occur.

### 7.3 Output disturbance and measurement noise

Run with:

```bash
python test_mpc.py --demo disturbance
```

This demonstration applies:

- a persistent additive output disturbance; and
- random measurement noise.

Expected behaviour: the controller should estimate and reject the persistent disturbance. The estimated bias will be smoother than the raw measurement because the disturbance estimator is filtered.

### 7.4 Hard output and move constraints

Run with:

```bash
python test_mpc.py --demo constraint
```

The setpoint is initially moved above the maximum permitted process output. The controller must operate as close as possible to the active output limit without violating it. The input is also subject to rate-of-change limits.

Expected behaviour:

- the output rises to the maximum output constraint;
- the constraint is maintained without the two-sample cycling seen in the earlier implementation;
- input moves respect the specified rate limits; and
- after the setpoint is reduced, the output returns to normal setpoint tracking.

### 7.5 Economic MPC

Run with:

```bash
python test_mpc.py --demo economic
```

The process starts at a feasible steady state inside a hard operating window. The controller has both:

- a tracking objective; and
- an economic reward for increasing the input.

Expected behaviour: the controller moves away from the nominal tracking target and operates at the upper output constraint to obtain greater economic benefit.

The reported steady-state tracking error in this demonstration is intentional. It is the result of the economic trade-off and is not a controller failure.

---

## 8. Interpreting the graph

Each demonstration graph contains four panels.

### 8.1 Output panel

The first panel shows:

- the setpoint;
- the true plant output;
- the measured output;
- the internal-model output;
- any finite output constraints; and
- selected future predicted trajectories.

The short orange dashed curves are prediction snapshots made at the samples specified by `snapshot_steps`.

### 8.2 Input panel

The second panel shows:

- the commanded input calculated by the MPC;
- the effective input when a load disturbance is present; and
- the upper and lower input limits.

The effective input differs from the commanded input only when `load_disturbance` is non-zero.

### 8.3 Disturbance panel

The third panel can show:

- the estimated output bias;
- the true additive output disturbance;
- the load disturbance; and
- measurement noise.

The output-bias estimate represents everything that causes the measured output to differ from the free-running internal model. It can therefore include the effects of a constant output disturbance, process-model mismatch and slowly varying unmeasured loads. It should not always be expected to equal the defined output disturbance exactly.

### 8.4 Solver panel

The fourth panel shows the number of optimisation iterations used at every control sample.

It also marks:

- unusable optimisation results; and
- slack-variable usage when soft output constraints are enabled.

A sudden increase in solver iterations usually indicates a setpoint change, a newly active constraint or a significant disturbance.

---

## 9. Printed performance summary

After every demonstration, the program prints a numerical summary.

| Metric | Interpretation |
|---|---|
| Final-window error | Mean controlled-output error over the final assessment window |
| Final-window standard deviation | Variation in the final tracking error |
| Settling time | Time after the last setpoint change until the error remains inside the specified tolerance |
| IAE | Integral of absolute tracking error |
| ISE | Integral of squared tracking error; available in `result.metrics` |
| Overshoot | Maximum movement beyond the final setpoint after its last change |
| Total `|delta u|` | Sum of the absolute input movements |
| Maximum output violation | Largest violation of `y_min` or `y_max` during the simulation |
| Maximum input violation | Largest violation of `u_min` or `u_max` |
| Maximum move violation | Largest violation of the input rate limits |
| Solver failures | Number of samples at which no usable optimisation result was obtained |
| Maximum slack | Largest soft-constraint relaxation used |
| Oscillation detected | Heuristic indication of a sustained alternating late-stage response |

`not settled` means that the output did not remain within the settling tolerance for the remainder of the simulation. This can be expected when:

- the setpoint is unreachable;
- the economic objective intentionally moves away from the setpoint;
- the simulation ends before the process settles; or
- persistent noise or oscillation exceeds the settling tolerance.

---

## 10. Creating a custom simulation

Rename the supplied program to `test_mpc.py` before importing it.

The following example creates and runs a custom scenario:

```python
import matplotlib.pyplot as plt

from test_mpc import Scenario, plot_result, run_simulation


scenario = Scenario(
    name="My MPC test",
    n_steps=100,
    dt=1.0,
    process_gain=1.2,
    process_tau=7.0,
    process_dead_time=2.5,
    setpoint_sequence={0: 0.0, 10: 1.0, 60: 0.6},
    prediction_horizon=20,
    control_horizon=5,
    move_weight=0.15,
    u_min=0.0,
    u_max=3.0,
    delta_u_min=-0.3,
    delta_u_max=0.3,
    snapshot_steps=(10, 60),
)

result = run_simulation(scenario)
plot_result(result, scenario)
plt.show()
```

`run_simulation()` returns a `Result` object containing the full time-series data, solver diagnostics and calculated performance metrics.

---

## 11. Scenario configuration reference

### 11.1 General simulation settings

| Parameter | Default | Meaning |
|---|---:|---|
| `name` | `"Unnamed"` | Descriptive scenario name |
| `n_steps` | `100` | Number of control intervals |
| `dt` | `1.0` | Sample interval and displayed time unit |
| `y0` | `0.0` | Initial true plant output |
| `u0` | `0.0` | Initial input and initial dead-time history value |

`u0` must lie between `u_min` and `u_max`.

### 11.2 Real plant parameters

| Parameter | Default | Meaning |
|---|---:|---|
| `process_gain` | `1.0` | Real plant gain $K$ |
| `process_tau` | `5.0` | Real plant time constant $\tau$ |
| `process_dead_time` | `2.0` | Real plant dead time $\theta$ |

The time constant must be positive and the dead time must not be negative.

### 11.3 Controller-model parameters

| Parameter | Default | Meaning |
|---|---:|---|
| `model_gain` | `None` | Model gain; `None` uses the plant gain |
| `model_tau` | `None` | Model time constant; `None` uses the plant time constant |
| `model_dead_time` | `None` | Model dead time; `None` uses the plant dead time |

Specify different plant and model values to test robustness against model mismatch.

### 11.4 Setpoint sequence

Setpoint changes are defined as a dictionary whose keys are sample indices:

```python
setpoint_sequence={0: 0.0, 10: 1.0, 60: 0.5}
```

This means:

- setpoint 0.0 from sample 0;
- setpoint 1.0 from sample 10; and
- setpoint 0.5 from sample 60.

The current controller assumes that the latest setpoint remains constant over its prediction horizon. It does not preview future entries in `setpoint_sequence`.

### 11.5 Disturbances and noise

| Parameter | Default | Meaning |
|---|---:|---|
| `load_disturbance` | `None` | Function returning an additive plant-input disturbance |
| `output_disturbance` | `None` | Function returning an additive output disturbance |
| `measurement_noise_std` | `0.0` | Standard deviation of Gaussian measurement noise |
| `random_seed` | `1` | Random seed used to make noise tests repeatable |

### 11.6 MPC horizons and tuning

| Parameter | Default | Meaning |
|---|---:|---|
| `prediction_horizon` | `15` | Number of future output samples predicted, $N_p$ |
| `control_horizon` | `5` | Number of independently optimised future inputs, $N_c$ |
| `tracking_weight` | `1.0` | Weight applied to predicted tracking error |
| `move_weight` | `0.1` | Weight applied to changes in the input |
| `terminal_weight` | `2.0` | Extra tracking weight on the final predicted output |
| `disturbance_filter` | `0.25` | Bias-estimator update factor from 0 to 1 |
| `y_scale` | `1.0` | Output scaling used in the objective function |
| `u_scale` | `1.0` | Input scaling used in the objective function |

`control_horizon` must not exceed `prediction_horizon`. After the control horizon, the final optimised input is held constant for the rest of the prediction horizon.

### 11.7 Input constraints

| Parameter | Default | Meaning |
|---|---:|---|
| `u_min` | `-5.0` | Minimum input |
| `u_max` | `5.0` | Maximum input |
| `delta_u_min` | `-infinity` | Minimum permitted input change per sample |
| `delta_u_max` | `infinity` | Maximum permitted input change per sample |

Input magnitude and input-move constraints are always treated as hard constraints.

### 11.8 Output constraints

| Parameter | Default | Meaning |
|---|---:|---|
| `y_min` | `-infinity` | Minimum output |
| `y_max` | `infinity` | Maximum output |
| `output_constraint_mode` | `"none"` | `"none"`, `"hard"` or `"soft"` |
| `slack_quadratic_weight` | `10000.0` | Quadratic soft-constraint slack penalty |
| `slack_linear_weight` | `100.0` | Linear soft-constraint slack penalty |

Setting `y_min` or `y_max` does not activate an output constraint unless `output_constraint_mode` is set to `"hard"` or `"soft"`.

### 11.9 Economic objective

| Parameter | Default | Meaning |
|---|---:|---|
| `economic_weight` | `0.0` | Weight given to the economic benefit |
| `economic_direction` | `1.0` | `1.0` rewards an increase; `-1.0` rewards a decrease |
| `economic_variable` | `"input"` | Economic signal: `"input"` or `"output"` |

An `economic_weight` of zero disables economic optimisation.

### 11.10 Optimiser and reporting

| Parameter | Default | Meaning |
|---|---:|---|
| `optimizer_max_iterations` | `300` | Maximum SLSQP iterations per control sample |
| `optimizer_tolerance` | `1e-8` | Optimiser convergence tolerance |
| `settling_tolerance` | `0.02` | Settling band expressed relative to `y_scale` |
| `snapshot_steps` | Empty tuple | Samples at which future predictions are stored and plotted |

Example:

```python
snapshot_steps=(10, 30, 60)
```

---

## 12. Adding disturbances

A disturbance function accepts the integer sample index and returns a floating-point value.

### 12.1 Persistent output disturbance

```python
def output_step(k: int) -> float:
    return 0.0 if k < 30 else 0.4


scenario = Scenario(
    name="Output disturbance test",
    output_disturbance=output_step,
    setpoint_sequence={0: 1.0},
)
```

This is appropriate for testing an additive measurement or controlled-output disturbance.

### 12.2 Temporary load disturbance

```python
def load_pulse(k: int) -> float:
    return 0.3 if 25 <= k < 45 else 0.0


scenario = Scenario(
    name="Load pulse test",
    load_disturbance=load_pulse,
    setpoint_sequence={0: 1.0},
)
```

This disturbance changes the effective input received by the real plant without informing the controller.

### 12.3 Measurement noise

```python
scenario = Scenario(
    name="Noisy measurement",
    measurement_noise_std=0.03,
    random_seed=42,
    disturbance_filter=0.1,
)
```

Using the same random seed produces the same noise sequence, which makes comparisons between controller tunings repeatable.

---

## 13. Hard and soft output constraints

### 13.1 Hard constraint

```python
scenario = Scenario(
    y_max=2.0,
    output_constraint_mode="hard",
)
```

The predicted output must remain at or below 2.0. A hard problem can become infeasible when, for example:

- the initial condition already violates the constraint;
- material or energy already travelling through the dead time makes a future violation unavoidable;
- an unanticipated disturbance occurs; or
- input and move limits prevent sufficient corrective action.

If the program cannot obtain a usable optimisation result, it records the failure and applies a safe fallback that holds the previous command within its limits.

### 13.2 Soft constraint

```python
scenario = Scenario(
    y_min=1.0,
    y_max=2.0,
    output_constraint_mode="soft",
    slack_quadratic_weight=10_000.0,
    slack_linear_weight=100.0,
)
```

Soft constraints introduce explicit non-negative slack variables. The controller can relax the output constraint when it is impossible or excessively costly to satisfy.

The amount of relaxation is recorded in `result.maximum_slack` and summarised as `result.metrics.maximum_slack`.

Use soft constraints when feasibility must be maintained. Increase the slack penalties when output-constraint violations should be more expensive, but avoid unnecessarily extreme values that can cause poor numerical scaling.

---

## 14. Offset-free disturbance estimation

At every sample, the controller calculates the difference between the measured output and its free-running internal model:

$$
e_{\text{bias},k}=y_{\text{measured},k}-y_{\text{model},k}
$$

The filtered estimate is:

$$
\hat d_k=\alpha e_{\text{bias},k}+(1-\alpha)\hat d_{k-1}
$$

where `disturbance_filter` is $\alpha$.

The estimate $\hat d_k$ is added to every future output prediction.

### 14.1 Selecting the filter value

- A value of `1.0` uses the complete latest model error and provides no filtering.
- A smaller value gives a smoother but slower estimate.
- Values around `0.05` to `0.3` are often useful starting points when noise or dynamic model mismatch is present.
- A value of `0.0` freezes the estimate at its current value.

This bias approach can remove steady-state offset for constant disturbances and reachable steady states under appropriate closed-loop conditions. It does not guarantee stability or zero offset for every possible model mismatch, constraint combination or disturbance.

---

## 15. MPC objective function

In simplified form, the controller minimises:

$$
J = Q_y\sum_{j=1}^{N_p}\left(\frac{\hat y_{k+j}-r}{y_{\text{scale}}}\right)^2
+R_{\Delta u}\sum_{j=0}^{N_c-1}\left(\frac{\Delta u_{k+j}}{u_{\text{scale}}}\right)^2
+Q_T\left(\frac{\hat y_{k+N_p}-r}{y_{\text{scale}}}\right)^2
-J_{\text{economic}}
+J_{\text{slack}}
$$

where:

- $Q_y$ is `tracking_weight`;
- $R_{\Delta u}$ is `move_weight`;
- $Q_T$ is `terminal_weight`;
- $r$ is the current setpoint;
- $J_{\text{economic}}$ is the optional economic reward; and
- $J_{\text{slack}}$ penalises soft-constraint relaxation.

The weights do not have universal values. Their effects depend on the units and scaling of the input and output.

---

## 16. Practical tuning procedure

The following procedure is suitable for initial simulation studies.

### Step 1: Choose the sample interval

Select `dt` small enough to represent the process dynamics and dead time. A common starting point is several samples within the fastest important process time constant.

### Step 2: Select the prediction horizon

Choose a horizon long enough to include:

- the dead time; and
- most of the important first-order response.

A practical starting estimate is:

$$
N_p \Delta t \geq \theta + 3\tau
$$

This is guidance rather than a strict rule. Longer horizons increase computation.

### Step 3: Select the control horizon

Begin with a control horizon between approximately 3 and 8 samples, ensuring that:

$$
1\leq N_c\leq N_p
$$

A shorter control horizon reduces the number of optimisation variables and normally produces smoother future input profiles.

### Step 4: Tune tracking and movement

Start with:

```python
tracking_weight=1.0
move_weight=0.1
```

- Increase `move_weight` for smoother, less aggressive input changes.
- Decrease `move_weight` for faster but more aggressive control.
- Use `delta_u_min` and `delta_u_max` to represent real actuator-rate limitations.

### Step 5: Select signal scales

Choose `y_scale` and `u_scale` to represent meaningful expected ranges. For example, if the normal output range is 100 units and the normal input range is 20 units:

```python
y_scale=100.0
u_scale=20.0
```

Appropriate scaling makes the tuning weights easier to interpret and improves numerical conditioning.

### Step 6: Tune the disturbance estimator

Reduce `disturbance_filter` when noise causes unnecessary input movement. Increase it when the disturbance estimate responds too slowly.

### Step 7: Introduce constraints

Add realistic:

- input limits;
- move limits; and
- output constraints.

Test hard constraints for feasibility. Use soft constraints where temporary violation is preferable to an infeasible optimisation.

### Step 8: Introduce model mismatch and disturbances

Do not judge robustness using only the exact-model case. Test realistic errors in:

- gain;
- time constant;
- dead time;
- initial condition;
- load disturbances; and
- measurement noise.

---

## 17. Economic MPC

Economic MPC adds a benefit term that can reward either the predicted input or predicted output.

### 17.1 Rewarding a higher input

```python
scenario = Scenario(
    y0=2.0,
    u0=2.0,
    setpoint_sequence={0: 2.0},
    tracking_weight=0.2,
    economic_weight=0.5,
    economic_direction=1.0,
    economic_variable="input",
    y_min=1.5,
    y_max=3.0,
    output_constraint_mode="hard",
)
```

### 17.2 Rewarding a lower output

```python
economic_weight=0.5
economic_direction=-1.0
economic_variable="output"
```

The economic objective and constraints are independent. The economic term is active throughout the horizon; it is not switched on and off by a discontinuous constraint gate.

For a meaningful industrial economic MPC application, the economic signal should ultimately be replaced or extended with an objective expressed in actual business terms, such as:

- production value;
- energy cost;
- raw-material cost;
- product-quality penalty; or
- constraint-related operating risk.

---

## 18. Accessing simulation results in Python

```python
result = run_simulation(scenario, verbose=False)

print(result.y_true)
print(result.u_applied)
print(result.metrics.integral_absolute_error)
print(result.metrics.solver_failures)
```

### 18.1 Main `Result` fields

| Field | Contents |
|---|---|
| `scenario_name` | Scenario name |
| `time` | Plant and output sample times |
| `setpoint` | Setpoint at every output sample |
| `y_true` | True plant output |
| `y_controlled` | True output plus output disturbance |
| `y_measured` | Controlled output plus measurement noise |
| `y_model` | Time-aligned internal-model output |
| `u_time` | Time vector for manipulated-variable samples |
| `u_applied` | MPC commands |
| `u_effective` | MPC command plus load disturbance |
| `load_disturbance` | Plant-input disturbance |
| `output_disturbance` | Additive output disturbance |
| `measurement_noise` | Generated noise sequence |
| `disturbance_estimate` | Filtered output-bias estimate |
| `solver_success` | Whether each optimisation produced a usable action |
| `solver_iterations` | Optimiser iterations per sample |
| `solver_objective` | Objective value per sample |
| `solver_messages` | Solver result message per sample |
| `solver_warnings` | Captured numerical warnings per sample |
| `minimum_constraint_residual` | Smallest optimisation-constraint residual per sample |
| `maximum_predicted_violation` | Largest predicted output-limit violation per sample |
| `maximum_slack` | Largest slack variable per sample |
| `prediction_snapshots` | Stored predicted trajectories |
| `metrics` | `PerformanceMetrics` object |

### 18.2 `PerformanceMetrics` fields

```python
metrics = result.metrics

print(metrics.steady_state_error)
print(metrics.steady_state_std)
print(metrics.integral_absolute_error)
print(metrics.integral_squared_error)
print(metrics.maximum_absolute_error)
print(metrics.overshoot)
print(metrics.settling_time)
print(metrics.total_input_movement)
print(metrics.maximum_output_constraint_violation)
print(metrics.maximum_input_constraint_violation)
print(metrics.maximum_move_constraint_violation)
print(metrics.solver_failures)
print(metrics.maximum_slack)
print(metrics.saturation_fraction)
print(metrics.oscillation_detected)
```

The oscillation detector is a diagnostic heuristic based on the final portion of the output. It is useful for detecting sustained alternating behaviour, but it is not a substitute for a formal closed-loop stability analysis.

---

## 19. Automated validation suite

The validation suite begins with three direct process-model checks:

- zero dead time responds in the current interval;
- fractional dead time is interpolated; and
- a cloned predictor preserves the delayed-input history.

It then runs eleven closed-loop scenarios:

1. Perfect process model
2. Combined gain, time-constant and dead-time mismatch
3. Exactly zero dead time
4. Fractional dead time
5. Measurement noise
6. Constant output disturbance
7. Dynamic load-disturbance pulse
8. Unreachable setpoint with actuator and rate limits
9. Hard output constraint
10. Initially infeasible soft output constraint
11. Negative process gain

Depending on the scenario, the suite checks:

- final-window tracking error;
- model and plant time alignment;
- solver usability;
- input-limit compliance;
- move-limit compliance;
- hard output-constraint compliance;
- expected slack usage;
- correct actuator-saturation behaviour; and
- absence of sustained alternating oscillation.

These tests verify the implemented examples. They do not prove stability for all possible controller configurations.

---

## 20. Troubleshooting

### 20.1 `ModuleNotFoundError`

Example:

```text
ModuleNotFoundError: No module named 'scipy'
```

Activate the intended virtual environment and install the dependencies:

```bash
python -m pip install numpy scipy matplotlib
```

### 20.2 The program cannot be imported

Python module names cannot conveniently contain parentheses. Rename:

```text
test_mpc(1).py
```

to:

```text
test_mpc.py
```

before using `from test_mpc import ...`.

### 20.3 No graph appears

Confirm that Matplotlib is installed and that the computer has a supported graphical backend. On a computer without a display, use:

```bash
python test_mpc.py --no-plots
```

or save the graphs:

```bash
python test_mpc.py --no-plots --save-plots plots
```

### 20.4 The output does not react immediately to an input move

This is expected when the process has dead time. Check `process_dead_time`, `model_dead_time` and `dt`.

### 20.5 The controller is too aggressive

Try one or more of the following:

- increase `move_weight`;
- tighten `delta_u_min` and `delta_u_max`;
- reduce `disturbance_filter` when noise is present;
- use realistic `y_scale` and `u_scale`; or
- increase the prediction horizon if important consequences currently fall beyond it.

### 20.6 The response is too slow

Try:

- decreasing `move_weight`;
- relaxing overly restrictive move limits;
- increasing the control horizon moderately; or
- increasing `disturbance_filter` if disturbance estimation is too slow.

### 20.7 A hard constraint causes solver failures

Investigate whether the problem is genuinely infeasible. In particular, check:

- the initial output;
- the input already stored in the dead-time pipeline;
- input magnitude limits;
- input rate limits;
- the process gain direction; and
- newly applied disturbances.

Use a soft output constraint when controlled relaxation is preferable to infeasibility.

### 20.8 Soft-constraint slack is unexpectedly large

Check whether:

- the output constraint is reachable;
- the initial state violates the constraint;
- the input bounds are too restrictive;
- the move bounds are too restrictive; or
- the model gain has the wrong sign.

### 20.9 A steady-state tracking error remains

Check for:

- an unreachable setpoint;
- active input or output constraints;
- economic optimisation intentionally trading tracking for benefit;
- inadequate simulation length;
- excessive disturbance filtering;
- persistent time-varying disturbances; or
- severe process-model mismatch.

### 20.10 The optimiser is slow

Try:

- reducing `prediction_horizon` while still covering the important dynamics;
- reducing `control_horizon`;
- avoiding unnecessarily extreme slack penalties;
- improving signal scaling; or
- reducing `optimizer_max_iterations` only after confirming that normal cases still converge reliably.

---

## 21. Extending the program

### 21.1 Add a demonstration

Add a new `Scenario` inside `demonstration_scenarios()` and include it in the returned dictionary:

```python
my_demo = Scenario(
    name="My demonstration",
    # configuration here
)

return {
    "baseline": baseline,
    # existing demonstrations
    "my_demo": my_demo,
}
```

The new dictionary key will automatically become a valid `--demo` choice.

### 21.2 Add a validation scenario

Add a `ValidationCase` inside `validation_cases()`. Each check consists of a description and a function returning `True` or `False`.

Example:

```python
ValidationCase(
    scenario=my_scenario,
    checks=(
        (
            "small final error",
            lambda result: abs(result.metrics.steady_state_error) < 0.02,
        ),
        (
            "no solver failures",
            lambda result: result.metrics.solver_failures == 0,
        ),
    ),
)
```

### 21.3 Use the controller in another simulation

The main reusable classes and functions are:

- `FOPDTProcess`
- `Scenario`
- `SISOMPC`
- `run_simulation()`
- `calculate_metrics()`
- `plot_result()`
- `run_self_tests()`

When using `SISOMPC` directly, preserve the program's timing convention: supply the current measurement, calculate the current command, and then advance the plant one interval.

---

## 22. Limitations

The current program deliberately remains a compact teaching and test platform. Its main limitations are:

- It supports only SISO processes.
- The internal model is a stable linear FOPDT model.
- Fractional dead time is approximated by linear interpolation.
- The output-bias estimate is assumed constant over each future horizon.
- It does not contain a Kalman filter or general state observer.
- It uses a general numerical optimiser rather than a dedicated quadratic-programming solver.
- Actual output constraints can still be violated by an unpredicted disturbance, noise or serious model mismatch even when predicted hard constraints are satisfied.
- It does not include plant communications, controller execution scheduling, signal validation, manual/automatic transfer logic, safety interlocks or operator-interface functions.
- It does not establish formal closed-loop stability for arbitrary tuning and model mismatch.
- The economic objective is generic and is not yet expressed in actual financial or production units.

The program should therefore be treated as a sound simulation framework and development starting point, not as a finished industrial APC product.

---

## 23. Glossary

| Term | Meaning |
|---|---|
| APC | Advanced Process Control |
| FOPDT | First Order Plus Dead Time |
| MPC | Model Predictive Control |
| SISO | Single Input, Single Output |
| Prediction horizon | Number of future output samples predicted |
| Control horizon | Number of independently optimised future input moves |
| Manipulated variable | Process input changed by the controller |
| Controlled variable | Process output the controller attempts to regulate |
| Model mismatch | Difference between the real plant and controller model |
| Output bias | Difference between the measured output and internal-model output |
| Hard constraint | Constraint that the optimiser is not permitted to relax |
| Soft constraint | Constraint that can be relaxed through a penalised slack variable |
| Slack variable | Non-negative variable measuring soft-constraint relaxation |
| Warm start | Initialising an optimisation with the shifted previous solution |
| Economic MPC | MPC that trades normal control objectives against an economic benefit |

---

## 24. Recommended first experiments

For a new user, the following sequence is recommended:

1. Run `python test_mpc.py --self-test`.
2. Run and study the `baseline` demonstration.
3. Compare `baseline` with `mismatch`.
4. Examine the estimated bias in the `disturbance` demonstration.
5. Study the predicted trajectories and active limit in `constraint`.
6. Compare tracking and economic objectives in `economic`.
7. Copy the custom-scenario example and modify one parameter at a time.
8. Compare the printed metrics rather than judging performance from the graph alone.

This progression makes it easier to distinguish the effects of tuning, constraints, disturbances and model mismatch.
