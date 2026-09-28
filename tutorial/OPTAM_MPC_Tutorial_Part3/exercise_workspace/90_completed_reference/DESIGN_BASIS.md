# Nonlinear heat exchanger: engineering design basis

Illustrative engineering assumptions, not a fitted or plant-validated model.

## Process and variables

A liquid stream splits into a heated branch and an unheated bypass. Each branch
has a valve. The streams rejoin; total flow and mixed temperature are measured
and controlled. Inlet temperature and upstream pressure are measured before the
split. Hot-side conditions and downstream pressure are assumed constant.

| Role | ID | Unit | Nominal | Application range |
|---|---|---|---:|---|
| CV | flow | m3/h | 60 | Protected 45–75; model domain 30–90 |
| CV | temperature | degC | 64 | Protected 55–75; model domain 45–85 |
| MV | hx_valve | % position | 50 | 35–65; max move 1 percentage point/sample |
| MV | bypass_valve | % position | 50 | 35–65; max move 1 percentage point/sample |
| DV | inlet_temperature | degC | 40 | 37–43 |
| DV | upstream_pressure | barg | 5 | 4.6–5.4 |

Nominal branch flows are each 30 m3/h. Downstream pressure is fixed at 3 barg;
nominal differential pressure is therefore 2 bar. Effective hot-side temperature
is 100 degC. Constant density and heat capacity, no phase change, no mixing heat
loss and negligible mixing holdup are assumed. A 1% MV move means one absolute
percentage point, not one percent of the current position.

## Equations the APC engineer supplies

Let `h = hx_valve/50`, `b = bypass_valve/50`, and
`s = sqrt((upstream_pressure - 3)/2)`. Then:

```
qh = 30 * (h + c*(h - 1)^2) * s
qb = 30 * (b + c*(b - 1)^2) * s
Qeq = qh + qb
Theated = 100 - (100 - Tin) * exp(-A/qh)
Teq = (qh*Theated + qb*Tin)/(qh + qb)
```

Here `c=0.1` and `A=30*ln(5)=48.28313737302301 m3/h`. The exponential is an
idealised constant-hot-temperature heat-exchanger relation: A represents a
lumped heat-transfer capacity expressed in flow units. It gives nominal
heated-branch temperature 88 degC and mixed temperature 64 degC.

The valve characteristic is illustrative, smooth and monotone over the stated
35–65% range. Its curvature adds at most 0.009 to the normalised valve-flow
factor over that interval. Pressure uses the physical square-root relationship;
the 4.6–5.4 barg range changes the pressure factor only from about 0.894 to 1.095.
Thermal mixing and heat transfer remain genuinely nonlinear. Keeping operation
near nominal makes the nonlinear effects modest; no claim of severe
nonlinearity or superior performance over linear MPC is needed for this example.

The two output dynamics are:

```
flow_next = af*flow + (1-af)*Qeq,       af = exp(-dt/10)
temperature_next = at*temperature + (1-at)*Teq, at = exp(-dt/40)
```

All times are seconds, with dt=5. These are assumed lumped first-order dynamics,
not a full transient mass/energy-balance model. The algebraic branch flows are
equilibrium quantities used to calculate output targets, not additional measured
states. No explicit transport delay is included; a change first appears in the
next sampled output. The engineer should replace the lags/model if a real plant
requires more detailed dynamics.

## Relation to the previous linear example

The operating point, signals and physical steady-state basis are retained.
The added valve curvature has zero first derivative at nominal, so the local
steady gains there remain:

| Input | Flow gain | Temperature gain |
|---|---:|---:|
| hx_valve | 0.6 m3/h per percentage point | 0.04686745 degC per percentage point |
| bypass_valve | 0.6 m3/h per percentage point | -0.24 degC per percentage point |
| inlet_temperature | 0 | 0.6 degC/degC |
| upstream_pressure | 15 m3/h/bar | -2.41415687 degC/bar |

Unlike the earlier seven-path linear model, this starter uses two output lags
and no path-specific delays. The temperature target excursion is 64→66→64 degC
instead of 64→67→64, and MV bounds are narrowed from 25–75% to 35–65%. This is
an intentionally small nonlinear engineering example, not an apples-to-apples
linear-versus-nonlinear performance experiment.

## Independent numerical checks

The application includes fixed expected next-sample values. With both valves
at 50%, pressure 5 barg and Tin changed from 40 to 37 degC, heated temperature
is `100 - 63/5 = 87.4`; mixed equilibrium is `(87.4+37)/2 = 62.2` degC.
Starting at 64 degC, the next temperature is therefore
`64 + (1-exp(-5/40))*(62.2-64) = 63.78849442465227` degC.
Flow remains 60 m3/h. This can be checked with a calculator independently of
the supplied Python implementation. Other checks cover both valves, upstream
pressure and unchanged nominal operation. The tests also check non-additive
response and pressure-dependent gain.

## Controller and acceptance settings

Prediction/control horizons are 30/10 samples (150/50 seconds). Both CV weights
are 1; normalisations are 5 m3/h and 2 degC. Both MV move weights are 0.5 and
move normalisations are 1 percentage point. Protected-limit weights are 1000,
normalisations 2 in the respective CV units. MV bounds and move limits are hard;
protected CV limits are soft penalties, checked in the resulting trajectory.

The combined scenario checks all CV/MV ranges, all MV moves, each settled target
stage and final errors. Fixed-target disturbance and mismatch cases additionally
require max absolute flow/temperature error ≤0.1 over samples 300–360 inclusive
(the final five simulated minutes). Final-point success alone is insufficient.

## Separate plant and modest mismatch

Nominal plant and controller use separate instances of the same model file and
the same numerical parameters. This is a software/engineering demonstration,
not independent evidence of physical accuracy.

In mismatch.toml only the plant changes: flow lag 10→12 seconds, temperature
lag 40→48 seconds and valve curvature 0.1→0.2. The controller retains nominal
values and the initial point remains stationary. The mismatch is deliberately
modest; nominal and mismatch cases use the same acceptance tolerances. Passing
it is not a robustness certificate for other parameter changes or a real plant.
