# Heat exchanger with bypass — linear engineering design basis

This is an illustrative heating service, not data from a commissioned plant.
Both the controller and digital twin execute linear models. The equations below
are used only to derive local gains; they are not evaluated inside the runtime.

## Arrangement and assumptions

One liquid stream splits into an exchanger branch and a bypass branch, each
with its own valve. They recombine before the outlet flow and temperature
measurements. Inlet temperature is measured before the split. Upstream pressure
is measured on the common supply. Downstream pressure is held approximately
constant; density and heat capacity are constant and equal in both branches.

| Quantity | Nominal assumption |
|---|---:|
| Exchanger valve | 50% |
| Bypass valve | 50% |
| Exchanger branch flow | 30 m3/h |
| Bypass branch flow | 30 m3/h |
| Combined flow | 60 m3/h |
| Process inlet temperature | 40 degC |
| Effective hot-side temperature | 100 degC, constant |
| Exchanger branch outlet temperature | 88 degC |
| Mixed outlet temperature | 64 degC |
| Upstream pressure | 5 barg |
| Downstream pressure | 3 barg, constant |
| Pressure difference | 2 bar |
| Valve operating bounds used by MPC | 25–75% |
| Valve move bound | 1 percentage point per 5-second sample |

The hot side is represented by a constant-temperature source with sufficient
capacity. Pipe heat loss, density changes, valve stiction and varying downstream
pressure are omitted. These assumptions must be revisited for an actual plant.

## Local gain calculation

Let h and b be valve positions in percent, P the upstream pressure in barg,
Ti the process inlet temperature, and Qh/Qb branch flows in m3/h:

$$
Q_h=30\frac{h}{50}\sqrt{\frac{P-3}{2}},\qquad
Q_b=30\frac{b}{50}\sqrt{\frac{P-3}{2}}.
$$

This is a simple local linear-valve hydraulic assumption. Actual installed
valve characteristics and piping pressure drops can differ substantially.

Use an idealised constant-hot-temperature exchanger:

$$
T_h=100-(100-T_i)\exp(-A/Q_h),\qquad A=30\ln(5).
$$

A has the same flow units as Qh and represents UA divided by volumetric heat
capacity in compatible units. At nominal flow the exponential is 0.2, so
Th=88 degC. Complete mixing gives:

$$
Q=Q_h+Q_b,\qquad
T=\frac{Q_hT_h+Q_bT_i}{Q_h+Q_b}=64\;\mathrm{degC}.
$$

Partial derivatives at the nominal point give the following linear model:

| Source | Destination | Steady gain | Tau (s) | Dead time (s) |
|---|---|---:|---:|---:|
| hx_valve | flow | 0.6 (m3/h)/% | 10 | 0 |
| bypass_valve | flow | 0.6 (m3/h)/% | 10 | 0 |
| hx_valve | temperature | +0.04686745051 degC/% | 45 | 5 |
| bypass_valve | temperature | -0.24 degC/% | 35 | 5 |
| inlet_temperature | temperature | +0.6 degC/degC | 25 | 15 |
| upstream_pressure | flow | +15 (m3/h)/bar | 10 | 0 |
| upstream_pressure | temperature | -2.414156869 degC/bar | 40 | 5 |

Inlet temperature to flow is zero under the constant-property assumptions and
is deliberately omitted, not silently forgotten. There are seven nonzero paths,
not eight mandatory paths.

Opening the exchanger valve raises the hot-branch fraction but also reduces its
residence time and outlet temperature. The net temperature gain is therefore
small and positive at this operating point. Opening the bypass cools the mix.
Increasing upstream pressure raises both flows and reduces exchanger heating
per unit mass, hence the negative pressure-to-temperature gain.

For checking the derivatives:

- dT/dQh = 0.4 - 0.2 ln(5) = 0.07811241751.
- dT/dQb = -0.4.
- dQh/dh = dQb/db = 0.6.
- dQh/dP = dQb/dP = 7.5.
- dT/dTi = 0.6.

The regression tests independently differentiate the physical equations
numerically and compare all configured gains.

## Dynamic interpretation

Every listed path uses G(s)=K exp(-theta s)/(tau s+1).
The lags and delays are illustrative lumped engineering assumptions representing
actuator, transport, thermal storage and measurement dynamics. They are not
deduced from the steady-state gain calculation or presented as identified data.

The longer inlet-temperature transport delay allows measured disturbance
feedforward to act before the outlet temperature fully responds. This is not
future knowledge: only the current inlet measurement is given to the MPC.

All models use deviations from the nominal values. A constant change produces
a finite steady output change; there are no integrating paths in this example.
Valve percentages are percentage points, not fractions between zero and one.

## Tuning and feasibility

Sample time is 5 s; prediction horizon 30 samples (150 s); control horizon
10 samples (50 s). Both CV weights are 1. Flow normalization is 5 m3/h and
temperature normalization 2 degC. Both valve move normalizations are 1%.
Protected CV limits are 45–75 m3/h and 55–75 degC with penalty weight 1000.
These are soft predicted limits, not safety trips.
Nominal-model tuning uses a move weight of 0.05 for each valve. The supplied
detuned mismatch configuration uses 0.5 for each valve, with all other model
and tuning settings unchanged.

The steady MV gain matrix is nonsingular. For example, increasing mixed
temperature by 3 degC at unchanged flow requires approximately equal and
opposite valve changes of 10.46 percentage points, within the stated bounds.
That does not establish feasibility for arbitrary combined demands.

The linearisation is local. Bounds and example changes are reasonable test
assumptions, not proof that these gains hold across that whole range.
An actual engineer must establish an acceptable operating region.

## Mismatch challenge

In mismatch.toml, every nonzero plant gain is multiplied by 1.15, every lag by
1.20 and every dead time is increased by 5 s. The controller model is unchanged.
The non-integrating plant remains linear.

This deliberately tests errors in gain, lag and transport timing together.
A final-value-only check can miss sustained oscillation. Acceptance therefore
requires both CVs to remain within 0.1 engineering units of target for the
entire final five-minute window, as well as meeting operating and move bounds.
