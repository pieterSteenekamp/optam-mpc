# OPTAM-MPC canonical development line v0.6.0

This is the single source of truth used to build the separate linear and
nonlinear APC-engineer starter packages. Frozen v0.5.0 linear, v0.1.0
nonlinear and v0.2.0 HX-bypass baseline releases remain unchanged.

## Authority and release structure

- `src/optam_mpc`: application-independent NMPC engine.
- `src/optam_mpc_linear`: declarative gain/lag/dead-time/integrator adapter.
- `product_sources/linear`: linear-only launchers, templates, examples and manual.
- `product_sources/nonlinear`: nonlinear-only launchers, templates, examples and manual.
- `release_builder`: deterministic clean assembly from the same canonical core.
- `tests`: canonical-core and product regression tests.

The SISO FOPDT work and multivariable linear work are one linear engine, not
parallel codebases. A first-order path with one CV and one MV is simply the
SISO case of the same declarative path model.

## Deliberate compatibility decision

The former two-MV-only `common_move_*` and `differential_move_*` fields are
not part of this new line. Use explicit `[[move_combinations]]` entries with
one coefficient per MV. This makes the behavior visible and dimension
independent. For two MVs, common and differential combinations are `[0.5,
0.5]` and `[0.5, -0.5]` respectively.

Online objective changes are CasADi runtime parameters. CV weights, CV rate
weights, target values, individual MV move weights and declared combination
weights no longer rebuild the NLP or discard its warm start.

## Archived HX model note

The old frozen HX-bypass demonstration is retained as historical evidence and
is not silently modified. Its controller-side effectiveness expression was
not structurally bounded. The current nonlinear starter uses the bounded
first-principles relation `exp(-A/q)` within a validated positive-flow domain,
so the old expression is not inherited.

## Build

Run `python release_builder/build_releases.py`. The builder excludes Python
caches, bytecode, virtual environments and transient run-result folders,
writes a SHA-256 manifest, orders files consistently and fixes ZIP timestamps.
