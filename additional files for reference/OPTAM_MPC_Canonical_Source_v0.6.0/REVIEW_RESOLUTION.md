# Independent review resolution

| Review point | Resolution in the consolidated line |
|---|---|
| Two-MV hardcoded common/differential penalties | Replaced with named move combinations having an explicit coefficient for every MV; verified for 1, 2 and 3 MVs. |
| Unbounded old HX effectiveness expression | Frozen demo remains unchanged and archived. The current nonlinear example uses `exp(-A/q)` inside a validated positive-flow domain and does not inherit that expression. |
| `update_tuning()` rebuilds the NLP | CV weights, CV rate weights, targets, MV weights and move-combination weights are runtime parameters; the solver object and warm start are retained. |
| 3.54 °C versus 3.60 °C documentation | The supplied frozen archive available for this review says 3.54 °C in both named documents, so the claimed internal mismatch could not be reproduced. New results are generated from `report.json` with unrounded values and source hashes to prevent manual drift. |
| Empty `__pycache__` | Release builder excludes caches and bytecode. |
| SISO FOPDT overlap | Explicitly converged into the same declarative linear gain/lag/dead-time/integrator engine. |
| Copied core between products | Separate user packages are now generated from one canonical `src` tree by a deterministic release builder. |

The frozen packages are evidence baselines. No finding was corrected by editing
those archives in place.
