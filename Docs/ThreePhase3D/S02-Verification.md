# S02 checkpoint verification — 10 November 2026

Status: **development checkpoint, S02 incomplete, review pending**.

Published source checkpoint: [`ee40c3b7e742cb053a0541cecaa779154e2ca159`](https://github.com/Marcmawson/transition_sim/commit/ee40c3b7e742cb053a0541cecaa779154e2ca159). All 29 published changed/new paths were checked against their local Git blob hashes after saving; zero mismatches. Later commits attach this report and evidence without changing the tested material code.

The local source was reconstructed through the GitHub connector because direct Git networking was unavailable. Its local snapshot history is not the GitHub history. This is not represented as a clean release run at a fictitious local Git SHA. The material report records its S01 base, canonical source SHA-256 fingerprints and the executable hash; published Git blob identities are recorded separately in the evidence directory. Acceptance remains pending.

## Measured results

| Check | Result | Limit |
| --- | --- | --- |
| S02 material suite | **2539 comparisons, 0 failures** | Official reference values, derivative checks, validity failures and 97 interior FP32 table probes per phase, with 12 properties each |
| Existing S01 suite | **25 check groups, 0 failures** | Development run after current evidence-index freshness was reconciled; original S01 history remains preserved |
| C# compilation | **Pass** | All ThreePhase3D C# compiled against installed Unity 2022.3.46f1 reference assemblies; not an Editor import/player build |
| Whitespace diff check | **Pass** | `git diff --check` |
| GPU/Editor execution | **Not established** | Batch attempts produced licence-client connection-refused startup messages, no GPU result bundle, and no confirmed shader execution |

Material run: `s02-materials-20261110T224300-a813190dc3674a47a289a3dcc316e39f`.

Published artifacts: [ValidationRuns/s02-materials-checkpoint-20261110](../../ValidationRuns/s02-materials-checkpoint-20261110/). This contains machine-readable comparisons with expected/actual/error/tolerance/unit/source, source fingerprints, executable identity, generated tables, a property plot and the captured Editor startup log. `files.sha256` is an integrity inventory, not a signature or an independent physical validation.

The passing table run uses liquid spacing 0.125 K and vapour spacing 0.25 K. The first liquid spacing (0.25 K) failed four density-temperature-derivative checks near the density maximum. Refinement removed those failures without changing tolerances. Reports of that failed development attempt remain locally in `ValidationRuns/local/`; the attached evidence is the final passing run.

## Numerical criteria and interpretation

Official values were transcribed from the primary release tables listed in [the handoff](S02-Handoff.md), with explicit SI conversion. Comparisons retain tolerances for each quantity. Rounded G8-10 composition/density inputs propagate rounding into outputs; those checks use declared absolute tolerances, not an assertion that every printed digit is exact experimental data. Table 6 water derivative checks use sums of the published rounded ideal and residual entries. Mathematical identities and finite differences provide additional checks, not an independent experimental dataset.

FP32 interpolation criteria use the project's 1e-4 relative target, with the specified h/u floor of 1 J/kg and entropy floor of 0.001 J/(kg K). The density-temperature derivative crosses zero, so its preregistered numerical absolute floor is 2e-6 kg/(m3 K). Other properties use a 1e-14 SI floor. Tests query interior temperatures rather than merely re-reading generator nodes. CPU float emulation passing does not establish HLSL compiler/driver behavior.

The same-mass ice/liquid volume check is material arithmetic, not a moving-body flotation test. Corrected ice-conductivity endpoint checks verify coefficient/unit transcription; they do not validate a different freezing preparation. The diffusion pressure-scaling test verifies the selected correlation, not a hot plume. The gas-only production validity margin is deliberately conservative and is not the pending chemical-equilibrium solve.

## Pending gates

V03–V05 and V32 have partial supporting evidence; they are **not accepted in full**. Humid-mixture viscosity/conductivity, near-saturation fugacity/equilibrium, broader diffusion/ice-conductivity coverage, multidimensional/ice/gas GPU tables and actual GPU execution remain open. The existing empty beaker is still the expected visible scene at this material-only stage.

Run the two S02 commands in the Unity menu described in [S02-Handoff.md](S02-Handoff.md). A successful local GPU bundle is required before GPU correctness can be reported.
