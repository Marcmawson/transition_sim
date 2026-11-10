# S01 conservation and evidence handoff

Implementation branch: physics3d/s01-evidence, based on the corrected S00 scene at d073b06e0f1b8da2e889fa93050afd4066ec1b1e. No legacy solver, scene, asset reference, package or rendering setting is changed. The S00 camera/light rotation fix is retained.

## Scope and status

This stage supplies accounting and reproducible evidence infrastructure. It does not advance water, heat, interfaces or ice mechanics. The associated empty beaker should look the same. S02 material providers and S03 GPU geometry/resources remain subsequent stages.

The user reported that the beaker renders after the quaternion correction. That observation establishes visible apparatus rendering only. Exact client commit, hardware, Console baseline and the new S01 Editor menu run have not yet been supplied, so no broader V36 or physical-validation pass is inferred.

Development checks and Unity-assembly compilation passed. The attached run record, once supplied, identifies the exact clean source commit, executable hash, configuration/reference hashes and measured metrics. Requirement acceptance/review remains PENDING. Do not equate bookkeeping transfers with implemented melting, spray conversion or resolved phase flow.

## Implemented interfaces

- ConservedInventory and InventoryState: per-phase water/dry-air mass, five storage-energy terms, linear/angular momentum, atomic matched transfers and negative/nonfinite guards.
- StableReduction: compensated FP64 CPU accumulation for diagnostic partials and body totals. Actual GPU field reductions are still S03/V35 work.
- ConservationLedger: independent current-versus-initial audit, separate external heat/work/mass channels and invalidation of numerical corrections.
- RunRecorder and RunManifest: immutable-by-API run directories, typed metadata, SI CSV series, paired transfer/boundary logs, snapshots, input/reference hashes, summary and sealed file inventory.
- EvidenceValidator: required identifiers/paths/symbols, dependency and output hashes, stale-evidence rejection, independent CSV residual and transfer checks, measured metric thresholds and function-comment format.
- FoundationRun and the Unity/PowerShell entry points: run the same synthetic suite, preserve failures and print the export location.

## Equations, conventions and test inputs

The governing bookkeeping definitions are Specification.md section 14:

R_water = M_water(t) - M_water(0) - M_in + M_out.

R_energy = E(t) - E(0) - E_in + E_out.

E sums fluid/ice internal, resolved kinetic, gravitational, fixed-solid internal and interface storage. Interphase/representation transfers cancel globally. Internal-energy references and angular-momentum origin must match on both sides. Pressure work and advected enthalpy belong in the chosen external energy convention; this stage does not derive a fluid boundary flux.

The fixture starts with 1 kg liquid-labelled inventory and 0.01 kg dry air, synthetic storage values and momentum, transfers a 0.125 fraction of the water packet to a parcel-labelled inventory and back, and records 10 J of absorbed external heat. These are deliberately chosen arithmetic inputs, not empirical material properties or a thermodynamic state. No temperature increase is inferred from them.

Tolerances are preregistered numerical test criteria: 1e-12 kg mass, 1e-9 J energy, 1e-12 kg m/s momentum and 1e-12 kg m2/s angular momentum. The fixture uses supplied instants 0 and 1 s, not a fluid integration timestep. The energy check is absolute in J; a large arbitrary enthalpy reference cannot dilute its error.

Sources and implementation contracts: Specification.md sections 5, 6.4, 12.2 and 14; EvidenceProtocol.md sections 2-6; Validation.md V02/V33/V34. No constitutive coefficients or detailed stencils from uninspected publications were transcribed. FP64 compensation is checked against cancellation-sensitive sums and an independent decimal arithmetic reference.

## Verification coverage

The suite includes SI/configuration checks, matched transfers and their atomic rejection, repeated small transfers, independent loss detection, humid boundary inflow, heater/support/radiation/work accounting, numerical-correction rejection, snapshot identity, quoted CSV fields, output corruption, missing seals, stale shader/table/dependency identities, schema and metric failures, fabricated passing CSV flags, dirty-evidence rejection for clean review, interruption/path safety and evidence-index ID/symbol checks.

V34 here verifies the **bookkeeping seam** among named representations. It does not establish remeshing, moving-body coupling, dispersed droplets or a conservative resolved interface. V33 verifies recorded identity and consistency; hashes cannot prove a false supplied field is physically correct. Later numerical and experimental gates remain mandatory.

## Run locally

1. Check out physics3d/s01-evidence and allow Unity 2022.3.46f1 to compile.
2. Use **Physics Sim > Three Phase 3D > Run S01 conservation checks**. The existing Run foundation checks command runs the same suite.
3. The Console prints each result, the exact source SHA/dirty status and the output directory: ValidationRuns/local/s01-foundation-<UTC>-<unique ID>/.
4. Use **Validate latest S01 bundle** to check integrity, balances and source freshness; **Validate evidence index** checks requirement/symbol/comment links.
5. Supply the run directory and Console observations for review. Each run is separate; do not edit the recorded inputs after the fact.

Standalone PowerShell (no activated Editor required):

```powershell
powershell -File Tools/ThreePhase3D/RunChecks.ps1 -CompileUnitySources
```

To recheck a supplied record against the current dependency tree:

```powershell
powershell -File Tools/ThreePhase3D/RunChecks.ps1 -VerifyBundle "ValidationRuns/local/<run-id>"
```

Unity batch entry point remains PhysicsSim.ThreePhase3D.Editor.ThreePhase3DMenu.RunFoundationChecksBatch. This compiles/tests diagnostics; no GPU performance, player build or complete simulator claim follows. Standalone hardware fields explicitly say not queried/executed. The Unity menu records its actual device/API/compute/readback reports; VRAM is an approximate conversion from Unity's reported MB, not a measured allocation or dedicated-memory guarantee.

Development records carry tracked diffs and exact untracked owned source bytes, and are ineligible for clean release review. Synthetic unit-test fixtures use explicitly synthetic identities and are not published as real source-run evidence. Interrupted/failed artifacts remain available. A final record is sealed only after completion.

## Rollback and next stage

Switch back to physics3d/s00-baseline or revert the S01 commits. No history rewrite or scene replacement is needed. S02's independently checked material reference models are next; this stage supplies the mass/energy bookkeeping and evidence gate they can use.
