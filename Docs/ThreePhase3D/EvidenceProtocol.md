# Code comments Git review and run evidence

Revision 1.0 · 10 November 2026

## 1 Evidence hierarchy

A function comment is a traceable engineering claim. The reviewer verifies that claim against the source code, its actual callers and GPU bindings, discrete conservation, automated tests and the user's local run outputs. A comment that says implemented or accurate without evidence is insufficient.

Use the following statuses distinctly:

| Status | Meaning |
| --- | --- |
| SPECIFIED | Requirement and model are described |
| IMPLEMENTED_UNVERIFIED | Code exists, with no accepted numerical evidence |
| VERIFIED_NUMERICALLY | Specified analytical/convergence gates passed |
| VALIDATED_FOR_CASE | A stated experimental comparison passed |
| INVALIDATED | A later change or failed evidence has removed the previous claim |
| BLOCKED_DATA | Necessary physical input or property data is missing |

A function may be numerically verified without the complete beaker experiment being validated. Never equate compilation, visible motion, source-comment coverage and physical validation.

## 2 Comments required beside code

Apply the contract to physics kernels, constitutive models, flux/phase transfer routines, boundary conditions, ice mechanics, numerical stabilisation, conservation reductions and changes of representation. Simple UI plumbing need not repeat a full physics contract.

Required fields:

- Physics-ID: stable unique function or kernel ID.
- Implements: one or more requirement IDs from Requirements.csv.
- Model: equation or section in Specification.md.
- Sources: source IDs plus exact equation/table/section in the pinned source version.
- Inputs-SI and Outputs-SI: variables and units, including per-volume versus per-area versus total quantities.
- State-Ownership: which buffers or body inventories can change.
- Conservation: paired transfers, boundary work, reference conventions.
- Discretisation: reconstruction, stencil, time level and iteration/convergence rule.
- Validity: supported states and approximations.
- Validation: benchmark IDs.
- Evidence: latest accepted record path, or PENDING.
- Status: one of the statuses above.
- Limitations: known unresolved concerns.

Do not copy an equation number from a different paper version. Source locators are added after opening the full pinned source, not guessed from an abstract.

An outcome note records the test configuration, metric definition, measured result, threshold, run ID and evidence file. Until a run exists, the only valid outcome is PENDING. See [the comment template](Templates/FunctionPhysicsContract.txt).

Comments should explain purpose, units, invariants and the reasoning behind nonobvious numerical choices. They must not duplicate long benchmark data arrays or be the sole place results are stored.

## 3 Actual outcomes live in immutable run records

Each run uses a unique ID and records:

- Git commit SHA, dirty-tree status and patch hash if applicable.
- Built executable hash, Unity version, active scene GUID and scene configuration hash.
- Operating system, GPU, VRAM, driver and graphics API.
- Physical dimensions, initial conditions, ambient T/p/humidity and material IDs.
- Material and lookup-table hashes, generator version and property reference conventions.
- Grid hierarchy, cell sizes, cut-cell rules, timestep policy and solver tolerances.
- Approximation flags, active closures and unresolved-scale indicators.
- Random seeds, parcel multiplicities, nucleation/seed conditions and event ordering.
- Requested versus simulated versus elapsed wall time.
- Diagnostic sampling intervals in physical seconds.
- Failure flags, rejected steps and boundary conditions.
- Reference dataset version/hash and experimental uncertainty.

A run is reproducible only when the executable, inputs and reference data are identifiable. A branch name alone is mutable and insufficient.

Use an accepted clean commit for final evidence. Development runs with a dirty tree may be useful, but must include the patch and are not final release evidence.

## 4 Files recorded per run

Recommended layout, relative to the repository or an explicitly linked immutable external evidence store:

~~~text
ValidationRuns/<run-id>/
    manifest.json
    summary.json
    timeseries.csv
    probes.csv
    phase_transfers.csv
    solver_history.csv
    field_snapshots/
    plots/
    console.log
    user_observations.md
~~~

Large evidence may use Git LFS or an external immutable artifact location. Commit its content hash, file inventory and accessible URI in the evidence index. Do not commit Unity Library, Temp, generated caches, credentials or large raw output blindly.

Minimum timeseries columns, with SI units encoded in the header:

~~~text
time_s,dt_s,m_ice_kg,m_liquid_kg,m_vapour_kg,m_parcel_water_kg,
m_dry_air_kg,m_water_in_kg,m_water_out_kg,water_mass_residual_kg,
internal_energy_J,kinetic_energy_J,gravitational_energy_J,
heater_absorbed_J,energy_in_J,energy_out_J,energy_residual_J,
entropy_J_per_K,entropy_in_J_per_K,entropy_out_J_per_K,
p_abs_min_Pa,p_abs_max_Pa,T_min_K,T_max_K,
max_speed_m_per_s,max_Mach,volume_constraint_linf_per_s,
pressure_relative_residual,nonlinear_relative_residual,
max_CFL,gpu_step_ms,wall_elapsed_s,valid_run
~~~

Additional interfacial, capillary or solid energy columns are required when those terms are part of the energy convention. Include a schema version so columns cannot silently change interpretation.

Log phase transfer per interface or aggregate interface type with donor, receiver, mass, energy, momentum, direction convention and time interval. In a closed pairwise transfer, donor loss plus receiver gain must cancel within the defined numerical tolerance.

Ambient inflow can contain water vapour. Count it in water_in; water_out is not automatically all evaporated initial sample. Use an optional provenance tracer if evaporated sample mass must be distinguished from background humidity.

## 5 Evidence index

Maintain Docs/ThreePhase3D/ImplementationEvidence.json during implementation, following [the example](Templates/ImplementationEvidence.example.json). Each record links:

requirement ID → actual code symbol → source locator → validation case → immutable run → review outcome.

The initial example contains only unimplemented records and no invented test results. Review may establish evidence inheritance from an earlier commit only after confirming that changes do not affect the relevant numerical path, materials, input or build configuration.

Evidence can otherwise become stale after changes to code, shader compiler options, tables, resolution, boundary conditions, geometry or test definitions. Preserve old results and add a new record; do not overwrite the historical evidence to make it appear current.

Automated checks should verify that IDs exist, symbols and paths are real, hashes match and required metrics are present. They cannot prove the physical model from syntax.

## 6 Review process

For each pull request the reviewer:

1. Reads the originating requirements and current specification.
2. Inspects the full affected code and callers at a fixed SHA.
3. Checks dimensions, physical signs, pressure/enthalpy conventions and phase mass versus volume.
4. Traces every GPU kernel's buffer binding and dispatch dependencies.
5. Checks interface exchange and external boundary fluxes for matching donor/receiver terms.
6. Reads comments and compares the stated model to the actual algorithm.
7. Examines independent reference calculations, convergence and regression evidence.
8. Compares experimental inputs and uncertainty to the validation claim.
9. Reviews user-local Console logs and the actual full-scene result.
10. Records accepted, rejected, pending-data or requires-new-run for each requirement.

Typical rejection examples include an arbitrary drag coefficient described as measured, a copied material constant outside its range, a zero-divergence projection used for boiling, mass lost through clamping, wall heat capacity omitted, a visual steam emitter with no water inventory, or a phase state changed by temperature without latent heat accounting.

## 7 Shared Git workflow

Use Marcmawson/transition_sim as the shared source of truth. Documentation changes are isolated in a documentation branch; runtime work proceeds in staged implementation branches. Preserve existing scenes.

Suggested branch names are physics3d/s00-baseline, physics3d/s01-evidence and so on. Names are organisational choices, not physical parameters.

The programmer AI posts or supplies a commit/PR containing implementation and evidence instructions. The user runs that exact version in Unity and supplies the run bundle or commits its manifest and results. The reviewer then checks the code and evidence at that SHA and records remaining failures.

Code reviews happen when a commit, PR or run is supplied. This document does not create an automatic watcher or a scheduled review service.

Do not force push shared history or merge unvalidated runtime code automatically. A proposed specification can remain reviewable while implementation starts on an appropriate branch.

## 8 Local execution by the user

The programmer must make benchmark running accessible from a Unity menu and support batch execution where the GPU/API allows it. Each benchmark gets a named scene or configuration and an export location shown in the UI.

User workflow:

1. Open the specified commit in Unity 2022.3.46f1 unless a reviewed upgrade has occurred.
2. Let imports finish; capture Console errors before running.
3. Run the supplied benchmark without changing its physical or numerical settings.
4. Confirm the displayed scene, commit and material-table identity.
5. Run to the benchmark's physical end time and export the bundle.
6. Record visible problems and any changed settings in LocalRunReport.md.
7. Supply the commit/PR and evidence location for review.

The programmer must provide the exact menu path and output path once implemented. No nonexistent menu or runtime is claimed by this specification.

## 9 Separation of calibration and validation

Calibration estimates apparatus inputs such as absorbed power, thermal contact resistance and wetting/nucleation parameters from designated observations. Validation evaluates predictions against separate data.

Keep dataset IDs and parameter-fit records. Report parameter uncertainty and sensitivity. Avoid adjusting heater efficiency, evaporation coefficients, viscosity or damping until a single desired curve matches and then claiming independent validation.

## 10 Review completion

An accepted stage has a review record listing the exact source SHA, requirements accepted, evidence hashes and limits of the conclusion. A release requires both numerical verification and the relevant experiment comparisons. Pending local execution remains pending even if an AI has read every line of code.
