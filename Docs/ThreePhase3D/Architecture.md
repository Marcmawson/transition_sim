# Actual implementation and field ownership

This file describes code that exists, separately from the target in Specification.md.

## S00 apparatus and configuration

Assets/Scripts/ThreePhase3D is independent of both prototype solvers. ScenarioConfigSI owns separate serialisable physical, material-reference, numerical and display input groups. Physical calculations use double SI values. One Unity unit represents one metre. The apparatus preview rejects a non-identity transform.

The default is an **empty synthetic cylindrical beaker**, not a measured apparatus. The cavity radius is 0.06 m, cavity height 0.16 m, wall/base thickness 0.003 m, and external base height 0.12 m. These are declared geometry inputs, not material constants. No pouring spout is present. A closed glass surface includes the base, inner/outer walls and rim. A tripod ring meets the external base, with three supporting legs. The camera and overhead display light are saved in Beaker3D.unity.

GeometryInputs is the authoritative dimensional description. ApparatusSurface creates its polygonal surface; both the preview renderer and static MeshCollider consume that surface. S03 must derive thermal/cut-cell geometry from the same surface/description. A preview collider is not a cut-cell or thermal discretisation. Display tessellation is distinct from physical grid resolution.

Generated preview objects, materials and meshes belong to Beaker3DSetup and are released on disable. They are not saved into legacy scenes. Lighting is visual only.

Default reference entries identify water, humid-air, DURAN and tripod data but supply **no implemented property providers**. Missing ranges, hashes or data block physical readiness. No DURAN heat capacity is invented. Synthetic geometry permits an empty preview, not missing thermodynamics. Mass/volume consistency accepts a density delegate for independent dimensional tests before S02 provides material models.

## Ownership for subsequent stages

There are currently no 3D field buffers and no physical timestep. S03 will own cell-centred phase masses, thermodynamic/species state, cut-cell geometry and separate x/y/z-face momentum. Transport owns common conservative face fluxes. Flow, thermal, interface and body solvers produce candidate states. Only the step controller may commit a converged, accepted coupled step. Diagnostics and rendering read the accepted state.

S01 adds conservation bookkeeping and run evidence, as described below. CPU diagnostics are permitted; continuum field transport and iterative solves remain GPU requirements for S04 onwards. No 2D buffer or test result is inherited as 3D verification.

## Validation workflow

Use Physics Sim > Three Phase 3D > Open empty Beaker3D. Run foundation checks executes dimensional/configuration/surface tests. Export pre-run report writes configuration and actual Unity graphics information under ValidationRuns/local/preflight-<UTC>/; this is not a physical run.

Tools/ThreePhase3D/RunChecks.ps1 -CompileUnitySources compiles against installed Unity assemblies and executes pure C# checks without requiring an activated Editor. It cannot establish import, rendering or GPU execution. This custom runner needs no Test Framework package, Editor upgrade or pipeline change.

The full S00 gate remains pending until V36 succeeds in Unity.

## S01 accounting and evidence

ConservedInventory owns compensated FP64 channel accumulators. InventoryState is a detached snapshot containing phase-resolved water mass, dry-air mass, fluid/ice internal energy, resolved kinetic energy, gravitational energy, fixed-solid internal energy, interface energy and linear/angular momentum. A paired transfer checks totals and both candidate inventories before committing either owner. A transfer packet is bookkeeping; it does not compute latent heat, interface motion or dynamics.

ConservationLedger owns external channels only. Heater, support conduction, radiation, mechanical work, open-boundary transport, injection and numerical corrections are distinct. Absorbed heater energy is a labelled subset of total energy input. Heat/work channels carry no mass; advected energy may have a signed thermodynamic reference. Any nonzero numerical correction invalidates the S01 check even if it makes the algebraic residual zero.

RunRecorder owns a cloned manifest, the external ledger and output writers. Independently reduced current inventory is supplied at accepted times. It never changes the physical state. The run has a unique directory, initial metadata, CSV series, transfer and boundary logs, snapshots, reference inputs, summary, final manifest and a manifest hash. Disposal before completion leaves an interrupted, unsealed record that cannot pass the evidence gate. The file API does not overwrite a completed run. SHA-256 detects alteration; it is not an authenticity signature.

EvidenceValidator recomputes water/air/energy residuals and paired transfer closure from CSV, checks recorded metric thresholds, hashes and the dependency set, and resolves evidence-index requirement IDs, source IDs, benchmark IDs and actual compiled code symbols. Function-contract checks establish structure and identity only. A requirement marked verified/validated needs an explicit accepted review and linked clean run; passing syntax checks does not promote a requirement automatically.

S01 executes only a declared synthetic bookkeeping fixture. It uses no EOS, physical phase-change law, continuum fields, GPU reductions, body mechanics or parcel dynamics. The beaker scene remains an apparatus preview. Unavailable temperatures, pressures, entropy and GPU timings remain blank in CSV. The metadata identifies the apparatus asset as an associated scene, not an executed fluid case. The S01 fixture and thresholds are versioned under Tools/ThreePhase3D/Fixtures.

Source text hashes use UTF-8/LF identity to tolerate Windows checkout line endings. Binary tables use byte hashes. Run outputs use exact byte hashes, and .gitattributes disables newline conversion for committed ValidationRuns. Source dependency hashes exclude the evidence index itself, allowing a separate review/evidence commit without a circular hash dependency.
