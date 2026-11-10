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

S01 adds conservation bookkeeping and run evidence. It is unfinished in this proposal. CPU diagnostics are permitted; continuum field transport and iterative solves remain GPU requirements for S04 onwards. No 2D buffer or test result is inherited as 3D verification.

## Validation workflow

Use Physics Sim > Three Phase 3D > Open empty Beaker3D. Run foundation checks executes dimensional/configuration/surface tests. Export pre-run report writes configuration and actual Unity graphics information under ValidationRuns/local/preflight-<UTC>/; this is not a physical run.

Tools/ThreePhase3D/RunChecks.ps1 -CompileUnitySources compiles against installed Unity assemblies and executes pure C# checks without requiring an activated Editor. It cannot establish import, rendering or GPU execution. This custom runner needs no Test Framework package, Editor upgrade or pipeline change.

The full S00 gate remains pending until V36 succeeds in Unity.
