# Unity three dimensional water simulation specification

Revision 1.0 · 10 November 2026 · Status: implementation specification, awaiting implementation and validation.

This pack specifies a GPU simulation of a glass beaker on a tripod, heated beneath the centre, containing any combination of ice Ih, liquid water and water vapour. It is intended for the programmer AI implementing the simulator and the reviewer checking its physics. The user runs the Unity scenes and supplies the numerical evidence.

The recommended foundation is a conservative three dimensional finite volume solver: staggered velocities, geometric volume of fluid interfaces, humid gas transport, conjugate solid conduction, and conservative phase change. Moving ice requires an additional coupled rigid body and melting boundary treatment. Unity supplies orchestration, compute shaders, rendering and interaction.

Implementation checkpoints: [S05 geometric transport](S05-Geometric-Transport.md) records the periodic liquid–gas transport and its Unity checks. [S04 single-phase flow](S04-Single-Phase-Flow.md) contains the projection and momentum benchmarks and Unity commands. [S03 grid and cut-cell geometry](S03-Grid-Geometry.md) contains the current grid checks and Unity preview/GPU commands. [S02 expanded materials](S02-Expanded-Materials.md) records the material providers and deferred transport limits. These checkpoints distinguish implemented code, numerical evidence and outstanding execution gates.

## Read in this order

1. [Specification](Specification.md) — scope, governing equations, numerical architecture and physical interfaces.
2. [Sources](Sources.md) — selected methods, primary references, alternatives, and evidence limitations.
3. [Implementation plan](ImplementationPlan.md) — stages with deliverables, dependencies and acceptance gates.
4. [Validation](Validation.md) — numerical verification, experiments and quantitative acceptance criteria.
5. [Evidence protocol](EvidenceProtocol.md) — function comments, immutable run evidence, Git review and local execution.
6. [Requirements](Requirements.csv) and [material register](Materials.csv) — traceability and data acquisition.
7. [Templates](Templates) — copyable contracts and run records.

Give the programmer AI the whole directory. Start with stages S00 and S01. Each subsequent stage must include executable validation scenes and numerical outputs; completion comments alone do not establish correctness.

## Repository baseline

Repository: [Marcmawson/transition_sim](https://github.com/Marcmawson/transition_sim).

The inspected main commit was **e51635ca16160a9afbf4570b2f9e3512fb831b8a**, tree **525ea7c2e636f41c1bc84c20ba2fc907ad922ef0**. This is a reproducible baseline, not a claim about later changes.

Verified against that commit:

- ProjectSettings/ProjectVersion.txt specifies Unity **2022.3.46f1**.
- ProjectSettings/GraphicsSettings.asset has no custom render pipeline assigned.
- README.md and Assets/Scripts/PhaseChange/ModelNotes.md describe a 2D thermal and flow prototype with approximate flotation and bubble transport.
- Assets/Scripts/FluidFlow/LiquidFlowMac.compute stores two face velocity components and a 2D output texture. It is not the proposed 3D multiphase solver.
- ModelNotes.md explicitly records missing full gas flow, glass heat capacity and ambient exchange.
- No repository AGENTS.md was present in the inspected recursive tree. The user's conversation instructions still apply.

Preserve the existing scenes as prototypes and comparison material. Put new runtime work under Assets/Scripts/ThreePhase3D, with new scenes and tests. Do not relabel the existing demonstration as quantitatively validated.

## Meaning of accuracy

Accuracy is established for a declared physical regime and a specified quantity, with property uncertainty, numerical convergence and experimental comparison. This pack does not claim that any implementation has already passed those gates.

Numerical tolerances and laboratory test conditions are design choices. Material properties are sourced constitutive data. Burner coupling, geometry, surface condition and ambient conditions are apparatus inputs that require specification or measurement. These categories must remain distinguishable.

The first supported regime is an open laboratory beaker, near atmospheric pressure, ordinary hexagonal ice, liquid water, vapour and non-condensable air, at speeds compatible with a validated low Mach approximation. Every requested core phenomenon remains in the implementation plan. Combustion chemistry, shocks, glass fracture and ice fracture require further models before results involving them can be called accurate.

The project prioritises measured physical fidelity over real time playback. GPU acceleration is required; real time speed is not promised before the user's GPU, VRAM and benchmark timings are known.

## Known information still required

The programmer can begin foundations and analytical tests immediately. Before a quantitatively validated beaker run, record:

- GPU model, VRAM, driver, operating system and graphics API.
- Beaker dimensions including base thickness, glass grade, tripod material and contact geometry.
- Burner absorbed heat flux versus radius and time, with uncertainty.
- Material data missing from Materials.csv, especially glass heat capacity over the actual temperature range.
- Ambient pressure, temperature, humidity and ventilation.
- Glass wetting measurements and a declared nucleation or seeded bubble protocol.
- Which physical observables must match an actual apparatus and their measurement uncertainty.

Use explicit synthetic benchmark conditions until those inputs exist. Label those runs as verification experiments rather than replicas of a measured apparatus.
