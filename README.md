# Phase Transition Simulation

Unity project for a physically grounded phase-transition simulation. The first experiment is an open glass beaker containing ice initially at -50 C, heated from below through melting, liquid heating, boiling, and vapour formation.

Physical state and material inputs are intended to use SI units. Numerical resolution and time-step controls are solver settings, not substitutes for physical parameters.

The repository is being initialized from the existing Unity project baseline; the beaker model will be developed as a separate scene and solver path so legacy scenes are preserved.

## Phase-change prototype

Open `Assets/Scenes/PhaseChange Beaker.unity` and press Play to see the SI thermal grid and the separate lattice-gas teaching view. The beaker uses a staggered-grid 2-D incompressible liquid-flow approximation with a free surface, an Archimedean ice-flotation closure, and buoyant bubble rise and venting. Full gas flow, beaker heat capacity, and ambient heat exchange remain future work; see `Assets/Scripts/PhaseChange/ModelNotes.md` for assumptions and limits.

## Three dimensional implementation specification

The new [3D water simulator specification](Docs/ThreePhase3D/README.md) defines the GPU multiphase architecture, SI material and apparatus inputs, 60 requirements, 19 implementation stages, 46 verification cases, and code-comment/run-evidence review protocol. This is a documentation handoff; it does not claim that the future 3D solver is implemented or validated.
