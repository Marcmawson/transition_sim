# Phase Transition Simulation

Unity project for a physically grounded phase-transition simulation. The first experiment is an open glass beaker containing ice initially at -50 C, heated from below through melting, liquid heating, boiling, and vapour formation.

Physical state and material inputs are intended to use SI units. Numerical resolution and time-step controls are solver settings, not substitutes for physical parameters.

The repository is being initialized from the existing Unity project baseline; the beaker model will be developed as a separate scene and solver path so legacy scenes are preserved.

## Phase-change prototype

The first model code is in `Assets/Scripts/PhaseChange`. Once Unity finishes compiling, use **Physics Sim → Create Phase Change Beaker Scene** to create and open the prototype scene. Press Play to see the SI energy ledger and the separate lattice-gas teaching view. The current implementation is explicitly an equilibrium enthalpy demonstration, not a spatially resolved heat/fluid simulation; see `Assets/Scripts/PhaseChange/ModelNotes.md` for assumptions and limits.
