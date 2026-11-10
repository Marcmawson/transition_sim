# Phase Transition Simulation

Unity project for a physically grounded phase-transition simulation. The first experiment is an open glass beaker containing ice initially at -50 C, heated from below through melting, liquid heating, boiling, and vapour formation.

Physical state and material inputs are intended to use SI units. Numerical resolution and time-step controls are solver settings, not substitutes for physical parameters.

The repository is being initialized from the existing Unity project baseline; the beaker model will be developed as a separate scene and solver path so legacy scenes are preserved.

## Phase-change prototype

Open `Assets/Scenes/PhaseChange Beaker.unity` and press Play to see the SI thermal grid and the separate lattice-gas teaching view. The beaker includes a first 2-D liquid convection approximation coupled to sensible-heat transport. Vapour flow, moving ice, beaker heat capacity, and ambient heat exchange remain future work; see `Assets/Scripts/PhaseChange/ModelNotes.md` for assumptions and limits.
