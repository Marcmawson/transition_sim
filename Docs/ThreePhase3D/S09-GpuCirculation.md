# S09 coupled cavity circulation check

Run **Physics Sim → Three Phase 3D → Run S09 GPU buoyancy check**. After the three single-step cases, the same menu runs 40 coupled steps of 0.05 s in the closed, fully liquid cavity. Each step reconciles the previous pressure flux, transports volume and enthalpy, remaps momentum, recovers liquid temperature, applies buoyancy from the atmospheric density table, and projects velocity.

The test requires upward velocity in the warm column and downward velocity in the return flow throughout the run. It checks enthalpy redistribution and bounds total enthalpy and volume drift. The CPU reference with the same mesh and step ordering ended at +0.00302471 m/s in the warm column and −0.00100025 m/s in the return flow, with an enthalpy drift of 2.27e-13 J.

The C# integration compiled against Unity 2022.3.46f1. The coupled GPU test still needs to run in Unity. This is a transient, initially warm column in an insulated cavity. The benchmark uses fixed transport density with density-dependent buoyancy. Viscosity, thermal diffusion, sustained wall heating, V19 steady benchmark validation and glass-to-flow coupling are outside this fixture.
