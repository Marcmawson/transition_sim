# S09 heated glass floor and moving water

Run **Physics Sim → Three Phase 3D → Run S09 GPU buoyancy check**. The menu now also runs a heated-floor case in the closed liquid cavity. Two 1.5 mm glass layers conduct through thickness and laterally. A 5 W input is split across two tiles on the underside. Each GPU interface calculation debits the upper glass cell and produces a signed joule transfer to its adjoining liquid group. The host applies those transfers to the liquid enthalpy before the existing GPU transport, temperature recovery, buoyancy and pressure steps.

After 40 steps, the CPU reference stored 9.9426384 J in the glass and transferred 0.0573616 J into the liquid. Near the floor the warmed column rose at 1.532 µm/s and adjacent return flow descended at 0.584 µm/s. The GPU test requires both directions, zero volume drift within 1e-12 m³, and closure of the 10 J energy input within 1e-4 J. The C# integration compiled against Unity 2022.3.46f1. GPU execution is pending.

The test uses a flat, synthetic glass floor and an insulated closed cavity. Mapping wetted faces of the curved beaker to the thermal grid and enabling the coupled solver in the beaker scene remain open.
