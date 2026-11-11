# S08 transient solid-conduction reference

The constant-property finite-volume reference evolves cell temperatures from heat capacities in J/K, signed external powers in W and internal face conductances in W/K. Each internal face is evaluated from the old temperature field and applied with opposite signs to its adjoining cells. The explicit time step is limited to the smallest cell capacity divided by its sum of internal conductances. Callers supplying temperature-dependent external power must also account for its stability limit.

Run `dotnet run -c Release --project Tools/ThreePhase3D/SolidConductionChecks.csproj`. The same C# sources passed using the compiler shipped with Unity 2022.3.46f1.

| Case | Result |
| --- | --- |
| V16 insulated slab, 16 and 32 cells | Maximum errors 0.000996239 K and 0.000275934 K after 20 s against the analytic cosine mode |
| V17 two materials and contact resistance | 84.955752 W at steady state, with equal and opposite boundary heat rates |
| V18 synthetic base geometry for 9 mL and 250 mL cases | Each 16 × 6 radial/axial mesh stored 30 J after 3 s at 10 W. The centre underside warmed above the centre upper face and the outer underside. A synthetic support node warmed through contact. |

The radial fixtures model the glass base with synthetic constant properties and one support heat capacity. After switch-off, the hot spot cools by internal redistribution while stored energy stays at 30 J. These fixtures establish the conduction reference, not full V16–V18 or V40 acceptance. Remaining work includes temperature-dependent enthalpy, resolved glass walls and tripod, comparison with a refined radial reference, ambient and radiation boundaries, GPU field integration, and water coupling. The 250 mL case exercises its larger base geometry only.
