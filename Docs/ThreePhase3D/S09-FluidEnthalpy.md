# S09 liquid enthalpy reference

The existing cut-cell volume-of-fluid (VOF) transport already carries an extensive liquid energy inventory on the accepted liquid face fluxes. The S09 reference supplies a temperature-to-enthalpy conversion and its inverse using the S02 atmospheric liquid-water table. It uses the same fixed transport density as the VOF mass model. Dry cells have no liquid temperature and out-of-range enthalpy is rejected.

Run `dotnet run -c Release --project Tools/ThreePhase3D/SurfaceGeometryChecks.csproj -- --fluid-enthalpy`.

The cut-cell CPU checks preserve 303.15 K in initially uniform moving liquid, carry a warm pulse into the downstream cell and close the open-boundary enthalpy ledger to within 1.2e-14 J. Temperature round trips include both ends of the table. At 101325 Pa the repository Water95 model gives 999.89416, 999.97487 and 999.71551 kg/m³ at 274, 277.13 and 283 K respectively.

The local Unity-shipped compiler required explicit casts for three pre-existing method-group expressions in temporary validation copies. The published flow sources were left intact.

This is an S09 CPU reference increment. The thermal preview still uses stationary water. GPU temperature recovery, coupling to the glass during moving-liquid runs, variable-density pressure and volume coupling, buoyancy, and V19/V20 flow validation remain open.
