# S10 ice-water enthalpy recovery

Run `dotnet run --project Tools/ThreePhase3D/SolidLiquidChecks.csproj` from the repository root.

At prescribed pressure, the recovery derives the melting temperature from the Ice Ih equilibrium curve. It inverts sensible enthalpy on either side and uses the equilibrium latent heat to recover phase masses at the melting temperature. Phase volumes follow the respective densities. The calculation uses the existing shared enthalpy reference and the dense IAPWS-95 root. It supports the near-melting liquid range up to 300 K and pressures whose melting temperature is at least 273.15 K, within the existing liquid EOS range. The upper endpoint must be a stable liquid state.

CPU checks at 2 kPa, 101.325 kPa and 125 kPa passed sensible temperature recovery, mass and enthalpy closure on the plateau, and phase volume interpolation. At 101.325 kPa, the calculated melting temperature is 273.152519 K, latent heat is 333.427 kJ/kg, and the volume increase on freezing 250 g is 22.672 mL.

This is the equilibrium recovery used by the next S10 work. The seeded planar melting and freezing fronts, GPU integration and compliant-volume response remain open under V21-V23.
