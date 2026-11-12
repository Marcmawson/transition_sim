# S10 ice-water enthalpy recovery

Run `dotnet run --project Tools/ThreePhase3D/SolidLiquidChecks.csproj` from the repository root.

At prescribed pressure, the recovery derives the melting temperature from the Ice Ih equilibrium curve. It inverts sensible enthalpy on either side and uses the equilibrium latent heat to recover phase masses at the melting temperature. Phase volumes follow the respective densities. The calculation uses the existing shared enthalpy reference and the dense IAPWS-95 root. It supports the near-melting liquid range up to 300 K and pressures whose melting temperature is at least 273.15 K, within the existing liquid EOS range. The upper endpoint must be a stable liquid state.

CPU checks at 2 kPa, 101.325 kPa and 125 kPa passed sensible temperature recovery, mass and enthalpy closure on the plateau, and phase volume interpolation. At 101.325 kPa, the calculated melting temperature is 273.152519 K, latent heat is 333.427 kJ/kg, and the volume increase on freezing 250 g is 22.672 mL.

This is the equilibrium recovery used by the next S10 work. See the [planar melting and freezing benchmarks](S10-PlanarStefan.md) for V21 and V22. GPU integration remains open.

## V23 headspace displacement

The CPU fixture recovers five equilibrium states from fully liquid to fully frozen, with 250 g of water throughout. The vented case holds atmospheric pressure and measures the volume of gas displaced by the expanding water inventory. The sealed case starts with a 1 L isothermal ideal-gas headspace at 101325 Pa. The outer vessel volume is fixed and the gas volume can shrink. A pressure solve closes the fixed gas inventory using p * Vgas = constant, recomputing the melting point and phase densities at each trial pressure. Ice fraction is prescribed for this quasistatic benchmark.

At full freezing the vented case displaces 22.6718 mL. The sealed case displaces 22.6717 mL and reaches 103675.4993 Pa. All five states passed the density-based displacement and pressure comparisons, gas inventory closure and water mass conservation. The fixture uses an isothermal ideal gas with no water vapour exchange. Transient heat transfer, free-surface motion and GPU coupling remain outside this benchmark.
