# First phase-transition model slice

## SI beaker thermal grid

`BeakerEnergyLedger` is a 2-D finite-volume enthalpy grid. Each cell stores its own mass and specific enthalpy. A 0.10 m wide by 0.12 m high beaker with 0.025 m model depth contains 0.25 kg of ice initially at −50 °C; a 20 by 24 grid is the numerical resolution. Heater power is in watts, geometry in metres, mass in kilograms, enthalpy in joules per kilogram, and cell heat transfer uses Fourier conduction (`q = k A ΔT / Δx`). Heater input is applied at the wet bottom boundary, with 100% of the selected heater power assumed to enter the sample.

`WaterThermodynamicsSI` maps each cell's local specific enthalpy and pressure to ice/liquid/vapour fractions. Therefore, a cell near the heated base can melt, then warm, then boil while cooler cells remain ice or liquid. Pressure in each cell includes ambient pressure and hydrostatic head. Ice uses the stated heat-capacity correlation; fusion uses latent heat; liquid uses a near-room-temperature heat capacity; boiling uses the IAPWS-IF97 saturation relation and a near-atmospheric latent-heat table.

This is still a conduction and local-equilibrium model, not a complete fluid simulation. Water and vapour are not advected; buoyancy-driven convection is absent; cells keep their assigned sample mass; there is no glass-metal heat capacity, burner-to-beaker loss, ambient cooling, surface evaporation, nucleation delay, or open-vessel mass loss. The vapour superheat heat capacity is approximate, and vaporisation enthalpy is held at its 100 °C tabulated value for the small hydrostatic correction above 100 °C. The displayed markers follow local phase fractions but are not molecules. These assumptions must be improved before treating results as quantitatively predictive.

## Lattice gas from the supplied diagram

`LatticeGas2D` implements binary site occupancy and a nearest-neighbour attraction with Metropolis acceptance `min(1, exp(−ΔE/(k_B T)))`. Kawasaki particle-vacancy exchanges conserve the fixed number of particles. Energy is in joules, temperature in kelvin, and the Boltzmann constant is exact in SI.

The helper `PairEnergyForSquareLatticeCriticalTemperature` calibrates the 2-D square-lattice model's critical temperature to a requested value. Calibrating to water's critical temperature matches only that one point; the lattice gas is not a water model and does not reproduce water's triple point, saturation curve, hydrogen bonding, or dynamics. Its purpose is to demonstrate how local attraction and thermal fluctuations can produce clustering and phase-like behaviour.

## References

- IAPWS, *Revised Release on the IAPWS Industrial Formulation 1997 for the Thermodynamic Properties of Water and Steam*: https://iapws.org/relguide/IF97-Rev.html
- IAPWS, *Revised Release on the Equation of State 2006 for H2O Ice Ih*: https://iapws.org/relguide/Ice-Rev2009.pdf
- NIST Chemistry WebBook, water phase-change data: https://webbook.nist.gov/cgi/cbook.cgi?ID=C7732185&Mask=4
- BIPM, standard gravity and standard atmosphere: https://www.bipm.org/en/publications/si-brochure
- David Tong, *Statistical Physics*, lattice gas / Ising model and phase transitions: https://www.damtp.cam.ac.uk/user/tong/statphys/statmechhtml/S5.html
