# First phase-transition model slice

## SI beaker thermal grid

`BeakerEnergyLedger` is a 2-D finite-volume enthalpy grid. Each cell stores its own mass and specific enthalpy. A 0.10 m wide by 0.12 m high beaker with 0.025 m model depth contains 0.25 kg of ice initially at −50 °C; a 20 by 24 grid is the numerical resolution. Heater power is in watts, geometry in metres, mass in kilograms, enthalpy in joules per kilogram, and cell heat transfer uses Fourier conduction (`q = k A ΔT / Δx`). Heater input is applied at the wet bottom boundary, with 100% of the selected heater power assumed to enter the sample.

`WaterThermodynamicsSI` maps each cell's local specific enthalpy and pressure to ice/liquid/vapour fractions. Therefore, a cell near the heated base can melt, then warm, then boil while cooler cells remain ice or liquid. Pressure in each cell includes ambient pressure and hydrostatic head. Ice uses the stated heat-capacity correlation; fusion uses latent heat; liquid uses a near-room-temperature heat capacity; boiling uses the IAPWS-IF97 saturation relation and a near-atmospheric latent-heat table.

The beaker now includes a first liquid-flow stage in `LiquidConvection2D`. It advances a 2-D incompressible velocity field using momentum advection, a pressure projection, viscous diffusion, and buoyancy from the Kell atmospheric-pressure liquid-water density correlation. Dynamic viscosity uses a compact temperature fit; flow is restricted to cells with liquid fraction and ice is stationary. Liquid sensible enthalpy is transported between cells by a conservative upwind flux. The UI can show the computed liquid-flow direction and peak speed.

This is not yet a complete multiphase fluid simulation. The grid keeps its assigned sample mass, so the solver transports liquid sensible heat but not water mass or phase fractions. Vapour has no flow field, gas compressibility is omitted, and ice does not move or float. The free surface is treated as a closed liquid boundary. There is no glass/metal heat capacity, burner-to-beaker loss, ambient cooling (including the 16 C room air), surface evaporation, nucleation delay, or open-vessel mass loss. The pressure projection is a low-resolution educational approximation and the viscosity fit is not the full IAPWS formulation. The vapour superheat heat capacity is approximate, and vaporisation enthalpy is held at its 100 C tabulated value for the small hydrostatic correction above 100 C. The displayed phase markers show local fractions, not individual molecules. These limitations prevent quantitative prediction.

## Lattice gas from the supplied diagram

`LatticeGas2D` implements binary site occupancy and a nearest-neighbour attraction with Metropolis acceptance `min(1, exp(−ΔE/(k_B T)))`. Kawasaki particle-vacancy exchanges conserve the fixed number of particles. Energy is in joules, temperature in kelvin, and the Boltzmann constant is exact in SI.

The helper `PairEnergyForSquareLatticeCriticalTemperature` calibrates the 2-D square-lattice model's critical temperature to a requested value. Calibrating to water's critical temperature matches only that one point; the lattice gas is not a water model and does not reproduce water's triple point, saturation curve, hydrogen bonding, or dynamics. Its purpose is to demonstrate how local attraction and thermal fluctuations can produce clustering and phase-like behaviour.

## References

- IAPWS, *Revised Release on the IAPWS Industrial Formulation 1997 for the Thermodynamic Properties of Water and Steam*: https://iapws.org/relguide/IF97-Rev.html
- IAPWS, *Release on the IAPWS Formulation 2008 for the Viscosity of Ordinary Water Substance*: https://www.iapws.org/relguide/viscosity.html
- Kell, G. S. (1975), *Density, thermal expansivity, and compressibility of liquid water from 0 to 150 C*, Journal of Chemical & Engineering Data, 20(1), 97-105, doi:10.1021/je60064a005.
- IAPWS, *Revised Release on the Equation of State 2006 for H2O Ice Ih*: https://iapws.org/relguide/Ice-Rev2009.pdf
- NIST Chemistry WebBook, water phase-change data: https://webbook.nist.gov/cgi/cbook.cgi?ID=C7732185&Mask=4
- BIPM, standard gravity and standard atmosphere: https://www.bipm.org/en/publications/si-brochure
- David Tong, *Statistical Physics*, lattice gas / Ising model and phase transitions: https://www.damtp.cam.ac.uk/user/tong/statphys/statmechhtml/S5.html
