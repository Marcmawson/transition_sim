# First phase-transition model slice

## SI beaker thermal grid

`BeakerEnergyLedger` is a 2-D finite-volume enthalpy grid. Each cell stores its own mass and specific enthalpy. A 0.10 m wide by 0.12 m high beaker with 0.025 m model depth contains 0.25 kg of ice initially at −50 °C; a 20 by 24 grid is the numerical resolution. Heater power is in watts, geometry in metres, mass in kilograms, enthalpy in joules per kilogram, and cell heat transfer uses Fourier conduction (`q = k A ΔT / Δx`). Heater input is applied at the lowest occupied cells using a normalized Gaussian footprint (5 cm full width at half maximum by default), with 100% of selected power assumed to enter the sample. The metal base and burner-to-beaker losses are not modeled.

`WaterThermodynamicsSI` maps each cell's local specific enthalpy and pressure to ice/liquid/vapour fractions. Therefore, a cell near the heated base can melt, then warm, then boil while cooler cells remain ice or liquid. Pressure in each cell includes ambient pressure and hydrostatic head. Ice uses the stated heat-capacity correlation; fusion uses latent heat; liquid uses a near-room-temperature heat capacity; boiling uses the IAPWS-IF97 saturation relation and a near-atmospheric latent-heat table.

The beaker includes a first liquid-flow stage in `LiquidConvection2D`. It advances a 2-D incompressible velocity field with momentum advection, pressure projection, viscous diffusion, and buoyancy from the Kell atmospheric-pressure liquid-water density correlation. Liquid mass and its sensible enthalpy move between cells using conservative, upwind face fluxes. The GUI shows the computed liquid direction and speed.

Vapour is represented as rising bubbles rather than a continuum gas-flow field. A terminal speed is estimated from buoyancy balanced by quadratic drag, using the inspector bubble diameter and drag coefficient; bubbles move upward between cells and leave through the open top. The cell phase state is recalculated from enthalpy after mass and energy transfer. This is not a complete multiphase solver: gas Navier-Stokes/compressibility and horizontal bubble motion are absent, ice stays fixed, liquid cannot spill over the free surface, and bubble condensation/nucleation are equilibrium approximations. There is no glass/metal heat capacity, burner-to-beaker loss, or ambient cooling (including the 16 C room air). The pressure projection and water-property fits are low-resolution approximations, so results are not quantitatively predictive.

## Lattice gas from the supplied diagram

`LatticeGas2D` implements binary site occupancy and a nearest-neighbour attraction with Metropolis acceptance `min(1, exp(−ΔE/(k_B T)))`. Kawasaki particle-vacancy exchanges conserve the fixed number of particles. Energy is in joules, temperature in kelvin, and the Boltzmann constant is exact in SI.

The helper `PairEnergyForSquareLatticeCriticalTemperature` calibrates the 2-D square-lattice model's critical temperature to a requested value. Calibrating to water's critical temperature matches only that one point; the lattice gas is not a water model and does not reproduce water's triple point, saturation curve, hydrogen bonding, or dynamics. Its purpose is to demonstrate how local attraction and thermal fluctuations can produce clustering and phase-like behaviour.

## References

- IAPWS, *Revised Release on the IAPWS Industrial Formulation 1997 for the Thermodynamic Properties of Water and Steam*: https://iapws.org/relguide/IF97-Rev.html
- IAPWS, *Release on the IAPWS Formulation 2008 for the Viscosity of Ordinary Water Substance*: https://www.iapws.org/relguide/viscosity.html
- Kell, G. S. (1975), *Density, thermal expansivity, and compressibility of liquid water from 0 to 150 C*, Journal of Chemical & Engineering Data, 20(1), 97-105, doi:10.1021/je60064a005.
- Clift, R., Grace, J. R., and Weber, M. E., *Bubbles, Drops, and Particles*, Dover Publications, 2005 (buoyancy-drag terminal-rise framework).
- IAPWS, *Revised Release on the Equation of State 2006 for H2O Ice Ih*: https://iapws.org/relguide/Ice-Rev2009.pdf
- NIST Chemistry WebBook, water phase-change data: https://webbook.nist.gov/cgi/cbook.cgi?ID=C7732185&Mask=4
- BIPM, standard gravity and standard atmosphere: https://www.bipm.org/en/publications/si-brochure
- David Tong, *Statistical Physics*, lattice gas / Ising model and phase transitions: https://www.damtp.cam.ac.uk/user/tong/statphys/statmechhtml/S5.html
