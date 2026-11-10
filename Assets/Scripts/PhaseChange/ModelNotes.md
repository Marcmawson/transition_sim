# First phase-transition model slice

## SI beaker thermal grid

`BeakerEnergyLedger` is a 2-D finite-volume enthalpy grid. Each cell stores its own mass and specific enthalpy. A 0.10 m wide by 0.24 m high beaker with 0.025 m model depth contains 0.25 kg of ice initially at −50 °C; the 20 by 24 grid is the numerical resolution. The initial ice occupies a centred 65% of the beaker width, leaving lateral air space as well as headspace. Heater power is in watts, geometry in metres, mass in kilograms, enthalpy in joules per kilogram, and cell heat transfer uses Fourier conduction (`q = k A ΔT / Δx`). Heater input is applied at the lowest occupied cells using a normalized Gaussian footprint (5 cm full width at half maximum by default), with 100% of selected power assumed to enter the sample. The metal base and burner-to-beaker losses are not modeled.

`WaterThermodynamicsSI` maps each cell's local specific enthalpy and pressure to ice/liquid/vapour fractions. Therefore, a cell near the heated base can melt, then warm, then boil while cooler cells remain ice or liquid. Pressure in each cell includes ambient pressure and hydrostatic head. Ice uses the stated heat-capacity correlation; fusion uses latent heat; liquid uses a near-room-temperature heat capacity; boiling uses the IAPWS-IF97 saturation relation and a near-atmospheric latent-heat table.

The beaker's `LiquidConvection2D` stage uses a staggered MAC grid: pressure is cell-centred and each velocity component is stored at its cell faces. The pressure Poisson operator uses the same face divergence and pressure-gradient stencil as the velocity correction. The model includes momentum advection, viscous diffusion, gravity and temperature-dependent Boussinesq buoyancy. Air cells provide an atmospheric-pressure free surface; ice cells are impermeable obstacles. The resulting velocity face fluxes advect liquid mass and sensible enthalpy across the interface. The interface and phase transfer are coarse approximations, not a full VOF or level-set method. The time step is limited by a cell-scale CFL estimate; the GUI reports peak speed and divergence for inspection.

The MAC flow step now receives a **liquid volume fraction** (liquid mass converted to volume using the water-density correlation), separate from the thermodynamic liquid mass fraction. Surface tension is added as a continuum-surface-force term, `f_sigma = sigma * kappa * grad(alpha)`, using interface curvature from the volume-fraction normal. `sigma(T)` uses the IAPWS R1-76(2014) pure-water correlation. The glass contact-angle parameter sets the interface normal in boundary cells; its default of 90 degrees is a neutral numerical reference, not a measured value for the glass used in an experiment. The rigid side and bottom walls enforce zero normal velocity; reflected tangential ghost velocities impose no-slip in the viscous stencil without zeroing interior faces after pressure projection. A capillary-wave time-step bound is applied in addition to the existing gravity and flow limits. These are low-resolution Cartesian approximations and need numerical validation before they are treated as predictive.

## Coupled-model implementation order

1. **Eulerian liquid/interface baseline:** separate liquid volume fraction from phase mass fraction, enforce vessel-wall velocity conditions, add water-air surface tension and glass contact angle. Validate hydrostatic rest, dam break, capillary relaxation, mass conservation, and divergence before extending the model.
2. **Homogeneous ice rigid body:** replace cell-by-cell ice motion with one SI rigid body carrying mass, dimensions, centre of mass, velocity, orientation, temperature, and bulk thermal properties. Couple its boundary to the MAC grid with an immersed-boundary or cut-cell method; compute buoyancy and collisions from the body geometry.
3. **Ice-water heat and melt exchange:** solve heat conduction in the ice and water, exchange heat across their interface, remove the latent heat of fusion, shrink the body, and transfer the melted mass and enthalpy conservatively into the liquid volume-fraction/enthalpy fields.
4. **Vapour and ambient air:** add a compressible gas mixture with an appropriate equation of state and transport properties, then couple evaporation/condensation to conserved mass and energy. Do not treat the present bubble-rise approximation as gas flow.
5. **Heater, beaker and room-air energy balance:** add beaker thermal mass and conductivity, burner-to-wall heat transfer, and heat exchange with the 16 C ambient air. Validate total energy and mass balances at every phase boundary.

Ice flotation is approximated by relocating the grid's ice phase to its Archimedean equilibrium height once liquid mass is sufficient to support the remaining ice weight. This conserves the ice phase mass and enthalpy but is not a rigid-body or deformable-ice solver. Ice blocks the liquid velocity field during each fluid step.

Vapour is represented as rising bubbles rather than a continuum gas-flow field. A terminal speed is estimated from buoyancy balanced by quadratic drag, using the inspector bubble diameter and drag coefficient; bubbles move upward between cells and leave through the open top. The cell phase state is recalculated from enthalpy after mass and energy transfer. This is not a complete multiphase solver: gas Navier-Stokes/compressibility and horizontal bubble motion are absent, liquid cannot spill over the free surface, and bubble condensation/nucleation are equilibrium approximations. There is no glass/metal heat capacity, burner-to-beaker loss, or ambient cooling (including the 16 C room air). The pressure projection and water-property fits are low-resolution approximations, so results are not quantitatively predictive.

## Lattice gas from the supplied diagram

`LatticeGas2D` implements binary site occupancy and a nearest-neighbour attraction with Metropolis acceptance `min(1, exp(−ΔE/(k_B T)))`. Kawasaki particle-vacancy exchanges conserve the fixed number of particles. Energy is in joules, temperature in kelvin, and the Boltzmann constant is exact in SI.

The helper `PairEnergyForSquareLatticeCriticalTemperature` calibrates the 2-D square-lattice model's critical temperature to a requested value. Calibrating to water's critical temperature matches only that one point; the lattice gas is not a water model and does not reproduce water's triple point, saturation curve, hydrogen bonding, or dynamics. Its purpose is to demonstrate how local attraction and thermal fluctuations can produce clustering and phase-like behaviour.

## References

- IAPWS, *Revised Release on the IAPWS Industrial Formulation 1997 for the Thermodynamic Properties of Water and Steam*: https://iapws.org/relguide/IF97-Rev.html
- IAPWS, *Release on the IAPWS Formulation 2008 for the Viscosity of Ordinary Water Substance*: https://www.iapws.org/relguide/viscosity.html
- Kell, G. S. (1975), *Density, thermal expansivity, and compressibility of liquid water from 0 to 150 C*, Journal of Chemical & Engineering Data, 20(1), 97-105, doi:10.1021/je60064a005.
- Clift, R., Grace, J. R., and Weber, M. E., *Bubbles, Drops, and Particles*, Dover Publications, 2005 (buoyancy-drag terminal-rise framework).
- IAPWS, *Revised Release on the Equation of State 2006 for H2O Ice Ih*: https://iapws.org/relguide/Ice-Rev2009.pdf
- IAPWS, *Revised Release on Surface Tension of Ordinary Water Substance* (R1-76(2014)): https://www.iapws.org/relguide/Surf-H2O.html
- Brackbill, Kothe, and Zemach (1992), *A Continuum Method for Modeling Surface Tension*, Journal of Computational Physics 100, 335-354, doi:10.1016/0021-9991(92)90240-Y.
- NIST Chemistry WebBook, water phase-change data: https://webbook.nist.gov/cgi/cbook.cgi?ID=C7732185&Mask=4
- BIPM, standard gravity and standard atmosphere: https://www.bipm.org/en/publications/si-brochure
- David Tong, *Statistical Physics*, lattice gas / Ising model and phase transitions: https://www.damtp.cam.ac.uk/user/tong/statphys/statmechhtml/S5.html
