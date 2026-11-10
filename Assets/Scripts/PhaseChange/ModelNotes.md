# First phase-transition model slice

## SI beaker energy ledger

`BeakerEnergyLedger` integrates heater energy as `Q = P Δt` and specific enthalpy as `Δh = Q/m`. It starts with 0.25 kg of ice at −50 °C and a 500 W heater as editable experiment inputs. The values are expressed in SI units; they describe the chosen apparatus and can be changed in the Inspector.

`WaterThermodynamicsSI` maps specific enthalpy to equilibrium ice/liquid/vapour mass fractions. It includes the ice heat-capacity correlation, latent heat of fusion, liquid heat capacity, tabulated near-atmospheric latent heat of vaporisation, and the IAPWS-IF97 saturation-pressure relation. Ambient pressure uses the standard-atmosphere troposphere relation and liquid pressure can include hydrostatic head.

This first slice deliberately does not claim a complete beaker simulation. It assumes a uniform sample temperature and pressure, and it has no spatial conduction, convection, vapor transport, nucleation delay, glass heat capacity, or heat loss. The vapour superheat branch uses a constant approximate heat capacity. The boiling enthalpy lookup is restricted to saturation temperatures from 0 to 100 °C. These limitations need to be removed before interpreting the scene as a quantitatively predictive experiment.

## Lattice gas from the supplied diagram

`LatticeGas2D` implements binary site occupancy and a nearest-neighbour attraction with Metropolis acceptance `min(1, exp(−ΔE/(k_B T)))`. Kawasaki particle-vacancy exchanges conserve the fixed number of particles. Energy is in joules, temperature in kelvin, and the Boltzmann constant is exact in SI.

The helper `PairEnergyForSquareLatticeCriticalTemperature` calibrates the 2-D square-lattice model's critical temperature to a requested value. Calibrating to water's critical temperature matches only that one point; the lattice gas is not a water model and does not reproduce water's triple point, saturation curve, hydrogen bonding, or dynamics. Its purpose is to demonstrate how local attraction and thermal fluctuations can produce clustering and phase-like behaviour.

## References

- IAPWS, *Revised Release on the IAPWS Industrial Formulation 1997 for the Thermodynamic Properties of Water and Steam*: https://iapws.org/relguide/IF97-Rev.html
- IAPWS, *Revised Release on the Equation of State 2006 for H2O Ice Ih*: https://iapws.org/relguide/Ice-Rev2009.pdf
- NIST Chemistry WebBook, water phase-change data: https://webbook.nist.gov/cgi/cbook.cgi?ID=C7732185&Mask=4
- BIPM, standard gravity and standard atmosphere: https://www.bipm.org/en/publications/si-brochure
- David Tong, *Statistical Physics*, lattice gas / Ising model and phase transitions: https://www.damtp.cam.ac.uk/user/tong/statphys/statmechhtml/S5.html
