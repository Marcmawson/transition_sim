# S14 dispersed droplets

## V37 isolated settling

Run `FlowMain --droplet-settling`. The fixture uses constant-radius spherical water droplets in uniform, stationary gas with prescribed liquid density 998 kg/m³, gas density 1.2 kg/m³, and gas viscosity 1.81e-5 Pa·s. The [Stokes continuum drag law](https://ntrs.nasa.gov/citations/20220007039) supplies the linear relaxation, and buoyancy reduces the gravitational acceleration. The routine accepts slip Reynolds numbers up to 0.1. The caller supplies gas conditions in the continuum regime.

The CPU implementation advances velocity analytically over each frozen-gas step and integrates position with the trapezoidal rule. It returns the equal and opposite drag impulse for gas coupling. The fixture checks the momentum ledger against the external gravity and buoyancy impulse. At radii of 5, 10 and 15 µm, terminal speeds were 3.0014, 12.0057 and 27.0127 mm/s. Position error decreased as the timestep halved from 2 ms to 1 ms to 0.5 ms. The finest errors were 0.196, 0.204 and 0.204 µm over 0.2 s.

## V37 isolated sensible heat

Run `FlowMain --droplet-heating`. This fixture uses the quasi-steady spherical conduction limit (diameter-based Nusselt number 2) with conductance `4 π k_g r`. It treats the droplet and its gas reservoir as uniform temperatures with constant heat capacities. The step integrates their temperature difference analytically and returns signed energy for equal and opposite updates. See [the spherical conduction limit, section 4](https://doi.org/10.1017/jfm.2025.10880), which defines Nusselt number using radius.

All nine CPU cases passed across radii 5, 10 and 15 µm and initial gas temperatures 285, 300 and 315 K. Results agreed with the analytical temperature within 1e-10 K for one, ten and one hundred steps over 5 ms, and the combined sensible energy balanced at every step. The 10 µm droplet warmed from 300 K to 308.757759 K in initially 315 K gas, which cooled to 314.124224 K.

## V37 isolated evaporation

Run `FlowMain --droplet-evaporation`. The model uses the stagnant-gas Spalding limit with fixed density, diffusivity and surface and ambient vapour mass fractions. The [diameter-squared law, equations 51–55](https://www.mdpi.com/1099-4300/25/8/1232) gives the exact radius update under these assumptions. The routine returns the vapour mass and enthalpy and the heat required to hold the droplet isothermal. It caps transfer at the initial liquid inventory when the step crosses extinction.

CPU checks covered 5, 10 and 15 µm initial radii, drying and saturation, and one, ten and one hundred steps. The drying slope was -3.6529456e-9 m²/s. Partial evaporation and extinction matched the reference, and the mass and enthalpy ledgers closed at each step. The saturated control transferred zero mass.

The isolated benchmark prescribes humidity and uses an external heat ledger.

## V37 coupled evaporation in one gas cell

Run `FlowMain --droplet-gas-cell`. The local CPU step updates the droplet temperature, gas vapour mass and gas enthalpy. Spherical conduction exchanges sensible heat, then stagnant-film evaporation draws latent heat from the droplet. Surface humidity uses the ideal-mixture conversion of the [IAPWS supplementary saturation pressure](https://www.iapws.org/relguide/Supp-sat.html), equation 1. The fixture assumes prescribed pressure, uniform phase temperatures, constant heat capacities and fixed film transport properties.

Over 4 ms, a 10 µm droplet cooled from 300 K to 292.017231 K, shrank to 9.896270 µm, and transferred 1.2874597e-13 kg of vapour. The 16, 32 and 64 step runs approached the 128 step reference. Each step conserved total water mass and enthalpy within 1e-12 relative to the initial inventories. The isolated evaporation, sensible-heat and settling regressions passed.

This local exchange is evaporation-only. Coupled extinction, condensation, momentum deposition, gas-grid transport and pressure coupling remain open.
