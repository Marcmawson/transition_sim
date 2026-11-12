# S14 dispersed droplets

## V37 isolated settling

Run `FlowMain --droplet-settling`. The fixture uses constant-radius spherical water droplets in uniform, stationary gas with prescribed liquid density 998 kg/m³, gas density 1.2 kg/m³, and gas viscosity 1.81e-5 Pa·s. The [Stokes continuum drag law](https://ntrs.nasa.gov/citations/20220007039) supplies the linear relaxation, and buoyancy reduces the gravitational acceleration. The routine accepts slip Reynolds numbers up to 0.1. The caller supplies gas conditions in the continuum regime.

The CPU implementation advances velocity analytically over each frozen-gas step and integrates position with the trapezoidal rule. It returns the equal and opposite drag impulse for gas coupling. The fixture checks the momentum ledger against the external gravity and buoyancy impulse. At radii of 5, 10 and 15 µm, terminal speeds were 3.0014, 12.0057 and 27.0127 mm/s. Position error decreased as the timestep halved from 2 ms to 1 ms to 0.5 ms. The finest errors were 0.196, 0.204 and 0.204 µm over 0.2 s.

## V37 isolated sensible heat

Run `FlowMain --droplet-heating`. This fixture uses the quasi-steady spherical conduction limit (diameter-based Nusselt number 2) with conductance `4 π k_g r`. It treats the droplet and its gas reservoir as uniform temperatures with constant heat capacities. The step integrates their temperature difference analytically and returns signed energy for equal and opposite updates. See [the spherical conduction limit, section 4](https://doi.org/10.1017/jfm.2025.10880), which defines Nusselt number using radius.

All nine CPU cases passed across radii 5, 10 and 15 µm and initial gas temperatures 285, 300 and 315 K. Results agreed with the analytical temperature within 1e-10 K for one, ten and one hundred steps over 5 ms, and the combined sensible energy balanced at every step. The 10 µm droplet warmed from 300 K to 308.757759 K in initially 315 K gas, which cooled to 314.124224 K.

Evaporation, gas-grid coupling and mass closure remain open.
