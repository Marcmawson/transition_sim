# S14 dispersed droplets

## V37 isolated settling

Run `FlowMain --droplet-settling`. The fixture uses constant-radius spherical water droplets in uniform, stationary gas with prescribed liquid density 998 kg/m³, gas density 1.2 kg/m³, and gas viscosity 1.81e-5 Pa·s. The [Stokes continuum drag law](https://ntrs.nasa.gov/citations/20220007039) supplies the linear relaxation, and buoyancy reduces the gravitational acceleration. The routine accepts slip Reynolds numbers up to 0.1. The caller supplies gas conditions in the continuum regime.

The CPU implementation advances velocity analytically over each frozen-gas step and integrates position with the trapezoidal rule. It returns the equal and opposite drag impulse for gas coupling. The fixture checks the momentum ledger against the external gravity and buoyancy impulse. At radii of 5, 10 and 15 µm, terminal speeds were 3.0014, 12.0057 and 27.0127 mm/s. Position error decreased as the timestep halved from 2 ms to 1 ms to 0.5 ms. The finest errors were 0.196, 0.204 and 0.204 µm over 0.2 s.

This is the mechanical part of V37. Droplet heating and evaporation, feedback into the gas grid, and mass and energy verification remain open.
