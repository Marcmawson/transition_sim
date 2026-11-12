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

The same command checks the volume source on a two-cell gas grid with one pressure outlet. The ideal-mixture volume at prescribed pressure includes both the change in gas temperature and the added vapour moles. One 0.4 ms exchange step produced a source of -1.4191307e-12 m³/s in the droplet cell. CPU pressure projection returned an outlet flux of +1.4191304e-12 m³/s and matched the internal and outlet volume budgets within 1e-5 relative.

The two-cell check also uses the projected face fluxes for first-order upwind transport of dry air, vapour and enthalpy. It uses the post-exchange gas volume for each donor and records the transported inventories at the open outlet. The downstream cell gained 3.9873958e-19 kg of vapour, and the outlet carried 3.2060744e-18 kg of vapour and 3.200185e-11 J of enthalpy. Total dry air, water and enthalpy closed to within 1e-12 of the starting inventories with the outlet included.

The multi-step run repeats exchange, pressure projection and upwind transport for 4 ms at 10, 20 and 40 steps. Each projection uses the difference between geometric cell volume and the post-exchange ideal-mixture volume, so the next step also corrects volume drift introduced by mixing. When the pressure outlet reverses, it supplies prescribed ambient dry air, vapour and enthalpy concentrations. The same signed boundary ledger accounts for incoming and outgoing inventories.

The 40-step CPU result ended at a 9.895136 µm droplet radius with 1.84217e-15 kg of net vapour entering through the boundary. Radius and net boundary vapour both refined toward this result, and dry air, water and enthalpy closed at every step. This gas-only fixture prescribes thermodynamic pressure and neglects displaced droplet volume.

## V37 droplet-to-gas momentum

The one-step fixture applies Stokes drag at the initial droplet radius and assigns evaporated mass the resulting droplet velocity. The gas receives the drag reaction plus the evaporated mass momentum. Gas advection carries that momentum using the same donor fractions as vapour and enthalpy, including the signed boundary ledger. The droplet transferred -1.4323637e-14 N·s vertically, and the combined momentum residual after accounting for gravity, buoyancy and the outlet was 1.2622e-29 N·s.

The multi-step CPU fixture uses a 2×2×1 grid. Each step recovers cell gas velocity from mass and momentum, uses it for droplet drag, and interpolates the post-exchange cell velocities to the staggered faces before projection. At the second step of the 40-step run, carried gas momentum changed projected face velocities by up to 5.2653349e-6 m/s against a control retaining only the current droplet impulse.

The cell momentum update applies the projected gauge-pressure impulse before advection. Interior faces use a shared mean pressure, so their impulses cancel in the domain sum. Rigid exterior faces use the adjacent cell pressure, and the outlet uses its prescribed pressure of 0.002 Pa gauge. The 40-step wall reaction along X was -1.6805139e-12 N·s. The outlet reaction matched its prescribed pressure times area and duration. Dry air, water, enthalpy and momentum closed at every step, including signed advective fluxes and the pressure reactions.

The pressure-force update is for a nonperiodic Cartesian box with no embedded solids.

## V37 moving droplet cell crossing

The droplet samples gas properties and deposits vapour, enthalpy and momentum in the pressure group containing its centre at the start of each step. The locator transforms world position into the grid frame. The crossing fixture starts 2 µm upstream of an interior face and checks that the droplet crosses once and exchanges with both cells. Dry air, water, enthalpy and momentum close at every step, including the signed boundary fluxes and pressure reactions.

Both CPU runs passed at 0.2 ms and 0.1 ms. The 0.1 ms run deposited 2.7858871e-14 kg of vapour in the starting cell and 1.0718757e-13 kg in the receiving cell. The transfer uses start-of-step sampling, so the partition between the two cells depends on when the crossing falls within a step.

## V37 open-boundary droplet exit

The exit case starts the droplet centre 2 µm inside the open pressure boundary. The step identifies the first exterior face crossed by the centre trajectory. When it is open, the fixture removes the remaining liquid and records its mass, enthalpy and momentum separately from gas transport. Gas integration continues after the droplet leaves.

Both CPU runs closed dry air, water, enthalpy and momentum at every step. At 0.1 ms, the droplet exited on step 5 carrying 4.1525538e-12 kg of liquid, 4.1608284e-7 J of liquid enthalpy and 1.3755498e-14 N·s of X momentum. Exchange and export are split at the end of the crossing step, so exit timing and inventory values have timestep dependence.

## V37 prescribed wall deposition

The deposition fixture starts with a 2 µm gap between the spherical droplet surface and a planar wall. On first contact with the prescribed sticking face, the step transfers the remaining liquid mass and enthalpy to the wall deposit inventory and transfers its momentum to the wall impulse ledger. The gas solver continues after deposition.

CPU runs at 0.2 and 0.1 ms closed the dry-air, water, enthalpy and momentum ledgers at every step. At 0.1 ms, impact occurred on step 5 and deposited 4.1525538e-12 kg of water and 4.1608284e-7 J of liquid enthalpy, with a wall impulse of -1.3964958e-14 N·s along X. Contact is resolved at the end of the step. This fixture assumes perfect sticking and uses an enthalpy ledger without impact kinetic heating.

## V37 wall-film re-evaporation

From the step after impact, the stationary deposit exchanges sensible heat and vapour with the adjacent gas cell. It uses the same saturation pressure and enthalpy conventions as the isolated droplet, with planar stagnant-film transfer over a prescribed 1e-10 m² wet area and 20 µm gas film. The enthalpy update debits the deposit by exactly what the gas gains, and gas projection and transport use the updated inventories.

The 0.1 ms run returned 6.2842125e-15 kg to the gas, with a net film enthalpy loss of 1.0235465e-8 J. Both timesteps passed the coupled conservation checks. This fixture uses a fixed wet area and an insulated wall.

The wall-film step caps the frozen-rate mass transfer at both the remaining water mass and the enthalpy available above the liquid reference temperature, including sensible heat from the gas. At dry-out it exports the remaining liquid inventory and leaves unused sensible heat in the gas. Later steps with zero film mass and enthalpy leave the gas inventories unchanged.

In the CPU dry-out fixture, 310 K gas evaporated the entire 1e-17 kg film over one 0.1 ms step. With the gas initially at the film temperature of 290 K, the heat-limited control retained 9.7218851e-18 kg. Both checks conserved water and enthalpy. Apparatus wetting, rebound, wall heat conduction, coupled droplet extinction and condensation remain open.
