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

In the CPU dry-out fixture, 310 K gas evaporated the entire 1e-17 kg film over one 0.1 ms step. With the gas initially at the film temperature of 290 K, the heat-limited control retained 9.7218851e-18 kg. Both checks conserved water and enthalpy. Apparatus wetting, rebound, wall heat conduction and coupled droplet extinction remain open.

## Seeded condensation

The local gas-cell exchange can now reverse the stagnant spherical film flux when ambient vapour exceeds saturation at the droplet surface. Condensed mass leaves the gas inventory with its vapour enthalpy, warming the growing droplet. The opt-in case uses the same prescribed pressure, constant heat capacities and flat-surface saturation relation as the evaporation fixture.

Over 1 ms, the 10 µm seed grew to 10.014102 µm, warmed from 290 K to 292.259523 K, and condensed 1.7710493e-14 kg. The 16- and 32-step results refined toward the 64-step reference. Water and enthalpy closed each step, and the zero-seed control left the gas unchanged.

The moving-gas case starts at 0.01 m/s with a stationary seed. After the drag update, condensed water carries the donor gas velocity and updates the droplet velocity by mass-weighted momentum mixing. The 64-step case reached a droplet velocity of 0.0054048763 m/s with a gas-plus-liquid momentum residual of 5.0487e-29 N·s. Each step checked the phase impulse against a drag-only update.

The multi-cell CPU case uses one seed in the 2×2×1 gas box. Condensation updates local inventories before the pressure projection and gas transport, with the open boundary drawing from the prescribed humid reservoir. Over 1 ms the 64-step run condensed 1.7710716e-14 kg and drew in a net 2.6344207e-16 kg of boundary vapour. Radius refined across the 16-, 32- and 64-step runs. Dry air, water, enthalpy and momentum balanced every step, including pressure reactions and signed boundary transport. The V38 multi-seed fog ensemble remains open.
## V38 prescribed seed ensemble

Run `FlowMain --seeded-fog`. The CPU fixture uses the multiplicity representation described by [Shima et al.](https://doi.org/10.1002/qj.441) with the existing local spherical exchange. It starts with 12, 20 and 8 droplets at radii 8, 10 and 12 µm, respectively. Each seed samples the same initial gas state each step with an equal share of its heat and vapour inventory. Multiplicity weights the summed changes to the shared gas cell.

Three weighted parcels matched the same 40 individually represented droplets in radius, temperature, vapour and gas enthalpy. The 16- and 32-step runs refined toward the 64-step result. Water and enthalpy closed each step. Over 1 ms, the drier case evaporated 8.3126268e-13 kg, and the supersaturated case condensed 6.8910263e-13 kg.

The random-sampling case compares a 2,304-droplet population sampled from nine equally probable radii (6–14 µm) to the same model evolved with exact initial bin weights. Each parcel count uses 24 reproducible random seeds and the same gas inventory. Water and enthalpy close against each sample's own initial inventory. After 1 ms, RMS cumulative size-fraction errors at the eight bin boundaries were 9.093, 4.764 and 2.448 percentage points at 16, 64 and 256 parcels. Relative RMS condensate errors were 6.947%, 4.008% and 1.791%. Both fine-sample metrics met the 4% fixture tolerance. The reference uses prescribed seed weights and the same exchange model. Spatial fog transport and opacity verification remain open.
## V38 spatial parcel transport

The three-cell CPU fixture moves two weighted parcels at a prescribed 0.02 m/s from drier gas into supersaturated gas. Parcels retain their multiplicity and sample the gas in their start-of-step pressure group. The fixture holds the Eulerian gas inventories in place between local exchanges. Both 0.1 and 0.05 ms runs matched the analytical parcel positions and conserved water and enthalpy at every step. In the finer run, upstream evaporation was 3.2906608e-13 kg and downstream condensation was 2.436634e-13 kg. The unvisited third cell was unchanged.

The gas-feedback variant uses the same prescribed parcel velocity and adds an open pressure boundary. It normalises the initial gas inventories to each geometric cell volume at 101325 Pa. Each exchange updates the gas volume source, then CPU pressure projection and upwind transport redistribute the gas inventories. Dry air, water, enthalpy and gas momentum balanced at every step with signed boundary fluxes and pressure reactions included. In the 0.05 ms run, the downstream parcels condensed 2.4205407e-13 kg and the open boundary supplied a net 1.6618656e-15 kg of vapour. Gas transport also changed the unseeded cell.

The moving-parcel variant replaces prescribed trajectories with Stokes motion. Each parcel samples velocity from the local gas momentum and mass at the beginning of the step. The gas receives the reaction to drag and the signed phase-change momentum, weighted by parcel multiplicity. The existing pressure projection and transport carry that momentum into the next step. Dry air, water, enthalpy and combined gas-plus-parcel momentum balanced at every step, including the signed boundary ledger and pressure reactions.

Both CPU runs crossed the first parcel into the humid cell. At 0.05 ms per step, it slowed from 0.02 to 0.0053355144 m/s over 1 ms and downstream parcels condensed 1.3190063e-13 kg. Start-of-step cell sampling makes the condensation sensitive to crossing time.

The open-boundary variant starts two parcels just inside the pressure outlet. The moving-parcel step exports each parcel when its centre first crosses the open face and records the remaining liquid, enthalpy, momentum and droplet count. The gas continues to advance after both parcels leave. Both CPU runs exported all 32 droplets and closed the combined inventories at every step. At 0.05 ms per step the outlet carried 1.0942039e-10 kg of liquid, 8.0147474e-6 J of liquid enthalpy and 1.785407e-12 N·s of X momentum. The export is evaluated at the end of the crossing step. Ensemble wall deposition, resolved/parcel conversion and opacity verification remain open.

## V38 weighted wall deposition

The CPU fixture directs two weighted parcels toward a prescribed perfectly sticking planar face. Contact uses the post-exchange spherical radius and end-of-step position. The step removes the parcel and credits the wall with its multiplicity, liquid mass, enthalpy and impact impulse. Both 0.1 and 0.05 ms runs deposited all 32 droplets and closed the combined inventories at every step. At 0.05 ms the wall received 1.0914108e-10 kg of liquid, 7.3196612e-6 J of enthalpy and -1.7949422e-12 N s of X impulse. Those values describe the deposition-only fixture.

The coupled benchmark now accumulates the deposited liquid into a stationary wall film. Starting on the step after each impact, the film exchanges sensible heat and vapour with the adjacent gas cell before pressure projection. It uses a prescribed wet area of 1e-10 m2, a 20 micrometre gas film and an insulated wall. Both CPU runs closed water, enthalpy and momentum including the remaining film and boundary ledgers. At 0.05 ms the film returned 6.6079126e-16 kg of vapour and lost 1.5891817e-9 J of net enthalpy. The benchmark covers one planar deposit location.

## Resolved-liquid and parcel inventory conversion

The CPU conversion maps a supplied liquid volume, sensible enthalpy and momentum to identical spherical droplets at a prescribed multiplicity, using the constant-density and constant-heat-capacity parcel model. The reverse map recovers the same extensive inventories. Nine round trips at three liquid volumes and three multiplicities closed to a maximum relative residual of 2.4425e-15. This check covers inventory representation. Identifying detached VOF components, conservative gas-volume remapping, and convergence of the representation-switch threshold remain open.

### Detached VOF components

Liquid connectivity uses overlap of the reconstructed liquid sections on shared faces. Caller-provided anchors identify bulk or attached liquid. Components connected to an anchor or open liquid face stay resolved, as do components exceeding the supplied volume threshold. The six-cell CPU fixture retained an anchored liquid neck and converted a detached component of volume 1e-13 m3. The conversion credits the parcel with the supplied liquid momentum and debits resolved volume and enthalpy. The separate parcel displacement inventory closes the geometric volume without altering gas volume, vapour or gas enthalpy.

The conversion returns staged inventories for a caller to use. Projection and VOF transport still need to consume the parcel displacement, and the representation threshold needs a convergence study.

The CPU displacement step now feeds the change in parcel occupancy into pressure projection and transports resolved phase inventories over the resulting fluxes. The six-cell check moves the parcel displacement one cell to the right over 0.4 ms and measures 1e-13 m3 of gas swept back to the left, with gas enthalpy and vapour following the donor inventory. Resolved liquid stays in place in this fixture. The parcel move is prescribed. Displaced groups must remain gas-only and the step currently has no open-boundary inflow reservoir or GPU transport path.

### Representation threshold under axial refinement

A CPU classification check integrates three planar liquid intervals into Cartesian VOF cells. One interval is anchored and two are detached, with volumes of 1.15e-12 and 1.75e-12 m3. Axial spacing is refined from 50 to 25 to 12.5 micrometres at fixed cross-section. At all three resolutions the 1.45e-12 m3 threshold selects only the smaller interval and its parcel recovers 1.15e-12 m3. The 1.0e-12 and 1.9e-12 m3 controls select zero and two detached intervals respectively. This verifies classification and inventory consistency for exact planar input. Dynamic three-dimensional threshold convergence and parcel-to-VOF insertion remain open.

### Parcel reinsertion

A parcel can now be reinserted into a supplied footprint whose total displacement matches its liquid volume. The VOF groups receive the reserved volume and proportional liquid enthalpy, and the routine returns matching per-group momentum credits. The six-cell CPU fixture converts detached liquid to a parcel, moves its displacement through the pressure and VOF transport step, and reinserts 1e-13 m3 at the destination. Gas volume, vapour and gas enthalpy are identical immediately before and after reinsertion. The restored fields pass a subsequent VOF step. Automatic footprint selection and applying the momentum credits to the staggered solver remain open.

The six-cell CPU benchmark now selects the destination footprint from the parcel centre and reserves its multiplicity-weighted volume in that pressure group. The destination must be gas-only and retain positive gas volume. The selected footprint drives displacement and subsequent reinsertion of 1e-13 m3. This is a single-group footprint. Multi-group footprint construction and applying the momentum credits to the staggered solver remain open.

Reinsertion now gathers the incoming staggered momentum using the pre-insertion phase masses, adds the parcel momentum credits, and scatters onto the updated faces using the existing aperture weights and post-insertion masses. The CPU case seeds nonzero incoming gas velocity and verifies the face momentum sum on each axis to 1e-26 N s absolute. The destination face changes from -0.02 to +0.00071344108 m/s. The remap rejects nonzero momentum on an axis with no active face. This benchmark verifies the remap before pressure projection.

The six-cell CPU benchmark also projects the remapped velocity using density rebuilt from the restored phase volumes. The closed one-dimensional domain arrests the destination face velocity to -1.4440944e-13 m/s and develops a 0.0091035081 Pa pressure rise across that face. The subsequent VOF step preserved each group inventory to the fixture tolerance. This verifies closed-domain pressure response and transport after reinsertion.

## V38 visible extinction

The CPU opacity check sums multiplicity times the extinction cross section of each droplet and divides by the sample volume. It uses the visible geometric-optics approximation Qext = 2 for radii of at least 5 micrometres and Beer-Lambert direct-beam attenuation. See [the effective-radius extinction relation, equation 10](https://amt.copernicus.org/articles/19/6607/2026/index.html). The 40-droplet reference gave 0.024630086 inverse metres and 99.97537295% transmission over 1 cm, with the same result for three weighted parcels. The check covers dilute independent-droplet extinction. Wavelength-resolved scattering and Unity rendering remain open.

Unity DX11 passed the six GPU transmission samples, with 99.97538000% transmission over 1 cm and maximum absolute error 4.3742e-8.

## Dielectric glass preview

Enable Use Dielectric Glass Preview on Beaker3DSetup in the overhead-lit apparatus scene. The Built-in Render Pipeline material uses the MAT015 DURAN index of 1.473 at 587.6 nm to set normal-incidence specular reflectance to approximately 3.66%. It renders specular highlights with premultiplied transparency. Refraction, spectral absorption and water rendering are separate remaining work. Visual verification in Unity is pending.

## Dielectric water preview

On Solid Beaker Thermal Preview, enable Use Dielectric Water Preview before entering Play Mode. The existing water mesh uses the same Built-in Render Pipeline dielectric shader as the optional glass material. The representative visible water index of 1.333 (O01) sets normal-incidence reflectance to approximately 2.04%. The default temperature-coloured mode remains available through the Inspector. The dielectric mode displays surface reflection under the scene lights. Refraction, absorption and visual verification in Unity remain open.
