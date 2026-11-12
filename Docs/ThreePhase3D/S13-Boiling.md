# S13 boiling and bubble condensation

## V29 planar pure-water Stefan interface

The equilibrium interface obtains temperature and both phase densities by inverting the IAPWS water saturation curve at the supplied absolute pressure. Liquid and vapour share the equilibrium interface temperature. The sum of the incoming heat fluxes divided by latent enthalpy gives the signed phase mass flux. The same transfer sets the interface recession relative to the liquid, the normal velocity jump and the conserved phase volume, vapour and enthalpy increments.

The CPU fixture runs four 2 ms steps through the reconstructed cut-cell interface and pressure transport at 101325 Pa, with a pure-vapour pressure outlet. For heat fluxes of ±10000 W/m², it measured mass flux ±0.0044316977 kg/m²/s, front shift ∓3.6993723e-8 m and normal velocity jump ±0.0074104975 m/s. Both directions matched the analytical planar balances at the physical liquid-to-vapour density ratio. Water and enthalpy closed including the outlet ledger.

Run the reference executable with `FlowMain --pure-stefan`. This planar fixture prescribes the heat flux and thermodynamic pressure. Curvature, resolved temperature gradients, seeded bubbles, and GPU verification remain open.

## V30 spherical seeded-bubble growth

The spherical interface uses the ambient liquid pressure plus the Young–Laplace jump 2σ/R to obtain the vapour saturation state. For a prescribed bulk liquid temperature, its quasi-steady spherical conduction flux is k(Tbulk − Tsat)/R. The pure-water Stefan balance converts that flux to vapour mass, and the transfer closes the liquid and vapour enthalpy ledger.

Run `FlowMain --bubble-growth`. At 101325 Pa and 2 K superheat, the zero-capillarity, constant-density reference grew from 0.5 to 0.67207957 mm in 0.1 s and consumed 0.0010087517 J. The 64-step CPU midpoint result matched the analytical radius within 4.1e-7 relative error and closed latent heat against vapour mass gain. A separate capillary check found a 15.816147 µm zero-flux radius for the prescribed tension. Twice that radius grew at a reduced flux, while half condensed.

This benchmark assumes a spherical seed and a quasi-steady prescribed liquid temperature field. The capillary check tests instantaneous rates. Time evolution with capillarity, resolved bubble motion and thermal fields, and GPU verification remain open.

## V31 spherical bubble condensation

Run `FlowMain --bubble-condensation`. The spherical CPU reference begins with a 0.5 mm radius at 101325 Pa in liquid 2 K below saturation. It uses constant vapour density, zero surface tension and quasi-steady spherical conduction. The analytical squared radius decreases linearly in time. The 128-step midpoint run reached 0.25000107 mm at the reference time for a 0.25 mm cutoff, improving on the 32-step run.

The fixture advances the unresolved tail to its analytical extinction time and transfers the entire remaining vapour inventory to the liquid using the same phase enthalpy balance. Extinction occurred at 0.12395206 s. The run condensed 3.1293235e-10 kg, released 0.00070612296 J of latent heat and left zero vapour. A general grid-scale cutoff, capillary time evolution and resolved flow verification remain open.

## V30–V31 capillary radius evolution

Run `FlowMain --capillary-bubble`. The spherical model recovers the radius from vapour mass, with vapour density evaluated at the saturation pressure including the capillary jump. A safeguarded Newton iteration uses the derivative along the saturation curve. The CPU fixture advances vapour mass with the midpoint phase flux and checks radius, capillary pressure and phase-transfer enthalpy on every step.

At 101325 Pa over 20 ms, a 0.5 mm seed reached 0.53760683 mm at 2 K superheat and 0.45645248 mm at 2 K subcooling. Latent heat into the interface was +0.00017184167 J and −0.00016913939 J, respectively. The 16- and 64-step runs differed by at most 1.05e-10 m in final radius. This fixture uses quasi-static spherical equilibrium and prescribed liquid heat conduction. It checks the phase-transfer ledger and excludes mechanical pressure work and surface energy from that ledger. Resolved bubble motion and GPU execution remain open.

## Seeded bubble buoyancy on the cut-cell grid

Run `FlowMain --bubble-rise`. The CPU fixture initializes a tessellated vapour seed of 1.6 mm radius in an 8³ closed grid with 1 mm cells. The variable-density pressure projection and phase momentum transport advance the bubble at saturation densities. Over six 0.4 ms steps the vapour centroid rose 2.6590927e-5 m under gravity and stayed still in the zero-gravity control. The phase volumes, vapour mass and both enthalpy inventories closed within tolerance.

The fixture exposed trace-phase roundoff in the CPU scalar transport. The donor update now separates retained material from receipts, limits transfers to the available donor volume, and carries its remaining enthalpy and vapour. Volume closure preserves the smaller phase directly. This check isolates early buoyant motion with phase change, viscosity and capillarity disabled. Rise-speed convergence, coupled growth and motion, and the GPU path remain open.

## Growing bubble with gravity and an open outlet

Run `FlowMain --bubble-growth-rise`. The 3D fixture enables an open top boundary and applies a prescribed 10000 W/m² heat flux to the reconstructed pure-water interface. It rebuilds the pressure density from the transported phase fractions and projects gravity and the Stefan volume source together. The projected velocity is carried to the next step. This fixture omits a separate momentum-advection update during phase exchange.

After six 0.4 ms steps, the gravity case gained 1.1182693e-9 m³ of vapour and its centroid rose 68.709737 µm. The open-outlet, zero-gravity control rose 41.688666 µm with phase change active. Both runs balanced vapour production, total water mass and enthalpy against the signed outlet ledger. This is an integration check of the coupled source and flow response. Heat flux is prescribed, and growth and rise rates remain unvalidated under spatial refinement.

## Momentum transport during pure-water phase exchange

The pure-water coupled step advects momentum with the same face mass transfers as the phase inventories. For a face-aligned phase transfer between pressure groups, the exchanged mass carries the donor group's aperture-weighted staggered momentum into the receiving group. A mixed-group transfer changes the phase composition without adding a group mass or momentum source. The momentum remap checks the new staggered mass against the updated phase volumes and returns the wall and outlet impulses.

The growing-bubble fixture now carries the remapped velocity to the next projection. The gravity case rose 81.593956 µm and gained 1.1635088e-9 m³ of vapour. The zero-gravity outlet control rose 43.50105 µm. Both runs closed the momentum ledger in each axis. The planar fixture passed four mixed-cell steps and a single initially face-aligned step in each flux direction. A longer run starting from a face-aligned interface missed the velocity-jump target, so repeated transitions between aligned and mixed geometry still need verification. The momentum model uses donor velocity for the transferred mass. Interfacial recoil stresses, spatial refinement and GPU verification remain open.

## V29 continuation from a face-aligned interface

When one step produces more vapour than the existing layer in a mixed cell, the old PLIC sweep can pull liquid into the adjacent gas group and create a second reconstructed interface. For a grid-parallel section covering the shared aperture, the pure-water source now deposits the new vapour and its enthalpy in the neighbouring pure gas group. The same phase transfer supplies the corresponding momentum and pressure-volume increments. This treatment is limited to full-aperture, grid-parallel sections.

The initially aligned and initially mixed planar fixtures now both pass four steps in both flux directions. They match the analytical front shift of ±3.6993723e-8 m and velocity jump of ±0.0074104975 m/s. The evaporation test also checks that the adjacent vapour cell remains free of liquid. The growing-bubble and S12 CPU regressions passed.

## Unity pressure backend verification

Unity DX11 passed the four planar cases and growing bubble after the trace-phase correction. The growing bubble rose 47.086119 µm with gravity and 10.661593 µm without, and gained 5.8324244e-10 m³ of vapour.

The same menu now runs a resolved condensing bubble on the same 8³ grid. Run `FlowMain --bubble-condensation-rise` for the CPU reference. With a prescribed outward heat flux of 10000 W/m², the vapour volume fell by 5.6192099e-10 m³ over six 0.4 ms steps. Its centroid rose 27.708753 µm with gravity and fell 10.47588 µm without gravity. Both paths check water, enthalpy, and momentum balances each step, including the outlet ledger. The new GPU case uses the same 1 µm displacement and 1% vapour-volume tolerances as the growth case. Its device result is pending. This fixture prescribes heat flux and does not model a resolved subcooling field or apparatus nucleation.

The first Unity run exposed sensitivity to pressure roundoff. Changing the CPU convergence check from every 16 iterations to every 31 reproduced a substantial change in bubble rise and growth. Pure-water heat exchange now excludes reconstructed sections where the smaller phase occupies at most 1e-10 of its pressure-group volume. The trace phase stays in the conserved inventories. The corrected CPU reference rises 47.08612 µm with gravity and 10.661593 µm without gravity, with a vapour volume gain of 5.8324244e-10 m³.

In Unity, select **Physics Sim → Three Phase 3D → Run S13 GPU boiling checks**. The menu runs the shared planar Stefan and growing-bubble fixtures with GPU pressure projection and CPU interface reconstruction, phase transport and momentum remapping. The planar cases check the analytical front shift, velocity jump and boundary ledgers. The growing case also checks the CPU reference within 1 µm for both centroid displacements and 1% for vapour volume gain. The shared CPU fixtures passed and the changed scripts compiled against Unity 2022.3.46f1 assemblies. Device results are pending.
