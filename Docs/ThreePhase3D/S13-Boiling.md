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
