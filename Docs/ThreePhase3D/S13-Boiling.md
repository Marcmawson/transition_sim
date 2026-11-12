# S13 boiling and bubble condensation

## V29 planar pure-water Stefan interface

The equilibrium interface obtains temperature and both phase densities by inverting the IAPWS water saturation curve at the supplied absolute pressure. Liquid and vapour share the equilibrium interface temperature. The sum of the incoming heat fluxes divided by latent enthalpy gives the signed phase mass flux. The same transfer sets the interface recession relative to the liquid, the normal velocity jump and the conserved phase volume, vapour and enthalpy increments.

The CPU fixture runs four 2 ms steps through the reconstructed cut-cell interface and pressure transport at 101325 Pa, with a pure-vapour pressure outlet. For heat fluxes of ±10000 W/m², it measured mass flux ±0.0044316977 kg/m²/s, front shift ∓3.6993723e-8 m and normal velocity jump ±0.0074104975 m/s. Both directions matched the analytical planar balances at the physical liquid-to-vapour density ratio. Water and enthalpy closed including the outlet ledger.

Run the reference executable with `FlowMain --pure-stefan`. This planar fixture prescribes the heat flux and thermodynamic pressure. Curvature, resolved temperature gradients, seeded bubbles, and GPU verification remain open.
