# S12 humid air and phase exchange

The closed gas diffusion step applies a binary mass-fraction gradient across gas-open cut-cell faces. It exchanges equal and opposite water-vapour and dry-air mass and transports the corresponding species enthalpy. The caller supplies gas-open area, group gas mass, component enthalpies and the binary diffusivity. An explicit timestep guard prevents overdraw. The two-cell CPU check recovered 8.910599e-10 kg vapour transfer and conserved total enthalpy.

The low Mach helpers derive the projection source from the fixed-mass change in EOS gas volume and derive buoyancy relative to the ambient hydrostatic pressure gradient. The CPU checks produced upward accelerations of 0.16780046 m/s² for a 5 K warming and 0.029469955 m/s² for an increase in water mass fraction from 0.005 to 0.01.

The planar film solve uses the S02 chemical-equilibrium composition and a Stefan correction for zero net dry-air flux. It balances conductive heat from the liquid and gas against latent heat at the interface. The dilute approximation uses pure saturated vapour enthalpy and prescribed gas-film thickness and thermal conductances. The CPU fixtures returned 292.931469 K for an evaporating interface between 293.15 K reservoirs and 283.370361 K for a condensing interface between 283.15 K reservoirs. The integrated inventory source closed liquid and vapour mass, enthalpy, and phase-change volume. At matched equilibrium composition, the isothermal flux was zero.

The reference executable entry point is `FlowMain --humid-diffusion`. Gas-open area reconstruction, open-boundary diffusion, integration into pressure and transport, and V27–V28 benchmark comparisons remain open.
