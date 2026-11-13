# S15 ice-vapour exchange and entropy

## V41 isothermal planar film

Run FlowMain --ice-vapour. The CPU fixture uses the existing IAPWS G8-10 humid-air/ice equilibrium (D06) and a constant-density stagnant film with zero dry-air flux, as in the S12 film model. The ice surface temperature is held fixed by a heat reservoir. Diffusivity is prescribed at 2e-5 m2/s and vapour enthalpy uses the S14 constant-heat-capacity approximation. The transferred mass debits the donor, and the difference between vapour and ice enthalpy is accounted for as signed reservoir heat.

At 101325 Pa and 253.15 K, the fluxes at half, full and one-and-a-half times the equilibrium vapour mass fraction were +8.890112e-6, zero and -8.8929454e-6 kg/m2/s. At 263.15 K the corresponding values were +2.1533258e-5, zero and -2.1550559e-5 kg/m2/s. Both temperatures passed the species and heat ledgers. This benchmark prescribes the film and interface temperature. A moving ice interface and transient heat coupling remain open.

## V41 transient lumped ice

The same command tests a finite isothermal ice sample with evolving temperature recovered from ice enthalpy. The gas temperature and conductance are prescribed, and gas vapour inventory evolves. Over 20 ms the 32-step cases cooled to 262.639252 K during sublimation and warmed to 263.658369 K during deposition, from 263.15 K. Vapour changed by +4.1017818e-11 and -4.0919119e-11 kg respectively. Temperature refined across 8, 16 and 32 steps. Water and enthalpy closed at every step, with sensible heat recorded against the prescribed gas reservoir. Spatial heat transport and interface movement remain open.

The isothermal film also returns a signed planar front displacement equal to the negative transferred mass divided by ice density and exposed area. At 263.15 K the 10 ms cases receded by 2.3452473e-10 m during sublimation and advanced by 2.3471316e-10 m during deposition. The equilibrium case had zero motion. Reconstructing the slab mass from its updated thickness closes the water ledger. This verifies the kinematic update. Geometry remapping and spatial heat transport remain open.
