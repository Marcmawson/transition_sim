# S15 ice-vapour exchange and entropy

## V41 isothermal planar film

Run FlowMain --ice-vapour. The CPU fixture uses the existing IAPWS G8-10 humid-air/ice equilibrium (D06) and a constant-density stagnant film with zero dry-air flux, as in the S12 film model. The ice surface temperature is held fixed by a heat reservoir. Diffusivity is prescribed at 2e-5 m2/s and vapour enthalpy uses the S14 constant-heat-capacity approximation. The transferred mass debits the donor, and the difference between vapour and ice enthalpy is accounted for as signed reservoir heat.

At 101325 Pa and 253.15 K, the fluxes at half, full and one-and-a-half times the equilibrium vapour mass fraction were +8.890112e-6, zero and -8.8929454e-6 kg/m2/s. At 263.15 K the corresponding values were +2.1533258e-5, zero and -2.1550559e-5 kg/m2/s. Both temperatures passed the species and heat ledgers. This benchmark prescribes the film and interface temperature. A moving ice interface and transient heat coupling remain open.
