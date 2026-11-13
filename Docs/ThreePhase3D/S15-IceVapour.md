# S15 ice-vapour exchange and entropy

## V41 isothermal planar film

Run FlowMain --ice-vapour. The CPU fixture uses the existing IAPWS G8-10 humid-air/ice equilibrium (D06) and a constant-density stagnant film with zero dry-air flux, as in the S12 film model. The ice surface temperature is held fixed by a heat reservoir. Diffusivity is prescribed at 2e-5 m2/s and vapour enthalpy uses the S14 constant-heat-capacity approximation. The transferred mass debits the donor, and the difference between vapour and ice enthalpy is accounted for as signed reservoir heat.

At 101325 Pa and 253.15 K, the fluxes at half, full and one-and-a-half times the equilibrium vapour mass fraction were +8.890112e-6, zero and -8.8929454e-6 kg/m2/s. At 263.15 K the corresponding values were +2.1533258e-5, zero and -2.1550559e-5 kg/m2/s. Both temperatures passed the species and heat ledgers. This benchmark prescribes the film and interface temperature. A moving ice interface and transient heat coupling remain open.

## V41 transient lumped ice

The same command tests a finite isothermal ice sample with evolving temperature recovered from ice enthalpy. The gas temperature and conductance are prescribed, and gas vapour inventory evolves. Over 20 ms the 32-step cases cooled to 262.639252 K during sublimation and warmed to 263.658369 K during deposition, from 263.15 K. Vapour changed by +4.1017818e-11 and -4.0919119e-11 kg respectively. Temperature refined across 8, 16 and 32 steps. Water and enthalpy closed at every step, with sensible heat recorded against the prescribed gas reservoir. Spatial heat transport and interface movement remain open.

The isothermal film also returns a signed planar front displacement equal to the negative transferred mass divided by ice density and exposed area. At 263.15 K the 10 ms cases receded by 2.3452473e-10 m during sublimation and advanced by 2.3471316e-10 m during deposition. The equilibrium case had zero motion. Reconstructing the slab mass from its updated thickness closes the water ledger. This verifies the kinematic update. Geometry remapping and spatial heat transport remain open.

The planar fixture rebuilds cut-cell geometry for the initial and displaced ice slab at both temperatures and all three humidity ratios. The summed ice-volume change matches the transferred mass divided by ice density, with the opposite change in fluid volume, within 1e-5 relative or 1e-20 m3 absolute. This checks geometric rebuilding. Gas inventory remapping and spatial heat transport remain open.

The planar gas remap allocates transferred vapour and enthalpy by absolute swept cell volume. Dry air stays in its original cell. All six CPU cases verified the gas ledgers and untouched cells, and both lumped heat regressions passed. This benchmark covers a front moving in one direction with gas donors remaining open. Pressure-driven redistribution remains open.

The isothermal CPU fixture now projects the gas volume mismatch on the rebuilt cut-cell grid. It uses ideal-mixture vapour partial volume at prescribed temperature and pressure. Gas advection tracks dry air, vapour and enthalpy through the top pressure opening. At 263.15 K, sublimation expelled 2.5786305e-11 m3 and deposition drew in 2.5807022e-11 m3. Equilibrium produced zero flow. All six cases matched the analytical boundary volume and closed the scalar ledgers. This is a single-step pressure and transport benchmark at prescribed temperature.

## V42 isolated heat exchange

Run FlowMain --entropy-conduction. The CPU fixture uses the existing exact two-capacity conduction update with constant heat capacities of 2 and 3 J/K, starting at 280 and 340 K. The energy-weighted equilibrium is 316 K. Each body entropy change is its heat capacity times ln(T/Tinitial), obtained by integrating dS = C dT/T. All 16-, 64- and 256-step runs conserved energy and produced nondecreasing total entropy, reaching 0.0222950088 J/K. This check derives entropy from temperature.

## V42 open heated control volume

Run `FlowMain --entropy-open`. The steady CPU fixture mixes two dilute humid-air streams at 101325 Pa with optional heating from a 350 K reservoir. It uses ideal-gas partial pressures and constant molar heat capacities as an analytical benchmark. Species molar inflows determine the outlet composition. Enthalpy rise determines the heat input. At zero storage, entropy production is net outward advective entropy minus heat input divided by reservoir temperature. The independent reference sums the ideal-mixture relative entropy of the inlet compositions and the finite-temperature heating contribution. The matched unheated streams give zero production. Heating alone, mixing alone and combined heating and mixing give positive production. The fully coupled transient beaker audit belongs to V43.

The conservation ledger now accumulates signed heat and advected entropy by external channel. It infers total production from independently reduced entropy storage minus net inward entropy. The open benchmark exercises this ledger over a one-second interval, including a nonzero storage test. V43 still requires wiring accepted beaker boundary fluxes and independently reducing the transient solver state.
