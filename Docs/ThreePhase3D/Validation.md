# Verification and validation requirements

Revision 1.0 · 10 November 2026 · All simulator tests are initially PENDING.

## 1 What these tests establish

Verification checks that the chosen equations are solved correctly. Validation compares predictions to physical observations in a defined regime. Agreement with an analytical constant-property benchmark verifies the solver; it does not by itself validate boiling water in a particular beaker.

Every test must pin its geometry, physical inputs, boundary conditions, initial state, source equations or dataset, physical duration, grid sequence, timestep sequence and observable definitions before results are produced. Use source-linked configurations, not visual adjustments made after viewing the desired answer.

Published benchmark geometry and synthetic mathematical test inputs are deliberately specified conditions. They are not substitutes for empirical water properties. Where an idealised analytical test assumes constant properties or equal phase densities, label that assumption explicitly.

## 2 Proposed numerical acceptance targets

These are project engineering targets, not empirically measured constants and not claims of achieved accuracy. The reviewer may require tighter targets where an observable is sensitive to small errors. Any relaxation must document the reason and impact.

| Quantity | Initial target | Interpretation |
| --- | --- | --- |
| CPU property implementation | Match official check values to their published precision | Includes derivatives and phase conventions |
| GPU density and transport lookup | Relative error at most 1e-4 against verified reference | Over the declared supported table domain |
| GPU enthalpy/internal energy lookup | Absolute error at most max(1 J/kg, 1e-4 times reference magnitude) | Also test phase enthalpy differences directly |
| GPU specific entropy lookup | Absolute error at most max(0.001 J/(kg K), 1e-4 times reference magnitude) | Use a coherent reference convention |
| Closed advection water mass | Relative residual at most 1e-6 over the full test | Report total correction separately |
| Closed coupled phase-change water mass | Relative residual at most 1e-4 over the full test | Includes all phases and parcels |
| Closed/simple thermal energy | Relative residual at most 1e-3 | Relative to a defined thermal change, with absolute floor |
| Full open beaker energy | Relative residual at most 1e-2 | Includes measured/modelled boundary exchanges |
| Step volume-constraint error | dt times max absolute divergence-minus-source at most 1e-5 | Dimensionless; include cut-cell treatment |
| Pressure algebraic residual | Relative norm at most 1e-6 plus absolute compatibility requirement | Zero-RHS cases use the volume-constraint target |
| Smooth conduction profile | Relative L2 error at most 1% on the accepted grid | Also demonstrate grid/timestep convergence |
| Stefan front position | Error at most 2% after the initial singular transient | Report absolute error in m as well |
| Static spherical drop pressure jump | Error at most 1% on the accepted resolved grid | Also limit spurious flow |
| Static parasitic velocity | At most 1e-3 times sqrt(sigma/(rho_l R)) | Benchmark capillary velocity scaling |
| Canonical resolved bubble observables | At most 2% versus the chosen numerical reference | Match the reference's dimension and parameters |
| Apparatus experiment | Within a preregistered combined uncertainty criterion | A numerical threshold alone is insufficient |
| Performance | Measure on the user's actual GPU | No unverified FPS or real-time claim |

For quantities near zero, specify an absolute physical scale before computing relative error. For example, use imposed thermal energy or a known latent heat budget, not the arbitrary zero of enthalpy.

A pressure residual is meaningful only with a declared norm, operator scaling and convergence definition. Report the post-projection volume-constraint residual directly; reducing an incorrectly assembled matrix's residual does not verify the physical constraint.

## 3 Convergence protocol

Use at least three systematically refined spatial grids and an independently refined timestep. Keep geometry, material inputs and physical duration fixed. Separate geometry error, property-table interpolation error, temporal error, algebraic solve error and physical-model uncertainty.

For smooth problems estimate observed order and, when an asymptotic sequence exists, use a Grid Convergence Index or equivalent documented estimate [B04]. Do not report an invented convergence order if errors are non-monotone.

For breakup, turbulence or stochastic nucleation/parcel events, compare appropriate integrated or statistical observables with uncertainty. Pointwise interface topology may not converge monotonically. Repeat random-seed runs where the model is stochastic.

A small grid difference is not proof of physical accuracy if every grid shares the same wrong boundary condition or omits the same physics.

## 4 Foundation and flow cases

| ID | Case and reference | Recorded observations and gate |
| --- | --- | --- |
| V01 | SI conversion and experiment configuration | Verify length, area, volume, T conversion, mass/volume input consistency and absolute/gauge pressure conversion. Reject missing material range or incompatible configuration. |
| V02 | Pairwise inventory transfers | Transfer known mass/enthalpy/momentum among two inventories with no boundary flux. The combined inventory remains constant; an intentionally unmatched transfer must fail. |
| V03 | Official water thermodynamic check states [D01] | Check EOS pressure, density inversion, h, u, cp, sound speed and derivatives against release tables. Test both phase branches and saturation points. |
| V04 | Ice and equilibrium check states [D02, D03] | Check ice density/h/cp/s, melting and sublimation curves, latent differences and reference consistency. Reject unstable/out-of-range states. |
| V05 | Transport and humid air check states [D04–D09, D12–D14] | Compare viscosity, conductivity, sigma, gas density/composition and diffusion across supported states. Check table interpolation independently of its generator. |
| V06 | Cut-cell geometry | Match analytic box/cylinder/sphere volumes and open face areas. Translate and rotate geometry relative to the grid; show decreasing error and no water in glass. |
| V07 | Hydrostatic rest | Compare p_abs = p_surface + rho g depth for constant rho. Measure spurious velocities, wall leakage, pressure nullspace handling and error after grid translation. |
| V08 | Pressure-driven viscous flow and decay | Use analytic Poiseuille flow and a decaying mode in a fully 3D grid. Check viscosity units, velocity profile, convergence and kinetic-to-thermal accounting if enabled. |
| V09 | Interface advection | Translate and deform a known volume through the domain using prescribed velocity. Check phase boundedness, shape error, mass and reverse-advection behaviour. |
| V10 | High density ratio mass and momentum transport [M13] | Transport and deform water/air-scale phase contrasts, including a pressure-free momentum test. Check both mass and momentum flux consistency, not just uniform velocity preservation. |

For V03, IAPWS-95 includes computer-program verification Tables 6–8. Import the released values with their units and significant digits, retain provenance and independently check any transcriptions. Do not paste an approximate web steam table and call it the official oracle.

## 5 Interfaces and free surface cases

| ID | Case and reference | Recorded observations and gate |
| --- | --- | --- |
| V11 | Static spherical drop [M01] | Compare pressure jump 2 sigma/R; check velocity decay and the parasitic-flow limit over several radii/resolutions. |
| V12 | Oscillating drop or capillary wave [M01] | Compare frequency and damping to the matched small-amplitude theory/reference. Resolve both spatial and temporal errors. |
| V13 | Sessile drop and contact-line motion [M08] | Check imposed synthetic angle first, then measured glass advancing/receding angles. Quantify grid/slip-law dependence and hysteresis. |
| V14 | Matched published dam break [B01] | Compare front position, surface height, sensor response and force/pressure impulse using the published geometry and uncertainty. Report non-resolved acoustic peaks separately. |
| V15 | Beaker partial release, slosh and spill | Check initial volume, no wall penetration, headspace, liquid rebound/slosh, resolved sheets/droplets and full domain water inventory. Compare refinement, not just appearance. |

V14 requires the published dimensional setup and sensor bandwidth. A pressure peak is resolution- and compressibility-sensitive; a low Mach computation may reproduce integrated loads while lacking the physics needed for an acoustic peak.

## 6 Thermal and ice cases

| ID | Case and reference | Recorded observations and gate |
| --- | --- | --- |
| V16 | One-dimensional transient conduction | Compare slab temperature profile with a Fourier-series or manufactured solution. Verify temperature-dependent material enthalpy integration separately. |
| V17 | Composite thermal resistance | Compare steady heat rate through two solids and an optional contact resistance with A deltaT divided by total resistance. Confirm equal opposite face energy transfer. |
| V18 | Radial heating of glass | Integrate the prescribed flux to P_abs independently of mesh. Compare radial/through-thickness temperatures with a refined solid reference and verify delay before water heating. |
| V19 | Differentially heated cavity [B03] | Match the published Rayleigh/Prandtl conditions and dimensionality via an extruded 3D mesh with correct symmetry. Compare Nusselt number and velocity extrema; then add a separate genuinely 3D convection case. |
| V20 | Water density anomaly | Use water states below and above its density maximum. Verify buoyancy direction from rho(T,p) and a matching reference, plus thermal energy conservation. |
| V21 | Seeded planar melting Stefan problem | Compare front position and temperature profile with the analytic constant-property solution. Use the declared idealised density assumption of that solution. |
| V22 | Seeded planar freezing Stefan problem | Apply an explicit cold boundary, compare front motion and released latent heat, and show convergence of any numerical mushy interval. |
| V23 | Closed water inventory with a free or compliant volume boundary | At fixed H2O mass, compare volumes from rho_ice and rho_liquid. Check deltaV = m(1/rho_ice - 1/rho_liquid), pressure response and displaced gas without water loss. Do not impose an incompatible constant-volume closed gas boundary. |
| V24 | Rigid ice flotation | For a sufficiently large, capillarity-negligible body, compare submerged volume to m_ice/rho_liquid. Include transient motion, orientation and conservation; repeat with grid offsets. |
| V25 | Ice contact and collision | Check no penetration, reaction forces, energy/momentum transfer and the declared friction/compliance model against analytic limiting cases or a separate contact reference. |
| V26 | Freely melting ice body | Track shape, mass, centre of mass, temperature, angular momentum and meltwater. Compare with a refined coupled reference and eventually a measured melt experiment. |

For the one-phase constant-property Stefan case, with initial material at the phase temperature and a fixed hotter boundary, a standard solution has front position s(t) = 2 lambda sqrt(a t), where a is the relevant thermal diffusivity and lambda is found from the matched Stefan-number relation. For the common melting formulation, Ste = sqrt(pi) lambda exp(lambda2) erf(lambda). State which phase carries conduction and which assumptions make this solution applicable. It is not a substitute for the density-change test V23.

Rigid-body buoyancy must be computed from fluid traction. Adding an analytical buoyancy force during V24 would only test the imposed formula rather than the coupling being verified.

## 7 Gas and phase change cases

| ID | Case and reference | Recorded observations and gate |
| --- | --- | --- |
| V27 | Diffusion-limited evaporation in humid air [M02] | Compare interfacial species gradient, evaporation rate, surface cooling and Stefan flow. Repeat at multiple humidity/pressure states including a no-net-flux equilibrium. |
| V28 | Condensation on a cooler interface | Start with a specified vapour composition and surface temperature. Check condensed mass, latent heating and dry-air conservation; compare a matched sharp-interface reference. |
| V29 | Planar liquid/vapour Stefan flow [M03, M05] | Check mass flux, interface speed, temperature jump treatment and the normal velocity jump j(1/rho_g - 1/rho_l). Use physical water density contrast. |
| V30 | Seeded bubble growth in superheated water [M05] | Compare bubble radius versus time and energy consumption with the matched theory/reference. Measure pressure/capillary effects and conservation. |
| V31 | Bubble condensation in subcooled water [M05] | Compare shrinking bubble radius and released latent heat. Account for the final unresolved inventory conservatively; do not merely delete the bubble. |
| V32 | Validity guards | Request states beyond each material range, invalid species fractions and a flow outside the low Mach envelope. The simulator must stop or mark the run invalid with a reproducible reason. |

For a pure-vapour phase-change test, water/vapour density contrast must not be reduced for numerical convenience. If a synthetic reduced-ratio benchmark is also used, keep it separately named.

## 8 Evidence GPU and dispersed phase cases

| ID | Case and reference | Recorded observations and gate |
| --- | --- | --- |
| V33 | Recorder and hash integrity | Check manifest completeness, version/hash mismatch detection, stale-evidence rejection and export/reload identity. A modified shader or table must invalidate affected evidence. |
| V34 | Conservation under representation changes | Convert known inventories between VOF, parcels and bodies. Test mass, enthalpy, momentum and angular momentum, including boundary exit and deposition. |
| V35 | GPU execution correctness and profiling | Check every kernel binding, dispatch bounds, buffer stride, lifetime and race-sensitive dependency; compare small cases to a CPU reference. Record per-kernel timings and actual allocations. |
| V36 | Unity import and baseline integrity | Import the new scene/project using the recorded Editor, compile C# and shaders, and verify all referenced assets. Check legacy scenes still open. |
| V37 | Isolated dispersed droplet [M11] | Check droplet heating, evaporation or validated condensation extension, drag/settling, gas feedback and mass/energy closure. Use matching assumptions; show size and timestep convergence. |
| V38 | Seeded fog ensemble [M12] | Prescribe seed/aerosol conditions; compare growth/evaporation distributions and total condensate against a reference. Vary parcel count/multiplicity and random seed; validate opacity separately. |
| V39 | Thermocapillary interface | Use a specified surface temperature gradient and sigma(T). Compare tangential stress and migration/flow with a matched reference. Check sign and absence of spurious net force. |
| V40 | Cooling and ambient loss | Turn off the burner and compare wall/sample cooldown with an independently parameterised heat-transfer reference or separate experiment. Check tripod conduction and radiative flux accounting. |

V38 does not establish spontaneous homogeneous nucleation in arbitrary clean air. That requires an appropriate additional physical model and data. Humidity loss into an invented visible plume is not acceptable evidence.

## 9 Coupled system and release cases

| ID | Case and reference | Recorded observations and gate |
| --- | --- | --- |
| V41 | Ice/vapour sublimation and deposition | Check an existing ice interface with prescribed ambient vapour state against ice/vapour equilibrium, mass-transfer and heat balances. Demonstrate both flux directions. |
| V42 | Entropy audit | For an isolated two-temperature conduction test, compare final equilibrium and nonnegative total entropy production. For an open case, include heat and mass entropy fluxes and mixing. |
| V43 | Fully coupled beaker conservation | Track every water phase, dry air, solid energy and boundary channel over heating, melting, boiling, condensation and cooldown. Meet the full-run mass/energy targets and report uncertainty. |
| V44 | Independent apparatus validation | Compare preregistered wall/water temperature probes, total mass loss, melt/boil onset and plume/condensation observables with held-out measurements. Fit no parameters to these validation runs. |
| V45 | Performance and refinement invariance | Compare baseline and optimised solver outputs at matched physical/numerical settings; show memory/time scaling and actual sim-time/wall-time. Refinement must not silently change physics. |
| V46 | Final requirement and evidence review | Every core requirement maps to real code, source locators and accepted evidence. Pending GPU runs, missing property data or unvalidated approximations remain explicit blockers. |

## 10 Apparatus measurements and uncertainty

For V44, define sensor positions, calibration, accuracy, temporal response and sampling frequency. Record sample mass, geometry, pressure, room T/humidity, ventilation, heater absorbed-power uncertainty, glass grade and surface preparation.

Preregister observables and their comparison rules. A possible criterion is that bias and discrepancies lie within a combined interval derived from independent measurement, input and numerical uncertainties, at the stated coverage level. The coverage factor and handling of systematic errors must be justified; do not automatically multiply every reported uncertainty by two.

Use sensitivity studies to identify whether uncertainty is dominated by burner power, glass heat capacity, contact resistance, ambient flow, nucleation or numerical resolution. Report that conclusion alongside comparisons.

## 11 What is supplied for review

Each test result contains measured values and units, error definitions, acceptance thresholds, pass/fail/pending, exact inputs, reference-data hash and run ID. Attach spatial/temporal convergence plots, not just one result at the finest grid.

The reviewer checks code and recorded results; the user provides actual Unity/GPU execution where unavailable to the reviewer. No benchmark in this pack has yet been run on the future 3D solver.
