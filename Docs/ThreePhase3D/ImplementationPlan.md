# Implementation plan and stage gates

Revision 1.0 · 10 November 2026

The programmer implements one stage per reviewable branch or pull request. A stage is complete only when its code, scene setup, test results and evidence agree. All stages below are required for the final supported beaker scenario unless a particular feature is explicitly marked as a conditional extension. Earlier stages remain useful verification tools; they must not be labelled the complete simulator.

Proposed paths are below Assets/Scripts/ThreePhase3D unless otherwise stated. Numerical criteria are defined in [Validation](Validation.md), while requirement IDs are in [Requirements.csv](Requirements.csv). No runtime results are asserted by this document.

## S00 Freeze the baseline and experiment definition

Dependencies: none.

Inspect current Git status, remote head, user instructions, Unity version, packages, rendering configuration and available hardware. Record any divergence from the baseline inspected in this specification. Preserve existing scenes and .meta GUIDs. Create the new runtime directory and an architecture note describing actual ownership of fields, buffers and solver stages.

Create ScenarioConfigSI with separate physical inputs, material references, numerical settings and rendering settings. Add configuration validation and a pre-run report of missing measured data. Record the GPU model, VRAM, driver, graphics API and compute/readback support. Use explicit synthetic cases to proceed when apparatus data is missing.

Deliver an empty Beaker3D scene with correctly scaled glass/tripod geometry, camera and overhead light. No water physics may be claimed yet.

Gate: V01 and V36. Record a baseline Console log and exact Git SHA. Identify the build/test workflow and add the Unity Test Framework only if required by the chosen automated tests. Do not change Editor version or rendering pipeline incidentally.

## S01 Build the evidence and conservation infrastructure

Dependencies: S00.

Implement Diagnostics/RunRecorder, ConservedInventory and stable reductions. Define phase mass, dry-air mass, internal energy, kinetic energy, gravitational energy and all external source/flux channels. Keep phase-exchange channels paired so they cancel globally.

Implement manifests, CSV series, snapshots and hashes as specified in [EvidenceProtocol](EvidenceProtocol.md). Add the function-comment format and an evidence-index validator. The validator checks identifiers, paths, hashes and evidence freshness; it must not infer physical correctness from the presence of comments.

Gate: V02, V33 and V34. Inject a deliberately unmatched transfer in a test and demonstrate that the ledger and evidence gate fail. This failure-detection test is necessary before conservation numbers can be trusted.

## S02 Implement and verify material reference models

Dependencies: S00, S01.

Implement Materials/IWaterProperties, IIceProperties, IHumidGasProperties and ISolidProperties as semantic contracts. Define supported states and enthalpy/entropy references. Establish an independently verified double-precision CPU oracle using the official property-check tables, then generate versioned GPU tables.

Separate liquid/vapour branches and phase-equilibrium functions. Include density and thermodynamic derivatives, heat capacity, viscosity, conductivity, surface tension, phase enthalpy differences and sound speed. Use the material register to flag data gaps. Implement humid-mixture transport consistently; no pure-steam-density shortcut for ambient air.

Pin generator/library versions and licences. If an external package implements IAPWS, independently check its published reference points before trusting it. Comparing a GPU table solely to its own generator cannot expose an erroneous generator.

Gate: V03–V05 and V32. Deliver property plots and machine-readable comparisons with units and validity bounds. Missing glass data blocks validated glass heating, not all analytical solver tests.

## S03 Implement three dimensional grid and embedded geometry

Dependencies: S00, S01.

Current implementation and evidence: [S03 grid/geometry checkpoint](S03-Grid-Geometry.md). CPU geometry checks and pinned-Unity C# compilation pass; measured execution of the S03 GPU checks remains pending. Flow and thermal operators are subsequent stages.

Implement Core/Grid3D and Geometry/CutCellGeometry. Provide cell volumes, x/y/z face areas, solid fractions, normals and interface distances. Distinguish fixed solid, moving ice and accessible fluid volume.

Implement GPU resource allocation, indexing, stride checks, ghost regions and a kernel binding registry. Use the same authoritative geometry for thermal transfer, collision and surface rendering. Define tiny-cell treatment with conservative accounting.

Gate: V06 and V35. Verify analytic volumes and surface areas for boxes, cylinders and spheres over translated and rotated grids, including thin walls. Record memory allocations and resolution-to-wall-thickness ratio. Do not hide geometry error by rescaling density.

## S04 Implement the single phase flow core

Dependencies: S02, S03.

Implement Flow/MomentumPredictor3D, VariableCoefficientProjection and BoundaryConditions3D. Use compatible divergence, gradient and pressure operators. Include physical viscosity and gravity. Handle closed-box pressure nullspaces and open pressure boundaries explicitly.

Implement iterative convergence criteria, maximum-iteration failure, timestep selection and rollback. All field solves run on the GPU, with a CPU reference for small verification grids where useful.

Gate: V07, V08 and the pressure portion of V35. Demonstrate hydrostatic rest, pressure-gradient flow, viscous dissipation and convergence under timestep refinement. Record actual pressure residual and divergence after correction.

## S05 Add conservative liquid and gas interface transport

Dependencies: S04.

Implement Flow/GeometricVof3D with PLIC reconstruction and phase-volume face fluxes. Couple phase mass and staggered momentum fluxes to that transport. An interface-advection benchmark alone does not prove that momentum is consistent at large density ratios.

Implement advection of energy and gas species using the same transports. Add bounded conservative limiting and report any roundoff reconciliation. Include a large-density-ratio translating/deforming interface test.

Gate: V09 and V10, with water/air-scale density contrast. Deliver phase-volume, mass, momentum and interface-shape errors. Unaccounted saturate/clamp operations fail the gate.

## S06 Add surface tension wetting and non-isothermal stresses

Dependencies: S05.

Implement PhaseChange/SurfaceGeometry and balanced capillary forces. Add compatible height-function curvature, fallback diagnostics, temperature-dependent sigma and Marangoni stresses. Add measured wetting data support and the declared contact-line regularisation.

Gate: V11–V13 and V39. Check spherical-drop pressure, parasitic currents, capillary oscillation, sessile-drop angle and an imposed surface-tension-gradient case. A stationary-drop pass does not substitute for translating-drop verification.

## S07 Validate dam break and free surface motion

Dependencies: S06.

Approved exception: S07 implementation may begin with V11 open. The S06 1 mm static-drop run measured a peak parasitic velocity of 0.1411 mm/s against the proposed 0.1016 mm/s limit. S07 acceptance for coupled cases sensitive to weak interface flow remains provisional until V11 closes or case-specific controls establish that numerical currents are small enough for the measured observable. See [S06 surface geometry results](S06-SurfaceGeometry.md).

The first CPU check applies gravity to the coupled cut-cell predictor before variable-density pressure projection. Run `dotnet run -c Release --project Tools/ThreePhase3D/SurfaceGeometryChecks.csproj -- --beaker-gravity`. The check starts the existing mixed-density beaker fixture at rest with a 1 ms timestep. Its zero-gravity control stays at rest. With 9.81 m/s² downward acceleration, the first pressure solve produces a peak of 0.04464 m/s and the second step redistributes 1.46e-8 m³ (sum of absolute group-volume changes), with 1.69e-21 m³ total liquid drift. This is an acceleration and inventory check ahead of the dedicated release scene and V14/V15 validation.

Build a partial-volume 3D water release, collision with the beaker base, sloshing, resolved spray, overflow and a gas-filled headspace. Include the physical exterior so escaped water remains in the domain inventory.

The CPU release fixture initializes a 20 × 20 mm rectangular water column 10 mm above the beaker floor. The requested mass or volume determines the column height, with density from the liquid-water property provider at the requested temperature and pressure. Exact overlap with fully fluid cells sets the starting liquid inventory and gas fills the available volume around it. The water starts at rest. Run `dotnet run -c Release --project Tools/ThreePhase3D/SurfaceGeometryChecks.csproj -- --beaker-release`. At 293.15 K and 101325 Pa the 9 mL release corresponds to 8.984 g. The two input modes gave matching fields and four 1 ms gravity steps conserved liquid volume to 1.69e-21 m³. This check covers initialization and early motion in the coarse closed-domain fixture. Impact, slosh, overflow, spray and Unity/GPU behavior still need validation.

Create a separate scene matching the geometry, initial conditions and sensors of a selected published dam-break experiment [B01]. Preserve the experiment's sidewall conditions; do not compare an unrelated cylinder and call it validation.

To view the beaker release, open `Assets/Scenes/ThreePhase3D/BeakerReleaseS07.unity` and enter Play Mode. **Spawn Water** prepares the 9 mL fixture on the CPU and loads it paused into the GPU cut-cell session. The preview adds a repeatable, volume-preserving height perturbation of up to 3 mm. The session uses GPU clipping, scalar transport, momentum remap and pressure projection, with gravity added to the predictor on the host. Existing GPU buffer readbacks are synchronous. Use **Physics Sim → Three Phase 3D → Run S07 GPU water release check** to check four steps and report mean wall-clock step time on the target machine. **Step** advances 1 ms and **Run** advances repeatedly. The water display reconstructs an interpolated surface at 2.5 mm sample spacing from the 10 mm solver cells. The water shader discards fragments inside the glass and permits exterior water to be shown. The surface threshold is for visualization. **Spawn Ice** places a stationary 20 mm block on the floor for inspection. The ice preview has no thermal or motion solver attached. Either spawn replaces the previous contents, and **Clear** empties the preview.

Gate: V14 and V15. Compare front position, surface height, integrated impact force/pressure impulse and published uncertainty. Report point-pressure limitations of the low Mach model. The beaker visual scene is an integration test, distinct from the matched experimental benchmark.

## S08 Add conjugate glass and tripod conduction

Dependencies: S02, S03, S05.

Implement Thermal/SolidConduction, CoupledHeatFaces and RadialHeatFluxBoundary. Match face heat flux across glass/fluid and glass/tripod interfaces. Apply the measured or explicit synthetic radial heat input to the external glass base only.

Record absorbed heater power, boundary radiation, support conduction and ambient heat exchange independently. Include a pre-run material-range check and heat-capacity check.

Gate: V16–V18 and V40. Demonstrate transient glass warming, radial spreading, nonuniform base temperature and heat loss after switch-off. Show spatial refinement through wall thickness. A direct heat source in water is not completion.

## S09 Add fluid heat transport and natural convection

Dependencies: S08, S04, S05.

Implement Thermal/FluidEnthalpy with conservative advection and solid/fluid exchange. Derive density changes and pressure/volume constraints consistently. Include the density anomaly near freezing.

Build a narrow-range classical cavity benchmark and a wider-range water heating case. Do not use a constant Boussinesq coefficient for the complete water temperature interval.

Gate: V19 and V20. Check heat flux, temperature profiles, flow response and energy storage. Show grid/timestep convergence and distinguish physical diffusion from numerical mixing.

## S10 Add stationary melting and freezing

Dependencies: S09.

Implement PhaseChange/SolidLiquidInterface and the pure-water enthalpy inversion. Start with seeded, stationary ice and a planar Stefan problem. Transfer mass, enthalpy and phase volume conservatively, using pressure-dependent melting equilibrium.

Resolve internal ice conduction. Distinguish a physical phase boundary from any numerical regularisation interval. Allow freezing from an explicit cold boundary; record the heat extraction.

Gate: V21–V23. Demonstrate front position, latent-heat consumption, temperature plateau, volume expansion/contraction and convergence as numerical smoothing is reduced. No moving-body fidelity is claimed at this stage.

## S11 Add moving ice and collision coupling

Dependencies: S10, S04, S03.

Implement Solids/IceBody, traction integration, implicit or converged pressure/body coupling and continuous contact handling. Track body momentum, angular momentum, centre of mass, orientation and inertia.

Start with nonmelting floating/settling ice at controlled temperature, then extend to melting bodies. Implement conservative mass/energy/momentum transfer from the changing solid geometry. Handle freezing attachment and geometry changes without teleportation or mass deletion.

Gate: V24–V26. Demonstrate hydrostatic flotation, transient motion, rotation, collisions and a melting-body case. An Archimedean final-height rule is not an implementation of this stage.

## S12 Add ambient humid air and species driven phase exchange

Dependencies: S09, S05, S02.

Implement Gas/HumidGasTransport and the low Mach EOS source term. Include gas thermal/compositional buoyancy, water-species diffusion, diffusive enthalpy and compatible open/backflow conditions.

Implement a coupled interface equilibrium solve for temperature, gas composition and liquid/gas mass flux. Use conservative interface displacement and the associated volume expansion and momentum conditions.

Gate: V27 and V28. Compare diffusion-limited droplet evaporation, humidity-controlled evaporation/condensation and a Stefan-flow benchmark [M02]. Verify below-boiling evaporation and no double counting of water mass.

## S13 Add resolved boiling and bubble condensation

Dependencies: S12, S08.

Implement a well-conditioned pure-vapour and mixed-gas interface treatment. Add seeded-bubble benchmarks, then apparatus-specific boiling nucleation using measured cavity/roughness and wetting data.

Include local pressure, surface tension, subcooling, vapour expansion, buoyant flow and condensation of bubbles entering cooler liquid. Bubble velocity must emerge from the resolved flow rather than a prescribed upward speed.

Gate: V29–V31 using boiling benchmarks [M03, M05]. A bubble spawned visually when a cell reaches a temperature threshold fails the stage. Distinguish degassed-water vapour bubbles from dissolved-air bubbles.

## S14 Add dispersed spray fog and optical rendering

Dependencies: S12, S13 for integrated boiling plume; single-droplet tests can run earlier.

Retain resolved droplets in VOF. Add a separate parcel inventory for unresolved spray and condensed fog when the physical feature scale requires it. Use a multiplicity-based droplet representation inspired by M12 for fog, with measured aerosol/seed data and validated growth/evaporation. Use M11 only within its verified isolated-droplet heat/mass-transfer regime.

Couple every parcel exchange to gas/VOF/solid inventories. Implement deposition, re-evaporation, open-boundary exit and conservative conversion between resolved and parcel droplets. Add physical optical parameters and an overhead-lighted glass/water renderer.

Gate: V37 and V38 plus V34. Match droplet growth/evaporation, terminal motion or its resolved reference, deposition inventory and statistical parcel convergence. A white-particle plume can only be enabled as a qualitative preview until these gates pass.

## S15 Add sublimation deposition and entropy auditing

Dependencies: S10, S12; S14 for deposited dispersed droplets.

Implement ice/vapour exchange where relevant, with ice/vapour equilibrium and matched energy/species fluxes. Add entropy storage, heat and mass entropy transport, and model-consistent entropy production.

Gate: V41 and V42. Validate an ice/vapour mass-flux test, an isolated heat-exchange entropy test and an open heated control-volume audit. Entropy is derived from physical state, never added as a driving force.

## S16 Complete the coupled beaker experiment

Dependencies: S07–S15.

Create the full requested scene: tripod, glass, central radial heat input, partial liquid release or ice on the base, external air, simultaneous phase changes, overhead illumination and SI probes. Add presets for ice heating, liquid dam break, boiling, wall condensation and cooling/freezing. Implement save/reload of the complete conserved state, bodies, parcels, solver time, configuration identity and any random generator state; compare a restarted run with uninterrupted execution.

Fit or measure apparatus-dependent inputs using a calibration dataset. Reserve separate runs for validation. Do not tune to a validation temperature trace and then treat the same trace as independent evidence.

Gate: V43 and V44. Run the complete mass/energy/entropy audit with spatial and timestep refinement and documented property/input uncertainty. Export a video as supporting evidence, never as the sole validation.

## S17 Optimise without changing physical meaning

Dependencies: accepted correctness gates for affected modules.

Profile CPU/GPU time per kernel, pressure convergence, property lookup cost, allocations, readbacks and mesh extraction. Add block refinement, improved preconditioners or rendering LOD only with conservative transfer and before/after numerical comparison.

Do not optimise by changing viscosity, latent heat, gravity, gas density ratio, surface tension or absorbed power. Larger timesteps need a demonstrated error budget. Document any mixed precision.

Gate: V35, V45 and all affected regression benchmarks. Record actual device timings and simulated-seconds/wall-second. If accuracy is slower than real time, retain the validated physical settings and offer offline execution.

## S18 Review and release the declared fidelity level

Dependencies: all core stages.

Create a fixed commit, a requirement-by-requirement evidence index, a material validity manifest, reproducible benchmark configurations and an honest known-limitations record. Check code comments against implementation, tests and local run outputs.

Gate: every core requirement has accepted evidence and V46 passes. Report separately: imported/compiled, numerical tests passed, experiment comparisons passed and user-local full-scene execution passed. Do not merge a feature's claimed accuracy solely because its code compiles.

## Stage handoff contents

For every stage provide:

- Exact changed files and integration points.
- Equations and primary-source locators used.
- Assumptions, units and data provenance.
- Numerical scheme and coupling order.
- Required scenes, controls and expected observables.
- Tests run, tests pending and attached results.
- Function comments and requirement mappings.
- A local run instruction and a rollback note.
- A statement of which next stage is now unblocked.

One reviewable change may contain supporting tests and data. Avoid mixing a new physical model with unrelated rendering, Unity upgrades or broad refactors.
