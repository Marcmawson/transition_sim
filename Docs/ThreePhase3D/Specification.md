# Specification for the three dimensional water simulator

Revision 1.0 · 10 November 2026

## 1 Purpose and physical contract

Build a Unity application that computes, renders and records the three dimensional evolution of water in its solid, liquid and gaseous states. The scene contains a glass beaker on a tripod and a heat source below its centre. Heat enters the external glass base, spreads through the glass and transfers to its contents and the surrounding air. Water may be released as a partial-volume dam break; ice may initially rest on the base. Different regions may melt, freeze, evaporate, boil or condense simultaneously.

The scene must expose an interpretable physical state: mass, volume, pressure, temperature, energy, phase fractions, velocity, heat flux, humidity and elapsed simulation time. Numerical failure must be detectable from diagnostics even when an image appears plausible. Requirement IDs are defined in [Requirements.csv](Requirements.csv).

This is a continuum laboratory simulator, using experimentally grounded thermodynamic and transport properties. It is not a molecular simulation. Continuum equations and closure laws have validity ranges; attaching SI labels to arbitrary acceleration, viscosity, phase-switch thresholds or particle lifetimes does not establish a physical model.

The target architecture below is a recommendation based on conservation, interface behaviour and testability. It combines published components through explicitly specified interfaces. No cited paper is asserted to solve this entire coupled problem without further engineering.

## 2 Fidelity and supported conditions

### 2.1 Initial declared regime

Support ordinary pure H2O, ice Ih, liquid water, water vapour and a non-condensable air mixture. The fluid domain includes headspace, the outside of the beaker and a plume region above the rim. Glass and tripod occupy separate solid regions.

Use the inherited prototype's 0.25 kg ice at 223.15 K and room air at 289.15 K as one optional comparison scenario. Those are chosen initial conditions, not universal water properties. The existing 2D dimensions do not define a 3D beaker; create and record a separate measured or explicitly synthetic 3D geometry.

A proposed first verification envelope is 223.15–473.15 K for water and gas where the selected property formulations and phase states are valid, and 80000–120000 Pa ambient pressure. This is an intended experiment envelope, not an assertion that every property provider supports every state inside that rectangle. Intersect all provider validity domains and exclude invalid combinations. Glass and tripod require their own supported temperature intervals. Heating outside available material data invalidates that run.

Use a low Mach, variable density formulation for the open beaker. It retains gas density changes from temperature and composition and expansion from phase change, while filtering acoustic waves. Record the peak Mach number using the local sound speed. Initially flag Ma greater than 0.1 for review; that engineering guard must itself be tested against a compressible reference. Low Mach does not mean a constant density gas.

Sealed vessels, rapid pressure transients, acoustic bubble collapse and shock waves are outside this initial solver's validated claim. A compressible extension is required for those scenarios.

### 2.2 What the solid models represent

Treat glass and tripod as fixed, impermeable solids with thermal inertia and conduction. Treat homogeneous ice as moving rigid material with internal temperature gradients and an evolving melting/freezing boundary. This is an explicit low-deformation approximation, not a claim to reproduce ice creep, crystal-scale anisotropy, cracking or glass failure.

Pressure and contact stresses must be monitored. If deformation, thermal expansion or fracture changes geometry significantly at the required error level, add thermoelasticity and a validated failure law before claiming that case is accurate. Do not describe a high-viscosity liquid or a drag-locked grid as a validated floating ice body.

### 2.3 Resolved and unresolved scales

Liquid sheets, droplets, bubbles, thermal layers and glass thickness need adequate spatial resolution. Report each relevant size divided by local grid spacing. A minimum feature-size warning is required; its threshold must be established by the feature's convergence study.

At beaker scale, many nucleation cavities, wetting films and fog droplets will be below a practical uniform grid's resolution. Use either a refined computation that resolves the phenomenon or an explicitly validated subgrid model. A visual particle emitter is not an acceptable replacement for water mass, latent heat or steam transport.

The final acceptance statement must distinguish resolved quantities, modelled subgrid quantities and unvalidated quantities. A stage being incomplete does not remove its requirement.

## 3 Scene and controls

### 3.1 Physical scene

Use Unity geometry with a declared conversion of one simulation metre per Unity unit. Transform scale must not silently alter density, viscosity or gravity. Define a Cartesian domain in metres, y upward, gravity downward.

The beaker geometry must include the internal cavity, external surface, base thickness, side thickness, rim and any spout that affects overflow. Collision and thermal geometry must derive from the same authoritative surface. Store geometry hash and meshing settings.

The tripod must contact the beaker at defined patches. It supplies mechanical support and conducts heat according to its material and thermal contact resistance. An insulating support is allowed only as an explicit idealised benchmark boundary condition.

### 3.2 Initialisation

Support liquid release by specifying mass in kg or volume in m3 with temperature and pressure; calculate the complementary quantity using the property provider. Spawn the water as a resolved region above or within the cavity, with zero or specified SI velocity, and let gravity produce the dam break and impact.

Do not automatically fill the vessel. Ensure headspace remains. Initial ice of specified mass and temperature is placed without penetration on the base by default, with volume computed from its own density. Gas fills the remaining accessible volume.

Also support deliberate vapour injection with specified mass, composition, pressure, temperature and momentum. Injection adds the corresponding mass, species, enthalpy and momentum to the ledger.

Initialise ambient humidity consistently with temperature, pressure and the selected humid air equation of state. Initialise a compatible pressure field and equilibrate the pressure solve before starting a timed benchmark. Do not mistake numerical initialisation transients for physical boiling.

### 3.3 User controls and instruments

Expose burner power and footprint, gravity, ambient temperature, pressure, humidity, initial water/ice amounts and temperatures, domain extent, numerical resolution and requested playback speed. Each control must identify whether it changes physics, numerical discretisation or display.

Allow pause, resume, single physical step, reset, save/reload and run export. Playback speed changes how much physical time is computed or displayed; it must not scale material properties or heat input.

Probe values must include coordinates in m, T in K with an optional °C presentation, absolute and gauge pressure in Pa, velocity in m/s, density in kg/m3, local water phase masses, volume fractions and heat flux in W/m2. Report probe interpolation support and its size.

Display total ice/liquid/vapour and dispersed-water mass, sensible/latent energy accounting, wall temperatures, absorbed heater power, open-boundary losses, conservation residuals, solver residuals, physical timestep, GPU step time and simulated seconds per wall-clock second.

## 4 Numerical architecture

### 4.1 Selected foundation

Use a staggered Cartesian finite volume grid for conservative transport. Store pressure, thermodynamic variables, species and phase geometry at cell centres, and velocity components on their respective faces. Use geometric VOF with piecewise linear interface reconstruction for liquid/gas boundaries. Retain a reconstructed signed-distance field for geometric queries and one-sided gradients, but VOF and conserved masses remain authoritative.

Use consistent mass, momentum, species and energy fluxes derived from the same phase-volume transport. Changing a velocity advection routine to an unrelated semi-Lagrangian scheme is not sufficient at water/air density contrast. References M01–M04 and the compatible staggered mass/momentum approach M13 in [Sources](Sources.md) support the selected component family. If adopting M13's finer transport grid, include that extra storage and its projection to the momentum grid in the derivation and memory budget.

Glass, tripod and ice cut the fluid grid. Represent accessible fluid volume and open face area explicitly. Resolve solid heat conduction on a conformal or embedded finite volume solid grid coupled by matched interface fluxes. Thin walls may use a separate solid mesh; a homogenised shell model needs its own validation before substituting for through-thickness conduction.

Start with a uniform 3D grid for correctness. Introduce GPU block refinement only after conservative restriction, prolongation, interface reconstruction and coarse/fine flux correction pass verification. Adaptivity is an optimisation stage, not permission to omit thin physical structures.

### 4.2 Component responsibilities

| Component | Responsibility | Main data |
| --- | --- | --- |
| Experiment controller | Physical configuration, resets, stepping, checkpoints | Immutable scenario and run manifest |
| Material service | EOS, saturation, transport and validity checks | Versioned property tables and derivatives |
| Geometry service | Solid occupancy, cut volumes, face areas, interfaces | SDF, cut-cell metrics, rigid body geometry |
| Conservative transport | VOF, phase mass, momentum, species and energy fluxes | Shared face fluxes |
| Flow solver | Variable density momentum and pressure constraints | Face velocity, pressure, residuals |
| Thermal solver | Fluid energy and solid conduction | Phase energy, wall energy, common fluxes |
| Phase interface solver | Interface equilibrium and mass/energy transfer | Surface fluxes and interface velocity |
| Ice solver | Body motion, contacts and shape change | Mass, inertia, thermal field, geometry |
| Gas and dispersed water solver | Humid air transport and unresolved droplets | Species and parcel inventory |
| Diagnostics | Conservation, validity and reproducibility | Reduction buffers and exported series |
| Renderer and UI | Surfaces, lights, probes and overlays | Read-only views of computed fields |

~~~mermaid
flowchart TD
    A["Experiment configuration"] --> B["GPU step controller"]
    B --> C["Conserved fields and face fluxes"]
    D["Material data"] --> C
    E["Solid and phase geometry"] --> C
    C <--> F["Flow and pressure"]
    C <--> G["Heat and phase interfaces"]
    F <--> H["Ice motion and contacts"]
    G <--> H
    C --> I["Conservation and run evidence"]
    C --> J["Rendering and SI probes"]
~~~

The coupling is bidirectional: phase changes alter mass, volume, pressure constraints and geometry, while pressure, temperature and gas composition affect phase change.

### 4.3 Unity organisation

Proposed runtime root: Assets/Scripts/ThreePhase3D. Use focused namespaces for Core, Materials, Geometry, Flow, Thermal, PhaseChange, Solids, Diagnostics and Rendering. Keep constitutive reference calculations and validation utilities in testable C# classes; keep GPU kernels and data layouts separate from MonoBehaviour orchestration.

Proposed scenes: Assets/Scenes/ThreePhase3D/Beaker3D.unity and one small scene per benchmark under Assets/Scenes/ThreePhase3D/Validation. Preserve .meta GUIDs and serialised references. Do not copy the prototype's 2D buffers into 3D without revisiting every flux, face area, volume and boundary condition.

Maintain the current Unity 2022.3.46f1 version and built-in rendering baseline initially. Any version or render-pipeline change needs a separate decision and validation. HLSL compute shaders and asynchronous GPU readback are supported Unity mechanisms, subject to actual device and graphics API capabilities [U01, U02].

## 5 Conserved state and SI quantities

For each Eulerian fluid control volume store phase-resolved mass and energy or an equivalent conservative representation. Store gas species mass, cell phase geometry, and face momentum/velocity. An ice body's inventory includes mass, centre of mass, angular and linear momentum, shape and thermal energy.

Let alpha_i, alpha_l and alpha_g denote ice, liquid and gas volume fractions within the space not occupied by fixed glass or tripod. Their sum is one there. Gas includes both dry air and water vapour; vapour mass fraction Y_v is not a separate gas volume fraction. Dispersed parcels have a separate inventory and, if their volume is non-negligible, displace carrier volume consistently.

Use m_k = rho_k alpha_k V and mass fractions x_k = m_k / sum_water(m_k). These mass fractions must never be substituted directly for alpha_k. The gas EOS acts on its own composition and state; the pure-water EOS does not describe the entire humid-air mixture.

| Quantity | Internal and exported unit |
| --- | --- |
| Length and volume | m and m3 |
| Time and timestep | s |
| Mass and density | kg and kg/m3 |
| Velocity and acceleration | m/s and m/s2 |
| Pressure and stress | Pa |
| Temperature and temperature difference | K and K |
| Energy and power | J and W |
| Specific internal energy and enthalpy | J/kg |
| Specific entropy | J/(kg K) |
| Thermal conductivity | W/(m K) |
| Specific heat capacity | J/(kg K) |
| Heat flux and volumetric heat source | W/m2 and W/m3 |
| Dynamic and kinematic viscosity | Pa s and m2/s |
| Surface tension | N/m |
| Diffusivity | m2/s |
| Interfacial mass flux | kg/(m2 s) |
| Humidity and phase fractions | Dimensionless, with declared basis |
| Spectral wavelength and irradiance | m and W/m2, with spectral basis when used |

Use T_K = T_°C + 273.15. Absolute temperature is required in EOS, radiation and entropy calculations [D00]. The UK presentation uses vapour, metre, colour and other UK spellings; there is no different UK thermodynamic SI system.

If nondimensional variables improve conditioning, define the length, time, temperature, density and pressure scales explicitly. Convert all inputs, outputs, diagnostics and material calls through one audited mapping. No unexplained world-scale multiplier is allowed.

## 6 Governing equations and interface conditions

These equations specify the required balances and conventions. The programmer must derive a matching discrete scheme; this section is not a complete standalone algorithm for cut-cell multiphase flow.

### 6.1 Mass and species

For phase k with density rho_k, velocity u_k and phase fraction alpha_k:

$$
\partial_t(\alpha_k\rho_k) +
\nabla\cdot(\alpha_k\rho_k\mathbf{u}_k) = \Gamma_k .
$$

Interphase sources sum to zero. Water species mass is conserved separately from dry-air mass, including inflow, outflow and parcels. For vapour in the gas:

$$
\partial_t(\alpha_g\rho_g Y_v) +
\nabla\cdot(\alpha_g\rho_g Y_v\mathbf{u}_g)
= -\nabla\cdot(\alpha_g\mathbf{j}_v)+\Gamma_v .
$$

A binary effective-air approximation uses j_v = -rho_g D grad(Y_v), with a measured water-in-air diffusivity model and an equal opposite dry-air diffusion flux. For a multicomponent gas, use a consistent multicomponent formulation. Include diffusive species enthalpy transport in the energy equation. Do not identify the NIST H2O–N2 coefficient as H2O–air without a justified mixture treatment [D08, D09].

### 6.2 Momentum and buoyancy

Away from solid interfaces, the mixture momentum balance has the conservative form:

$$
\partial_t(\rho\mathbf{u})+
\nabla\cdot(\rho\mathbf{u}\otimes\mathbf{u}) =
-\nabla p+\nabla\cdot\boldsymbol{\tau}
+\rho\mathbf{g}+\mathbf{f}_{\Gamma}+\mathbf{f}_{solid}.
$$

Use the Newtonian stress appropriate to the chosen low Mach constitutive model. Derive interfacial momentum and stress jumps, including phase-change mass transfer and recoil when relevant. Match normal and tangential interface velocities through the jump conditions rather than forcing one continuous velocity across a vaporising boundary [M02, M03, M05].

Apply gravity once to the supported conserved mass. If using a hydrostatic pressure split, derive the remaining buoyancy term consistently. Do not add a second Archimedean force to a body whose resolved pressure traction already supplies buoyancy.

Use temperature-dependent water density, including its density maximum near 277 K, for the validated temperature range [D01]. A constant positive thermal expansion coefficient is not valid across freezing-to-boiling conditions. A Boussinesq model is permitted only for separately validated narrow temperature benchmarks.

### 6.3 Pressure and volume expansion

Write p_abs = p_base + pi, with the base pressure convention recorded. p_abs is used for physical properties and saturation; pi is the pressure correction or gauge field according to the solver convention. Do not add an extra hydrostatic head if it is already contained in the solved pressure.

In a constant-density liquid/gas limit, interfacial mass flux produces:

$$
\nabla\cdot\mathbf{u} =
j_{lg}\left(\frac{1}{\rho_g}-\frac{1}{\rho_l}\right)\delta_{\Gamma_{lg}}
+
j_{il}\left(\frac{1}{\rho_l}-\frac{1}{\rho_i}\right)\delta_{\Gamma_{il}} .
$$

For variable-density bulk phases, add the EOS-consistent bulk density change contribution, locally -D(rho_k)/Dt divided by rho_k, with the appropriate phase weighting. Thermal expansion and gas composition changes therefore affect the pressure constraint even away from phase interfaces.

The variable-coefficient projection must satisfy the chosen discrete volume constraint:

$$
\nabla\cdot\left(\frac{\Delta t}{\rho}\nabla\pi\right)
= \nabla\cdot\mathbf{u}^{*}-S .
$$

Use the same discrete face gradients, density coefficients, cut areas and cell volumes in the Poisson operator and velocity correction. Measure the final divergence error relative to S. A projection to zero divergence everywhere would remove the physical expansion of boiling [M02, M03, M10].

Closed low Mach verification cases must also satisfy the global volume constraint. A nonzero integrated expansion source requires a compatible compliant/open volume boundary or an evolving thermodynamic base pressure derived from the EOS. A pressure-nullspace fix alone cannot resolve incompatible net expansion in a sealed constant-volume domain. This does not extend the physical validation claim to rapid sealed-vessel pressure transients.

### 6.4 Thermal energy and enthalpy

Use a coherent thermodynamic reference for all H2O phases [D01, D02]. Internal energy u and enthalpy h are related by h = u + p/rho. The gas mixture also includes constituent enthalpies and a consistent mixture entropy reference.

For each bulk phase, the enthalpy equation includes conservative advection, conduction, pressure work, viscous dissipation when significant, species diffusion enthalpy and physical heat sources. In a single-phase continuum form:

$$
\rho\frac{Dh}{Dt} =
\frac{Dp}{Dt}+\nabla\cdot(k\nabla T)+
\boldsymbol{\tau}:\nabla\mathbf{u}
-\nabla\cdot\sum_a h_a\mathbf{j}_a+Q .
$$

The implemented multiphase finite volume form must additionally honour moving interfaces and interfacial fluxes. If terms are neglected in a low Mach approximation, quantify the omitted energy and show it is below the declared error budget.

Energy supplied at a phase interface changes phase mass through latent enthalpy. It must not simultaneously be counted as an independent latent source and as the same enthalpy difference in transferred mass. Choose one coherent total-energy or phase-enthalpy accounting convention and document the derivation.

Do not interpret increased temperature as a prescribed increase of resolved flow speed. Temperature describes microscopic thermal energy; bulk kinetic energy changes through pressure, buoyancy and stresses. Melting and vaporisation can absorb energy without a corresponding temperature rise.

### 6.5 Liquid and gas interfaces

Define n pointing from liquid to gas and j_lg positive for evaporation. The mass jump is:

$$
j_{lg}=\rho_l(\mathbf{u}_l-\mathbf{u}_\Gamma)\cdot\mathbf{n}
=\rho_g(\mathbf{u}_g-\mathbf{u}_\Gamma)\cdot\mathbf{n}.
$$

Use one-sided temperature and species gradients at the reconstructed surface. Under a local-equilibrium approximation and neglecting small mechanical terms, the heat-flux jump obeys:

$$
(\mathbf{q}_l-\mathbf{q}_g)\cdot\mathbf{n}
=j_{lg}(h_g-h_l),\qquad \mathbf{q}=-k\nabla T .
$$

For a non-condensable gas, solve interface temperature, vapour composition and mass flux together using water chemical-potential or fugacity equilibrium plus energy and species balances. In a validated ideal-mixture limit, the species condition becomes:

$$
j_{lg}(1-Y_{v,\Gamma})
=-\rho_g D\nabla Y_v\cdot\mathbf{n}.
$$

The published pure-steam boiling boundary and the humid-air evaporation boundary must be joined through a consistent interface formulation. Do not add an independent temperature-triggered evaporation rate on top. The limit Y_v approaching one must remain well posed; dividing blindly by 1-Y_v is not an algorithm for pure-vapour boiling.

Use IAPWS equilibrium properties and humid-air fugacity as the reference [D01, D06, D07]. An ideal-gas simplification needs a quantified error comparison. Evaporation can occur below the normal boiling temperature; condensation depends on local water vapour equilibrium, not solely on total gas pressure.

Include curvature and liquid pressure effects consistently for small interfaces when they exceed the error budget. Do not apply a Kelvin correction twice if the chemical-potential calculation already uses the capillary liquid pressure.

### 6.6 Surface tension and wetting

Use sigma(T) from the IAPWS ordinary-water surface-tension release within its validity domain [D05]. Apply balanced surface forces and compatible pressure gradients, with height-function curvature where geometric support is adequate [M01]. When support is inadequate, use a documented fallback and record its use.

Choose and test a sign convention against the static spherical-drop relation p_inside - p_outside = 2 sigma/R. Add tangential stresses from gradients of surface tension where non-isothermal conditions make them significant. Surface tension persists during droplet breakup; droplets form through interface dynamics rather than a switch that disables surface tension.

Solid impermeability alone does not provide surface tension or the correct contact angle. Supply measured equilibrium/dynamic wetting and hysteresis data for the actual glass preparation. Moving contact lines need a declared regularisation, such as a calibrated slip and dynamic-angle law; no-slip at every scale combined with a moving contact line is not a closed physical description [M08].

A 90 degree angle is a useful synthetic test condition, not a universal glass-water property.

### 6.7 Ice and liquid interfaces

Use the pressure-dependent equilibrium melting temperature from consistent water and ice thermodynamics [D01–D03]. At a seeded solid/liquid interface, enforce the appropriate Stefan heat balance and transfer the corresponding mass, energy and momentum.

For a pure material at equilibrium, fusion occurs at a sharply defined temperature for a specified pressure. If a finite numerical enthalpy interval is introduced, classify its width as regularisation and demonstrate convergence as it is reduced alongside the grid. Do not claim a wide artificial mushy range as a measured property of pure water.

Conduction proceeds inside the ice; melting is local. Retain separate ice and liquid densities so melting changes occupied volume and freezing expands it. Update the free surface and pressure constraint to accommodate that change. Do not preserve an ice block's volume while removing its mass.

A fixed enthalpy/porosity model can verify stationary melting stages [M06]. It is not the final model for freely floating solid ice. The final solid-motion coupling must preserve a body's shape apart from resolved phase-boundary motion and actual contact response.

### 6.8 Nucleation and simultaneous phases

Interface-resolved equations grow or shrink existing interfaces; they do not by themselves specify when microscopic vapour or ice nuclei first appear. Use one of two declared protocols:

1. Seeded verification: prescribe initial bubble or crystal geometry and record it. Measure subsequent growth, melting or condensation against a reference.
2. Apparatus validation: use measured cavity/roughness and wetting data with a sourced nucleation model and uncertainty. Hsu's model is a starting point for active boiling cavities, not a universal bubble-spawn rule [M07].

A temperature threshold at 373.15 K independent of pressure, wall condition and subcooling is unacceptable. Dissolved air affects early bubbles and nucleation; either simulate it with supported solubility/transport data or use a degassed-water experiment and record that restriction.

Supercooling and nucleation hysteresis require explicit treatment; forcing all sub-zero liquid to freeze everywhere is only an equilibrium test simplification. Different physical interfaces may advance in opposite directions at the same time.

Include sublimation and deposition in the requirement set if the apparatus conditions make their contribution measurable. They use the ice/vapour equilibrium relation [D02, D03] and the same conservative exchange contract. Their stage is not satisfied by switching ice directly to gas without a flux balance.

## 7 Glass conduction and the heat source

### 7.1 Glass and tripod heat equations

For a stationary solid region:

$$
\partial_t(\rho_s h_s(T))=\nabla\cdot(k_s(T)\nabla T)+Q_s .
$$

Glass temperature is a field through its base and walls. Heating the central external base produces radial and through-thickness temperature gradients that evolve through conduction. An instantaneously uniform wall temperature is not the requested behaviour.

For a local interface between two finite volume regions, use a common conductive heat rate based on thermal resistances:

$$
\dot Q_{a\rightarrow b} =
\frac{A(T_a-T_b)}
{d_a/k_a+R''_{contact}+d_b/k_b} .
$$

A is the physical contact area; d_a and d_b are centre-to-interface distances; R''_contact has units m2 K/W. Apply +Q and -Q to the two inventories once. For a perfectly coupled interface R''_contact is zero by a declared physical idealisation. If an air layer or wetting film exists, resolve it or use its measured effective resistance.

SCHOTT's DURAN data provide density 2230 kg/m3 at 298.15 K and conductivity approximately 1.2 W/(m K) at 363.15 K [D10]. These are state-specific manufacturer values, not full temperature-dependent curves. The consulted DURAN sheet does not supply the required heat-capacity curve. Obtain that data or declare a different fully specified virtual glass material. Do not silently import BOROFLOAT heat capacity into DURAN: D11 describes a different named product.

### 7.2 Burner model

The first implementation uses a prescribed absorbed heat-flux boundary on the external base, replacing a combustion calculation with measured input. Define:

$$
q''_{burner}(\mathbf{x},t)
= P_{abs}(t)\frac{w(r,t)}
{\int_{A_{heated}}w(r,t)\,dA}.
$$

Here P_abs is W absorbed by the modelled glass, w is a non-negative dimensionless radial profile, and q'' is W/m2. Integrate over actual base facets or cut faces, not a count of grid cells.

A fitted Gaussian profile w = exp(-r2/(2 b2)) is an available parameterisation; its width b in m must come from a measurement or an explicitly chosen test. A gas flame can produce a ring-shaped heat-flux maximum. Support tabulated radial profiles rather than assuming every burner peaks at the centre. The user's centre-peaked profile is a valid prescribed experiment condition.

Report absorbed power separately from fuel chemical power. If P_abs is inferred using efficiency, that efficiency is a measured apparatus parameter. It is not universally one. Do not heat the water directly before conduction through the glass.

The burner flame mesh may be visual. A computed flame and hot plume require a separate reacting-gas model with fuel composition, flow, chemistry, radiation and validation. Do not combine prescribed absorbed burner flux with a second computed flame flux on the same surface.

### 7.3 Heat loss and ambient interaction

Resolve air transport around the walls and above the liquid where the domain includes it. Conjugate heat exchange then arises from the solid/fluid calculation. Do not also apply an empirical convection coefficient to the same resolved interface.

At physical interfaces omitted from the domain, a measured or published heat-transfer boundary law can be used, with its range and uncertainty. Thermal radiation requires emissivity/absorptivity or spectral data for the actual glass and surroundings. A grey-surface Stefan–Boltzmann approximation is allowed only with appropriate measured coefficients and a stated error range; transparent glass may need spectral transmission.

Glass–tripod conduction and heat loss continue when the burner is off. Freezing requires heat extraction to a colder boundary, sufficient evaporative cooling, or another explicit cooling mechanism. A 289.15 K room and a positive burner cannot be assumed to freeze an initially warm sample.

## 8 Ice mechanics and collision treatment

Compute pressure and viscous traction on the resolved moving solid boundary and apply the opposite reaction to the fluid. Use body gravity, hydrodynamic force, torque and contact constraints consistently. For changing mass and shape, use a control-volume balance including momentum carried by melt or freezing mass; merely changing mass in F = m a does not preserve momentum.

Update centre of mass, inertia tensor and shape after conservative phase transfer. Transfers between a voxelised body and liquid cells must include position and velocity so angular momentum is traceable. Remeshing, splitting or joining a body requires an explicit inventory reconciliation.

Prevent interpenetration of ice/glass, ice/tripod and multiple ice pieces using continuous collision checks or timestep bounds plus a validated contact solver. Friction, compliance and restitution are measured contact parameters; absent data must be labelled as a declared frictionless or rigid-limit benchmark, not tuned until it looks correct.

Ice and water have comparable densities. Use an implicitly coupled or converged subiterated fluid/body treatment to avoid added-mass instability. Arbitrary velocity damping does not establish correct buoyancy or settling. M09 provides a useful pressure-coupling component reference; phase change and heat transfer require additional work.

A body initially touching the base may lift once the resolved force balance supports it. Do not teleport it to an Archimedean height. Freezing attached to the wall and the detachment of a free solid are separate physical/contact cases and must not be conflated.

## 9 Air vapour droplets and visible steam

Solve gas momentum, heat and water-species transport throughout headspace and the resolved external plume. Use the selected humid-air EOS and transport data [D06–D09]. Thermal and compositional buoyancy must follow density, including the different molar mass of water vapour and air.

Pure water vapour must not be rendered as an opaque white fluid. The familiar visible steam cloud involves condensed droplets. Render gas and condensed water according to their separately represented inventories and optical properties [O01, O02].

Resolved liquid droplets remain part of the VOF model. If fine spray or fog cannot be resolved, add a Lagrangian parcel model that stores number or statistical weight, radius distribution, mass, temperature, velocity and composition. Use two-way mass, momentum and heat exchange with the gas. Record deposition to glass/liquid/ice, re-evaporation and boundary exit.

Use the multiplicity-based representation and condensation/evaporation framework of M12 as the selected starting point for unresolved fog, with a beaker-specific coupling and validation. M11 is a candidate for isolated-droplet heat/mass transfer in its applicable regime. Neither is, by itself, a general water-fog nucleation model. Its dilute, approximately spherical droplet assumptions and condensation extension need explicit validation. Fog birth requires measured aerosol/seed conditions or a separately sourced nucleation treatment. A seeded-fog test is acceptable before that closure exists, but must be labelled.

Transfer between resolved and parcel droplets must conserve mass, energy and momentum. Its numerical size threshold must be convergence-tested. Do not delete small bubbles or droplets or expire parcels on a visual lifetime without reconciling the inventory.

## 10 Boundary conditions

| Boundary | Mechanical condition | Thermal and species condition |
| --- | --- | --- |
| Glass or tripod solid | Impermeable; resolved no-slip away from contact-line regularisation | Matched solid/fluid heat flux, species impermeability except declared deposition |
| Ice interface | Moving solid boundary with phase mass flux | Stefan balance, equilibrium or declared kinetic relation |
| Liquid/gas interface | Stress and phase-change velocity jumps | Coupled heat/species balance and phase equilibrium |
| Far-field open boundary | Compatible ambient pressure and outflow; permit backflow | Ambient T and composition on inflow, conservative transported inventory on outflow |
| Ground/support outside model | Explicit support/contact condition | Measured conductive, insulated or prescribed-temperature condition |
| Symmetry benchmark plane | Zero normal velocity and compatible tangential derivatives | Zero normal heat/species flux |
| Closed verification box | No mass flux; pressure nullspace handled | Declared adiabatic or controlled heat flux |

The beaker rim is an internal geometric opening, not a pressure boundary at every liquid surface cell. Air above the water must remain part of the flow model. Spilled water and spray outside the vessel stay in the computational inventory until they cross a declared external boundary.

Increase external-domain size and check plume behaviour and evaporation rates. A boundary too near the beaker can suppress entrainment or remove heat unrealistically. If a sponge layer is used, record its momentum/energy removal and prove boundary independence of the target observables.

## 11 Thermodynamic properties and entropy

Use IAPWS-95 as the reference for liquid/vapour thermodynamics and consistent ice-Ih thermodynamics for solids [D01, D02]. Derive density, enthalpy, internal energy, heat capacities, saturation and entropy from coherent formulations. Use the separate IAPWS transport and surface-tension releases [D04, D05, D12]. Thermodynamic EOS data alone do not provide all transport coefficients.

Build GPU lookup tables from an independently verified double-precision reference. Preserve phase branches and saturation boundaries; do not interpolate through an unstable or two-phase region as if it were a single phase. Track source revision, generator version, coefficient licence, table bounds, units, interpolation error and derivatives.

A table entry must identify reference temperature, pressure, phase, composition, uncertainty and extrapolation policy. Unsupported states trigger a run validity failure. Clamping a property to its table endpoint is not an acceptable silent continuation.

Specific entropy is calculated from the thermodynamic state; it is not an independent knob that controls water's behaviour. For gas mixtures include mixing entropy. For an isolated adiabatic verification system, total entropy should not decrease beyond numerical error. For the open heated beaker, compute entropy storage plus heat and mass entropy fluxes; the subsystem's entropy can decrease while entropy leaves.

A basic conduction contribution to entropy production is:

$$
\dot s'''_{gen,cond}=k|\nabla T|^2/T^2 .
$$

Also account for viscous dissipation, diffusion, finite-resistance heat transfer and phase-interface irreversibility according to the selected model. Validate the entropy accounting separately from mass/energy. A numerical temperature floor can conceal a nonphysical entropy balance.

## 12 GPU implementation and timestep

### 12.1 Storage and dispatch

Use structure-of-arrays buffers or correctly typed 3D textures, with explicit memory stride and index ordering. Cell storage and x/y/z-face storage have different dimensions. Include ghost regions and cut-cell metrics in the memory estimate. Each kernel must bounds-check the dispatch domain.

Bind every buffer used by every kernel, including buffers not used on a particular branch. Zero-initialise required resources and validate their sizes on the CPU. Record kernel names and required resources in a testable binding registry. This addresses the failure class of a property being set for one kernel but missing for another.

Use separate dispatches or appropriate barriers between dependent GPU passes. A thread-group barrier is not a global synchronisation primitive. Avoid in-place neighbour updates unless the algorithm explicitly proves race freedom.

Query device compute support, memory and API limits. GPU acceleration means transport and iterative field solves run on the GPU. CPU orchestration, property generation, small body solves and exported reductions are permissible. Do not read back full fields synchronously every frame to perform the fluid calculation on the CPU.

### 12.2 Pressure and thermal solves

Implement a variable-coefficient geometric multigrid or a suitable preconditioned Krylov method. Demonstrate convergence for high density contrast, embedded boundaries and disconnected fluid regions. A fixed count of Jacobi or SOR sweeps is not a physical convergence criterion.

Handle pressure nullspaces per connected Neumann component. A closed box and an open beaker have different solvability constraints. Reconcile tiny cut cells through conservative merging or another documented stable method, never by deleting their water.

Use implicit thermal diffusion where useful to remove severe diffusive timestep limits. Iteratively couple material properties, interface temperature, mass flux and pressure/volume constraints until the specified nonlinear residuals are met.

FP32 field arithmetic may be sufficient only after comparison with high-accuracy references. Use stable GPU reductions and higher-precision or compensated accumulation for global ledgers. Report pressure residuals and physically normalised errors rather than relying solely on raw floating-point differences.

### 12.3 Timestep constraints

Calculate the allowed timestep from the selected scheme's stability limits, including advection, explicit viscosity/diffusion, surface tension, gravity/acceleration, moving solids, contact, phase-front displacement and positive phase/species inventory.

Useful scales include delta_x/U, delta_x2/nu, delta_x2/thermal_diffusivity, sqrt(rho_effective delta_x3/sigma), and rho delta_x/abs(j). Their prefactors depend on the algorithm and must be justified by analysis and convergence tests. Do not copy a timestep constant from an unrelated paper without the same discretisation.

Subcycle or reject and retry steps that exceed limits. Log the limiting mechanism and rejected steps. Implicit diffusion does not remove capillary, advection or interface-accuracy constraints. Unity render deltaTime and FixedUpdate frequency are not substitutes for the physical solver timestep.

### 12.4 Illustrative memory budget

For an explicitly assumed **160 bytes per base cell**, excluding additional solid meshes, parcels, mesh extraction, multigrid overhead and graphics buffers:

| Uniform grid | Cell count | Illustrative memory |
| --- | --- | --- |
| 128 cubed | 2097152 | 320 MiB |
| 256 cubed | 16777216 | 2.5 GiB |
| 512 cubed | 134217728 | 20 GiB |

These are arithmetic estimates, not measured allocations. The actual field inventory must supply an allocation report. The GPU/VRAM and required smallest resolved scale remain inputs to the performance decision.

## 13 Coupled timestep contract

Use an auditable sequence with explicit old, predicted and accepted states:

1. Evaluate geometry and valid material state; determine the timestep from all active limits.
2. Predict body movement, phase geometry and interface conditions without committing inventory.
3. Construct common conservative face transports for phase masses, species, momentum and enthalpy.
4. Advance conservative transports using bounded fluxes.
5. Solve solid/fluid heat transfer and interface heat/species equilibrium; obtain candidate phase mass fluxes.
6. Apply matched phase inventory changes and update interface geometry and variable density.
7. Solve momentum and the pressure/volume constraint, including phase expansion and body boundary velocity.
8. Correct body motion and contacts and recalculate coupled thermal/interface quantities.
9. Iterate the coupled quantities until residuals and inventory constraints satisfy the stage tolerances.
10. Reconcile mesh/body/parcel transfers, calculate diagnostics, and accept or reject the complete timestep.
11. On acceptance, advance physical time, export sampled evidence and render read-only state.

The final discretisation may change ordering or use predictor/corrector half-steps. It must document temporal accuracy, coupling residuals and conservation. A single pass through unrelated solvers without a coupling-error estimate is not the final design.

## 14 Conservation and run evidence

Total H2O mass includes solid bodies, liquid VOF, gas vapour and water parcels. Dry air has its own ledger. Glass and tripod thermal energy remain in the coupled system. Track added water, injected gas, boundary fluxes, heater energy, support conduction, radiation and numerical corrections.

For water:

$$
R_M(t)=M_{H2O}(t)-M_{H2O}(0)-M_{in}(t)+M_{out}(t).
$$

For a total-energy convention:

$$
R_E(t)=E(t)-E(0)-E_{input}(t)+E_{output}(t).
$$

Define E using internal, kinetic, gravitational and relevant interface/solid energies consistently with the model. For isothermal constant-sigma benchmarks, sigma times area can represent capillary free energy. At varying sigma and temperature, use the selected interfacial thermodynamic energy convention; do not silently assume that expression is always total surface internal energy.

Boundary energy includes advected enthalpy and the pressure/mechanical-work convention of the chosen open-system balance. Do not compare summed enthalpy storage to a total internal-energy balance without the associated pressure-volume terms.

Normalise energy errors by a declared physical energy change or imposed heat budget, with an absolute floor defined in the benchmark. A large arbitrary enthalpy reference or a large ambient-air inventory must not hide error in a much smaller sample.

Every accepted step records aggregate residuals. Export full spatial snapshots at selected times and on failure. Record clipping, remeshing, smoothing and parcel conversion contributions. A correction that restores mass by distributing it arbitrarily is diagnostic evidence of error, not proof of conservation.

## 15 Rendering and lighting

Extract liquid/ice surfaces from the authoritative geometry. Water refraction and absorption should follow documented optical data at relevant wavelengths [O01]. Glass uses its selected product's index and transmission; ice may use O02. Reflection, exposure and lights must not alter the thermodynamic solution unless a radiative heating model explicitly couples their physical irradiance.

Provide overhead illumination, orbit camera, cross-section, transparent-glass mode, temperature overlay, velocity vectors, pressure overlay and phase display. Keep colour maps labelled with units and limits. False-colour temperature is an instrument view, not the physical visible colour of hot water.

Use a separate radiometric configuration if overhead lights are intended to heat the scene. A rendering light's brightness or emissive material parameter is not automatically a heat source in watts. Account for absorbed irradiance through the heat ledger only when that mode is enabled and validated.

## 16 Failure handling and completion

Detect and stop or invalidate runs with nonfinite values, negative mass, nonphysical phase fractions beyond roundoff, unsupported material states, unconverged pressure/thermal solves, unresolved contact penetration, unaccounted inventory correction or GPU resource failure. Preserve the last valid checkpoint and diagnostics.

The application may provide a clearly labelled interactive preview before every gate is complete. A validated status requires the corresponding benchmarks, source data and uncertainty evidence. A visually convincing beaker scene, comments stating completion, and an FPS number cannot establish the requested physics.

Completion requires all core requirements in the matrix, successful local execution on the user's declared GPU, reviewed code at a fixed Git SHA, and quantitative validation of the supported physical regime. Future source reviews are performed on supplied commits or pull requests; no continuous unattended monitoring is implied.
