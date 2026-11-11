# S06 surface geometry

The first S06 implementation slice computes signed curvature from the compact periodic S05 volume-of-fluid field. Curvature is in inverse metres and is positive for a convex liquid interface. The graph curvature uses height functions following [M01](Sources.md#m01-balanced-capillary-vof), with the Cartesian cell spacing applied independently along each axis.

For a mixed cell, the estimator ranks axes by the local liquid-fraction gradient. A candidate axis uses nine columns of seven cells. Every column must transition monotonically from full liquid to full gas, within a fraction tolerance of 1e-12. An alternate axis is tried when the dominant-axis stencil is unsupported. Each result reports dominant-axis, alternate-axis, unresolved, or pure-phase support. The last two return NaN so a downstream force assembly can distinguish missing curvature from a valid flat interface.

The implementation applies to regular periodic cells. Embedded-wall geometry and the complete S06 V11–V13 and V39 gates remain open.

## Isothermal capillary pressure coupling

`PeriodicVofProjection.AdvanceCapillary` transports the accepted S05 fields, evaluates curvature on the moved interface, and adds `dt * mobility * sigma * kappaFace * (alphaHigh - alphaLow)` to the tentative velocity. The pressure solve subtracts `dt * mobility * (pHigh - pLow)` on the same staggered face. For constant exact curvature, `p = sigma * kappa * alpha + constant` balances the force at the stencil level. See [M01](Sources.md#m01-balanced-capillary-vof).

Face curvature uses the available adjacent estimate or their mean. If both adjacent estimates are missing, the solver averages finite estimates in the first supported neighbourhood around the face pair, up to three cells, and reports the number of such faces. If that search contains no supported estimate, the step fails. Fraction jumps at or below 1e-12 contribute zero normal capillary forcing and retain their transported inventory. The explicit timestep limit is `sqrt((rhoLiquid + rhoGas) * hMin^3 / (4 * pi * sigma))`. The pressure output is gauge pressure in compact periodic cell order.

The capillary API accepts a uniform surface tension in N/m. The CPU fixture uses an initially stationary 7 mm sphere at 1 mm grid spacing, with densities 997 and 1.2 kg/m³ and surface tension 0.072 N/m. The fixture checks the Young–Laplace pressure jump, maximum face speed, and liquid volume drift over three steps, plus parity with the S05 projection at zero surface tension. It covers one radius and resolution. Unity execution, GPU curvature, wall/contact-line coupling, and translating or oscillating droplets remain to be verified.

Run the CPU geometry checks with the .NET 8 SDK from the repository root.

```sh
dotnet run --project Tools/ThreePhase3D/SurfaceGeometryChecks.csproj
```

The checks exercise flat and quadratic surfaces on an anisotropic grid, reversed phase orientation, the alternate-axis fallback, and the unsupported-stencil diagnostic.

## Prescribed-temperature capillary and Marangoni forces

`PeriodicVofProjection.AdvanceThermocapillary` accepts a prescribed interface-temperature extension at the projection time, with one kelvin value per compact periodic cell. It evaluates the existing [D05](Sources.md#d05-water-surface-tension) IAPWS correlation over 273.16–647.096 K. Maximum surface tension sets the capillary timestep bound and the arithmetic face mean sets the normal capillary term.

The continuum tangential force density is approximated by `|grad(alpha)| * (I - n nᵀ) grad(sigma)`, where `n` is the normalised volume-fraction gradient. Both gradients use periodic centred differences. The cell force components are averaged to the staggered faces and multiplied by the same inverse face density used by pressure. The surface-force decomposition follows [Seric, Afkhami and Kondic (2018)](https://web.njit.edu/~kondic/thin_films/2018_JCP_surface_tension.pdf). The centred-gradient approximation used here requires a smooth extension of interface temperature through the stencil.

The CPU check uses a flat periodic slab with a sinusoidal temperature profile. It compares integrated tangential stress against the analytical derivative of the IAPWS correlation, checks zero tangential force for normal-only temperature variation, measures the net tangential force, verifies flow toward colder water, and checks uniform-temperature parity with the isothermal path. The check covers one prescribed-temperature case. Coupled heat transport, curved-interface migration, a general discrete net-force guarantee, wall wetting, and Unity/GPU validation remain open.

## Measured wetting and contact-line closure

`WettingProfile` records surface preparation, measurement provenance, equilibrium angle, a positive Navier slip length and the distance at which apparent angles were measured. Supply separate advancing and receding curves, each starting with its measured zero-speed limit and continuing with strictly increasing speed magnitudes. Angles are measured through the liquid. Positive wall-relative line speed selects the advancing curve and negative speed selects the receding curve. The law interpolates angle versus speed and rejects speeds outside the measured range. At zero speed it retains the apparent angle within the hysteresis interval and uses the nearest bound outside it.

A `CutCellVofTransport` caller can supply a contact sample for each active mixed donor cell. The sample identifies an exposed fixed-wall patch and supplies the signed line speed. The reconstruction retains the wall-parallel direction of the estimated liquid-to-gas normal and imposes the calibrated angle against the solid-to-fluid wall normal. The PLIC intercept is solved again for the original liquid inventory. The same normal reaches the CPU clipping code and the GPU clipping job builder.

The stationary-wall Navier closure evaluates tangential wall traction as `-mu * uTangential / (distance + slipLength)`. This is a two-point local approximation for wall momentum flux. The contact-line closure uses an empirical dynamic-angle curve at the declared matching distance, consistent with the need for a matching scale described in [M08](Sources.md#m08-dynamic-wetting). It does not use a Cox asymptotic fit to extrapolate beyond the data.

The CPU fixture uses synthetic calibration data. It checks both speed branches, zero-speed hysteresis, out-of-range rejection, normal orientation, Navier traction, and conservation during wetting-aware cut-cell transport. Actual glass calibration and the dynamic sessile-drop V13 gate remain open.

### Navier slip in the embedded-wall momentum predictor

The optional wall-wetting selector on `MomentumPredictor3D` supplies a calibrated profile for selected fixed-wall patches of the staggered dual cells. It is evaluated when the stencil is built. The selected tangential momentum components use the wall rate `nu * area / (dualVolume * (distance + slipLength))`. The same rate enters the CPU predictor, explicit timestep bound, and rate buffer consumed by the existing Unity compute kernel. Normal momentum retains its original wall treatment and the pressure topology continues to block flow through the solid. Patches outside the selected region retain their original wall drag.

For an oblique patch, the drag matrix per unit area and unit kinematic viscosity is `n nᵀ / distance + (I - n nᵀ) / (distance + slipLength)`. The two transverse components at each staggered face use the existing four-point MAC interpolation, including zero samples at inactive faces. The explicit timestep bound includes the absolute cross-component rates. This is a local centroid approximation. Patch normals and positive resolved dual-centroid distances are required, and the predictor still requires uniform fluid properties.

The checks cover an off-grid planar wall and a cylindrical section of the beaker geometry. The cylindrical tests check reduced axial plug damping and exercise cross-component coupling with a swirl. The two-phase contact-region path is described below.

### Automatic contact detection for cut-cell transport

Use `CutCellVofTransport.WithTrackedWetting(plan, profile)` to opt in to automatic detection for the calibrated fixed wall. For each mixed donor with wall geometry, the tracker intersects the unconstrained volume-matching PLIC plane with the clipped wall polygons. A polygon must have vertices strictly on both sides of the plane. If several qualify, the largest intersected patch determines the wall normal. Patches partially covered by another solid are excluded because the tracker has the original clipped polygon and the covered area only.

The tracker estimates signed line speed from the aperture-weighted cell velocity projected along the wall-parallel liquid-to-gas normal. This local advection proxy selects the advancing or receding calibration curve. It does not estimate geometric line displacement between timesteps. The synthetic flat-wall fixture checks an intersection, an interface separated from the wall, receding speed and agreement with explicitly prescribed wetting during transport.

Tracking currently supplies the angle constraint during donor reconstruction. The following section describes its use in the CPU wall momentum step.

### Two-phase beaker contact traction

`AdvanceCoupledWithWetting` reconstructs the contact region from the transported liquid inventory, samples remapped staggered velocity with the pressure-group aperture weights, and applies tangential Navier traction before pressure projection. The same weights scatter the impulse back to the staggered momentum and the wall ledger. Normal impermeability is handled by the existing pressure topology. The local dynamic viscosity is a liquid-volume-weighted phase blend and each intersected triangle patch contributes its exposed area. The explicit drag bound rejects a step whose summed patch weights exceed the affected face mass. Patches outside the detected band receive no viscous wall force in this isolated fixture.

Run the synthetic eight-step CPU beaker case with:

```sh
dotnet run --project Tools/ThreePhase3D/SurfaceGeometryChecks.csproj -- --wetting
```

The fixture checks that contacts and wall impulse occur, liquid moves, and the volume, staggered mass, and pre-projection momentum ledgers close. The calibrated glass data, bulk viscous stresses outside the contact band, Unity validation of the GPU contact traction step, and the moving sessile-drop validation remain open.

### GPU contact traction

The Unity command **Physics Sim → Three Phase 3D → Run S06 coupled wetting GPU checks** runs the same synthetic beaker through eight accepted steps. It requires GPU clipping and GPU traction dispatches on every step and records the detected patch count and conservation ledgers. The GPU evaluates the two aperture-weighted traction contributions to each staggered face from the local velocity and wall tensor prepared on the CPU. The host compares each FP32 face impulse with the FP64 reference, then applies the returned impulse to momentum and the wall ledger before GPU pressure projection.

The CPU still owns contact detection, wall geometry, coefficient assembly, and ledger checks. The Unity run uses the existing GPU clipping, scalar transport, momentum remap and pressure solver. A local SwiftShader Vulkan check executes the production traction shader with the other stages on the CPU. Unity Direct3D 11 verification covers the combined path.

### Synthetic sessile-cap tracking check

The CPU surface suite samples a prescribed 60° spherical cap above a flat wall at 1.5 mm and 1.0 mm grid spacing. It tests two positions separated by 1 mm and supplies a wall-relative translation speed of 0.01 m/s. The tracker must identify the advancing and receding sides, select the corresponding synthetic angle curve, and keep detected patch centroids within one cell width of the analytical contact circle. The maximum error must decrease under refinement. The centroid distance is a locator check for finite wall patches. The angle check tests the imposed reconstruction normal.

The two snapshots prescribe the motion. A free sessile-drop evolution benchmark, slip-length sensitivity, and comparison against separately measured glass observations are still required for V13. The actual glass preparation, temperature, angle and speed measurements with uncertainty, slip length, and matching distance are pending under [MAT028](Materials.csv).

The reported Unity run `s06-beaker-20261111T142027-2c19e7c3542a4fb59225a94ec8576e10` passed at [commit 5372635](https://github.com/Marcmawson/transition_sim/commit/5372635857edcfae9f9fd2babbad2d214542a892) with eight accepted steps, eight clipping dispatches, eight traction dispatches, and 184 contact patch occurrences.

### Evolving sessile cap and slip sensitivity

The CPU suite also seeds a closed four-face circulation across the footprint of a sampled spherical cap. After the initial projection, each timestep takes its liquid inventory and velocity from the preceding accepted solve. Both runs start from the same state and compare 50 µm and 500 µm slip lengths over three 1 ms steps. The fixture checks liquid redistribution, contact activation, volume and staggered-mass closure, pressure convergence, and reduced first-step wall impulse with the longer slip length.

This isolates the response of tracked contact traction to a flow perturbation. The cut-cell momentum step in this fixture includes advection, contact traction, and pressure projection. Wall-compatible capillary forcing and bulk viscosity are required to test free capillary relaxation and compare against a measured sessile-drop trajectory.

### Capillary pressure at a 90° planar wall

`CutCellVofTransport.WithPlanarWallCapillarity` enables a CPU capillary reference for a grid-aligned lower wall. At 90° the liquid fraction is reflected evenly across the wall. The existing height functions then estimate curvature in the reflected field, including at the contact line. Interior faces use the available adjacent curvature estimate or their mean, with a local average of supported estimates if both are missing. The capillary velocity increment and pressure correction share the same staggered face mobility. See [Afkhami and Bussmann (2009)](https://web.njit.edu/~shahriar/download/AB_IJNMF09.pdf) for height-function treatment of wall contact angles.

The configuration requires full fluid cells above the wall, a closed domain, and a three-cell gas buffer at the other boundaries. The CPU fixture advances an initially stationary 7 mm hemisphere for two 0.1 ms steps and checks pressure against the Young–Laplace jump, parasitic face speed, and liquid-volume drift.

This symmetry case supplies the first wall-compatible capillary pressure check. General contact angles, curved walls, bulk viscous relaxation, and the measured-glass V13 benchmark remain open.

### Prescribed planar-wall angles

The planar-wall capillary reference accepts a static contact angle through the liquid. At the wall row, a height graph with its axial direction parallel to the wall uses an extrapolated ghost height. The wall-centred slope is `-cot(theta) * sqrt(1 + H_transverse^2)`, so the graph's outward normal has wall-normal component `cos(theta)`. For angles other than 90°, wall-normal columns stop at the wall and require a pure endpoint across the stencil and unsupported estimates use the existing nearby-curvature fallback. At 90° the existing mirror path applies.

The CPU fixture compares initially stationary 75°, 90°, and 105° spherical caps with the same Young–Laplace jump. This is a prescribed-angle curvature and pressure check. Dynamic PLIC wetting and bulk viscous relaxation remain separate capabilities from this planar reference, and the measured-glass V13 comparison remains open.

For the angled cases the nearby-curvature search expands up to three cells if the first ring has no usable height function. The exploratory angled-cap check caps spurious speed at 0.5% of `sqrt(sigma / (rhoLiquid * radius))`. The 105° case exceeds the tighter 90° regression bound of 0.25 mm/s on its second step. Longer-time spurious-current control remains open.

### Dynamic wetting in planar capillary pressure

`CutCellVofTransport.WithPlanarWallWetting` combines tracked contact-angle reconstruction, contact traction, and planar-wall capillary forcing. Use `AdvanceCoupledWithWetting` with the two phase viscosities. After transport and contact traction, the pressure predictor redetects the contact region and evaluates the calibrated angle from the local wall-relative speed and apparent angle. Each wall height-function column uses its own tracked angle where available and the equilibrium angle elsewhere. The accepted step reports the number of wall cells with a tracked capillary angle.

The synthetic CPU fixture seeds a small circulation at the foot of a 90° sessile cap with a synthetic 89.5°–90.5° hysteresis band, then advances twice using the accepted velocity. It checks capillary and traction contact activation, pressure and volume, and compares the first step with the same wetting and traction calculation with surface tension disabled. This couples dynamic wetting and capillary forcing on the planar reference geometry. Bulk viscous relaxation, curved-wall capillary curvature, and measured-glass V13 validation remain open.

An exploratory 87°–93° hysteresis band exceeded the short-run spurious-speed limit on its first step at 1 mm resolution. The narrower-band fixture verifies the coupling. Wider hysteresis and longer runs still need a resolution study and improved spurious-current control.

### Symmetric bulk viscous stress in the planar reference

Enable `bulkViscosity: true` on `WithPlanarWallWetting` and advance with the two dynamic viscosities. After the momentum remap and contact traction, the planar reference applies the interior stress `mu * (grad(u) + grad(u)^T)`. Normal derivatives are centred in cells. Cross derivatives share an edge and use the harmonic mean of the four adjacent cell viscosities. Cell viscosities are liquid-volume-weighted blends. Each strain stencil scatters the stress through the negative transpose of its derivative, so the two faces of each component exchange equal and opposite impulse. An absolute row-sum bound checks the explicit timestep.

The CPU sessile-cap check compares the first step against the same capillary and wetting run with bulk stress disabled. It checks kinetic-energy dissipation, component momentum, wall impulse, pressure and liquid volume. The variable-viscosity rigid-rotation check exercises cancellation between the two cross derivatives. This reference assembles interior stencils on full cells. Stencils touching inactive boundary faces are omitted. Contact traction supplies the wall stress in the detected band. Boundary viscous stresses elsewhere, curved-wall bulk stress, and quantitative relaxation validation remain open.

### Planar wall traction outside the contact band

With `bulkViscosity: true`, the lower planar wall applies stationary no-slip tangential traction outside the detected contact band. Detected patches use the calibrated Navier slip length. Both use the same aperture-weighted velocity samples and transpose scatter, with a shared explicit wall-drag bound. The wall impulse ledger includes both contributions. The side and top domain boundaries are outside this checkpoint.

The planar plug fixture checks the integrated no-slip impulse against `-dt * mu * area * speed / (h/2)` with no liquid–gas contact. The sessile-cap fixture checks that the combined viscous momentum change equals the change in the wall ledger, and that interior stress still dissipates energy.

### Analytical planar shear relaxation

The standalone CPU suite compares the lower-wall traction and interior stress updates against the exact shear mode `u(y,t) = U sin(k y) exp(-nu k^2 t)`, with `k = pi / (2 H)`, no slip at the bottom and zero tangential stress at the top. The single-phase fixture holds density and viscosity constant and evolves the viscous substeps over 40 ms. It samples the central column beyond the stencil reach of the lateral boundaries. The refined run halves the wall-normal spacing from 1 mm to 0.5 mm and reduces the timestep by a factor of sixteen. Both runs must pass the analytical profile tolerance, and the fine-grid relative L2 error must be less than half the coarse-grid error.

This isolates viscous relaxation. Quantitative capillary-driven sessile-drop relaxation and comparison with measured glass data remain open.

A run with half the spacing and a quarter of the timestep gave relative L2 errors of 2.64e-5 and 3.06e-5. The tighter timestep in the accepted fine run addresses the separate wall and bulk updates. The paired refinement therefore checks combined spatial and temporal error.

### Capillary-driven sessile shape relaxation

The CPU suite initializes a 7 mm sessile hemisphere at rest, stretches one horizontal axis by 1.12, and compresses the other by the reciprocal factor. It advances the resulting half-ellipsoid for twelve 0.2 ms steps using dynamic wetting, wall traction, interior viscous stress, capillary forcing, and pressure projection. The shape metric is the difference between the horizontal second moments divided by their sum, measured about the initial centre. The check requires a decrease toward zero, conserved liquid volume, closed staggered mass, and converged pressure at every step.

The control uses the same initial shape and tracked contact traction with surface tension disabled. Its velocity stays at rest, so bulk and no-slip drag would also contribute zero. This checks the initial capillary restoring response. Oscillation frequency, damping over longer times, resolution dependence, and comparison with measured glass remain open.

The original 0.2 ms trial exceeded the transport pressure-flux correction bound and the 0.05 ms trial passed over 0.4 ms. The planar CPU pressure solver now checks the same reconciliation bound used by the next transport step before accepting a projected field. If the algebraic residual passes while the transport check fails, pressure iteration continues up to the existing iteration limit. The cutoff remains 1e-6 Courant units. The predicate uses the same flux-reconciliation routine as transport and leaves the candidate velocity unchanged. The regular CPU and GPU pressure entry points retain their existing convergence criteria.

### Sessile oscillation probe

Run the optional longer CPU probe with:

```sh
dotnet run -c Release --project Tools/ThreePhase3D/SurfaceGeometryChecks.csproj -- --capillary-oscillation
```

This probe starts a 5 mm sessile half-ellipsoid from rest, with reciprocal horizontal stretches of 1.05 and 1/1.05. It records the signed horizontal shape moment after each 0.6 ms step. A positive local peak after the first sign reversal gives the initial-to-return-peak period and the logarithmic amplitude change per second. The probe stops after two inviscid reference periods if it finds no return peak. It retains pressure, transport, staggered mass, contact, and liquid-volume checks.

The printed reference is the inviscid free-contact-line hemisphere's quadrupole period from the [Rayleigh–Lamb spectrum](https://www.cambridge.org/core/journals/journal-of-fluid-mechanics/article/effects-of-gravity-on-natural-oscillations-of-sessile-drops/9373ADEB7650B13ACA0BF3A057A9D071). The synthetic fixture includes viscosity, wall drag, slip and hysteresis. A frequency and damping comparison for V12 requires a matched reference and spatial and temporal refinement.

The first local probe reached its first shape sign reversal at 22.2 ms and exceeded the initial 0.02 m/s synthetic calibration range. With the probe's synthetic range extended to 0.1 m/s, the run reached 41.4 ms, where the last accepted shape moment was −0.25594 versus an initial +0.09394. The following step hit the three-cell gas-buffer guard before a positive return peak. The existing solver limits remain in force. These results leave frequency and damping unverified.

### V11 radius, grid, and translation checks

Run the standalone CPU matrix with:

```sh
dotnet run -c Release --project Tools/ThreePhase3D/SurfaceGeometryChecks.csproj -- --v11
```

The matrix uses 997 kg/m³ liquid, 1.2 kg/m³ gas, and 0.072 N/m surface tension. The static cases use a 7 mm drop at 1.4 and 1.0 mm spacing and a 6 mm drop at 1.0 mm spacing. The translating case uses the 7 mm drop at 1.0 mm spacing, with a uniform 0.5 m/s initial velocity. It advances for twenty 0.1 ms steps so its analytical displacement is one fine-grid cell. The runner checks pressure jump, maximum face speed relative to the prescribed frame, liquid volume, and relative L1 error against the sampled sphere at its expected position. It reports first and last speeds to expose growth. The short-run speed limit of 0.5% of `sqrt(sigma / (rhoLiquid * radius))` is exploratory. The periodic capillary path currently omits physical viscosity, so this matrix does not establish viscous decay.

| Case | Maximum pressure error | Maximum speed relative to frame | Final shape error |
| --- | ---: | ---: | ---: |
| 7 mm static, 1.4 mm cells | 1.998% | 0.0791 mm/s | <0.001% |
| 7 mm static, 1.0 mm cells | 0.672% | 0.323 mm/s | 0.001% |
| 6 mm static, 1.0 mm cells | 1.027% | 0.205 mm/s | 0.001% |
| 7 mm translating, 1.0 mm cells | 6.809% | 20.85 mm/s | 1.371% |

The three static cases pass the short-run limits, with speed increasing over the sampled interval. The translating case completes one cell of travel and conserves liquid volume to 1.23e-20 m³, but fails the pressure and speed limits. With surface tension disabled, the translating control preserves the prescribed velocity exactly and gives 1.408% shape error. That isolates the velocity error to the interaction of transported interface geometry and capillary forcing. V11 remains open for translating-drop accuracy and viscous decay.

### Local filtering of periodic curvature

The periodic capillary predictor now averages supported height-function curvature over a one-cell stencil before evaluating the face force. The weights are `alpha * (1 - alpha) / (1 + offsetSquared)` in grid-index coordinates. This favours nearby mixed cells and reduces the influence of nearly pure cells. The filter operates on curvature only, retains support diagnostics, and leaves the transported phase inventories and shared pressure mobility intact. Closely spaced interfaces within the stencil may influence one another. The planar wall path uses its existing curvature calculation.

With the same V11 limits, maximum translating speed error fell from 20.85 to 6.24 mm/s, pressure error changed from 6.809% to 6.767%, and shape error changed from 1.371% to 1.396%. The translating case still fails the speed and pressure limits. Static maximum speeds fell to 0.0179, 0.0474 and 0.0596 mm/s in the three cases, with pressure errors of 1.857%, 0.702% and 1.180%, respectively. Longer-run decay remains unverified.
