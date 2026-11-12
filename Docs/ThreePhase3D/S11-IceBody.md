# S11 rigid ice body

Run `dotnet run --project Tools/ThreePhase3D/IceBodyChecks.csproj` from the repository root.

The constant-mass rigid state stores world linear momentum and angular momentum about the centre of mass. Its frame follows the principal axes. Angular velocity uses the inverse inertia tensor in the current orientation. The body supplies point velocity for a moving fluid boundary and accepts a force and torque or an impulse at a world point. Updates return a new state so a coupled solver can reject a trial before accepting it.

The CPU verification passed for a homogeneous rectangular body. It checks analytic mass and inertia, constant-force translation, principal-axis rotation under constant torque, off-centre impulse, point velocity, rotated inertia and unforced principal-axis motion over 1000 steps. The orientation integrator is explicit and tested here in principal-axis motion.

This supplies the rigid state for V24. The pressure-load verification below supplies the next piece. Coupled flotation remains open.

## Cut-cell pressure load

Run `dotnet run --project Tools/ThreePhase3D/IcePressureChecks.csproj`. The pressure load integrates fluid pressure on the cut ice surface and computes world force and torque about the centre of mass. A projection overload reads group pressures and accepts an optional hydrostatic background. The body update applies gravity once alongside this traction. Three-point triangle quadrature integrates affine pressure and its torque exactly on each planar polygon. Partially blocked solid-contact polygons require exposed geometry before they can be integrated.

The CPU test uses a rotated, fully submerged box on two grid offsets. It recovered 0.23544 N of hydrostatic lift and zero hydrostatic torque within the stated tolerances. A separate prescribed top-face pressure gradient produced -0.000128 N m of torque. The rigid-body step reproduced the analytic initial upward velocity under pressure and gravity. Pressure is prescribed in this fixture. The moving-boundary pressure iteration, fluid reaction, free-surface flotation and GPU execution remain open for V24.

## Moving-wall pressure projection

The pressure projection can now accept one outward boundary volume flux per pressure group. The ice supplies it by integrating rigid-body normal velocity over its cut-cell surface. Both projection backends include that flux in the pressure equation and divergence diagnostics. A closed pressure component rejects an incompatible net imposed flux.

The CPU fixture prescribes combined translation and rotation in the existing geometry and projects fluid initially at rest. At both grid offsets the pressure reaction opposes the prescribed motion. The body-side pressure power matches the sum of pressure times outward boundary flux to within 1e-10 W. The GPU solver C# integration compiled against Unity 2022.3.46f1. The next section verifies the coupled step on CPU. GPU execution and moving geometry remain open.

## Coupled pressure and body timestep

The fixed-geometry step iterates the ice momentum and the fluid projection using the same trial normal wall flux. Each candidate body momentum uses the original momentum plus the current pressure and gravity impulse. Under-relaxation updates the trial. The step accepts when the translational velocity mismatch and the angular mismatch scaled by the inertia length are both within 1e-8 m/s. It returns the accepted body and fluid states together and throws on nonconvergence.

The CPU fixture started a rotated submerged box and fluid at rest. On both grid offsets the solve converged in 25 iterations. The accepted upward speeds were 0.00596879 and 0.00602221 m/s, below the pressure-only rise speed of 0.00887939 m/s. Both momentum updates matched the final pressure and gravity impulses. The pressure projection backend is injected, and this verification used the CPU backend. Moving cut-cell geometry and conservative remapping between poses remain open, as does free-surface flotation.

## Conservative inventory transfer between poses

MovingCutCellRemap takes two cut-cell projection plans on the same grid and signed swept fluid volumes on geometric faces. It transfers liquid and gas volumes, phase energies and vapour from the original donor composition, then gathers retained and incoming inventory into the new pressure groups. It rejects incompatible volume closure, an overdrawn donor, exterior transfers and transfers with no fluid group. The operator uses first-order donor composition.

The CPU fixture rebuilds the geometry after translating a box 4 mm across one face. The exact swept transfer for that fixture is 0.054 mL. All five inventories and the new group volume closure pass. This verifies the remap given a known face transfer. The coupled transfer check follows below.

## Swept volumes from a coupled step

The reconciliation starts from the projected face velocity times old aperture and timestep. It computes each cell's residual against the exact old-to-new fluid capacity change. A weighted graph Laplacian distributes the correction across interior faces open at both poses. The solve rejects an unclosed residual or a face correction larger than the supplied budget. The scalar remap separately checks available donor volume and final cell closure.

The CPU fixture advances the submerged box using the coupled pressure step, rebuilds the geometry at its accepted pose, and uses the projected velocities to remap water and enthalpy. The check passed at both grid offsets, preserving uniform energy density and total water volume. The correction budget was 0.5% of one cell volume per face. The current path requires a closed domain and rejects projected flow through faces that close or lose a fluid endpoint. The staggered momentum transfer follows below. Free-surface flotation remains open.

## Staggered momentum across poses

The remap gathers face momentum into old pressure groups using the existing aperture-weighted phase masses. It transports all three components using the same donor-volume fractions as the phase inventory, then distributes them to active faces on the new pose using the new apertures and phase masses. This is a first-order group-average remap and can smooth within-group velocity variation. It rejects a nonzero component that has no active receiving face.

The coupled moving-box CPU test now also checks linear momentum in each direction to 1e-12 kg m/s at both grid offsets. The returned velocities are tentative for the new geometry. The next pressure projection must enforce its moving-wall constraint. The submerged continuation check below exercises that projection. Free-surface flotation and GPU runtime verification remain open.

## Successive coupled steps

The CPU fixture now advances three consecutive 10 ms steps at both grid offsets. After each geometry update it remaps volume, enthalpy and momentum, then uses the remapped velocities as the tentative input to the next moving-wall projection. Water volume and enthalpy stayed within 1e-15 m3 and 1e-13 J of the initial totals, respectively. The submerged body rose 0.257786 and 0.260333 mm at the two offsets. This fixture supplies a hydrostatic background pressure and checks submerged continuation. It does not exercise a free surface or equilibrium flotation.

## Unity GPU pressure verification

Run **Physics Sim > Three Phase 3D > Run S11 GPU submerged ice check**. This repeats the three-step submerged continuation at both grid offsets using the Unity compute shader for pressure projection. It reuses pressure buffers during each coupling solve and seeds each trial with the previous pressure. Geometry rebuilds, swept-volume reconciliation, and remapping run on the host in this fixture.

The run checks rise against the CPU result to within 1 micrometre and repeats the water and enthalpy conservation checks. The runtime and Editor entry point compiled separately against Unity 2022.3.46f1 assemblies. Device execution and free-surface flotation remain open.

## V24 hydrostatic waterline traction

The prescribed flat-waterline reference clips each exposed ice polygon against the submerged half-space and integrates gauge pressure over the clipped polygon. Air gauge pressure is zero in this fixture. The normal is determined by gravity in world space, including when the computational grid is tilted.

The CPU fixture passed at both grid offsets for dry, half-submerged, 91.7% submerged, and fully submerged states. The lift agreed with displaced water weight to 1e-10 N, and the symmetric box torque was within 1e-11 N m. At 91.7% immersion, the integrated pressure balanced the ice weight. The waterline is prescribed here. Advancing the liquid interface and testing transient flotation against this reference remain open.

## Initial coupled two-phase motion

The CPU fixture seeds liquid and gas by clipping the planar interface against the ice and the grid cells. It initializes group densities from the two phase inventories, then runs a coupled moving-wall pressure step from rest and remaps both phases into the new geometry.

The half-submerged box sank at 0.00747123 m/s after the first 2 ms step. Water, gas, both enthalpies and vapour inventories remained conserved to the test tolerance. The sum of absolute liquid volume differences against a freshly seeded flat surface at the new pose was 2.15171e-9 m3. This run uses first-order phase transport and the initial planar hydrostatic background for its one pressure step. The next section checks a pressure update from the remapped water inventory.

## Hydrostatic pressure from the transported inventory

For a gravity-aligned grid with one waterline per column, reconstruct that height by matching the column liquid volume against the current cut-cell geometry. The hydrostatic background uses water below the reconstructed surface and air above it, with zero gauge pressure at the domain top. Ice traction samples the column belonging to each fluid-side patch. The pressure projection also uses the updated phase-weighted density.

In the CPU fixture, the initial flat surface reproduced the analytic column pressure away from the ice. After the first remap, the largest sampled pressure change was 0.0560986 Pa. The second coupled step used the rebuilt pressure and reached a downward speed of 0.0143223 m/s. All five phase inventories passed conservation through the second remap. The reconstructed pressure drives ice traction in this fixture. The fluid face update follows below. Long-time flotation and GPU verification of the two-phase case remain open.

## Fluid velocity from column pressure

The predictor uses an analytic vertical gravity/pressure balance within each column. Horizontal faces receive the pressure jump between adjacent columns at the face height, multiplied by the same face mobility as the dynamic projection. The coupled solve projects these tentative velocities with the trial ice boundary flux.

The CPU fixture kept a flat surface at rest and reproduced the analytic downslope tentative face speed of -0.00195965 m/s for a 1 mm water-height difference across one face. The two-phase ice fixture now uses this predictor in its second step and still closes all five inventory ledgers. The column model assumes one waterline per column on a gravity-aligned closed grid. Equilibrium settling and GPU execution of this two-phase path remain open.

## Unity GPU two-phase verification

Run **Physics Sim > Three Phase 3D > Run S11 GPU two-phase ice check**. This uses the same two-step V24 fixture and conservation checks as the CPU runner. Unity dispatches the moving-wall pressure solve on the GPU and compares both ice speeds with the CPU reference to within 2e-6 m/s. Geometry, phase inventories, hydrostatic reconstruction and remapping run on the host.

The CPU fixture passed after extraction and the runtime and Editor code compiled separately against Unity 2022.3.46f1. Device execution and the longer flotation run remain pending.
