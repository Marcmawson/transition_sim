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

The CPU fixture advances the submerged box using the coupled pressure step, rebuilds the geometry at its accepted pose, and uses the projected velocities to remap water and enthalpy. The check passed at both grid offsets, preserving uniform energy density and total water volume. The correction budget was 0.5% of one cell volume per face. The current path requires a closed domain. A face that closes can carry its old-pose projected transfer if an old donor and a new receiver exist. Corrections use faces open at both poses. The staggered momentum transfer follows below. Free-surface flotation remains open.

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

## Extended two-phase continuation

The CPU fixture advances 256 consecutive 2 ms steps from 50% submerged and checks all five phase inventories after each remap. The final displaced volume is measured by intersecting the ice with the reconstructed liquid columns. The earlier 64-step run reached 95.0972% immersion at 0.128 s. The extended run reached 92.3779% at 0.512 s and passed the conservation checks. The final vertical speed was +5.78106e-5 m/s, while the peak absolute vertical speed over the last 32 steps was 0.00435034 m/s. The small final speed alone does not establish settling.

The longer run exercised faces that close during a pose change. The remap retains their first-order projected transfers when the donor exists in the old geometry and the receiver exists in the new geometry. It still checks donor capacity, volume closure and the correction budget. This is a transient result. Settling to the equilibrium displaced volume and GPU execution of the two-phase check remain open.

## Late-time flotation window

The 512-step CPU runs initially exceeded the phase-volume closure tolerance after repeated remaps. Tightening the swept-volume residual from 1e-11 to 1e-13 of one cell volume let both grids complete 1.024 s with all five inventory checks passing.

| X grid offset | Final immersion | Final 32-step immersion range | Peak vertical speed | Peak angular speed |
| ---: | ---: | ---: | ---: | ---: |
| 0 mm | 92.0701% | 91.8123% to 92.0701% | 0.4488 mm/s | 0.7878 rad/s |
| 3 mm | 91.6418% | 91.6418% to 91.7382% | 0.1220 mm/s | 0.2210 rad/s |

The hydrostatic reference is approximately 91.7% submerged. The two offsets still differ and the body continues rotating. V24 settled flotation remains open. The Unity GPU check still covers the short two-step fixture.

## Waterline torque quadrature

Column pressure has a slope break at the waterline. Integrating the air contribution over the whole patch and the excess water pressure over only the clipped wet polygon removes the quadrature error at that break. Pressure from the dynamic projection continues to act over the whole exposed patch.

The CPU torque fixture rolls the nearly equilibrated box by -0.01, 0 and +0.01 rad on both grid offsets. The restoring torque is about 4.51659e-7 N m in magnitude at either tilt, consistent with the [small-angle metacentric reference](https://web.mit.edu/13.400/www/problemset2sol.pdf). The previous unsplit sampling produced a vector torque error of 6.63122e-7 N m in the first case. The split integration agrees with the exact flat-waterline integral within 3e-19 N m across all six cases.

Both 512-step CPU reruns conserved all five inventories. The unshifted and 3 mm shifted runs ended at 92.0511% and 91.6315% immersion, respectively. Peak angular speeds in the final 32 steps were 0.7828 and 0.2594 rad/s. The correction resolves the static quadrature error. The dynamic rotation and offset dependence still require investigation before accepting settled flotation.

## V25 fixed-wall point impact

IceWallContact resolves a detected single-point impact against a fixed, frictionless wall. The impulse uses the inverse mass and world-space inverse inertia at the contact point. It applies the specified normal restitution and returns the equal and opposite wall reaction. Separating contacts receive no impulse. The model follows the [rigid-body collision impulse reference](https://www.cs.cmu.edu/~baraff/sigcourse/notesd2.pdf).

The CPU rigid-body checks passed for central and off-centre impacts at restitution 0, 0.5 and 1. They compared normal rebound speed, unchanged tangential translation, wall reaction, angular impulse and kinetic energy loss against analytic values. Continuous collision detection, simultaneous and resting contacts, and the coupled wall nonpenetration check remain open for V25. V24 settled flotation is tracked separately above.

## V25 continuous translation to a wall

The plane sweep finds the first contact time from the hull's closest vertex and normal speed. It advances the rigid body only to that time and returns the contact point for impulse resolution. The sweep assumes constant translation velocity and orientation. Rotating sweeps, simultaneous contacts and resting support remain open.

The CPU fixture translated a tilted box at 20 m/s toward the plane during a 20 ms step. It detected contact after 5.012981 ms, matching the analytic support distance divided by normal speed. No vertex penetrated at the returned pose. The contact point rebounded at 10 m/s with restitution 0.5. The fixture also verified an interval ending before impact and an approaching contact at the start of a step.

## V25 principal-axis rotating sweep

The rotating sweep handles constant translation and torque-free principal-axis spin against a fixed plane. It advances by the current clearance divided by a conservative normal vertex-speed bound. This prevents a clear endpoint from hiding an impact within the interval. It returns first contact within a 1e-10 m distance tolerance or rejects a sweep that cannot reach a safe result.

The CPU fixture spins a box at 10 rad/s through a full revolution with both endpoint hulls clear. It detected first contact at 20 ms, agreeing with the analytic time to within 1 ns, and all hull vertices remained on the admissible side within the distance tolerance. A 10 ms sweep stayed clear. General tumbling, simultaneous and resting contacts, and coupled beaker-wall collision remain open.

## V25 simultaneous wall impact

The frictionless fixed-wall solver resolves a set of contact points at one impact pose. It holds restitution targets from the incident point speeds, iterates nonnegative normal impulses, and accepts only a converged contact velocity solution without kinetic-energy gain. It returns the total equal and opposite reaction on the wall.

The CPU fixture hit the wall squarely at 0.4 m/s and resolved all four corners at once. At restitution 0, 0.5 and 1, all corners matched the analytic rebound speed, the body stayed spin-free within 1e-8 rad/s, and tangential translation, net impulse and energy loss matched the reference. The fixture uses both forward and reverse corner traversal. Sweep-to-manifold integration, sustained support and beaker-wall coupling remain open.

## V25 sweep-to-contact integration

The combined step advances to first contact, gathers the hull vertices within the contact tolerance of the first point, then resolves their normal impulses together. It returns the updated body, equal and opposite wall reaction, and time consumed up to impact. The caller owns the remainder of the timestep.

The CPU fixture drove a level box into a wall at 20 m/s. It detected contact after 5 ms of a 20 ms interval. All four corners rebounded at 10 m/s with restitution 0.5, with no hull penetration within 1e-12 m and no spin above 1e-8 rad/s. Reversing the hull vertex order gave the same result within tolerance. A 2 ms clear sweep produced no impulse. Sustained support under gravity and coupled beaker-wall contact remain open.

## V25 resting support under gravity

The supported step starts with a touching hull. It applies the external force and torque impulse, solves simultaneous zero-restitution normal contact, then advances the pose with the constrained momentum. The fixed wall applies no tangential impulse. An inward drift beyond the wall tolerance rejects the step.

The CPU fixture held a level box under a 0.21589848 N weight for 1000 consecutive 1 ms steps. The wall reaction matched that weight, and the hull stayed within 1e-10 m of the support plane without penetration beyond tolerance. A second run preserved a 0.1 m/s tangential slide and its kinetic energy. A lift-off test returned zero wall reaction under an outward load. Integration with the coupled fluid step and the curved beaker wall remains open.

## V25 resting contact in the pressure solve

The supported pressure step applies the plane constraint to each pressure and gravity correction. The next projection uses the corrected rigid velocity on the original cut-cell geometry. The accepted result reports the equal and opposite wall impulse alongside the body and projected fluid. Cut-cell geometry records roundoff-sized corrections to fully covered interface patches in its existing area ledger.

The CPU fixture rests an ice box on a fixed slab in a closed cut-cell grid. With gravity and 25 Pa applied on the fluid-exposed surface, the downward wall impulse was 5.124329e-5 N s over 1 ms, matching the weight and pressure load. The projected fluid remained still and the boundary velocity mismatch was 2.2676e-14 m/s. Pressure on genuinely partially covered patches, curved beaker contact and Unity GPU verification remain open.

## V25 pressure and flux at partial wall coverage

The cut-cell builder retains the signed clipped pieces of a partially covered ice patch. Pressure integration subtracts their force and moment from the full patch. The moving boundary uses the same covered pieces to subtract their rigid-body flux. Hydrostatic integration clips both terms at the waterline before applying the pressure quadrature. The force, torque and flux integrals are exact for affine pressure and rigid velocity on the clipped polygons up to clipping and arithmetic tolerances.

The CPU fixture places a slab edge across an ice face. The exposed strip produced 0.002688 N of force and 1.6272e-5 N m of torque under affine pressure. Translation and spin gave a net fluid boundary flux of -2.688e-6 m3/s. All three matched analytic integrals. The full resting-floor and both waterline fixtures also passed. Curved beaker contact and Unity GPU verification remain open.

## V25 first impact inside the beaker

The translation sweep uses the glass floor and the inner side planes at the same angular resolution as the polygonal beaker mesh. At the first hit it collects touching hull vertices across all active planes and resolves their normal impulses together. It returns the impact pose, elapsed time and total wall reaction. This step assumes a non-spinning hull below the rim and leaves the rest of the timestep to the caller.

The CPU fixture used a 24-facet beaker and a 0.4 m/s side approach, both with and without a simultaneous 0.4 m/s floor approach. Both impacts arrived at 5 ms. At restitution 0.5, the normal velocities rebounded at 0.2 m/s, the wall impulse balanced the ice momentum change, and all vertices stayed inside the vessel within 1e-12 m. The shorter clear sweep and existing plane impact and support checks passed. Rotating beaker sweeps, coupled pressure and impact stepping, and Unity GPU verification remain open.

## V25 principal-axis spin inside the beaker

The beaker sweep now uses the conservative rotating plane sweep for the floor and each side facet, then resolves the earliest contact manifold. A centre-height and hull-radius bound keeps the full rotating arc below the rim. The supported motion is constant translation with torque-free principal-axis spin.

In separate CPU fixtures a box spinning at 10 rad/s hit the floor and a side facet after 20 ms, within 1 ns of the analytic contact times. Both sweeps had clear start and end poses over a full revolution. The wall reactions balanced the linear impulse, and the translational beaker and resting-support checks passed again. General tumbling, coupled pressure and impact stepping, and Unity GPU verification remain open.

## V25 torque-free tumbling inside the beaker

For angular momentum off the principal axes, the sweep advances the existing midpoint rigid-body integrator in substeps of at most 0.002 radians of bounded rotation. All beaker planes use the same trajectory. The gap to the nearest plane and the point-speed bound, using angular momentum magnitude divided by minimum principal inertia, limit each advance so the numerical trajectory stays on the admissible side up to the contact tolerance.

An asymmetric box with initial angular velocity (3, 4, 10) rad/s hit the floor at 13.950243 ms, within 1 microsecond of a separate 1-microsecond stepping run. All vertices remained above the floor within 1e-10 m and the wall reaction balanced the change in ice momentum. A shorter clear sweep preserved world angular momentum and kept relative kinetic-energy drift below 1e-7. Coupled pressure and impact stepping and Unity GPU verification remain open.

## V25 pressure step up to first beaker impact

The fixed-geometry solve applies the converged pressure and gravity impulse to the incident ice momentum, then sweeps with that momentum. If contact shortens the step, the solver repeats projection from the same fluid state at the revised duration. It accepts impact once the projected duration matches the sweep time within 1e-10 s. It returns the projected fluid, rebounding body, elapsed time and wall reaction. The caller owns the geometry remap and any remaining time. Existing contact requires a separate contact step.

With a 10 ms request and initially downward-moving ice in a closed cut-cell air domain above a glass slab, the CPU projection and impact agreed at 2.916205 ms. The wall reaction was -1.156161e-4 N s vertically and balanced the momentum change from the pressure- and gravity-corrected incident state. Touching points rebounded and the hull stayed above the floor within tolerance. A 1 ms clear step returned zero wall impulse. Unity GPU verification remains open.

The same fixture is available in Unity under **Physics Sim → Three Phase 3D → Run S11 GPU beaker impact check**. It runs the production GPU pressure projection and compares contact time and wall impulse against the CPU baseline within 1 microsecond and 1e-7 N s respectively. The CPU fixture passed and the changed scripts compiled against the Unity 2022.3.46f1 assemblies. Batch mode exited before execution because the licensing client connection timed out. GPU results are pending.
