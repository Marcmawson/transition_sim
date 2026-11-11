# S05 geometric liquid–gas transport checkpoint

S05 adds conservative, three-dimensional geometric volume-of-fluid transport on a regular periodic grid. The verification cases prescribe a divergence-free velocity field and use fixed liquid and gas densities of 997 and 1.2 kg/m³. The staggered velocity carried as momentum is a distinct field so the verification can test pressure-free momentum transport under nonuniform, prescribed advection.

## Discrete transport

A central difference of the neighbouring volume fractions defines a plane normal in cell coordinates. Bisection positions the plane so its clipped volume matches the stored liquid fraction. Each face measures the liquid volume in its upwind swept slab by analytically integrating the piecewise quadratic plane-cut fraction. Gas takes the remaining swept volume. The direction of the shared face determines the sign of both transfers.

An unsplit geometric candidate can count an overlapping corner twice when several faces draw from one cell. The limiter starts from the feasible donor-cell transfer and scales positive and negative geometric corrections against the remaining liquid and gas inventories of that donor. This bounds total outgoing volume for each phase. The recipient receives the same signed transfer the donor loses. The limiter reports the absolute change it applied. Face and cell roundoff reconciliation, vapour roundoff and energy reconciliation are recorded separately. A phase-bound violation above the roundoff tolerance rejects the step.

Cell mass flux is the sum of the two phase-volume fluxes weighted by their densities. For a staggered component, average the two neighbouring cell mass fluxes onto each face of its momentum control volume. Its updated mass is the same average of the two updated cell masses. Momentum uses this mass transfer multiplied by the upwind staggered velocity. The benchmark measures the difference between independently updated staggered mass and the mass inferred from phase transport. Each phase's energy and the gas vapour inventory use the same phase transfers with donor concentrations.

The periodic GPU path reconstructs and sweeps in FP32. The donor limiter, shared phase transfers, scalar updates and staggered momentum store each scalar as a leading FP32 word plus a residual word. Compensated arithmetic preserves the small transfers, using the FP32 exponent range. The host validates input CFL and divergence, decodes GPU state and reduces the output evidence. Endpoint rounding during decoding is included in the vapour and energy reconciliation totals. This revision removes native FP64 shader operations after Unity Direct3D 11 rejected the earlier donor limiter. The periodic coupling uses S04's pressure iteration and velocity correction with one shared face at each periodic seam.

## Verification

| Case | Measured result |
| --- | --- |
| V09 sphere translated diagonally in 3D | Relative L1 fraction error was 9.15% at 12³ and 6.46% at 24³ against an 8³-sample-per-cell reference. |
| V09 periodic deformation and reversal | Relative L1 return error was 4.16% on a 16³ grid. |
| V10 high density contrast | Density ratio 831:1. The tests check liquid volume, total mixture mass, vapour, both phase energies, dual mass closure and all three momentum components. A nonuniform momentum case also checks a hand-calculated face update. |

The CPU and software Vulkan results are in [the recorded verification bundle](../../ValidationRuns/s05-transport-20261111). The DXC-compiled PLIC kernels were dispatched through SwiftShader, with the same CPU transport updates used by the Unity GPU entry point. Unity import, device execution and the Game-view preview remain to be checked.

In Unity, stop Play mode and use the **Physics Sim > Three Phase 3D** top menu:

1. **Run S05 GPU geometry and transport checks**. Look for `gpuExecuted=True; cutCellGpuExecuted=True; passed=True` in the Console. The evidence is saved under `ValidationRuns/local/s05-gpu-*`.
2. **Create S05 droplet transport preview**, press Play and open **Game**. The image is an XY slice through the periodic 3D volume, with liquid volume and drift above it.
3. **Run S05 CPU transport checks** runs the reference path.

## Cut-cell transport and pressure coupling

The periodic benchmark transports phase and staggered momentum, rebuilds the face pressure coefficients from the new phase density, and projects the advected velocity using the S04 solver. A shared-face FP64 correction removes the residual divergence before the next volume-of-fluid step. The maximum correction is recorded in Courant units and the trial fails above 1e-6.

The original Unity Direct3D 11 preview reported an invalid `Sweep` kernel. The revised shader reconstructs and sweeps in one `SweepPlic` dispatch. It removes the intermediate plane buffer and uses explicit axis arguments. Direct3D 11 retesting is required. The revised checkpoint passed 61 checks on each of the CPU and software Vulkan paths. [Revised verification logs](../../ValidationRuns/s05-periodic-pressure-20261111)

The cut-cell CPU path clips each PLIC half-space and swept slab against the original solid triangles. It finds the swept width from accessible volume, then limits each group’s outgoing liquid and gas transfers. Closed faces require zero normal flux. Open inflows have explicit reservoir phase, energy, vapour and velocity values. The open-boundary ledger records net inward transfers.

For cut-cell momentum, each pressure group distributes its mass to active staggered faces by aperture for each component. The same phase mass transfers remove the donor face contributions and distribute their momentum to the recipient. Momentum entering a group without a degree of freedom for a component contributes to a reported wall impulse. This first-order remap mixes donor contributions. The coupled path rebuilds the S04 pressure coefficients after transport and projects the advected velocity. Before the next transport step, bounded shared-face flux reconciliation removes residual divergence from the pressure result.

The expanded suite passed 84 checks on the managed reference and on software Vulkan with full periodic GPU transport. Cut-cell transport runs on the CPU in both suites. Tests cover an open cut channel with a reservoir, closed circulation, and one conservative step through a curved cut of the tessellated beaker. [Expanded verification logs](../../ValidationRuns/s05-gpu-cut-transport-20261111)

### GPU cut-cell scalar update

The GPU cut-cell path consumes clipped swept-volume candidates and the existing pressure-group incidence lists. It limits the liquid transfer, forms the complementary gas transfer, and transports phase energy and vapour through the same faces. A second kernel gathers those signed transfers into each pressure group. The host computes clipping, assembles donor budgets, records boundary totals and reconciles phase-volume roundoff. GPU and CPU comparisons cover all five inventories, both phase face fluxes and the net boundary ledger on the open channel, closed circulation and curved beaker cases.

## Open integration

The compensated GPU path passed 84 checks on both lavapipe and SwiftShader with shaderFloat64 disabled. The user reported a passing Unity Direct3D 11 run for the periodic checkpoint at commit `9562ce4`. The new GPU cut-cell scalar kernels still need a Unity Direct3D 11 run. [Compensated GPU verification](../../ValidationRuns/s05-compensated-gpu-20261111) Cut-cell PLIC clipping, donor-budget assembly and staggered momentum updates remain on the CPU. The curved beaker test uses prescribed circulation. The beaker scene still needs a coupled time integration and a grid that resolves its embedded viscous wall distances.

Method references are [M04 and M13](Sources.md). The capacity limiter and dual-volume averaging above are the discretisation used by this implementation.
