# S05 geometric liquid–gas transport checkpoint

S05 adds conservative, three-dimensional geometric volume-of-fluid transport on a regular periodic grid. The verification cases prescribe a divergence-free velocity field and use fixed liquid and gas densities of 997 and 1.2 kg/m³. The staggered velocity carried as momentum is a distinct field so the verification can test pressure-free momentum transport under nonuniform, prescribed advection.

## Discrete transport

A central difference of the neighbouring volume fractions defines a plane normal in cell coordinates. Bisection positions the plane so its clipped volume matches the stored liquid fraction. Each face measures the liquid volume in its upwind swept slab by analytically integrating the piecewise quadratic plane-cut fraction. Gas takes the remaining swept volume. The direction of the shared face determines the sign of both transfers.

An unsplit geometric candidate can count an overlapping corner twice when several faces draw from one cell. The limiter starts from the feasible donor-cell transfer and scales positive and negative geometric corrections against the remaining liquid and gas inventories of that donor. This bounds total outgoing volume for each phase. The recipient receives the same signed transfer the donor loses. The limiter reports the absolute change it applied. Face and cell roundoff reconciliation, vapour roundoff and energy reconciliation are recorded separately. A phase-bound violation above the roundoff tolerance rejects the step.

Cell mass flux is the sum of the two phase-volume fluxes weighted by their densities. For a staggered component, average the two neighbouring cell mass fluxes onto each face of its momentum control volume. Its updated mass is the same average of the two updated cell masses. Momentum uses this mass transfer multiplied by the upwind staggered velocity. The benchmark measures the difference between independently updated staggered mass and the mass inferred from phase transport. Each phase's energy and the gas vapour inventory use the same phase transfers with donor concentrations.

The geometric reconstruction kernels can run in FP32 on the GPU. The conservative limiter, scalar and momentum updates and evidence reductions currently run on the CPU in FP64. This is a hybrid verification path. It shares the S03 grid definition. The periodic coupling uses S04's pressure iteration and velocity correction with one shared face at each periodic seam.

## Verification

| Case | Measured result |
| --- | --- |
| V09 sphere translated diagonally in 3D | Relative L1 fraction error was 9.15% at 12³ and 6.46% at 24³ against an 8³-sample-per-cell reference. |
| V09 periodic deformation and reversal | Relative L1 return error was 4.16% on a 16³ grid. |
| V10 high density contrast | Density ratio 831:1. The tests check liquid volume, total mixture mass, vapour, both phase energies, dual mass closure and all three momentum components. A nonuniform momentum case also checks a hand-calculated face update. |

The CPU and software Vulkan results are in [the recorded verification bundle](../../ValidationRuns/s05-transport-20261111). The DXC-compiled PLIC kernels were dispatched through SwiftShader, with the same CPU transport updates used by the Unity GPU entry point. Unity import, device execution and the Game-view preview remain to be checked.

In Unity, stop Play mode and use the **Physics Sim > Three Phase 3D** top menu:

1. **Run S05 GPU geometry and transport checks**. Look for `gpuExecuted=True; passed=True` in the Console. The evidence is saved under `ValidationRuns/local/s05-gpu-*`.
2. **Create S05 droplet transport preview**, press Play and open **Game**. The image is an XY slice through the periodic 3D volume, with liquid volume and drift above it.
3. **Run S05 CPU transport checks** runs the reference path.

## Open integration

The periodic benchmark transports phase and staggered momentum, rebuilds the face pressure coefficients from the new phase density, and projects the advected velocity using the S04 solver. A shared-face FP64 correction removes the residual divergence before the next volume-of-fluid step. The maximum correction is recorded in Courant units and the trial fails above 1e-6.

The original Unity Direct3D 11 preview reported an invalid `Sweep` kernel. The revised shader reconstructs and sweeps in one `SweepPlic` dispatch. It removes the intermediate plane buffer and uses explicit axis arguments. Direct3D 11 retesting is required. The revised checkpoint passed 61 checks on each of the CPU and software Vulkan paths. [Revised verification logs](../../ValidationRuns/s05-periodic-pressure-20261111)

The beaker needs interface geometry clipped against solid cut cells and transport across closed/open boundaries. Moving the limiter, conservative state updates and momentum transport to the GPU remains open. This checkpoint establishes the periodic verification cases. The existing beaker scene does not run this transport.

Method references are [M04 and M13](Sources.md). The capacity limiter and dual-volume averaging above are the discretisation used by this implementation.
