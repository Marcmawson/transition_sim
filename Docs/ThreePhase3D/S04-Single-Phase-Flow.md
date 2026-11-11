# S04 single-phase flow checkpoint

11 November 2026. Grid baseline `d78f0a4a775337b0c62044278dbc13ac5a6446c2`.

S04 adds a three-dimensional MAC momentum predictor, variable-coefficient pressure projection and a GPU channel preview. The supported viscous cases fill a regular Cartesian domain with one fluid of uniform density and dynamic viscosity. The embedded-wall verification cases use stationary, inviscid flow with prescribed density.

## Discretisation

For an active face oriented from its low to high cell, the solver uses face velocity \(u_f\), open area \(A_f\), centre separation \(d_f\) and face density \(\rho_f\). Density is averaged arithmetically from the adjacent cells, with the interior density used at a domain boundary. Prescribed gauge pressure is applied a half cell from the first cell centre. Stationary walls close the normal face. The momentum stencil reflects tangential velocity at an aligned no-slip wall, copies it at a free-slip wall and extrapolates it with zero normal gradient at an open side.

The predictor applies world gravity in the local grid frame, first-order upwind self-advection and centred diffusion with \(\nu=\mu/\rho\). The projection uses

```text
u_corrected = u_predicted - dt (p_high - p_low) / (rho_face d_face)
group divergence = sum(outward signs × open face area × u_corrected) / group volume
```

S03 tiny cells join an adjacent group through a shared aperture. Projection uses the aggregate volume, one pressure per group and its external faces. Internal faces have no independent solved velocity in the grouped model. A clipping residue no larger than 64 double-precision machine epsilons times the full face area is treated as closed consistently in grouping and projection. The sum of this area is exposed as `RoundoffClosedAreaM2`. The aligned-wall regression measured \(8.67\times 10^{-18}\) m² of such residue. No cell volume is removed.

Each closed connected component has compatible flux because its walls are impermeable and every internal face contributes opposite signs to the two groups. Its pressure gauge is reported with a zero volume-weighted mean. Open components are referenced to their imposed boundary pressure. No phase-expansion source is applied in S04.

## Convergence and failure

The GPU predictor, weighted-Jacobi pressure iteration and velocity correction use separate buffers and bounded dispatches. The host reduces tentative volume flux in FP64 from the existing predictor readback and uploads one net flux per group. The pressure kernel accumulates pressure flux separately. This preserves small net fluxes when much larger incoming and outgoing velocities nearly cancel. The host measures the actual corrected divergence and recomputes the algebraic pressure residual with double accumulation. The algebraic residual is an L2 norm of integrated group residuals, in m³/s². The pressure gate is relative residual ≤ 1e-6 for nonzero RHS. The volume-constraint gate is `dt × max absolute divergence ≤ 1e-5`, including zero-RHS cases.

The timestep is bounded by the explicit advection, viscosity and acceleration rates and by the requested interval. A step returns a new field only after convergence. The iteration cap raises a diagnostic exception and leaves the caller's accepted velocity unchanged. The rollback test retries from that original velocity at a smaller timestep. The preview retries a rejected step up to three times with a halved timestep and displays the final residual, RHS and divergence on failure. This verification solver uses synchronous readback and starts each pressure trial from zero; it has no large-grid throughput claim.

## Verification

| Benchmark | Recorded result |
| --- | --- |
| V07 constant and stratified-density hydrostatics, rotated and translated grid, and disconnected cut-cell chambers | Maximum spurious velocity \(4.11\times10^{-8}\) m/s across CPU cases and \(3.36\times10^{-8}\) m/s across the software Vulkan cases. Wall leakage was zero in the checked cases. |
| V08 pressure-driven plane Poiseuille flow | Accelerated from rest under imposed pressure. The 16-cell wall-normal grid gave 0.535% relative L2 profile error. Halving the cell width reduced the error by a factor of about four. |
| V08 viscous mode decay | Kinetic energy decreased at every accepted step. Halving the timestep reduced amplitude error by a factor of about two against the semi-discrete exponential. |
| V35 pressure portion | The three kernels compiled to DXIL and SPIR-V. The SPIR-V ran through SwiftShader software Vulkan, including tail guards and field comparisons with the CPU reference. Unity import, device execution and profiling are pending. |

The CPU reference and standalone shader run each passed **89 comparisons**. See the [CPU log](../../ValidationRuns/s04-flow-20261111/cpu-results.txt), [software Vulkan log](../../ValidationRuns/s04-flow-20261111/software-vulkan-results.txt) and [run details](../../ValidationRuns/s04-flow-20261111/runner.txt). The benchmark implementation is in [FlowChecks.cs](../../Assets/Scripts/ThreePhase3D/Verification/FlowChecks.cs). It records analytical targets, pressure residual and measured divergence in each run.

## Run in Unity

Pull `main` and allow Unity 2022.3.46f1 to reimport. With Play mode stopped, use the **Physics Sim > Three Phase 3D** top menu.

1. Select **Run S04 GPU flow checks**. The Console should report `gpuExecuted=True; passed=True`. The command saves an evidence bundle under `ValidationRuns/local/s04-gpu-*`.
2. To see velocity develop from rest, select **Create S04 channel flow preview**, enter Play mode and open the **Game** tab. The opaque channel frame and cyan velocity arrows are rendered by the preview camera. The preview uses a synthetic pressure-driven channel with fixed density and viscosity. The Game view reports simulation time and the measured divergence.
3. **Run S04 CPU flow checks** runs the analytical suite without compute shader dispatches.

Standalone CPU command in PowerShell:

```powershell
.\Tools\ThreePhase3D\RunFlowChecks.ps1
```

The [embedded-wall viscosity follow-up](S04-Embedded-Wall-Viscosity.md) adds the stationary no-slip closure and records its verification. Phase transport, water–air free surfaces and the beaker dam-break case follow in later stages. No beaker scene is changed by this checkpoint.
