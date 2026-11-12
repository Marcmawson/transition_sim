# S09 V19 cavity thermal baseline

Run from the repository root:

```powershell
dotnet run --project Tools/ThreePhase3D/V19ThermalChecks.csproj
```

The CPU fixture uses the existing finite-volume conduction operator on the square cross-section of a cavity with uniform depth. The left and right walls are held at 300.5 K and 299.5 K, with insulated top, bottom, front and back. Wall flux uses the half-cell distance. Initial temperature is 299.75 K so the cumulative net wall input must account for a nonzero change in stored heat.

With the velocity held at zero, the exact steady temperature is linear in x and the mean Nusselt number at each side wall is 1. The 8, 16 and 32 cells per side checks passed with maximum temperature error below 9e-10 K and absolute energy residual below 7e-9 J. The three meshes check the conductive limit. A separate coupled flow run must establish grid and timestep convergence.

The first coupled target is the de Vahl Davis square air cavity at Ra = 1000 and Pr = 0.71, with no-slip walls and spanwise symmetry. Use the thermal-diffusion velocity scale (thermal diffusivity divided by cavity width). The published targets are mean Nusselt number 1.118, peak horizontal velocity 3.649 on the vertical centreline and peak vertical velocity 3.697 on the horizontal centreline. See [B03](Sources.md#b03-natural-convection-benchmark) and [V19](Validation.md). The GPU square-cavity run awaits Unity execution. The genuinely 3D convection case remains open.

The separate S09 heated-floor GPU regression passed in Unity 2022.3.46f1 on the user run. It supplied 10.000 J, retained 9.9426 J in glass and transferred 0.0574 J to water. The reported residual was 4.6584e-7 J, with a rising velocity of 1.5449e-6 m/s and a return velocity of -5.893e-7 m/s.

## Coupled square cavity

In Unity, run **Physics Sim → Three Phase 3D → Run S09 V19 GPU cavity benchmark**. The benchmark uses one spanwise cell with symmetry, the existing GPU staggered momentum and pressure solver, and a GPU finite-volume temperature update with upwind advection and centred diffusion. Buoyancy uses the benchmark's constant Boussinesq coefficient. Temperature stays on the GPU between heat transport and momentum prediction. The existing flow solver still reads back velocity and pressure for its convergence checks.

The menu runs 8, 16 and 32 cells per side at timestep 0.08/N² in thermal-diffusion time units, then repeats the 16-cell case at half timestep. It reports mean wall Nusselt number, both positive centreline velocity peaks and hot/cold wall Nusselt imbalance. The run requires steady state on each mesh, a decreasing norm of relative reference errors, a maximum fine-grid relative error of 10% and a half-timestep change below 1% of each reference value. These are initial verification thresholds for the project's first-order upwind discretisation.

An earlier independent FP64 check used a cosine-transform pressure solve and a base timestep of 0.12/N². Its results were:

| Mesh | Timestep scale | Mean Nu | Centreline U max | Centreline V max |
| --- | ---: | ---: | ---: | ---: |
| 8² | 1 | 1.19686 | 3.47748 | 3.49452 |
| 16² | 1 | 1.14850 | 3.47126 | 3.54275 |
| 32² | 1 | 1.13088 | 3.55756 | 3.61429 |
| 16² | 0.5 | 1.14850 | 3.47127 | 3.54275 |

The GPU run must independently establish these checks in Unity.

## Three-dimensional cavity check

Run **Physics Sim → Three Phase 3D → Run S09 V19 3D GPU cavity check**. This case uses an 8 × 8 × 8 cube with no-slip walls on all six sides, fixed temperatures on the two x walls, and zero heat flux on the remaining walls. It uses the same Rayleigh and Prandtl numbers as the square case. The heat kernel exchanges flux through all three axes. The cube uses a fixed timestep of 0.055/N² to allow for the stricter no-slip diffusion and advection bounds.

The check requires a steady solution, hot/cold wall Nusselt numbers within 0.001 of each other, mean Nusselt number above the conductive limit of 1, and a nonzero interior spanwise speed above 0.001 in thermal-diffusion units. These checks exercise three-dimensional coupling. Quantitative published-reference and refinement checks apply to the separate square benchmark.

The runtime and Editor entry point compiled separately against Unity 2022.3.46f1 assemblies. GPU execution remains pending.

## GPU results

Unity reported a square-cavity pass with 2.51% maximum fine-grid reference error and a timestep change rounded to 0.00%. The 8 cubed cavity also passed with mean Nu 1.14270, peak spanwise speed 0.13605 and hot/cold wall Nu difference reported as zero. Both runs used the corrected fixed timesteps.
