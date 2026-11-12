# S09 V19 cavity thermal baseline

Run from the repository root:

```powershell
dotnet run --project Tools/ThreePhase3D/V19ThermalChecks.csproj
```

The CPU fixture uses the existing finite-volume conduction operator on the square cross-section of a cavity with uniform depth. The left and right walls are held at 300.5 K and 299.5 K, with insulated top, bottom, front and back. Wall flux uses the half-cell distance. Initial temperature is 299.75 K so the cumulative net wall input must account for a nonzero change in stored heat.

With the velocity held at zero, the exact steady temperature is linear in x and the mean Nusselt number at each side wall is 1. The 8, 16 and 32 cells per side checks passed with maximum temperature error below 9e-10 K and absolute energy residual below 7e-9 J. The three meshes check the conductive limit. A separate coupled flow run must establish grid and timestep convergence.

The first coupled target is the de Vahl Davis square air cavity at Ra = 1000 and Pr = 0.71, with no-slip walls and spanwise symmetry. Use the thermal-diffusion velocity scale (thermal diffusivity divided by cavity width). The published targets are mean Nusselt number 1.118, peak horizontal velocity 3.649 on the vertical centreline and peak vertical velocity 3.697 on the horizontal centreline. See [B03](Sources.md#b03-natural-convection-benchmark) and [V19](Validation.md). The GPU coupled steady cavity and the genuinely 3D convection case are still open.

The separate S09 heated-floor GPU regression passed in Unity 2022.3.46f1 on the user run. It supplied 10.000 J, retained 9.9426 J in glass and transferred 0.0574 J to water. The reported residual was 4.6584e-7 J, with a rising velocity of 1.5449e-6 m/s and a return velocity of -5.893e-7 m/s.
