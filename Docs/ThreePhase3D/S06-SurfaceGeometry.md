# S06 surface geometry

The first S06 implementation slice computes signed curvature from the compact periodic S05 volume-of-fluid field. Curvature is in inverse metres and is positive for a convex liquid interface. The graph curvature uses height functions following [M01](Sources.md#m01-balanced-capillary-vof), with the Cartesian cell spacing applied independently along each axis.

For a mixed cell, the estimator ranks axes by the local liquid-fraction gradient. A candidate axis uses nine columns of seven cells. Every column must transition monotonically from full liquid to full gas, within a fraction tolerance of 1e-12. An alternate axis is tried when the dominant-axis stencil is unsupported. Each result reports dominant-axis, alternate-axis, unresolved, or pure-phase support. The last two return NaN so a downstream force assembly can distinguish missing curvature from a valid flat interface.

The implementation applies to regular periodic cells. Embedded-wall geometry and the complete S06 V11–V13 and V39 gates remain open.

## Isothermal capillary pressure coupling

`PeriodicVofProjection.AdvanceCapillary` transports the accepted S05 fields, evaluates curvature on the moved interface, and adds `dt * mobility * sigma * kappaFace * (alphaHigh - alphaLow)` to the tentative velocity. The pressure solve subtracts `dt * mobility * (pHigh - pLow)` on the same staggered face. For constant exact curvature, `p = sigma * kappa * alpha + constant` balances the force at the stencil level. See [M01](Sources.md#m01-balanced-capillary-vof).

Face curvature uses the available adjacent estimate or their mean. If both adjacent estimates are missing, the solver averages the finite estimates in a one-cell extension around the face pair and reports the number of such faces. If that neighbourhood contains no supported estimate, the step fails. The explicit timestep limit is `sqrt((rhoLiquid + rhoGas) * hMin^3 / (4 * pi * sigma))`. The pressure output is gauge pressure in compact periodic cell order.

The capillary API accepts a uniform surface tension in N/m. The CPU fixture uses an initially stationary 7 mm sphere at 1 mm grid spacing, with densities 997 and 1.2 kg/m³ and surface tension 0.072 N/m. The fixture checks the Young–Laplace pressure jump, maximum face speed, and liquid volume drift over three steps, plus parity with the S05 projection at zero surface tension. It covers one radius and resolution. Unity execution, GPU curvature, wall/contact-line coupling, and translating or oscillating droplets remain to be verified.

Run the CPU geometry checks with the .NET 8 SDK from the repository root.

```sh
dotnet run --project Tools/ThreePhase3D/SurfaceGeometryChecks.csproj
```

The checks exercise flat and quadratic surfaces on an anisotropic grid, reversed phase orientation, the alternate-axis fallback, and the unsupported-stencil diagnostic.
