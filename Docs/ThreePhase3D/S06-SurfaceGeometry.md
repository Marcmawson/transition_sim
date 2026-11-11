# S06 surface geometry

The first S06 implementation slice computes signed curvature from the compact periodic S05 volume-of-fluid field. Curvature is in inverse metres and is positive for a convex liquid interface. The graph curvature uses height functions following [M01](Sources.md#m01-balanced-capillary-vof), with the Cartesian cell spacing applied independently along each axis.

For a mixed cell, the estimator ranks axes by the local liquid-fraction gradient. A candidate axis uses nine columns of seven cells. Every column must transition monotonically from full liquid to full gas, within a fraction tolerance of 1e-12. An alternate axis is tried when the dominant-axis stencil is unsupported. Each result reports dominant-axis, alternate-axis, unresolved, or pure-phase support. The last two return NaN so a downstream force assembly can distinguish missing curvature from a valid flat interface.

The implementation applies to regular periodic cells. Embedded-wall geometry, balanced pressure coupling, and the S06 V11–V13 and V39 gates are subsequent work.

Run the CPU geometry checks with the .NET 8 SDK from the repository root.

```sh
dotnet run --project Tools/ThreePhase3D/SurfaceGeometryChecks.csproj
```

The checks exercise flat and quadratic surfaces on an anisotropic grid, reversed phase orientation, the alternate-axis fallback, and the unsupported-stencil diagnostic.
