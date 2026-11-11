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

## Prescribed-temperature capillary and Marangoni forces

`PeriodicVofProjection.AdvanceThermocapillary` accepts a prescribed interface-temperature extension at the projection time, with one kelvin value per compact periodic cell. It evaluates the existing [D05](Sources.md#d05-water-surface-tension) IAPWS correlation over 273.16–647.096 K. Maximum surface tension sets the capillary timestep bound and the arithmetic face mean sets the normal capillary term.

The continuum tangential force density is approximated by `|grad(alpha)| * (I - n nᵀ) grad(sigma)`, where `n` is the normalised volume-fraction gradient. Both gradients use periodic centred differences. The cell force components are averaged to the staggered faces and multiplied by the same inverse face density used by pressure. The surface-force decomposition follows [Seric, Afkhami and Kondic (2018)](https://web.njit.edu/~kondic/thin_films/2018_JCP_surface_tension.pdf). The centred-gradient approximation used here requires a smooth extension of interface temperature through the stencil.

The CPU check uses a flat periodic slab with a sinusoidal temperature profile. It compares integrated tangential stress against the analytical derivative of the IAPWS correlation, checks zero tangential force for normal-only temperature variation, measures the net tangential force, verifies flow toward colder water, and checks uniform-temperature parity with the isothermal path. The check covers one prescribed-temperature case. Coupled heat transport, curved-interface migration, a general discrete net-force guarantee, wall wetting, and Unity/GPU validation remain open.

## Measured wetting and contact-line closure

`WettingProfile` records surface preparation, measurement provenance, equilibrium angle, a positive Navier slip length and the distance at which apparent angles were measured. Supply separate advancing and receding curves, each starting with its measured zero-speed limit and continuing with strictly increasing speed magnitudes. Angles are measured through the liquid. Positive wall-relative line speed selects the advancing curve and negative speed selects the receding curve. The law interpolates angle versus speed and rejects speeds outside the measured range. At zero speed it retains the apparent angle within the hysteresis interval and uses the nearest bound outside it.

A `CutCellVofTransport` caller can supply a contact sample for each active mixed donor cell. The sample identifies an exposed fixed-wall patch and supplies the signed line speed. The reconstruction retains the wall-parallel direction of the estimated liquid-to-gas normal and imposes the calibrated angle against the solid-to-fluid wall normal. The PLIC intercept is solved again for the original liquid inventory. The same normal reaches the CPU clipping code and the GPU clipping job builder.

The stationary-wall Navier closure evaluates tangential wall traction as `-mu * uTangential / (distance + slipLength)`. This is a two-point local approximation for wall momentum flux. The contact-line closure uses an empirical dynamic-angle curve at the declared matching distance, consistent with the need for a matching scale described in [M08](Sources.md#m08-dynamic-wetting). It does not use a Cox asymptotic fit to extrapolate beyond the data.

The CPU fixture uses synthetic calibration data. It checks both speed branches, zero-speed hysteresis, out-of-range rejection, normal orientation, Navier traction, and conservation during wetting-aware cut-cell transport. Actual glass calibration, automatic contact-line tracking, and the dynamic sessile-drop V13 gate remain open.

### Navier slip in the embedded-wall momentum predictor

The optional wall-wetting selector on `MomentumPredictor3D` supplies a calibrated profile for selected fixed-wall patches of the staggered dual cells. It is evaluated when the stencil is built. The selected tangential momentum components use the wall rate `nu * area / (dualVolume * (distance + slipLength))`. The same rate enters the CPU predictor, explicit timestep bound, and rate buffer consumed by the existing Unity compute kernel. Normal momentum retains its original wall treatment and the pressure topology continues to block flow through the solid. Patches outside the selected region retain their original wall drag.

For an oblique patch, the drag matrix per unit area and unit kinematic viscosity is `n nᵀ / distance + (I - n nᵀ) / (distance + slipLength)`. The two transverse components at each staggered face use the existing four-point MAC interpolation, including zero samples at inactive faces. The explicit timestep bound includes the absolute cross-component rates. This is a local centroid approximation. Patch normals and positive resolved dual-centroid distances are required, and the predictor still requires uniform fluid properties.

The checks cover an off-grid planar wall and a cylindrical section of the beaker geometry. The cylindrical tests check reduced axial plug damping and exercise cross-component coupling with a swirl. Automatic contact-line selection, fully coupled two-phase slip in the beaker, and the sessile-drop benchmark remain open.
