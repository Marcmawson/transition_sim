# S02 material reference checkpoint

Branch: `physics3d/s02-materials`, based on S01 `260f9a4a376b4eb81a3e188a944eb573eb75e1cd`.

**S02 is incomplete.** This checkpoint rebuilds the material layer after the previous unsaved work was lost. It supplies CPU references, bounded FP32 water tables and a GPU verification command. It does not advance fluid, thermal or phase geometry in the scene. Existing scenes, renderer settings, packages and legacy solvers are preserved.

## Implemented

- `Materials/MaterialContracts.cs`: semantic water, ice, humid-gas and solid contracts; immutable SI thermodynamic states. Composition means water mass fraction, not relative humidity. Absolute pressure is required.
- `Water95`: direct FP64 IAPWS-95 Helmholtz formulation, second derivatives through a small forward-differentiation value type, homogeneous reference checks, branch-specific density inversion and liquid/vapour coexistence from equal pressure and Gibbs energy.
- `IceIh`: complex Gibbs formulation and derivatives, pressure-dependent Ice Ih melting and sublimation functions. Ice and fluid use the unshifted IAPWS energy/entropy references. Ice conductivity requires explicit preparation-model selection.
- `WaterTransport`: IAPWS viscosity, conductivity (including enhancement computed from the EOS susceptibility) and surface tension/temperature derivative. Public transport calls have the narrower beaker bounds below. Official background-only checks are separate.
- `HumidGas`: G8-10 component Helmholtz energies plus cross-virial mixing, mass/mole conversion, density inversion, thermodynamic derivatives and a homogeneous water chemical-potential check seam. Ambient air is not assigned pure-steam density.
- Massman's water-in-air diffusion correlation, with units converted from cm2/s, explicitly limited to a dilute, near-STP application. Its reported uncertainty is approximately 7%; the numerical arithmetic check is not a new experimental validation.
- Corrected Bonales slow-frozen-ice conductivity, **only** for an explicitly selected preparation and -40 to -5 Celsius. The original paper interchanged the coefficients; the 2018 correction is used. This does not supply conductivity at the inherited -50 Celsius initial state or near melting.
- `WaterPropertyTable`: versioned FP32, 101325 Pa isobar tables, phase-separated, schema/units/reference/shape guards and CPU float interpolation. Liquid spacing is 0.125 K; vapour spacing is 0.25 K. The liquid spacing was refined after a measured density-derivative interpolation failure; tolerances were retained.
- `WaterPropertyLookup.compute` and `Editor/MaterialVerificationMenu`: isolated GPU interpolation/guard checks against the verified CPU oracle. All four kernel buffers are bound; scalar/float2 strides are 4/8 bytes. Test dispatch has 67 queries and a partial final thread group; all owned buffers are released in `finally`.

## Validity and exclusions

| Provider | Implemented public envelope | Additional restriction |
| --- | --- | --- |
| Water T,p | 273.16–600 K, 611.66 Pa–5 MPa | Requested branch must be stable relative to the computed saturation curve |
| Water saturation | 273.16–625 K | Two distinct stable branches; converged pressure/Gibbs residuals required |
| Homogeneous water reference seam | 273.15–1273 K, density 1e-12–1300 kg/m3 | Not a global stable-phase selector; unstable states and immediate critical singular region rejected |
| Water viscosity/conductivity | 273.16–600 K, 611.66 Pa–5 MPa | Requires a thermodynamic state; wider background routines exist for official checks only |
| Ice T,p | 200–273.16 K, 611.657 Pa–5 MPa | Below Ice Ih melting curve; broader reference seam is separately named |
| Bonales conductivity | 233.15–268.15 K | Explicit `BonalesSlowFrozenCorrected2018` selection; no extrapolation |
| Humid-gas T,p,Y | 273.16–473 K, 80–120 kPa, Y in [0,1] | Conservative gas-only margin: mole fraction times pressure at most 95% of pure-water saturation pressure; **not** a nonideal equilibrium criterion |
| Massman dilute-air diffusion | 273.16–373.15 K, 80–101.325 kPa, Y at most 0.02 | Composition and pressure bounds are conservative project restrictions, not new measured source limits; no steam-rich-plume claim |
| FP32 water tables | 101325 Pa exactly | Liquid 274–373 K, vapour 374–600 K; no pressure interpolation and no interpolation across phase gap |
| DURAN | Published single-state density and conductivity only | Heat capacity, enthalpy and entropy throw rather than borrowing another glass's data |

Homogeneous reference seams are for fixed named checks. They must not be used to bypass production validity checks in a coupled solver. The near-melting sub-273.16 K liquid branch and hot/saturated gas closure still need explicit support before the full experiment is enabled. Background transport routines do not claim that an omitted critical enhancement is zero at arbitrary states.

## Sources, equations and licences

Numerical formulation data are attributed to IAPWS. The implementation is original C# and uses only the Unity/.NET standard libraries; no third-party EOS software is vendored or linked. Generator identity: `S02-Water95-1`; table schema: `S02-ISOBAR-FP32-1`. Compiler/reference assemblies are pinned to Unity 2022.3.46f1. The repository licence is retained.

| Register | Exact formulation/check locator |
| --- | --- |
| D01 | [R6-95(2018)](https://iapws.org/technical-guidance/release/IAPWS-95.download), Eqs 4–6, Tables 1–3, numerical Tables 6–8. Table 6 checks the sum of the rounded ideal and residual columns; Table 7 includes 11 T,rho states; Table 8 includes 275, 450 and 625 K coexistence. |
| D02 | [R10-06(2009)](https://iapws.org/technical-guidance/release/Ice-2009.download), Eq 1, Tables 1–4 and numerical verification table. Triple-point fluid and ice references are coherent. |
| D03 | [R14-08(2011)](https://iapws.org/technical-guidance/release/MeltSub.download), Ice Ih Eq 1/Table 1 and sublimation Eq 6/Table 6; numerical melting/sublimation check points. |
| D04 | [R12-08](https://iapws.org/technical-guidance/release/viscosity.download), Eqs 10–22, coefficients and Table 4 background checks. |
| D05 | [R1-76(2014)](https://iapws.org/technical-guidance/release/Surf-H2O.download), sigma=0.2358 tau^1.256(1-0.625 tau) N/m. Water/air applicability remains a clean-interface assumption. |
| D06 | [G8-10](https://iapws.org/technical-guidance/release/SeaAir.download), Tables 5–11 and 13. Cross-virial functions T45/T55/T58/T61; mixture state and chemical-potential comparisons at all three rounded official inputs. |
| D08 | [Massman 1998](https://doi.org/10.1016/S1352-2310(97)00391-9), Eq 1, H2O discussion, Table 8: D0=0.2178 cm2/s at 273.15 K and 1 atm, exponent 1.81. Full primary text inspected via a hosted copy; no paper is redistributed. |
| D12 | [R15-11](https://iapws.org/technical-guidance/release/ThCond.download), Eqs 10,16–23 and Table 4. Public enhancement uses IAPWS-95 derivatives at reference T=1.5 Tc. |
| D15 | [Bonales 2017](https://doi.org/10.1080/10942912.2017.1306551), section on different freezing processes; [2018 corrigendum](https://doi.org/10.1080/10942912.2018.1482123): k=2.0526-0.0176 T_C W/(m K). Original slow-freezing measurement range and method are retained. Arithmetic coefficient checks do not independently validate the fit for a different preparation. |

## Verification and local commands

See [S02-Verification.md](S02-Verification.md) for the measured checkpoint results and exact artifact paths. CPU tests include official values, SI conversion, independently differenced thermodynamic derivatives, density anomaly, same-mass ice/liquid volume comparison, phase selection, nonfinite inputs, missing-data failures and table interpolation/guards. Same-mass volume comparison does not validate moving-ice buoyancy.

```powershell
powershell -ExecutionPolicy Bypass -File Tools/ThreePhase3D/RunMaterialChecks.ps1
powershell -ExecutionPolicy Bypass -File Tools/ThreePhase3D/RunChecks.ps1 -CompileUnitySources
```

In Unity: **Physics Sim > Three Phase 3D > Run S02 CPU material checks**, then **Run S02 GPU table checks**. Both write unique `ValidationRuns/local/` records. GPU checks perform diagnostic synchronous readback only; they are not a field solver. Run them on the actual graphics device with compute enabled. The CPU report always says `gpu_executed=false` and `complete_stage=false`; a separate GPU report is required.

The current evidence index demotes stale linked S01 run claims rather than weakening freshness checks. The original S01 index is preserved at `History/S01-ImplementationEvidence.json`, and its immutable run remains in `ValidationRuns/`. The existing validator is unchanged. All requirement acceptance/review remains pending.

## Remaining work before S02 acceptance

1. Add and independently verify humid-mixture viscosity/conductivity, including dry-air reference checks and a sourced mixing rule. Current calls throw.
2. Finish nonideal near-saturation interface equilibrium/fugacity and its guards; the gas-only safety margin is not a replacement. Extend diffusion beyond the restricted dilute, near-STP regime only with evidence.
3. Source an ice-conductivity model covering the chosen initial state through melting, with preparation and uncertainty recorded. The corrected Bonales option covers only its measured subrange.
4. Validate actual GPU execution and shader import. Extend water-table pressure/temperature coverage and add ice/gas tables with independent error checks; the first isobar tables are deliberately limited.
5. Complete derivative/transport-enhancement check coverage, coherent phase enthalpy-difference checks at the required interfaces and reviewed evidence mappings. DURAN and tripod apparatus data remain separately blocked.

Rollback: switch to `physics3d/s01-evidence`. This branch introduces no scene wiring or runtime water. S03 geometry/resource work can be designed independently, but S02 acceptance is not claimed.
