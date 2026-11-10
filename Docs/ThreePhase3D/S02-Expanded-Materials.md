# S02 expanded material checkpoint

Source baseline: `4d571dc2f0e2b2649b7cd2a7fae63063260778a6`. S02 remains incomplete; the scene still has no coupled fluid/heat/phase evolution.

| Requested gap | This checkpoint | Remaining limit |
| --- | --- | --- |
| Steam-rich transport | Added a separately named low-density modified Vesovic-Wakeham research candidate, with modern calculated-reference observations | Full finite-density/contact and collision-integral treatment is not verified; production retains its Y<=0.10 or pure-vapour guard |
| Wider diffusion | Explicitly selectable Mason-Monchick/ASHRAE table provider, 203.15-473 K, 80-101.325 kPa, Y<=0.02 | Steam-rich composition dependence remains open; table interpolation is not an experimental validation |
| Ice conductivity through near melting | Explicit `Yen1981PurePolycrystalline` model, 200-273.15 K | A broad sourced regression, with preparation scatter and no supplied uncertainty bound; not a universal sample-specific property |
| Ice-contact equilibrium and G11 | Bracketed G8/ice-Gibbs equilibrium, supersaturation rejection, complete G11 official coefficient/fugacity checks and G8 cross-model comparisons | Near-melting liquid interval below 273.16 K still lacks the intended supercooled-liquid provider; interface kinetics are later work |
| Expanded GPU tables | Seven phase/composition-separated T,p surfaces and a new compute check command | Measured execution on a licensed Unity Editor is pending |

## Checks and evidence

The CPU runner passes **5,817 comparisons**, zero failures. **33 informational observations** remain separate from passing assertions. All **25 foundation check groups** and compilation of the ThreePhase3D C# sources against Unity **2022.3.46f1** assemblies pass.

[CPU report](../../ValidationRuns/s02-expanded-materials-20261110/comparisons.json), [CSV](../../ValidationRuns/s02-expanded-materials-20261110/comparisons.csv), [source fingerprints](../../ValidationRuns/s02-expanded-materials-20261110/source-hashes.txt), and [file seals](../../ValidationRuns/s02-expanded-materials-20261110/files.sha256) identify the development-run inputs. These are scoped development checks, not a clean reviewed acceptance run.

G11 checks reproduce all seven coefficients at 200/300/400 K from Table 7, both Table 8 compositions and three pressures, and beta/gamma check values. G8 full-Helmholtz fugacity is compared with G11's truncated virial formulation at four states using a declared 0.1% project cross-model limit. That limit is separate from numerical rounding and physical uncertainty. Ice-contact checks cover 200, 223.15, 250, 270 and 273 K at 80, 101.325 and 1000 kPa; the official 200 K / 1 Pa G8 reference is checked separately.

The original Marrero-Mason equimolar diffusion fit differs from ASHRAE's calculated reference by about +14.1% at 190 Celsius. It remains a research seam. The wider production option uses the published ASHRAE table directly, with explicit log-log interpolation and inverse-pressure scaling; the Massman default is unchanged. Numerical table-node reproduction does not independently validate the underlying calculated model.

The transport research candidate uses unit contact factors, zero bulk/critical terms and A*=B*=1.1. Its component order is explicitly water then air in the composition-dependent interaction length. Selected 60-90 Celsius saturated calculated comparisons differ by less than approximately 0.9%; a separate near-200 Celsius comparison is also recorded (the provider is limited to 473 K, versus 473.15 K in that reference table). These observations do not establish the omitted finite-density terms or a general uncertainty bound. The candidate is not called by the production provider.

## Expanded surface domains

All surfaces use 80-120 kPa with 2 kPa spacing. Pressure spacing was refined after gas-entropy interpolation exceeded the original 1e-4 relative target; tolerances were retained.

| Surface | Temperature | Composition / properties |
| --- | --- | --- |
| IceYen | 200-273 K, 0.5 K spacing | Ice thermodynamics plus explicitly selected Yen conductivity |
| Liquid | 274-365 K, 0.125 K spacing | Liquid water thermodynamics and transport |
| Vapour | 380-600 K, 0.5 K spacing | Water vapour thermodynamics and transport |
| GasDry, GasLean, GasRich, GasSteam | 380-473 K, 0.5 K spacing | Fixed water mass fraction 0, 0.01, 0.90, 1 respectively; thermodynamics only |

These are immutable bilinear T,p surfaces. Composition is checked exactly in FP32, with no interpolation between gas surfaces. Phase boundaries and unsupported gaps are not crossed. Metadata, units, shape, finite values and query bounds are guarded. Generated tables are retained with the CPU report.

## GPU execution required

The attempted batch Editor run stopped **before shader import**: the licensing IPC connection was refused, timed out after 60 seconds, and Unity reported exit code 199. [Exact startup log](../../ValidationRuns/s02-expanded-materials-20261110/editor-startup.log). No new GPU comparison passed or failed in that attempt.

After pulling main, open the project in the licensed Unity Editor and select:

**Physics Sim > Three Phase 3D > Run S02 expanded GPU material checks**

It checks all seven tables, including both axis endpoints, interior interpolation, bad temperature/pressure/composition and NaN rejection, and a partial final thread group. It binds all four buffers and releases them in `finally`. The resulting Console line must report `executed=True; passed=True`; the full result and exact tables are written under `ValidationRuns/local/s02-expanded-gpu-*`. Existing evidence for the earlier two water isobars does not validate this shader.

Sources and equation locators: [Sources.md](Sources.md), D07 and D17-D20. Run the existing PowerShell material and foundation commands from [S02-Handoff.md](S02-Handoff.md). Apparatus-specific glass and tripod data remain separate blockers for validated heating.
