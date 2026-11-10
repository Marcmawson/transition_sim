# S02 humid-gas continuation — 10 November 2026

**S02 remains incomplete.** This continuation extends the material library; it does not enable a moving-fluid scene. Source baseline: `82f9329d58cd6c744b216817a2861c9bbb1f7a85`.

## Implemented and checked

- G8-10 liquid-contact equilibrium solves equal water chemical potentials, with bracketed composition and density inversion. The 300 K / 100 kPa and 400 K / 1 MPa official reference states are reproduced. Ice-contact equilibrium remains open.
- Dry-air viscosity and conductivity implement Lemmon–Jacobsen equations and reproduce all six Table V states, including thermal critical enhancement, to the printed rounding precision.
- Humid-mixture transport combines modern component properties with the Melling viscosity curvature and Mason–Saxena conductivity shape. This is an explicitly documented project adaptation, including a low-temperature extension; it is not a new experimentally validated correlation.
- Production mixture transport is restricted to 273.16–473 K, 80–120 kPa and water mass fraction Y<=0.10, plus the pure-vapour endpoint. Richer mixtures raise an explicit error. Thermodynamic evaluation has its separate, wider envelope in [S02-Handoff.md](S02-Handoff.md).

The runner passes **2,662 numerical comparisons**, with zero failures. All **25 foundation check groups** pass, and all ThreePhase3D C# sources compile against Unity **2022.3.46f1** reference assemblies. This compilation is not an Editor import or a new GPU execution.

The report also records **18 informational model observations** against ASHRAE 2025 saturated-air calculated values. These observations are not counted as passing numerical tests. At 90 Celsius, the wider research approximation differs by approximately +8.53% in viscosity and +11.83% in conductivity. The production composition restriction therefore remains material. Conductivity shape checks against the fitted 80 Celsius dataset are regression checks, not independent validation.

Evidence: [CPU comparisons](../../ValidationRuns/s02-humid-continuation-20261110/comparisons.json), [CSV](../../ValidationRuns/s02-humid-continuation-20261110/comparisons.csv), [source fingerprints](../../ValidationRuns/s02-humid-continuation-20261110/source-hashes.txt), and [file hashes](../../ValidationRuns/s02-humid-continuation-20261110/files.sha256). The archived run was generated in the development checkout; exact source fingerprints identify its inputs.

## User GPU execution

The user executed the existing water lookup checks in Unity at 2026-11-10T23:07:08 UTC: **executed=true, passed=true, 1,742 comparisons, zero failures**, on an NVIDIA GeForce RTX 5060 Laptop GPU using Direct3D 11 and Unity 2022.3.46f1.

[Original GPU result](../../ValidationRuns/s02-gpu-user-20261110T230708/gpu-results.json) and both exact input tables are archived with [provenance observations](../../ValidationRuns/s02-gpu-user-20261110T230708/provenance-observation.json). The exact shader source is retained: its raw hash differs from the workspace file solely because of CRLF versus LF line endings; canonical LF hashes match. Table hashes match the recorded dependencies exactly.

This evidence covers the existing liquid/vapour isobar lookup and guard checks. It does not validate the new humid-gas routines on the GPU or a coupled fluid, heat or phase solver.

## Subsequent checkpoint

See [S02-Expanded-Materials.md](S02-Expanded-Materials.md) for the later ice/G11/diffusion/table additions and current remaining limits. The results above describe this earlier checkpoint.

## Remaining S02 work at this checkpoint

- Steam-rich humid-mixture transport beyond the bounded production provider.
- Wider diffusion coverage beyond the existing dilute, near-STP Massman application.
- Sourced ice conductivity outside the selected Bonales slow-frozen preparation and its -40 to -5 Celsius range, including near melting.
- Ice-contact humid equilibrium and the planned G11 cross-check.
- Expanded material GPU tables and corresponding measured GPU evidence.

## Reproduce

Run from the repository root in PowerShell:

```powershell
powershell -ExecutionPolicy Bypass -File Tools/ThreePhase3D/RunMaterialChecks.ps1
powershell -ExecutionPolicy Bypass -File Tools/ThreePhase3D/RunChecks.ps1 -CompileUnitySources
```

Equation locators, primary source links and adaptation limits are recorded in [Sources.md](Sources.md), entries D06, D13, D16 and D17. Material status remains partial with review pending; these checks do not establish completion of S02.
