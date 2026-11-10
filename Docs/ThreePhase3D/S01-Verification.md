# S01 recorded verification

Run: s01-foundation-20261110T211825-32982064feaa4af7a17f24c80d079760

Tested clean source: **4f1c3a019672d786f22b47ea389ccf382ab9ef72**.

Evidence directory: [recorded run](../../ValidationRuns/s01-foundation-20261110T211825-32982064feaa4af7a17f24c80d079760/manifest.json).

Manifest SHA-256: `d55b06b8d2ddc0ef36636b21177ad7cc7b4de2ae73317e5bf7a71d1be5972570`.

## Observed results

| Check | Observation | Declared limit |
| --- | --- | --- |
| Standalone suite | 25 check groups passed; 0 failed | All declared assertions must pass |
| Exported closed water inventory | Absolute residual 0 kg | 1e-12 kg |
| Exported dry-air inventory | Absolute residual 0 kg | 1e-12 kg |
| Exported total storage energy after 10 J input | Absolute residual 0 J | 1e-9 J |
| Compensated sum versus independent decimal oracle | Absolute error 3.61932706027801e-14 | 1e-12 |
| Detecting an unrecorded 0.001 kg loss | Residual measurement error 8.67361737988404e-19 kg | 1e-12 kg |
| Deliberate unmatched mass/energy/momentum transfers | Rejected before either owner was committed | Every tested mismatch must fail |
| Lost mass/energy, fabricated passing CSV flags, corrupt/stale evidence | Rejected by the independent audit/evidence gate | Every injected failure must fail |
| New C# compilation | Passed against installed Unity 2022.3.46f1 assemblies | No compile errors |

The measured zero residuals apply to this particular synthetic arithmetic fixture. They are not a claim of exact conservation or physical accuracy for the future 3D solver. See the summary.json check metrics and source for the individual assertions.

## Scope and remaining checks

This is numerical verification of the **S01 bookkeeping/evidence seam**. No continuum field, material EOS, GPU reduction, physical phase interface, body dynamics or parcel dynamics was executed. Temperature, pressure, entropy, solver residuals and GPU timings are unavailable and remain blank in the CSV.

The actual standalone host did not query a graphics device; manifest fields explicitly record that. The Unity menu will record the user's graphics reports when run locally. Menu invocation/import in Unity, a player build, target-device GPU checks and independent review remain pending. The user separately reported that the S00 beaker renders after its quaternion correction.

The following evidence commit adds this report, links and immutable output bytes only. Numerical source, scene, configuration/reference files and build script are unchanged from the tested commit. The bundle validator checks those dependency identities against the current tree; this is a scoped source-identity comparison, not automatic acceptance of all requirements.

Function comments and the requirement matrix retain pending-review statuses. Later stages must supply their own physical and convergence evidence.
