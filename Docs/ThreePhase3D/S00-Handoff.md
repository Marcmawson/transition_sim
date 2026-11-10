# S00 baseline and SI apparatus handoff

Status: draft implementation; development checks observed; published source requires local rerun and V36 import evidence.

## Baseline and recovery

- Specification commit: 332570fe7d09a6b66307f2ae58aa31fec48a58e6; tree 5567af7b203bfdd00e75d1afa83e0f1502946e05.
- Runtime parent/main: e51635ca16160a9afbf4570b2f9e3512fb831b8a.
- The original prototype checkout at 77e42af4c96d4bb1a94fbc0d570af1db86c90ed3, including unpublished 2D changes, was preserved.
- S00 was developed in an isolated worktree and locally committed as 7227493d355a48ec3e9450c39dd340aff28eb10d.
- The workspace disconnected before that commit could be pushed. This draft recovers the source from the recorded edits through GitHub. New asset metadata was reconstructed; the apparatus script reference GUID was preserved. It is not represented as the exact locally tested Git tree.
- Unity 2022.3.46f1 (8e9b8558c41a), built-in rendering, package versions and project settings remain unchanged. No repository AGENTS.md was found at the baseline.

## Scope and equations

See Architecture.md for ownership and limits. D00, BIPM SI Brochure 9th edition v4.01 (2026), section 2.3.1 defines the Celsius offset 273.15 K; sections 2.3.4 and 3 cover derived units and prefixes. Gauge pressure uses p_abs = p_reference + p_gauge. The mesh test compares closed glass volume with the analytic cylindrical annulus/base volume times N sin(2 pi/N)/(2 pi).

All runtime/editor/verification code and the scene are new ThreePhase3D paths. Legacy scenes and their metadata are unchanged. A 1000 kg/m3 density is explicitly synthetic test input, not an implemented water property.

## Development observations

Before the disconnection, six standalone V01 groups passed: conversions/invalid inputs; missing-data gate; incompatible/nonfinite configuration; supplied-density amount consistency; serialisation identity; preview mesh scale/winding/volume. C# compilation against installed Unity 2022.3 assemblies also passed. These observations concern the development tree; no clean immutable evidence bundle or accepted published-commit result is claimed. See S00-DevelopmentObservations.json.

The Editor import was blocked by licensing IPC connection refusal and terminated with code 199. Thus V36 scene rendering/import and a Console-clean baseline remain pending. The environment GPU identity could not be established; the preflight menu queries the actual device, approximate VRAM, graphics API/driver description, compute support and async readback support on the user's machine.

## Local execution

1. Fetch and check out physics3d/s00-baseline in the project clone.
2. Open Assets/Scenes/ThreePhase3D/Beaker3D.unity, or Physics Sim > Three Phase 3D > Open empty Beaker3D.
3. Inspect the empty beaker, tripod ring/legs and overhead light. Keep the apparatus root at identity transform.
4. Run Physics Sim > Three Phase 3D > Run foundation checks, then Export pre-run report. The Console prints the output location under ValidationRuns/local/preflight-<UTC>/.
5. Batch entry point: -batchmode -projectPath <project> -executeMethod PhysicsSim.ThreePhase3D.Editor.ThreePhase3DMenu.RunFoundationChecksBatch -logFile <log>.
6. Standalone PowerShell: powershell -File Tools/ThreePhase3D/RunChecks.ps1 -CompileUnitySources.

There is no water, heat or phase solver in S00. Burner input defaults to zero. Material warnings are readiness blockers. S01 conservation/evidence work was started locally but is not part of this draft and has no claimed test results.

Rollback by reverting this stage's commit or switching branches; no shared history rewrite is needed. S01 bookkeeping is the next implementation stage; physical solver acceptance remains gated.
