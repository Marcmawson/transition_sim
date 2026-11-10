using System;
using System.IO;
using System.Linq;
using System.Runtime.Serialization;
using System.Text.RegularExpressions;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Diagnostics;

namespace PhysicsSim.ThreePhase3D.Verification
{
    [DataContract]
    public sealed class FoundationReference
    {
        [DataMember] public string schema_version;
        [DataMember] public string case_definition;
        [DataMember] public InventoryState initial_inventory;
        [DataMember] public double duration_s;
        [DataMember] public double transfer_fraction;
        [DataMember] public double absorbed_heater_J;
        [DataMember] public ConservationTolerance tolerance;
    }
    public sealed class FoundationRunResult
    {
        public string directory;
        public string gitCommit;
        public bool dirty;
        public CheckResult[] checks;
        public bool Passed => checks.All(check => check.passed);
    }

    public static class FoundationRun
    {
        public static FoundationRunResult Execute(string projectRoot, string outputRoot, string executablePath, string executableKind, RunHardware hardware, string unityVersion)
        {
            var beforeSources = ProvenanceCapture.CaptureSources(projectRoot);
            byte[] referenceBytes = File.ReadAllBytes(Path.Combine(projectRoot, "Tools/ThreePhase3D/Fixtures/S01-reference.json"));
            var reference = RecordJson.Decode<FoundationReference>(referenceBytes);
            if (reference.schema_version != "1.0" || reference.duration_s != 1 || reference.transfer_fraction != 0.125 || reference.absorbed_heater_J != 10 ||
                RecordJson.Hash(RecordJson.Encode(reference.initial_inventory)) != RecordJson.Hash(RecordJson.Encode(FoundationChecks.FixtureInitial())))
                throw new InvalidOperationException("The preregistered arithmetic fixture changed; revise the checks and evidence explicitly.");
            string scratch = Path.Combine(Path.GetTempPath(), "threephase3d-checks-" + Guid.NewGuid().ToString("N"));
            Directory.CreateDirectory(scratch);
            var checks = FoundationChecks.RunS00();
            foreach (var check in checks) check.metrics = new[] { FoundationChecks.Metric("assertion_failures", "1", check.passed ? 0 : 1, 0) };
            checks.AddRange(FoundationChecks.RunS01(projectRoot, scratch));
            var scenario = new ScenarioConfigSI { scenarioId = "S01-bookkeeping-no-field-solver" };
            scenario.physical.initialPhase = InitialPhase.Liquid;
            scenario.physical.sampleMassKg = reference.initial_inventory.WaterKg;
            scenario.physical.sampleTemperatureK = scenario.physical.ambientTemperatureK;
            scenario.physical.absorbedBurnerPowerW = reference.absorbed_heater_J / reference.duration_s;
            byte[] scenarioBytes = RecordJson.Encode(scenario);
            byte[] dirtyPatch;
            string meta = File.ReadAllText(Path.Combine(projectRoot, "Assets/Scenes/ThreePhase3D/Beaker3D.unity.meta"));
            string sceneGuid = Regex.Match(meta, "guid: ([0-9a-f]{32})").Groups[1].Value;
            var provenance = ProvenanceCapture.Capture(projectRoot, executablePath, executableKind, unityVersion, sceneGuid, scenarioBytes, out dirtyPatch);
            if (beforeSources.Length != provenance.source_files.Length || beforeSources.Where((file, i) => file.path != provenance.source_files[i].path || file.sha256 != provenance.source_files[i].sha256).Any())
                throw new InvalidOperationException("Source dependencies changed while the checks ran; repeat from a fixed source state.");
            var manifest = new RunManifest {
                run_id = "s01-foundation-" + DateTime.UtcNow.ToString("yyyyMMddTHHmmss") + "-" + Guid.NewGuid().ToString("N"),
                run_kind = "foundation-bookkeeping", provenance = provenance, hardware = hardware,
                materials = new MaterialRunIdentity { references = scenario.materials, generator_version = "NOT_USED_S01",
                    enthalpy_reference = "No material enthalpy evaluated; synthetic storage J from reference.json.", entropy_reference = "NOT_COMPUTED_S01", validity_domain = "Bookkeeping only; no material provider invoked." },
                physics = scenario.physical, numerics = scenario.numerics,
                execution = new RunExecution { requested_duration_s = reference.duration_s, diagnostic_interval_s = reference.duration_s },
                initial_inventory = reference.initial_inventory, conservation_tolerance = reference.tolerance,
                geometry_sha256 = RecordJson.Hash(RecordJson.Encode(scenario.physical.geometry)),
                reference_dataset_sha256 = RecordJson.Hash(referenceBytes), reference_definition = reference.case_definition,
                energy_convention = "E=fluid/ice internal+resolved kinetic+gravitational+fixed-solid internal+interface storage. Synthetic fixture J; no EOS or latent law. energy_in_J includes heater_absorbed_J exactly once.",
                boundary_conditions = "Closed mass inventory; prescribed 10 J external absorbed heat in the arithmetic fixture. Associated scene is the existing apparatus asset, not an executed fluid case.",
                timestep_policy = "Supplied fixture instants 0 and 1 s; no continuum timestep or field solve.", precision = "Compensated FP64 CPU diagnostic accumulation; no GPU field allocation.",
                seed_protocol = "Deterministic fixture. Seeded physics and stochastic models not active.",
                approximation_flags = new[] { "BOOKKEEPING_ONLY", "NO_3D_FLOW_OR_THERMAL_SOLVER", "ASSOCIATED_SCENE_NOT_PHYSICAL_RUN", "OPTIONAL_PHYSICAL_DIAGNOSTICS_UNAVAILABLE" },
                unresolved_scales = new[] { "No continuum fields, interface reconstruction, material tables, ice dynamics or parcels are executed." }
            };
            using (var recorder = new RunRecorder(outputRoot, manifest, scenarioBytes, referenceBytes, dirtyPatch))
            {
                recorder.Log("Git " + provenance.git_commit_sha + "; dirty=" + provenance.dirty_tree + "; " + executableKind);
                foreach (var check in checks) recorder.Log((check.passed ? "PASS " : "FAIL ") + check.validationId + " " + check.name + " " + check.detail);
                var initial = reference.initial_inventory.Copy();
                recorder.Record(new DiagnosticSample { timeS = 0, dtS = 0, inventory = initial });
                recorder.WriteSnapshot("initial.json", RecordJson.Encode(initial));
                var donor = new ConservedInventory("Eulerian arithmetic inventory", initial);
                var parcel = new ConservedInventory("parcel arithmetic inventory", new InventoryState());
                var packet = InventoryState.FromArray(initial.ToArray().Select(value => value * reference.transfer_fraction).ToArray());
                packet.dryAirKg = 0; // This synthetic water packet carries no noncondensable gas.
                var gain = packet.Copy(); gain.parcelWaterKg = gain.liquidKg; gain.liquidKg = 0;
                ConservedInventory.Transfer(donor, parcel, packet, gain, reference.tolerance);
                recorder.RecordTransfer("water-to-parcel", 0, 0, donor.Id, parcel.Id, packet, gain);
                ConservedInventory.Transfer(parcel, donor, gain, packet, reference.tolerance);
                recorder.RecordTransfer("parcel-deposition", 0, 0, parcel.Id, donor.Id, gain, packet);
                var current = InventoryState.Combine(new[] { donor.Snapshot, parcel.Snapshot });
                recorder.RecordExternal(0, reference.duration_s, ExternalChannel.HeaterAbsorbed, FluxDirection.IntoDomain, 0, 0, reference.absorbed_heater_J);
                current.internalEnergyJ += reference.absorbed_heater_J;
                var residual = recorder.Record(new DiagnosticSample { timeS = reference.duration_s, dtS = reference.duration_s, inventory = current });
                recorder.WriteSnapshot("final.json", RecordJson.Encode(current));
                var metrics = new[] { FoundationChecks.Metric("water_mass_residual_abs", "kg", Math.Abs(residual.waterKg), reference.tolerance.massKg),
                    FoundationChecks.Metric("dry_air_mass_residual_abs", "kg", Math.Abs(residual.dryAirKg), reference.tolerance.massKg),
                    FoundationChecks.Metric("total_energy_residual_abs", "J", Math.Abs(residual.energyJ), reference.tolerance.energyJ) };
                checks.Add(new CheckResult { validationId = "V02", name = "Exported inventory and external heat close independently", passed = residual.passed && metrics.All(metric => metric.passed), metrics = metrics,
                    detail = "Synthetic 1 kg water inventory; 10 J external absorbed heat; paired water-parcel/deposition transfers." });
                recorder.Complete(checks.ToArray(), new[] {
                    "S01 verifies bookkeeping and evidence infrastructure, not fluid motion or phase-change physics.",
                    "GPU field reductions/bindings are S03/V35; full representation coupling is S11/S14.",
                    "V36 new menu execution in Unity requires user-local confirmation; apparatus rendering was confirmed separately by the user.",
                    "Requirement review remains PENDING; passing this suite is not full beaker validation."
                });
                EvidenceValidator.VerifyBundle(recorder.DirectoryPath, projectRoot, checks.All(check => check.passed));
                return new FoundationRunResult { directory = recorder.DirectoryPath, gitCommit = provenance.git_commit_sha, dirty = provenance.dirty_tree, checks = checks.ToArray() };
            }
        }
    }
}
