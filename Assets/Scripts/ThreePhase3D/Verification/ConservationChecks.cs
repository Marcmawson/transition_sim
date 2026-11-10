using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.Serialization;
using System.Text;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Diagnostics;

namespace PhysicsSim.ThreePhase3D.Verification
{
    public static partial class FoundationChecks
    {
        public static List<CheckResult> RunS01(string projectRoot, string scratchRoot)
        {
            var checks = new List<CheckResult>();
            Case(checks, "Compensated reduction retains cancellation-sensitive terms", "V02", () =>
            {
                double sum = StableReduction.Sum(new[] { 1e16, 1.0, -1e16 });
                Near(sum, 1, 0);
                var terms = new List<double> { 1e12 };
                decimal oracle = 0;
                for (int i = 0; i < 10000; i++) { terms.Add(0.0001); oracle += 0.0001m; }
                terms.Add(-1e12);
                double error = Math.Abs(StableReduction.Sum(terms) - (double)oracle);
                Reject(() => StableReduction.Sum(new[] { double.NaN }));
                return new[] { Metric("cancellation_sum_error", "1", Math.Abs(sum - 1), 0), Metric("decimal_oracle_error", "1", error, 1e-12) };
            });
            Case(checks, "Body to Eulerian to parcel to deposition preserves inventory", "V34", () =>
            {
                var initial = FixtureInitial(); initial.iceKg = initial.liquidKg; initial.liquidKg = 0; initial.dryAirKg = 0;
                var body = new ConservedInventory("body", initial);
                var eulerian = new ConservedInventory("Eulerian", new InventoryState());
                var parcel = new ConservedInventory("parcel", new InventoryState());
                var loss = Portion(initial, 0.125); var melt = loss.Copy(); melt.liquidKg = melt.iceKg; melt.iceKg = 0;
                var tolerance = new ConservationTolerance();
                ConservedInventory.Transfer(body, eulerian, loss, melt, tolerance);
                var spray = melt.Copy(); spray.parcelWaterKg = spray.liquidKg; spray.liquidKg = 0;
                ConservedInventory.Transfer(eulerian, parcel, melt, spray, tolerance);
                ConservedInventory.Transfer(parcel, eulerian, spray, melt, tolerance);
                var combined = InventoryState.Combine(new[] { body.Snapshot, eulerian.Snapshot, parcel.Snapshot });
                Require(tolerance.Matches(initial, combined), "Representation exchange failed.");
                return new[] { Metric("water_mass_residual_abs", "kg", Math.Abs(combined.WaterKg - initial.WaterKg), tolerance.massKg),
                    Metric("total_storage_energy_residual_abs", "J", Math.Abs(combined.TotalEnergyJ - initial.TotalEnergyJ), tolerance.energyJ),
                    Metric("angular_momentum_x_residual_abs", "kg m2/s", Math.Abs(combined.angularMomentumXKgM2PerS - initial.angularMomentumXKgM2PerS), tolerance.angularMomentumKgM2PerS) };
            });
            Case(checks, "Repeated small transfers retain total mass and storage energy", "V02", () =>
            {
                var initial = FixtureInitial(); var donor = new ConservedInventory("liquid", initial); var receiver = new ConservedInventory("parcel", new InventoryState());
                var loss = Portion(initial, 1e-6); var gain = loss.Copy(); gain.parcelWaterKg = gain.liquidKg; gain.liquidKg = 0;
                for (int i = 0; i < 1000; i++) ConservedInventory.Transfer(donor, receiver, loss, gain, new ConservationTolerance());
                var current = InventoryState.Combine(new[] { donor.Snapshot, receiver.Snapshot });
                var residual = new ConservationLedger(initial, new ConservationTolerance()).Audit(current);
                Require(residual.passed, "Small transfers drifted outside tolerance.");
                return new[] { Metric("water_mass_residual_abs", "kg", Math.Abs(residual.waterKg), 1e-12), Metric("energy_residual_abs", "J", Math.Abs(residual.energyJ), 1e-9) };
            });
            Case(checks, "Unmatched mass energy linear and angular momentum are rejected atomically", "V02", () =>
            {
                int detected = 0;
                foreach (int channel in new[] { 0, 5, 10, 13 })
                {
                    var initial = FixtureInitial(); var donor = new ConservedInventory("donor", initial); var receiver = new ConservedInventory("receiver", new InventoryState());
                    var loss = Portion(initial, 0.125); var gain = loss.Copy();
                    double[] values = gain.ToArray(); values[channel == 0 ? 1 : channel] += 0.001; gain = InventoryState.FromArray(values);
                    Reject(() => ConservedInventory.Transfer(donor, receiver, loss, gain, new ConservationTolerance())); detected++;
                    Require(RecordJson.Hash(RecordJson.Encode(donor.Snapshot)) == RecordJson.Hash(RecordJson.Encode(initial)), "Rejected transfer changed donor.");
                    Near(receiver.Snapshot.WaterKg, 0, 0);
                }
                return new[] { Metric("missed_unmatched_transfer_types", "1", 4 - detected, 0) };
            });
            Case(checks, "Overdraw and nonfinite inventories are rejected", "V02", () =>
            {
                var initial = FixtureInitial(); var donor = new ConservedInventory("donor", initial); var receiver = new ConservedInventory("receiver", new InventoryState());
                var overdraw = Portion(initial, 2);
                Reject(() => ConservedInventory.Transfer(donor, receiver, overdraw, overdraw.Copy(), new ConservationTolerance()));
                Near(donor.Snapshot.WaterKg, initial.WaterKg, 0); Near(receiver.Snapshot.WaterKg, 0, 0);
                var bad = initial.Copy(); bad.iceKg = -1;
                Reject(() => new ConservedInventory("negative", bad));
                bad = initial.Copy(); bad.kineticEnergyJ = double.PositiveInfinity;
                Reject(() => new ConservedInventory("infinite", bad));
                return PassMetric();
            });
            Case(checks, "Independent audit detects unrecorded mass and heat loss", "V02", () =>
            {
                var initial = FixtureInitial(); var ledger = new ConservationLedger(initial, new ConservationTolerance());
                var massLoss = initial.Copy(); massLoss.liquidKg -= 0.001;
                var energyLoss = initial.Copy(); energyLoss.internalEnergyJ -= 1;
                Require(!ledger.Audit(massLoss).passed && !ledger.Audit(energyLoss).passed, "An unmatched loss was accepted.");
                return new[] { Metric("mass_loss_detection_error", "kg", Math.Abs(ledger.Audit(massLoss).waterKg + 0.001), 1e-12),
                    Metric("energy_loss_detection_error", "J", Math.Abs(ledger.Audit(energyLoss).energyJ + 1), 1e-9) };
            });
            Case(checks, "Humid inflow heater support radiation and mechanical work close once", "V02", () =>
            {
                var initial = FixtureInitial(); var ledger = new ConservationLedger(initial, new ConservationTolerance());
                ledger.RecordExternal(ExternalChannel.OpenBoundary, FluxDirection.IntoDomain, 0.001, 0.002, 5);
                ledger.RecordExternal(ExternalChannel.HeaterAbsorbed, FluxDirection.IntoDomain, 0, 0, 10);
                ledger.RecordExternal(ExternalChannel.SupportConduction, FluxDirection.IntoDomain, 0, 0, 2);
                ledger.RecordExternal(ExternalChannel.Radiation, FluxDirection.OutOfDomain, 0, 0, 3);
                ledger.RecordExternal(ExternalChannel.OpenBoundary, FluxDirection.OutOfDomain, 0.0005, 0.001, 1);
                ledger.RecordExternal(ExternalChannel.MechanicalWork, FluxDirection.OutOfDomain, 0, 0, 2);
                var current = initial.Copy(); current.vapourKg += 0.001; current.liquidKg -= 0.0005; current.dryAirKg += 0.001; current.internalEnergyJ += 11;
                Near(ledger.HeaterAbsorbedJ, 10, 0); Near(ledger.EnergyInJ, 17, 0); Near(ledger.EnergyOutJ, 6, 0);
                Require(ledger.Audit(current).passed, "Boundary ledger failed.");
                current.internalEnergyJ += 10;
                Require(!ledger.Audit(current).passed, "Double-counted heater was accepted.");
                return PassMetric();
            });
            Case(checks, "Numerical corrections remain invalid even if they restore the balance", "V02", () =>
            {
                var initial = FixtureInitial(); var ledger = new ConservationLedger(initial, new ConservationTolerance());
                ledger.RecordExternal(ExternalChannel.NumericalCorrection, FluxDirection.IntoDomain, 0.01, 0, 0);
                var current = initial.Copy(); current.liquidKg += 0.01;
                Require(!ledger.Audit(current).passed, "Mass redistribution was passed off as conservation.");
                Reject(() => ledger.RecordExternal(ExternalChannel.OpenBoundary, FluxDirection.IntoDomain, -1, 0, 0));
                Reject(() => ledger.RecordExternal(ExternalChannel.HeaterAbsorbed, FluxDirection.OutOfDomain, 0, 0, 1));
                return PassMetric();
            });
            Case(checks, "Bundle export snapshot reload and source identity round trip", "V33", () =>
            {
                var fixture = CreateFixture(scratchRoot); string path = WriteFixture(fixture);
                var restored = EvidenceValidator.VerifyBundle(path, fixture.sourceRoot);
                Require(restored.run_id == fixture.manifest.run_id, "Run identity changed.");
                var restoredState = RecordJson.Decode<InventoryState>(File.ReadAllBytes(Path.Combine(path, "field_snapshots/final.json")));
                Require(new ConservationTolerance().Matches(FixtureInitial(), restoredState), "Snapshot changed inventory.");
                Require(EvidenceValidator.ReadCsv(Path.Combine(path, "phase_transfers.csv"))[1][3] == "body,0", "Quoted CSV owner did not survive.");
                Reject(() => new RunRecorder(fixture.outputRoot, fixture.manifest, fixture.scenario, fixture.reference));
                return PassMetric();
            });
            Case(checks, "Unmatched transfer and lost heat invalidate exported evidence", "V33", () =>
            {
                var fixture = CreateFixture(scratchRoot); string path = WriteFixture(fixture, true);
                var record = EvidenceValidator.VerifyBundle(path, fixture.sourceRoot, false);
                Require(record.status == "INVALIDATED", "Broken conservation run was marked verified.");
                Reject(() => EvidenceValidator.VerifyBundle(path, fixture.sourceRoot));
                return PassMetric();
            });
            Case(checks, "Modified output bytes and missing seal are rejected", "V33", () =>
            {
                var fixture = CreateFixture(scratchRoot); string path = WriteFixture(fixture);
                string snapshot = Path.Combine(path, "field_snapshots/final.json");
                File.AppendAllText(snapshot, " "); Reject(() => EvidenceValidator.VerifyBundle(path, fixture.sourceRoot));
                var next = CreateFixture(scratchRoot); string incomplete = WriteFixture(next);
                File.Delete(Path.Combine(incomplete, "manifest.sha256")); Reject(() => EvidenceValidator.VerifyBundle(incomplete, next.sourceRoot));
                return PassMetric();
            });
            Case(checks, "Changed shader table or dependency set makes evidence stale", "V33", () =>
            {
                int rejected = 0;
                foreach (string change in new[] { "Fixture.compute", "MaterialTable.bytes", "NewKernel.compute" })
                {
                    var fixture = CreateFixture(scratchRoot); string path = WriteFixture(fixture);
                    File.AppendAllText(Path.Combine(fixture.sourceRoot, "Assets/Scripts/ThreePhase3D", change), "changed");
                    Reject(() => EvidenceValidator.VerifyBundle(path, fixture.sourceRoot)); rejected++;
                }
                return new[] { Metric("missed_dependency_changes", "1", 3 - rejected, 0) };
            });
            Case(checks, "Schema and required metric failures cannot be repaired by resealing hashes", "V33", () =>
            {
                var fixture = CreateFixture(scratchRoot); string path = WriteFixture(fixture);
                var manifest = LoadManifest(path); manifest.schema_version = "99.0"; Reseal(path, manifest);
                Reject(() => EvidenceValidator.VerifyBundle(path, fixture.sourceRoot));
                fixture = CreateFixture(scratchRoot); path = WriteFixture(fixture); manifest = LoadManifest(path);
                var summary = RecordJson.Decode<CheckSummary>(File.ReadAllBytes(Path.Combine(path, "summary.json")));
                summary.checks[0].metrics = new CheckMetric[0]; File.WriteAllBytes(Path.Combine(path, "summary.json"), RecordJson.Encode(summary));
                manifest.output_hashes["summary.json"] = RecordJson.FileHash(Path.Combine(path, "summary.json")); Reseal(path, manifest);
                Reject(() => EvidenceValidator.VerifyBundle(path, fixture.sourceRoot));
                return PassMetric();
            });
            Case(checks, "Independent CSV audit rejects fabricated passing balance and transfer flags", "V33", () =>
            {
                var fixture = CreateFixture(scratchRoot); string path = WriteFixture(fixture);
                string csvPath = Path.Combine(path, "timeseries.csv");
                var lines = File.ReadAllLines(csvPath); var columns = lines[1].Split(',');
                columns[3] = "0.999"; lines[1] = string.Join(",", columns); File.WriteAllLines(csvPath, lines);
                var manifest = LoadManifest(path); manifest.output_hashes["timeseries.csv"] = RecordJson.FileHash(csvPath); Reseal(path, manifest);
                Reject(() => EvidenceValidator.VerifyBundle(path, fixture.sourceRoot));
                fixture = CreateFixture(scratchRoot); path = WriteFixture(fixture); csvPath = Path.Combine(path, "phase_transfers.csv");
                var rows = EvidenceValidator.ReadCsv(csvPath); rows[2][8] = "3.40625"; // Original packet storage energy is 2.40625 J.
                File.WriteAllLines(csvPath, rows.Select(row => string.Join(",", row.Select(cell => "\"" + cell.Replace("\"", "\"\"") + "\""))));
                manifest = LoadManifest(path); manifest.output_hashes["phase_transfers.csv"] = RecordJson.FileHash(csvPath); Reseal(path, manifest);
                Reject(() => EvidenceValidator.VerifyBundle(path, fixture.sourceRoot));
                return PassMetric();
            });
            Case(checks, "Dirty development bundle is identifiable and rejected for clean review", "V33", () =>
            {
                var fixture = CreateFixture(scratchRoot);
                byte[] patch = Encoding.UTF8.GetBytes("synthetic dirty-fixture patch\n");
                fixture.manifest.provenance.dirty_tree = true; fixture.manifest.provenance.patch_sha256 = RecordJson.Hash(patch);
                Reject(() => new RunRecorder(fixture.outputRoot, fixture.manifest, fixture.scenario, fixture.reference));
                string path = WriteFixture(fixture, false, patch);
                EvidenceValidator.VerifyBundle(path, fixture.sourceRoot);
                Reject(() => EvidenceValidator.VerifyBundle(path, fixture.sourceRoot, true, true));
                return PassMetric();
            });
            Case(checks, "Interrupted runs path escapes and writes after sealing are rejected", "V33", () =>
            {
                var fixture = CreateFixture(scratchRoot); string path;
                using (var recorder = new RunRecorder(fixture.outputRoot, fixture.manifest, fixture.scenario, fixture.reference))
                { path = recorder.DirectoryPath; Reject(() => recorder.WriteSnapshot("../escape.json", new byte[0])); }
                Reject(() => EvidenceValidator.VerifyBundle(path, fixture.sourceRoot));
                Reject(() => ProvenanceCapture.SafePath(fixture.sourceRoot, "../../escape"));
                fixture = CreateFixture(scratchRoot);
                using (var recorder = new RunRecorder(fixture.outputRoot, fixture.manifest, fixture.scenario, fixture.reference))
                {
                    recorder.Record(new DiagnosticSample { timeS = 0, dtS = 0, inventory = fixture.manifest.initial_inventory.Copy() });
                    recorder.WriteSnapshot("final.json", RecordJson.Encode(fixture.manifest.initial_inventory));
                    recorder.Complete(new[] { PassingCheck("V33", "sealed") }, new[] { "Unit-test fixture only." });
                    Reject(() => recorder.Log("late write"));
                }
                return PassMetric();
            });
            Case(checks, "Evidence index resolves all requirements and actual code symbols", "V33", () =>
            {
                EvidenceValidator.VerifyIndex(projectRoot);
                EvidenceValidator.VerifyFunctionContracts(projectRoot);
                return PassMetric();
            });
            Case(checks, "Evidence index rejects unknown identifiers and nonexistent symbols", "V33", () =>
            {
                string localFolder = "ValidationRuns/local/index-fixtures-" + Guid.NewGuid().ToString("N");
                Directory.CreateDirectory(ProvenanceCapture.SafePath(projectRoot, localFolder));
                string indexPath = "Docs/ThreePhase3D/ImplementationEvidence.json";
                var index = RecordJson.Decode<EvidenceIndex>(File.ReadAllBytes(Path.Combine(projectRoot, indexPath)));
                index.records[0].requirement_id = "R999";
                string relative = localFolder + "/unknown.json"; File.WriteAllBytes(ProvenanceCapture.SafePath(projectRoot, relative), RecordJson.Encode(index));
                Reject(() => EvidenceValidator.VerifyIndex(projectRoot, relative));
                index = RecordJson.Decode<EvidenceIndex>(File.ReadAllBytes(Path.Combine(projectRoot, indexPath)));
                index.records[0].code_symbols = new[] { new CodeSymbol { path = "Assets/Scripts/ThreePhase3D/Diagnostics/ConservedInventory.cs", symbol = "PhysicsSim.ThreePhase3D.Diagnostics.ConservedInventory.DoesNotExist" } };
                relative = localFolder + "/symbol.json"; File.WriteAllBytes(ProvenanceCapture.SafePath(projectRoot, relative), RecordJson.Encode(index));
                Reject(() => EvidenceValidator.VerifyIndex(projectRoot, relative));
                return PassMetric();
            });
            return checks;
        }

        public static InventoryState FixtureInitial() => new InventoryState {
            liquidKg = 1, dryAirKg = 0.01, internalEnergyJ = 10, kineticEnergyJ = 2, gravitationalEnergyJ = 3,
            solidInternalEnergyJ = 4, interfaceEnergyJ = 0.25, momentumXKgMPerS = 0.5, momentumYKgMPerS = 0.2, momentumZKgMPerS = -0.1,
            angularMomentumXKgM2PerS = 0.2, angularMomentumYKgM2PerS = -0.1, angularMomentumZKgM2PerS = 0.05 };
        static InventoryState Portion(InventoryState state, double fraction) => InventoryState.FromArray(state.ToArray().Select(value => value * fraction).ToArray());

        public sealed class RecorderFixture
        { public string sourceRoot, outputRoot; public RunManifest manifest; public byte[] scenario, reference; }
        static RecorderFixture CreateFixture(string scratchRoot)
        {
            string root = Path.Combine(scratchRoot, Guid.NewGuid().ToString("N"));
            string sources = Path.Combine(root, "source"); Directory.CreateDirectory(Path.Combine(sources, "Assets/Scripts/ThreePhase3D"));
            File.WriteAllText(Path.Combine(sources, "Assets/Scripts/ThreePhase3D/Fixture.compute"), "// synthetic dependency identity; not an executed GPU kernel\n");
            File.WriteAllBytes(Path.Combine(sources, "Assets/Scripts/ThreePhase3D/MaterialTable.bytes"), new byte[] { 1, 2, 3, 4 });
            var scenario = new ScenarioConfigSI();
            byte[] scenarioBytes = RecordJson.Encode(scenario), reference = Encoding.UTF8.GetBytes("S01 unit-test arithmetic identity, not measured material data.\n");
            return new RecorderFixture { sourceRoot = sources, outputRoot = Path.Combine(root, "runs"), scenario = scenarioBytes, reference = reference,
                manifest = new RunManifest {
                    run_id = "fixture-" + Guid.NewGuid().ToString("N"), run_kind = "foundation-bookkeeping",
                    provenance = new RunProvenance { git_commit_sha = new string('0', 40), dirty_tree = false,
                        executable_sha256 = RecordJson.FileHash(typeof(FoundationChecks).Assembly.Location), executable_kind = "Synthetic test fixture host",
                        unity_version = "2022.3.46f1", scene_guid = new string('0', 32), scenario_sha256 = RecordJson.Hash(scenarioBytes), source_files = ProvenanceCapture.CaptureSources(sources) },
                    hardware = new RunHardware { os = Environment.OSVersion.ToString(), gpu = "NOT_EXECUTED", graphics_api = "NOT_EXECUTED", driver = "NOT_QUERIED", gpu_executed = false },
                    materials = new MaterialRunIdentity { generator_version = "NOT_USED", enthalpy_reference = "Synthetic zero chosen for arithmetic", entropy_reference = "NOT_COMPUTED", validity_domain = "No physical material model", references = scenario.materials },
                    physics = scenario.physical, numerics = scenario.numerics, execution = new RunExecution { requested_duration_s = 0, diagnostic_interval_s = 0.1 },
                    energy_convention = "Sum internal+kinetic+gravitational+solid+interface storage; synthetic input J, not an EOS.",
                    initial_inventory = FixtureInitial(), conservation_tolerance = new ConservationTolerance(),
                    geometry_sha256 = RecordJson.Hash(RecordJson.Encode(scenario.physical.geometry)), reference_dataset_sha256 = RecordJson.Hash(reference),
                    reference_definition = "Synthetic unit-test fixture, no physical accuracy claim.", boundary_conditions = "Closed bookkeeping fixture", timestep_policy = "No field timestep", precision = "FP64 diagnostic arithmetic", seed_protocol = "Deterministic; no stochastic physics",
                    approximation_flags = new[] { "BOOKKEEPING_ONLY", "SYNTHETIC_TEST_IDENTITIES" }, unresolved_scales = new[] { "No continuum fields allocated." }
                } };
        }

        static string WriteFixture(RecorderFixture fixture, bool broken = false, byte[] dirtyPatch = null)
        {
            using (var recorder = new RunRecorder(fixture.outputRoot, fixture.manifest, fixture.scenario, fixture.reference, dirtyPatch))
            {
                recorder.Record(new DiagnosticSample { timeS = 0, dtS = 0, inventory = fixture.manifest.initial_inventory.Copy() });
                var loss = Portion(fixture.manifest.initial_inventory, 0.125); var gain = loss.Copy(); if (broken) gain.internalEnergyJ += 1;
                recorder.RecordTransfer("transfer-1", 0, 0, "body,0", "Eulerian \"water\"", loss, gain);
                recorder.WriteSnapshot("final.json", RecordJson.Encode(fixture.manifest.initial_inventory));
                recorder.Complete(new[] { PassingCheck("V33", "fixture") }, new[] { "Synthetic fixture for recorder failure detection; not a physical run." });
                return recorder.DirectoryPath;
            }
        }
        static RunManifest LoadManifest(string path) => RecordJson.Decode<RunManifest>(File.ReadAllBytes(Path.Combine(path, "manifest.json")));
        static void Reseal(string path, RunManifest manifest)
        { File.WriteAllBytes(Path.Combine(path, "manifest.json"), RecordJson.Encode(manifest)); File.WriteAllText(Path.Combine(path, "manifest.sha256"), RecordJson.FileHash(Path.Combine(path, "manifest.json")) + "\n"); }
        public static CheckResult PassingCheck(string id, string name) => new CheckResult { validationId = id, name = name, passed = true, detail = "Declared assertions passed.", metrics = PassMetric() };
        public static CheckMetric Metric(string name, string unit, double actual, double maximum)
        { var metric = new CheckMetric { name = name, unit = unit, definition = "Absolute error/detection failure count for " + name, actual = actual, maximum_allowed = maximum }; metric.passed = EvidenceValidator.MetricPasses(metric); return metric; }
        public static CheckMetric[] PassMetric() => new[] { Metric("assertion_failures", "1", 0, 0) };
        static void Case(List<CheckResult> checks, string name, string id, Func<CheckMetric[]> action)
        {
            try { var metrics = action(); checks.Add(new CheckResult { name = name, validationId = id, passed = metrics.All(metric => metric.passed), detail = "Declared assertions executed.", metrics = metrics }); }
            catch (Exception ex) { checks.Add(new CheckResult { name = name, validationId = id, passed = false, detail = ex.ToString(), metrics = new[] { Metric("assertion_failures", "1", 1, 0) } }); }
        }
        static void Reject(Action action)
        {
            bool rejected = false;
            try { action(); } catch (ArgumentException) { rejected = true; } catch (InvalidOperationException) { rejected = true; } catch (IOException) { rejected = true; }
            Require(rejected, "Expected invalid operation/evidence was accepted.");
        }
    }
}
