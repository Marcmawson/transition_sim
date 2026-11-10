using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Runtime.Serialization;
using System.Text;
using System.Text.RegularExpressions;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Verification;

namespace PhysicsSim.ThreePhase3D.Diagnostics
{
    public sealed class DiagnosticSample
    {
        public double timeS, dtS;
        public InventoryState inventory;
        public double? entropyJPerK, entropyInJPerK, entropyOutJPerK;
        public double? pressureMinPa, pressureMaxPa, temperatureMinK, temperatureMaxK;
        public double? maximumSpeedMPerS, maximumMach, volumeConstraintLinfPerS;
        public double? pressureRelativeResidual, nonlinearRelativeResidual, maximumCfl, gpuStepMs;
    }

    [DataContract]
    public sealed class CheckSummary
    {
        [DataMember] public string schema_version = "1.0";
        [DataMember] public string run_id;
        [DataMember] public string status;
        [DataMember] public CheckResult[] checks;
        [DataMember] public string[] limitations;
    }

    public sealed class RunRecorder : IDisposable
    {
        public const string TimeseriesHeader = "time_s,dt_s,m_ice_kg,m_liquid_kg,m_vapour_kg,m_parcel_water_kg,m_dry_air_kg,m_water_in_kg,m_water_out_kg,water_mass_residual_kg,internal_energy_J,kinetic_energy_J,gravitational_energy_J,heater_absorbed_J,energy_in_J,energy_out_J,energy_residual_J,entropy_J_per_K,entropy_in_J_per_K,entropy_out_J_per_K,p_abs_min_Pa,p_abs_max_Pa,T_min_K,T_max_K,max_speed_m_per_s,max_Mach,volume_constraint_linf_per_s,pressure_relative_residual,nonlinear_relative_residual,max_CFL,gpu_step_ms,wall_elapsed_s,valid_run,solid_internal_energy_J,interface_energy_J,m_dry_air_in_kg,m_dry_air_out_kg,dry_air_mass_residual_kg";
        public string DirectoryPath { get; }
        readonly RunManifest manifest;
        readonly ConservationLedger ledger;
        readonly Stopwatch clock = Stopwatch.StartNew();
        readonly List<string> failures = new List<string>();
        readonly HashSet<string> transferIds = new HashSet<string>(StringComparer.Ordinal);
        StreamWriter timeseries, transfers, boundary, console;
        double lastTime = -1;
        bool completed, disposed;

        public RunRecorder(string parentDirectory, RunManifest manifest, byte[] scenario, byte[] reference, byte[] dirtyPatch = null)
        {
            ValidateManifest(manifest);
            if (RecordJson.Hash(scenario) != manifest.provenance.scenario_sha256 || RecordJson.Hash(reference) != manifest.reference_dataset_sha256)
                throw new InvalidOperationException("Scenario/reference hash differs from manifest.");
            ValidateScenario(manifest, scenario);
            if (manifest.output_hashes == null || manifest.output_hashes.Count != 0) throw new InvalidOperationException("A new run cannot reuse sealed output metadata.");
            if (manifest.provenance.dirty_tree && (dirtyPatch == null || RecordJson.Hash(dirtyPatch) != manifest.provenance.patch_sha256))
                throw new InvalidOperationException("Dirty development run requires its patch.");
            DirectoryPath = ProvenanceCapture.SafePath(parentDirectory, manifest.run_id);
            if (Directory.Exists(DirectoryPath)) throw new IOException("Run already exists; it cannot be overwritten.");
            Directory.CreateDirectory(DirectoryPath);
            using (new FileStream(Path.Combine(DirectoryPath, "run.lock"), FileMode.CreateNew, FileAccess.Write)) { }
            this.manifest = RecordJson.Decode<RunManifest>(RecordJson.Encode(manifest));
            ledger = new ConservationLedger(this.manifest.initial_inventory, this.manifest.conservation_tolerance);
            WriteNew("manifest.inprogress.json", RecordJson.Encode(this.manifest));
            WriteNew("scenario.json", scenario); WriteNew("reference.json", reference);
            if (dirtyPatch != null) WriteNew("working-tree.patch", dirtyPatch);
            Directory.CreateDirectory(Path.Combine(DirectoryPath, "field_snapshots"));
            Directory.CreateDirectory(Path.Combine(DirectoryPath, "plots"));
            timeseries = Writer("timeseries.csv", TimeseriesHeader);
            transfers = Writer("phase_transfers.csv", "event_id,time_s,dt_s,donor,receiver,side,m_water_kg,m_dry_air_kg,total_storage_energy_J,px_kg_m_per_s,py_kg_m_per_s,pz_kg_m_per_s,Lx_kg_m2_per_s,Ly_kg_m2_per_s,Lz_kg_m2_per_s");
            boundary = Writer("boundary_fluxes.csv", "time_s,dt_s,channel,direction,water_kg,dry_air_kg,energy_J");
            console = Writer("console.log", "S01 bookkeeping; unavailable physical fields are blank, never invented as zero.");
            WriteNew("probes.csv", Encoding.UTF8.GetBytes("time_s,probe_id,x_m,y_m,z_m,T_K,p_abs_Pa,u_m_per_s,v_m_per_s,w_m_per_s,support_radius_m\n"));
            WriteNew("solver_history.csv", Encoding.UTF8.GetBytes("time_s,stage,iteration,relative_residual,volume_constraint_linf_per_s,accepted\n"));
            WriteNew("user_observations.md", Encoding.UTF8.GetBytes("No user-local observations have been supplied for this run.\n"));
        }

        // Physics-ID: DIAG-RECORD-ACCEPTED-STATE
        // Implements: R039 R042 R048 R056
        // Model: Specification.md section 14; EvidenceProtocol.md sections 3 and 4.
        // Sources: project control-volume and run-schema contracts.
        // Source-Locators: Specification.md section 14, R_M and R_E; EvidenceProtocol.md section 4.
        // Inputs-SI: time and dt [s]; independently reduced inventory [kg, J]; optional SI diagnostics.
        // Outputs-SI: CSV columns labelled with units; unavailable quantities are empty.
        // State-Ownership: recorder only; never changes simulated state or advances time.
        // Conservation: initial storage is pinned; residuals use current inventory and boundary sums.
        // Discretisation: compensated FP64 audit at explicitly supplied accepted times.
        // Validity: finite data; initial t=0; subsequent time increments equal dt.
        // Validation: V02 V33 V34
        // Evidence: PENDING
        // Status: IMPLEMENTED_UNVERIFIED
        // Limitations: caller must supply actual reduced fields; S01 exercises synthetic inventories only.
        public LedgerResidual Record(DiagnosticSample sample)
        {
            EnsureOpen();
            SiUnits.RequireNonnegative(sample.timeS, "time s"); SiUnits.RequireNonnegative(sample.dtS, "dt s");
            if (lastTime >= 0 && (sample.timeS <= lastTime || Math.Abs(sample.timeS - lastTime - sample.dtS) > 1e-10 * Math.Max(1, sample.timeS)))
                throw new InvalidOperationException("Time must advance by the stated dt.");
            if (lastTime < 0 && (sample.timeS != 0 || sample.dtS != 0)) throw new InvalidOperationException("Record the initial state at t=0, dt=0.");
            sample.inventory.ValidatePhysicalInventory();
            if (lastTime < 0 && RecordJson.Hash(RecordJson.Encode(sample.inventory)) != RecordJson.Hash(RecordJson.Encode(manifest.initial_inventory)))
                throw new InvalidOperationException("Initial inventory differs from manifest.");
            var residual = ledger.Audit(sample.inventory);
            if (!residual.passed) Fail("CONSERVATION_AUDIT_FAILED");
            var m = sample.inventory;
            var values = new double?[] { sample.timeS, sample.dtS, m.iceKg, m.liquidKg, m.vapourKg, m.parcelWaterKg, m.dryAirKg,
                ledger.WaterInKg, ledger.WaterOutKg, residual.waterKg, m.internalEnergyJ, m.kineticEnergyJ, m.gravitationalEnergyJ,
                ledger.HeaterAbsorbedJ, ledger.EnergyInJ, ledger.EnergyOutJ, residual.energyJ,
                sample.entropyJPerK, sample.entropyInJPerK, sample.entropyOutJPerK,
                sample.pressureMinPa, sample.pressureMaxPa, sample.temperatureMinK, sample.temperatureMaxK,
                sample.maximumSpeedMPerS, sample.maximumMach, sample.volumeConstraintLinfPerS,
                sample.pressureRelativeResidual, sample.nonlinearRelativeResidual, sample.maximumCfl, sample.gpuStepMs, clock.Elapsed.TotalSeconds };
            var cells = new List<string>(); foreach (var value in values) cells.Add(Number(value));
            cells.Add(residual.passed && failures.Count == 0 ? "true" : "false");
            foreach (double value in new[] { m.solidInternalEnergyJ, m.interfaceEnergyJ, ledger.DryAirInKg, ledger.DryAirOutKg, residual.dryAirKg }) cells.Add(Number(value));
            timeseries.WriteLine(string.Join(",", cells)); lastTime = sample.timeS; return residual;
        }

        public void RecordExternal(double timeS, double dtS, ExternalChannel channel, FluxDirection direction, double waterKg, double dryAirKg, double energyJ)
        {
            EnsureOpen(); SiUnits.RequireNonnegative(timeS, nameof(timeS)); SiUnits.RequireNonnegative(dtS, nameof(dtS));
            ledger.RecordExternal(channel, direction, waterKg, dryAirKg, energyJ);
            boundary.WriteLine(string.Join(",", new[] { Number(timeS), Number(dtS), channel.ToString(), direction.ToString(), Number(waterKg), Number(dryAirKg), Number(energyJ) }));
        }
        public void RecordTransfer(string eventId, double timeS, double dtS, string donor, string receiver, InventoryState loss, InventoryState gain)
        {
            EnsureOpen(); SiUnits.RequireNonnegative(timeS, nameof(timeS)); SiUnits.RequireNonnegative(dtS, nameof(dtS));
            if (string.IsNullOrWhiteSpace(donor) || string.IsNullOrWhiteSpace(receiver) || donor == receiver) throw new ArgumentException("Distinct donor and receiver IDs required.");
            if (string.IsNullOrWhiteSpace(eventId) || !transferIds.Add(eventId)) throw new ArgumentException("Unique transfer event ID required.");
            WriteTransferSide(eventId, timeS, dtS, donor, receiver, "donor_loss", loss);
            WriteTransferSide(eventId, timeS, dtS, donor, receiver, "receiver_gain", gain);
            if (!manifest.conservation_tolerance.Matches(loss, gain)) Fail("UNMATCHED_PHASE_TRANSFER:" + eventId);
        }
        void WriteTransferSide(string id, double time, double dt, string donor, string receiver, string side, InventoryState state)
        {
            state.ValidatePhysicalInventory();
            var cells = new List<string> { Csv(id), Number(time), Number(dt), Csv(donor), Csv(receiver), side };
            foreach (double value in new[] { state.WaterKg, state.dryAirKg, state.TotalEnergyJ, state.momentumXKgMPerS, state.momentumYKgMPerS, state.momentumZKgMPerS,
                state.angularMomentumXKgM2PerS, state.angularMomentumYKgM2PerS, state.angularMomentumZKgM2PerS }) cells.Add(Number(value));
            transfers.WriteLine(string.Join(",", cells));
        }
        public void WriteSnapshot(string name, byte[] contents)
        {
            EnsureOpen();
            if (!Regex.IsMatch(name ?? "", "^[A-Za-z0-9_-]+\\.(json|bin)$")) throw new ArgumentException("Simple snapshot filename required.");
            WriteNew("field_snapshots/" + name, contents);
        }
        public void Log(string message) { EnsureOpen(); console.WriteLine(message); }
        public void Fail(string reason)
        {
            EnsureOpen(); if (string.IsNullOrWhiteSpace(reason)) throw new ArgumentException("Failure reason required.");
            if (!failures.Contains(reason)) failures.Add(reason);
        }

        public void Complete(CheckResult[] checks, string[] limitations)
        {
            EnsureOpen(); if (checks == null || checks.Length == 0) throw new ArgumentException("Actual check results required.");
            foreach (var check in checks) if (!check.passed) Fail("CHECK_FAILED:" + check.name);
            if (lastTime < 0 || Math.Abs(lastTime - manifest.execution.requested_duration_s) > 1e-10) Fail("REQUESTED_END_TIME_NOT_REACHED");
            CloseWriters();
            manifest.execution.simulated_duration_s = Math.Max(0, lastTime);
            manifest.execution.wall_duration_s = clock.Elapsed.TotalSeconds; manifest.execution.failure_flags = failures.ToArray();
            manifest.status = failures.Count > 0 ? "INVALIDATED" : "VERIFIED_NUMERICALLY";
            WriteNew("summary.json", RecordJson.Encode(new CheckSummary { run_id = manifest.run_id, status = manifest.status, checks = checks, limitations = limitations }));
            WriteNew("boundary_totals.json", RecordJson.Encode(ledger.ExternalChannelTotals()));
            foreach (string path in Directory.GetFiles(DirectoryPath, "*", SearchOption.AllDirectories))
                manifest.output_hashes.Add(path.Substring(DirectoryPath.Length + 1).Replace('\\', '/'), RecordJson.FileHash(path));
            WriteNew("manifest.json", RecordJson.Encode(manifest));
            WriteNew("manifest.sha256", Encoding.ASCII.GetBytes(RecordJson.FileHash(Path.Combine(DirectoryPath, "manifest.json")) + "\n"));
            completed = true;
        }

        public static void ValidateManifest(RunManifest m)
        {
            if (m == null || m.schema_version != "1.0" || !Regex.IsMatch(m.run_id ?? "", "^[A-Za-z0-9_-]+$") ||
                m.provenance == null || m.hardware == null || m.physics == null || m.numerics == null || m.execution == null || m.materials == null || m.initial_inventory == null || m.conservation_tolerance == null)
                throw new InvalidOperationException("Incomplete manifest.");
            m.initial_inventory.ValidatePhysicalInventory(); m.conservation_tolerance.Validate();
            if (!Regex.IsMatch(m.provenance.git_commit_sha ?? "", "^[0-9a-f]{40}$") || !IsHash(m.provenance.executable_sha256) ||
                !IsHash(m.provenance.scenario_sha256) || !IsHash(m.geometry_sha256) || !IsHash(m.reference_dataset_sha256) ||
                string.IsNullOrWhiteSpace(m.energy_convention) || string.IsNullOrWhiteSpace(m.boundary_conditions) ||
                string.IsNullOrWhiteSpace(m.timestep_policy) || string.IsNullOrWhiteSpace(m.precision) || string.IsNullOrWhiteSpace(m.seed_protocol) ||
                string.IsNullOrWhiteSpace(m.provenance.executable_kind) || string.IsNullOrWhiteSpace(m.provenance.unity_version) ||
                !Regex.IsMatch(m.provenance.scene_guid ?? "", "^[0-9a-f]{32}$") || m.provenance.source_files == null || m.provenance.source_files.Length == 0 ||
                string.IsNullOrWhiteSpace(m.hardware.os) || string.IsNullOrWhiteSpace(m.hardware.gpu) ||
                string.IsNullOrWhiteSpace(m.hardware.graphics_api) || string.IsNullOrWhiteSpace(m.hardware.driver) ||
                string.IsNullOrWhiteSpace(m.reference_definition) || string.IsNullOrWhiteSpace(m.materials.generator_version) ||
                string.IsNullOrWhiteSpace(m.materials.enthalpy_reference) || string.IsNullOrWhiteSpace(m.materials.entropy_reference) ||
                string.IsNullOrWhiteSpace(m.materials.validity_domain) || m.materials.references == null ||
                m.execution.failure_flags == null || m.approximation_flags == null || m.unresolved_scales == null)
                throw new InvalidOperationException("Missing source/build/input identity.");
            SiUnits.RequireNonnegative(m.execution.requested_duration_s, "requested duration s");
            SiUnits.RequireNonnegative(m.execution.simulated_duration_s, "simulated duration s");
            SiUnits.RequireNonnegative(m.execution.wall_duration_s, "elapsed wall duration s");
            SiUnits.RequirePositive(m.execution.diagnostic_interval_s, "diagnostic interval s");
            if (m.run_kind != "foundation-bookkeeping" || m.hardware.gpu_executed)
                throw new InvalidOperationException("S01 records CPU bookkeeping checks; a physical/GPU run requires the later benchmark contract.");
            var paths = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            foreach (var file in m.provenance.source_files)
                if (!paths.Add(file.path) || !IsHash(file.sha256) || (file.hash_kind != "sha256-lf-text" && file.hash_kind != "sha256-bytes")) throw new InvalidOperationException("Invalid source identity.");
        }
        public static void ValidateScenario(RunManifest manifest, byte[] scenarioBytes)
        {
            var scenario = RecordJson.Decode<ScenarioConfigSI>(scenarioBytes);
            if (!scenario.Validate().CanPreview || RecordJson.Hash(RecordJson.Encode(scenario.physical)) != RecordJson.Hash(RecordJson.Encode(manifest.physics)) ||
                RecordJson.Hash(RecordJson.Encode(scenario.numerics)) != RecordJson.Hash(RecordJson.Encode(manifest.numerics)) ||
                RecordJson.Hash(RecordJson.Encode(scenario.materials)) != RecordJson.Hash(RecordJson.Encode(manifest.materials.references)) ||
                RecordJson.Hash(RecordJson.Encode(scenario.physical.geometry)) != manifest.geometry_sha256)
                throw new InvalidOperationException("Manifest and scenario settings/geometry disagree.");
        }
        public static bool IsHash(string value) => Regex.IsMatch(value ?? "", "^[0-9a-f]{64}$");
        void WriteNew(string relative, byte[] bytes)
        { using (var stream = new FileStream(ProvenanceCapture.SafePath(DirectoryPath, relative), FileMode.CreateNew, FileAccess.Write)) stream.Write(bytes, 0, bytes.Length); }
        StreamWriter Writer(string relative, string header)
        {
            var writer = new StreamWriter(new FileStream(Path.Combine(DirectoryPath, relative), FileMode.CreateNew, FileAccess.Write), new UTF8Encoding(false)) { NewLine = "\n", AutoFlush = true };
            writer.WriteLine(header); return writer;
        }
        void EnsureOpen() { if (completed || disposed) throw new InvalidOperationException("Run record is sealed or disposed."); }
        static string Csv(string text)
        {
            if (text != null && (text.Contains("\r") || text.Contains("\n"))) throw new ArgumentException("CSV identifiers cannot contain line breaks.");
            return "\"" + (text ?? "").Replace("\"", "\"\"") + "\"";
        }
        static string Number(double? value)
        { if (!value.HasValue) return ""; SiUnits.RequireFinite(value.Value, "recorded scalar"); return value.Value.ToString("R", CultureInfo.InvariantCulture); }
        void CloseWriters() { timeseries?.Dispose(); transfers?.Dispose(); boundary?.Dispose(); console?.Dispose(); }
        public void Dispose()
        {
            if (disposed) return;
            if (!completed) { Fail("RECORDER_DISPOSED_BEFORE_COMPLETE"); CloseWriters(); WriteNew("interrupted.json", RecordJson.Encode(failures.ToArray())); }
            disposed = true;
        }
    }
}
