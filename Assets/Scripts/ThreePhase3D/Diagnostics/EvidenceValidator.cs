using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Runtime.Serialization;
using System.Text;
using System.Text.RegularExpressions;
using PhysicsSim.ThreePhase3D.Core;

namespace PhysicsSim.ThreePhase3D.Diagnostics
{
    [DataContract] public sealed class CodeSymbol
    { [DataMember] public string path; [DataMember] public string symbol; [DataMember] public string source_sha256; }
    [DataContract] public sealed class EvidenceRunLink
    { [DataMember] public string path; [DataMember] public string manifest_sha256; [DataMember] public string git_commit_sha; }
    [DataContract] public sealed class EvidenceReview
    { [DataMember] public string decision; [DataMember] public string git_commit_sha; [DataMember] public string reviewer; }
    [DataContract] public sealed class RequirementEvidence
    {
        [DataMember] public string requirement_id;
        [DataMember] public string status;
        [DataMember] public CodeSymbol[] code_symbols;
        [DataMember] public string[] source_ids;
        [DataMember] public string[] source_locators;
        [DataMember] public string[] validation_ids;
        [DataMember] public EvidenceRunLink[] runs;
        [DataMember] public EvidenceReview review;
    }
    [DataContract] public sealed class EvidenceIndex
    { [DataMember] public string schema_version; [DataMember] public RequirementEvidence[] records; }

    public static class EvidenceValidator
    {
        public static readonly string[] Statuses = { "SPECIFIED", "IMPLEMENTED_UNVERIFIED", "VERIFIED_NUMERICALLY", "VALIDATED_FOR_CASE", "INVALIDATED", "BLOCKED_DATA" };

        // Physics-ID: DIAG-EVIDENCE-INTEGRITY-GATE
        // Implements: R047 R048 R056
        // Model: EvidenceProtocol.md sections 3-6; Specification.md section 14.
        // Sources: project evidence protocol; no physical constitutive law.
        // Source-Locators: EvidenceProtocol.md sections 4 (outputs) and 5 (freshness).
        // Inputs-SI: immutable bundle and current dependency tree; inventory/flux CSV in SI.
        // Outputs-SI: integrity/staleness errors; recomputed mass [kg] and energy [J] balances.
        // State-Ownership: read-only; never changes evidence or physical state.
        // Conservation: independently recomputes global residuals and pairwise transfer closure.
        // Discretisation: compensated FP64 sums of recorded data; exact SHA-256 identity.
        // Validity: S01 bookkeeping schema; clean-tree requirement is explicit for release review.
        // Validation: V02 V33 V34
        // Evidence: PENDING
        // Status: IMPLEMENTED_UNVERIFIED
        // Limitations: an internally consistent false input cannot be disproved by hashes/comments alone.
        public static RunManifest VerifyBundle(string directory, string sourceRoot, bool requirePassing = true, bool requireClean = false)
        {
            directory = Path.GetFullPath(directory); sourceRoot = Path.GetFullPath(sourceRoot);
            string manifestPath = Path.Combine(directory, "manifest.json");
            Require(File.Exists(manifestPath) && File.Exists(Path.Combine(directory, "manifest.sha256")), "Incomplete/unsealed run.");
            Require(File.ReadAllText(Path.Combine(directory, "manifest.sha256")).Trim() == RecordJson.FileHash(manifestPath), "Manifest seal mismatch.");
            var manifest = RecordJson.Decode<RunManifest>(File.ReadAllBytes(manifestPath));
            RunRecorder.ValidateManifest(manifest);
            if (requireClean) Require(!manifest.provenance.dirty_tree, "Dirty development evidence is not accepted release evidence.");
            if (manifest.provenance.dirty_tree)
                Require(RecordJson.FileHash(Path.Combine(directory, "working-tree.patch")) == manifest.provenance.patch_sha256, "Dirty patch mismatch.");
            Require(manifest.output_hashes != null && manifest.output_hashes.Count > 0, "Missing output inventory.");
            foreach (var pair in manifest.output_hashes)
            {
                string path = ProvenanceCapture.SafePath(directory, pair.Key);
                Require(File.Exists(path) && RunRecorder.IsHash(pair.Value) && RecordJson.FileHash(path) == pair.Value, "Output hash mismatch: " + pair.Key);
            }
            var actualPaths = Directory.GetFiles(directory, "*", SearchOption.AllDirectories)
                .Select(path => path.Substring(Path.GetFullPath(directory).TrimEnd(Path.DirectorySeparatorChar).Length + 1).Replace('\\', '/'))
                .Where(path => path != "manifest.json" && path != "manifest.sha256").OrderBy(path => path, StringComparer.Ordinal).ToArray();
            Require(actualPaths.SequenceEqual(manifest.output_hashes.Keys.OrderBy(path => path, StringComparer.Ordinal)), "Output file inventory changed.");
            foreach (string required in new[] { "manifest.inprogress.json", "scenario.json", "reference.json", "summary.json", "timeseries.csv", "probes.csv", "phase_transfers.csv", "boundary_fluxes.csv", "boundary_totals.json", "solver_history.csv", "console.log", "user_observations.md" })
                Require(manifest.output_hashes.ContainsKey(required), "Required output absent: " + required);
            Require(manifest.output_hashes.Keys.Any(path => path.StartsWith("field_snapshots/", StringComparison.Ordinal)), "No snapshot exported.");
            Require(RecordJson.FileHash(Path.Combine(directory, "scenario.json")) == manifest.provenance.scenario_sha256 &&
                RecordJson.FileHash(Path.Combine(directory, "reference.json")) == manifest.reference_dataset_sha256, "Input identity mismatch.");
            RunRecorder.ValidateScenario(manifest, File.ReadAllBytes(Path.Combine(directory, "scenario.json")));
            var current = ProvenanceCapture.CaptureSources(sourceRoot);
            Require(current.Length == manifest.provenance.source_files.Length, "Dependency file set changed.");
            var recorded = manifest.provenance.source_files.ToDictionary(file => file.path, StringComparer.Ordinal);
            foreach (var file in current) Require(recorded.ContainsKey(file.path) && recorded[file.path].sha256 == file.sha256 && recorded[file.path].hash_kind == file.hash_kind, "Stale evidence: " + file.path);
            var summary = RecordJson.Decode<CheckSummary>(File.ReadAllBytes(Path.Combine(directory, "summary.json")));
            Require(summary.schema_version == manifest.schema_version && summary.run_id == manifest.run_id && summary.status == manifest.status && summary.checks != null && summary.checks.Length > 0, "Summary identity mismatch.");
            foreach (var check in summary.checks)
            {
                Require(Regex.IsMatch(check.validationId ?? "", "^V[0-9]{2}$") && check.metrics != null && check.metrics.Length > 0, "Missing test IDs/metrics.");
                foreach (var metric in check.metrics)
                {
                    bool within = MetricPasses(metric);
                    Require(within == metric.passed, "Metric result disagrees with threshold: " + metric.name);
                    if (requirePassing) Require(metric.passed && check.passed, "A benchmark check failed.");
                }
            }
            VerifyTimeseries(directory, manifest, requirePassing);
            VerifyTransfers(directory, manifest.conservation_tolerance, requirePassing);
            if (requirePassing) Require(manifest.status == "VERIFIED_NUMERICALLY" && manifest.execution.failure_flags.Length == 0 && summary.checks.All(check => check.passed), "Run is invalidated or incomplete.");
            return manifest;
        }

        static void VerifyTimeseries(string directory, RunManifest manifest, bool requirePassing)
        {
            var rows = ReadCsv(Path.Combine(directory, "timeseries.csv"));
            Require(rows.Count >= 2 && string.Join(",", rows[0]) == RunRecorder.TimeseriesHeader, "Timeseries schema/initial sample missing.");
            double last = -1;
            for (int r = 1; r < rows.Count; r++)
            {
                var row = rows[r]; Require(row.Length == rows[0].Length, "Timeseries column count changed.");
                for (int i = 0; i < row.Length; i++) if (i != 32 && row[i].Length > 0) Number(row[i]);
                double time = Number(row[0]), dt = Number(row[1]);
                Require((last < 0 && time == 0 && dt == 0) || (time > last && Math.Abs(time - last - dt) <= 1e-10 * Math.Max(1, time)), "Inconsistent physical time.");
                double water = StableReduction.Sum(new[] { Number(row[2]), Number(row[3]), Number(row[4]), Number(row[5]), -manifest.initial_inventory.WaterKg, -Number(row[7]), Number(row[8]) });
                double energy = StableReduction.Sum(new[] { Number(row[10]), Number(row[11]), Number(row[12]), Number(row[33]), Number(row[34]), -manifest.initial_inventory.TotalEnergyJ, -Number(row[14]), Number(row[15]) });
                double air = StableReduction.Sum(new[] { Number(row[6]), -manifest.initial_inventory.dryAirKg, -Number(row[35]), Number(row[36]) });
                var tolerance = manifest.conservation_tolerance;
                Require(Math.Abs(water - Number(row[9])) <= tolerance.massKg && Math.Abs(energy - Number(row[16])) <= tolerance.energyJ && Math.Abs(air - Number(row[37])) <= tolerance.massKg, "Recorded residual disagrees with inventory.");
                Require(row[32] == "true" || row[32] == "false", "Invalid validity flag.");
                if (requirePassing) Require(row[32] == "true" && Math.Abs(water) <= tolerance.massKg && Math.Abs(air) <= tolerance.massKg && Math.Abs(energy) <= tolerance.energyJ, "Unmatched mass/energy transfer.");
                for (int i = 2; i <= 6; i++) Require(Number(row[i]) >= 0, "Negative mass in record.");
                last = time;
            }
            Require(Math.Abs(last - manifest.execution.simulated_duration_s) <= 1e-10, "Execution time differs from CSV.");
            if (requirePassing) Require(Math.Abs(last - manifest.execution.requested_duration_s) <= 1e-10, "Requested duration not reached.");
            var fluxes = ReadCsv(Path.Combine(directory, "boundary_fluxes.csv"));
            var sums = new StableReduction.Accumulator[6];
            for (int r = 1; r < fluxes.Count; r++)
            {
                var row = fluxes[r]; Require(row.Length == 7 && Enum.IsDefined(typeof(ExternalChannel), row[2]) && Enum.IsDefined(typeof(FluxDirection), row[3]), "Invalid boundary flux.");
                int offset = row[3] == "IntoDomain" ? 0 : 3;
                for (int i = 0; i < 3; i++) { double value = Number(row[4 + i]); if (i < 2) Require(value >= 0, "Negative boundary mass."); sums[offset + i].Add(value); }
                Require(Number(row[0]) >= 0 && Number(row[1]) >= 0 && Number(row[0]) + Number(row[1]) <= last + 1e-10, "Boundary event outside recorded interval.");
            }
            var final = rows[rows.Count - 1]; int[] columns = { 7, 35, 14, 8, 36, 15 };
            for (int i = 0; i < sums.Length; i++) Require(Math.Abs(sums[i].Value - Number(final[columns[i]])) <= (i % 3 == 2 ? manifest.conservation_tolerance.energyJ : manifest.conservation_tolerance.massKg), "Boundary totals disagree with event log.");
        }

        static void VerifyTransfers(string directory, ConservationTolerance tolerance, bool requirePassing)
        {
            var rows = ReadCsv(Path.Combine(directory, "phase_transfers.csv"));
            foreach (var group in rows.Skip(1).GroupBy(row => row[0]))
            {
                var pair = group.ToArray(); Require(pair.Length == 2 && pair.All(row => row.Length == 15), "Incomplete/duplicated transfer pair.");
                var loss = pair.Single(row => row[5] == "donor_loss"); var gain = pair.Single(row => row[5] == "receiver_gain");
                Require(loss.Take(5).SequenceEqual(gain.Take(5)), "Transfer owner/time mismatch.");
                for (int i = 6; i < 15; i++)
                {
                    double a = Number(loss[i]), b = Number(gain[i]);
                    if (requirePassing) Require(Math.Abs(a - b) <= (i < 8 ? tolerance.massKg : i == 8 ? tolerance.energyJ : i < 12 ? tolerance.momentumKgMPerS : tolerance.angularMomentumKgM2PerS), "Unmatched paired transfer.");
                }
            }
        }

        public static void VerifyIndex(string projectRoot, string relativePath = "Docs/ThreePhase3D/ImplementationEvidence.json")
        {
            var index = RecordJson.Decode<EvidenceIndex>(File.ReadAllBytes(ProvenanceCapture.SafePath(projectRoot, relativePath)));
            Require(index.schema_version == "1.0" && index.records != null, "Unsupported evidence index.");
            var requirements = new HashSet<string>(Regex.Matches(File.ReadAllText(Path.Combine(projectRoot, "Docs/ThreePhase3D/Requirements.csv")), "(?m)^\"(R[0-9]{3})\"").Cast<Match>().Select(match => match.Groups[1].Value));
            string sourceRegister = File.ReadAllText(Path.Combine(projectRoot, "Docs/ThreePhase3D/Sources.md"));
            string validations = File.ReadAllText(Path.Combine(projectRoot, "Docs/ThreePhase3D/Validation.md"));
            var ids = new HashSet<string>();
            foreach (var record in index.records)
            {
                Require(requirements.Contains(record.requirement_id) && ids.Add(record.requirement_id) && Statuses.Contains(record.status), "Unknown/duplicate requirement or status.");
                foreach (string id in record.source_ids ?? new string[0]) Require(Regex.IsMatch(sourceRegister, "(?m)^### " + Regex.Escape(id) + "\\b"), "Unknown source ID: " + id);
                foreach (string id in record.validation_ids ?? new string[0]) Require(Regex.IsMatch(validations, "\\| " + Regex.Escape(id) + " \\|"), "Unknown benchmark ID: " + id);
                foreach (var symbol in record.code_symbols ?? new CodeSymbol[0])
                {
                    string path = ProvenanceCapture.SafePath(projectRoot, symbol.path); Require(File.Exists(path), "Missing code path.");
                    int split = symbol.symbol.LastIndexOf('.'); Require(split > 0, "Qualified code symbol required.");
                    Type type = typeof(EvidenceValidator).Assembly.GetType(symbol.symbol.Substring(0, split));
                    Require(type != null && type.GetMember(symbol.symbol.Substring(split + 1), BindingFlags.Static | BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic).Length > 0, "Code symbol does not exist: " + symbol.symbol);
                    if (symbol.source_sha256 != null) Require(symbol.source_sha256 == ProvenanceCapture.SourceHash(path), "Code mapping hash is stale.");
                }
                foreach (var run in record.runs ?? new EvidenceRunLink[0])
                {
                    string path = ProvenanceCapture.SafePath(projectRoot, run.path);
                    Require(RecordJson.FileHash(Path.Combine(path, "manifest.json")) == run.manifest_sha256, "Linked run hash mismatch.");
                    var manifest = VerifyBundle(path, projectRoot, true, record.status == "VERIFIED_NUMERICALLY" || record.status == "VALIDATED_FOR_CASE");
                    Require(manifest.provenance.git_commit_sha == run.git_commit_sha, "Linked commit mismatch.");
                }
                if (record.status == "VERIFIED_NUMERICALLY" || record.status == "VALIDATED_FOR_CASE")
                    Require(record.code_symbols != null && record.code_symbols.Length > 0 && record.runs != null && record.runs.Length > 0 &&
                        record.review != null && record.review.decision == "ACCEPTED" && !string.IsNullOrWhiteSpace(record.review.reviewer) &&
                        record.runs.All(run => run.git_commit_sha == record.review.git_commit_sha), "Accepted requirement lacks review/run evidence.");
            }
            Require(ids.SetEquals(requirements), "Evidence index must cover every requirement.");
        }

        public static bool MetricPasses(Verification.CheckMetric metric)
        {
            Require(!string.IsNullOrWhiteSpace(metric.name) && !string.IsNullOrWhiteSpace(metric.unit) && !string.IsNullOrWhiteSpace(metric.definition) && SiUnits.IsFinite(metric.actual), "Incomplete/nonfinite metric.");
            Require(metric.minimum_allowed.HasValue || metric.maximum_allowed.HasValue, "Metric threshold absent.");
            if (metric.minimum_allowed.HasValue) SiUnits.RequireFinite(metric.minimum_allowed.Value, "lower threshold");
            if (metric.maximum_allowed.HasValue) SiUnits.RequireFinite(metric.maximum_allowed.Value, "upper threshold");
            Require(!metric.minimum_allowed.HasValue || !metric.maximum_allowed.HasValue || metric.minimum_allowed.Value <= metric.maximum_allowed.Value, "Inverted metric thresholds.");
            return (!metric.minimum_allowed.HasValue || metric.actual >= metric.minimum_allowed.Value) && (!metric.maximum_allowed.HasValue || metric.actual <= metric.maximum_allowed.Value);
        }

        public static void VerifyFunctionContracts(string projectRoot)
        {
            var ids = new HashSet<string>();
            string requirementText = File.ReadAllText(Path.Combine(projectRoot, "Docs/ThreePhase3D/Requirements.csv"));
            string validationText = File.ReadAllText(Path.Combine(projectRoot, "Docs/ThreePhase3D/Validation.md"));
            foreach (string path in Directory.GetFiles(Path.Combine(projectRoot, "Assets/Scripts/ThreePhase3D"), "*.cs", SearchOption.AllDirectories))
            {
                string text = File.ReadAllText(path);
                foreach (Match block in Regex.Matches(text, "(?m)^\\s*// Physics-ID: ([A-Z0-9-]+)[^\\r\\n]*(?:\\r?\\n[ \\t]*//[^\\r\\n]*)*"))
                {
                    Require(ids.Add(block.Groups[1].Value), "Duplicate Physics-ID.");
                    foreach (string field in new[] { "Implements", "Model", "Sources", "Source-Locators", "Inputs-SI", "Outputs-SI", "State-Ownership", "Conservation", "Discretisation", "Validity", "Validation", "Evidence", "Status", "Limitations" })
                        Require(Regex.IsMatch(block.Value, "// " + field + ": \\S"), "Missing contract field " + field + " in " + path);
                    string status = Regex.Match(block.Value, "// Status: ([A-Z_]+)").Groups[1].Value;
                    Require(Statuses.Contains(status), "Unknown function status.");
                    string requirements = Regex.Match(block.Value, "// Implements: ([^\\r\\n]+)").Groups[1].Value;
                    foreach (Match requirement in Regex.Matches(requirements, "R[0-9]{3}")) Require(requirementText.Contains("\"" + requirement.Value + "\""), "Unknown function requirement ID.");
                    foreach (Match benchmark in Regex.Matches(Regex.Match(block.Value, "// Validation: ([^\\r\\n]+)").Groups[1].Value, "V[0-9]{2}"))
                        Require(validationText.Contains("| " + benchmark.Value + " |"), "Unknown function benchmark ID.");
                    string evidence = Regex.Match(block.Value, "// Evidence: ([^\\r\\n]+)").Groups[1].Value.Trim();
                    if (evidence != "PENDING") Require(File.Exists(ProvenanceCapture.SafePath(projectRoot, evidence)), "Function evidence path absent.");
                }
            }
            Require(ids.Count >= 7, "Expected implemented physics/diagnostic contracts are missing.");
        }

        public static List<string[]> ReadCsv(string path)
        {
            var result = new List<string[]>();
            foreach (string line in File.ReadAllLines(path))
            {
                var fields = new List<string>(); var field = new StringBuilder(); bool quoted = false;
                for (int i = 0; i < line.Length; i++)
                {
                    char c = line[i];
                    if (c == '"') { if (quoted && i + 1 < line.Length && line[i + 1] == '"') { field.Append('"'); i++; } else quoted = !quoted; }
                    else if (c == ',' && !quoted) { fields.Add(field.ToString()); field.Clear(); } else field.Append(c);
                }
                Require(!quoted, "Unclosed CSV quote."); fields.Add(field.ToString()); result.Add(fields.ToArray());
            }
            return result;
        }
        static double Number(string value) { double result = double.Parse(value, NumberStyles.Float, CultureInfo.InvariantCulture); SiUnits.RequireFinite(result, "CSV value"); return result; }
        static void Require(bool condition, string reason) { if (!condition) throw new InvalidOperationException(reason); }
    }
}
