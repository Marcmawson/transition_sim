using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Runtime.Serialization;
using System.Text;
using PhysicsSim.ThreePhase3D.Core;

namespace PhysicsSim.ThreePhase3D.Diagnostics
{
    [DataContract]
    public sealed class ArtifactHash
    {
        [DataMember] public string path;
        [DataMember] public string sha256;
        [DataMember] public string hash_kind = "sha256-lf-text";
    }
    [DataContract]
    public sealed class RunProvenance
    {
        [DataMember] public string git_commit_sha;
        [DataMember] public bool dirty_tree;
        [DataMember] public string patch_sha256;
        [DataMember] public string executable_sha256;
        [DataMember] public string executable_kind;
        [DataMember] public string unity_version;
        [DataMember] public string scene_guid;
        [DataMember] public string scenario_sha256;
        [DataMember] public ArtifactHash[] source_files;
    }
    [DataContract]
    public sealed class RunHardware
    {
        [DataMember] public string os;
        [DataMember] public string gpu;
        [DataMember] public long? vram_bytes;
        [DataMember] public bool vram_is_approximate;
        [DataMember] public string driver;
        [DataMember] public string graphics_api;
        [DataMember] public bool? supports_compute;
        [DataMember] public bool? supports_async_readback;
        [DataMember] public bool gpu_executed;
    }
    [DataContract]
    public sealed class MaterialRunIdentity
    {
        [DataMember] public string table_sha256;
        [DataMember] public string generator_version;
        [DataMember] public string enthalpy_reference;
        [DataMember] public string entropy_reference;
        [DataMember] public string validity_domain;
        [DataMember] public MaterialReference[] references;
    }
    [DataContract]
    public sealed class RunExecution
    {
        [DataMember] public double requested_duration_s;
        [DataMember] public double simulated_duration_s;
        [DataMember] public double wall_duration_s;
        [DataMember] public double diagnostic_interval_s;
        [DataMember] public string[] failure_flags = new string[0];
        [DataMember] public int rejected_steps;
    }
    [DataContract]
    public sealed class RunManifest
    {
        [DataMember] public string schema_version = "1.0";
        [DataMember] public string run_id;
        [DataMember] public string run_kind;
        [DataMember] public string status = "IMPLEMENTED_UNVERIFIED";
        [DataMember] public RunProvenance provenance;
        [DataMember] public RunHardware hardware;
        [DataMember] public MaterialRunIdentity materials;
        [DataMember] public PhysicalInputs physics;
        [DataMember] public NumericalSettings numerics;
        [DataMember] public RunExecution execution;
        [DataMember] public string energy_convention;
        [DataMember] public string geometry_sha256;
        [DataMember] public string[] approximation_flags;
        [DataMember] public string[] unresolved_scales;
        [DataMember] public string reference_dataset_sha256;
        [DataMember] public string reference_definition;
        [DataMember] public InventoryState initial_inventory;
        [DataMember] public ConservationTolerance conservation_tolerance;
        [DataMember] public string boundary_conditions;
        [DataMember] public string timestep_policy;
        [DataMember] public string precision;
        [DataMember] public string seed_protocol;
        [DataMember] public SortedDictionary<string, string> output_hashes = new SortedDictionary<string, string>(StringComparer.Ordinal);
    }

    public static class ProvenanceCapture
    {
        public static RunProvenance Capture(string projectRoot, string executablePath, string executableKind,
            string unityVersion, string sceneGuid, byte[] scenarioBytes, out byte[] dirtyPatch)
        {
            string sha = Git(projectRoot, "rev-parse HEAD").Trim();
            if (!System.Text.RegularExpressions.Regex.IsMatch(sha, "^[0-9a-f]{40}$")) throw new InvalidOperationException("A fixed Git commit is required.");
            bool dirty = Git(projectRoot, "status --porcelain=v1 --untracked-files=all").Length > 0;
            ArtifactHash[] sources = CaptureSources(projectRoot);
            // Dirty development evidence includes tracked diffs and exact untracked owned source bytes.
            // It remains ineligible for final acceptance even when its arithmetic checks pass.
            var patch = new StringBuilder(Git(projectRoot, "diff --binary HEAD"));
            if (dirty) foreach (var artifact in sources)
            {
                if (Git(projectRoot, "ls-files --error-unmatch -- \"" + artifact.path + "\"", false).exitCode != 0)
                    patch.Append("\nUNTRACKED-SOURCE-BASE64 ").Append(artifact.path).Append('\n').Append(Convert.ToBase64String(File.ReadAllBytes(SafePath(projectRoot, artifact.path)))).Append('\n');
            }
            dirtyPatch = dirty ? Encoding.UTF8.GetBytes(patch.ToString()) : null;
            return new RunProvenance { git_commit_sha = sha, dirty_tree = dirty,
                patch_sha256 = dirty ? RecordJson.Hash(dirtyPatch) : null, executable_sha256 = RecordJson.FileHash(executablePath),
                executable_kind = executableKind, unity_version = unityVersion, scene_guid = sceneGuid,
                scenario_sha256 = RecordJson.Hash(scenarioBytes), source_files = sources };
        }

        public static ArtifactHash[] CaptureSources(string projectRoot)
        {
            var paths = new List<string>();
            foreach (string folder in new[] { "Assets/Scripts/ThreePhase3D", "Assets/Scenes/ThreePhase3D", "Tools/ThreePhase3D" })
            {
                string directory = SafePath(projectRoot, folder);
                if (Directory.Exists(directory)) foreach (string path in Directory.GetFiles(directory, "*", SearchOption.AllDirectories)) paths.Add(path);
            }
            foreach (string path in new[] { "ProjectSettings/ProjectVersion.txt", "ProjectSettings/GraphicsSettings.asset", "Packages/manifest.json", "Packages/packages-lock.json",
                "Docs/ThreePhase3D/Specification.md", "Docs/ThreePhase3D/Requirements.csv", "Docs/ThreePhase3D/Materials.csv", "Docs/ThreePhase3D/Validation.md", "Docs/ThreePhase3D/EvidenceProtocol.md", ".gitattributes" })
                if (File.Exists(SafePath(projectRoot, path))) paths.Add(SafePath(projectRoot, path));
            paths.Sort(StringComparer.Ordinal);
            var artifacts = new List<ArtifactHash>();
            string root = Path.GetFullPath(projectRoot).TrimEnd(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar) + Path.DirectorySeparatorChar;
            foreach (string path in paths) artifacts.Add(new ArtifactHash { path = path.Substring(root.Length).Replace('\\', '/'), sha256 = SourceHash(path),
                hash_kind = Path.GetExtension(path).Equals(".bytes", StringComparison.OrdinalIgnoreCase) || Path.GetExtension(path).Equals(".bin", StringComparison.OrdinalIgnoreCase) ? "sha256-bytes" : "sha256-lf-text" });
            return artifacts.ToArray();
        }

        public static string SourceHash(string path)
        {
            // Text identity survives Windows checkout newline conversion; tables/binaries retain byte identity.
            string extension = Path.GetExtension(path).ToLowerInvariant();
            if (extension == ".bin" || extension == ".bytes") return RecordJson.FileHash(path);
            return RecordJson.Utf8Hash(File.ReadAllText(path).Replace("\r\n", "\n").Replace("\r", "\n"));
        }

        public static string SafePath(string root, string relative)
        {
            if (string.IsNullOrWhiteSpace(relative) || Path.IsPathRooted(relative) || relative.IndexOf(':') >= 0)
                throw new InvalidOperationException("Relative artifact path required.");
            string prefix = Path.GetFullPath(root).TrimEnd(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar) + Path.DirectorySeparatorChar;
            string full = Path.GetFullPath(Path.Combine(prefix, relative.Replace('/', Path.DirectorySeparatorChar)));
            if (!full.StartsWith(prefix, StringComparison.OrdinalIgnoreCase)) throw new InvalidOperationException("Artifact escapes its root.");
            // Avoid following a symlink/junction outside an otherwise lexically safe artifact root.
            for (string check = full; check.Length >= prefix.Length; check = Path.GetDirectoryName(check))
                if ((File.Exists(check) || Directory.Exists(check)) && (File.GetAttributes(check) & FileAttributes.ReparsePoint) != 0)
                    throw new InvalidOperationException("Artifact path contains a link/junction.");
            return full;
        }

        static string Git(string root, string arguments) => Git(root, arguments, true).output;
        static (int exitCode, string output) Git(string root, string arguments, bool mustSucceed)
        {
            var start = new ProcessStartInfo("git", arguments) { WorkingDirectory = root, UseShellExecute = false,
                RedirectStandardOutput = true, RedirectStandardError = true, CreateNoWindow = true };
            using (var process = Process.Start(start))
            {
                string output = process.StandardOutput.ReadToEnd(); string error = process.StandardError.ReadToEnd(); process.WaitForExit();
                if (mustSucceed && process.ExitCode != 0) throw new InvalidOperationException("Git provenance query failed: " + error);
                return (process.ExitCode, output);
            }
        }
    }
}
