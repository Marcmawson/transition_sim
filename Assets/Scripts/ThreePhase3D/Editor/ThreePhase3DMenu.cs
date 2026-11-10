using System;
using System.IO;
using System.Text;
using System.Linq;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Diagnostics;
using PhysicsSim.ThreePhase3D.Rendering;
using PhysicsSim.ThreePhase3D.Verification;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Editor
{
    public static class ThreePhase3DMenu
    {
        public const string BeakerScene = "Assets/Scenes/ThreePhase3D/Beaker3D.unity";

        [MenuItem("Physics Sim/Three Phase 3D/Open empty Beaker3D")]
        public static void OpenBeaker()
        {
            if (EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo()) EditorSceneManager.OpenScene(BeakerScene);
        }

        [MenuItem("Physics Sim/Three Phase 3D/Run foundation checks")]
        [MenuItem("Physics Sim/Three Phase 3D/Run S01 conservation checks")]
        public static void RunFoundationChecks()
        {
            string project = Directory.GetParent(Application.dataPath).FullName;
            var hardware = new RunHardware { os = SystemInfo.operatingSystem, gpu = SystemInfo.graphicsDeviceName,
                vram_bytes = (long)SystemInfo.graphicsMemorySize * 1000000, vram_is_approximate = true,
                driver = SystemInfo.graphicsDeviceVersion, graphics_api = SystemInfo.graphicsDeviceType.ToString(),
                supports_compute = SystemInfo.supportsComputeShaders, supports_async_readback = SystemInfo.supportsAsyncGPUReadback, gpu_executed = false };
            var run = FoundationRun.Execute(project, Path.Combine(project, "ValidationRuns", "local"), typeof(FoundationRun).Assembly.Location,
                "Unity Editor-hosted managed foundation assembly; not a player build", hardware, Application.unityVersion);
            foreach (var check in run.checks)
            {
                string text = (check.passed ? "PASS " : "FAIL ") + check.validationId + " " + check.name + " " + check.detail;
                if (check.passed) Debug.Log(text); else Debug.LogError(text);
            }
            Debug.Log("S01 run bundle: " + run.directory + "\nSource " + run.gitCommit + "; dirty=" + run.dirty);
            if (!run.Passed) throw new InvalidOperationException("Foundation checks failed; the failed bundle was preserved.");
            Debug.Log(run.checks.Length + " foundation checks passed. No continuum fields or phase-change laws were executed.");
        }

        public static void RunFoundationChecksBatch()
        {
            try { RunFoundationChecks(); EditorApplication.Exit(0); }
            catch (Exception ex) { Debug.LogException(ex); EditorApplication.Exit(1); }
        }

        [MenuItem("Physics Sim/Three Phase 3D/Validate latest S01 bundle")]
        public static void ValidateLatestBundle()
        {
            string project = Directory.GetParent(Application.dataPath).FullName;
            string parent = Path.Combine(project, "ValidationRuns", "local");
            string latest = Directory.Exists(parent) ? Directory.GetDirectories(parent, "s01-foundation-*").OrderByDescending(Directory.GetCreationTimeUtc).FirstOrDefault() : null;
            if (latest == null) throw new InvalidOperationException("Run S01 conservation checks first.");
            EvidenceValidator.VerifyBundle(latest, project);
            Debug.Log("PASS integrity, balances and dependency freshness: " + latest);
        }

        [MenuItem("Physics Sim/Three Phase 3D/Validate evidence index")]
        public static void ValidateEvidenceIndex()
        {
            string project = Directory.GetParent(Application.dataPath).FullName;
            EvidenceValidator.VerifyIndex(project); EvidenceValidator.VerifyFunctionContracts(project);
            Debug.Log("PASS evidence identifiers, code symbols, paths and comment contracts. Physical correctness still requires benchmark and review evidence.");
        }

        [MenuItem("Physics Sim/Three Phase 3D/Export pre-run report")]
        public static void ExportPreRunReport()
        {
            var setup = UnityEngine.Object.FindObjectOfType<Beaker3DSetup>();
            if (setup == null) throw new InvalidOperationException("Open the Beaker3D scene first.");
            string project = Directory.GetParent(Application.dataPath).FullName;
            string output = Path.Combine(project, "ValidationRuns", "local", "preflight-" + DateTime.UtcNow.ToString("yyyyMMddTHHmmssfffffffZ"));
            Directory.CreateDirectory(output);
            File.WriteAllBytes(Path.Combine(output, "scenario.json"), RecordJson.Encode(setup.scenario));
            File.WriteAllText(Path.Combine(output, "preflight.txt"), "Unity " + Application.unityVersion + "\n" +
                setup.HardwareDescription + "\n" + setup.scenario.Validate() +
                "\nS00 only: no physical run has been executed.\n", new UTF8Encoding(false));
            Debug.Log("Pre-run report: " + output);
        }
    }
}
