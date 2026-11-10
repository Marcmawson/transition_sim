using System;
using System.IO;
using System.Text;
using PhysicsSim.ThreePhase3D.Core;
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
        public static void RunFoundationChecks()
        {
            var results = FoundationChecks.RunS00();
            foreach (var check in results)
            {
                string text = (check.passed ? "PASS " : "FAIL ") + check.validationId + " " + check.name + " " + check.detail;
                if (check.passed) Debug.Log(text); else Debug.LogError(text);
            }
            if (results.Exists(check => !check.passed)) throw new InvalidOperationException("Foundation checks failed; see Console.");
            Debug.Log("S00 checks passed. This does not verify a 3D flow solver or Unity scene import on another machine.");
        }

        public static void RunFoundationChecksBatch()
        {
            try { RunFoundationChecks(); EditorApplication.Exit(0); }
            catch (Exception ex) { Debug.LogException(ex); EditorApplication.Exit(1); }
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
