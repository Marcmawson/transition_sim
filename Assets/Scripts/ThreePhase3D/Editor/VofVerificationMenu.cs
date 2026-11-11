using System;
using System.IO;
using System.Linq;
using System.Runtime.Serialization;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;
using PhysicsSim.ThreePhase3D.Rendering;
using PhysicsSim.ThreePhase3D.Verification;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Editor
{
    public static class VofVerificationMenu
    {
        [DataContract]
        private sealed class Report
        {
            [DataMember] public string Device, GraphicsApi, Unity, Error;
            [DataMember] public bool GpuExecuted, CutCellGpuExecuted, Passed;
            [DataMember] public double CandidateRoundoffM3;
            [DataMember] public FlowCheckRow[] Checks;
        }

        [MenuItem("Physics Sim/Three Phase 3D/Run S05 CPU transport checks")]
        public static void RunCpu() => Run(false);

        [MenuItem("Physics Sim/Three Phase 3D/Run S05 GPU geometry and transport checks")]
        public static void RunGpu() => Run(true);

        [MenuItem("Physics Sim/Three Phase 3D/Create S05 droplet transport preview")]
        public static void CreatePreview()
        {
            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo()) return;
            EditorSceneManager.NewScene(NewSceneSetup.DefaultGameObjects, NewSceneMode.Single);
            if (Camera.main != null)
            {
                Camera.main.clearFlags = CameraClearFlags.SolidColor;
                Camera.main.backgroundColor = new Color(.035f, .05f, .08f, 1);
            }
            var root = new GameObject("S05 droplet transport preview");
            Undo.RegisterCreatedObjectUndo(root, "Create S05 droplet preview");
            Undo.AddComponent<VofSlicePreview>(root);
            Selection.activeGameObject = root;
            Debug.Log("Press Play and open Game to see the droplet cross-section and the liquid-volume ledger.");
        }

        private static void Run(bool useGpu)
        {
            string root = Directory.GetParent(Application.dataPath).FullName;
            string output = Path.Combine(root, "ValidationRuns/local", "s05-" + (useGpu ? "gpu" : "cpu") + "-" +
                DateTime.UtcNow.ToString("yyyyMMddTHHmmss") + "-" + Guid.NewGuid().ToString("N"));
            Directory.CreateDirectory(output);
            var report = new Report { Device = SystemInfo.graphicsDeviceName,
                GraphicsApi = SystemInfo.graphicsDeviceType.ToString(), Unity = Application.unityVersion };
            GeometricVof3D active = null;
            VofGpuGeometry gpu = null;
            VofChecks checks = null;
            try
            {
                if (useGpu)
                {
                    var shader = Resources.Load<ComputeShader>("ThreePhase3D/VofGeometry");
                    var pressureShader = Resources.Load<ComputeShader>("ThreePhase3D/SinglePhaseFlow");
                    var cutShader = Resources.Load<ComputeShader>("ThreePhase3D/CutCellVofScalars");
                    checks = new VofChecks((vof, fields, velocity, dt) =>
                    {
                        if (active != vof)
                        {
                            if (gpu != null)
                            {
                                report.CandidateRoundoffM3 += gpu.CandidateRoundoffM3;
                                gpu.Dispose();
                                gpu = null;
                            }
                            gpu = new VofGpuGeometry(vof, shader);
                            active = vof;
                        }
                        var result = gpu.Advance(fields, velocity, dt);
                        report.GpuExecuted = true;
                        return result;
                    }, (plan, tentative, dt, cap) => FlowGpuSolver.Project(plan,
                        pressureShader, tentative, dt, cap), (plan, state, velocity, dt, inflow) =>
                    {
                        using (var cut = new CutCellGpuScalars(plan, cutShader))
                        {
                            var result = cut.Advance(state, velocity, dt, inflow);
                            report.CutCellGpuExecuted = true;
                            return result;
                        }
                    });
                }
                else checks = new VofChecks();
                checks.Run();
                report.Passed = checks.Passed && (!useGpu || (report.GpuExecuted && report.CutCellGpuExecuted));
                if (!report.Passed) throw new InvalidOperationException("S05 transport checks failed.");
            }
            catch (Exception e) { report.Error = e.ToString(); throw; }
            finally
            {
                if (gpu != null) { report.CandidateRoundoffM3 += gpu.CandidateRoundoffM3; gpu.Dispose(); }
                report.Checks = checks?.Rows.ToArray() ?? Array.Empty<FlowCheckRow>();
                string file = Path.Combine(output, "transport-results.json");
                File.WriteAllBytes(file, RecordJson.Encode(report));
                File.WriteAllText(Path.Combine(output, "files.sha256"), RecordJson.FileHash(file) + "  transport-results.json\n");
                Debug.Log($"S05 evidence: {output}; gpuExecuted={report.GpuExecuted}; cutCellGpuExecuted={report.CutCellGpuExecuted}; passed={report.Passed}");
            }
        }
    }
}
