using System;
using System.Collections.Generic;
using System.IO;
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
    public static class FlowVerificationMenu
    {
        [DataContract]
        private sealed class GpuStep
        {
            [DataMember] public int Sequence, PressureIterations;
            [DataMember] public bool Accepted;
            [DataMember] public double DtS, ResidualL2, RhsL2, MaxDivergencePerS;
            [DataMember] public long AllocatedBytes;
            [DataMember] public double PredictionWallMs, IterationWallMs, CorrectionWallMs;
            [DataMember] public double MaxCpuVelocityDifferenceMPerS, MaxCpuPressureDifferencePa;
        }

        [DataContract]
        private sealed class Report
        {
            [DataMember] public string Schema = "S04-flow-1";
            [DataMember] public string Utc, Unity, Device, GraphicsApi, Os, ShaderSha256, CaseSourceSha256, Error;
            [DataMember] public int GraphicsMemoryMiB;
            [DataMember] public bool GpuExecuted, Passed;
            [DataMember] public FlowCheckRow[] Checks;
            [DataMember] public GpuStep[] GpuSteps;
        }

        [MenuItem("Physics Sim/Three Phase 3D/Run S04 CPU flow checks")]
        public static void RunCpu() => Run(false);

        [MenuItem("Physics Sim/Three Phase 3D/Run S04 GPU flow checks")]
        public static void RunGpu() => Run(true);

        [MenuItem("Physics Sim/Three Phase 3D/Create S04 channel flow preview")]
        public static void CreatePreview()
        {
            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo()) return;
            EditorSceneManager.NewScene(NewSceneSetup.DefaultGameObjects, NewSceneMode.Single);
            var root = new GameObject("S04 single-phase channel");
            Undo.RegisterCreatedObjectUndo(root, "Create S04 channel preview");
            Undo.AddComponent<FlowChannelPreview>(root);
            Selection.activeGameObject = root;
            SceneView.lastActiveSceneView?.LookAt(new Vector3(.01f, .01f, .0075f),
                Quaternion.Euler(15, -22, 0), .04f, true);
            Debug.Log("Enter Play mode and enable Scene view Gizmos to see velocity arrows. This is the pressure-driven channel benchmark.");
        }

        public static void RunGpuBatch()
        {
            try { RunGpu(); EditorApplication.Exit(0); }
            catch (Exception failure) { Debug.LogException(failure); EditorApplication.Exit(1); }
        }

        private static void Run(bool gpu)
        {
            string root = Directory.GetParent(Application.dataPath).FullName;
            string output = Path.Combine(root, "ValidationRuns/local",
                "s04-" + (gpu ? "gpu" : "cpu") + "-" + DateTime.UtcNow.ToString("yyyyMMddTHHmmss") + "-" + Guid.NewGuid().ToString("N"));
            Directory.CreateDirectory(output);
            var report = new Report
            {
                Utc = DateTime.UtcNow.ToString("o"),
                Unity = Application.unityVersion,
                Device = SystemInfo.graphicsDeviceName,
                GraphicsApi = SystemInfo.graphicsDeviceType.ToString(),
                GraphicsMemoryMiB = SystemInfo.graphicsMemorySize,
                Os = SystemInfo.operatingSystem,
                CaseSourceSha256 = RecordJson.FileHash(Path.Combine(root, "Assets/Scripts/ThreePhase3D/Verification/FlowChecks.cs"))
            };
            var steps = new List<GpuStep>();
            FlowChecks suite = null;
            FlowGpuSolver solver = null;
            MomentumPredictor3D activePredictor = null;
            try
            {
                if (gpu)
                {
                    var shader = Resources.Load<ComputeShader>("ThreePhase3D/SinglePhaseFlow");
                    if (shader == null) throw new InvalidOperationException("SinglePhaseFlow.compute failed to import/load.");
                    report.ShaderSha256 = RecordJson.FileHash(Path.Combine(root, "Assets/Resources/ThreePhase3D/SinglePhaseFlow.compute"));
                    suite = new FlowChecks((predictor, velocity, acceleration, dt, cap) =>
                    {
                        if (activePredictor != predictor)
                        {
                            solver?.Dispose();
                            solver = new FlowGpuSolver(predictor, shader);
                            activePredictor = predictor;
                        }
                        var entry = new GpuStep { Sequence = steps.Count + 1, AllocatedBytes = solver.AllocatedBytes };
                        steps.Add(entry);
                        double predictionBefore = solver.PredictionWallMs, iterationBefore = solver.IterationWallMs,
                            correctionBefore = solver.CorrectionWallMs;
                        try
                        {
                            var actual = solver.Advance(velocity, acceleration, dt, cap);
                            report.GpuExecuted = true;
                            entry.Accepted = true;
                            SaveDiagnostics(entry, actual.Diagnostics);
                            var expected = FlowCpuReference.Advance(predictor, velocity, acceleration, dt, cap);
                            for (int face = 0; face < actual.VelocityMPerS.Length; face++)
                                entry.MaxCpuVelocityDifferenceMPerS = Math.Max(entry.MaxCpuVelocityDifferenceMPerS,
                                    Math.Abs((double)actual.VelocityMPerS[face] - expected.VelocityMPerS[face]));
                            for (int group = 0; group < actual.GroupPressurePa.Length; group++)
                                entry.MaxCpuPressureDifferencePa = Math.Max(entry.MaxCpuPressureDifferencePa,
                                    Math.Abs(actual.GroupPressurePa[group] - expected.GroupPressurePa[group]));
                            if (entry.MaxCpuVelocityDifferenceMPerS > 2e-6 || entry.MaxCpuPressureDifferencePa > .04)
                                throw new InvalidOperationException("GPU fields differ from the CPU reference beyond the benchmark tolerances.");
                            return actual;
                        }
                        catch (FlowConvergenceException failure)
                        {
                            report.GpuExecuted = true;
                            SaveDiagnostics(entry, failure.Diagnostics);
                            throw;
                        }
                        finally
                        {
                            entry.PredictionWallMs = solver.PredictionWallMs - predictionBefore;
                            entry.IterationWallMs = solver.IterationWallMs - iterationBefore;
                            entry.CorrectionWallMs = solver.CorrectionWallMs - correctionBefore;
                        }
                    });
                }
                else suite = new FlowChecks(FlowCpuReference.Advance);

                suite.Run();
                report.Passed = suite.Passed && (!gpu || report.GpuExecuted);
                if (!report.Passed) throw new InvalidOperationException("S04 flow checks failed.");
            }
            catch (Exception failure)
            {
                report.Error = failure.ToString();
                throw;
            }
            finally
            {
                solver?.Dispose();
                report.Checks = suite?.Rows.ToArray() ?? Array.Empty<FlowCheckRow>();
                report.GpuSteps = steps.ToArray();
                string results = Path.Combine(output, "flow-results.json");
                File.WriteAllBytes(results, RecordJson.Encode(report));
                File.WriteAllText(Path.Combine(output, "files.sha256"), RecordJson.FileHash(results) + "  flow-results.json\n");
                Debug.Log("S04 flow evidence: " + output + "; gpuExecuted=" + report.GpuExecuted + "; passed=" + report.Passed);
            }
        }

        private static void SaveDiagnostics(GpuStep entry, FlowDiagnostics diagnostics)
        {
            entry.PressureIterations = diagnostics.Iterations;
            entry.DtS = diagnostics.Dt;
            entry.ResidualL2 = diagnostics.ResidualL2;
            entry.RhsL2 = diagnostics.RhsL2;
            entry.MaxDivergencePerS = diagnostics.MaxDivergencePerS;
        }
    }
}
