using System;
using System.IO;
using System.Linq;
using System.Runtime.Serialization;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;
using PhysicsSim.ThreePhase3D.Rendering;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Editor
{
    public static class BeakerVofMenu
    {
        [MenuItem("Physics Sim/Three Phase 3D/Create S05 coupled beaker preview")]
        public static void CreatePreview()
        {
            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo()) return;
            EditorSceneManager.NewScene(NewSceneSetup.DefaultGameObjects,NewSceneMode.Single);
            var root = new GameObject("S05 coupled beaker");
            Undo.RegisterCreatedObjectUndo(root,"Create S05 coupled beaker");
            var setup = Undo.AddComponent<Beaker3DSetup>(root);
            setup.scenario.physical.geometry = BeakerVofTimeIntegration.GeometryInputsForPreview();
            setup.scenario.display.cylinderSegments = BeakerVofTimeIntegration.Segments;
            setup.scenario.display.showSetupReport = false;
            setup.Refresh();
            Undo.AddComponent<BeakerVofPreview>(root);
            var camera = Camera.main;
            if (camera != null)
            {
                var target = new Vector3(.045f,.12f,0);
                camera.transform.position = target + new Vector3(.24f,.15f,-.32f);
                camera.transform.LookAt(target);
                camera.orthographic = true;
                camera.orthographicSize = .17f;
                camera.nearClipPlane = .01f;
                camera.farClipPlane = 2;
                camera.clearFlags = CameraClearFlags.SolidColor;
                camera.backgroundColor = new Color(.035f,.05f,.08f,1);
            }
            Selection.activeGameObject = root;
            Debug.Log("Press Play and open Game for the coupled beaker and liquid-volume slice.");
        }

        [DataContract]
        private sealed class Evidence
        {
            [DataMember] public bool Passed;
            [DataMember] public int Steps, ClippingDispatches;
            [DataMember] public double LiquidDriftM3, LiquidRedistributionM3, MaxDualMassErrorKg, MaxMomentumErrorKgMPerS;
            [DataMember] public string Device, GraphicsApi, Error;
        }

        [MenuItem("Physics Sim/Three Phase 3D/Run S05 coupled beaker checks")]
        public static void RunChecks()
        {
            var report = new Evidence { Device = SystemInfo.graphicsDeviceName, GraphicsApi = SystemInfo.graphicsDeviceType.ToString() };
            string folder = Path.Combine(Directory.GetParent(Application.dataPath).FullName, "ValidationRuns/local",
                "s05-beaker-" + DateTime.UtcNow.ToString("yyyyMMddTHHmmss") + "-" + Guid.NewGuid().ToString("N"));
            Directory.CreateDirectory(folder);
            try
            {
                using (var session = new BeakerVofGpuSession())
                {
                    var initial = (double[])session.Simulation.State.LiquidM3.Clone();
                    for (int i = 0; i < 8; i++)
                    {
                        session.Step();
                        var sim = session.Simulation;
                        report.Steps = sim.Steps;
                        report.ClippingDispatches = session.ClippingDispatches;
                        report.LiquidDriftM3 = sim.LiquidDriftM3;
                        report.LiquidRedistributionM3 = sim.State.LiquidM3.Zip(initial,(a,b) => Math.Abs(a-b)).Sum();
                        report.MaxDualMassErrorKg = sim.MaxDualMassErrorKg;
                        report.MaxMomentumErrorKgMPerS = sim.MaxMomentumLedgerErrorKgMPerS;
                    }
                }
                report.Passed = report.Steps == 8 && report.ClippingDispatches == report.Steps && report.LiquidRedistributionM3 > 1e-9;
                if (!report.Passed) throw new InvalidOperationException("Expected GPU clipping in each beaker step and a nonzero redistribution of liquid.");
            }
            catch (Exception exception) { report.Error = exception.ToString(); throw; }
            finally
            {
                File.WriteAllBytes(Path.Combine(folder,"beaker-results.json"),RecordJson.Encode(report));
                Debug.Log($"S05 coupled beaker evidence: {folder}; steps={report.Steps}; clippingDispatches={report.ClippingDispatches}; passed={report.Passed}");
            }
        }
    }
}
