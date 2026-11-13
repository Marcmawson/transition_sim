using System;
using PhysicsSim.ThreePhase3D.Flow;
using PhysicsSim.ThreePhase3D.Rendering;
using PhysicsSim.ThreePhase3D.Verification;
using UnityEditor;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Editor
{
    public static class HumidPhaseExchangeVerificationMenu
    {
        [MenuItem("Physics Sim/Three Phase 3D/Run S12 GPU phase exchange check")]
        public static void Run()
        {
            var shader = Resources.Load<ComputeShader>("ThreePhase3D/SinglePhaseFlow");
            if (!SystemInfo.supportsComputeShaders || shader == null)
                throw new NotSupportedException("The S12 phase exchange check requires the GPU pressure solver.");
            HumidPhaseExchangeCheck.Run((plan, tentative, dt, source) =>
                FlowGpuSolver.Project(plan, shader, tentative, dt, 4096, boundaryFlux: source),
                message => Debug.Log("GPU " + message));
        }
        [MenuItem("Physics Sim/Three Phase 3D/Run S16 GPU pressure beaker benchmark")]
        public static void RunCoupledBeaker()
        {
            var shader = Resources.Load<ComputeShader>("ThreePhase3D/SinglePhaseFlow");
            if (!SystemInfo.supportsComputeShaders || shader == null)
                throw new NotSupportedException("The S16 beaker benchmark requires the GPU pressure solver.");

            HumidPhaseExchangeCheck.RunCoupledBeaker((plan, tentative, dt, source) =>
                FlowGpuSolver.Project(plan, shader, tentative, dt, 32768, boundaryFlux: source),
                message => Debug.Log("GPU pressure / CPU coupled benchmark · " + message));
            Debug.Log("S16 reduced beaker benchmark passed. Full-scene V43 validation remains open.");
        }
    }
}

namespace PhysicsSim.ThreePhase3D.Editor
{
    public sealed class CoupledBeakerWindow : EditorWindow
    {
        private System.Collections.Generic.IEnumerator<HumidPhaseExchangeCheck.BeakerSample> run;
        private HumidPhaseExchangeCheck.BeakerSample sample;
        private string status;
        private bool running;
        private Beaker3DSetup apparatus;

        [MenuItem("Physics Sim/Three Phase 3D/Open S16 beaker diagnostics")]
        private static void Open() => GetWindow<CoupledBeakerWindow>("S16 Beaker");

        private void OnEnable() => SceneView.duringSceneGui += DrawApparatusReadings;

        private void OnDisable()
        {
            SceneView.duringSceneGui -= DrawApparatusReadings;
            Stop();
            SceneView.RepaintAll();
        }

        private void DrawApparatusReadings(SceneView view)
        {
            if (!EditorApplication.isPlaying || apparatus == null || sample.Time <= 0) return;
            var geometry = apparatus.scenario?.physical?.geometry;
            if (geometry == null) return;
            float floor = (float)(geometry.externalBaseHeightM + geometry.baseThicknessM);
            float height = (float)geometry.cavityHeightM;
            float radius = (float)geometry.innerRadiusM;
            var origin = apparatus.transform;
            Label(0, floor + height, $"Reduced benchmark · {sample.Time:F4} s");
            Label(0, floor + height * .66f, $"Headspace {sample.Gas:F3} K\nVapour {sample.Vapour:G5} kg · Dry air {sample.Air:G5} kg");
            Label(0, floor + height * .28f, $"Liquid {sample.Liquid:G5} kg · Ice {sample.Ice:G5} kg");
            Label(radius, floor + height * .1f, $"Inner glass {sample.Wall:F3} K");
            Label(radius, (float)geometry.externalBaseHeightM, $"Outer glass {sample.Back:F3} K · {(sample.Heating ? "Heating" : "Burner off")}");

            void Label(float x, float y, string text) =>
                Handles.Label(origin.TransformPoint(new Vector3(x, y, 0)), text, EditorStyles.whiteLabel);
        }

        private void Stop()
        {
            running = false;
            run?.Dispose();
            run = null;
        }

        private void Update()
        {
            if (!EditorApplication.isPlaying)
            {
                if (run != null) Stop();
                apparatus = null;
                sample = default;
                return;
            }
            if (running && !EditorApplication.isPaused) Advance();
        }

        private void Advance()
        {
            try
            {
                if (run.MoveNext())
                {
                    sample = run.Current;
                    status = sample.Heating ? "Heating" : "Burner off";
                }
                else
                {
                    Stop();
                    status = "Reduced benchmark passed";
                }
            }
            catch (Exception error)
            {
                Stop();
                status = error.Message;
                Debug.LogException(error);
            }
            Repaint();
            SceneView.RepaintAll();
        }

        private void OnGUI()
        {
            EditorGUILayout.LabelField("Reduced beaker · GPU vent pressure");
            if (!EditorApplication.isPlaying)
            {
                EditorGUILayout.HelpBox("Enter Play Mode to run the benchmark.", MessageType.Info);
                return;
            }

            EditorGUI.BeginChangeCheck();
            apparatus = (Beaker3DSetup)EditorGUILayout.ObjectField("Scene beaker", apparatus, typeof(Beaker3DSetup), true);
            if (EditorGUI.EndChangeCheck()) SceneView.RepaintAll();

            if (GUILayout.Button("Start or reset benchmark"))
            {
                Stop();
                sample = default;
                SceneView.RepaintAll();
                var shader = Resources.Load<ComputeShader>("ThreePhase3D/SinglePhaseFlow");
                if (!SystemInfo.supportsComputeShaders || shader == null)
                    status = "GPU pressure solver unavailable";
                else
                {
                    run = HumidPhaseExchangeCheck.StepCoupledBeaker((plan, velocity, dt, source) =>
                        FlowGpuSolver.Project(plan, shader, velocity, dt, 32768, boundaryFlux: source),
                        message => Debug.Log(message)).GetEnumerator();
                    status = "Ready";
                }
            }

            using (new EditorGUI.DisabledScope(run == null))
            {
                if (GUILayout.Button(running ? "Pause benchmark" : "Run benchmark")) running = !running;
                if (GUILayout.Button("Advance one timestep"))
                {
                    running = false;
                    Advance();
                }
            }

            EditorGUILayout.LabelField(status ?? "Ready to initialise");
            if (sample.Time <= 0) return;
            EditorGUILayout.LabelField($"Simulated time {sample.Time:F4} s");
            EditorGUILayout.LabelField($"Glass inner/outer {sample.Wall:F4} / {sample.Back:F4} K");
            EditorGUILayout.LabelField($"Headspace {sample.Gas:F4} K");
            EditorGUILayout.LabelField($"Ice {sample.Ice:G6} kg · Liquid {sample.Liquid:G6} kg");
            EditorGUILayout.LabelField($"Vapour {sample.Vapour:G6} kg · Dry air {sample.Air:G6} kg");
            EditorGUILayout.LabelField($"Melted {sample.Melted:G6} kg · Boiled {sample.Boiled:G6} kg");
            EditorGUILayout.LabelField($"Condensed {sample.Condensed:G6} kg · Room heat {sample.RoomHeat:G6} J");
            EditorGUILayout.LabelField($"Water residual {sample.WaterResidual:E2} kg · Air residual {sample.AirResidual:E2} kg");
            EditorGUILayout.LabelField($"Enthalpy residual {sample.EnthalpyResidual:E2} J");
            EditorGUILayout.LabelField($"Entropy production {sample.Entropy:E2} J/K");
        }
    }
}
