using System;
using System.Collections.Generic;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;
using PhysicsSim.ThreePhase3D.Verification;
using UnityEditor;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Editor
{
    public static class PureWaterBoilingVerificationMenu
    {
        [MenuItem("Physics Sim/Three Phase 3D/Run S13 GPU boiling checks")]
        public static void Run()
        {
            var shader = Resources.Load<ComputeShader>("ThreePhase3D/SinglePhaseFlow");
            if (!SystemInfo.supportsComputeShaders || shader == null)
                throw new NotSupportedException("The S13 boiling checks require the GPU pressure solver.");

            PureWaterBoilingCheck.RunPlanarStefan(Project, Log);
            PureWaterBoilingCheck.RunBubbleRise(Project, Log, PureWaterBoilingCheck.BubbleCase.Evaporation);
            PureWaterBoilingCheck.RunBubbleRise(Project, Log, PureWaterBoilingCheck.BubbleCase.Condensation);

            FlowStep Project(VariableCoefficientProjection plan, float[] velocity,
                double dt, int iterations, double[] source) =>
                FlowGpuSolver.Project(plan, shader, velocity, dt, iterations, boundaryFlux: source);
            void Log(string message) => Debug.Log("GPU pressure / CPU transport · " + message);
        }
    }
    public sealed class SeededBoilingWindow : EditorWindow
    {
        private readonly List<(int Step, Grid3D Grid, double[] Vapour, double Volume, double Height)> frames =
            new List<(int, Grid3D, double[], double, double)>();
        private int frame;
        private string status;

        [MenuItem("Physics Sim/Three Phase 3D/Open seeded boiling preview")]
        private static void Open() => GetWindow<SeededBoilingWindow>("Seeded Boiling");

        private void Run()
        {
            frames.Clear();
            frame = 0;
            try
            {
                var shader = Resources.Load<ComputeShader>("ThreePhase3D/SinglePhaseFlow");
                if (!SystemInfo.supportsComputeShaders || shader == null)
                    throw new NotSupportedException("GPU pressure solver unavailable.");
                PureWaterBoilingCheck.RunBubbleRise(
                    (plan, velocity, dt, iterations, source) =>
                        FlowGpuSolver.Project(plan, shader, velocity, dt, iterations, boundaryFlux: source),
                    message => { status = message; Debug.Log(message); },
                    PureWaterBoilingCheck.BubbleCase.Evaporation,
                    (step, grid, vapour, volume, height) => frames.Add((step, grid, vapour, volume, height)));
            }
            catch (Exception error)
            {
                frames.Clear();
                status = error.Message;
                Debug.LogException(error);
            }
        }

        private void OnGUI()
        {
            EditorGUILayout.LabelField("S13 seeded bubble · GPU pressure / CPU phase transport");
            if (GUILayout.Button("Run seeded boiling test")) Run();
            if (frames.Count == 0)
            {
                if (status != null) EditorGUILayout.HelpBox(status, MessageType.Error);
                return;
            }
            frame = EditorGUILayout.IntSlider("Solver step", frame, 0, frames.Count - 1);
            var sample = frames[frame];
            EditorGUILayout.LabelField($"Time {sample.Step * .4:F1} ms");
            EditorGUILayout.LabelField($"Vapour volume {sample.Volume * 1e9:F4} mm³");
            EditorGUILayout.LabelField($"Centre rise {(sample.Height - frames[0].Height) * 1e6:F2} µm");
            EditorGUILayout.LabelField("Central vertical slice · dark = liquid, pale = vapour");
            var rect = GUILayoutUtility.GetAspectRect(1, GUILayout.MaxWidth(320));
            if (Event.current.type != EventType.Repaint) return;
            int z = sample.Grid.Cells.Z / 2;
            float dx = rect.width / sample.Grid.Cells.X, dy = rect.height / sample.Grid.Cells.Y;
            for (int y = 0; y < sample.Grid.Cells.Y; y++)
            for (int x = 0; x < sample.Grid.Cells.X; x++)
            {
                double fraction = sample.Vapour[sample.Grid.Cells.Index(x, y, z)];
                EditorGUI.DrawRect(new Rect(rect.x + x * dx, rect.yMax - (y + 1) * dy, dx, dy),
                    Color.Lerp(new Color(.06f, .17f, .33f), new Color(.84f, .97f, 1f), (float)fraction));
            }
        }
    }
}
