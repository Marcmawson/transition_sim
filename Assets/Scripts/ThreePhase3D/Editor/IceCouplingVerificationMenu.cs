using System;
using PhysicsSim.ThreePhase3D.Rendering;
using UnityEditor;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Editor
{
    public static class IceCouplingVerificationMenu
    {
        [MenuItem("Physics Sim/Three Phase 3D/Run S11 GPU submerged ice check")]
        public static void Run() => RunWithProgress("S11 GPU submerged ice", IceCouplingGpuCheck.Run);

        [MenuItem("Physics Sim/Three Phase 3D/Run S11 GPU two-phase ice check")]
        public static void RunTwoPhase() => RunWithProgress("S11 GPU two-phase ice", IceCouplingGpuCheck.RunTwoPhase);

        [MenuItem("Physics Sim/Three Phase 3D/Run S11 GPU beaker impact check")]
        public static void RunImpact()
        {
            if (Application.isBatchMode) IceCouplingGpuCheck.RunImpact(null);
            else RunWithProgress("S11 GPU beaker impact", IceCouplingGpuCheck.RunImpact);
        }

        private static void RunWithProgress(string title, Action<Func<string, float, bool>> check)
        {
            try
            {
                check((message, progress) =>
                    EditorUtility.DisplayCancelableProgressBar(title, message, progress));
            }
            catch (OperationCanceledException)
            {
                Debug.Log($"{title} check cancelled.");
            }
            finally
            {
                EditorUtility.ClearProgressBar();
            }
        }
    }
}
