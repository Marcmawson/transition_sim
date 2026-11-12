using System;
using PhysicsSim.ThreePhase3D.Rendering;
using UnityEditor;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Editor
{
    public static class IceCouplingVerificationMenu
    {
        [MenuItem("Physics Sim/Three Phase 3D/Run S11 GPU submerged ice check")]
        public static void Run()
        {
            try
            {
                IceCouplingGpuCheck.Run((message, progress) =>
                    EditorUtility.DisplayCancelableProgressBar("S11 GPU submerged ice", message, progress));
            }
            catch (OperationCanceledException)
            {
                Debug.Log("S11 GPU submerged ice check cancelled.");
            }
            finally
            {
                EditorUtility.ClearProgressBar();
            }
        }
    }
}
