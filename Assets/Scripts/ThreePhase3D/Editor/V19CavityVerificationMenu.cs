using System;
using PhysicsSim.ThreePhase3D.Rendering;
using UnityEditor;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Editor
{
    public static class V19CavityVerificationMenu
    {
        [MenuItem("Physics Sim/Three Phase 3D/Run S09 V19 GPU cavity benchmark")]
        public static void Run()
        {
            try
            {
                V19CavityGpuCheck.Run((message, fraction) =>
                    EditorUtility.DisplayCancelableProgressBar("V19 GPU cavity", message, fraction));
            }
            catch (OperationCanceledException)
            {
                Debug.Log("S09 V19 GPU cavity benchmark cancelled.");
            }
            finally
            {
                EditorUtility.ClearProgressBar();
            }
        }
    }
}
