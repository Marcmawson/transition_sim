using System;
using PhysicsSim.ThreePhase3D.Flow;
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
    }
}
