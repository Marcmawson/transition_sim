using System;
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
}
