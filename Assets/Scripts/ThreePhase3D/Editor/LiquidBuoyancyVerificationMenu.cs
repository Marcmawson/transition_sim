using PhysicsSim.ThreePhase3D.Rendering;
using UnityEditor;

namespace PhysicsSim.ThreePhase3D.Editor
{
    public static class LiquidBuoyancyVerificationMenu
    {
        [MenuItem("Physics Sim/Three Phase 3D/Run S09 GPU buoyancy check")]
        public static void Run() => LiquidBuoyancyGpuCheck.Run();
    }
}
