using PhysicsSim.ThreePhase3D.Rendering;
using UnityEditor;

namespace PhysicsSim.ThreePhase3D.Editor
{
    public static class LiquidEnthalpyVerificationMenu
    {
        [MenuItem("Physics Sim/Three Phase 3D/Run S09 GPU enthalpy check")]
        public static void Run() => LiquidEnthalpyGpuCheck.Run();
    }
}
