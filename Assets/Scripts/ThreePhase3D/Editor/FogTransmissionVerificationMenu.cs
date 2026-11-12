using PhysicsSim.ThreePhase3D.Rendering;
using UnityEditor;

namespace PhysicsSim.ThreePhase3D.Editor
{
    public static class FogTransmissionVerificationMenu
    {
        [MenuItem("Physics Sim/Three Phase 3D/Run S14 GPU fog transmission check")]
        public static void Run() => FogTransmissionGpuCheck.Run();
    }
}
