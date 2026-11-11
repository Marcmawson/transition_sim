using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Rendering
{
    [DisallowMultipleComponent]
    public sealed class BeakerSimulationControls : MonoBehaviour
    {
        [Header("Water spawn · next spawn")]
        [Range(4.1f, 19.9f)] [SerializeField] private float waterVolumeMl = 9f;
        [Range(0f, 5f)] [SerializeField] private float spawnHeightJitterMm = 3f;

        [Header("GPU solver · live")]
        [Tooltip("Negative points down.")] [SerializeField] private float gravityY = -9.81f;
        [Tooltip("Upper bound. Cut-cell CFL can reduce the actual timestep.")]
        [Range(.1f, 8f)] [SerializeField] private float maximumStepMs = 8f;
        [Tooltip("Pressure solve iteration cap. The solver still requires convergence.")]
        [Min(31)] [SerializeField] private int pressureIterationLimit = 16384;

        [Header("Water particles · live")]
        [Range(1, 8)] [SerializeField] private int samplesPerCellAxis = 5;
        [Range(.1f, 2f)] [SerializeField] private float particleDiameterMm = .8f;
        [Range(0f, 1f)] [SerializeField] private float particleJitter = .6f;
        [Range(1, 60)] [SerializeField] private int particleRefreshHz = 30;

        internal double WaterVolumeM3 => Mathf.Clamp(waterVolumeMl, 4.1f, 19.9f) * 1e-6;
        internal double SpawnHeightJitterM => Mathf.Clamp(spawnHeightJitterMm, 0f, 5f) * .001;
        internal double GravityY => gravityY;
        internal double MaximumStepS => Mathf.Clamp(maximumStepMs, .1f, 8f) * .001;
        internal int PressureIterationLimit => Mathf.Max(31, pressureIterationLimit);
        internal int SamplesPerCellAxis => Mathf.Clamp(samplesPerCellAxis, 1, 8);
        internal float ParticleDiameterM => Mathf.Clamp(particleDiameterMm, .1f, 2f) * .001f;
        internal float ParticleJitter => Mathf.Clamp01(particleJitter);
        internal double ParticleRefreshSeconds => 1.0 / Mathf.Clamp(particleRefreshHz, 1, 60);
    }
}
