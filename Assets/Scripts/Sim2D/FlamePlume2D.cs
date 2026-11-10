using UnityEngine;

namespace Seb.Fluid2D.Simulation
{
    /// <summary>
    /// GPU particle plume for the Bunsen flame. Particles are advected by buoyancy,
    /// turbulent lateral flow, and entrainment; they are continuously emitted at the nozzle.
    /// </summary>
    public sealed class FlamePlume2D : MonoBehaviour
    {
        public ThermalLoopLayout2D thermalLoop;
        public FluidSim2D fluidSim;
        [Range(512, 8192)] public int particleCount = 4096;
        [Range(0.08f, 0.7f)] public float particleSize = 0.34f;

        ComputeShader plumeCompute;
        ComputeBuffer flameState;
        ComputeBuffer flameVelocity;
        ComputeBuffer drawArgs;
        Material material;
        Mesh quad;
        int kernel;
        Bounds drawBounds;

        void Start()
        {
            plumeCompute = Resources.Load<ComputeShader>("FlamePlume");
            Shader flameShader = Shader.Find("Instanced/FlameParticle2D");
            if (plumeCompute == null || flameShader == null)
            {
                Debug.LogError("Flame plume assets are missing. Check Assets/Resources/FlamePlume.compute and the FlameParticle2D shader.");
                enabled = false;
                return;
            }

            kernel = plumeCompute.FindKernel("SimulateFlame");
            flameState = new ComputeBuffer(particleCount, sizeof(float) * 4);
            flameVelocity = new ComputeBuffer(particleCount, sizeof(float) * 4);
            drawArgs = new ComputeBuffer(1, sizeof(uint) * 5, ComputeBufferType.IndirectArguments);

            Vector4[] states = new Vector4[particleCount];
            Vector4[] velocities = new Vector4[particleCount];
            for (int i = 0; i < particleCount; i++)
            {
                states[i] = new Vector4(1000, 1000, 2, 1); // respawn on first dispatch
                velocities[i] = new Vector4(0, 0, i * 0.6180339f, 0);
            }
            flameState.SetData(states);
            flameVelocity.SetData(velocities);

            quad = CreateQuad();
            material = new Material(flameShader) { name = "Bunsen Flame Fluid Particles" };
            material.SetBuffer("FlameState", flameState);
            material.SetFloat("particleSize", particleSize);
            uint[] args = { quad.GetIndexCount(0), (uint)particleCount, quad.GetIndexStart(0), quad.GetBaseVertex(0), 0 };
            drawArgs.SetData(args);
        }

        void LateUpdate()
        {
            if (plumeCompute == null || thermalLoop == null || flameState == null) return;

            Vector2 tankCentre = fluidSim != null ? fluidSim.boundsCentre : thermalLoop.heatedTankCentre;
            Vector2 tankSize = fluidSim != null ? fluidSim.boundsSize : thermalLoop.heatedTankSize;
            Vector2 origin = new Vector2(tankCentre.x + thermalLoop.burnerLocalX, tankCentre.y - tankSize.y * 0.5f - 2.7f);

            plumeCompute.SetInt("particleCount", particleCount);
            plumeCompute.SetFloat("deltaTime", Mathf.Min(Time.deltaTime, 1f / 30f));
            plumeCompute.SetFloat("timeNow", Time.time);
            plumeCompute.SetVector("plumeOrigin", new Vector4(origin.x, origin.y, 0, 0));
            plumeCompute.SetInt("burnerEnabled", thermalLoop.burnerEnabled ? 1 : 0);
            plumeCompute.SetFloat("airCollar", thermalLoop.airCollarOpening);
            plumeCompute.SetFloat("burnerPower", thermalLoop.burnerPower);
            plumeCompute.SetBuffer(kernel, "FlameState", flameState);
            plumeCompute.SetBuffer(kernel, "FlameVelocity", flameVelocity);
            plumeCompute.Dispatch(kernel, Mathf.CeilToInt(particleCount / 64f), 1, 1);

            material.SetBuffer("FlameState", flameState);
            material.SetFloat("airCollar", thermalLoop.airCollarOpening);
            material.SetFloat("particleSize", particleSize);
            material.SetFloat("burnerEnabled", thermalLoop.burnerEnabled ? 1 : 0);
            drawBounds = new Bounds(new Vector3(tankCentre.x, tankCentre.y, 0), new Vector3(14, tankSize.y + 12, 8));
            Graphics.DrawMeshInstancedIndirect(quad, 0, material, drawBounds, drawArgs);
        }

        static Mesh CreateQuad()
        {
            Mesh mesh = new Mesh { name = "Flame fluid billboard" };
            mesh.vertices = new[]
            {
                new Vector3(-0.5f, -0.5f), new Vector3(0.5f, -0.5f),
                new Vector3(-0.5f, 0.5f), new Vector3(0.5f, 0.5f)
            };
            mesh.uv = new[] { new Vector2(0, 0), new Vector2(1, 0), new Vector2(0, 1), new Vector2(1, 1) };
            mesh.triangles = new[] { 0, 2, 1, 2, 3, 1 };
            mesh.bounds = new Bounds(Vector3.zero, Vector3.one * 10000);
            return mesh;
        }

        void OnDestroy()
        {
            flameState?.Release();
            flameVelocity?.Release();
            drawArgs?.Release();
            if (material != null) Destroy(material);
            if (quad != null) Destroy(quad);
        }
    }
}
