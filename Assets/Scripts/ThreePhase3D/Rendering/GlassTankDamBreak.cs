using System.Collections.Generic;
using Seb.Fluid.Rendering;
using Seb.Fluid.Simulation;
using UnityEngine;
using UnityEngine.Rendering;

namespace PhysicsSim.ThreePhase3D.Rendering
{
    public sealed class GlassTankDamBreak : MonoBehaviour
    {
        private static readonly Vector3 TankSize = new Vector3(1.2f, .5f, .4f);
        private static readonly Vector3 ColumnSize = new Vector3(.2f, .3f, .4f);
        private const float StepSeconds = .0004f;
        private const float VolumeLitres = 24f;
        private const float ArrivalPlane = .55f;

        [SerializeField] private ComputeShader fluidCompute;
        [SerializeField] private ComputeShader surfaceArguments;
        [SerializeField] private Shader waterShader;

        private readonly List<Material> materials = new List<Material>();
        private GameObject solverObject;
        private FluidSim fluid;
        private ComputeShader solverCompute;
        private bool fine, running, sampling;
        private float pendingSeconds, nextSample, arrival = -1, retained = -1, submitMs;
        private int generation;

        private void Start()
        {
            var sunlight = new GameObject("Tank daylight").AddComponent<Light>();
            sunlight.transform.SetParent(transform);
            sunlight.transform.rotation = Quaternion.Euler(48, -30, 0);
            sunlight.type = LightType.Directional;
            sunlight.intensity = 1.25f;
            sunlight.shadows = LightShadows.Soft;

            var stone = MakeMaterial(new Color(.36f, .38f, .38f));
            var glass = MakeMaterial(new Color(.46f, .78f, .85f, .12f));
            glass.SetOverrideTag("RenderType", "Transparent");
            glass.SetInt("_SrcBlend", (int)BlendMode.SrcAlpha);
            glass.SetInt("_DstBlend", (int)BlendMode.OneMinusSrcAlpha);
            glass.SetInt("_ZWrite", 0);
            glass.EnableKeyword("_ALPHABLEND_ON");
            glass.renderQueue = (int)RenderQueue.Transparent;
            Box("Tank base", new Vector3(0, -.025f, 0), new Vector3(1.25f, .05f, .45f), stone);
            Box("Bench", new Vector3(0, -.09f, 0), new Vector3(2.5f, .08f, 1.3f), stone);
            for (int sign = -1; sign <= 1; sign += 2)
            {
                Box("Long glass wall", new Vector3(0, .25f, sign * .204f),
                    new Vector3(1.216f, .5f, .008f), glass);
                Box("End glass wall", new Vector3(sign * .604f, .25f, 0),
                    new Vector3(.008f, .5f, .4f), glass);
            }
            if (SystemInfo.supportsComputeShaders && fluidCompute != null && surfaceArguments != null &&
                waterShader != null && waterShader.isSupported)
                PrepareColumn();
        }

        private Material MakeMaterial(Color colour)
        {
            var material = new Material(Shader.Find("Standard")) { color = colour };
            materials.Add(material);
            return material;
        }

        private void Box(string label, Vector3 position, Vector3 size, Material material)
        {
            var panel = GameObject.CreatePrimitive(PrimitiveType.Cube);
            panel.name = label;
            panel.transform.SetParent(transform);
            panel.transform.position = position;
            panel.transform.localScale = size;
            var renderer = panel.GetComponent<MeshRenderer>();
            renderer.sharedMaterial = material;
            if (material.renderQueue == (int)RenderQueue.Transparent)
                renderer.shadowCastingMode = ShadowCastingMode.Off;
            Destroy(panel.GetComponent<Collider>());
        }

        private void PrepareColumn()
        {
            generation++;
            running = sampling = false;
            pendingSeconds = nextSample = submitMs = 0;
            arrival = retained = -1;
            if (solverObject != null)
            {
                solverObject.SetActive(false);
                Destroy(solverObject);
                Destroy(solverCompute);
            }

            solverObject = new GameObject("GPU tank water");
            solverObject.SetActive(false);
            solverObject.transform.SetParent(transform);
            solverObject.transform.position = Vector3.up * (TankSize.y * .5f);
            solverObject.transform.localScale = TankSize;
            var spawner = solverObject.AddComponent<Spawner3D>();
            spawner.particleSpawnDensity = fine ? 3375000 : 1000000;
            spawner.spawnRegions = new[] { new Spawner3D.SpawnRegion
            {
                centre = new Vector3(-.5f, .15f, 0),
                boxSize = ColumnSize
            }};
            float spacing = fine ? 1f / 150f : .01f;
            float radius = 2 * spacing;
            solverCompute = Instantiate(fluidCompute);
            fluid = solverObject.AddComponent<FluidSim>();
            fluid.compute = solverCompute;
            fluid.spawner = spawner;
            fluid.manualControl = true;
            fluid.gravity = -9.81f;
            fluid.smoothingRadius = radius;
            fluid.targetDensity = InteriorDensity(spacing, radius);
            fluid.pressureMultiplier = 100f;
            fluid.nearPressureMultiplier = 1f;
            fluid.viscosityStrength = 2e-8f * Mathf.Pow(spacing / .001f, 3);
            fluid.collisionDamping = .05f;
            fluid.maxFoamParticleCount = 1;
            fluid.renderToTex3D = fluid.worldSpaceSurface = true;
            fluid.densityTextureRes = fine ? 192 : 128;

            var surface = solverObject.AddComponent<MarchingCubesFluidRenderer>();
            surface.sim = fluid;
            surface.drawShader = waterShader;
            surface.renderArgsCompute = Instantiate(surfaceArguments);
            surface.worldSpaceSurface = true;
            surface.col = new Color(1, 1, 1, .12f);
            surface.isoLevel = fluid.targetDensity * .45f;
            solverObject.SetActive(true);
        }

        private static float InteriorDensity(float spacing, float radius)
        {
            float sum = 0;
            for (int x = -2; x <= 2; x++)
            for (int y = -2; y <= 2; y++)
            for (int z = -2; z <= 2; z++)
            {
                float support = Mathf.Max(0, radius - new Vector3(x, y, z).magnitude * spacing);
                sum += support * support;
            }
            return sum * 15 / (2 * Mathf.PI * Mathf.Pow(radius, 5));
        }

        private void Update()
        {
            if (!running || fluid.positionBuffer == null) return;
            pendingSeconds = Mathf.Min(pendingSeconds + Time.unscaledDeltaTime * .25f, .008f);
            int steps = Mathf.Min(12, Mathf.FloorToInt(pendingSeconds / StepSeconds));
            if (steps == 0) return;

            double start = Time.realtimeSinceStartupAsDouble;
            fluid.AdvanceManually(StepSeconds, steps);
            submitMs = (float)((Time.realtimeSinceStartupAsDouble - start) * 1000 / steps);
            pendingSeconds -= steps * StepSeconds;
            if (!sampling && SystemInfo.supportsAsyncGPUReadback && fluid.SimulatedSeconds >= nextSample)
                SampleWater();
        }

        private void SampleWater()
        {
            sampling = true;
            int run = generation;
            float simulatedSeconds = fluid.SimulatedSeconds;
            nextSample = simulatedSeconds + .05f;
            AsyncGPUReadback.Request(fluid.positionBuffer, request =>
            {
                if (this == null || run != generation) return;
                sampling = false;
                if (request.hasError) return;
                var points = request.GetData<Vector3>();
                int inside = 0, atEnd = 0;
                foreach (Vector3 point in points)
                {
                    if (!(Mathf.Abs(point.x) <= .601f && Mathf.Abs(point.z) <= .201f &&
                        point.y >= -.001f && point.y <= .501f)) continue;
                    inside++;
                    if (point.x >= ArrivalPlane && point.y <= .05f) atEnd++;
                }
                retained = VolumeLitres * inside / points.Length;
                if (arrival < 0 && atEnd >= Mathf.Max(3, points.Length / 1000))
                {
                    arrival = simulatedSeconds;
                    Debug.Log($"Tank dam break · {points.Length:N0} particles · arrival {arrival:F3} s · retained {retained:F2} L", this);
                }
            });
        }

        private void OnGUI()
        {
            GUILayout.BeginArea(new Rect(16, 16, 340, 230), GUI.skin.box);
            GUILayout.Label("Glass Tank Dam Break · 24 L");
            if (fluid == null)
            {
                GUILayout.Label("GPU water assets or compute support unavailable.");
            }
            else
            {
                bool ready = fluid.positionBuffer != null;
                bool previousEnabled = GUI.enabled;
                GUI.enabled = previousEnabled && ready;
                if (GUILayout.Button(running ? "Pause" : fluid.SimulatedSeconds > 0 ? "Resume" : "Release"))
                {
                    running = !running;
                    pendingSeconds = 0;
                }
                GUI.enabled = previousEnabled && ready && !sampling;
                if (GUILayout.Button("Reset column")) PrepareColumn();
                bool selectedFine = GUILayout.Toggle(fine, "Fine resolution (81,000 particles)");
                if (selectedFine != fine)
                {
                    fine = selectedFine;
                    PrepareColumn();
                }
                GUI.enabled = previousEnabled;
                GUILayout.Label($"{(fluid.positionBuffer?.count ?? 0):N0} particles · {fluid.SimulatedSeconds:F3} s simulated");
                GUILayout.Label(arrival < 0 ? "Wave arrival pending" : $"Wave arrival at x = 0.55 m · {arrival:F3} s");
                GUILayout.Label(!SystemInfo.supportsAsyncGPUReadback ? "Volume readback unavailable" :
                    retained < 0 ? "Retained volume pending" : $"Retained volume · {retained:F2} L");
                GUILayout.Label($"CPU submission per step · {submitMs:F3} ms");
            }
            GUILayout.EndArea();
        }

        private void OnDestroy()
        {
            if (solverCompute != null) Destroy(solverCompute);
            foreach (var material in materials) Destroy(material);
        }
    }
}
