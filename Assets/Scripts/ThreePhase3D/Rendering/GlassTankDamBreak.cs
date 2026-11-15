using System.Collections.Generic;
using PhysicsSim.ThreePhase3D.Flow;
using Seb.Fluid.Rendering;
using Seb.Fluid.Simulation;
using UnityEngine;
using UnityEngine.Rendering;

namespace PhysicsSim.ThreePhase3D.Rendering
{
    public sealed class GlassTankDamBreak : MonoBehaviour
    {
        private static readonly Vector3 TankSize = new Vector3(1.2f, .5f, .8f);
        private const float PressureCellSize = .05f;
        private const int FineParticleDensity = 3375000;
        private Vector3 ColumnSize => GetColumnSize(doubleVolume);
        private static Vector3 GetColumnSize(bool doubled) => new Vector3(doubled ? .4f : .2f, .3f, TankSize.z);
        private static readonly int[] ParticleStepMicroseconds = { 400, 800, 1000 };
        private static readonly string[] ParticleStepLabels = { "0.4 ms", "0.8 ms", "1.0 ms" };
        private int particleStepIndex;
        private long elapsedMicroseconds;
        private float VolumeLitres => GetVolumeLitres(ColumnSize);
        private static float GetVolumeLitres(Vector3 size) => size.x * size.y * size.z * 1000f;
        private const float ArrivalPlane = .55f;
        private static readonly Vector3 BaffleSize = new Vector3(.025f, .25f, .275f);
        private const float BaffleAngle = 30f;
        private static readonly Vector3 BaffleHalfSize = BaffleSize * .5f;
        private static readonly float BaffleSin = Mathf.Sin(BaffleAngle * Mathf.Deg2Rad);
        private static readonly float BaffleCos = Mathf.Cos(BaffleAngle * Mathf.Deg2Rad);
        private static readonly float BaffleExtentX = BaffleCos * BaffleHalfSize.x + BaffleSin * BaffleHalfSize.z;
        private static readonly float BaffleExtentZ = BaffleSin * BaffleHalfSize.x + BaffleCos * BaffleHalfSize.z;
        private static readonly float BaffleOffset = TankSize.z * .5f - BaffleExtentZ;
        private static Quaternion BaffleRotation(int index) =>
            Quaternion.Euler(0, index % 2 == 0 ? BaffleAngle : -BaffleAngle, 0);
        private static readonly Vector3[] BaffleCentres =
        {
            new Vector3(-.125f, BaffleSize.y * .5f, -BaffleOffset),
            new Vector3(.125f, BaffleSize.y * .5f, BaffleOffset),
            new Vector3(.375f, BaffleSize.y * .5f, -BaffleOffset)
        };

        [SerializeField] private ComputeShader fluidCompute;
        [SerializeField] private ComputeShader surfaceArguments;
        [SerializeField] private Shader waterShader;
        [SerializeField] private Shader sprayShader;
        [SerializeField] private ComputeShader sprayArguments;
        private bool sprayEnabled = true;
        private bool samplingSpray;
        private int activeSpray = -1;

        private readonly List<Material> materials = new List<Material>();
        private GameObject solverObject;
        private FluidSim fluid;
        private ComputeShader solverCompute;
        private bool doubleVolume = true;
        private bool fine, running, sampling;
        private bool adaptivePressure;
        private int pressureSamples;
        private const int ProjectionIntervalMicroseconds = 20000;
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
            Box("Tank base", new Vector3(0, -.025f, 0), new Vector3(1.25f, .05f, TankSize.z + .05f), stone);
            Box("Bench", new Vector3(0, -.09f, 0), new Vector3(2.5f, .08f, 1.3f), stone);
            var baffle = MakeMaterial(new Color(.24f, .36f, .39f));
            for (int i = 0; i < BaffleCentres.Length; i++)
                Box($"Channel baffle {i + 1}", BaffleCentres[i], BaffleSize, baffle, BaffleRotation(i));
            for (int sign = -1; sign <= 1; sign += 2)
            {
                Box("Long glass wall", new Vector3(0, .25f, sign * (TankSize.z * .5f + .004f)),
                    new Vector3(1.216f, .5f, .008f), glass);
                Box("End glass wall", new Vector3(sign * .604f, .25f, 0),
                    new Vector3(.008f, .5f, TankSize.z), glass);
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

        private void Box(string label, Vector3 position, Vector3 size, Material material, Quaternion? rotation = null)
        {
            var panel = GameObject.CreatePrimitive(PrimitiveType.Cube);
            panel.name = label;
            panel.transform.SetParent(transform);
            panel.transform.SetPositionAndRotation(position, rotation ?? Quaternion.identity);
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
            running = sampling = samplingSpray = false;
            activeSpray = -1;
            pendingSeconds = nextSample = submitMs = 0;
            arrival = retained = -1;
            elapsedMicroseconds = 0;
            pressureSamples = 0;
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
            spawner.particleSpawnDensity = fine ? FineParticleDensity : 1000000;
            Vector3 column = ColumnSize;
            spawner.spawnRegions = new[] { new Spawner3D.SpawnRegion
            {
                centre = new Vector3((column.x - TankSize.x) * .5f, column.y * .5f, 0),
                boxSize = column
            }};
            float spacing = fine ? 1f / 150f : .01f;
            float radius = 2 * spacing;
            solverCompute = Instantiate(fluidCompute);
            fluid = solverObject.AddComponent<FluidSim>();
            fluid.compute = solverCompute;
            fluid.spawner = spawner;
            fluid.manualControl = true;
            fluid.gravity = -9.81f;
            for (int i = 0; i < BaffleCentres.Length; i++)
                fluid.SetStaticBoxBoundary(i, BaffleCentres[i], BaffleSize, BaffleRotation(i));
            fluid.IceCount = BaffleCentres.Length;
            fluid.smoothingRadius = radius;
            fluid.targetDensity = InteriorDensity(spacing, radius);
            fluid.pressureMultiplier = 100f;
            fluid.nearPressureMultiplier = 1f;
            fluid.viscosityStrength = 2e-8f * Mathf.Pow(spacing / .001f, 3);
            fluid.collisionDamping = .05f;
            bool sprayAvailable = sprayShader != null && sprayShader.isSupported && sprayArguments != null;
            fluid.maxFoamParticleCount = sprayAvailable ? 48000 : 1;
            fluid.foamActive = sprayAvailable && sprayEnabled;
            fluid.sprayOnly = sprayAvailable;
            fluid.trappedAirSpawnRate = fine ? 80f : 180f;
            fluid.trappedAirVelocityMinMax = new Vector2(.35f, 5f);
            fluid.foamKineticEnergyMinMax = new Vector2(.16f, 2.5f);
            fluid.sprayClassifyMaxNeighbours = 16;
            if (sprayAvailable)
            {
                var spray = solverObject.AddComponent<FoamRenderTest>();
                spray.shaderBillboard = sprayShader;
                spray.copyCountToArgsCompute = Instantiate(sprayArguments);
                spray.autoDraw = true;
                spray.visibleSpray = true;
                spray.scale = .08f;
            }
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
            int stepMicroseconds = ParticleStepMicroseconds[particleStepIndex];
            float stepSeconds = stepMicroseconds / 1000000f;
            int steps = Mathf.Min(12, Mathf.FloorToInt(pendingSeconds / stepSeconds));
            if (adaptivePressure)
            {
                long untilProjection = ProjectionIntervalMicroseconds - elapsedMicroseconds % ProjectionIntervalMicroseconds;
                steps = Mathf.Min(steps, (int)(untilProjection / stepMicroseconds));
            }
            if (steps == 0) return;

            double start = Time.realtimeSinceStartupAsDouble;
            fluid.AdvanceManually(stepSeconds, steps);
            submitMs = (float)((Time.realtimeSinceStartupAsDouble - start) * 1000 / steps);
            pendingSeconds -= steps * stepSeconds;
            elapsedMicroseconds += steps * stepMicroseconds;
            if (adaptivePressure && elapsedMicroseconds % ProjectionIntervalMicroseconds == 0)
                ApplyAdaptivePressure();
            if (!sampling && SystemInfo.supportsAsyncGPUReadback && fluid.SimulatedSeconds >= nextSample)
                SampleWater();
        }

        private void ApplyAdaptivePressure()
        {
            const int width = 24, height = 10;
            const float cell = PressureCellSize;
            int depth = Mathf.RoundToInt(TankSize.z / cell);
            var points = new Vector3[fluid.positionBuffer.count];
            var velocities = new Vector3[points.Length];
            fluid.positionBuffer.GetData(points);
            fluid.velocityBuffer.GetData(velocities);
            var cells = new int[points.Length];
            var counts = new int[width * height * depth];
            var sums = new Vector3[counts.Length];
            for (int i = 0; i < points.Length; i++)
            {
                Vector3 pos = points[i];
                int x = Mathf.Clamp(Mathf.FloorToInt((pos.x + TankSize.x * .5f) / cell), 0, width - 1);
                int y = Mathf.Clamp(Mathf.FloorToInt(pos.y / cell), 0, height - 1);
                int z = Mathf.Clamp(Mathf.FloorToInt((pos.z + TankSize.z * .5f) / cell), 0, depth - 1);
                int index = x + width * (y + height * z);
                cells[i] = index;
                counts[index]++;
                sums[index] += velocities[i];
            }

            var heights = new int[width * depth];
            int[] raisedFloor = BuildPressureFloor(width, depth, cell);
            int minimum = Mathf.Max(2, Mathf.RoundToInt(points.Length * .00025f));
            for (int z = 0; z < depth; z++)
            for (int x = 0; x < width; x++)
            {
                int y = raisedFloor[x + width * z];
                while (y < height - 1 && counts[x + width * (y + height * z)] >= minimum) y++;
                heights[x + width * z] = y;
            }

            var solver = new TallCellPressureReference(width, height, depth, cell, heights, true, FaceVelocity, raisedFloor);
            pressureSamples = solver.PressureSamples;
            if (pressureSamples == 0) return;
            var result = solver.Project();
            if (result.RelativeResidual > 1e-6) return;
            double[] correction = solver.CellVelocityCorrections(result.Velocity);
            for (int i = 0; i < velocities.Length; i++)
            {
                int offset = cells[i] * 3;
                velocities[i] += new Vector3((float)correction[offset], (float)correction[offset + 1],
                    (float)correction[offset + 2]);
            }
            fluid.velocityBuffer.SetData(velocities);

            double FaceVelocity(int axis, double px, double py, double pz)
            {
                int x = (int)System.Math.Round(px / cell - (axis == 0 ? 0 : .5));
                int y = (int)System.Math.Round(py / cell - (axis == 1 ? 0 : .5));
                int z = (int)System.Math.Round(pz / cell - (axis == 2 ? 0 : .5));
                int upper = x + width * (y + height * z);
                int lower = upper - (axis == 0 ? 1 : axis == 1 ? width : width * height);
                int column = x + width * z;
                int lowColumn = column - (axis == 0 ? 1 : axis == 2 ? width : 0);
                int lowY = axis == 1 ? y - 1 : y;
                bool lowWet = lowY >= raisedFloor[lowColumn] && lowY < heights[lowColumn];
                bool highWet = y >= raisedFloor[column] && y < heights[column];
                float value = 0;
                if (lowWet) value += sums[lower][axis] / counts[lower];
                if (highWet) value += sums[upper][axis] / counts[upper];
                return lowWet && highWet ? .5 * value : value;
            }
        }

        private static int[] BuildPressureFloor(int width, int depth, float spacing)
        {
            var floor = new int[width * depth];
            for (int z = 0; z < depth; z++)
            for (int x = 0; x < width; x++)
            {
                float worldX = (x + .5f) * spacing - TankSize.x * .5f;
                float worldZ = (z + .5f) * spacing - TankSize.z * .5f;
                for (int i = 0; i < BaffleCentres.Length; i++)
                {
                    float dx = worldX - BaffleCentres[i].x;
                    float dz = worldZ - BaffleCentres[i].z;
                    float halfCell = spacing * .5f;
                    if (Mathf.Abs(dx) >= BaffleExtentX + halfCell ||
                        Mathf.Abs(dz) >= BaffleExtentZ + halfCell) continue;

                    float sin = i % 2 == 0 ? BaffleSin : -BaffleSin;
                    float localPadding = halfCell * (BaffleCos + BaffleSin);
                    if (Mathf.Abs(BaffleCos * dx - sin * dz) < BaffleHalfSize.x + localPadding &&
                        Mathf.Abs(sin * dx + BaffleCos * dz) < BaffleHalfSize.z + localPadding)
                        floor[x + width * z] = Mathf.RoundToInt(BaffleSize.y / spacing);
                }
            }
            return floor;
        }

        private void SampleWater()
        {
            sampling = true;
            int run = generation;
            float simulatedSeconds = fluid.SimulatedSeconds;
            nextSample = simulatedSeconds + .05f;
            if (fluid.foamActive && !samplingSpray) SampleSprayCount(run);
            AsyncGPUReadback.Request(fluid.positionBuffer, request =>
            {
                if (this == null || run != generation) return;
                sampling = false;
                if (request.hasError) return;
                var points = request.GetData<Vector3>();
                int inside = 0, atEnd = 0;
                foreach (Vector3 point in points)
                {
                    if (!(Mathf.Abs(point.x) <= .601f && Mathf.Abs(point.z) <= TankSize.z * .5f + .001f &&
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

        private void SampleSprayCount(int run)
        {
            samplingSpray = true;
            AsyncGPUReadback.Request(fluid.foamCountBuffer, request =>
            {
                if (this == null || run != generation) return;
                samplingSpray = false;
                if (!request.hasError && sprayEnabled)
                    activeSpray = (int)request.GetData<uint>()[0];
            });
        }

        private void OnGUI()
        {
            GUILayout.BeginArea(new Rect(16, 16, 390, 410), GUI.skin.box);
            GUILayout.Label($"Glass Tank Dam Break · {VolumeLitres:F0} L");
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
                GUI.enabled = previousEnabled && ready && !sampling && !samplingSpray;
                bool reset = GUILayout.Button("Reset column");
                GUILayout.Label("GPU particle timestep · changing it resets the column");
                int selectedStep = GUILayout.Toolbar(particleStepIndex, ParticleStepLabels);
                if (fluid.sprayOnly)
                {
                    bool selectedSpray = GUILayout.Toggle(sprayEnabled, activeSpray >= 0 && sprayEnabled
                        ? $"Fine spray · {activeSpray:N0} active" : "Fine spray");
                    if (selectedSpray != sprayEnabled)
                    {
                        sprayEnabled = selectedSpray;
                        activeSpray = -1;
                        fluid.foamActive = selectedSpray;
                        fluid.foamCountBuffer.SetData(new uint[2]);
                    }
                }
                adaptivePressure = GUILayout.Toggle(adaptivePressure, "Adaptive pressure (experimental CPU/GPU)");
                bool selectedDouble = GUILayout.Toggle(doubleVolume,
                    $"Double water volume ({GetVolumeLitres(GetColumnSize(true)):F0} L)");
                var fineRegion = new Spawner3D.SpawnRegion { boxSize = GetColumnSize(selectedDouble) };
                Vector3Int fineDimensions = fineRegion.BoxParticleCount(FineParticleDensity);
                int fineCount = fineDimensions.x * fineDimensions.y * fineDimensions.z;
                bool selectedFine = GUILayout.Toggle(fine, $"Fine resolution ({fineCount:N0} particles)");
                if (reset || selectedStep != particleStepIndex || selectedDouble != doubleVolume || selectedFine != fine)
                {
                    particleStepIndex = selectedStep;
                    doubleVolume = selectedDouble;
                    fine = selectedFine;
                    PrepareColumn();
                }
                GUI.enabled = previousEnabled;
                GUILayout.Label($"{(fluid.positionBuffer?.count ?? 0):N0} particles · {fluid.SimulatedSeconds:F3} s simulated");
                GUILayout.Label(arrival < 0 ? "Wave arrival pending" : $"Wave arrival at x = 0.55 m · {arrival:F3} s");
                GUILayout.Label(!SystemInfo.supportsAsyncGPUReadback ? "Volume readback unavailable" :
                    retained < 0 ? "Retained volume pending" : $"Retained volume · {retained:F2} L");
                GUILayout.Label($"CPU submission per step · {submitMs:F3} ms");
                if (adaptivePressure) GUILayout.Label($"Live adaptive pressure samples · {pressureSamples:N0}");
                if (GUILayout.Button("Compare CPU pressure projections")) ComparePressureProjections();
            }
            GUILayout.EndArea();
        }

        private void ComparePressureProjections()
        {
            const double spacing = .05;
            const int width = 24, height = 10;
            int depth = Mathf.RoundToInt(TankSize.z / PressureCellSize);
            var heights = new int[width * depth];
            int[] raisedFloor = BuildPressureFloor(width, depth, (float)spacing);
            System.Array.Copy(raisedFloor, heights, heights.Length);
            int wetColumns = Mathf.RoundToInt(ColumnSize.x / (float)spacing);
            for (int z = 0; z < depth; z++)
            for (int x = 0; x < wetColumns; x++) heights[x + width * z] = 6;

            var uniform = new TallCellPressureReference(width, height, depth, spacing, heights, false, SampleVelocity, raisedFloor);
            var adaptive = new TallCellPressureReference(width, height, depth, spacing, heights, true, SampleVelocity, raisedFloor);
            var baseline = uniform.Project();
            var reduced = adaptive.Project();
            double difference = 0;
            for (int i = 0; i < baseline.Velocity.Length; i++)
            {
                double error = baseline.Velocity[i] - reduced.Velocity[i];
                difference += error * error;
            }
            Debug.Log($"Tank CPU pressure reference · {VolumeLitres:F0} L · uniform {uniform.PressureSamples} samples, " +
                $"residual {baseline.RelativeResidual:E2} in {baseline.Iterations} iterations · adaptive " +
                $"{adaptive.PressureSamples} samples, residual {reduced.RelativeResidual:E2} in {reduced.Iterations} iterations · " +
                $"face velocity RMS difference {System.Math.Sqrt(difference / baseline.Velocity.Length):E2} m/s", this);

            double SampleVelocity(int axis, double x, double y, double z) =>
                axis == 1 ? -9.81 * .0004 : .025 * System.Math.Sin(9 * x + 4 * y + 7 * z + axis);
        }

        private void OnDestroy()
        {
            if (solverCompute != null) Destroy(solverCompute);
            foreach (var material in materials) Destroy(material);
        }
    }
}
