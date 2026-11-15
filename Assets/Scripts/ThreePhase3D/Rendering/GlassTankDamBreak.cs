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
        private static readonly Vector3 CityTankSize = new Vector3(1.2f, .5f, .8f);
        // Kleefsman et al. (2005), Fig. 17, reflected so the wave moves in +X.
        private static readonly Vector3 BenchmarkTankSize = new Vector3(3.22f, 1f, 1f);
        private static readonly Vector3 BenchmarkColumn = new Vector3(1.228f, .55f, 1f);
        private static readonly Bounds BenchmarkBox =
            new Bounds(new Vector3(.9465f, .0805f, 0), new Vector3(.161f, .161f, .403f));
        private const float BenchmarkH2X = .618f;
        private const float JetSliceDepth = .06f;
        private const float JetSupportHeight = .04f;
        private const int BenchmarkSampleInterval = 20000;
        private const int BenchmarkEndTime = 1800000;
        private bool benchmark;
        private bool benchmarkWallPressure;
        private const float StandardBenchmarkSpacing = .02f;
        private const float FineBenchmarkSpacing = .015f;
        private bool benchmarkFine;
        private bool selectedBenchmarkFine;
        private (bool WallPressure, bool Fine)? pendingBenchmarkRun;
        private float BenchmarkSpacing => benchmarkFine ? FineBenchmarkSpacing : StandardBenchmarkSpacing;
        private int benchmarkReadsPending;
        private float benchmarkH2, benchmarkSheet, benchmarkPeak, benchmarkJet, benchmarkJetPeak;
        private Transform tankGeometry;
        private Material tankStone, tankGlass, benchmarkMaterial;
        private Material[] cityPalette;
        private Camera benchmarkCamera;
        private Vector3 savedCameraPosition;
        private Vector3 TankSize => benchmark ? BenchmarkTankSize : CityTankSize;
        private const float PressureCellSize = .05f;
        private const int FineParticleDensity = 3375000;
        private Vector3 ColumnSize => benchmark ? BenchmarkColumn : GetColumnSize(doubleVolume);
        private static Vector3 GetColumnSize(bool doubled) => new Vector3(doubled ? .4f : .2f, .3f, CityTankSize.z);
        private static readonly int[] ParticleStepMicroseconds = { 400, 800, 1000 };
        private static readonly string[] ParticleStepLabels = { "0.4 ms", "0.8 ms", "1.0 ms" };
        private int particleStepIndex;
        private long elapsedMicroseconds;
        private float VolumeLitres => GetVolumeLitres(ColumnSize);
        private static float GetVolumeLitres(Vector3 size) => size.x * size.y * size.z * 1000f;
        private const float ArrivalPlane = .55f;
        private static readonly (Bounds Bounds, Quaternion Rotation)[] Buildings =
        {
            Building(-.080f, -.285f, .055f, .150f, .140f, -22f),
            Building(-.045f, -.135f, .065f, .100f, .090f, 12f),
            Building(.065f, -.295f, .080f, .250f, .085f, 10f),
            Building(.115f, -.145f, .090f, .350f, .095f, -15f),
            Building(.225f, -.290f, .130f, .150f, .055f, 24f),
            Building(.265f, -.115f, .060f, .200f, .070f, -12f),
            Building(.100f, .135f, .075f, .150f, .120f, 24f),
            Building(.195f, .285f, .085f, .300f, .085f, -18f),
            Building(.265f, .105f, .115f, .100f, .080f, -24f),
            Building(.365f, .245f, .080f, .250f, .115f, 18f),
            Building(.420f, .095f, .080f, .150f, .065f, 14f),
            Building(.470f, .345f, .105f, .200f, .035f, -10f)
        };

        private static (Bounds, Quaternion) Building(float x, float z, float width, float height, float depth, float yaw) =>
            (new Bounds(new Vector3(x, height * .5f, z), new Vector3(width, height, depth)),
                Quaternion.Euler(0, yaw, 0));

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

            tankStone = MakeMaterial(new Color(.36f, .38f, .38f));
            benchmarkMaterial = MakeMaterial(new Color(.42f, .48f, .48f));
            var glass = tankGlass = MakeMaterial(new Color(.46f, .78f, .85f, .12f));
            glass.SetOverrideTag("RenderType", "Transparent");
            glass.SetInt("_SrcBlend", (int)BlendMode.SrcAlpha);
            glass.SetInt("_DstBlend", (int)BlendMode.OneMinusSrcAlpha);
            glass.SetInt("_ZWrite", 0);
            glass.EnableKeyword("_ALPHABLEND_ON");
            glass.renderQueue = (int)RenderQueue.Transparent;
            cityPalette = new[]
            {
                MakeMaterial(new Color(.27f, .39f, .42f)),
                MakeMaterial(new Color(.39f, .46f, .45f)),
                MakeMaterial(new Color(.51f, .54f, .53f))
            };
            RebuildTank();
            if (SystemInfo.supportsComputeShaders && fluidCompute != null && surfaceArguments != null &&
                waterShader != null && waterShader.isSupported)
                PrepareColumn();
        }

        private void RebuildTank()
        {
            if (tankGeometry != null)
            {
                tankGeometry.gameObject.SetActive(false);
                Destroy(tankGeometry.gameObject);
            }
            tankGeometry = new GameObject("Tank geometry").transform;
            tankGeometry.SetParent(transform, false);
            Vector3 size = TankSize;
            Box("Tank base", new Vector3(0, -.025f, 0), new Vector3(size.x + .05f, .05f, size.z + .05f), tankStone);
            Box("Bench", new Vector3(0, -.09f, 0), new Vector3(Mathf.Max(2.5f, size.x + .2f), .08f, 1.3f), tankStone);
            if (benchmark)
                Box("MARIN benchmark obstacle", BenchmarkBox.center, BenchmarkBox.size, benchmarkMaterial);
            else
                BuildCity();
            for (int sign = -1; sign <= 1; sign += 2)
            {
                Box("Long glass wall", new Vector3(0, size.y * .5f, sign * (size.z * .5f + .004f)),
                    new Vector3(size.x + .016f, size.y, .008f), tankGlass);
                Box("End glass wall", new Vector3(sign * (size.x * .5f + .004f), size.y * .5f, 0),
                    new Vector3(.008f, size.y, size.z), tankGlass);
            }
        }

        private void SetBenchmark(bool enabled)
        {
            benchmark = enabled;
            if (enabled)
            {
                benchmarkCamera = Camera.main;
                if (benchmarkCamera != null)
                {
                    savedCameraPosition = benchmarkCamera.transform.position;
                    Vector3 focus = Vector3.up * .35f;
                    benchmarkCamera.transform.position = focus + (savedCameraPosition - focus) * 2.25f;
                }
            }
            else if (benchmarkCamera != null)
            {
                benchmarkCamera.transform.position = savedCameraPosition;
                benchmarkCamera = null;
            }
            RebuildTank();
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
            panel.transform.SetParent(tankGeometry);
            panel.transform.SetPositionAndRotation(position, rotation ?? Quaternion.identity);
            panel.transform.localScale = size;
            var renderer = panel.GetComponent<MeshRenderer>();
            renderer.sharedMaterial = material;
            if (material.renderQueue == (int)RenderQueue.Transparent)
                renderer.shadowCastingMode = ShadowCastingMode.Off;
            Destroy(panel.GetComponent<Collider>());
        }

        private void BuildCity()
        {
            for (int i = 0; i < Buildings.Length; i++)
            {
                var (bounds, rotation) = Buildings[i];
                Material material = cityPalette[i % cityPalette.Length];
                Box($"City building {i + 1}", bounds.center, bounds.size, material, rotation);
            }
        }

        private void PrepareColumn()
        {
            generation++;
            running = sampling = samplingSpray = false;
            activeSpray = -1;
            pendingSeconds = nextSample = submitMs = 0;
            arrival = retained = -1;
            benchmarkH2 = benchmarkSheet = benchmarkPeak = benchmarkJet = benchmarkJetPeak = 0;
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
            Vector3 solverSize = TankSize;
            // Keep the solver ceiling above the open rim in the reference experiment.
            if (benchmark) solverSize.y += .5f;
            solverObject.transform.position = Vector3.up * (solverSize.y * .5f);
            solverObject.transform.localScale = solverSize;
            var spawner = solverObject.AddComponent<Spawner3D>();
            float spacing = benchmark ? BenchmarkSpacing : fine ? 1f / 150f : .01f;
            spawner.particleSpawnDensity = benchmark
                ? Mathf.RoundToInt(1f / (spacing * spacing * spacing))
                : fine ? FineParticleDensity : 1000000;
            Vector3 column = ColumnSize;
            spawner.spawnRegions = new[] { new Spawner3D.SpawnRegion
            {
                centre = new Vector3((column.x - TankSize.x) * .5f, column.y * .5f, 0),
                boxSize = column
            }};
            float radius = 2 * spacing;
            solverCompute = Instantiate(fluidCompute);
            fluid = solverObject.AddComponent<FluidSim>();
            fluid.compute = solverCompute;
            fluid.spawner = spawner;
            fluid.manualControl = true;
            fluid.gravity = -9.81f;
            if (benchmark)
            {
                fluid.SetStaticBoxBoundary(0, BenchmarkBox.center, BenchmarkBox.size);
                fluid.IceCount = 1;
                fluid.boxWallPressure = benchmarkWallPressure;
            }
            else
            {
                for (int i = 0; i < Buildings.Length; i++)
                {
                    var (bounds, rotation) = Buildings[i];
                    fluid.SetStaticBoxBoundary(i, bounds.center, bounds.size, rotation);
                }
                fluid.IceCount = Buildings.Length;
            }
            fluid.smoothingRadius = radius;
            fluid.targetDensity = InteriorDensity(spacing, radius);
            fluid.pressureMultiplier = 100f;
            fluid.nearPressureMultiplier = 1f;
            fluid.viscosityStrength = 2e-8f * Mathf.Pow(spacing / .001f, 3);
            fluid.collisionDamping = .05f;
            bool sprayAvailable = !benchmark && sprayShader != null && sprayShader.isSupported && sprayArguments != null;
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
            if (pendingBenchmarkRun.HasValue && benchmarkReadsPending == 0)
            {
                (benchmarkWallPressure, benchmarkFine) = pendingBenchmarkRun.Value;
                pendingBenchmarkRun = null;
                PrepareColumn();
                running = true;
                Debug.Log($"MARIN run started · {BenchmarkSpacing * 1000:F0} mm particle spacing · GPU wall pressure {(benchmarkWallPressure ? "ON (1)" : "OFF (0)")}", this);
                return;
            }

            if (!running || fluid.positionBuffer == null) return;
            pendingSeconds = Mathf.Min(pendingSeconds + Time.unscaledDeltaTime * .25f, .008f);
            int stepMicroseconds = benchmark ? 400 : ParticleStepMicroseconds[particleStepIndex];
            float stepSeconds = stepMicroseconds / 1000000f;
            int steps = Mathf.Min(12, Mathf.FloorToInt(pendingSeconds / stepSeconds));
            if (benchmark)
            {
                long remaining = BenchmarkSampleInterval - elapsedMicroseconds % BenchmarkSampleInterval;
                steps = Mathf.Min(steps, (int)(remaining / stepMicroseconds));
            }
            else if (adaptivePressure)
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
            if (benchmark)
            {
                if (elapsedMicroseconds % BenchmarkSampleInterval == 0 && SystemInfo.supportsAsyncGPUReadback)
                    SampleBenchmark();
                if (elapsedMicroseconds >= BenchmarkEndTime) running = false;
                return;
            }
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

        private int[] BuildPressureFloor(int width, int depth, float spacing)
        {
            var floor = new int[width * depth];
            for (int z = 0; z < depth; z++)
            for (int x = 0; x < width; x++)
            {
                float worldX = (x + .5f) * spacing - TankSize.x * .5f;
                float worldZ = (z + .5f) * spacing - TankSize.z * .5f;
                foreach (var (bounds, rotation) in Buildings)
                {
                    Vector3 offset = new Vector3(worldX - bounds.center.x, 0, worldZ - bounds.center.z);
                    Vector3 local = Quaternion.Inverse(rotation) * offset;
                    if (Mathf.Abs(local.x) < bounds.extents.x && Mathf.Abs(local.z) < bounds.extents.z)
                        floor[x + width * z] = Mathf.Max(floor[x + width * z],
                            Mathf.RoundToInt(bounds.size.y / spacing));
                }
            }
            return floor;
        }

        private void SampleBenchmark()
        {
            int run = generation;
            float seconds = elapsedMicroseconds / 1000000f;
            int wallPressure = benchmarkWallPressure ? 1 : 0;
            float spacingMillimetres = BenchmarkSpacing * 1000;
            benchmarkReadsPending++;
            AsyncGPUReadback.Request(fluid.positionBuffer, request =>
            {
                if (this == null || run != generation) return;
                benchmarkReadsPending--;
                if (request.hasError)
                {
                    Debug.LogWarning($"MARIN benchmark readback failed at {seconds:F3} s.", this);
                    return;
                }
                var gauge = new List<float>();
                var impact = new List<float>();
                var jet = new List<float>();
                foreach (Vector3 point in request.GetData<Vector3>())
                {
                    if (Mathf.Abs(point.x - BenchmarkH2X) <= .04f && Mathf.Abs(point.z) <= .04f)
                        gauge.Add(point.y);
                    if (Mathf.Abs(point.z) > BenchmarkBox.extents.z) continue;
                    if (point.x >= BenchmarkBox.min.x - .08f && point.x <= BenchmarkBox.max.x + .04f)
                        impact.Add(point.y);
                    if (point.x >= BenchmarkBox.min.x - JetSliceDepth && point.x <= BenchmarkBox.min.x)
                        jet.Add(point.y);
                }

                benchmarkH2 = SurfacePercentile(gauge);
                benchmarkSheet = SurfacePercentile(impact);
                benchmarkPeak = Mathf.Max(benchmarkPeak, benchmarkSheet);
                jet.Sort();
                float jetMax = jet.Count == 0 ? 0 : jet[jet.Count - 1];
                benchmarkJet = SupportedJetHeight(jet);
                benchmarkJetPeak = Mathf.Max(benchmarkJetPeak, benchmarkJet);
                Debug.Log(System.FormattableString.Invariant(
                    $"MARIN,t_s,H2_p95_m,box_zone_p95_m,jet_max_m,jet_supported_m,jet_count,spacing_mm,wall_pressure: {seconds:F3},{benchmarkH2:F3},{benchmarkSheet:F3},{jetMax:F3},{benchmarkJet:F3},{jet.Count},{spacingMillimetres:F0},{wallPressure}"), this);
            });
        }

        private static float SupportedJetHeight(List<float> sortedHeights)
        {
            // Require three particles in a 4 cm vertical band before counting a peak.
            for (int i = sortedHeights.Count - 1; i >= 2; i--)
            {
                if (sortedHeights[i] - sortedHeights[i - 2] <= JetSupportHeight)
                    return sortedHeights[i];
            }
            return 0;
        }

        private static float SurfacePercentile(List<float> heights)
        {
            if (heights.Count == 0) return 0;
            heights.Sort();
            return heights[Mathf.CeilToInt(heights.Count * .95f) - 1];
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

        private void QueueBenchmarkRun(bool useWallPressure)
        {
            running = false;
            pendingBenchmarkRun = (useWallPressure, selectedBenchmarkFine);
        }

        private void OnGUI()
        {
            GUILayout.BeginArea(new Rect(16, 16, 390, 410), GUI.skin.box);
            GUILayout.Label($"Glass Tank Dam Break · {VolumeLitres:F0} L");
            if (fluid != null)
            {
                bool modeControlEnabled = GUI.enabled;
                GUI.enabled = modeControlEnabled && !pendingBenchmarkRun.HasValue && benchmarkReadsPending == 0 && !sampling && !samplingSpray;
                if (GUILayout.Button(benchmark ? "Return to city tank" : "Single-box benchmark"))
                    SetBenchmark(!benchmark);
                GUI.enabled = modeControlEnabled;
            }
            if (benchmark)
            {
                GUILayout.Label("MARIN single-box benchmark · GPU liquid · step 0.4 ms");
                GUILayout.Label("Bulk-water sheet only · 20 ms samples in Console");
                GUILayout.Label($"Time {elapsedMicroseconds / 1000000f:F3} s / 1.800 s");
                GUILayout.Label($"H2 {benchmarkH2:F3} m · box zone {benchmarkSheet:F3} m · peak {benchmarkPeak:F3} m (p95)");
                GUILayout.Label($"Upstream jet {benchmarkJet:F3} m · peak {benchmarkJetPeak:F3} m (supported)");
                if (!SystemInfo.supportsAsyncGPUReadback) GUILayout.Label("GPU readback unavailable.");
                GUILayout.Label($"GPU wall pressure: {(benchmarkWallPressure ? "ON (1)" : "OFF (0)")}");
                bool controlsEnabled = GUI.enabled;
                GUI.enabled = controlsEnabled && !pendingBenchmarkRun.HasValue;
                selectedBenchmarkFine = GUILayout.Toggle(selectedBenchmarkFine, "Finer benchmark · 15 mm spacing (next run)");
                GUILayout.Label($"Current spacing {BenchmarkSpacing * 1000:F0} mm · {fluid.positionBuffer?.count ?? 0:N0} particles");
                if (GUILayout.Button("Start baseline run · wall pressure OFF"))
                    QueueBenchmarkRun(false);
                if (GUILayout.Button("Start wall-pressure run · ON"))
                    QueueBenchmarkRun(true);
                if (fluid.positionBuffer != null && (running || elapsedMicroseconds > 0) &&
                    elapsedMicroseconds < BenchmarkEndTime && GUILayout.Button(running ? "Pause" : "Resume"))
                {
                    running = !running;
                    pendingSeconds = 0;
                }
                GUI.enabled = controlsEnabled;
                GUILayout.EndArea();
                return;
            }
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
