using System.Collections.Generic;
using PhysicsSim.ThreePhase3D.Flow;
using Seb.Fluid.Rendering;
using Seb.Fluid.Simulation;
using UnityEngine;
using UnityEngine.Rendering;
#if UNITY_EDITOR
using System.Collections;
using System.IO;
using UnityEditor.Media;
#endif

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
#if UNITY_EDITOR
        private bool offlineQueued, offlineCapture, offlineFramePending;
        private int offlineFrames;
        private MediaEncoder offlineEncoder;
        private string offlineOutput, lastOfflineVideo;
#endif
        private bool benchmarkWallPressure;
        private const float MixedFineLayerHeight = .05f;
        private bool benchmarkMixed, selectedBenchmarkMixed, profileMixed;
        private static readonly float[] BenchmarkSpacings = { .02f, .0175f, .015f, .0125f, .01f, .0075f, .005f };
        private static readonly string[] BenchmarkSpacingLabels = { "20", "17.5", "15", "12.5", "10", "7.5", "5" };
        private int benchmarkSpacingIndex;
        private int selectedBenchmarkSpacing = 5;
        private bool benchmarkMeasurementsOnly;
        private bool selectedMeasurementsOnly;
        private double benchmarkStartTime;
        private (bool WallPressure, int SpacingIndex, bool MeasurementsOnly, bool Mixed)? pendingBenchmarkRun;
        private float BenchmarkSpacing => BenchmarkSpacings[benchmarkSpacingIndex];
        private int BenchmarkStepMicroseconds => Mathf.RoundToInt(400 * BenchmarkSpacing / BenchmarkSpacings[0]);
        private int benchmarkReadsPending;
        private float benchmarkSampledSeconds;
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

        [SerializeField] private bool solverPerformanceScene;
        private const int ProfileWarmupSteps = 8;
        private const int ProfileMeasuredSteps = 64;
        private const int ProfileBatchSteps = 4;
        private bool profileRunning, profileWallPressure, profileAtImpact;
        private int profileSpacing, profileWarmup, profileWarmupTarget, profileSubmitted;
        private string[] profileResults;

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
            if (solverPerformanceScene)
            {
                benchmark = true;
                benchmarkMeasurementsOnly = true;
                benchmarkSpacingIndex = profileSpacing = BenchmarkSpacings.Length - 1;
                return;
            }

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
            benchmarkSampledSeconds = 0;
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
            int coarseCount = 0;
            float coarseMass = 1;
            if (benchmark && benchmarkMixed)
            {
                var coarse = spawner.spawnRegions[0];
                var fineLayer = coarse;
                coarse.boxSize.y -= MixedFineLayerHeight;
                coarse.centre.y = coarse.boxSize.y * .5f;
                coarse.boxParticleDensityOverride = 1000000;
                fineLayer.boxSize.y = MixedFineLayerHeight;
                fineLayer.centre.y = coarse.boxSize.y + MixedFineLayerHeight * .5f;
                spawner.spawnRegions = new[] { coarse, fineLayer };
                Vector3Int coarseGrid = coarse.BoxParticleCount(spawner.particleSpawnDensity);
                Vector3Int fineGrid = fineLayer.BoxParticleCount(spawner.particleSpawnDensity);
                coarseCount = coarseGrid.x * coarseGrid.y * coarseGrid.z;
                int fineCount = fineGrid.x * fineGrid.y * fineGrid.z;
                // Each particle carries its sampled volume at the same reference water density.
                coarseMass = (coarse.Volume / coarseCount) / (fineLayer.Volume / fineCount);
            }

            float radius = 2 * spacing;
            solverCompute = Instantiate(fluidCompute);
            fluid = solverObject.AddComponent<FluidSim>();
            fluid.compute = solverCompute;
            fluid.spawner = spawner;
            fluid.MixedCoarseParticleCount = coarseCount;
            fluid.MixedCoarseMass = coarseMass;
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
            fluid.worldSpaceSurface = true;
            fluid.renderToTex3D = !benchmark || !benchmarkMeasurementsOnly;
            fluid.densityTextureRes = fine ? 192 : 128;

            if (fluid.renderToTex3D)
            {
                var surface = solverObject.AddComponent<MarchingCubesFluidRenderer>();
                surface.sim = fluid;
                surface.drawShader = waterShader;
                surface.renderArgsCompute = Instantiate(surfaceArguments);
                surface.worldSpaceSurface = true;
                surface.col = new Color(1, 1, 1, .12f);
                surface.isoLevel = fluid.targetDensity * .45f;
            }
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
            if (solverPerformanceScene)
            {
                UpdatePerformanceRun();
                return;
            }
#if UNITY_EDITOR
            if (offlineQueued && benchmarkReadsPending == 0)
            {
                offlineQueued = false;
                StartOfflineCapture();
                return;
            }
            if (offlineCapture)
            {
                if (Input.GetKeyDown(KeyCode.Escape)) FinishOfflineCapture(false);
                else if (!offlineFramePending && fluid.positionBuffer != null)
                {
                    try
                    {
                        AdvanceOfflineCapture();
                    }
                    catch (System.Exception error)
                    {
                        Debug.LogException(error, this);
                        FinishOfflineCapture(false);
                    }
                }
                return;
            }
#endif
            if (pendingBenchmarkRun.HasValue && benchmarkReadsPending == 0)
            {
                (benchmarkWallPressure, benchmarkSpacingIndex, benchmarkMeasurementsOnly, benchmarkMixed) = pendingBenchmarkRun.Value;
                pendingBenchmarkRun = null;
                PrepareColumn();
                running = true;
                benchmarkStartTime = Time.realtimeSinceStartupAsDouble;
                Debug.Log($"MARIN run started · {BenchmarkSpacing * 1000:0.#} mm particle spacing · {BenchmarkStepMicroseconds / 1000f:0.###} ms max step · GPU wall pressure {(benchmarkWallPressure ? "ON (1)" : "OFF (0)")}", this);
                return;
            }

            if (!running || fluid.positionBuffer == null || (benchmark && benchmarkReadsPending > 0)) return;
            float playbackRate = benchmark ? 1f : .25f;
            float maxPendingSeconds = benchmark ? 2 * BenchmarkSampleInterval / 1000000f : .008f;
            pendingSeconds = Mathf.Min(pendingSeconds + Time.unscaledDeltaTime * playbackRate, maxPendingSeconds);
            int stepMicroseconds = benchmark ? BenchmarkStepMicroseconds : ParticleStepMicroseconds[particleStepIndex];
            int maxSteps = 12;
            if (benchmark)
            {
                int remaining = BenchmarkSampleInterval - (int)(elapsedMicroseconds % BenchmarkSampleInterval);
                // Take a shorter last step when needed to land exactly on the sample time.
                stepMicroseconds = Mathf.Min(stepMicroseconds, remaining);
                // Keep GPU submissions short at the highest particle counts.
                int batchLimit = BenchmarkSpacing <= .0075f ? 8 : 24;
                maxSteps = Mathf.Min(batchLimit, remaining / stepMicroseconds);
            }
            float stepSeconds = stepMicroseconds / 1000000f;
            int steps = Mathf.Min(maxSteps, Mathf.FloorToInt(pendingSeconds / stepSeconds));
            if (!benchmark && adaptivePressure)
            {
                long untilProjection = ProjectionIntervalMicroseconds - elapsedMicroseconds % ProjectionIntervalMicroseconds;
                steps = Mathf.Min(steps, (int)(untilProjection / stepMicroseconds));
            }
            if (steps == 0) return;

            long nextElapsed = elapsedMicroseconds + steps * stepMicroseconds;
            bool benchmarkSample = benchmark && nextElapsed % BenchmarkSampleInterval == 0;
            double start = Time.realtimeSinceStartupAsDouble;
            fluid.AdvanceManually(stepSeconds, steps, updateSurface: !benchmark || benchmarkSample);
            submitMs = (float)((Time.realtimeSinceStartupAsDouble - start) * 1000 / steps);
            pendingSeconds -= steps * stepSeconds;
            elapsedMicroseconds = nextElapsed;
            if (benchmark)
            {
                if (benchmarkSample && SystemInfo.supportsAsyncGPUReadback)
                    SampleBenchmark();
                if (elapsedMicroseconds >= BenchmarkEndTime)
                {
                    running = false;
                    if (!SystemInfo.supportsAsyncGPUReadback) LogBenchmarkCompletion();
                }
                return;
            }
            if (adaptivePressure && elapsedMicroseconds % ProjectionIntervalMicroseconds == 0)
                ApplyAdaptivePressure();
            if (!sampling && SystemInfo.supportsAsyncGPUReadback && fluid.SimulatedSeconds >= nextSample)
                SampleWater();
        }

        private void StartPerformanceRun()
        {
            benchmarkMixed = profileMixed;
            benchmarkSpacingIndex = profileMixed ? BenchmarkSpacings.Length - 1 : profileSpacing;
            benchmarkWallPressure = !profileMixed && profileWallPressure;
            PrepareColumn();
            profileWarmup = profileSubmitted = 0;
            profileWarmupTarget = profileAtImpact ? Mathf.RoundToInt(560000f / BenchmarkStepMicroseconds) : ProfileWarmupSteps;
            profileResults = null;
            profileRunning = true;
        }

        private void UpdatePerformanceRun()
        {
            if (!profileRunning || fluid.positionBuffer == null) return;
            float stepSeconds = BenchmarkStepMicroseconds / 1000000f;

            if (profileWarmup < profileWarmupTarget)
            {
                int steps = Mathf.Min(24, profileWarmupTarget - profileWarmup);
                fluid.AdvanceManually(stepSeconds, steps, updateSurface: false);
                profileWarmup += steps;
                return;
            }

            try
            {
                if (profileSubmitted == 0)
                {
                    fluid.WaitForSubmittedWork();
                    fluid.StageCompletionTicks = new long[FluidSim.StageProfileLabels.Length];
                }

                fluid.AdvanceManually(stepSeconds, ProfileBatchSteps, updateSurface: false);
                profileSubmitted += ProfileBatchSteps;
                if (profileSubmitted == ProfileMeasuredSteps) FinishPerformanceRun(stepSeconds);
            }
            catch (System.Exception error)
            {
                fluid.StageCompletionTicks = null;
                profileRunning = false;
                profileResults = new[] { "Stage timing failed. See the Console for details." };
                Debug.LogException(error, this);
            }
        }

        private void FinishPerformanceRun(float stepSeconds)
        {
            long[] ticks = fluid.StageCompletionTicks;
            fluid.StageCompletionTicks = null;
            profileRunning = false;

            profileResults = new string[ticks.Length];
            double millisecondsPerTickPerStep = 1000d / System.Diagnostics.Stopwatch.Frequency / ProfileMeasuredSteps;
            for (int i = 0; i < profileResults.Length; i++)
                profileResults[i] = $"{FluidSim.StageProfileLabels[i]} · {ticks[i] * millisecondsPerTickPerStep:F3} ms/step";

            Debug.Log($"MARIN synchronized stage timings · {SystemInfo.graphicsDeviceName} · {SystemInfo.graphicsDeviceType} · " +
                $"{fluid.positionBuffer.count:N0} particles · {BenchmarkSpacing * 1000:0.#} mm · " +
                $"{BenchmarkStepMicroseconds / 1000f:0.###} ms step · sampled from {profileWarmup * stepSeconds:F4} s · " +
                $"{(benchmarkMixed ? "mixed 5/10 mm" : "uniform")} · box wall pressure {(benchmarkWallPressure ? "ON" : "OFF")} · {profileSubmitted} completed steps\n" +
                "Elapsed time includes CPU submission and a one-element GPU readback per stage.\n" +
                string.Join("\n", profileResults), this);
        }

        private void DrawPerformanceControls()
        {
            GUILayout.BeginArea(new Rect(16, 16, 570, 410), GUI.skin.box);
            GUILayout.Label("MARIN Solver Performance");
            GUILayout.Label($"{SystemInfo.graphicsDeviceName} · {SystemInfo.graphicsDeviceType}");
            GUILayout.Label("64 measured steps · synchronized stage timings");
            bool enabled = GUI.enabled;
            GUI.enabled = enabled && !profileRunning;
            GUILayout.Label("Particle spacing (mm)");
            profileSpacing = GUILayout.Toolbar(profileSpacing, BenchmarkSpacingLabels);
            profileMixed = GUILayout.Toggle(profileMixed, "Mixed spacing · 5 mm surface / 10 mm bulk");
            GUI.enabled = enabled && !profileRunning && !profileMixed;
            profileWallPressure = GUILayout.Toggle(profileWallPressure, "Box wall pressure (uniform runs)");
            GUI.enabled = enabled && !profileRunning;
            profileAtImpact = GUILayout.Toggle(profileAtImpact, "Profile near impact (advance to 0.56 s first)");
            if (!SystemInfo.supportsComputeShaders || !SystemInfo.supportsAsyncGPUReadback || fluidCompute == null)
                GUILayout.Label("GPU compute or readback unavailable.");
            else if (GUILayout.Button("Measure solver stages"))
                StartPerformanceRun();
            GUI.enabled = enabled;

            if (profileRunning)
                GUILayout.Label($"Warmup {profileWarmup}/{profileWarmupTarget} · measured steps submitted {profileSubmitted}/{ProfileMeasuredSteps}");
            if (profileResults != null)
                foreach (string result in profileResults) GUILayout.Label(result);
            GUILayout.EndArea();
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
            int mixedResolution = benchmarkMixed ? 1 : 0;
            float spacingMillimetres = BenchmarkSpacing * 1000;
            float maxStepMilliseconds = BenchmarkStepMicroseconds / 1000f;
            benchmarkReadsPending++;
            AsyncGPUReadback.Request(fluid.positionBuffer, request =>
            {
                if (this == null || run != generation) return;
                benchmarkReadsPending--;
                if (request.hasError)
                {
                    Debug.LogWarning($"MARIN benchmark readback failed at {seconds:F3} s.", this);
                    if (seconds >= BenchmarkEndTime / 1000000f) LogBenchmarkCompletion();
                    return;
                }
                benchmarkSampledSeconds = seconds;
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
                    $"MARIN,t_s,H2_p95_m,box_zone_p95_m,jet_max_m,jet_supported_m,jet_count,max_step_ms,spacing_mm,mixed_resolution,wall_pressure: {seconds:F3},{benchmarkH2:F3},{benchmarkSheet:F3},{jetMax:F3},{benchmarkJet:F3},{jet.Count},{maxStepMilliseconds:0.###},{spacingMillimetres:0.#},{mixedResolution},{wallPressure}"), this);
                if (seconds >= BenchmarkEndTime / 1000000f) LogBenchmarkCompletion();
            });
        }

        private void LogBenchmarkCompletion()
        {
            double wallSeconds = Time.realtimeSinceStartupAsDouble - benchmarkStartTime;
            Debug.Log($"MARIN completed in {wallSeconds:F1} wall-clock seconds · " +
                $"{BenchmarkEndTime / 1000000d / wallSeconds:P1} of real time · " +
                $"{BenchmarkSpacing * 1000:0.#} mm · {(benchmarkMeasurementsOnly ? "measurements only" : "water rendering on")}", this);
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

#if UNITY_EDITOR
        private void StartOfflineCapture()
        {
            try
            {
                string directory = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "MARIN Captures"));
                Directory.CreateDirectory(directory);
                offlineOutput = Path.Combine(directory, $"MARIN-5mm-{System.DateTime.Now:yyyyMMdd-HHmmss-fff}.mp4");
                benchmarkSpacingIndex = BenchmarkSpacings.Length - 1;
                benchmarkMeasurementsOnly = false;
                benchmarkMixed = false;
                PrepareColumn();
                offlineFrames = 0;
                offlineCapture = true;
                Debug.Log($"MARIN offline capture started · 5 mm · 0.100 ms solver step · 50 FPS · wall pressure {(benchmarkWallPressure ? "ON" : "OFF")}", this);
            }
            catch (System.Exception error)
            {
                Debug.LogException(error, this);
            }
        }

        private void AdvanceOfflineCapture()
        {
            const int stepsPerBatch = 24;
            int stepMicroseconds = BenchmarkStepMicroseconds;
            int remaining = BenchmarkSampleInterval - (int)(elapsedMicroseconds % BenchmarkSampleInterval);
            int steps = Mathf.Min(stepsPerBatch, remaining / stepMicroseconds);
            offlineFramePending = steps * stepMicroseconds == remaining;
            fluid.AdvanceManually(stepMicroseconds / 1000000f, steps, updateSurface: offlineFramePending);
            elapsedMicroseconds += steps * stepMicroseconds;
            if (offlineFramePending) StartCoroutine(CaptureOfflineFrame());
        }

        private IEnumerator CaptureOfflineFrame()
        {
            yield return new WaitForEndOfFrame();
            if (!offlineCapture) yield break;

            Texture2D frame = null;
            Texture2D cropped = null;
            try
            {
                frame = ScreenCapture.CaptureScreenshotAsTexture();
                int width = frame.width & ~1;
                int height = frame.height & ~1;
                if (width != frame.width || height != frame.height)
                {
                    cropped = new Texture2D(width, height, TextureFormat.RGBA32, false);
                    cropped.SetPixels(frame.GetPixels(0, 0, width, height));
                    cropped.Apply();
                }

                offlineEncoder ??= new MediaEncoder(offlineOutput, new VideoTrackAttributes
                {
                    frameRate = new MediaRational(1000000 / BenchmarkSampleInterval),
                    width = (uint)width,
                    height = (uint)height,
                    includeAlpha = false
                });
                if (!offlineEncoder.AddFrame(cropped != null ? cropped : frame))
                    throw new System.InvalidOperationException("Unity could not encode the frame. Keep the Game view at a fixed size during capture.");

                offlineFrames++;
                if (elapsedMicroseconds >= BenchmarkEndTime) FinishOfflineCapture(true);
            }
            catch (System.Exception error)
            {
                Debug.LogException(error, this);
                FinishOfflineCapture(false);
            }
            finally
            {
                offlineFramePending = false;
                if (cropped != null) Destroy(cropped);
                if (frame != null) Destroy(frame);
            }
        }

        private void FinishOfflineCapture(bool completed)
        {
            offlineCapture = offlineFramePending = false;
            offlineEncoder?.Dispose();
            offlineEncoder = null;
            if (!completed) return;

            lastOfflineVideo = offlineOutput;
            Debug.Log($"MARIN offline capture complete · {offlineFrames} frames · {lastOfflineVideo}", this);
            PlayOfflineVideo();
        }

        private void PlayOfflineVideo()
        {
            if (File.Exists(lastOfflineVideo))
                Application.OpenURL(new System.Uri(lastOfflineVideo).AbsoluteUri);
        }
#endif

        private void QueueBenchmarkRun(bool useWallPressure)
        {
            running = false;
            pendingBenchmarkRun = (useWallPressure, useWallPressure || !selectedBenchmarkMixed
                ? selectedBenchmarkSpacing : BenchmarkSpacings.Length - 1,
                selectedMeasurementsOnly, !useWallPressure && selectedBenchmarkMixed);
        }

        private void OnGUI()
        {
            if (solverPerformanceScene)
            {
                DrawPerformanceControls();
                return;
            }
#if UNITY_EDITOR
            if (offlineFramePending) return;
            if (offlineCapture)
            {
                GUILayout.BeginArea(new Rect(16, 16, 390, 90), GUI.skin.box);
                GUILayout.Label($"Rendering MARIN at 5 mm · {offlineFrames} / {BenchmarkEndTime / BenchmarkSampleInterval} frames");
                if (GUILayout.Button("Cancel offline capture")) FinishOfflineCapture(false);
                GUILayout.EndArea();
                return;
            }
#endif
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
                GUILayout.Label($"MARIN single-box benchmark · GPU · max step {BenchmarkStepMicroseconds / 1000f:0.###} ms");
                GUILayout.Label("Bulk-water sheet only · 20 ms samples in Console");
                string status = pendingBenchmarkRun.HasValue ? "Starting"
                    : benchmarkReadsPending > 0 ? "Waiting for GPU sample"
                    : running ? "Running" : elapsedMicroseconds >= BenchmarkEndTime ? "Finished"
                    : elapsedMicroseconds > 0 ? "Paused" : "Ready";
                GUILayout.Label($"{status} · submitted {elapsedMicroseconds / 1000000f:F3} / 1.800 s");
                if (SystemInfo.supportsAsyncGPUReadback)
                    GUILayout.Label($"Last completed sample · {benchmarkSampledSeconds:F3} s");
                GUILayout.Label($"H2 {benchmarkH2:F3} m · box zone {benchmarkSheet:F3} m · peak {benchmarkPeak:F3} m (p95)");
                GUILayout.Label($"Upstream jet {benchmarkJet:F3} m · peak {benchmarkJetPeak:F3} m (supported)");
                if (!SystemInfo.supportsAsyncGPUReadback) GUILayout.Label("GPU readback unavailable.");
                GUILayout.Label($"GPU wall pressure: {(benchmarkWallPressure ? "ON (1)" : "OFF (0)")}");
                bool controlsEnabled = GUI.enabled;
                GUI.enabled = controlsEnabled && !pendingBenchmarkRun.HasValue;
                GUILayout.Label("Particle spacing for next run (mm)");
                selectedBenchmarkSpacing = GUILayout.Toolbar(selectedBenchmarkSpacing, BenchmarkSpacingLabels);
                selectedMeasurementsOnly = GUILayout.Toggle(selectedMeasurementsOnly, "Measurements only · skip water rendering (next run)");
                GUILayout.Label($"Current spacing {BenchmarkSpacing * 1000:0.#} mm · {fluid.positionBuffer?.count ?? 0:N0} particles");
                if (benchmarkMeasurementsOnly) GUILayout.Label("Water hidden · timing and jet measurements are active");
                selectedBenchmarkMixed = GUILayout.Toggle(selectedBenchmarkMixed, "Mixed spacing · 5 mm surface / 10 mm bulk");
                if (benchmarkMixed) GUILayout.Label("Current run: mixed 5/10 mm");
                if (GUILayout.Button(selectedBenchmarkMixed ? "Start mixed run · wall pressure OFF" : "Start baseline run · wall pressure OFF"))
                    QueueBenchmarkRun(false);
                GUI.enabled = controlsEnabled && !pendingBenchmarkRun.HasValue && !selectedBenchmarkMixed;
                if (GUILayout.Button("Start wall-pressure run · ON"))
                    QueueBenchmarkRun(true);
                GUI.enabled = controlsEnabled && !pendingBenchmarkRun.HasValue;
                if (fluid.positionBuffer != null && (running || elapsedMicroseconds > 0) &&
                    elapsedMicroseconds < BenchmarkEndTime && GUILayout.Button(running ? "Pause" : "Resume"))
                {
                    running = !running;
                    pendingSeconds = 0;
                }
#if UNITY_EDITOR
                if (GUILayout.Button("Render 5 mm offline · 50 FPS"))
                {
                    running = false;
                    offlineQueued = true;
                }
                if (File.Exists(lastOfflineVideo) && GUILayout.Button("Play last 5 mm video"))
                    PlayOfflineVideo();
#endif
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
#if UNITY_EDITOR
            if (offlineCapture) FinishOfflineCapture(false);
#endif
            if (solverCompute != null) Destroy(solverCompute);
            foreach (var material in materials) Destroy(material);
        }
    }
}
