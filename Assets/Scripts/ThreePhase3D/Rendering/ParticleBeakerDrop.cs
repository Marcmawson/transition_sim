using System;
using System.Collections.Generic;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;
using PhysicsSim.ThreePhase3D.Geometry;
using PhysicsSim.ThreePhase3D.Materials;
using Seb.Fluid.Rendering;
using Seb.Fluid.Simulation;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.SceneManagement;

namespace PhysicsSim.ThreePhase3D.Rendering
{
    [RequireComponent(typeof(Beaker3DSetup))]
    [DefaultExecutionOrder(10)]
    public sealed class ParticleBeakerDrop : MonoBehaviour
    {
        [SerializeField] private bool showBoiling;
        private ParticleSystem vapour;
        private VapourPlume plume;
        private readonly ParticleSystem.Particle[] vapourMarkers = new ParticleSystem.Particle[4096];
        private Material vapourMaterial;
        private Vector3[] vapourSources;
        private const double VapourStepS = .05;
        private double vapourStepSeconds;
        private double pendingVapourKg;
        private const float VapourWhiteSpeedMps = .35f;
        private double initialLiquidDensity;
        private const float SmallVolumeMl = 9;
        private const float LargeVolumeMl = 250;
        private const int MaxMeshDropParticles = 100000;
        private const float DropGapM = .01f;
        private const int TargetFrameRate = 60;
        private const int BenchmarkMicroseconds = 400000;
        private const int CirculationMicroseconds = 30000000;
        private const double MaxPendingSeconds = 0.032;
        private enum SolverTimestep
        {
            [InspectorName("0.10 ms")] Ms010 = 100,
            [InspectorName("0.20 ms")] Ms020 = 200,
            [InspectorName("0.25 ms")] Ms025 = 250,
            [InspectorName("0.40 ms")] Ms040 = 400,
            [InspectorName("0.50 ms")] Ms050 = 500,
            [InspectorName("0.80 ms")] Ms080 = 800,
            [InspectorName("1.00 ms")] Ms100 = 1000
        }

        private static readonly SolverTimestep[] Timesteps =
            (SolverTimestep[])Enum.GetValues(typeof(SolverTimestep));
        [SerializeField] private ComputeShader fluidCompute;
        [SerializeField] private Shader particleShader;

        [Header("Release Settings")]
        [SerializeField, Tooltip("Use the 120 mm beaker and release 250 mL instead of 9 mL.")]
        private bool largerBeaker = false;

        [Header("Thermal Coupling")]
        [SerializeField] private bool heatMovingWater = false;
        [SerializeField] private bool thermalBuoyancy = true;
        [SerializeField, Range(0f, 100f)] private float absorbedBurnerWatts = 10f;
        [SerializeField, Range(20.1f, 100f)] private float hotColourC = 30f;

        [Header("Simulation Settings")]
        [SerializeField] private SolverTimestep solverTimestep = SolverTimestep.Ms040;
        [SerializeField, Range(5f, 100f)] private float playbackPercent = 25f;
        [SerializeField, Range(0f, 20f), Tooltip("Downward acceleration in m/s².")]
        private float gravity = 9.81f;
        [SerializeField, Range(0f, 6f)] private float pressureMultiplier = 2.88f;
        [SerializeField, Range(0f, .1f)] private float nearPressureMultiplier = .0216f;
        [SerializeField, Range(0f, 1f)] private float viscosityStrength;
        [SerializeField, Range(0f, 1f)] private float wallBounce = .2f;
        [SerializeField] private bool glassFloorPressure = true;

        [SerializeField, Range(1f, 3f), Tooltip("Set before Play. Changes the solver particle count and neighbourhood spacing.")] private float particleDensityMultiplier = 1.5f;
        [SerializeField] private Shader surfaceShader;
        [SerializeField] private ComputeShader surfaceArguments;
        private float spacingScale = 1;
        private FluidSim fluid;
        private BeakerFreeLookCamera beakerCamera;
        private ParticleDisplay3D particleDisplay;
        private Texture2D temperatureLegend, speedLegend;
        private Gradient temperatureColours, speedColours;
        private bool showTemperature;
        private static readonly string[] ColourViews = { "Temperature", "Speed" };
        private MovingBeakerHeat movingHeat;
        private ComputeShader computeInstance;
        private GameObject solverObject;
        private GeometryInputs geometry;
        private bool running, measuring, finishOnNextFrame, readbackPending;
        private double startTime, lastTick, pendingSeconds;
        private int measuredFrames;
        private long solverSteps;
        private int previousFrameRate, previousVSync;
        private string evidence;
        private float frameMs;
        private Action pendingPanelAction;
        private const int MaxMultiBatchStepsPerFrame = 8;
        private float playbackSpeed;
        private float measuredPlaybackSpeed, measuredBurnerWatts;
        private bool measuredBuoyancy, measuredFloorPressure;
        private int timestepIndex;
        private int benchmarkSteps;
        private Vector2 panelScroll;
        private MarchingCubesFluidRenderer waterSurface;
        private float waterTint;
        [SerializeField, Min(5f)] private float nextIceSideMm = 12f;
        private const float IceDensityKgM3 = 917f;
        private const float MeltwaterDensityKgM3 = 998.2f;
        private sealed class IceCube
        {
            public Rigidbody Body;
            public Transform Visual;
            public BoxCollider Collider;
            public float Side, Mass, Heat, YieldFraction, ReleasedFraction;
        }

        private readonly List<IceCube> iceCubes = new List<IceCube>();
        private Scene icePhysicsScene;
        private bool hasWater, samplingWaterline;
        private int waterlineVersion, waterBatches;
        private float waterline, sampledWaterline, nextWaterlineSample, waterVolumeMl;
        private Material iceMaterial;
        private PhysicMaterial iceContactMaterial;
        private float? previousBounceThreshold;
        private Mesh iceMesh;
        private GameObject iceContacts;
        private float iceSpawnY, iceFloor;
        private const float IceStartC = -5f;
        private const float IceHeatCapacity = 2050f;
        private const float MeltReleaseIncrement = .125f;
        private const float IceLatentHeat = 333550f;

        private int StepMicroseconds => (int)Timesteps[timestepIndex];
        private double StepSeconds => StepMicroseconds / 1_000_000.0;
        [SerializeField, Range(1f, BeakerRenderMockup.MaxMarkedVolumeMl), Tooltip("Cylindrical release volume in mL. Set before pressing Play.")]
        private float meshReleaseVolumeMl = 9f;
        [SerializeField, Range(.6f, 1.2f), Tooltip("Settled fill calibration for the mesh drop. Lower values reduce particle packing density and raise the waterline.")]
        private float meshPackingDensityScale = .825f;
        [SerializeField, Range(0f, 150f), Tooltip("Height in millimetres from the inside floor of the beaker to the bottom of the water cylinder. Set before Play.")]
        private float spawnHeightAboveBottomMm = 100f;
        private float releaseVolumeMl;
        private Vector3 releaseViewCentre;
        private float releaseViewHeight;

        private void OnEnable()
        {
            previousFrameRate = Application.targetFrameRate;
            previousVSync = QualitySettings.vSyncCount;
            QualitySettings.vSyncCount = 0;
            Application.targetFrameRate = TargetFrameRate;
        }

        private void OnDisable()
        {
            Application.targetFrameRate = previousFrameRate;
            QualitySettings.vSyncCount = previousVSync;
            running = measuring = finishOnNextFrame = false;
            pendingSeconds = 0;
        }

        private void Start()
        {
            if (!SystemInfo.supportsComputeShaders || fluidCompute == null || particleShader == null)
            {
                evidence = "The GPU particle solver or its shaders are unavailable.";
                return;
            }

            beakerCamera = FindObjectOfType<BeakerFreeLookCamera>();
            var setup = GetComponent<Beaker3DSetup>();
            geometry = setup.scenario.physical.geometry;
            releaseVolumeMl = largerBeaker ? LargeVolumeMl : SmallVolumeMl;
            // The render mock-up sets up the shared vessel before this component starts.
            // Refreshing it here would replace its glass and tripod meshes.
            if (!TryGetComponent<BeakerRenderMockup>(out var mockup) || !mockup.isActiveAndEnabled)
            {
                geometry.innerRadiusM = largerBeaker ? .06 : .033;
                geometry.cavityHeightM = largerBeaker ? .16 : .06;
                geometry.tripodRingRadiusM = geometry.OuterRadiusM * .75;
                setup.scenario.display.glassOpacity = .28f;
                setup.scenario.display.showSetupReport = false;
                setup.Refresh();
            }

            bool meshWater = surfaceShader != null && surfaceArguments != null && !heatMovingWater;
            float floor = (float)(geometry.externalBaseHeightM + geometry.baseThicknessM);
            float rim = (float)geometry.RimHeightM;
            if (meshWater) releaseVolumeMl = Mathf.Clamp(meshReleaseVolumeMl, 1f, BeakerRenderMockup.MaxMarkedVolumeMl);
            float volumeM3 = releaseVolumeMl * 1e-6f;
            float side = Mathf.Pow(volumeM3, 1f / 3f);
            float cylinderRadius = Mathf.Min((float)geometry.innerRadiusM * .9f,
                Mathf.Pow(volumeM3 / (2 * Mathf.PI), 1f / 3f));
            float cylinderHeight = volumeM3 / (Mathf.PI * cylinderRadius * cylinderRadius);
            float dropBottom = meshWater ? floor + Mathf.Clamp(spawnHeightAboveBottomMm, 0f, 150f) * .001f : floor + DropGapM;
            if (meshWater)
            {
                iceFloor = floor;
                iceSpawnY = dropBottom;
                float apparatusTop = dropBottom + cylinderHeight;
                releaseViewCentre = Vector3.up * (apparatusTop / 2f);
                releaseViewHeight = apparatusTop;
            }
            solverObject = new GameObject("GPU liquid particles");
            solverObject.SetActive(false);
            solverObject.transform.position = new Vector3(0, .175f, 0);
            solverObject.transform.localScale = new Vector3(.3f, .35f, .3f);
            if (meshWater)
            {
                float width = (float)geometry.OuterRadiusM * 2 + .018f;
                float height = dropBottom + cylinderHeight - floor + .024f;
                solverObject.transform.position = Vector3.up * (floor + height / 2 - .012f);
                solverObject.transform.localScale = new Vector3(width, height, width);
            }
            var spawner = solverObject.AddComponent<Spawner3D>();
            // Keep full-scale releases within the same approximate GPU particle budget.
            spawner.particleSpawnDensity = meshWater
                ? Mathf.RoundToInt(Mathf.Min(700000000f * particleDensityMultiplier, MaxMeshDropParticles / volumeM3))
                : 700000000;
            spawner.spawnRegions = new[]
            {
                new Spawner3D.SpawnRegion
                {
                    centre = new Vector3(0, dropBottom + (meshWater ? cylinderHeight : side) / 2, 0),
                    size = side,
                    cylinderRadius = meshWater ? cylinderRadius : 0,
                    cylinderHeight = meshWater ? cylinderHeight : 0
                }
            };

            if (meshWater)
                spacingScale = Mathf.Pow(700000000f / spawner.particleSpawnDensity, 1f / 3f);
            spawner.jitterStrength = .00018f * spacingScale;

            computeInstance = Instantiate(fluidCompute);
            fluid = solverObject.AddComponent<FluidSim>();
            fluid.compute = computeInstance;
            fluid.spawner = spawner;
            fluid.manualControl = true;
            fluid.transportTemperature = heatMovingWater;
            fluid.trackLiquidMass = showBoiling && heatMovingWater;
            if (fluid.trackLiquidMass)
                initialLiquidDensity = new Water95().Evaluate(293.15, 101325, WaterBranch.Liquid).DensityKgM3;
            if (heatMovingWater)
            {
                fluid.buoyancyDensityByDegree = BuildBuoyancyDensityTable();
                var water = new Water95();
                var state = water.Evaluate(293.15, 101325, WaterBranch.Liquid);
                fluid.liquidThermalDiffusivity = (float)(water.ConductivityWmK(state) /
                    (state.DensityKgM3 * state.CpJkgK));
            }
            // Start from the demo's liquid parameters scaled to metre-sized particles.
            fluid.smoothingRadius = .002f * spacingScale;
            fluid.targetDensity = meshWater
                ? InteriorSpawnDensity(fluid.smoothingRadius, cylinderRadius, cylinderHeight, spawner.particleSpawnDensity) * meshPackingDensityScale
                : 630e6f;
            // Start the mesh drop with modest pressure and neighbour damping.
            // The legacy particle scene keeps its own tuning.
            if (meshWater)
            {
                pressureMultiplier = 2.7f;
                nearPressureMultiplier = .0433f;
                viscosityStrength = .00000002f;
            }
            ApplySceneSettings();
            fluid.maxFoamParticleCount = 1;
            float particleRadiusM = .0004f * spacingScale;
            fluid.beakerCollision = new Vector4(0, 0,
                (float)geometry.innerRadiusM * Mathf.Cos(Mathf.PI / setup.scenario.display.cylinderSegments) - particleRadiusM,
                floor + particleRadiusM);
            fluid.beakerRimY = rim;
            fluid.beakerGlassFloorY = floor;

            var display = particleDisplay = solverObject.AddComponent<ParticleDisplay3D>();
            display.sim = fluid;
            display.mode = ParticleDisplay3D.DisplayMode.Billboard;
            display.shaderBillboard = particleShader;
            display.scale = particleRadiusM * 100;
            display.gradientResolution = 32;
            display.velocityDisplayMax = .65f;
            showTemperature = heatMovingWater;
            speedColours = CreateColourMap(Color.white);
            display.colourMap = speedColours;
            ParticleDisplay3D.TextureFromGradient(ref speedLegend, 64, speedColours);
            if (heatMovingWater)
            {
                temperatureColours = CreateColourMap(new Color(1f, .08f, .04f));
                display.colourMap = temperatureColours;
                display.temperatureRiseDisplayMax = hotColourC - 20f;
                ParticleDisplay3D.TextureFromGradient(ref temperatureLegend, 64, temperatureColours);
            }
            if (meshWater)
            {
                display.enabled = false;
                fluid.renderToTex3D = true;
                fluid.worldSpaceSurface = true;
                fluid.densityTextureRes = 96;
                var surface = solverObject.AddComponent<MarchingCubesFluidRenderer>();
                surface.sim = fluid;
                surface.drawShader = surfaceShader;
                surface.renderArgsCompute = Instantiate(surfaceArguments);
                surface.worldSpaceSurface = true;
                surface.isoLevel = fluid.targetDensity * .45f;
                waterSurface = surface;
                waterSurface.col = new Color(1f, 1f, 1f, waterTint);
            }
            solverObject.SetActive(true);
            hasWater = !meshWater;
            waterBatches = hasWater ? 1 : 0;
            waterVolumeMl = hasWater ? releaseVolumeMl : 0;
            if (meshWater) ClearBeaker();
            SetRunning(!meshWater);
            if (meshWater) beakerCamera?.FrameRelease(releaseViewCentre, releaseViewHeight);
        }

        private static float InteriorSpawnDensity(float radius, float cylinderRadius, float height, int particleDensity)
        {
            float spacing = Mathf.Pow(1f / particleDensity, 1f / 3f);
            float horizontal = 2 * cylinderRadius / Mathf.Max(1, Mathf.RoundToInt(2 * cylinderRadius / spacing));
            float vertical = height / Mathf.Max(1, Mathf.RoundToInt(height / spacing));
            float sum = 0;
            int across = Mathf.CeilToInt(radius / horizontal);
            int up = Mathf.CeilToInt(radius / vertical);
            for (int x = -across; x <= across; x++)
            for (int y = -up; y <= up; y++)
            for (int z = -across; z <= across; z++)
            {
                float distance = new Vector3(x * horizontal, y * vertical, z * horizontal).magnitude;
                float support = Mathf.Max(0, radius - distance);
                sum += support * support;
            }
            return sum * 15 / (2 * Mathf.PI * Mathf.Pow(radius, 5));
        }

        private static float[] BuildBuoyancyDensityTable()
        {
            var water = new Water95();
            var density = new float[76];
            for (int i = 0; i < density.Length; i++)
                density[i] = (float)water.Evaluate(293.15 + i, 101325, WaterBranch.Liquid).DensityKgM3;
            return density;
        }

        private static Gradient CreateColourMap(Color endpoint)
        {
            var gradient = new Gradient();
            gradient.SetKeys(new[] { new GradientColorKey(new Color(.05f, .55f, .88f), 0),
                new GradientColorKey(endpoint, 1) }, new[] { new GradientAlphaKey(1, 0), new GradientAlphaKey(1, 1) });
            return gradient;
        }

        private void Update()
        {
            float elapsedMs = Time.unscaledDeltaTime * 1000f;
            if (elapsedMs > 0) frameMs = frameMs == 0 ? elapsedMs : Mathf.Lerp(frameMs, elapsedMs, .1f);
            if (finishOnNextFrame)
            {
                finishOnNextFrame = false;
                FinishMeasurement();
            }
            if (fluid == null || fluid.positionBuffer == null) return;
            if (pendingPanelAction != null)
            {
                if (samplingWaterline) return;
                var action = pendingPanelAction;
                pendingPanelAction = null;
                action();
                lastTick = Time.realtimeSinceStartupAsDouble;
                return;
            }
            if (heatMovingWater && movingHeat == null)
            {
                try { movingHeat = new MovingBeakerHeat(fluid, geometry, releaseVolumeMl * 1e-6, showBoiling); }
                catch (Exception exception)
                {
                    evidence = exception.Message;
                    Debug.LogException(exception, this);
                    enabled = false;
                    return;
                }
            }
            movingHeat?.SampleIfDue(Time.realtimeSinceStartupAsDouble);
            if (!running) return;
            double now = Time.realtimeSinceStartupAsDouble;
            double stepSeconds = StepSeconds;
            float speed = measuring ? measuredPlaybackSpeed : playbackSpeed;
            pendingSeconds = Math.Min(pendingSeconds + (now - lastTick) * speed, MaxPendingSeconds);
            lastTick = now;
            int steps = (int)(pendingSeconds / stepSeconds);
            if (measuring)
            {
                measuredFrames++;
                steps = (int)Math.Min(steps, benchmarkSteps - solverSteps);
            }
            // Extra water increases the work per GPU step. Prevent slow frames from
            // scheduling an ever larger catch-up burst while keeping the same timestep.
            if (waterSurface != null && (waterBatches > 1 || iceCubes.Count > 1) && !measuring)
                steps = Math.Min(steps, MaxMultiBatchStepsPerFrame);
            if (steps == 0) return;
            if (iceCubes.Count > 0)
            {
                float dt = (float)stepSeconds;
                for (int remaining = steps; remaining > 0; remaining--)
                {
                    if (iceCubes.Count > 0)
                    {
                        float blend = 1 - Mathf.Exp(-dt / .12f);
                        waterline = Mathf.Lerp(waterline, sampledWaterline, blend);
                        foreach (var cube in iceCubes) ApplyIceForces(cube);
                        icePhysicsScene.GetPhysicsScene().Simulate(dt);
                        for (int i = iceCubes.Count - 1; i >= 0; i--) AdvanceIce(i, dt);
                    }
                    UpdateIceWaterContact();
                    if (hasWater)
                    {
                        AdvanceDrop(stepSeconds, 1, remaining == 1);
                        solverSteps++;
                    }
                }
            }
            else if (hasWater)
            {
                UpdateIceWaterContact();
                AdvanceDrop(stepSeconds, steps);
                solverSteps += steps;
            }
            pendingSeconds -= steps * stepSeconds;
            if (measuring && solverSteps == benchmarkSteps)
            {
                running = false;
                finishOnNextFrame = true;
            }
        }

        private void AdvanceDrop(double stepSeconds, int steps, bool updateSurface = true)
        {
            fluid.thermalBuoyancy = measuring ? measuredBuoyancy : thermalBuoyancy;
            fluid.beakerFloorPressure = measuring ? measuredFloorPressure : glassFloorPressure;
            if (fluid.worldSpaceSurface && beakerCamera != null)
            {
                float distance = Vector3.Distance(beakerCamera.transform.position, solverObject.transform.position);
                fluid.densityTextureRes = fluid.densityTextureRes == 96
                    ? (distance > .65f ? 56 : 96)
                    : (distance < .55f ? 96 : 56);
            }
            fluid.AdvanceManually((float)stepSeconds, steps, updateSurface);
            double dt = stepSeconds * steps;
            movingHeat?.Advance(dt, measuring ? measuredBurnerWatts : absorbedBurnerWatts);
            if (showBoiling && movingHeat != null)
            {
                if (vapour == null) CreateVapourDisplay();
                vapourStepSeconds += dt;
                while (vapourStepSeconds >= VapourStepS)
                {
                    vapourStepSeconds -= VapourStepS;
                    movingHeat.CollectVapour(vapourSources, EmitVapour);
                    try
                    {
                        if (movingHeat.VapourGeneratedKg > 0) plume.Step(VapourStepS);
                    }
                    catch (FlowConvergenceException error)
                    {
                        running = false;
                        measuring = false;
                        evidence = "Vapour pressure solve paused: " + error.Message;
                        Debug.LogException(error, this);
                        break;
                    }
                }
                int count = vapour.GetParticles(vapourMarkers);
                for (int i = 0; i < count; i++)
                    vapourMarkers[i].velocity = plume.VelocityAt(vapourMarkers[i].position);
                vapour.SetParticles(vapourMarkers, count);
                vapour.Simulate((float)dt, false, false, false);
            }
        }

        private void ResetHeat()
        {
            movingHeat?.Reset();
            vapour?.Clear();
            plume?.Reset();
            pendingVapourKg = vapourStepSeconds = 0;
        }

        private void CreateVapourDisplay()
        {
            plume = new VapourPlume(geometry);
            var display = new GameObject("Vapour parcels");
            display.transform.SetParent(transform, false);
            vapour = display.AddComponent<ParticleSystem>();
            vapour.Stop(false, ParticleSystemStopBehavior.StopEmittingAndClear);
            var main = vapour.main;
            main.playOnAwake = false;
            main.simulationSpace = ParticleSystemSimulationSpace.World;
            main.maxParticles = 4096;
            main.startLifetime = 2;
            main.startSize = .0005f;
            main.startColor = Color.white;
            var renderer = display.GetComponent<ParticleSystemRenderer>();
            renderer.renderMode = ParticleSystemRenderMode.Billboard;
            renderer.minParticleSize = 0;
            var shader = Shader.Find("Particles/Standard Unlit");
            if (shader != null)
            {
                vapourMaterial = new Material(shader);
                renderer.sharedMaterial = vapourMaterial;
            }
            var emission = vapour.emission;
            emission.enabled = false;
            var colours = vapour.colorBySpeed;
            colours.enabled = true;
            colours.range = new Vector2(0, VapourWhiteSpeedMps);
            var gradient = new Gradient();
            gradient.SetKeys(new[] { new GradientColorKey(new Color(.55f, .8f, 1), 0),
                new GradientColorKey(Color.white, 1) },
                new[] { new GradientAlphaKey(.65f, 0), new GradientAlphaKey(.65f, 1) });
            colours.color = new ParticleSystem.MinMaxGradient(gradient);
            vapourSources = new Vector3[fluid.positionBuffer.count];
        }

        private void EmitVapour(Vector3 origin, float massKg)
        {
            const double displayParcelKg = 1e-8;
            origin.y = Mathf.Max(origin.y, (float)geometry.RimHeightM + .0001f);
            plume.Inject(origin, massKg);
            pendingVapourKg += massKg;
            int count = (int)Math.Min(32, pendingVapourKg / displayParcelKg);
            if (count == 0) return;
            pendingVapourKg -= count * displayParcelKg;
            vapour.Emit(new ParticleSystem.EmitParams { position = origin, velocity = plume.VelocityAt(origin) }, count);
        }

        private void SetRunning(bool value)
        {
            running = value;
            foreach (var cube in iceCubes)
            {
                var ice = cube.Body;
                ice.isKinematic = !value;
                ice.collisionDetectionMode = value
                    ? CollisionDetectionMode.ContinuousDynamic : CollisionDetectionMode.ContinuousSpeculative;
            }
            pendingSeconds = 0;
            lastTick = Time.realtimeSinceStartupAsDouble;
        }

        private void ClearBeaker()
        {
            hasWater = false;
            waterBatches = 0;
            waterVolumeMl = 0;
            waterlineVersion++;
            waterline = sampledWaterline = iceFloor;
            fluid.IceCount = 0;
            foreach (var cube in iceCubes)
            {
                if (cube.Body == null) continue;
                cube.Body.gameObject.SetActive(false);
                Destroy(cube.Body.gameObject);
            }
            iceCubes.Clear();
            if (iceContacts != null) iceContacts.SetActive(false);
            waterSurface.enabled = false;
            RestoreBounceThreshold();
        }

        private void SpawnIce()
        {
            var state = new IceCube { Side = SelectedIceSideMm * .001f,
                YieldFraction = SelectedIceYieldMl / releaseVolumeMl };
            float iceSide = state.Side;
            state.Mass = IceDensityKgM3 * iceSide * iceSide * iceSide;
            if (!previousBounceThreshold.HasValue)
            {
                previousBounceThreshold = Physics.bounceThreshold;
                Physics.bounceThreshold = Mathf.Min(Physics.bounceThreshold, .3f);
            }

            if (iceContacts == null)
            {
                icePhysicsScene = SceneManager.CreateScene($"Beaker ice physics {GetInstanceID()}",
                    new CreateSceneParameters(LocalPhysicsMode.Physics3D));
                iceContacts = new GameObject("Ice beaker contacts");
                SceneManager.MoveGameObjectToScene(iceContacts, icePhysicsScene);
                float radius = (float)geometry.innerRadiusM;
                float height = (float)geometry.cavityHeightM;
                AddContact(Vector3.up * (iceFloor - .005f), new Vector3(2 * radius + .008f, .01f, 2 * radius + .008f), 0);
                const int wallCount = 24;
                for (int i = 0; i < wallCount; i++)
                {
                    float angle = i * 360f / wallCount;
                    Vector3 radial = Quaternion.Euler(0, angle, 0) * Vector3.forward;
                    AddContact(Vector3.up * (iceFloor + height * .5f) + radial * (radius + .002f),
                        new Vector3(2 * (radius + .004f) * Mathf.Tan(Mathf.PI / wallCount), height, .004f), angle);
                }
            }

            var cube = new GameObject("Ice cube");
            SceneManager.MoveGameObjectToScene(cube, icePhysicsScene);
            float spawnY = Mathf.Max(iceSpawnY + iceSide * .68f, (float)geometry.RimHeightM + iceSide);
            foreach (var existing in iceCubes)
                spawnY = Mathf.Max(spawnY, existing.Collider.bounds.max.y + iceSide + .002f);
            cube.transform.SetPositionAndRotation(Vector3.up * spawnY, Quaternion.Euler(13, 19, 9));
            var mesh = GameObject.CreatePrimitive(PrimitiveType.Cube);
            mesh.name = "Ice cube mesh";
            if (iceMesh == null) iceMesh = BuildIceMesh();
            mesh.GetComponent<MeshFilter>().sharedMesh = iceMesh;
            mesh.transform.SetParent(cube.transform, false);
            mesh.transform.localScale = Vector3.one * iceSide;
            state.Visual = mesh.transform;
            mesh.GetComponent<BoxCollider>().enabled = false;
            var iceCollider = state.Collider = cube.AddComponent<BoxCollider>();
            iceCollider.size = Vector3.one * iceSide;
            iceCollider.contactOffset = .0003f;
            if (iceContactMaterial == null)
            {
                iceContactMaterial = new PhysicMaterial("Ice on glass")
                {
                    bounciness = .52f,
                    bounceCombine = PhysicMaterialCombine.Maximum,
                    staticFriction = .14f,
                    dynamicFriction = .09f,
                    frictionCombine = PhysicMaterialCombine.Minimum
                };
            }
            iceCollider.sharedMaterial = iceContactMaterial;
            if (iceMaterial == null)
            {
                var shader = Resources.Load<Shader>("ThreePhase3D/BeakerIceSurface");
                iceMaterial = shader != null && shader.isSupported
                    ? new Material(shader)
                    : new Material(Shader.Find("Standard")) { color = new Color(.65f, .78f, .84f) };
            }
            mesh.GetComponent<MeshRenderer>().sharedMaterial = iceMaterial;
            var ice = state.Body = cube.AddComponent<Rigidbody>();
            ice.mass = state.Mass;
            ice.useGravity = false;
            ice.interpolation = RigidbodyInterpolation.Interpolate;
            ice.isKinematic = true;
            ice.collisionDetectionMode = CollisionDetectionMode.ContinuousSpeculative;
            ice.centerOfMass = iceSide * new Vector3(.08f, -.05f, .06f);
            iceCubes.Add(state);
            if (iceContacts != null) iceContacts.SetActive(true);
            SetRunning(running);
        }

        private static Mesh BuildIceMesh()
        {
            const int segments = 16;
            var vertices = new System.Collections.Generic.List<Vector3>();
            var triangles = new System.Collections.Generic.List<int>();
            foreach (var normal in new[] { Vector3.up, Vector3.down, Vector3.right,
                Vector3.left, Vector3.forward, Vector3.back })
            {
                Vector3 tangent = Mathf.Abs(normal.y) > .5f ? Vector3.right : Vector3.up;
                Vector3 bitangent = Vector3.Cross(normal, tangent);
                int first = vertices.Count;
                for (int row = 0; row <= segments; row++)
                for (int column = 0; column <= segments; column++)
                {
                    Vector3 p = normal * .5f + tangent * ((float)column / segments - .5f)
                        + bitangent * ((float)row / segments - .5f);
                    Vector3 inset = new Vector3(Mathf.Clamp(p.x, -.45f, .45f),
                        Mathf.Clamp(p.y, -.45f, .45f), Mathf.Clamp(p.z, -.45f, .45f));
                    float ripple = Mathf.Sin(19 * p.x + 9 * p.y) * Mathf.Sin(23 * p.z - 13 * p.y);
                    vertices.Add(inset + (p - inset).normalized * (.047f + .003f * ripple));
                    if (row == segments || column == segments) continue;
                    int a = first + row * (segments + 1) + column;
                    int b = a + segments + 1;
                    triangles.AddRange(new[] { a, a + 1, b, a + 1, b + 1, b });
                }
            }
            var mesh = new Mesh { name = "Irregular bevelled ice" };
            mesh.SetVertices(vertices);
            mesh.SetTriangles(triangles, 0);
            mesh.RecalculateNormals();
            return mesh;
        }

        private void AddContact(Vector3 centre, Vector3 size, float yaw)
        {
            var contact = new GameObject("Glass contact");
            contact.transform.SetParent(iceContacts.transform, false);
            contact.transform.SetPositionAndRotation(centre, Quaternion.Euler(0, yaw, 0));
            var collider = contact.AddComponent<BoxCollider>();
            collider.size = size;
            collider.contactOffset = .0003f;
        }

        private void ApplyIceForces(IceCube cube)
        {
            var ice = cube.Body;
            ice.AddForce(Vector3.up * fluid.gravity, ForceMode.Acceleration);
            if (!hasWater || (sampledWaterline <= iceFloor && waterline <= iceFloor)) return;
            if (waterline - iceFloor < .0001f) return;

            // Integrate the submerged box volume with evenly spaced sample points.
            const int samples = 4;
            float side = cube.Collider.size.x;
            float volumePerSample = side * side * side / (samples * samples * samples);
            float forcePerSample = 998.2f * volumePerSample * Mathf.Abs(fluid.gravity);
            for (int x = 0; x < samples; x++)
            for (int y = 0; y < samples; y++)
            for (int z = 0; z < samples; z++)
            {
                var local = new Vector3(x + .5f, y + .5f, z + .5f) / samples - Vector3.one * .5f;
                Vector3 point = ice.transform.TransformPoint(local * side);
                if (point.x * point.x + point.z * point.z > fluid.beakerCollision.z * fluid.beakerCollision.z) continue;
                float fraction = Mathf.Clamp01(.5f + (waterline - point.y) * samples / side);
                Vector3 velocity = ice.GetPointVelocity(point);
                float dragFactor = (ice.mass * 2f + .5f * 998.2f * side * side * velocity.magnitude)
                    / (samples * samples * samples);
                Vector3 drag = velocity * dragFactor;
                ice.AddForceAtPosition((Vector3.up * forcePerSample - drag) * fraction, point);
            }
        }

        private void UpdateIceWaterContact()
        {
            fluid.IceCount = hasWater ? iceCubes.Count : 0;
            for (int i = 0; i < fluid.IceCount; i++)
            {
                var cube = iceCubes[i];
                fluid.SetIceBoundary(i, cube.Body, cube.Collider.size.x * .5f);
            }
            if (fluid.IceCount == 0) return;

            if (samplingWaterline || Time.unscaledTime < nextWaterlineSample) return;
            samplingWaterline = true;
            nextWaterlineSample = Time.unscaledTime + .05f;
            int version = waterlineVersion;
            float rim = (float)geometry.RimHeightM;
            float radius = fluid.beakerCollision.z;
            AsyncGPUReadback.Request(fluid.positionBuffer, request =>
            {
                if (this == null) return;
                samplingWaterline = false;
                if (request.hasError || version != waterlineVersion) return;
                var points = request.GetData<Vector3>();
                var heights = new int[48];
                int count = 0;
                for (int i = 0; i < points.Length; i += 8)
                {
                    Vector3 point = points[i];
                    if (!(point.y >= iceFloor && point.y < rim &&
                        point.x * point.x + point.z * point.z < radius * radius)) continue;
                    int bin = Mathf.Clamp((int)((point.y - iceFloor) / (rim - iceFloor) * heights.Length),
                        0, heights.Length - 1);
                    heights[bin]++;
                    count++;
                }
                int bottomCount = heights[0] + heights[1] + heights[2];
                if (count == 0 || bottomCount < Mathf.Max(2, count / 50))
                {
                    sampledWaterline = iceFloor;
                    return;
                }
                int accumulated = 0;
                for (int i = 0; i < heights.Length; i++)
                {
                    accumulated += heights[i];
                    if (accumulated < count * .97f) continue;
                    sampledWaterline = iceFloor + (rim - iceFloor) * (i + .5f) / heights.Length;
                    break;
                }
            });
        }

        private float MaxIceSideMm => Mathf.Min((float)geometry.innerRadiusM * 1000f,
            1000f * Mathf.Pow(releaseVolumeMl * 1e-6f * MeltwaterDensityKgM3 / IceDensityKgM3, 1f / 3f));

        private float SelectedIceSideMm => Mathf.Clamp(nextIceSideMm, 5f, MaxIceSideMm);

        private float SelectedIceYieldMl
        {
            get
            {
                float side = SelectedIceSideMm * .001f;
                return IceDensityKgM3 * side * side * side / MeltwaterDensityKgM3 * 1e6f;
            }
        }

        private void SpawnWater(Vector3 centre, float volumeFraction = 1, bool fromIce = false,
            float puddleRadius = 0, float puddleHeight = 0)
        {
            waterlineVersion++;
            fluid.AddWater(centre, hasWater, volumeFraction, puddleRadius, puddleHeight);
            waterVolumeMl += releaseVolumeMl * volumeFraction;
            hasWater = true;
            // Meltwater increments share the same pool and are not full water releases.
            if (!fromIce) waterBatches++;
            waterSurface.enabled = true;
        }

        private void AdvanceIce(int index, float seconds)
        {
            var cube = iceCubes[index];
            var ice = cube.Body;
            var iceCollider = cube.Collider;
            float iceMass = cube.Mass;
            float iceSide = cube.Side;
            // Approximate room heating and apply the selected absorbed burner power near the floor.
            float warmup = iceMass * IceHeatCapacity * -IceStartC;
            float latent = iceMass * IceLatentHeat;
            float temperature = Mathf.Min(0, IceStartC + cube.Heat / (iceMass * IceHeatCapacity));
            float area = 6 * iceCollider.size.x * iceCollider.size.x;
            float burner = ice.position.y - iceFloor < iceSide ? absorbedBurnerWatts / iceCubes.Count : 0;
            cube.Heat += (8f * area * (20 - temperature) + burner) * seconds;
            float melted = Mathf.Clamp01((cube.Heat - warmup) / latent);
            float remaining = 1 - melted;
            if (remaining > 0)
            {
                float side = iceSide * Mathf.Pow(remaining, 1f / 3f);
                if (side < iceCollider.size.x - .0001f)
                {
                    cube.Visual.localScale = Vector3.one * side;
                    iceCollider.size = Vector3.one * side;
                    ice.mass = iceMass * remaining;
                    ice.ResetInertiaTensor();
                    ice.centerOfMass = side * new Vector3(.08f, -.05f, .06f);
                }
            }

            float increment = melted - cube.ReleasedFraction;
            if (samplingWaterline || readbackPending || increment <= 0 ||
                (melted < 1 && increment < MeltReleaseIncrement)) return;

            var region = fluid.spawner.spawnRegions[0];
            float fraction = cube.YieldFraction * increment;
            if (!TryReleaseContactPuddle(iceCollider, fraction))
                ReleaseAirborneMeltwater(ice, iceCollider, fraction, region);
            cube.ReleasedFraction = melted;

            if (melted < 1) return;
            ice.gameObject.SetActive(false);
            Destroy(ice.gameObject);
            iceCubes.RemoveAt(index);
            fluid.IceCount = 0;
            if (iceCubes.Count == 0) RestoreBounceThreshold();
        }

        private bool TryReleaseContactPuddle(BoxCollider collider, float fraction)
        {
            float half = collider.size.x * .5f;
            var body = collider.attachedRigidbody;
            Vector3 down = Quaternion.Inverse(body.rotation) * Vector3.down;
            // A downward-facing axis selects the low face. Neutral axes keep the
            // midpoint of a contact edge or the centre of a contact face.
            Vector3 local = new Vector3(ContactAxis(down.x), ContactAxis(down.y), ContactAxis(down.z)) * half;
            Vector3 contact = body.position + body.rotation * local;
            if (contact.y > iceFloor + .001f) return false;

            float volume = releaseVolumeMl * 1e-6f * fraction;
            float depth = Mathf.Min(.002f, Mathf.Pow(volume, 1f / 3f) * .4f);
            float innerRadius = fluid.beakerCollision.z;
            float radius = Mathf.Min(innerRadius * .75f, Mathf.Sqrt(volume / (Mathf.PI * depth)));
            depth = volume / (Mathf.PI * radius * radius);
            Vector2 centre = Vector2.ClampMagnitude(new Vector2(contact.x, contact.z),
                Mathf.Max(0, innerRadius - radius - .0002f));
            SpawnWater(new Vector3(centre.x, fluid.beakerCollision.w + depth * .5f, centre.y),
                fraction, fromIce: true, puddleRadius: radius, puddleHeight: depth);
            return true;
        }

        private static float ContactAxis(float direction) =>
            Mathf.Abs(direction) < .005f ? 0 : Mathf.Sign(direction);

        private void ReleaseAirborneMeltwater(Rigidbody ice, BoxCollider iceCollider,
            float fraction, Spawner3D.SpawnRegion region)
        {
            float scale = Mathf.Pow(fraction, 1f / 3f);
            float halfHeight = region.cylinderHeight * scale * .5f;
            float clearance = iceCollider.size.x * .87f + region.cylinderRadius * scale + .001f;
            Vector2 horizontal = new Vector2(ice.position.x, ice.position.z);
            Vector2 direction = horizontal.sqrMagnitude > 1e-8f ? -horizontal.normalized : Vector2.right;
            horizontal = Vector2.ClampMagnitude(horizontal + direction * clearance,
                Mathf.Max(0, fluid.beakerCollision.z - region.cylinderRadius * scale - .001f));
            float poolHeight = Mathf.Max(iceFloor, sampledWaterline);
            float height = Mathf.Clamp(ice.position.y, poolHeight + halfHeight + .001f,
                Mathf.Max(poolHeight + halfHeight + .001f, region.centre.y));
            SpawnWater(new Vector3(horizontal.x, height, horizontal.y), fraction, fromIce: true);
        }

        private void StartMeasurement(int durationMicroseconds = BenchmarkMicroseconds)
        {
            fluid.ResetParticles();
            ResetHeat();
            evidence = null;
            measuredFrames = 0;
            solverSteps = 0;
            measuredPlaybackSpeed = playbackSpeed;
            measuredBurnerWatts = absorbedBurnerWatts;
            measuredBuoyancy = thermalBuoyancy;
            measuredFloorPressure = glassFloorPressure;
            benchmarkSteps = durationMicroseconds / StepMicroseconds;
            SetRunning(true);
            startTime = lastTick;
            measuring = true;
        }

        private void FinishMeasurement()
        {
            measuring = false;
            double elapsed = Time.realtimeSinceStartupAsDouble - startTime;
            string timing = FormattableString.Invariant(
                $"GPU drop: {measuredFrames} frames, {measuredFrames / elapsed:F1} FPS, {fluid.SimulatedSeconds:F3} s simulated in {elapsed:F3} s at {measuredPlaybackSpeed:P0} playback · {StepMicroseconds / 1000.0:0.##} ms per step · {solverSteps} solver steps · {releaseVolumeMl:F0} mL release");
            timing += $" · floor pressure {(measuredFloorPressure ? "on" : "off")}";
            if (movingHeat != null) timing += $" · {measuredBurnerWatts:F0} W · buoyancy {(measuredBuoyancy ? "on" : "off")} · " + movingHeat.Measure();
            evidence = timing;
            MeasureContainedWater(timing);
        }

        private void MeasureContainedWater(string timing)
        {
            if (!SystemInfo.supportsAsyncGPUReadback)
            {
                evidence = timing + " · GPU position readback unavailable";
                Debug.Log(evidence, this);
                return;
            }
            readbackPending = true;
            float floor = fluid.beakerCollision.w;
            float glassFloor = fluid.beakerGlassFloorY;
            float graduationRadius = (float)geometry.innerRadiusM;
            float rim = fluid.beakerRimY;
            float radius = fluid.beakerCollision.z;
            AsyncGPUReadback.Request(fluid.positionBuffer, result =>
            {
                if (this == null) return;
                readbackPending = false;
                if (result.hasError)
                {
                    evidence = timing + " · position snapshot failed";
                    Debug.LogWarning(evidence, this);
                    return;
                }
                int nonfinite = 0, outsideBelowRim = 0, inside = 0;
                float lowY = float.PositiveInfinity, radialExtent = 0;
                var positions = result.GetData<Vector3>();
                var centralHeights = new System.Collections.Generic.List<float>();
                foreach (var p in positions)
                {
                    if (!SiUnits.IsFinite(p.x) || !SiUnits.IsFinite(p.y) || !SiUnits.IsFinite(p.z))
                    {
                        nonfinite++;
                        continue;
                    }
                    lowY = Mathf.Min(lowY, p.y);
                    float radial = new Vector2(p.x, p.z).magnitude;
                    radialExtent = Mathf.Max(radialExtent, radial);
                    if (p.y < rim && (p.y < floor - .00001f || radial > radius + .00001f))
                        outsideBelowRim++;
                    else if (p.y < rim)
                    {
                        inside++;
                        if (radial < radius * .5f)
                            centralHeights.Add(p.y - glassFloor);
                    }
                }
                float containedMl = (waterSurface != null ? waterVolumeMl : releaseVolumeMl) * inside / positions.Length;
                evidence = FormattableString.Invariant(
                    $"{timing} · {containedMl:F1} mL represented by {inside} particles inside the beaker · {positions.Length} particles · {nonfinite} nonfinite · {outsideBelowRim} outside below rim · lowest Y {lowY * 1000:F2} mm · radial extent {radialExtent * 1000:F2} mm");
                if (centralHeights.Count > 0)
                {
                    centralHeights.Sort();
                    // Use the upper 1% boundary near the centre to reject isolated spray.
                    float depth = centralHeights[Mathf.FloorToInt(.99f * (centralHeights.Count - 1))];
                    float area = Mathf.PI * graduationRadius * graduationRadius;
                    float expectedDepth = containedMl * 1e-6f / area;
                    evidence += FormattableString.Invariant(
                        $" · central particle depth (99th percentile) {depth * 1000:F1} mm · expected graduation depth {expectedDepth * 1000:F1} mm");
                }
                Debug.Log(evidence, this);
            });
        }

        private void OnGUI()
        {
            var bounds = new Rect(16, 16, Mathf.Max(1, Mathf.Min(460, Screen.width - 32)),
                Mathf.Max(1, Mathf.Min(heatMovingWater ? 590 : 400, Screen.height - 32)));
            GUILayout.BeginArea(bounds, GUI.skin.box);
            GUILayout.Label($"Simulation Controls · {releaseVolumeMl:F0} mL release", GUI.skin.box);
            if (beakerCamera != null && geometry != null &&
                GUILayout.Button(beakerCamera.IsUndersideView ? "Return to free camera" : "Underside view"))
                beakerCamera.ToggleUndersideView((float)geometry.externalBaseHeightM, (float)geometry.OuterRadiusM);

            panelScroll = GUILayout.BeginScrollView(panelScroll);
            if (fluid != null && fluid.positionBuffer != null)
            {
                GUILayout.Label($"{(hasWater ? fluid.positionBuffer.count : 0)} particles · {(frameMs > 0 ? 1000 / frameMs : 0):F1} FPS");
                GUILayout.Label($"{(hasWater ? fluid.SimulatedSeconds : 0):F4} s simulated · {solverSteps:N0} solver steps");
                if (movingHeat != null)
                {
                    double solid = movingHeat.SampledSolidJ, water = movingHeat.SampledWaterJ;
                    GUILayout.Label($"Absorbed {movingHeat.SampledInputJ:F4} J · Stored in glass and liquid {solid + water:F4} J");
                    GUILayout.Label($"Heat to room {movingHeat.SampledRoomHeatJ:F4} J");
                    GUILayout.Label($"Entropy change {movingHeat.SampledStoredEntropyJPerK:G4} J/K · (glass and liquid only)");
                    GUILayout.Label($"Solids {solid:F4} J · Water {water:F4} J");
                    GUILayout.Label(movingHeat.FlowDescription);
                    if (showBoiling)
                    {
                        GUILayout.Label($"Liquid remaining {movingHeat.SampledLiquidMassKg / initialLiquidDensity * 1e6:F4} mL (reference density)");
                        GUILayout.Label($"Vapour generated {movingHeat.VapourGeneratedKg * 1e6:G4} mg · Latent heat {movingHeat.VapourLatentHeatJ:G4} J");
                        GUILayout.Label("Vapour speed · light blue 0 to white 0.35 m/s");
                    }
                    bool selected = GUILayout.Toolbar(showTemperature ? 0 : 1, ColourViews) == 0;
                    if (selected != showTemperature)
                    {
                        showTemperature = selected;
                        particleDisplay.showTemperature = selected;
                        particleDisplay.colourMap = selected ? temperatureColours : speedColours;
                    }
                }
                if (waterSurface != null)
                {
                    waterTint = Slider($"Water tint  {waterTint:P0} (Clear to Dark blue)", waterTint, 0f, 1f);
                    waterSurface.col = new Color(1f, 1f, 1f, waterTint);
                }
                else
                {
                    GUILayout.Label(showTemperature
                        ? $"Water temperature · blue 20.0 °C to red {hotColourC:F1} °C"
                        : $"Water speed · blue 0 to white {particleDisplay.velocityDisplayMax:F2} m/s");
                    Rect legend = GUILayoutUtility.GetRect(1, 12, GUILayout.ExpandWidth(true));
                    GUI.DrawTexture(legend, showTemperature ? temperatureLegend : speedLegend, ScaleMode.StretchToFill);
                }
                bool controlsReady = !readbackPending && !finishOnNextFrame && pendingPanelAction == null;
                GUI.enabled = controlsReady;
                GUILayout.BeginHorizontal();
                if (GUILayout.Button(running ? "Pause" : "Run"))
                {
                    SetRunning(!running);
                    measuring = false;
                }
                GUI.enabled = controlsReady;
                if (GUILayout.Button("Reset")) pendingPanelAction = () =>
                {
                    SetRunning(false);
                    measuring = false;
                    if (fluid.worldSpaceSurface)
                    {
                        fluid.densityTextureRes = 96;
                    }
                    if (waterSurface != null) ClearBeaker();
                    else fluid.ResetParticles();
                    ResetHeat();
                    solverSteps = 0;
                    evidence = null;
                };
                GUI.enabled = controlsReady && hasWater && iceCubes.Count == 0 && waterBatches == 1 && waterVolumeMl == releaseVolumeMl;
                if (GUILayout.Button("Measure drop + slosh")) pendingPanelAction = () => StartMeasurement();
                GUILayout.EndHorizontal();
                GUI.enabled = controlsReady;
                if (waterSurface != null)
                    nextIceSideMm = Slider($"Next ice cube · {SelectedIceSideMm:F1} mm per side",
                        SelectedIceSideMm, 5f, MaxIceSideMm);

                if (waterSurface != null)
                {
                    GUILayout.BeginHorizontal();
                    float reservedMl = waterVolumeMl;
                    foreach (var cube in iceCubes)
                        reservedMl += releaseVolumeMl * cube.YieldFraction * (1 - cube.ReleasedFraction);
                    bool canSpawn = controlsReady && !measuring;
                    GUI.enabled = canSpawn && reservedMl + releaseVolumeMl <= BeakerRenderMockup.MaxMarkedVolumeMl;
                    if (GUILayout.Button($"Spawn water ({releaseVolumeMl:F0} mL)"))
                        pendingPanelAction = () => SpawnWater(fluid.spawner.spawnRegions[0].centre);
                    GUI.enabled = canSpawn && iceCubes.Count < FluidSim.MaxIceBodies && reservedMl + SelectedIceYieldMl <= BeakerRenderMockup.MaxMarkedVolumeMl;
                    if (GUILayout.Button($"Spawn ice ({SelectedIceYieldMl:F1} mL yield)"))
                        pendingPanelAction = SpawnIce;
                    GUILayout.EndHorizontal();
                }
                GUI.enabled = controlsReady;
                if (iceCubes.Count > 0)
                {
                    float mass = 0;
                    foreach (var cube in iceCubes) mass += cube.Body.mass;
                    GUILayout.Label($"Ice cubes {iceCubes.Count} · {mass * 1000:F1} g remaining");
                    absorbedBurnerWatts = Slider($"Absorbed burner power {absorbedBurnerWatts:F0} W",
                        absorbedBurnerWatts, 0, 100);
                }
                if (fluid.worldSpaceSurface)
                {
                    GUI.enabled = controlsReady && !running && hasWater;
                    if (GUILayout.Button("Measure contained water"))
                        pendingPanelAction = () => MeasureContainedWater(FormattableString.Invariant($"Nominal release {waterVolumeMl:F1} mL"));
                    GUI.enabled = !readbackPending && !finishOnNextFrame;
                }
                if (movingHeat != null && GUILayout.Button("Measure circulation · 30 simulated s"))
                    StartMeasurement(CirculationMicroseconds);

                GUI.enabled = controlsReady && !running && !measuring && hasWater && iceCubes.Count == 0;
                if (GUILayout.Button("Step once"))
                {
                    AdvanceDrop(StepSeconds, 1);
                    solverSteps++;
                }

                GUI.enabled = !measuring && !readbackPending && !finishOnNextFrame;
                int nextTimestep = Mathf.RoundToInt(Slider($"Solver step  {StepMicroseconds / 1000.0:0.##} ms",
                    timestepIndex, 0, Timesteps.Length - 1));
                if (nextTimestep != timestepIndex)
                {
                    timestepIndex = nextTimestep;
                    SetRunning(running);
                }

                playbackSpeed = Slider($"Playback speed  {playbackSpeed:P0}", playbackSpeed, .05f, 1f);
                float gravity = Slider($"Gravity  {-fluid.gravity:F2} m/s²", -fluid.gravity, 0, 20);
                fluid.gravity = -gravity;
                // The shader uses number density and pressure per particle mass.
                // Convert to Pa using the reference mass represented by one particle.
                const float referenceWaterDensityKgM3 = 998.2f;
                float particleMassKg = referenceWaterDensityKgM3 * (hasWater && waterSurface != null ? waterVolumeMl : releaseVolumeMl) * 1e-6f /
                    fluid.positionBuffer.count;
                float referencePressure = particleMassKg * fluid.targetDensity;
                float pascalsPerMultiplier = referencePressure * spacingScale;

                float bulkPressurePa = fluid.pressureMultiplier * referencePressure;
                const float waterBulkModulusPa = 2.2e9f;
                float bulkStiffnessPpm = bulkPressurePa / waterBulkModulusPa * 1e6f;
                bulkStiffnessPpm = Slider($"Effective bulk modulus  {bulkPressurePa:F2} Pa",
                    bulkStiffnessPpm, 0, 6 * pascalsPerMultiplier / waterBulkModulusPa * 1e6f);
                bulkPressurePa = bulkStiffnessPpm * 1e-6f * waterBulkModulusPa;
                GUILayout.Label($"Water reference  2.2 GPa · solver {bulkStiffnessPpm:F3} ppm of reference");
                fluid.pressureMultiplier = bulkPressurePa / referencePressure;

                float nearPressurePa = fluid.nearPressureMultiplier * referencePressure;
                nearPressurePa = Slider($"Near pressure at reference density  {nearPressurePa:F2} Pa",
                    nearPressurePa, 0, .1f * pascalsPerMultiplier);
                fluid.nearPressureMultiplier = nearPressurePa / referencePressure;
                GUILayout.Label("Near-pressure reference uses the bulk target density as its input.");
                float dampingScale = spacingScale * spacingScale * spacingScale;
                float damping = fluid.viscosityStrength / dampingScale;
                float dampingLimit = fluid.worldSpaceSurface ? .0000001f : 1f;
                string dampingLabel = fluid.worldSpaceSurface ? $"Viscosity strength  {damping:E2}" : $"Viscosity strength  {damping:F2}";
                fluid.viscosityStrength = Slider(dampingLabel, damping, 0, dampingLimit) * dampingScale;
                fluid.collisionDamping = Slider($"Wall bounce  {fluid.collisionDamping:P0}", fluid.collisionDamping, 0, 1);
                glassFloorPressure = GUILayout.Toggle(glassFloorPressure, "Glass floor pressure support");
                if (movingHeat != null)
                {
                    absorbedBurnerWatts = Slider($"Burner {absorbedBurnerWatts:F0} W", absorbedBurnerWatts, 0, 100);
                    thermalBuoyancy = GUILayout.Toggle(thermalBuoyancy, "Thermal buoyancy");
                    if (showTemperature)
                    {
                        hotColourC = Slider($"Red endpoint  {hotColourC:F1} °C", hotColourC, 20.1f, 100f);
                        particleDisplay.temperatureRiseDisplayMax = hotColourC - 20f;
                    }
                }
                if (GUILayout.Button("Restore scene settings"))
                {
                    ApplySceneSettings();
                    SetRunning(running);
                }

                GUI.enabled = true;
            }
            if (evidence != null) GUILayout.Label(evidence);
            GUILayout.Label("WASD move · Right mouse look · Wheel zoom · R reset view");
            GUILayout.EndScrollView();
            GUILayout.EndArea();
        }

        private static float Slider(string label, float value, float min, float max)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(label, GUILayout.Width(168));
            float selected = GUILayout.HorizontalSlider(value, min, max);
            GUILayout.EndHorizontal();
            return selected;
        }

        private void ApplySceneSettings()
        {
            timestepIndex = Array.IndexOf(Timesteps, solverTimestep);
            if (timestepIndex < 0) timestepIndex = Array.IndexOf(Timesteps, SolverTimestep.Ms040);

            playbackSpeed = playbackPercent / 100f;
            fluid.gravity = -gravity;
            fluid.pressureMultiplier = pressureMultiplier * spacingScale;
            fluid.nearPressureMultiplier = nearPressureMultiplier * spacingScale;
            fluid.viscosityStrength = viscosityStrength * spacingScale * spacingScale * spacingScale;
            fluid.collisionDamping = wallBounce;
        }

        private void RestoreBounceThreshold()
        {
            if (!previousBounceThreshold.HasValue) return;
            Physics.bounceThreshold = previousBounceThreshold.Value;
            previousBounceThreshold = null;
        }

        private void OnDestroy()
        {
            RestoreBounceThreshold();
            if (iceContactMaterial != null) Destroy(iceContactMaterial);
            // The local scene owns the cubes and contacts, including any already destroyed during shutdown.
            iceCubes.Clear();
            if (icePhysicsScene.IsValid() && icePhysicsScene.isLoaded)
                SceneManager.UnloadSceneAsync(icePhysicsScene);
            if (iceMaterial != null) Destroy(iceMaterial);
            if (iceMesh != null) Destroy(iceMesh);
            movingHeat?.Dispose();
            plume?.Dispose();
            if (vapour != null) Destroy(vapour.gameObject);
            if (vapourMaterial != null) Destroy(vapourMaterial);
            if (temperatureLegend != null) Destroy(temperatureLegend);
            if (speedLegend != null) Destroy(speedLegend);
            if (solverObject != null) Destroy(solverObject);
            if (computeInstance != null) Destroy(computeInstance);
        }
    }
    // Boussinesq air plume above the rim. Vapour markers follow the projected air.
    internal sealed class VapourPlume : IDisposable
    {
        private const double AirDensity = 1.2;
        private readonly Grid3D grid;
        private readonly VariableCoefficientProjection plan;
        private readonly FlowGpuSolver pressureSolver;
        private (double DryAirKg, double VapourKg, double EnthalpyJ, D3 MomentumKgMPerS)[] gas;
        private FlowStep flow;

        internal VapourPlume(GeometryInputs geometry)
        {
            double width = geometry.OuterRadiusM * 4;
            grid = new Grid3D(8, 12, 8, new D3(width / 8, width / 8, width / 8),
                RigidFrame.Euler(new D3(-width / 2, geometry.RimHeightM, -width / 2), 0, 0, 0));
            var sides = new FlowBoundary[6];
            sides[3] = new FlowBoundary(FlowBoundaryKind.OpenPressure);
            plan = new VariableCoefficientProjection(CutCellGeometry.Build(grid),
                new BoundaryConditions3D(sides), _ => AirDensity, _ => 0);
            var shader = Resources.Load<ComputeShader>("ThreePhase3D/SinglePhaseFlow");
            if (shader == null) throw new InvalidOperationException("Vapour pressure shader unavailable.");
            Reset();
            pressureSolver = new FlowGpuSolver(plan, null, shader);
        }

        internal void Reset()
        {
            flow = null;
            gas = new (double, double, double, D3)[plan.GroupCount];
            for (int i = 0; i < gas.Length; i++) gas[i].DryAirKg = AirDensity * plan.Volumes[i];
        }

        internal void Inject(Vector3 origin, double massKg)
        {
            if (TryCell(origin, out int i, out int j, out int k))
                gas[plan.PressureGroup(i, j, k)].VapourKg += massKg;
        }

        internal void Step(double seconds)
        {
            // Advect a vapour mass tracer in a constant-reference-density air solve.
            // The buoyancy term uses the dilute ideal-gas composition correction.
            var predicted = HumidGasAdvection.PredictFaces(plan, gas);
            for (int index = 0; index < predicted.Length; index++)
            {
                var face = plan.Faces[index];
                if (face.Active == 0 || face.Axis != 1) continue;
                int low = face.Low < 0 ? face.High : face.Low;
                int high = face.High < 0 ? low : face.High;
                double fraction = .5 * (Fraction(low) + Fraction(high));
                predicted[index] += (float)(seconds * 9.81 * .378 * fraction);
            }
            FlowStep projected;
            try
            {
                projected = pressureSolver.Project(predicted, seconds, 4096,
                    initialPressure: flow?.GroupPressurePa);
            }
            catch (FlowConvergenceException) when (seconds > .0125)
            {
                // Commit both half-steps together, or restore the prior plume state.
                var previousGas = gas;
                var previousFlow = flow;
                try
                {
                    Step(seconds / 2);
                    Step(seconds / 2);
                }
                catch
                {
                    gas = previousGas;
                    flow = previousFlow;
                    throw;
                }
                return;
            }
            var trialGas = ((double DryAirKg, double VapourKg, double EnthalpyJ, D3 MomentumKgMPerS)[])gas.Clone();
            for (int k = 0; k < grid.Cells.Z; k++)
            for (int j = 0; j < grid.Cells.Y; j++)
            for (int i = 0; i < grid.Cells.X; i++)
            {
                int group = plan.PressureGroup(i, j, k);
                Vector3 velocity = CellVelocity(projected, i, j, k);
                trialGas[group].MomentumKgMPerS = new D3(velocity.x, velocity.y, velocity.z) *
                    (gas[group].DryAirKg + gas[group].VapourKg);
            }
            gas = HumidGasAdvection.Advance(plan, projected, plan.Volumes, trialGas,
                (AirDensity, 0, 0, default(D3))).Gas;
            flow = projected;
        }

        public void Dispose() => pressureSolver.Dispose();

        private double Fraction(int group) => gas[group].VapourKg /
            (gas[group].DryAirKg + gas[group].VapourKg);

        internal Vector3 VelocityAt(Vector3 point) => flow != null && TryCell(point, out int i, out int j, out int k)
            ? CellVelocity(flow, i, j, k) : Vector3.zero;

        private bool TryCell(Vector3 point, out int i, out int j, out int k)
        {
            i = Mathf.FloorToInt((float)((point.x - grid.Frame.Origin.X) / grid.Spacing.X));
            j = Mathf.FloorToInt((float)((point.y - grid.Frame.Origin.Y) / grid.Spacing.Y));
            k = Mathf.FloorToInt((float)((point.z - grid.Frame.Origin.Z) / grid.Spacing.Z));
            return grid.Cells.Interior(i, j, k);
        }

        private Vector3 CellVelocity(FlowStep flow, int i, int j, int k)
        {
            var u = flow.VelocityMPerS;
            return .5f * new Vector3(
                u[plan.FaceIndex(0, i, j, k)] + u[plan.FaceIndex(0, i + 1, j, k)],
                u[plan.FaceIndex(1, i, j, k)] + u[plan.FaceIndex(1, i, j + 1, k)],
                u[plan.FaceIndex(2, i, j, k)] + u[plan.FaceIndex(2, i, j, k + 1)]);
        }
    }
}
