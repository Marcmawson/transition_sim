using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;
using PhysicsSim.ThreePhase3D.Geometry;
using PhysicsSim.ThreePhase3D.Materials;
using Seb.Fluid.Rendering;
using Seb.Fluid.Simulation;
using UnityEngine;
using UnityEngine.Rendering;

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
        private float playbackSpeed;
        private float measuredPlaybackSpeed, measuredBurnerWatts;
        private bool measuredBuoyancy, measuredFloorPressure;
        private int timestepIndex;
        private int benchmarkSteps;
        private Vector2 panelScroll;
        private MarchingCubesFluidRenderer waterSurface;
        private float waterTint;
        [SerializeField] private bool spawnIce;
        private Rigidbody ice;
        private Material iceMaterial;
        private GameObject iceContacts;
        private float iceSide, iceMass, iceHeat, iceSpawnY, iceFloor;
        private const float IceStartC = -5f;
        private const float IceHeatCapacity = 2050f;
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
                iceMass = volumeM3 * 998.2f;
                iceSide = Mathf.Pow(iceMass / 917f, 1f / 3f);
                iceFloor = floor;
                iceSpawnY = dropBottom + iceSide * .68f;
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
            if (spawnIce && meshWater) ResetIce();
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
            if (ice != null)
            {
                AdvanceIce(Time.unscaledDeltaTime * playbackSpeed);
                return;
            }
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
            if (steps == 0) return;
            AdvanceDrop(stepSeconds, steps);
            solverSteps += steps;
            pendingSeconds -= steps * stepSeconds;
            if (measuring && solverSteps == benchmarkSteps)
            {
                running = false;
                finishOnNextFrame = true;
            }
        }

        private void AdvanceDrop(double stepSeconds, int steps)
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
            fluid.AdvanceManually((float)stepSeconds, steps);
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
            if (ice != null) ice.isKinematic = !value;
            pendingSeconds = 0;
            lastTick = Time.realtimeSinceStartupAsDouble;
        }

        private void ResetIce()
        {
            if (ice != null)
            {
                ice.gameObject.SetActive(false);
                Destroy(ice.gameObject);
                ice = null;
            }
            if (iceContacts != null) iceContacts.SetActive(spawnIce);
            waterSurface.enabled = !spawnIce;
            if (!spawnIce) return;

            if (iceContacts == null)
            {
                iceContacts = new GameObject("Ice beaker contacts");
                float radius = (float)geometry.innerRadiusM;
                float height = (float)geometry.cavityHeightM;
                AddContact(Vector3.up * (iceFloor - .005f), new Vector3(2 * radius, .01f, 2 * radius), 0);
                const int wallCount = 24;
                for (int i = 0; i < wallCount; i++)
                {
                    float angle = i * 360f / wallCount;
                    Vector3 radial = Quaternion.Euler(0, angle, 0) * Vector3.forward;
                    AddContact(Vector3.up * (iceFloor + height * .5f) + radial * (radius + .002f),
                        new Vector3(2 * radius * Mathf.Tan(Mathf.PI / wallCount), height, .004f), angle);
                }
            }

            var cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
            cube.name = "Ice cube";
            cube.transform.SetPositionAndRotation(Vector3.up * iceSpawnY, Quaternion.Euler(13, 19, 9));
            cube.transform.localScale = Vector3.one * iceSide;
            if (iceMaterial == null)
            {
                var shader = Resources.Load<Shader>("ThreePhase3D/BeakerRefraction");
                iceMaterial = new Material(shader != null && shader.isSupported ? shader : Shader.Find("Standard"));
                iceMaterial.SetFloat("_RefractiveIndex", 1.31f);
                iceMaterial.SetFloat("_NormalReflectance", .018f);
            }
            cube.GetComponent<MeshRenderer>().sharedMaterial = iceMaterial;
            ice = cube.AddComponent<Rigidbody>();
            ice.mass = iceMass;
            ice.useGravity = false;
            ice.interpolation = RigidbodyInterpolation.Interpolate;
            ice.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
            ice.centerOfMass = iceSide * new Vector3(.08f, -.05f, .06f);
            ice.isKinematic = true;
            iceHeat = 0;
        }

        private void AddContact(Vector3 centre, Vector3 size, float yaw)
        {
            var contact = new GameObject("Glass contact");
            contact.transform.SetParent(iceContacts.transform, false);
            contact.transform.SetPositionAndRotation(centre, Quaternion.Euler(0, yaw, 0));
            contact.AddComponent<BoxCollider>().size = size;
        }

        private void FixedUpdate()
        {
            if (running && ice != null)
                ice.AddForce(Vector3.up * fluid.gravity, ForceMode.Acceleration);
        }

        private void AdvanceIce(float seconds)
        {
            // Approximate room heating and apply the selected absorbed burner power near the floor.
            float warmup = iceMass * IceHeatCapacity * -IceStartC;
            float latent = iceMass * IceLatentHeat;
            float temperature = Mathf.Min(0, IceStartC + iceHeat / (iceMass * IceHeatCapacity));
            float area = 6 * ice.transform.localScale.x * ice.transform.localScale.x;
            float burner = ice.position.y - iceFloor < iceSide ? absorbedBurnerWatts : 0;
            iceHeat += (8f * area * (20 - temperature) + burner) * seconds;
            float remaining = 1 - Mathf.Clamp01((iceHeat - warmup) / latent);
            if (remaining > .0001f)
            {
                float side = iceSide * Mathf.Pow(remaining, 1f / 3f);
                ice.transform.localScale = Vector3.one * side;
                ice.mass = iceMass * remaining;
                ice.centerOfMass = side * new Vector3(.08f, -.05f, .06f);
                return;
            }

            Vector3 originalCentre = fluid.spawner.spawnRegions[0].centre;
            Vector3 meltCentre = new Vector3(0, Mathf.Max(ice.position.y, iceFloor +
                fluid.spawner.spawnRegions[0].cylinderHeight * .5f + .001f), 0);
            meltCentre.y = Mathf.Min(meltCentre.y, originalCentre.y);
            Destroy(ice.gameObject);
            ice = null;
            waterSurface.enabled = true;
            fluid.ReleaseMeltwater(meltCentre);
            lastTick = Time.realtimeSinceStartupAsDouble;
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
                float containedMl = releaseVolumeMl * inside / positions.Length;
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
                GUILayout.Label($"{fluid.positionBuffer.count} particles · {(frameMs > 0 ? 1000 / frameMs : 0):F1} FPS");
                GUILayout.Label($"{fluid.SimulatedSeconds:F4} s simulated · {solverSteps:N0} solver steps");
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
                GUI.enabled = !readbackPending && !finishOnNextFrame;
                GUILayout.BeginHorizontal();
                if (GUILayout.Button(running ? "Pause" : "Run"))
                {
                    SetRunning(!running);
                    measuring = false;
                }
                if (GUILayout.Button("Reset"))
                {
                    SetRunning(false);
                    measuring = false;
                    if (fluid.worldSpaceSurface)
                    {
                        fluid.densityTextureRes = 96;
                    }
                    fluid.ResetParticles();
                    if (waterSurface != null) ResetIce();
                    ResetHeat();
                    solverSteps = 0;
                    evidence = null;
                }
                if (!spawnIce && GUILayout.Button("Measure drop + slosh")) StartMeasurement();
                GUILayout.EndHorizontal();
                if (waterSurface != null && !running && !readbackPending)
                {
                    bool selected = GUILayout.Toggle(spawnIce, "Spawn ice cube");
                    if (selected != spawnIce)
                    {
                        spawnIce = selected;
                        fluid.ResetParticles();
                        ResetIce();
                        solverSteps = 0;
                    }
                }
                if (ice != null)
                {
                    float temperature = Mathf.Min(0, IceStartC + iceHeat / (iceMass * IceHeatCapacity));
                    GUILayout.Label($"Ice {temperature:F1} °C · {ice.mass * 1000:F1} g remaining");
                    absorbedBurnerWatts = Slider($"Absorbed burner power {absorbedBurnerWatts:F0} W",
                        absorbedBurnerWatts, 0, 100);
                }
                if (fluid.worldSpaceSurface)
                {
                    GUI.enabled = !running && !readbackPending && !finishOnNextFrame;
                    if (ice == null && GUILayout.Button("Measure contained water"))
                        MeasureContainedWater(FormattableString.Invariant($"Nominal release {releaseVolumeMl:F1} mL"));
                    GUI.enabled = !readbackPending && !finishOnNextFrame;
                }
                if (movingHeat != null && GUILayout.Button("Measure circulation · 30 simulated s"))
                    StartMeasurement(CirculationMicroseconds);

                GUI.enabled = !running && !measuring && !readbackPending && !finishOnNextFrame;
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
                float particleMassKg = referenceWaterDensityKgM3 * releaseVolumeMl * 1e-6f /
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

        private void OnDestroy()
        {
            if (ice != null) Destroy(ice.gameObject);
            if (iceContacts != null) Destroy(iceContacts);
            if (iceMaterial != null) Destroy(iceMaterial);
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
