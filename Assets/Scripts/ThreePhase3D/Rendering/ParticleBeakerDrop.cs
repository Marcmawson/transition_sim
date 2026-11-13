using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Materials;
using Seb.Fluid.Rendering;
using Seb.Fluid.Simulation;
using UnityEngine;
using UnityEngine.Rendering;

namespace PhysicsSim.ThreePhase3D.Rendering
{
    [RequireComponent(typeof(Beaker3DSetup))]
    public sealed class ParticleBeakerDrop : MonoBehaviour
    {
        private const float SmallVolumeMl = 9;
        private const float LargeVolumeMl = 250;
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

        private int StepMicroseconds => (int)Timesteps[timestepIndex];
        private double StepSeconds => StepMicroseconds / 1_000_000.0;
        private float releaseVolumeMl;

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
            geometry.innerRadiusM = largerBeaker ? .06 : .033;
            geometry.cavityHeightM = largerBeaker ? .16 : .06;
            geometry.tripodRingRadiusM = geometry.OuterRadiusM * .75;
            setup.scenario.display.glassOpacity = .28f;
            setup.scenario.display.showSetupReport = false;
            setup.Refresh();

            float floor = (float)(geometry.externalBaseHeightM + geometry.baseThicknessM);
            float rim = (float)geometry.RimHeightM;
            float side = Mathf.Pow(releaseVolumeMl * 1e-6f, 1f / 3f);
            solverObject = new GameObject("GPU liquid particles");
            solverObject.SetActive(false);
            solverObject.transform.position = new Vector3(0, .175f, 0);
            solverObject.transform.localScale = new Vector3(.3f, .35f, .3f);
            var spawner = solverObject.AddComponent<Spawner3D>();
            spawner.particleSpawnDensity = 700000000;
            spawner.jitterStrength = .00018f;
            spawner.spawnRegions = new[]
            {
                new Spawner3D.SpawnRegion
                {
                    centre = new Vector3(0, floor + DropGapM + side / 2, 0),
                    size = side
                }
            };

            computeInstance = Instantiate(fluidCompute);
            fluid = solverObject.AddComponent<FluidSim>();
            fluid.compute = computeInstance;
            fluid.spawner = spawner;
            fluid.manualControl = true;
            fluid.transportTemperature = heatMovingWater;
            if (heatMovingWater) fluid.buoyancyDensityByDegree = BuildBuoyancyDensityTable();
            // Start from the demo's liquid parameters scaled to metre-sized particles.
            fluid.smoothingRadius = .002f;
            fluid.targetDensity = 630e6f;
            ApplySceneSettings();
            fluid.maxFoamParticleCount = 1;
            const float particleRadiusM = .0004f;
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
            solverObject.SetActive(true);
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
                try { movingHeat = new MovingBeakerHeat(fluid, geometry, releaseVolumeMl * 1e-6); }
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
            fluid.AdvanceManually((float)stepSeconds, steps);
            movingHeat?.Advance(stepSeconds * steps, measuring ? measuredBurnerWatts : absorbedBurnerWatts);
        }

        private void SetRunning(bool value)
        {
            running = value;
            pendingSeconds = 0;
            lastTick = Time.realtimeSinceStartupAsDouble;
        }

        private void StartMeasurement(int durationMicroseconds = BenchmarkMicroseconds)
        {
            fluid.ResetParticles();
            movingHeat?.Reset();
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
            if (!SystemInfo.supportsAsyncGPUReadback)
            {
                Debug.Log(timing, this);
                return;
            }
            readbackPending = true;
            float floor = fluid.beakerCollision.w;
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
                int nonfinite = 0, outsideBelowRim = 0;
                float lowY = float.PositiveInfinity, radialExtent = 0;
                var positions = result.GetData<Vector3>();
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
                }
                evidence = FormattableString.Invariant(
                    $"{timing} · {positions.Length} particles · {nonfinite} nonfinite · {outsideBelowRim} outside below rim · lowest Y {lowY * 1000:F2} mm · radial extent {radialExtent * 1000:F2} mm");
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
                    GUILayout.Label($"Absorbed {movingHeat.SampledInputJ:F4} J · Retained {solid + water:F4} J");
                    GUILayout.Label($"Entropy change {movingHeat.SampledStoredEntropyJPerK:G4} J/K · Inferred production {movingHeat.SampledProducedEntropyJPerK:G4} J/K");
                    GUILayout.Label($"Solids {solid:F4} J · Water {water:F4} J");
                    GUILayout.Label(movingHeat.FlowDescription);
                    bool selected = GUILayout.Toolbar(showTemperature ? 0 : 1, ColourViews) == 0;
                    if (selected != showTemperature)
                    {
                        showTemperature = selected;
                        particleDisplay.showTemperature = selected;
                        particleDisplay.colourMap = selected ? temperatureColours : speedColours;
                    }
                }
                GUILayout.Label(showTemperature
                    ? $"Water temperature · blue 20.0 °C to red {hotColourC:F1} °C"
                    : $"Water speed · blue 0 to white {particleDisplay.velocityDisplayMax:F2} m/s");
                Rect legend = GUILayoutUtility.GetRect(1, 12, GUILayout.ExpandWidth(true));
                GUI.DrawTexture(legend, showTemperature ? temperatureLegend : speedLegend, ScaleMode.StretchToFill);
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
                    fluid.ResetParticles();
                    movingHeat?.Reset();
                    solverSteps = 0;
                    evidence = null;
                }
                if (GUILayout.Button("Measure drop + slosh")) StartMeasurement();
                GUILayout.EndHorizontal();
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
                fluid.pressureMultiplier = Slider($"Pressure  {fluid.pressureMultiplier:F2}", fluid.pressureMultiplier, 0, 6);
                fluid.nearPressureMultiplier = Slider($"Near pressure  {fluid.nearPressureMultiplier:F4}", fluid.nearPressureMultiplier, 0, .1f);
                fluid.viscosityStrength = Slider($"Viscosity strength  {fluid.viscosityStrength:F2}", fluid.viscosityStrength, 0, 1);
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
            fluid.pressureMultiplier = pressureMultiplier;
            fluid.nearPressureMultiplier = nearPressureMultiplier;
            fluid.viscosityStrength = viscosityStrength;
            fluid.collisionDamping = wallBounce;
        }

        private void OnDestroy()
        {
            movingHeat?.Dispose();
            if (temperatureLegend != null) Destroy(temperatureLegend);
            if (speedLegend != null) Destroy(speedLegend);
            if (solverObject != null) Destroy(solverObject);
            if (computeInstance != null) Destroy(computeInstance);
        }
    }
}
