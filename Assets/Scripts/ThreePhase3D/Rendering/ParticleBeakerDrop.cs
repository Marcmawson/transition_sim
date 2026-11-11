using System;
using PhysicsSim.ThreePhase3D.Core;
using Seb.Fluid.Rendering;
using Seb.Fluid.Simulation;
using UnityEngine;
using UnityEngine.Rendering;

namespace PhysicsSim.ThreePhase3D.Rendering
{
    [RequireComponent(typeof(Beaker3DSetup))]
    public sealed class ParticleBeakerDrop : MonoBehaviour
    {
        private const float VolumeMl = 9;
        private const float DropGapM = .01f;
        private const float StepSeconds = .0005f;
        private const float TestSeconds = .4f;
        private const int StepsPerFrame = 8;
        [SerializeField] private ComputeShader fluidCompute;
        [SerializeField] private Shader particleShader;

        private FluidSim fluid;
        private ComputeShader computeInstance;
        private GameObject solverObject;
        private GeometryInputs geometry;
        private bool running, measuring, finishOnNextFrame, readbackPending;
        private double startTime;
        private int measuredFrames;
        private string evidence;
        private float frameMs;

        private void Start()
        {
            if (!SystemInfo.supportsComputeShaders || fluidCompute == null || particleShader == null)
            {
                evidence = "The GPU particle solver or its shaders are unavailable.";
                return;
            }

            var setup = GetComponent<Beaker3DSetup>();
            geometry = setup.scenario.physical.geometry;
            geometry.innerRadiusM = .033;
            geometry.cavityHeightM = .06;
            setup.scenario.display.glassOpacity = .28f;
            setup.scenario.display.showSetupReport = false;
            setup.Refresh();

            float floor = (float)(geometry.externalBaseHeightM + geometry.baseThicknessM);
            float rim = (float)geometry.RimHeightM;
            float side = Mathf.Pow(VolumeMl * 1e-6f, 1f / 3f);
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
            fluid.iterationsPerFrame = StepsPerFrame;
            fluid.gravity = -9.81f;
            // Start from the demo's liquid parameters scaled to metre-sized particles.
            fluid.smoothingRadius = .002f;
            fluid.targetDensity = 630e6f;
            fluid.pressureMultiplier = 2.88f;
            fluid.nearPressureMultiplier = .0216f;
            fluid.collisionDamping = .2f;
            fluid.maxFoamParticleCount = 1;
            const float particleRadiusM = .0004f;
            fluid.beakerCollision = new Vector4(0, 0,
                (float)geometry.innerRadiusM * Mathf.Cos(Mathf.PI / setup.scenario.display.cylinderSegments) - particleRadiusM,
                floor + particleRadiusM);
            fluid.beakerRimY = rim;

            var display = solverObject.AddComponent<ParticleDisplay3D>();
            display.sim = fluid;
            display.mode = ParticleDisplay3D.DisplayMode.Billboard;
            display.shaderBillboard = particleShader;
            display.scale = particleRadiusM * 100;
            display.gradientResolution = 32;
            display.velocityDisplayMax = .65f;
            display.colourMap = new Gradient();
            display.colourMap.SetKeys(new[]
            {
                new GradientColorKey(new Color(.05f, .55f, .88f), 0),
                new GradientColorKey(new Color(.52f, .92f, 1f), 1)
            }, new[] { new GradientAlphaKey(1, 0), new GradientAlphaKey(1, 1) });
            solverObject.SetActive(true);
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
            if (!running || fluid == null || fluid.positionBuffer == null) return;
            fluid.AdvanceManually(StepSeconds);
            if (measuring)
            {
                measuredFrames++;
                if (fluid.SimulatedSeconds >= TestSeconds - StepSeconds / 2)
                {
                    running = false;
                    finishOnNextFrame = true;
                }
            }
        }

        private void StartMeasurement()
        {
            fluid.ResetParticles();
            evidence = null;
            measuredFrames = 0;
            startTime = Time.realtimeSinceStartupAsDouble;
            running = measuring = true;
        }

        private void FinishMeasurement()
        {
            measuring = false;
            double elapsed = Time.realtimeSinceStartupAsDouble - startTime;
            string timing = FormattableString.Invariant(
                $"GPU drop: {measuredFrames} frames, {measuredFrames / elapsed:F1} FPS, {fluid.SimulatedSeconds:F3} s simulated in {elapsed:F3} s");
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
            GUILayout.BeginArea(new Rect(16, 16, Mathf.Min(480, Screen.width - 32), 195), GUI.skin.box);
            GUILayout.Label("GPU particle beaker · 9 mL drop");
            if (fluid != null && fluid.positionBuffer != null)
            {
                GUILayout.Label($"{fluid.positionBuffer.count} particles · {fluid.SimulatedSeconds:F3} s · {(frameMs > 0 ? 1000 / frameMs : 0):F1} FPS");
                GUI.enabled = !readbackPending && !finishOnNextFrame;
                GUILayout.BeginHorizontal();
                if (GUILayout.Button(running ? "Pause" : "Run"))
                {
                    running = !running;
                    measuring = false;
                }
                if (GUILayout.Button("Reset"))
                {
                    running = measuring = false;
                    fluid.ResetParticles();
                    evidence = null;
                }
                if (GUILayout.Button("Measure drop + slosh")) StartMeasurement();
                GUILayout.EndHorizontal();
                GUI.enabled = true;
            }
            if (evidence != null) GUILayout.Label(evidence);
            GUILayout.Label("WASD move · Right mouse look · Wheel zoom · R reset view");
            GUILayout.EndArea();
        }

        private void OnDestroy()
        {
            if (solverObject != null) Destroy(solverObject);
            if (computeInstance != null) Destroy(computeInstance);
        }
    }
}
