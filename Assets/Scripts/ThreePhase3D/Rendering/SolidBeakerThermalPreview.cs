using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using PhysicsSim.ThreePhase3D.Thermal;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Rendering
{
    [RequireComponent(typeof(Beaker3DSetup))]
    public sealed class SolidBeakerThermalPreview : MonoBehaviour
    {
        private const int RadialCells = BeakerSolidMesh.BaseRadialCells;
        private const int DepthCells = BeakerSolidMesh.BaseDepthCells;
        private const double InitialK = 293.15;
        private const float ColdWaterC = 20f;
        private static readonly Color ColdWaterColour = new Color(.06f, .32f, 1f);
        private static readonly Color HotWaterColour = new Color(1f, .06f, .04f);

        [SerializeField] private bool largerBeaker = false;
        [SerializeField] private bool includeWater = true;
        [SerializeField, Range(21f, 100f)] private float hotWaterColourC = 30f;
        [SerializeField, Range(0f, 100f)] private float absorbedBurnerPowerW = 10f;

        [StructLayout(LayoutKind.Sequential)]
        private struct FaceLink
        {
            public int Other;
            public float Conductance;
        }

        private ComputeShader shader;
        private ComputeBuffer read, write, capacity, burner, faceStart, links;
        private Texture2D map;
        private ParticleSystem waterDisplay;
        private ParticleSystem.Particle[] waterParticles;
        private int[] waterParticleCells;
        private Material waterMaterial;
        private Texture2D waterDot;
        private BeakerSolidMesh mesh;
        private float[] snapshot;
        private readonly Color32[] pixels = new Color32[RadialCells * DepthCells];
        private double absorbedJ, sampledAbsorbedJ, storedJ, waterJ, waterCapacity;
        private int kernel;
        private double stepSeconds, pendingSeconds, simulatedSeconds, nextSnapshot;
        private bool burnerOn, running, ready;
        private string status;
        private Vector2 panelScroll;

        private void Start()
        {
            try
            {
                Initialize();
                ready = true;
            }
            catch (Exception exception)
            {
                status = exception.Message;
                Debug.LogException(exception, this);
            }
        }

        private void Initialize()
        {
            var asset = Resources.Load<ComputeShader>("ThreePhase3D/SolidBeakerConduction");
            if (!SystemInfo.supportsComputeShaders || asset == null)
                throw new InvalidOperationException("S08 requires compute shader support and the solid conduction shader.");

            var setup = GetComponent<Beaker3DSetup>();
            var geometry = setup.scenario.physical.geometry;
            geometry.innerRadiusM = largerBeaker ? .06 : .033;
            geometry.cavityHeightM = largerBeaker ? .16 : .06;
            geometry.tripodRingRadiusM = geometry.OuterRadiusM * .75;
            setup.scenario.display.showSetupReport = false;
            setup.Refresh();

            double waterMl = includeWater ? (largerBeaker ? 250 : 9) : 0;
            mesh = new BeakerSolidMesh(geometry, waterMl * 1e-6);
            for (int i = mesh.WaterStart; i < mesh.CapacityJPerK.Length; i++)
                waterCapacity += mesh.CapacityJPerK[i];
            int cellCount = mesh.CapacityJPerK.Length;
            snapshot = new float[cellCount];
            var initial = new double[cellCount];
            for (int i = 0; i < cellCount; i++) initial[i] = InitialK + (i % 7) * .5;

            var reference = new SolidConduction(mesh.CapacityJPerK, initial, mesh.Faces);
            stepSeconds = Math.Min(.01, reference.MaximumStepSeconds * .5);
            var adjacency = new List<FaceLink>[cellCount];
            for (int i = 0; i < cellCount; i++) adjacency[i] = new List<FaceLink>();
            foreach (var face in mesh.Faces)
            {
                adjacency[face.A].Add(new FaceLink { Other = face.B, Conductance = (float)face.ConductanceWPerK });
                adjacency[face.B].Add(new FaceLink { Other = face.A, Conductance = (float)face.ConductanceWPerK });
            }
            var starts = new int[cellCount + 1];
            var flattened = new List<FaceLink>();
            for (int i = 0; i < cellCount; i++)
            {
                starts[i] = flattened.Count;
                flattened.AddRange(adjacency[i]);
            }
            starts[cellCount] = flattened.Count;

            shader = Instantiate(asset);
            kernel = shader.FindKernel("Conduct");
            read = new ComputeBuffer(cellCount, sizeof(float));
            write = new ComputeBuffer(cellCount, sizeof(float));
            capacity = new ComputeBuffer(cellCount, sizeof(float));
            burner = new ComputeBuffer(cellCount, sizeof(float));
            faceStart = new ComputeBuffer(starts.Length, sizeof(int));
            links = new ComputeBuffer(flattened.Count, sizeof(int) + sizeof(float));
            capacity.SetData(Array.ConvertAll(mesh.CapacityJPerK, value => (float)value));
            burner.SetData(Array.ConvertAll(mesh.BurnerWeights, value => (float)value));
            faceStart.SetData(starts);
            links.SetData(flattened);
            shader.SetInt("CellCount", cellCount);
            shader.SetFloat("StepSeconds", (float)stepSeconds);
            shader.SetBuffer(kernel, "Capacity", capacity);
            shader.SetBuffer(kernel, "BurnerPower", burner);
            shader.SetBuffer(kernel, "FaceStart", faceStart);
            shader.SetBuffer(kernel, "FaceLinks", links);
            map = new Texture2D(RadialCells, DepthCells, TextureFormat.RGBA32, false)
                { filterMode = FilterMode.Point, wrapMode = TextureWrapMode.Clamp };

            read.SetData(Array.ConvertAll(initial, value => (float)(value - InitialK)));
            var checkPower = Array.ConvertAll(mesh.BurnerWeights, value => value * 10);
            for (int i = 0; i < 20; i++)
            {
                Dispatch(10);
                reference.Step(stepSeconds, checkPower);
            }
            read.GetData(snapshot);
            double maxError = 0;
            for (int i = 0; i < cellCount; i++)
            {
                if (float.IsNaN(snapshot[i]) || float.IsInfinity(snapshot[i]))
                    throw new InvalidOperationException("S08 GPU conduction returned a non-finite temperature.");
                maxError = Math.Max(maxError, Math.Abs(snapshot[i] + InitialK - reference.TemperatureK[i]));
            }
            if (maxError > .001)
                throw new InvalidOperationException($"S08 CPU/GPU comparison failed: {maxError:G4} K.");
            status = $"GPU check passed · max error {maxError:G3} K";
            Debug.Log($"S08 {status}", this);
            ResetTemperatures();
            if (includeWater)
                SpawnWaterDisplay(geometry.innerRadiusM, geometry.externalBaseHeightM + geometry.baseThicknessM);
        }

        private void SpawnWaterDisplay(double radiusM, double floorM)
        {
            var shaderAsset = Shader.Find("Sprites/Default");
            if (shaderAsset == null)
                throw new InvalidOperationException("The water particle shader is unavailable.");

            waterDot = BeakerWaterParticles.CreateDot();
            waterMaterial = new Material(shaderAsset) { mainTexture = waterDot, renderQueue = 2999 };
            var container = new GameObject("Water temperature particles");
            container.transform.SetParent(transform, false);
            waterDisplay = container.AddComponent<ParticleSystem>();
            waterDisplay.Stop(false, ParticleSystemStopBehavior.StopEmittingAndClear);
            var main = waterDisplay.main;
            main.playOnAwake = false;
            main.simulationSpace = ParticleSystemSimulationSpace.Local;
            var emission = waterDisplay.emission;
            emission.enabled = false;
            var renderer = container.GetComponent<ParticleSystemRenderer>();
            renderer.sharedMaterial = waterMaterial;
            renderer.minParticleSize = 0;

            const int radialCells = BeakerSolidMesh.InnerRadialCells;
            const int depthCells = BeakerSolidMesh.WaterHeightCells;
            float radialStep = (float)radiusM / radialCells;
            float depthStep = (float)mesh.WaterDepthM / depthCells;
            float diameter = Mathf.Min(radialStep * .62f, (float)mesh.WaterDepthM * .35f);
            var particles = new List<ParticleSystem.Particle>();
            var indices = new List<int>();
            for (int r = 0; r < radialCells; r++)
            {
                float radius = (r + .5f) * radialStep;
                int around = Mathf.CeilToInt(2 * Mathf.PI * radius / radialStep);
                for (int z = 0; z < depthCells; z++)
                    for (int i = 0; i < around; i++)
                    {
                        float angle = (i + .5f * (z % 2)) * 2 * Mathf.PI / around;
                        particles.Add(new ParticleSystem.Particle
                        {
                            position = new Vector3(radius * Mathf.Cos(angle),
                                (float)floorM + (z + .5f) * depthStep, radius * Mathf.Sin(angle)),
                            startSize = diameter,
                            startLifetime = 1,
                            remainingLifetime = 1
                        });
                        indices.Add(mesh.WaterStart + r * depthCells + z);
                    }
            }
            waterParticles = particles.ToArray();
            waterParticleCells = indices.ToArray();
            main.maxParticles = waterParticles.Length;
            waterDisplay.Play(false);
            waterDisplay.Pause(false);
            RefreshWaterColours();
        }

        private void RefreshWaterColours()
        {
            if (waterDisplay == null) return;
            float span = Mathf.Max(1, hotWaterColourC - ColdWaterC);
            for (int i = 0; i < waterParticles.Length; i++)
            {
                float fraction = Mathf.Clamp01(snapshot[waterParticleCells[i]] / span);
                waterParticles[i].startColor = Color.Lerp(ColdWaterColour, HotWaterColour, fraction);
            }
            waterDisplay.SetParticles(waterParticles, waterParticles.Length);
        }

        private void Update()
        {
            if (!ready || !running) return;
            pendingSeconds += Math.Min(Time.unscaledDeltaTime, .1f);
            while (pendingSeconds >= stepSeconds)
            {
                float watts = burnerOn ? absorbedBurnerPowerW : 0;
                Dispatch(watts);
                absorbedJ += watts * stepSeconds;
                pendingSeconds -= stepSeconds;
                simulatedSeconds += stepSeconds;
            }
            if (simulatedSeconds >= nextSnapshot)
            {
                read.GetData(snapshot);
                storedJ = waterJ = 0;
                for (int i = 0; i < snapshot.Length; i++)
                {
                    double cellJ = snapshot[i] * mesh.CapacityJPerK[i];
                    storedJ += cellJ;
                    if (i >= mesh.WaterStart) waterJ += cellJ;
                }
                sampledAbsorbedJ = absorbedJ;
                RefreshMap();
                RefreshWaterColours();
                nextSnapshot = simulatedSeconds + .25;
            }
        }

        private void Dispatch(float watts)
        {
            shader.SetFloat("AbsorbedPowerW", watts);
            shader.SetBuffer(kernel, "TemperatureRead", read);
            shader.SetBuffer(kernel, "TemperatureWrite", write);
            shader.Dispatch(kernel, (snapshot.Length + 63) / 64, 1, 1);
            (read, write) = (write, read);
        }

        private void ResetTemperatures()
        {
            Array.Clear(snapshot, 0, snapshot.Length);
            absorbedJ = sampledAbsorbedJ = storedJ = waterJ = 0;
            read.SetData(snapshot);
            simulatedSeconds = pendingSeconds = nextSnapshot = 0;
            running = burnerOn = false;
            RefreshMap();
            RefreshWaterColours();
        }

        private void RefreshMap()
        {
            for (int r = 0; r < RadialCells; r++)
                for (int z = 0; z < DepthCells; z++)
                {
                    float rise = Mathf.Clamp01(snapshot[r * DepthCells + z] / 25);
                    pixels[z * RadialCells + r] = Color.Lerp(new Color(.12f, .32f, .65f), new Color(1, .3f, .04f), rise);
                }
            map.SetPixels32(pixels);
            map.Apply(false);
        }

        private void OnGUI()
        {
            float width = Mathf.Max(1, Mathf.Min(430, Screen.width - 24));
            float height = Mathf.Max(1, Mathf.Min(waterCapacity > 0 ? 425 : 320, Screen.height - 24));
            GUILayout.BeginArea(new Rect(12, 12, width, height), GUI.skin.box);
            panelScroll = GUILayout.BeginScrollView(panelScroll);
            GUILayout.Label(largerBeaker ? "S08 · 120 mm beaker" : "S08 · 66 mm beaker");
            GUILayout.Label(status ?? "Initialising GPU conduction");
            if (ready)
            {
                GUILayout.BeginHorizontal();
                if (GUILayout.Button(running ? "Pause" : "Run")) running = !running;
                if (GUILayout.Button(burnerOn ? "Burner off" : "Burner on"))
                {
                    burnerOn = !burnerOn;
                    if (burnerOn) running = true;
                }
                if (GUILayout.Button("Reset")) ResetTemperatures();
                GUILayout.EndHorizontal();
                GUILayout.Label($"Absorbed power: {absorbedBurnerPowerW:F0} W · {simulatedSeconds:F2} s");
                absorbedBurnerPowerW = GUILayout.HorizontalSlider(absorbedBurnerPowerW, 0, 100);
                GUILayout.Label($"Underside centre {snapshot[0] + 20:F2} °C · Upper centre {snapshot[DepthCells - 1] + 20:F2} °C");
                GUILayout.Label($"Lower wall {snapshot[mesh.LowerWall] + 20:F2} °C · Rim {snapshot[mesh.Rim] + 20:F2} °C");
                GUILayout.Label($"Support ring {snapshot[mesh.Ring] + 20:F2} °C · Leg top {snapshot[mesh.LegTop] + 20:F2} °C · Foot {snapshot[mesh.Foot] + 20:F2} °C");
                GUILayout.Label($"Absorbed {sampledAbsorbedJ:F2} J · Stored {storedJ:F2} J");
                GUILayout.Label("Glass base cross-section · centre to rim · 20–45 °C");
                Rect rect = GUILayoutUtility.GetRect(1, 70, GUILayout.ExpandWidth(true));
                GUI.DrawTexture(rect, map, ScaleMode.StretchToFill);
                if (waterCapacity > 0)
                {
                    GUILayout.Label($"Water {mesh.WaterVolumeM3 * 1e6:F0} mL · depth {mesh.WaterDepthM * 1000:F2} mm");
                    GUILayout.Label($"Water mean {20 + waterJ / waterCapacity:F3} °C · stored {waterJ:F2} J");
                    GUILayout.Label($"Water colours · blue {ColdWaterC:F0} °C to red {hotWaterColourC:F0} °C");
                }
            }
            GUILayout.EndScrollView();
            GUILayout.EndArea();
        }

        private void OnDestroy()
        {
            read?.Release();
            write?.Release();
            capacity?.Release();
            burner?.Release();
            faceStart?.Release();
            links?.Release();
            if (waterDisplay != null) Destroy(waterDisplay.gameObject);
            if (waterDot != null) Destroy(waterDot);
            if (waterMaterial != null) Destroy(waterMaterial);
            if (map != null) Destroy(map);
            if (shader != null) Destroy(shader);
        }
    }
}
