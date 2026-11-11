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

        [SerializeField] private bool largerBeaker = false;
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
        private BeakerSolidMesh mesh;
        private float[] snapshot;
        private readonly Color32[] pixels = new Color32[RadialCells * DepthCells];
        private double absorbedJ, sampledAbsorbedJ, storedJ;
        private int kernel;
        private double stepSeconds, pendingSeconds, simulatedSeconds, nextSnapshot;
        private bool burnerOn, running, ready;
        private string status;

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

            mesh = new BeakerSolidMesh(geometry);
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
                storedJ = 0;
                for (int i = 0; i < snapshot.Length; i++) storedJ += snapshot[i] * mesh.CapacityJPerK[i];
                sampledAbsorbedJ = absorbedJ;
                RefreshMap();
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
            absorbedJ = sampledAbsorbedJ = storedJ = 0;
            read.SetData(snapshot);
            simulatedSeconds = pendingSeconds = nextSnapshot = 0;
            running = burnerOn = false;
            RefreshMap();
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
            GUILayout.BeginArea(new Rect(12, 12, Mathf.Max(1, Mathf.Min(430, Screen.width - 24)), 300), GUI.skin.box);
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
                GUILayout.Label("Synthetic dry apparatus · no water or ambient exchange");
            }
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
            if (map != null) Destroy(map);
            if (shader != null) Destroy(shader);
        }
    }
}
