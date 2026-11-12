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
        [SerializeField] private bool useDielectricWaterPreview = false;
        [SerializeField] private bool useWaterRefractionPreview = false;
        [SerializeField, Range(21f, 100f)] private float hotWaterColourC = 30f;
        [SerializeField, Range(0f, 100f)] private float absorbedBurnerPowerW = 10f;
        [SerializeField, Range(0f, 30f), Tooltip("Synthetic film coefficient to a 20 °C room, in W/(m² K).")]
        private float roomFilmWm2K = 8f;

        [StructLayout(LayoutKind.Sequential)]
        private struct FaceLink
        {
            public int Other;
            public float Conductance;
        }

        private ComputeShader shader;
        private ComputeBuffer read, write, capacity, burner, faceStart, links, exposedArea, roomEnergy;
        private Texture2D map;
        private GameObject waterDisplay;
        private Mesh waterMesh;
        private Material waterMaterial;
        private int[] waterVertexCells;
        private Color32[] waterVertexColours;
        private BeakerSolidMesh mesh;
        private float[] snapshot;
        private float[] roomSnapshot;
        private double lostJ;
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
            roomSnapshot = new float[cellCount];
            var initial = new double[cellCount];
            for (int i = 0; i < cellCount; i++) initial[i] = InitialK + (i % 7) * .5;

            var reference = new SolidConduction(mesh.CapacityJPerK, initial, mesh.Faces);
            stepSeconds = Math.Min(.01, reference.MaximumStepSeconds * .5);
            for (int i = 0; i < cellCount; i++)
                if (mesh.ExposedAreaM2[i] > 0)
                    stepSeconds = Math.Min(stepSeconds, .5 * mesh.CapacityJPerK[i] / (30 * mesh.ExposedAreaM2[i]));
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
            exposedArea = new ComputeBuffer(cellCount, sizeof(float));
            roomEnergy = new ComputeBuffer(cellCount, sizeof(float));
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
            exposedArea.SetData(Array.ConvertAll(mesh.ExposedAreaM2, value => (float)value));
            roomEnergy.SetData(roomSnapshot);
            shader.SetBuffer(kernel, "ExposedArea", exposedArea);
            shader.SetBuffer(kernel, "RoomEnergyJ", roomEnergy);
            shader.SetBuffer(kernel, "FaceStart", faceStart);
            shader.SetBuffer(kernel, "FaceLinks", links);
            map = new Texture2D(RadialCells, DepthCells, TextureFormat.RGBA32, false)
                { filterMode = FilterMode.Point, wrapMode = TextureWrapMode.Clamp };

            read.SetData(Array.ConvertAll(initial, value => (float)(value - InitialK)));
            var checkPower = new double[cellCount];
            for (int i = 0; i < 20; i++)
            {
                Dispatch(10);
                for (int j = 0; j < cellCount; j++)
                    checkPower[j] = 10 * mesh.BurnerWeights[j] -
                        Mathf.Clamp(roomFilmWm2K, 0, 30) * mesh.ExposedAreaM2[j] * (reference.TemperatureK[j] - InitialK);
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
                SpawnWaterDisplay(geometry.innerRadiusM, geometry.externalBaseHeightM + geometry.baseThicknessM,
                    setup.scenario.display.cylinderSegments);
        }

        private void SpawnWaterDisplay(double radiusM, double floorM, int segments)
        {
            string shaderName = useDielectricWaterPreview
                ? (useWaterRefractionPreview ? "BeakerRefraction" : "BeakerDielectric")
                : "WaterTemperatureSurface";
            var shaderAsset = Resources.Load<Shader>("ThreePhase3D/" + shaderName);
            if (shaderAsset == null)
                throw new InvalidOperationException("The selected water surface shader is unavailable.");

            const int rings = BeakerSolidMesh.InnerRadialCells;
            const int layers = BeakerSolidMesh.WaterHeightCells;
            float radius = (float)(radiusM * .999);
            float floor = (float)(floorM + Math.Min(1e-5, mesh.WaterDepthM * .01));
            float depth = (float)(floorM + mesh.WaterDepthM) - floor;
            var vertices = new List<Vector3>();
            var normals = new List<Vector3>();
            var indices = new List<int>();
            var cells = new List<int>();

            void AddVertex(Vector3 position, Vector3 normal, int ring, int layer)
            {
                vertices.Add(position);
                normals.Add(normal);
                cells.Add(mesh.WaterStart + ring * layers + layer);
            }

            Vector3 Direction(int segment)
            {
                float angle = 2 * Mathf.PI * segment / segments;
                return new Vector3(Mathf.Cos(angle), 0, Mathf.Sin(angle));
            }

            void Triangle(int i, int j, int k, bool reverse)
            {
                indices.Add(i);
                indices.Add(reverse ? k : j);
                indices.Add(reverse ? j : k);
            }

            for (int cap = 0; cap < 2; cap++)
            {
                bool top = cap == 1;
                float y = top ? floor + depth : floor;
                Vector3 normal = top ? Vector3.up : Vector3.down;
                int layer = top ? layers - 1 : 0;
                int centre = vertices.Count;
                AddVertex(Vector3.up * y, normal, 0, layer);
                for (int r = 0; r < rings; r++)
                {
                    float edge = radius * (r == rings - 1 ? 1 : (r + .5f) / rings);
                    int first = vertices.Count;
                    for (int s = 0; s < segments; s++)
                        AddVertex(Direction(s) * edge + Vector3.up * y, normal, r, layer);

                    for (int s = 0; s < segments; s++)
                    {
                        int next = (s + 1) % segments;
                        if (r == 0)
                            Triangle(centre, first + s, first + next, top);
                        else
                        {
                            int inner = first - segments;
                            Triangle(inner + s, first + s, first + next, top);
                            Triangle(inner + s, first + next, inner + next, top);
                        }
                    }
                }
            }

            for (int z = 0; z < layers; z++)
            {
                float height = z == 0 ? 0 : z == layers - 1 ? 1 : (z + .5f) / layers;
                int first = vertices.Count;
                for (int s = 0; s < segments; s++)
                {
                    Vector3 direction = Direction(s);
                    AddVertex(direction * radius + Vector3.up * (floor + height * depth),
                        direction, rings - 1, z);
                }
                if (z == 0) continue;
                int below = first - segments;
                for (int s = 0; s < segments; s++)
                {
                    int next = (s + 1) % segments;
                    Triangle(below + s, first + s, first + next, false);
                    Triangle(below + s, first + next, below + next, false);
                }
            }

            waterMesh = new Mesh { name = "Water temperature surface" };
            waterMesh.MarkDynamic();
            waterMesh.SetVertices(vertices);
            waterMesh.SetNormals(normals);
            waterMesh.SetTriangles(indices, 0);
            waterVertexCells = cells.ToArray();
            waterVertexColours = new Color32[cells.Count];

            waterDisplay = new GameObject("Water temperature surface");
            waterDisplay.transform.SetParent(transform, false);
            waterDisplay.AddComponent<MeshFilter>().sharedMesh = waterMesh;
            waterMaterial = new Material(shaderAsset);
            if (useDielectricWaterPreview)
            {
                const float waterIndex = 1.333f; // O01, representative visible index.
                float amplitude = (waterIndex - 1) / (waterIndex + 1);
                waterMaterial.SetFloat("_NormalReflectance", amplitude * amplitude);
                if (useWaterRefractionPreview) waterMaterial.SetFloat("_RefractiveIndex", waterIndex);
            }
            waterDisplay.AddComponent<MeshRenderer>().sharedMaterial = waterMaterial;
            RefreshWaterColours();
        }

        private void RefreshWaterColours()
        {
            if (waterMesh == null) return;
            float span = Mathf.Max(1, hotWaterColourC - ColdWaterC);
            for (int i = 0; i < waterVertexColours.Length; i++)
            {
                float fraction = Mathf.Clamp01(snapshot[waterVertexCells[i]] / span);
                waterVertexColours[i] = Color.Lerp(ColdWaterColour, HotWaterColour, fraction);
            }
            waterMesh.colors32 = waterVertexColours;
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
                roomEnergy.GetData(roomSnapshot);
                storedJ = waterJ = lostJ = 0;
                for (int i = 0; i < snapshot.Length; i++)
                {
                    double cellJ = snapshot[i] * mesh.CapacityJPerK[i];
                    storedJ += cellJ;
                    lostJ += roomSnapshot[i];
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
            shader.SetFloat("RoomFilmWm2K", Mathf.Clamp(roomFilmWm2K, 0, 30));
            shader.SetBuffer(kernel, "TemperatureRead", read);
            shader.SetBuffer(kernel, "TemperatureWrite", write);
            shader.Dispatch(kernel, (snapshot.Length + 63) / 64, 1, 1);
            (read, write) = (write, read);
        }

        private void ResetTemperatures()
        {
            Array.Clear(snapshot, 0, snapshot.Length);
            absorbedJ = sampledAbsorbedJ = storedJ = waterJ = lostJ = 0;
            Array.Clear(roomSnapshot, 0, roomSnapshot.Length);
            roomEnergy.SetData(roomSnapshot);
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
                GUILayout.Label($"Absorbed {sampledAbsorbedJ:F2} J · Stored {storedJ:F2} J · To room {lostJ:F2} J");
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
            exposedArea?.Release();
            roomEnergy?.Release();
            faceStart?.Release();
            links?.Release();
            if (waterDisplay != null) Destroy(waterDisplay);
            if (waterMesh != null) Destroy(waterMesh);
            if (waterMaterial != null) Destroy(waterMaterial);
            if (map != null) Destroy(map);
            if (shader != null) Destroy(shader);
        }
    }
}
