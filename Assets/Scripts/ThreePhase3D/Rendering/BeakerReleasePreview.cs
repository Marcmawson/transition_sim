using System;
using System.Threading.Tasks;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Rendering
{
    [RequireComponent(typeof(Beaker3DSetup))]
    public sealed class BeakerReleasePreview : MonoBehaviour
    {
        private BeakerWaterSurface waterSurface;
        private Mesh waterMesh;
        private GameObject waterObject;
        private BeakerVofTimeIntegration release;
        private BeakerVofGpuSession gpuSession;
        private GameObject ice;
        private Material waterMaterial, iceMaterial;
        private GameObject cavityGuides;
        private Material guideMaterial;
        private bool running;
        private const double SurfaceRefreshSeconds = 1.0 / 30.0;
        private double nextSurfaceRefresh;
        private double solverStepMs, surfaceBuildMs;
        private float frameMs;
        private string error;
        private Action pendingAction;
        private Task<Action> solverWork;
        private int generation, workGeneration;
        private string waterStatus, volumeStatus;

        private void OnEnable()
        {
            var setup = GetComponent<Beaker3DSetup>();
            setup.scenario.physical.geometry = BeakerVofTimeIntegration.GeometryInputsForPreview();
            setup.scenario.display.cylinderSegments = BeakerVofTimeIntegration.Segments;
            setup.scenario.display.glassOpacity = .1f;
            setup.scenario.display.showSetupReport = false;
            setup.Refresh();
            DrawCavityGuides(setup.scenario.physical.geometry);
        }

        private void DrawCavityGuides(GeometryInputs geometry)
        {
            var shader = Shader.Find("Sprites/Default");
            if (shader == null) return;
            cavityGuides = new GameObject("Cavity outlines");
            cavityGuides.transform.SetParent(transform, false);
            guideMaterial = new Material(shader);
            float floor = (float)(geometry.externalBaseHeightM + geometry.baseThicknessM);
            float top = floor + (float)geometry.cavityHeightM;
            float radius = (float)geometry.innerRadiusM;
            var lower = new Vector3[BeakerVofTimeIntegration.Segments];
            var upper = new Vector3[lower.Length];
            for (int i = 0; i < lower.Length; i++)
            {
                float angle = 2 * Mathf.PI * i / lower.Length;
                lower[i] = new Vector3(radius * Mathf.Cos(angle), floor, radius * Mathf.Sin(angle));
                upper[i] = new Vector3(lower[i].x, top, lower[i].z);
            }
            var edge = new Color(.53f, .78f, .89f, .85f);
            AddLine("Inner rim", upper, edge, true);
            AddLine("Cavity floor edge", lower, edge, true);
            for (int i = 0; i < lower.Length; i += lower.Length / 4)
                AddLine("Cavity wall guide", new[] { lower[i], upper[i] }, new Color(.53f, .78f, .89f, .48f), false);
        }

        private void AddLine(string label, Vector3[] points, Color color, bool closed)
        {
            var line = new GameObject(label).AddComponent<LineRenderer>();
            line.transform.SetParent(cavityGuides.transform, false);
            line.useWorldSpace = false;
            line.sharedMaterial = guideMaterial;
            line.startColor = line.endColor = color;
            line.widthMultiplier = .001f;
            line.loop = closed;
            line.positionCount = points.Length;
            line.SetPositions(points);
        }

        private void Update()
        {
            float elapsedMs = Time.unscaledDeltaTime * 1000f;
            if (elapsedMs > 0) frameMs = frameMs == 0 ? elapsedMs : Mathf.Lerp(frameMs, elapsedMs, .1f);
            var action = pendingAction;
            pendingAction = null;
            action?.Invoke();
            if (solverWork != null && solverWork.IsCompleted)
            {
                var completed = solverWork;
                solverWork = null;
                try
                {
                    var apply = completed.GetAwaiter().GetResult();
                    if (workGeneration == generation) apply();
                }
                catch (Exception exception)
                {
                    if (workGeneration == generation) Fail(exception);
                }
            }
            if (running) Advance();
        }

        private void SpawnWater()
        {
            if (solverWork != null) return;
            Clear();
            StartSolverWork(() =>
            {
                var simulation = BeakerVofTimeIntegration.CreateRelease(9e-6, false, 293.15, 101325, surfaceJitterM: .003);
                return () =>
                {
                    gpuSession = new BeakerVofGpuSession(simulation);
                    release = simulation;
                    CreateWaterSurface();
                    ShowWaterState();
                };
            });
        }

        private void SpawnIce()
        {
            Clear();
            try
            {
                if (iceMaterial == null) iceMaterial = CreateMaterial(new Color(.77f, .91f, 1f));
                var geometry = GetComponent<Beaker3DSetup>().scenario.physical.geometry;
                const float side = .02f;
                ice = CreateBlock("Stationary ice preview", iceMaterial);
                ice.transform.localPosition = new Vector3(0, (float)(geometry.externalBaseHeightM + geometry.baseThicknessM) + side / 2, 0);
                ice.transform.localScale = Vector3.one * side;
            }
            catch (Exception exception) { Fail(exception); }
        }

        private void Advance()
        {
            if (solverWork != null || release == null) return;
            try
            {
                var timer = System.Diagnostics.Stopwatch.StartNew();
                gpuSession.Step();
                solverStepMs = timer.Elapsed.TotalMilliseconds;
                ShowWaterState();
            }
            catch (Exception exception) { Fail(exception); }
        }

        private void StartSolverWork(Func<Action> work)
        {
            workGeneration = generation;
            solverWork = Task.Run(work);
        }

        private void ShowWaterState()
        {
            double now = Time.realtimeSinceStartupAsDouble;
            if (!running || now >= nextSurfaceRefresh)
            {
                var timer = System.Diagnostics.Stopwatch.StartNew();
                waterSurface.Rebuild(release, waterMesh);
                surfaceBuildMs = timer.Elapsed.TotalMilliseconds;
                nextSurfaceRefresh = now + SurfaceRefreshSeconds;
            }
            waterStatus = $"9 mL water · GPU  |  {release.TimeS:F3} s  |  Step {release.Steps}  |  Δt {release.Pressure.Diagnostics.Dt * 1000:F3} ms";
            volumeStatus = $"Liquid volume drift {release.LiquidDriftM3:E2} m³";
        }

        private void CreateWaterSurface()
        {
            var shader = Resources.Load<Shader>("ThreePhase3D/BeakerWaterSurface");
            if (shader == null) throw new InvalidOperationException("The beaker water surface shader is unavailable.");
            var geometry = GetComponent<Beaker3DSetup>().scenario.physical.geometry;
            waterMaterial = new Material(shader);
            waterMaterial.SetVector("_Beaker", new Vector4((float)geometry.innerRadiusM,
                (float)geometry.OuterRadiusM, (float)(geometry.externalBaseHeightM + geometry.baseThicknessM),
                (float)geometry.RimHeightM));
            waterMaterial.SetFloat("_BaseY", (float)geometry.externalBaseHeightM);
            waterMesh = new Mesh { name = "Beaker liquid surface", indexFormat = UnityEngine.Rendering.IndexFormat.UInt32 };
            waterMesh.MarkDynamic();
            waterSurface = new BeakerWaterSurface(release);
            waterObject = new GameObject("Water surface");
            waterObject.transform.SetParent(transform, false);
            waterObject.AddComponent<MeshFilter>().sharedMesh = waterMesh;
            waterObject.AddComponent<MeshRenderer>().sharedMaterial = waterMaterial;
        }

        private GameObject CreateBlock(string label, Material material)
        {
            var block = GameObject.CreatePrimitive(PrimitiveType.Cube);
            block.name = label;
            block.transform.SetParent(transform, false);
            block.GetComponent<Renderer>().sharedMaterial = material;
            var collider = block.GetComponent<Collider>();
            collider.enabled = false;
            Destroy(collider);
            return block;
        }

        private static Material CreateMaterial(Color color)
        {
            var shader = Shader.Find("Standard");
            if (shader == null) throw new InvalidOperationException("The Standard shader is unavailable.");
            return new Material(shader) { color = color };
        }

        private void OnGUI()
        {
            GUILayout.BeginArea(new Rect(16, 16, Mathf.Min(Screen.width - 32, 370), 236), GUI.skin.box);
            GUILayout.Label("S07 Beaker release");
            GUILayout.BeginHorizontal();
            bool controlsEnabled = GUI.enabled;
            GUI.enabled = controlsEnabled && solverWork == null;
            if (GUILayout.Button("Spawn Water")) pendingAction = SpawnWater;
            GUI.enabled = controlsEnabled;
            if (GUILayout.Button("Spawn Ice")) pendingAction = SpawnIce;
            if (GUILayout.Button("Clear")) pendingAction = Clear;
            GUILayout.EndHorizontal();

            if (release != null)
            {
                GUILayout.Label(waterStatus);
                GUILayout.Label(volumeStatus);
                if (frameMs > 0)
                    GUILayout.Label($"Game {1000f / frameMs:F1} FPS  |  Step {solverStepMs:F1} ms ({release.Pressure.Diagnostics.Iterations} it)  |  Surface {surfaceBuildMs:F1} ms");
                if (error == null)
                {
                    GUILayout.BeginHorizontal();
                    if (GUILayout.Button(running ? "Pause" : "Run")) pendingAction = ToggleRunning;
                    GUI.enabled = controlsEnabled && solverWork == null;
                    if (!running && GUILayout.Button("Step")) pendingAction = Advance;
                    GUI.enabled = controlsEnabled;
                    GUILayout.EndHorizontal();
                }
            }
            else if (ice != null) GUILayout.Label("20 mm ice block · stationary preview");
            if (solverWork != null && workGeneration == generation)
                GUILayout.Label("Preparing water release…");
            if (error != null) GUILayout.Label(error);
            GUILayout.Label("WASD move · Q/E down/up · Shift faster");
            GUILayout.Label("Right mouse look · Wheel zoom · R reset");
            GUILayout.Label("I invert vertical mouse look");
            GUILayout.EndArea();
        }

        private void Fail(Exception exception)
        {
            running = false;
            error = exception.Message;
            Debug.LogException(exception, this);
        }

        private void ToggleRunning()
        {
            running = !running;
            if (running) return;
            try { ShowWaterState(); }
            catch (Exception exception) { Fail(exception); }
        }

        private void Clear()
        {
            running = false;
            nextSurfaceRefresh = 0;
            solverStepMs = 0;
            surfaceBuildMs = 0;
            generation++;
            release = null;
            gpuSession?.Dispose();
            gpuSession = null;
            error = null;
            if (waterObject != null)
            {
                waterObject.SetActive(false);
                Destroy(waterObject);
            }
            if (waterMesh != null) Destroy(waterMesh);
            if (waterMaterial != null) Destroy(waterMaterial);
            waterObject = null;
            waterMesh = null;
            waterMaterial = null;
            waterSurface = null;
            if (ice != null)
            {
                ice.SetActive(false);
                Destroy(ice);
                ice = null;
            }
        }

        private void OnDisable()
        {
            pendingAction = null;
            Clear();
            if (iceMaterial != null) Destroy(iceMaterial);
            if (cavityGuides != null) Destroy(cavityGuides);
            if (guideMaterial != null) Destroy(guideMaterial);
        }
    }
}
