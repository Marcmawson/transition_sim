using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Rendering
{
    [RequireComponent(typeof(Beaker3DSetup))]
    public sealed class BeakerReleasePreview : MonoBehaviour
    {
        private static readonly D3 Gravity = new D3(0, -9.81, 0);
        private readonly List<GameObject> waterCells = new List<GameObject>();
        private BeakerVofTimeIntegration release;
        private CutCellVofTransport transport;
        private GameObject ice;
        private Material waterMaterial, iceMaterial;
        private GameObject cavityGuides;
        private Material guideMaterial;
        private bool running;
        private float nextStep;
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
            if (running && Time.unscaledTime >= nextStep) Advance();
        }

        private void SpawnWater()
        {
            if (solverWork != null) return;
            Clear();
            StartSolverWork(() =>
            {
                var simulation = BeakerVofTimeIntegration.CreateRelease(9e-6, false, 293.15, 101325);
                var simulationTransport = new CutCellVofTransport(simulation.Plan);
                return () =>
                {
                    release = simulation;
                    transport = simulationTransport;
                    if (waterMaterial == null) waterMaterial = CreateMaterial(new Color(.06f, .73f, .96f));
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
            var simulation = release;
            var simulationTransport = transport;
            StartSolverWork(() =>
            {
                simulation.Step((state, flow) => simulationTransport.AdvanceCoupledWithAcceleration(state, flow,
                    simulation.LiquidDensityKgM3, BeakerVofTimeIntegration.GasDensity, Gravity,
                    maxIterations: BeakerVofTimeIntegration.IterationLimit));
                return ShowWaterState;
            });
        }

        private void StartSolverWork(Func<Action> work)
        {
            workGeneration = generation;
            solverWork = Task.Run(work);
        }

        private void ShowWaterState()
        {
            DrawWater();
            waterStatus = $"9 mL water  |  {release.TimeS:F3} s  |  Step {release.Steps}";
            volumeStatus = $"Liquid volume drift {release.LiquidDriftM3:E2} m³";
            nextStep = Time.unscaledTime + .15f;
        }

        private void DrawWater()
        {
            var grid = release.Geometry.Grid;
            int visible = 0;
            for (int k = 0; k < grid.Cells.Z; k++)
            for (int j = 0; j < grid.Cells.Y; j++)
            for (int i = 0; i < grid.Cells.X; i++)
            {
                int cell = grid.Cells.Index(i, j, k);
                double fraction = release.LiquidFractionAt(i, j, k) *
                    release.Geometry.FluidVolumeM3[cell] / grid.CellVolume;
                if (fraction < .01) continue;
                if (visible == waterCells.Count) waterCells.Add(CreateBlock("Water cell", waterMaterial));
                var block = waterCells[visible++];
                var centre = grid.Centre(i, j, k);
                block.transform.localPosition = new Vector3((float)centre.X, (float)centre.Y, (float)centre.Z);
                block.transform.localScale = new Vector3((float)grid.Spacing.X, (float)grid.Spacing.Y,
                    (float)grid.Spacing.Z) * Mathf.Pow((float)fraction, 1f / 3f);
                block.SetActive(true);
            }
            for (int i = visible; i < waterCells.Count; i++) waterCells[i].SetActive(false);
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
            GUILayout.BeginArea(new Rect(16, 16, Mathf.Min(Screen.width - 32, 370), 184), GUI.skin.box);
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
                GUILayout.Label(release == null ? "Preparing water release…" : "Solving next step…");
            if (error != null) GUILayout.Label(error);
            GUILayout.EndArea();
        }

        private void Fail(Exception exception)
        {
            running = false;
            error = exception.Message;
            Debug.LogException(exception, this);
        }

        private void ToggleRunning() => running = !running;

        private void Clear()
        {
            running = false;
            nextStep = 0;
            generation++;
            release = null;
            transport = null;
            error = null;
            foreach (var block in waterCells) block.SetActive(false);
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
            foreach (var block in waterCells) if (block != null) Destroy(block);
            waterCells.Clear();
            if (waterMaterial != null) Destroy(waterMaterial);
            if (iceMaterial != null) Destroy(iceMaterial);
            if (cavityGuides != null) Destroy(cavityGuides);
            if (guideMaterial != null) Destroy(guideMaterial);
        }
    }
}
