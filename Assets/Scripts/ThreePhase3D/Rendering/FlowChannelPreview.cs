using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;
using PhysicsSim.ThreePhase3D.Geometry;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Rendering
{
    public sealed class FlowChannelPreview : MonoBehaviour
    {
        private static readonly D3 Spacing = new D3(.005, .0025, .005);
        private const int Nx = 4, Ny = 8, Nz = 3;
        private VariableCoefficientProjection projection;
        private FlowGpuSolver solver;
        private float[] velocity;
        private float nextUpdate;
        private double simulatedSeconds;
        private string status;
        private GameObject visualRoot;
        private Material frameMaterial, arrowMaterial;
        private LineRenderer[] arrows;
        private static Vector3 ChannelSize => new Vector3(
            (float)(Nx * Spacing.X), (float)(Ny * Spacing.Y), (float)(Nz * Spacing.Z));

        private void OnEnable()
        {
            if (!Application.isPlaying) return;
            try
            {
                var openIn = new FlowBoundary(FlowBoundaryKind.OpenPressure, 0.6);
                var openOut = new FlowBoundary(FlowBoundaryKind.OpenPressure);
                var wall = new FlowBoundary(FlowBoundaryKind.NoSlip);
                var slip = new FlowBoundary(FlowBoundaryKind.FreeSlip);
                var grid = new Grid3D(Nx, Ny, Nz, Spacing);
                projection = new VariableCoefficientProjection(CutCellGeometry.Build(grid),
                    new BoundaryConditions3D(openIn, openOut, wall, wall, slip, slip), _ => 1000, _ => .01);
                solver = new FlowGpuSolver(new MomentumPredictor3D(projection),
                    Resources.Load<ComputeShader>("ThreePhase3D/SinglePhaseFlow"));
                velocity = new float[projection.FaceCount];
                CreateVisuals();
                nextUpdate = Time.unscaledTime;
                simulatedSeconds = 0;
                status = "Starting from rest";
            }
            catch (Exception failure) { Stop(failure); }
        }

        private void Update()
        {
            if (solver == null || Time.unscaledTime < nextUpdate) return;
            try
            {
                var step = AdvancePreview();
                velocity = step.VelocityMPerS;
                UpdateArrows();
                simulatedSeconds += step.Diagnostics.Dt;
                double dtDivergence = step.Diagnostics.Dt * step.Diagnostics.MaxDivergencePerS;
                status = $"t = {simulatedSeconds:F2} s   |   dt × max divergence = {dtDivergence:E2}";
                nextUpdate = Time.unscaledTime + .15f;
            }
            catch (Exception failure) { Stop(failure); }
        }

        private FlowStep AdvancePreview()
        {
            double requested = .1;
            for (int retry = 0; ; retry++)
            {
                try { return solver.Advance(velocity, new D3(), requested); }
                catch (FlowConvergenceException failure) when (retry < 3)
                {
                    requested = failure.Diagnostics.Dt * .5;
                }
            }
        }

        private void Stop(Exception failure)
        {
            solver?.Dispose();
            solver = null;
            status = failure.Message;
            Debug.LogException(failure, this);
        }

        private void OnDisable()
        {
            solver?.Dispose();
            solver = null;
            velocity = null;
            if (visualRoot != null) Destroy(visualRoot);
            if (frameMaterial != null) Destroy(frameMaterial);
            if (arrowMaterial != null) Destroy(arrowMaterial);
        }

        private void OnGUI() => GUI.Label(new Rect(16, 16, 500, 60),
            "S04 pressure-driven channel\n" + status + "\nCyan arrows show axial velocity.");

        private void CreateVisuals()
        {
            var shader = Shader.Find("Unlit/Color");
            if (shader == null) throw new InvalidOperationException("The preview requires the built-in Unlit/Color shader.");
            frameMaterial = new Material(shader) { color = new Color(.65f, .74f, .83f, 1f) };
            arrowMaterial = new Material(shader) { color = Color.cyan };
            visualRoot = new GameObject("Channel frame and velocity arrows");
            visualRoot.transform.SetParent(transform, false);

            Vector3 size = ChannelSize;
            for (int corner = 0; corner < 8; corner++)
            for (int axis = 0; axis < 3; axis++)
            {
                int bit = 1 << axis;
                if ((corner & bit) != 0) continue;
                var line = CreateLine("Channel edge", frameMaterial, .00016f, 2);
                line.SetPosition(0, Corner(corner, size));
                line.SetPosition(1, Corner(corner | bit, size));
            }

            arrows = new LineRenderer[(Nx + 1) * Ny];
            for (int index = 0; index < arrows.Length; index++)
                arrows[index] = CreateLine("Velocity arrow", arrowMaterial, .00014f, 5);
            UpdateArrows();
        }

        private static Vector3 Corner(int index, Vector3 size) => new Vector3(
            (index & 1) == 0 ? 0 : size.x,
            (index & 2) == 0 ? 0 : size.y,
            (index & 4) == 0 ? 0 : size.z);

        private LineRenderer CreateLine(string objectName, Material material, float width, int points)
        {
            var child = new GameObject(objectName);
            child.transform.SetParent(visualRoot.transform, false);
            var line = child.AddComponent<LineRenderer>();
            line.useWorldSpace = false;
            line.sharedMaterial = material;
            line.widthMultiplier = width;
            line.positionCount = points;
            return line;
        }

        private void UpdateArrows()
        {
            for (int j = 0; j < Ny; j++)
            for (int i = 0; i <= Nx; i++)
            {
                var arrow = arrows[j * (Nx + 1) + i];
                float speed = velocity[projection.FaceIndex(0, i, j, Nz / 2)];
                float length = Mathf.Clamp(speed * .026f, -.0044f, .0044f);
                arrow.enabled = Mathf.Abs(length) > 1e-6f;
                if (!arrow.enabled) continue;
                var start = new Vector3((float)(i * Spacing.X), (float)((j + .5) * Spacing.Y), ChannelSize.z * .5f);
                var end = start + Vector3.right * length;
                float head = Mathf.Min(Mathf.Abs(length) * .35f, .0006f);
                var back = end - Vector3.right * (Mathf.Sign(length) * head);
                arrow.SetPosition(0, start);
                arrow.SetPosition(1, end);
                arrow.SetPosition(2, back + Vector3.up * (head * .6f));
                arrow.SetPosition(3, end);
                arrow.SetPosition(4, back - Vector3.up * (head * .6f));
            }
        }
    }
}
