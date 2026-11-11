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
                var step = solver.Advance(velocity, new D3(), .1);
                velocity = step.VelocityMPerS;
                simulatedSeconds += step.Diagnostics.Dt;
                double dtDivergence = step.Diagnostics.Dt * step.Diagnostics.MaxDivergencePerS;
                status = $"t = {simulatedSeconds:F2} s   |   dt × max divergence = {dtDivergence:E2}";
                nextUpdate = Time.unscaledTime + .15f;
            }
            catch (Exception failure) { Stop(failure); }
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
        }

        private void OnGUI() => GUI.Label(new Rect(12, 12, 740, 60),
            "S04 pressure-driven channel\n" + status + "\nVelocity arrows are shown in Scene view with Gizmos enabled.");

        private void OnDrawGizmos()
        {
            Gizmos.matrix = transform.localToWorldMatrix;
            var size = new Vector3((float)(Nx * Spacing.X), (float)(Ny * Spacing.Y), (float)(Nz * Spacing.Z));
            Gizmos.color = Color.gray;
            Gizmos.DrawWireCube(size * .5f, size);
            if (velocity == null) return;
            Gizmos.color = Color.cyan;
            for (int j = 0; j < Ny; j++)
            for (int i = 0; i <= Nx; i++)
            {
                float speed = velocity[projection.FaceIndex(0, i, j, Nz / 2)];
                var start = new Vector3((float)(i * Spacing.X), (float)((j + .5) * Spacing.Y), size.z * .5f);
                var end = start + Vector3.right * (speed * .12f);
                Gizmos.DrawLine(start, end);
                if (Mathf.Abs(speed) > 1e-5f)
                    Gizmos.DrawLine(end, end + new Vector3(-Mathf.Sign(speed) * .0003f, .00015f, 0));
            }
        }
    }
}
