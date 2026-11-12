using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;
using PhysicsSim.ThreePhase3D.Geometry;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Rendering
{
    public static class V19CavityGpuCheck
    {
        private const float Prandtl = 0.71f;
        private const float Rayleigh = 1000;
        private static readonly Vector3 Reference = new Vector3(1.118f, 3.649f, 3.697f);

        public static void Run(Func<string, float, bool> cancel)
        {
            if (!SystemInfo.supportsComputeShaders) throw new NotSupportedException("V19 requires compute shaders.");
            Vector3 coarse = RunMesh(8, 1, cancel);
            Vector3 medium = RunMesh(16, 1, cancel);
            Vector3 fine = RunMesh(32, 1, cancel);
            Vector3 halfStep = RunMesh(16, 0.5, cancel);
            Vector3 temporal = Relative(halfStep - medium);
            if (Relative(fine - Reference).magnitude >= Relative(medium - Reference).magnitude ||
                Relative(medium - Reference).magnitude >= Relative(coarse - Reference).magnitude ||
                MaxAbs(Relative(fine - Reference)) > 0.10f || MaxAbs(temporal) > 0.01f)
                throw new InvalidOperationException("V19 square cavity missed the benchmark or refinement limits.");
            Debug.Log($"S09 V19 square GPU cavity passed: maximum fine-grid reference error {MaxAbs(Relative(fine - Reference)):P2}, timestep change {MaxAbs(temporal):P2}.");
        }

        public static void RunThreeDimensional(Func<string, float, bool> cancel) => RunMesh(8, 1, cancel, true);

        private static Vector3 RunMesh(int n, double stepScale, Func<string, float, bool> cancel, bool cube = false)
        {
            // Thermal-diffusion units: length L, time L²/alpha, temperature delta T.
            double h = 1.0 / n;
            int depth = cube ? n : 1;
            var grid = new Grid3D(n, n, depth, new D3(h, h, h));
            var noSlip = new FlowBoundary(FlowBoundaryKind.NoSlip);
            var symmetry = new FlowBoundary(cube ? FlowBoundaryKind.NoSlip : FlowBoundaryKind.FreeSlip);
            var plan = new VariableCoefficientProjection(CutCellGeometry.Build(grid),
                new BoundaryConditions3D(noSlip, noSlip, noSlip, noSlip, symmetry, symmetry),
                _ => 1, _ => Prandtl);

            var heat = Resources.Load<ComputeShader>("ThreePhase3D/V19CavityHeat");
            var momentum = Resources.Load<ComputeShader>("ThreePhase3D/SinglePhaseFlow");
            if (heat == null || momentum == null) throw new InvalidOperationException("V19 compute shaders are missing.");
            int kernel = heat.FindKernel("AdvanceHeat");
            if (!heat.IsSupported(kernel)) throw new NotSupportedException("V19 heat kernel is unsupported.");
            int count = n * n * depth;
            var groups = new int[count];
            var faces = new FaceRow[count];
            var temperatures = new float[count];
            var previousTemperature = new float[count];
            var velocity = new float[plan.FaceCount];
            var previousVelocity = new float[plan.FaceCount];
            for (int z = 0; z < depth; z++)
            for (int y = 0; y < n; y++)
            for (int x = 0; x < n; x++)
            {
                int cell = (z * n + y) * n + x;
                groups[cell] = plan.PressureGroup(x, y, z);
                faces[cell] = new FaceRow(plan.FaceIndex(0, x, y, z), plan.FaceIndex(0, x + 1, y, z),
                    plan.FaceIndex(1, x, y, z), plan.FaceIndex(1, x, y + 1, z),
                    plan.FaceIndex(2, x, y, z), plan.FaceIndex(2, x, y, z + 1));
                temperatures[groups[cell]] = 0.5f - (x + 0.5f) / n;
            }

            using var solver = new FlowGpuSolver(new MomentumPredictor3D(plan), momentum, true);
            using var groupBuffer = new ComputeBuffer(count, sizeof(int));
            using var faceBuffer = new ComputeBuffer(count, 6 * sizeof(int));
            using var velocityBuffer = new ComputeBuffer(plan.FaceCount, sizeof(float));
            using var temperatureA = new ComputeBuffer(count, sizeof(float));
            using var temperatureB = new ComputeBuffer(count, sizeof(float));
            groupBuffer.SetData(groups);
            faceBuffer.SetData(faces);
            temperatureA.SetData(temperatures);
            heat.SetInt("Width", n);
            heat.SetInt("Depth", depth);
            heat.SetFloat("InverseSpacing", n);
            heat.SetBuffer(kernel, "CellGroups", groupBuffer);
            heat.SetBuffer(kernel, "CellFaces", faceBuffer);
            heat.SetBuffer(kernel, "FaceVelocity", velocityBuffer);
            ComputeBuffer current = temperatureA, next = temperatureB;
            double dt = (cube ? 0.08 : 0.12) * h * h * stepScale;
            int sampleEvery = (int)Math.Ceiling(0.02 / dt);
            int steps = (int)Math.Ceiling(1 / dt);
            double[] pressure = null;
            for (int step = 1; step <= steps; step++)
            {
                var flow = solver.Advance(velocity, new D3(0, 0, 0), dt, 8192,
                    current, Rayleigh * Prandtl, pressure);
                if (flow.Diagnostics.Dt < dt * 0.99999)
                    throw new InvalidOperationException("V19 requested timestep exceeded the momentum stability bound.");
                velocity = flow.VelocityMPerS;
                pressure = flow.GroupPressurePa;
                velocityBuffer.SetData(velocity);
                heat.SetFloat("Dt", (float)flow.Diagnostics.Dt);
                heat.SetBuffer(kernel, "TemperatureIn", current);
                heat.SetBuffer(kernel, "TemperatureOut", next);
                heat.Dispatch(kernel, (count + 63) / 64, 1, 1);
                (current, next) = (next, current);

                if (step % 32 == 0 && (cancel?.Invoke(
                    $"{n}×{n}, timestep scale {stepScale:G}, step {step}/{steps}", (float)step / steps) ?? false))
                    throw new OperationCanceledException();
                if (step % sampleEvery != 0) continue;

                current.GetData(temperatures);
                Vector3 metrics = Measure(plan, groups, temperatures, velocity, n, depth, out float wallDifference);
                float temperatureChange = MaxChange(temperatures, previousTemperature);
                float velocityChange = MaxChange(velocity, previousVelocity);
                if (step >= 2 * sampleEvery && temperatureChange < 2e-5f &&
                    velocityChange < 2e-4f && wallDifference < 0.001f)
                {
                    Debug.Log($"S09 V19 {n}×{n}, dt {flow.Diagnostics.Dt:G5}: Nu {metrics.x:F5}, Umax {metrics.y:F5}, Vmax {metrics.z:F5}, wall Nu difference {wallDifference:G4}.");
                    if (cube)
                    {
                        float spanwisePeak = 0;
                        for (int z = 1; z < depth; z++)
                        for (int y = 0; y < n; y++)
                        for (int x = 0; x < n; x++)
                            spanwisePeak = Mathf.Max(spanwisePeak, Mathf.Abs(velocity[plan.FaceIndex(2, x, y, z)]));
                        if (spanwisePeak <= 0.001f || metrics.x <= 1 || metrics.y <= 0 || metrics.z <= 0)
                            throw new InvalidOperationException("V19 cube did not develop three-dimensional convection.");
                        Debug.Log($"S09 V19 3D GPU cavity passed: {n} cubed, Nu {metrics.x:F5}, peak spanwise speed {spanwisePeak:F5}.");
                    }
                    return metrics;
                }
                Array.Copy(temperatures, previousTemperature, count);
                Array.Copy(velocity, previousVelocity, velocity.Length);
            }
            throw new InvalidOperationException($"V19 {n}×{n} did not reach steady state within one diffusion time.");
        }

        private static Vector3 Measure(VariableCoefficientProjection plan, int[] groups, float[] t,
            float[] velocity, int n, int depth, out float wallDifference)
        {
            float hot = 0, cold = 0, u = 0, v = 0;
            for (int z = 0; z < depth; z++)
            for (int i = 0; i < n; i++)
            {
                hot += 2 * (0.5f - t[groups[(z * n + i) * n]]);
                cold += 2 * (t[groups[(z * n + i) * n + n - 1]] + 0.5f);
                u = Mathf.Max(u, velocity[plan.FaceIndex(0, n / 2, i, z)]);
                v = Mathf.Max(v, velocity[plan.FaceIndex(1, i, n / 2, z)]);
            }
            hot /= depth;
            cold /= depth;
            wallDifference = Mathf.Abs(hot - cold);
            return new Vector3((hot + cold) / 2, u, v);
        }

        private static float MaxChange(float[] current, float[] previous)
        {
            float maximum = 0;
            for (int i = 0; i < current.Length; i++)
            {
                if (!SiUnits.IsFinite(current[i])) throw new InvalidOperationException("V19 produced nonfinite state.");
                maximum = Mathf.Max(maximum, Mathf.Abs(current[i] - previous[i]));
            }
            return maximum;
        }

        private static Vector3 Relative(Vector3 difference) =>
            new Vector3(difference.x / Reference.x, difference.y / Reference.y, difference.z / Reference.z);

        private static float MaxAbs(Vector3 v) => Mathf.Max(Mathf.Abs(v.x), Mathf.Max(Mathf.Abs(v.y), Mathf.Abs(v.z)));

        private readonly struct FaceRow
        {
            public readonly int Left, Right, Bottom, Top, Front, Back;
            public FaceRow(int left, int right, int bottom, int top, int front, int back)
            {
                Left = left;
                Right = right;
                Bottom = bottom;
                Top = top;
                Front = front;
                Back = back;
            }
        }
    }
}
