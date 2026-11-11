using System;
using System.Linq;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;
using PhysicsSim.ThreePhase3D.Geometry;
using PhysicsSim.ThreePhase3D.Materials;
using PhysicsSim.ThreePhase3D.Thermal;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Rendering
{
    public static class LiquidBuoyancyGpuCheck
    {
        public static void Run()
        {
            const double transportDensity = 1000;
            const float dt = .05f, gravity = 9.80665f;
            if (!SystemInfo.supportsComputeShaders)
                throw new NotSupportedException("The S09 buoyancy check requires compute shader support.");
            var shader = Resources.Load<ComputeShader>("ThreePhase3D/LiquidBuoyancy");
            var pressure = Resources.Load<ComputeShader>("ThreePhase3D/SinglePhaseFlow");
            if (shader == null || pressure == null) throw new InvalidOperationException("S09 buoyancy or pressure shader is missing.");

            var grid = new Grid3D(4, 4, 2, new D3(.01, .01, .01));
            var plan = new VariableCoefficientProjection(CutCellGeometry.Build(grid),
                BoundaryConditions3D.Closed, _ => transportDensity, _ => 0);
            var table = new WaterPropertyTable(WaterPropertyTable.Generate(WaterBranch.Liquid));
            var enthalpy = new FluidEnthalpy(table, transportDensity);
            var packed = table.CopyValues();
            var density = Enumerable.Range(0, table.Count).Select(i => packed[i * WaterPropertyTable.Width]).ToArray();
            var vertical = new Vector2Int[plan.FaceCount];
            for (int f = 0; f < vertical.Length; f++)
            {
                var face = plan.Faces[f];
                vertical[f] = face.Active != 0 && face.Axis == 1 && face.Low >= 0 && face.High >= 0
                    ? new Vector2Int(face.Low, face.High) : new Vector2Int(-1, -1);
            }

            using (var transport = new CutCellGpuScalars(plan, Resources.Load<ComputeShader>("ThreePhase3D/CutCellVofScalars")))
            using (var recovery = new LiquidEnthalpyGpuRecovery(
                Resources.Load<ComputeShader>("ThreePhase3D/LiquidEnthalpyRecovery"), table, plan.GroupCount, transportDensity))
            using (var densities = new ComputeBuffer(density.Length, sizeof(float)))
            using (var neighbours = new ComputeBuffer(vertical.Length, 2 * sizeof(int)))
            using (var initialVelocity = new ComputeBuffer(vertical.Length, sizeof(float)))
            using (var tentative = new ComputeBuffer(vertical.Length, sizeof(float)))
            {
                int kernel = shader.FindKernel("PredictBuoyancy");
                if (!shader.IsSupported(kernel)) throw new NotSupportedException("S09 buoyancy kernel is unsupported.");
                densities.SetData(density);
                neighbours.SetData(vertical);
                initialVelocity.SetData(new float[vertical.Length]);
                shader.SetInt("FaceCount", plan.FaceCount);
                shader.SetInt("TableCount", table.Count);
                shader.SetFloat("MinimumK", (float)table.MinimumK);
                shader.SetFloat("StepK", (float)table.StepK);
                shader.SetFloat("GravityMagnitude", gravity);
                shader.SetFloat("Dt", dt);
                shader.SetBuffer(kernel, "DensityTable", densities);
                shader.SetBuffer(kernel, "VerticalNeighbours", neighbours);
                shader.SetBuffer(kernel, "VelocityRead", initialVelocity);
                shader.SetBuffer(kernel, "TentativeVelocity", tentative);
                shader.SetBuffer(kernel, "LiquidState", recovery.State);
                shader.SetBuffer(kernel, "RecoveryStatus", recovery.Status);
                var velocity = new float[vertical.Length];
                var zeroFlow = new double[vertical.Length];
                var zero = new double[plan.GroupCount];
                var status = new uint[plan.GroupCount];

                float Check(float ambientK, float columnK, int direction)
                {
                    var energy = plan.Volumes.Select(v => enthalpy.EnergyJ(v, ambientK)).ToArray();
                    for (int k = 0; k < grid.Cells.Z; k++)
                        for (int j = 0; j < grid.Cells.Y; j++)
                        {
                            int group = plan.PressureGroup(1, j, k);
                            energy[group] = enthalpy.EnergyJ(plan.Volumes[group], columnK);
                        }

                    transport.AdvanceWithTemperature(new CutCellInventories(plan.Volumes, zero, energy, zero, zero),
                        zeroFlow, dt, recovery);
                    recovery.Status.GetData(status);
                    if (status.Any(value => value != 0)) throw new InvalidOperationException("S09 cavity contains invalid liquid state.");
                    float rho = table.EvaluateFloat(ambientK, 101325)[0];
                    shader.SetFloat("ReferenceDensity", rho);
                    shader.Dispatch(kernel, (plan.FaceCount + 63) / 64, 1, 1);
                    tentative.GetData(velocity);
                    var projected = FlowGpuSolver.Project(plan, pressure, velocity, dt, 4096);
                    float actual = projected.VelocityMPerS[plan.FaceIndex(1, 1, 2, 0)];
                    if (!projected.Diagnostics.Converged || float.IsNaN(actual) ||
                        (direction == 0 ? Math.Abs(actual) > 1e-8 : direction * actual <= 1e-7))
                        throw new InvalidOperationException($"S09 unexpected projected velocity {actual:G6} m/s.");
                    return actual;
                }

                float uniform = Check(277.125f, 277.125f, 0);
                float sinking = Check(274f, 277.125f, -1);
                float rising = Check(277.125f, 283f, 1);
                Debug.Log($"S09 buoyancy check passed: uniform {uniform:G5} m/s, warmed below 4 °C {sinking:G5} m/s, warmed above 4 °C {rising:G5} m/s.");
            }
        }
    }
}
