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
    public static class LiquidEnthalpyGpuCheck
    {
        public static void Run()
        {
            const double density = 997;
            var table = new WaterPropertyTable(WaterPropertyTable.Generate(WaterBranch.Liquid));
            var reference = new FluidEnthalpy(table, density);
            var grid = new Grid3D(4, 1, 1, new D3(.01, .01, .01));
            var solid = SurfacePrimitives.Box(new D3(-.01, -.01, -.01), new D3(.05, .004, .02));
            var open = new FlowBoundary(FlowBoundaryKind.OpenPressure);
            var wall = new FlowBoundary(FlowBoundaryKind.NoSlip);
            var plan = new VariableCoefficientProjection(CutCellGeometry.Build(grid, solid),
                new BoundaryConditions3D(open, open, wall, wall, wall, wall), _ => density, _ => 0);
            var liquid = plan.Volumes.Select((v, i) => v * (.2 + .15 * i)).ToArray();
            var gas = plan.Volumes.Zip(liquid, (v, l) => v - l).ToArray();
            var energy = liquid.Select((v, i) => reference.EnergyJ(v, i == 0 ? 323.15 : 293.15)).ToArray();
            var before = new CutCellInventories(liquid, gas, energy, new double[gas.Length], new double[gas.Length]);
            var velocity = new double[plan.FaceCount];
            for (int i = 0; i <= 4; i++)
            {
                int face = plan.FaceIndex(0, i, 0, 0);
                velocity[face] = 6e-8 / plan.Faces[face].Area;
            }

            using (var transport = new CutCellGpuScalars(plan,
                Resources.Load<ComputeShader>("ThreePhase3D/CutCellVofScalars"),
                Resources.Load<ComputeShader>("ThreePhase3D/CutCellVofClipping")))
            using (var recovery = new LiquidEnthalpyGpuRecovery(
                Resources.Load<ComputeShader>("ThreePhase3D/LiquidEnthalpyRecovery"), table, plan.GroupCount, density))
            {
                var after = transport.AdvanceWithTemperature(before, velocity, 1, recovery,
                    _ => new CutCellInflow(0, 0, 0, 0));
                var state = new Vector2[plan.GroupCount];
                var status = new uint[state.Length];
                recovery.State.GetData(state);
                recovery.Status.GetData(status);
                double error = 0;
                for (int i = 0; i < state.Length; i++)
                {
                    double? expected = reference.TemperatureK(after.Fields.LiquidM3[i], after.Fields.LiquidEnergyJ[i]);
                    if (status[i] != 0 || !expected.HasValue || float.IsNaN(state[i].x) || float.IsInfinity(state[i].x))
                        throw new InvalidOperationException($"S09 GPU recovery rejected wet group {i}.");
                    error = Math.Max(error, Math.Abs(state[i].x - expected.Value));
                }

                double residual = after.Fields.LiquidEnergyJ.Sum() - energy.Sum() - after.BoundaryInflow[2];
                if (error > .0002 || Math.Abs(residual) > 1e-8 || state[1].x <= 293.15 || state[1].x >= 323.15)
                    throw new InvalidOperationException($"S09 mismatch: {error:G6} K, ledger {residual:G6} J.");
                Debug.Log($"S09 GPU check passed: downstream {state[1].x:F3} K, maximum error {error:G6} K, ledger {residual:G6} J.");
            }
        }
    }
}
