using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Geometry;

namespace PhysicsSim.ThreePhase3D.Flow
{
    // S05 transport owns momentum advection. Reuse only the S04 pressure
    // projection so the accepted velocity is not advected twice.
    public sealed class PeriodicVofProjection
    {
        public delegate FlowStep PressureSolve(VariableCoefficientProjection plan, float[] velocity, double dt, int maxIterations);
        private readonly GeometricVof3D transport;
        private readonly CutCellGeometry geometry;
        private readonly double liquidDensity, gasDensity;

        public PeriodicVofProjection(Grid3D grid, double liquidDensity, double gasDensity)
        {
            transport = new GeometricVof3D(grid, liquidDensity, gasDensity);
            geometry = CutCellGeometry.Build(grid);
            this.liquidDensity = liquidDensity;
            this.gasDensity = gasDensity;
        }

        public PeriodicVofStep Advance(VofFields state, double dt, int maxIterations = 4096) =>
            Advance(state, dt, maxIterations, transport.Advance, FlowCpuReference.Project);

        internal PeriodicVofStep Advance(VofFields state, double dt, int maxIterations,
            Func<VofFields, double[][], double, VofAdvance> advect, PressureSolve project)
        {
            dt = (float)dt;
            if (state == null || !SiUnits.IsFinite(dt) || dt <= 0 || maxIterations < 1)
                throw new ArgumentException("Invalid periodic flow step.");
            var moved = advect(state, state.FaceVelocity, dt).Fields;
            var grid = geometry.Grid;
            var shape = grid.Cells;
            var plan = new VariableCoefficientProjection(geometry, BoundaryConditions3D.Closed, padded =>
            {
                shape.Coordinates(padded, out int i, out int j, out int k);
                double fraction = moved.LiquidFraction[transport.CellIndex(i, j, k)];
                return liquidDensity * fraction + gasDensity * (1 - fraction);
            }, _ => 0, true);
            var tentative = new float[plan.FaceCount];
            ForEachFace((a, c, i, j, k) => tentative[plan.FaceIndex(a, i, j, k)] = (float)moved.FaceVelocity[a][c]);
            var pressure = project(plan, tentative, dt, maxIterations);
            var velocity = new[] { new double[transport.CellCount], new double[transport.CellCount], new double[transport.CellCount] };
            ForEachFace((a, c, i, j, k) => velocity[a][c] = pressure.VelocityMPerS[plan.FaceIndex(a, i, j, k)]);
            double correction = ReconcileDivergence(velocity, dt);
            return new PeriodicVofStep(new VofFields(moved.LiquidFraction, velocity, moved.LiquidEnergyJ,
                moved.GasEnergyJ, moved.VapourKg), pressure.Diagnostics, correction);
        }

        private void ForEachFace(Action<int, int, int, int, int> visit)
        {
            var shape = geometry.Grid.Cells;
            for (int a = 0; a < 3; a++)
            for (int k = 0; k < shape.Z; k++)
            for (int j = 0; j < shape.Y; j++)
            for (int i = 0; i < shape.X; i++)
                visit(a, transport.CellIndex(i, j, k), i + (a == 0 ? 1 : 0), j + (a == 1 ? 1 : 0), k + (a == 2 ? 1 : 0));
        }

        private double ReconcileDivergence(double[][] velocity, double dt)
        {
            // A shared tree-face correction removes the FP32 divergence
            // remainder in FP64, with no cell volume or mass edits. Visit each
            // child before its parent. Bound the largest correction in CFL units.
            var shape = geometry.Grid.Cells;
            var h = geometry.Grid.Spacing;
            double maximum = 0;
            for (int k = shape.Z - 1; k >= 0; k--)
            for (int j = shape.Y - 1; j >= 0; j--)
            for (int i = shape.X - 1; i >= 0; i--)
            {
                int cell = transport.CellIndex(i, j, k);
                if (cell == 0) continue;
                int x = transport.CellIndex((i + shape.X - 1) % shape.X, j, k);
                int y = transport.CellIndex(i, (j + shape.Y - 1) % shape.Y, k);
                int z = transport.CellIndex(i, j, (k + shape.Z - 1) % shape.Z);
                double remainder = (velocity[0][cell] - velocity[0][x]) / h.X +
                    (velocity[1][cell] - velocity[1][y]) / h.Y +
                    (velocity[2][cell] - velocity[2][z]) / h.Z;
                int axis = i > 0 ? 0 : j > 0 ? 1 : 2;
                int parent = axis == 0 ? x : axis == 1 ? y : z;
                maximum = Math.Max(maximum, Math.Abs(remainder * dt));
                velocity[axis][parent] += remainder * h[axis];
            }
            if (maximum > 1e-6)
                throw new InvalidOperationException("Pressure divergence reconciliation exceeds 1e-6 Courant units.");
            return maximum;
        }
    }

    public sealed class PeriodicVofStep
    {
        public readonly VofFields Fields;
        public readonly FlowDiagnostics Pressure;
        public readonly double MaxFluxCorrectionCourant;
        internal PeriodicVofStep(VofFields fields, FlowDiagnostics pressure, double correction)
        {
            Fields = fields;
            Pressure = pressure;
            MaxFluxCorrectionCourant = correction;
        }
    }
}
