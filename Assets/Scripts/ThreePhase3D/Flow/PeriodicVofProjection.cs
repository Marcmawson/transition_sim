using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Geometry;
using PhysicsSim.ThreePhase3D.Materials;
using PhysicsSim.ThreePhase3D.PhaseChange;

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

        public PeriodicVofStep AdvanceCapillary(VofFields state, double dt, double surfaceTensionNPerM,
            int maxIterations = 4096) =>
            Advance(state, dt, maxIterations, surfaceTensionNPerM > 0 ? AdvectCapillary : transport.Advance,
                FlowCpuReference.Project, surfaceTensionNPerM);

        internal VofAdvance AdvectCapillary(VofFields state, double[][] velocity, double dt)
        {
            var normals = new D3[transport.CellCount];
            SurfaceGeometry.Evaluate(transport, state.LiquidFraction, -1, 0, extendColumns: true, normals: normals);
            return transport.Advance(state, velocity, dt, normals);
        }

        // Prescribed interface-temperature extension in compact periodic cell order,
        // evaluated at the projection time. This entry point does not evolve temperature.
        public PeriodicVofStep AdvanceThermocapillary(VofFields state, double dt, double[] temperatureKelvin,
            int maxIterations = 4096)
        {
            if (temperatureKelvin == null || temperatureKelvin.Length != transport.CellCount)
                throw new ArgumentException("Expected one temperature per periodic cell.", nameof(temperatureKelvin));
            var sigma = new double[temperatureKelvin.Length];
            double maximum = 0;
            double minimum = double.PositiveInfinity;
            for (int cell = 0; cell < sigma.Length; cell++)
            {
                sigma[cell] = WaterTransport.SurfaceTension(temperatureKelvin[cell]);
                maximum = Math.Max(maximum, sigma[cell]);
                minimum = Math.Min(minimum, sigma[cell]);
            }
            return Advance(state, dt, maxIterations, transport.Advance, FlowCpuReference.Project,
                maximum, minimum == maximum ? null : sigma);
        }

        internal PeriodicVofStep Advance(VofFields state, double dt, int maxIterations,
            Func<VofFields, double[][], double, VofAdvance> advect, PressureSolve project, double surfaceTensionNPerM = 0, double[] sigmaByCell = null,
            Func<double[], SurfaceCurvature[]> evaluateCurvature = null)
        {
            dt = (float)dt;
            if (state == null || !SiUnits.IsFinite(dt) || dt <= 0 || maxIterations < 1)
                throw new ArgumentException("Invalid periodic flow step.");
            if (!SiUnits.IsFinite(surfaceTensionNPerM) || surfaceTensionNPerM < 0)
                throw new ArgumentOutOfRangeException(nameof(surfaceTensionNPerM));
            if (surfaceTensionNPerM > 0)
            {
                var spacing = geometry.Grid.Spacing;
                double h = Math.Min(spacing.X, Math.Min(spacing.Y, spacing.Z));
                double capillaryLimit = Math.Sqrt((liquidDensity + gasDensity) * h * h * h /
                    (4 * Math.PI * surfaceTensionNPerM));
                if (dt > capillaryLimit)
                    throw new ArgumentOutOfRangeException(nameof(dt), "Step exceeds the explicit capillary timestep limit.");
            }
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
            var curvature = surfaceTensionNPerM > 0 ? (evaluateCurvature ?? EvaluateCapillaryCurvature)(moved.LiquidFraction) : null;
            var tangentialForce = sigmaByCell == null ? null :
                MarangoniStress.ForceDensity(transport, moved.LiquidFraction, sigmaByCell);
            int extrapolatedFaces = 0;
            ForEachFace((axis, cell, i, j, k) =>
            {
                int face = plan.FaceIndex(axis, i, j, k);
                double predicted = moved.FaceVelocity[axis][cell];
                if (curvature != null)
                {
                    int high = transport.CellIndex(i % shape.X, j % shape.Y, k % shape.Z);
                    double mobility = plan.Faces[face].Mobility;
                    double jump = moved.LiquidFraction[high] - moved.LiquidFraction[cell];
                    if (Math.Abs(jump) > 1e-12)
                    {
                        double lowCurvature = curvature[cell].InverseMetres;
                        double highCurvature = curvature[high].InverseMetres;
                        double faceCurvature = SiUnits.IsFinite(lowCurvature)
                            ? (SiUnits.IsFinite(highCurvature) ? .5 * (lowCurvature + highCurvature) : lowCurvature)
                            : highCurvature;
                        if (!SiUnits.IsFinite(faceCurvature))
                        {
                            faceCurvature = NearbyCurvature(curvature, axis, i, j, k);
                            extrapolatedFaces++;
                        }
                        double faceSigma = sigmaByCell == null ? surfaceTensionNPerM :
                            .5 * (sigmaByCell[cell] + sigmaByCell[high]);
                        predicted += dt * mobility * faceSigma * faceCurvature * jump;
                    }
                    if (tangentialForce != null)
                        predicted += dt * mobility * grid.Spacing[axis] *
                            .5 * (tangentialForce[cell][axis] + tangentialForce[high][axis]);
                }
                tentative[face] = (float)predicted;
                if (!SiUnits.IsFinite(tentative[face]))
                    throw new InvalidOperationException("Predicted face velocity exceeds FP32 range.");
            });
            var pressure = project(plan, tentative, dt, maxIterations);
            var velocity = new[] { new double[transport.CellCount], new double[transport.CellCount], new double[transport.CellCount] };
            ForEachFace((a, c, i, j, k) => velocity[a][c] = pressure.VelocityMPerS[plan.FaceIndex(a, i, j, k)]);
            double correction = ReconcileDivergence(velocity, dt);
            return new PeriodicVofStep(new VofFields(moved.LiquidFraction, velocity, moved.LiquidEnergyJ,
                moved.GasEnergyJ, moved.VapourKg), pressure.Diagnostics, correction, pressure.GroupPressurePa, extrapolatedFaces);
        }

        internal SurfaceCurvature[] EvaluateCapillaryCurvature(double[] fraction) =>
            FilterCurvature(SurfaceGeometry.Evaluate(transport, fraction, -1, 0, extendColumns: true), fraction);

        private SurfaceCurvature[] FilterCurvature(SurfaceCurvature[] raw, double[] fraction)
        {
            var shape = geometry.Grid.Cells;
            var spacing = geometry.Grid.Spacing;
            var normals = new D3[raw.Length];
            for (int k = 0; k < shape.Z; k++)
            for (int j = 0; j < shape.Y; j++)
            for (int i = 0; i < shape.X; i++)
            {
                int cell = Index(i, j, k);
                if (!SiUnits.IsFinite(raw[cell].InverseMetres)) continue;
                var gradient = new D3(
                    (fraction[Index(i + 1, j, k)] - fraction[Index(i - 1, j, k)]) / spacing.X,
                    (fraction[Index(i, j + 1, k)] - fraction[Index(i, j - 1, k)]) / spacing.Y,
                    (fraction[Index(i, j, k + 1)] - fraction[Index(i, j, k - 1)]) / spacing.Z);
                if (gradient.Length > 0) normals[cell] = gradient.Unit();
            }

            return FilterPass(FilterPass(raw));

            SurfaceCurvature[] FilterPass(SurfaceCurvature[] source)
            {
                var result = (SurfaceCurvature[])source.Clone();
                for (int k = 0; k < shape.Z; k++)
                for (int j = 0; j < shape.Y; j++)
                for (int i = 0; i < shape.X; i++)
                {
                    int cell = Index(i, j, k);
                    if (!SiUnits.IsFinite(source[cell].InverseMetres)) continue;
                    double weighted = 0, weights = 0;
                    for (int z = -Math.Min(2, shape.Z / 2); z <= Math.Min(2, (shape.Z - 1) / 2); z++)
                    for (int y = -Math.Min(2, shape.Y / 2); y <= Math.Min(2, (shape.Y - 1) / 2); y++)
                    for (int x = -Math.Min(2, shape.X / 2); x <= Math.Min(2, (shape.X - 1) / 2); x++)
                    {
                        int sample = Index(i + x, j + y, k + z);
                        double curvature = source[sample].InverseMetres;
                        if (!SiUnits.IsFinite(curvature)) continue;
                        double alignment = D3.Dot(normals[cell], normals[sample]);
                        if (alignment < .5) continue;
                        double alpha = fraction[sample];
                        double weight = alignment * alpha * (1 - alpha) / (1 + x * x + y * y + z * z);
                        weighted += weight * curvature;
                        weights += weight;
                    }
                    if (weights > 0) result[cell] = new SurfaceCurvature(source[cell].Support, weighted / weights);
                }
                return result;
            }

            int Index(int i, int j, int k) => transport.CellIndex(
                (i + shape.X) % shape.X, (j + shape.Y) % shape.Y, (k + shape.Z) % shape.Z);
        }

        private double NearbyCurvature(SurfaceCurvature[] curvature, int axis, int i, int j, int k)
        {
            // Use the first supported neighbourhood, with a bounded reach.
            var shape = geometry.Grid.Cells;
            for (int reach = 1; reach <= 3; reach++)
            {
                int count = 0;
                double sum = 0;
                for (int dz = -reach - (axis == 2 ? 1 : 0); dz <= reach; dz++)
                for (int dy = -reach - (axis == 1 ? 1 : 0); dy <= reach; dy++)
                for (int dx = -reach - (axis == 0 ? 1 : 0); dx <= reach; dx++)
                {
                    int x = ((i + dx) % shape.X + shape.X) % shape.X;
                    int y = ((j + dy) % shape.Y + shape.Y) % shape.Y;
                    int z = ((k + dz) % shape.Z + shape.Z) % shape.Z;
                    double value = curvature[transport.CellIndex(x, y, z)].InverseMetres;
                    if (!SiUnits.IsFinite(value)) continue;
                    sum += value;
                    count++;
                }
                if (count > 0) return sum / count;
            }
            throw new InvalidOperationException($"No supported capillary curvature near axis {axis} face ({i}, {j}, {k}).");
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
        // Regular periodic projection has one pressure group per cell, in compact VOF order.
        public readonly double[] CellPressurePa;
        public readonly int ExtrapolatedCurvatureFaces;
        internal PeriodicVofStep(VofFields fields, FlowDiagnostics pressure, double correction, double[] cellPressurePa,
            int extrapolatedCurvatureFaces)
        {
            Fields = fields;
            Pressure = pressure;
            MaxFluxCorrectionCourant = correction;
            CellPressurePa = cellPressurePa;
            ExtrapolatedCurvatureFaces = extrapolatedCurvatureFaces;
        }
    }
}
