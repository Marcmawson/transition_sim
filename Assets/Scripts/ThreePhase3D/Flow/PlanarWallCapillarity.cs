using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Geometry;
using PhysicsSim.ThreePhase3D.PhaseChange;

namespace PhysicsSim.ThreePhase3D.Flow
{
    // Grid-aligned lower wall with a prescribed static contact angle.
    internal sealed class PlanarWallCapillarity
    {
        private readonly GeometricVof3D reflected;
        internal int FirstFluidY { get; }
        private readonly double sigma, timestepLimit, contactCotangent;

        internal PlanarWallCapillarity(VariableCoefficientProjection plan, int firstFluidY, double sigma, double contactAngleDegrees)
        {
            var grid = plan.Grid;
            var shape = grid.Cells;
            if (firstFluidY < 1 || firstFluidY >= shape.Y || !SiUnits.IsFinite(sigma) || sigma <= 0)
                throw new ArgumentException("A valid lower wall row and positive surface tension are required.");
            for (int axis = 0; axis < 3; axis++)
            for (int side = 0; side < 2; side++)
                if (plan.Boundaries.Side(axis, side != 0).Kind == FlowBoundaryKind.OpenPressure)
                    throw new NotSupportedException("The reflected capillary domain requires closed boundaries.");

            double wallY = firstFluidY * grid.Spacing.Y;
            if (plan.Geometry.Patches.Count == 0)
                throw new NotSupportedException("A planar embedded wall is required.");
            foreach (var patch in plan.Geometry.Patches)
                if (patch.Role != SolidRole.Fixed || Math.Abs(patch.NormalLocal.Y - 1) > 1e-10 ||
                    Math.Abs(patch.CentroidLocalM.Y - wallY) > grid.Spacing.Y * 1e-10)
                    throw new NotSupportedException("Capillary reflection requires one grid-aligned lower wall.");

            for (int k = 0; k < shape.Z; k++)
            for (int j = 0; j < shape.Y; j++)
            for (int i = 0; i < shape.X; i++)
            {
                int cell = shape.Index(i, j, k), group = plan.GroupByCell[cell];
                double expected = j < firstFluidY ? 0 : grid.CellVolume;
                if (Math.Abs(plan.Geometry.FluidVolumeM3[cell] - expected) > 1e-10 * grid.CellVolume ||
                    (j < firstFluidY ? group >= 0 : group < 0 || plan.Volumes[group] > 1.000000001 * grid.CellVolume))
                    throw new NotSupportedException("Capillary reflection requires full cells above the wall and solid cells below.");
            }
            if (!SiUnits.IsFinite(contactAngleDegrees) || contactAngleDegrees <= 0 || contactAngleDegrees >= 180)
                throw new ArgumentOutOfRangeException(nameof(contactAngleDegrees));
            double radians = contactAngleDegrees * Math.PI / 180;
            contactCotangent = contactAngleDegrees == 90 ? 0 : Math.Cos(radians) / Math.Sin(radians);
            FirstFluidY = firstFluidY;
            this.sigma = sigma;
            var h = grid.Spacing;
            double minimum = Math.Min(h.X, Math.Min(h.Y, h.Z));
            timestepLimit = minimum * minimum * minimum / (4 * Math.PI * sigma);
            reflected = new GeometricVof3D(new Grid3D(shape.X, checked(2 * (shape.Y - firstFluidY)), shape.Z, h), 1, 1);
        }

        internal void Apply(VariableCoefficientProjection plan, CutCellInventories fields,
            int[] lowCell, int[] highCell, double rhoL, double rhoG, double dt, float[] tentative,
            System.Collections.Generic.IReadOnlyDictionary<int, double> contactAngles = null)
        {
            if (dt > Math.Sqrt((rhoL + rhoG) * timestepLimit))
                throw new ArgumentOutOfRangeException(nameof(dt), "Step exceeds the explicit capillary timestep limit.");
            var shape = plan.Grid.Cells;
            int half = shape.Y - FirstFluidY;
            var alpha = new double[reflected.CellCount];
            for (int k = 0; k < shape.Z; k++)
            for (int j = FirstFluidY; j < shape.Y; j++)
            for (int i = 0; i < shape.X; i++)
            {
                int group = plan.GroupByCell[shape.Index(i, j, k)];
                double fraction = fields.LiquidM3[group] / plan.Volumes[group];
                if (fraction > 1e-12 && (i < 3 || k < 3 || i >= shape.X - 3 || k >= shape.Z - 3 || j >= shape.Y - 3))
                    throw new NotSupportedException("Keep three gas cells between the interface and the outer reflection boundaries.");
                int y = half + j - FirstFluidY;
                alpha[reflected.CellIndex(i, y, k)] = fraction;
                alpha[reflected.CellIndex(i, 2 * half - 1 - y, k)] = fraction;
            }
            bool angledWall = contactCotangent != 0 || contactAngles?.Count > 0;
            Func<int, int, double> wallCotangent = contactAngles?.Count > 0 ? CotangentAt : null;
            var curvature = angledWall
                ? SurfaceGeometry.Evaluate(reflected, alpha, half, contactCotangent, wallCotangent)
                : SurfaceGeometry.Evaluate(reflected, alpha);

            double CotangentAt(int i, int k)
            {
                if (!contactAngles.TryGetValue(shape.Index(i, FirstFluidY, k), out double angle))
                    return contactCotangent;
                double radians = angle * Math.PI / 180;
                return angle == 90 ? 0 : Math.Cos(radians) / Math.Sin(radians);
            }
            for (int f = 0; f < tentative.Length; f++)
            {
                var face = plan.Faces[f];
                if (face.Active == 0) continue;
                int low = MirrorIndex(lowCell[f]), high = MirrorIndex(highCell[f]);
                double jump = alpha[high] - alpha[low];
                if (Math.Abs(jump) <= 1e-12) continue;
                double a = curvature[low].InverseMetres, b = curvature[high].InverseMetres;
                double value = SiUnits.IsFinite(a) ? (SiUnits.IsFinite(b) ? .5 * (a + b) : a) : b;
                if (!SiUnits.IsFinite(value)) value = Nearby(low, high);
                tentative[f] = (float)(tentative[f] + dt * face.Mobility * sigma * value * jump);
                if (!SiUnits.IsFinite(tentative[f]))
                    throw new InvalidOperationException("Capillary prediction exceeds FP32 range.");
            }

            int MirrorIndex(int cell)
            {
                shape.Coordinates(cell, out int i, out int j, out int k);
                return reflected.CellIndex(i, half + j - FirstFluidY, k);
            }

            double Nearby(int low, int high)
            {
                int nx = shape.X, ny = 2 * half;
                int maxReach = angledWall ? 3 : 1;
                for (int reach = 1; reach <= maxReach; reach++)
                {
                    double sum = 0;
                    int count = 0;
                    for (int side = 0; side < 2; side++)
                    {
                        int cell = side == 0 ? low : high;
                        int x = cell % nx, y = cell / nx % ny, z = cell / nx / ny;
                        for (int dz = -reach; dz <= reach; dz++)
                        for (int dy = -reach; dy <= reach; dy++)
                        for (int dx = -reach; dx <= reach; dx++)
                        {
                            if ((angledWall && y + dy < half) ||
                                !reflected.Grid.Cells.Interior(x + dx, y + dy, z + dz)) continue;
                            double value = curvature[reflected.CellIndex(x + dx, y + dy, z + dz)].InverseMetres;
                            if (!SiUnits.IsFinite(value)) continue;
                            sum += value;
                            count++;
                        }
                    }
                    if (count != 0) return sum / count;
                }
                throw new InvalidOperationException($"No supported curvature near reflected cells {low} and {high}.");
            }
        }
    }
}
