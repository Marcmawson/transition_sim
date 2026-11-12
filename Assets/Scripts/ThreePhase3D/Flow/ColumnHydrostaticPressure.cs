using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Geometry;

namespace PhysicsSim.ThreePhase3D.Flow
{
    // Piecewise-linear hydrostatic reconstruction from the current phase
    // inventory. The top of each column is the zero gauge reference.
    public sealed class ColumnHydrostaticPressure
    {
        private readonly Grid3D grid;
        private readonly VariableCoefficientProjection plan;
        private readonly double[] waterline;
        private readonly double liquidSlope, gasSlope;

        public ColumnHydrostaticPressure(VariableCoefficientProjection plan, CutCellInventories inventory,
            D3 gravityMPerS2, double liquidDensityKgM3, double gasDensityKgM3)
        {
            if (plan == null) throw new ArgumentNullException(nameof(plan));
            if (inventory == null) throw new ArgumentNullException(nameof(inventory));
            if (!SiUnits.IsFinite(liquidDensityKgM3) || liquidDensityKgM3 <= 0 ||
                !SiUnits.IsFinite(gasDensityKgM3) || gasDensityKgM3 <= 0)
                throw new ArgumentException("Positive finite phase densities are required.");
            if (inventory.LiquidM3.Length != plan.GroupCount || inventory.GasM3.Length != plan.GroupCount)
                throw new ArgumentException("Inventory does not match the pressure groups.", nameof(inventory));

            this.plan = plan;
            grid = plan.Grid;
            double gravity = -D3.Dot(gravityMPerS2, grid.Frame.V);
            if (!(gravity > 0) || (gravityMPerS2 + grid.Frame.V * gravity).Length > 1e-12 * gravity)
                throw new ArgumentException("Column pressure requires gravity along the negative grid Y axis.", nameof(gravityMPerS2));

            liquidSlope = gravity * liquidDensityKgM3;
            gasSlope = gravity * gasDensityKgM3;
            waterline = new double[grid.Cells.X * grid.Cells.Z];
            var fractions = new double[plan.GroupCount];
            for (int group = 0; group < plan.GroupCount; group++)
            {
                double liquid = inventory.LiquidM3[group], gas = inventory.GasM3[group];
                if (!SiUnits.IsFinite(liquid) || liquid < 0 || !SiUnits.IsFinite(gas) || gas < 0 ||
                    Math.Abs(liquid + gas - plan.Volumes[group]) > 1e-10 * grid.CellVolume)
                    throw new ArgumentException("Invalid phase volumes for column pressure.", nameof(inventory));
                fractions[group] = liquid / (liquid + gas);
            }

            var clipping = new CutCellVofGeometry(plan.Geometry);
            for (int z = 0; z < grid.Cells.Z; z++)
            for (int x = 0; x < grid.Cells.X; x++)
            {
                double remaining = 0;
                for (int y = 0; y < grid.Cells.Y; y++)
                {
                    int cell = grid.Cells.Index(x, y, z);
                    int group = plan.GroupByCell[cell];
                    if (group >= 0) remaining += fractions[group] * plan.Geometry.FluidVolumeM3[cell];
                }

                double height = 0;
                for (int y = 0; y < grid.Cells.Y && remaining > 0; y++)
                {
                    int cell = grid.Cells.Index(x, y, z);
                    double capacity = plan.Geometry.FluidVolumeM3[cell];
                    if (remaining < capacity)
                    {
                        height = clipping.Reconstruct(cell, new D3(0, 1, 0), remaining).Offset;
                        break;
                    }
                    remaining -= capacity;
                    height = (y + 1) * grid.Spacing.Y;
                }
                waterline[z * grid.Cells.X + x] = height;
            }
        }


        // The analytic vertical column gradient cancels gravity on each
        // side of the waterline. Across columns, use the pressure jump
        // and the same face mobility as the dynamic pressure projection.
        public float[] Predict(float[] velocityMPerS, double dt)
        {
            if (velocityMPerS == null || velocityMPerS.Length != plan.FaceCount)
                throw new ArgumentException("Velocity does not match the staggered grid.", nameof(velocityMPerS));
            if (!SiUnits.IsFinite(dt) || dt <= 0) throw new ArgumentOutOfRangeException(nameof(dt));
            var tentative = (float[])velocityMPerS.Clone();
            for (int axis = 0; axis < 3; axis++)
            {
                var shape = grid.Shape((GridLocation)(axis + 1));
                for (int index = 0; index < shape.Count; index++)
                {
                    int faceIndex = plan.FaceOffsets[axis] + index;
                    var face = plan.Faces[faceIndex];
                    if (!SiUnits.IsFinite(tentative[faceIndex]) || (face.Active == 0 && tentative[faceIndex] != 0))
                        throw new ArgumentException("Nonfinite or blocked-face velocity.", nameof(velocityMPerS));
                    if (face.Active == 0) continue;
                    if (face.Low < 0 || face.High < 0)
                        throw new NotSupportedException("Column pressure prediction requires closed boundaries.");
                    if (axis == 1) continue;
                    shape.Coordinates(index, out int x, out int y, out int z);
                    int lowX = x - (axis == 0 ? 1 : 0), lowZ = z - (axis == 2 ? 1 : 0);
                    double pointY = (y + 0.5) * grid.Spacing.Y;
                    double jump = Pressure(x, z, pointY) - Pressure(lowX, lowZ, pointY);
                    tentative[faceIndex] = (float)(tentative[faceIndex] - dt * face.Mobility * jump);
                    if (!SiUnits.IsFinite(tentative[faceIndex]))
                        throw new InvalidOperationException("Column pressure produced nonfinite face velocity.");
                }
            }
            return tentative;
        }

        // Sample using the patch's fluid-side cell so an ice surface on
        // a grid face never selects the adjacent solid cell by rounding.
        public double At(int cell, D3 worldPointM)
        {
            grid.Cells.Coordinates(cell, out int x, out int y, out int z);
            if (!grid.Cells.Interior(x, y, z)) throw new ArgumentOutOfRangeException(nameof(cell));
            return Pressure(x, z, grid.Frame.ToLocal(worldPointM).Y);
        }

        internal double WaterlineLocalM(int x, int z) => waterline[z * grid.Cells.X + x];

        private double Pressure(int x, int z, double pointY)
        {
            double height = WaterlineLocalM(x, z);
            double waterDepth = Math.Max(0, height - pointY);
            double airDepth = grid.Cells.Y * grid.Spacing.Y - Math.Max(pointY, height);
            return liquidSlope * waterDepth + gasSlope * airDepth;
        }
    }
}
