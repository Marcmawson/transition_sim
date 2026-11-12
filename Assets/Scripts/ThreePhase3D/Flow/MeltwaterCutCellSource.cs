using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Geometry;
using PhysicsSim.ThreePhase3D.Solids;

namespace PhysicsSim.ThreePhase3D.Flow
{
    public static class MeltwaterCutCellSource
    {
        public readonly struct Cell
        {
            public readonly double LiquidM3, EnthalpyJ, ContractionM3;
            public readonly D3 MomentumKgMPerS;

            internal Cell(double liquid, double enthalpy, double contraction, D3 momentum)
            {
                LiquidM3 = liquid;
                EnthalpyJ = enthalpy;
                ContractionM3 = contraction;
                MomentumKgMPerS = momentum;
            }
        }

        // Geometry changes by face recession at one pose. The contraction
        // must be filled by the flow update when it reconciles the new capacity.
        public static Cell[] Build(CutCellGeometry before, CutCellGeometry after, IceBody released,
            double releasedIceM3, double liquidM3, double liquidEnthalpyJ)
        {
            if (before == null) throw new ArgumentNullException(nameof(before));
            if (after == null) throw new ArgumentNullException(nameof(after));
            if (released == null) throw new ArgumentNullException(nameof(released));
            if (before.Grid != after.Grid) throw new ArgumentException("Both geometries must use the same grid.");
            if (!SiUnits.IsFinite(releasedIceM3) || releasedIceM3 <= 0 ||
                !SiUnits.IsFinite(liquidM3) || liquidM3 <= 0 || liquidM3 > releasedIceM3 ||
                !SiUnits.IsFinite(liquidEnthalpyJ))
                throw new ArgumentException("The meltwater source requires finite melting volumes and enthalpy.");

            var grid = before.Grid;
            var oldMoments = before.WithFluidCentroidMoments();
            var newMoments = after.WithFluidCentroidMoments();
            var sources = new Cell[grid.Cells.Count];
            double totalVolume = 0;
            D3 totalMoment = default;
            double tolerance = grid.CellVolume * 1e-12;
            for (int k = 0; k < grid.Cells.Z; k++)
            for (int j = 0; j < grid.Cells.Y; j++)
            for (int i = 0; i < grid.Cells.X; i++)
            {
                int cell = grid.Cells.Index(i, j, k);
                double oldVolume = before.FluidVolumeM3[cell], newVolume = after.FluidVolumeM3[cell];
                double vacated = newVolume - oldVolume;
                if (vacated < -tolerance || Math.Abs(before.FixedVolumeM3[cell] - after.FixedVolumeM3[cell]) > tolerance)
                    throw new ArgumentException("Meltwater deposition requires nested ice recession with fixed vessel geometry.");
                if (vacated <= tolerance) continue;
                D3 offsetMoment = FluidMoment(newMoments, cell, newVolume) - FluidMoment(oldMoments, cell, oldVolume);
                D3 centre = grid.Frame.ToWorld(grid.Lower(i, j, k) + grid.Spacing * 0.5 + offsetMoment / vacated);
                double fraction = vacated / releasedIceM3;
                double deposited = liquidM3 * fraction;
                sources[cell] = new Cell(deposited, liquidEnthalpyJ * fraction, vacated - deposited,
                    released.VelocityAt(centre) * (released.MassKg * fraction));
                totalVolume += vacated;
                totalMoment += (centre - released.Frame.Origin) * vacated;
            }
            if (Math.Abs(totalVolume - releasedIceM3) > releasedIceM3 * 1e-9 ||
                totalMoment.Length > releasedIceM3 * grid.Spacing.Length * 1e-9)
                throw new InvalidOperationException("Receding grid geometry does not account for the released ice volume and centroid.");
            return sources;

            D3 FluidMoment(CutCellGeometry geometry, int cell, double volume) =>
                volume > 0 ? geometry.FluidCentroidOffset(cell) * volume : default;
        }
    }
}
