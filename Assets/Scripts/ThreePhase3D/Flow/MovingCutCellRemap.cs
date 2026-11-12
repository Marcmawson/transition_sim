using System;
using PhysicsSim.ThreePhase3D.Core;

namespace PhysicsSim.ThreePhase3D.Flow
{
    // First-order scalar remap across two ice poses on the same MAC grid.
    public static class MovingCutCellRemap
    {
        // Signed swept fluid volumes are in geometric X/Y/Z face order.
        // Positive transfer is from the low cell to the high cell.
        public static CutCellInventories Apply(VariableCoefficientProjection oldPlan,
            VariableCoefficientProjection newPlan, CutCellInventories state, double[][] sweptM3)
        {
            if (oldPlan == null || newPlan == null || state == null)
                throw new ArgumentNullException("Both projection plans and the old inventory are required.");
            var grid = oldPlan.Grid;
            if (grid != newPlan.Grid)
                throw new ArgumentException("Both poses must use the same grid instance.", nameof(newPlan));
            if (sweptM3 == null || sweptM3.Length != 3)
                throw new ArgumentException("Three geometric face volume arrays are required.", nameof(sweptM3));

            double tolerance = grid.CellVolume * 1e-10;
            int count = grid.Cells.Count;
            var outgoing = new double[count];
            var incoming = new double[count];
            var result = new double[state.Planes.Length][];
            for (int field = 0; field < result.Length; field++)
            {
                var source = state.Planes[field];
                if (source == null || source.Length != oldPlan.GroupCount ||
                    !Array.TrueForAll(source, value => SiUnits.IsFinite(value)))
                    throw new ArgumentException("Old inventory does not match the pressure groups.", nameof(state));
                result[field] = new double[newPlan.GroupCount];
            }

            for (int group = 0; group < oldPlan.GroupCount; group++)
                if (state.LiquidM3[group] < 0 || state.GasM3[group] < 0 || state.VapourKg[group] < 0 ||
                    Math.Abs(state.LiquidM3[group] + state.GasM3[group] - oldPlan.Volumes[group]) > tolerance)
                    throw new ArgumentException("The old phase volumes must fill their fluid groups.", nameof(state));

            for (int axis = 0; axis < 3; axis++)
            {
                var shape = grid.Shape((GridLocation)(axis + 1));
                var swept = sweptM3[axis];
                if (swept == null || swept.Length != shape.Count)
                    throw new ArgumentException("Swept volume array does not match the geometric faces.", nameof(sweptM3));
                var oldAreas = oldPlan.Geometry.Areas(axis);
                var newAreas = newPlan.Geometry.Areas(axis);
                for (int face = 0; face < swept.Length; face++)
                {
                    double signed = swept[face];
                    if (signed == 0) continue;
                    shape.Coordinates(face, out int i, out int j, out int k);
                    int x = i - (axis == 0 ? 1 : 0), y = j - (axis == 1 ? 1 : 0), z = k - (axis == 2 ? 1 : 0);
                    if (!SiUnits.IsFinite(signed) || !grid.Cells.Interior(i, j, k) || !grid.Cells.Interior(x, y, z) ||
                        Math.Max(oldAreas[face], newAreas[face]) <= 0)
                        throw new ArgumentException("Transfer requires a finite volume and an accessible interior face.", nameof(sweptM3));
                    int low = grid.Cells.Index(x, y, z), high = grid.Cells.Index(i, j, k);
                    int donor = signed > 0 ? low : high, receiver = signed > 0 ? high : low;
                    double volume = Math.Abs(signed);
                    outgoing[donor] += volume;
                    incoming[receiver] += volume;
                    Add(oldPlan.GroupByCell[donor], newPlan.GroupByCell[receiver], volume);
                }
            }

            for (int cell = 0; cell < count; cell++)
            {
                grid.Cells.Coordinates(cell, out int i, out int j, out int k);
                if (!grid.Cells.Interior(i, j, k)) continue;
                double oldVolume = oldPlan.Geometry.FluidVolumeM3[cell];
                double newVolume = newPlan.Geometry.FluidVolumeM3[cell];
                double retained = oldVolume - outgoing[cell];
                if (retained < 0 || Math.Abs(retained + incoming[cell] - newVolume) > tolerance)
                    throw new InvalidOperationException("Swept face volumes do not close the moving-cell volume balance.");
                int receiver = newPlan.GroupByCell[cell];
                if (retained > 0)
                    Add(oldPlan.GroupByCell[cell], receiver, retained);
            }

            return new CutCellInventories(result);

            void Add(int donor, int receiver, double volume)
            {
                if (donor < 0 || receiver < 0)
                    throw new InvalidOperationException("A remap transfer has no fluid donor or receiver.");
                double fraction = volume / oldPlan.Volumes[donor];
                for (int field = 0; field < result.Length; field++)
                    result[field][receiver] += fraction * state.Planes[field][donor];
            }
        }
    }
}
