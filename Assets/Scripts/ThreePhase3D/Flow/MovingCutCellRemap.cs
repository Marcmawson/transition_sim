using System;
using System.Collections.Generic;
using PhysicsSim.ThreePhase3D.Core;

namespace PhysicsSim.ThreePhase3D.Flow
{
    // First-order scalar remap across two ice poses on the same MAC grid.
    public static class MovingCutCellRemap
    {
        // Reconcile a closed-domain projected flux with the change in cell
        // capacity. Only faces open at both poses carry correction.
        public static double[][] Reconcile(VariableCoefficientProjection oldPlan,
            VariableCoefficientProjection newPlan, FlowStep flow, double maxCorrectionM3)
        {
            if (oldPlan == null || newPlan == null || flow == null) throw new ArgumentNullException();
            var grid = oldPlan.Grid;
            if (grid != newPlan.Grid || flow.VelocityMPerS.Length != oldPlan.FaceCount ||
                !flow.Diagnostics.Converged || !SiUnits.IsFinite(flow.Diagnostics.Dt) || flow.Diagnostics.Dt <= 0)
                throw new ArgumentException("Reconciliation requires two poses on one grid and a converged old-pose projection.");
            if (!SiUnits.IsFinite(maxCorrectionM3) || maxCorrectionM3 < 0)
                throw new ArgumentOutOfRangeException(nameof(maxCorrectionM3));

            int count = grid.Cells.Count;
            var rhs = new double[count];
            for (int cell = 0; cell < count; cell++)
            {
                grid.Cells.Coordinates(cell, out int i, out int j, out int k);
                if (grid.Cells.Interior(i, j, k))
                    rhs[cell] = oldPlan.Geometry.FluidVolumeM3[cell] - newPlan.Geometry.FluidVolumeM3[cell];
            }

            var faces = new List<(int Axis, int Index, int Low, int High, double Weight)>();
            var swept = new double[3][];
            for (int axis = 0; axis < 3; axis++)
            {
                var shape = grid.Shape((GridLocation)(axis + 1));
                var oldArea = oldPlan.Geometry.Areas(axis);
                var newArea = newPlan.Geometry.Areas(axis);
                swept[axis] = new double[shape.Count];
                for (int face = 0; face < shape.Count; face++)
                {
                    int packed = oldPlan.FaceOffsets[axis] + face;
                    double velocity = flow.VelocityMPerS[packed];
                    if (!SiUnits.IsFinite(velocity)) throw new ArgumentException("Projected velocity must be finite.", nameof(flow));
                    shape.Coordinates(face, out int i, out int j, out int k);
                    int x = i - (axis == 0 ? 1 : 0), y = j - (axis == 1 ? 1 : 0), z = k - (axis == 2 ? 1 : 0);
                    if (!grid.Cells.Interior(i, j, k) || !grid.Cells.Interior(x, y, z))
                    {
                        if (velocity != 0) throw new NotSupportedException("Moving-cell reconciliation requires a closed domain.");
                        continue;
                    }
                    int low = grid.Cells.Index(x, y, z), high = grid.Cells.Index(i, j, k);
                    double weight = Math.Min(oldArea[face], newArea[face]) / grid.FaceArea(axis);
                    if (weight <= 1e-12 || oldPlan.GroupByCell[low] < 0 || oldPlan.GroupByCell[high] < 0 ||
                        newPlan.GroupByCell[low] < 0 || newPlan.GroupByCell[high] < 0)
                    {
                        if (velocity != 0) throw new NotSupportedException("Projected flux crosses a face blocked at an endpoint pose.");
                        continue;
                    }
                    double transfer = velocity * oldArea[face] * flow.Diagnostics.Dt;
                    swept[axis][face] = transfer;
                    rhs[low] -= transfer;
                    rhs[high] += transfer;
                    faces.Add((axis, face, low, high, weight));
                }
            }

            // Weighted graph Laplacian. Its gradient yields a local face
            // correction; a component with incompatible volume cannot close.
            var potential = new double[count];
            var direction = (double[])rhs.Clone();
            var product = new double[count];
            double residualSquared = Dot(rhs, rhs);
            double tolerance = grid.CellVolume * 1e-11;
            int maxIterations = Math.Min(count, 4096);
            for (int iteration = 0; residualSquared > tolerance * tolerance && iteration < maxIterations; iteration++)
            {
                Array.Clear(product, 0, count);
                foreach (var face in faces)
                {
                    double value = face.Weight * (direction[face.Low] - direction[face.High]);
                    product[face.Low] += value;
                    product[face.High] -= value;
                }
                double curvature = Dot(direction, product);
                if (!(curvature > 0)) break;
                double step = residualSquared / curvature;
                for (int cell = 0; cell < count; cell++)
                {
                    potential[cell] += step * direction[cell];
                    rhs[cell] -= step * product[cell];
                }
                double next = Dot(rhs, rhs);
                double beta = next / residualSquared;
                for (int cell = 0; cell < count; cell++) direction[cell] = rhs[cell] + beta * direction[cell];
                residualSquared = next;
            }
            if (!SiUnits.IsFinite(residualSquared) || residualSquared > tolerance * tolerance)
                throw new InvalidOperationException("Swept volume reconciliation did not converge.");

            foreach (var face in faces)
            {
                double correction = face.Weight * (potential[face.Low] - potential[face.High]);
                if (!SiUnits.IsFinite(correction) || Math.Abs(correction) > maxCorrectionM3)
                    throw new InvalidOperationException("Swept volume reconciliation exceeds the face correction budget.");
                swept[face.Axis][face.Index] += correction;
            }
            return swept;
        }

        private static double Dot(double[] a, double[] b)
        {
            double result = 0;
            for (int i = 0; i < a.Length; i++) result += a[i] * b[i];
            return result;
        }

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
