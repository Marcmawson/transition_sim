using System;
using System.Collections.Generic;
using PhysicsSim.ThreePhase3D.Core;

namespace PhysicsSim.ThreePhase3D.Geometry
{
    // Integrates a PLIC half-space and a swept slab against the original
    // watertight solids. Signed tetrahedra preserve hollow volumes.
    internal sealed class CutCellVofGeometry
    {
        private readonly Grid3D grid;
        private readonly List<SignedTetrahedron>[] solids;

        internal CutCellVofGeometry(CutCellGeometry geometry)
        {
            grid = geometry.Grid;
            solids = new List<SignedTetrahedron>[grid.Cells.Count];
            foreach (var surface in geometry.BuildSolidTetrahedra())
            foreach (var tetrahedron in surface)
            {
                int[] lo = new int[3], hi = new int[3];
                int[] extents = { grid.Cells.X, grid.Cells.Y, grid.Cells.Z };
                for (int axis = 0; axis < 3; axis++)
                {
                    lo[axis] = Math.Max(0, (int)Math.Floor(tetrahedron.Min[axis] / grid.Spacing[axis]));
                    hi[axis] = Math.Min(extents[axis] - 1, (int)Math.Floor(tetrahedron.Max[axis] / grid.Spacing[axis]));
                }
                for (int k = lo[2]; k <= hi[2]; k++)
                for (int j = lo[1]; j <= hi[1]; j++)
                for (int i = lo[0]; i <= hi[0]; i++)
                {
                    int cell = grid.Cells.Index(i, j, k);
                    if (solids[cell] == null) solids[cell] = new List<SignedTetrahedron>();
                    solids[cell].Add(tetrahedron);
                }
            }
        }

        internal ClipPlane Reconstruct(int cell, D3 normal, double liquidVolume)
        {
            Bounds(cell, out D3 lo, out D3 hi);
            normal = normal.Length == 0 ? new D3(1, 0, 0) : normal.Unit();
            D3 centre = (lo + hi) * .5;
            double radius = (Math.Abs(normal.X) * grid.Spacing.X + Math.Abs(normal.Y) * grid.Spacing.Y +
                Math.Abs(normal.Z) * grid.Spacing.Z) * .5;
            double left = -radius, right = radius;
            for (int iteration = 0; iteration < 42; iteration++)
            {
                double middle = (left + right) * .5;
                var plane = new ClipPlane(normal, centre + normal * middle);
                if (FluidVolume(cell, lo, hi, plane) < liquidVolume) left = middle;
                else right = middle;
            }
            return new ClipPlane(normal, centre + normal * ((left + right) * .5));
        }

        internal double Sweep(int cell, int axis, bool positive, double sweptVolume, ClipPlane liquid)
        {
            Bounds(cell, out D3 lo, out D3 hi);
            double left = 0, right = grid.Spacing[axis];
            for (int iteration = 0; iteration < 42; iteration++)
            {
                double middle = (left + right) * .5;
                Slab(lo, hi, axis, positive, middle, out D3 slabLo, out D3 slabHi);
                if (FluidVolume(cell, slabLo, slabHi, null) < sweptVolume) left = middle;
                else right = middle;
            }
            Slab(lo, hi, axis, positive, (left + right) * .5, out D3 lower, out D3 upper);
            return FluidVolume(cell, lower, upper, liquid);
        }

        internal double LiquidVolume(int cell, ClipPlane plane)
        {
            Bounds(cell, out D3 lo, out D3 hi);
            return FluidVolume(cell, lo, hi, plane);
        }

        private void Bounds(int cell, out D3 lo, out D3 hi)
        {
            grid.Cells.Coordinates(cell, out int i, out int j, out int k);
            if (!grid.Cells.Interior(i, j, k)) throw new ArgumentOutOfRangeException(nameof(cell));
            lo = grid.Lower(i, j, k);
            hi = lo + grid.Spacing;
        }

        private static void Slab(D3 lo, D3 hi, int axis, bool positive, double width, out D3 lower, out D3 upper)
        {
            var delta = new D3(axis == 0 ? width : 0, axis == 1 ? width : 0, axis == 2 ? width : 0);
            // Keep both transverse bounds from the full cell.
            lower = positive ? lo + Along(hi - lo, axis) - delta : lo;
            upper = positive ? hi : hi - Along(hi - lo, axis) + delta;
        }

        private static D3 Along(D3 value, int axis) =>
            new D3(axis == 0 ? value.X : 0, axis == 1 ? value.Y : 0, axis == 2 ? value.Z : 0);

        private double FluidVolume(int cell, D3 lo, D3 hi, ClipPlane? liquid)
        {
            double boxVolume = (hi.X - lo.X) * (hi.Y - lo.Y) * (hi.Z - lo.Z);
            if (boxVolume <= 0) return 0;
            double available = liquid.HasValue ? PolyhedronClip.Volume(PolyhedronClip.Clip(Box(lo, hi), liquid.Value)) : boxVolume;
            var candidates = solids[cell];
            if (candidates == null) return available;
            ClipPlane[] bounds = PolyhedronClip.Bounds(lo, hi);
            double blocked = 0;
            foreach (var tetra in candidates)
            {
                if (tetra.Max.X <= lo.X || tetra.Min.X >= hi.X || tetra.Max.Y <= lo.Y ||
                    tetra.Min.Y >= hi.Y || tetra.Max.Z <= lo.Z || tetra.Min.Z >= hi.Z) continue;
                var faces = tetra.Faces;
                foreach (var bound in bounds)
                {
                    faces = PolyhedronClip.Clip(faces, bound);
                    if (faces.Count == 0) break;
                }
                if (liquid.HasValue && faces.Count > 0) faces = PolyhedronClip.Clip(faces, liquid.Value);
                blocked += tetra.Sign * PolyhedronClip.Volume(faces);
            }
            double fluid = available - blocked;
            double tolerance = 1e-10 * grid.CellVolume;
            if (fluid < -tolerance || fluid > available + tolerance)
                throw new InvalidOperationException("Solid clipping produced an invalid swept fluid volume.");
            return Math.Max(0, Math.Min(available, fluid));
        }

        private static List<List<D3>> Box(D3 lo, D3 hi)
        {
            var p = new[]
            {
                lo, new D3(hi.X, lo.Y, lo.Z), new D3(hi.X, hi.Y, lo.Z), new D3(lo.X, hi.Y, lo.Z),
                new D3(lo.X, lo.Y, hi.Z), new D3(hi.X, lo.Y, hi.Z), hi, new D3(lo.X, hi.Y, hi.Z)
            };
            return new List<List<D3>>
            {
                new List<D3>{p[0],p[3],p[2],p[1]}, new List<D3>{p[4],p[5],p[6],p[7]},
                new List<D3>{p[0],p[1],p[5],p[4]}, new List<D3>{p[3],p[7],p[6],p[2]},
                new List<D3>{p[0],p[4],p[7],p[3]}, new List<D3>{p[1],p[2],p[6],p[5]}
            };
        }
    }
}
