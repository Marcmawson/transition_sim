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

        internal void PackTetrahedra(int cell, Action<D3, D3, D3, D3, double> emit)
        {
            Bounds(cell, out D3 lo, out D3 hi);
            var bounds = PolyhedronClip.Bounds(lo, hi);
            Pack(Box(lo, hi), 1);
            if (solids[cell] == null) return;
            foreach (var tetra in solids[cell])
            {
                var faces = tetra.Faces;
                foreach (var bound in bounds)
                {
                    faces = PolyhedronClip.Clip(faces, bound);
                    if (faces.Count == 0) break;
                }
                Pack(faces, -tetra.Sign);
            }

            void Pack(List<List<D3>> faces, double sign)
            {
                if (faces.Count == 0) return;
                D3 origin = Unit(faces[0][0]);
                foreach (var face in faces)
                for (int i = 1; i + 1 < face.Count; i++)
                {
                    D3 a = Unit(face[0]), b = Unit(face[i]), c = Unit(face[i+1]);
                    double sixVolume = D3.Dot(a-origin, D3.Cross(b-origin,c-origin));
                    if (sixVolume == 0) continue;
                    emit(origin,a,b,c,sign * Math.Sign(sixVolume));
                }
            }

            D3 Unit(D3 p) => new D3((p.X-lo.X)/grid.Spacing.X,(p.Y-lo.Y)/grid.Spacing.Y,(p.Z-lo.Z)/grid.Spacing.Z);
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

        internal double LiquidGasInterfaceArea(int cell, ClipPlane plane)
        {
            Bounds(cell, out D3 lo, out D3 hi);
            double area = PolyhedronClip.SectionArea(Box(lo, hi), plane);
            if (area == 0 || solids[cell] == null) return area;
            var bounds = PolyhedronClip.Bounds(lo, hi);
            double blocked = 0;
            foreach (var tetra in solids[cell])
            {
                var faces = tetra.Faces;
                foreach (var bound in bounds)
                {
                    faces = PolyhedronClip.Clip(faces, bound);
                    if (faces.Count == 0) break;
                }
                if (faces.Count > 0) blocked += tetra.Sign * PolyhedronClip.SectionArea(faces, plane);
            }
            double open = area - blocked;
            double tolerance = 1e-10 * (grid.FaceArea(0) + grid.FaceArea(1) + grid.FaceArea(2));
            if (open < -tolerance || open > area + tolerance)
                throw new InvalidOperationException("Solid clipping produced an invalid liquid–gas interface area.");
            return Math.Max(0, Math.Min(area, open));
        }

        internal double GasFaceSection(int cell, int axis, bool upper, ClipPlane? gasA, ClipPlane? gasB)
        {
            Bounds(cell, out D3 lo, out D3 hi);
            int u = (axis + 1) % 3, v = (axis + 2) % 3;
            // Sample just inside the owning cell, as for the embedded face aperture.
            double coordinate = upper ? hi[axis] - grid.Spacing[axis] * 1e-9 :
                lo[axis] + grid.Spacing[axis] * 1e-9;
            D3 Point(double a, double b) => axis == 0 ? new D3(coordinate, a, b) :
                axis == 1 ? new D3(b, coordinate, a) : new D3(a, b, coordinate);
            var polygon = new List<D3> { Point(lo[u], lo[v]), Point(hi[u], lo[v]),
                Point(hi[u], hi[v]), Point(lo[u], hi[v]) };
            if (gasA.HasValue) polygon = PolyhedronClip.Polygon(polygon, gasA.Value);
            if (gasB.HasValue) polygon = PolyhedronClip.Polygon(polygon, gasB.Value);
            double area = PolyhedronClip.Area(polygon);
            if (area == 0 || solids[cell] == null) return area;
            double blocked = 0;
            foreach (var tetra in solids[cell])
            {
                var section = polygon;
                foreach (var plane in tetra.Planes)
                {
                    section = PolyhedronClip.Polygon(section, plane);
                    if (section.Count < 3) break;
                }
                blocked += tetra.Sign * PolyhedronClip.Area(section);
            }
            double open = area - blocked;
            double tolerance = 1e-10 * grid.FaceArea(axis);
            if (open < -tolerance || open > area + tolerance)
                throw new InvalidOperationException("Solid clipping produced an invalid gas aperture.");
            return Math.Max(0, Math.Min(area, open));
        }

        internal double FluidNormalDistance(int cell, D3 point, D3 outward)
        {
            Bounds(cell, out D3 lo, out D3 hi);
            var halfSpace = new ClipPlane(-outward, point);
            var bounds = PolyhedronClip.Bounds(lo, hi);
            double volume = 0, moment = 0;
            Accumulate(Box(lo, hi), 1);
            if (solids[cell] != null)
                foreach (var tetra in solids[cell])
                {
                    var faces = tetra.Faces;
                    foreach (var bound in bounds)
                    {
                        faces = PolyhedronClip.Clip(faces, bound);
                        if (faces.Count == 0) break;
                    }
                    if (faces.Count > 0) Accumulate(faces, -tetra.Sign);
                }
            return volume > 0 ? moment / volume : 0;

            void Accumulate(List<List<D3>> faces, double sign)
            {
                var clipped = PolyhedronClip.Clip(faces, halfSpace);
                double amount = PolyhedronClip.Volume(clipped);
                if (amount <= 0) return;
                volume += sign * amount;
                moment += sign * amount * D3.Dot(PolyhedronClip.VolumeCentroid(clipped) - point, outward);
            }
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
