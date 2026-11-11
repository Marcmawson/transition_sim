using System.Collections.Generic;
using PhysicsSim.ThreePhase3D.Flow;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Rendering
{
    internal sealed class BeakerWaterSurface
    {
        private const int Subdivisions = 4;
        private const float SurfaceFraction = .18f;
        private static readonly int[,] Tetrahedra =
        {
            { 0, 1, 3, 7 }, { 0, 3, 2, 7 }, { 0, 2, 6, 7 },
            { 0, 6, 4, 7 }, { 0, 4, 5, 7 }, { 0, 5, 1, 7 }
        };
        private readonly List<Vector3> vertices = new List<Vector3>();
        private readonly List<Vector3> normals = new List<Vector3>();
        private readonly List<int> triangles = new List<int>();
        private readonly int[] corners = new int[8], inside = new int[4], outside = new int[4];
        private readonly Vector3[] positions, gradients;
        private readonly float[] samples, fractions, weights;
        private readonly int nx, ny, nz;
        private readonly Vector3 spacing;

        public BeakerWaterSurface(BeakerVofTimeIntegration release)
        {
            var grid = release.Geometry.Grid;
            var cells = grid.Cells;
            nx = (cells.X + 1) * Subdivisions + 1;
            ny = (cells.Y + 1) * Subdivisions + 1;
            nz = (cells.Z + 1) * Subdivisions + 1;
            spacing = new Vector3((float)grid.Spacing.X, (float)grid.Spacing.Y, (float)grid.Spacing.Z) / Subdivisions;
            var origin = grid.Centre(0, 0, 0);
            var first = new Vector3((float)origin.X, (float)origin.Y, (float)origin.Z) - spacing * Subdivisions;
            positions = new Vector3[nx * ny * nz];
            gradients = new Vector3[positions.Length];
            samples = new float[positions.Length];
            fractions = new float[cells.Count];
            weights = new float[cells.Count];
            for (int z = 0; z < nz; z++)
            for (int y = 0; y < ny; y++)
            for (int x = 0; x < nx; x++)
                positions[Index(x, y, z)] = first + Vector3.Scale(spacing, new Vector3(x, y, z));
            for (int z = 0; z < cells.Z; z++)
            for (int y = 0; y < cells.Y; y++)
            for (int x = 0; x < cells.X; x++)
            {
                int cell = cells.Index(x, y, z);
                weights[cell] = (float)(release.Geometry.FluidVolumeM3[cell] / grid.CellVolume);
            }
        }

        public void Rebuild(BeakerVofTimeIntegration release, Mesh mesh)
        {
            var cells = release.Geometry.Grid.Cells;
            for (int z = 0; z < cells.Z; z++)
            for (int y = 0; y < cells.Y; y++)
            for (int x = 0; x < cells.X; x++)
                fractions[cells.Index(x, y, z)] = (float)release.LiquidFractionAt(x, y, z);

            for (int z = 0; z < nz; z++)
            for (int y = 0; y < ny; y++)
            for (int x = 0; x < nx; x++)
            {
                int i = x / Subdivisions - 1, j = y / Subdivisions - 1, k = z / Subdivisions - 1;
                float u = (float)(x % Subdivisions) / Subdivisions;
                float v = (float)(y % Subdivisions) / Subdivisions;
                float w = (float)(z % Subdivisions) / Subdivisions;
                float numerator = 0, denominator = 0;
                for (int dz = 0; dz <= 1; dz++)
                for (int dy = 0; dy <= 1; dy++)
                for (int dx = 0; dx <= 1; dx++)
                {
                    float weight = (dx == 0 ? 1 - u : u) * (dy == 0 ? 1 - v : v) * (dz == 0 ? 1 - w : w);
                    if (!cells.Interior(i + dx, j + dy, k + dz))
                    {
                        denominator += weight;
                        continue;
                    }
                    int cell = cells.Index(i + dx, j + dy, k + dz);
                    weight *= weights[cell];
                    denominator += weight;
                    numerator += weight * fractions[cell];
                }
                samples[Index(x, y, z)] = denominator > 0 ? numerator / denominator : 0;
            }

            for (int z = 1; z < nz - 1; z++)
            for (int y = 1; y < ny - 1; y++)
            for (int x = 1; x < nx - 1; x++)
                gradients[Index(x, y, z)] = new Vector3(
                    (samples[Index(x - 1, y, z)] - samples[Index(x + 1, y, z)]) / spacing.x,
                    (samples[Index(x, y - 1, z)] - samples[Index(x, y + 1, z)]) / spacing.y,
                    (samples[Index(x, y, z - 1)] - samples[Index(x, y, z + 1)]) / spacing.z);

            vertices.Clear();
            normals.Clear();
            triangles.Clear();
            for (int z = 0; z < nz - 1; z++)
            for (int y = 0; y < ny - 1; y++)
            for (int x = 0; x < nx - 1; x++)
            {
                int liquidCorners = 0;
                for (int c = 0; c < corners.Length; c++)
                {
                    int node = Index(x + (c & 1), y + ((c >> 1) & 1), z + (c >> 2));
                    corners[c] = node;
                    if (samples[node] >= SurfaceFraction) liquidCorners++;
                }
                if (liquidCorners == 0 || liquidCorners == corners.Length) continue;
                for (int t = 0; t < Tetrahedra.GetLength(0); t++) Polygonize(t);
            }
            mesh.Clear();
            mesh.SetVertices(vertices);
            mesh.SetNormals(normals);
            mesh.SetTriangles(triangles, 0);
        }

        private int Index(int x, int y, int z) => x + nx * (y + ny * z);

        private void Polygonize(int tetrahedron)
        {
            int wet = 0, dry = 0;
            for (int c = 0; c < 4; c++)
            {
                int node = corners[Tetrahedra[tetrahedron, c]];
                if (samples[node] >= SurfaceFraction) inside[wet++] = node;
                else outside[dry++] = node;
            }
            if (wet == 0 || dry == 0) return;
            Vector3 outward = positions[outside[0]] - positions[inside[0]];
            if (wet == 1)
                AddTriangle(Edge(inside[0], outside[0]), Edge(inside[0], outside[1]), Edge(inside[0], outside[2]), outward);
            else if (dry == 1)
                AddTriangle(Edge(inside[0], outside[0]), Edge(inside[1], outside[0]), Edge(inside[2], outside[0]), outward);
            else
            {
                int a = Edge(inside[0], outside[0]), b = Edge(inside[0], outside[1]);
                int c = Edge(inside[1], outside[0]), d = Edge(inside[1], outside[1]);
                AddTriangle(a, b, c, outward);
                AddTriangle(b, d, c, outward);
            }
        }

        private int Edge(int a, int b)
        {
            float t = (SurfaceFraction - samples[a]) / (samples[b] - samples[a]);
            int index = vertices.Count;
            vertices.Add(Vector3.LerpUnclamped(positions[a], positions[b], t));
            normals.Add(Vector3.LerpUnclamped(gradients[a], gradients[b], t).normalized);
            return index;
        }

        private void AddTriangle(int a, int b, int c, Vector3 outward)
        {
            bool forward = Vector3.Dot(Vector3.Cross(vertices[b] - vertices[a], vertices[c] - vertices[a]), outward) >= 0;
            triangles.Add(a);
            triangles.Add(forward ? b : c);
            triangles.Add(forward ? c : b);
        }
    }
}
