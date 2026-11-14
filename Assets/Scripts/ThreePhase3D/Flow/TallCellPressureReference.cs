using System;
using System.Collections.Generic;

namespace PhysicsSim.ThreePhase3D.Flow
{
    // Reference projection on a regular MAC grid restricted to a quadtree tall-cell
    // pressure basis. Fine-grid velocities are retained through the projection.
    public sealed class TallCellPressureReference
    {
        private sealed class Face
        {
            public int[] Nodes;
            public double[] Gradient;
            public double Velocity;
            public int Low, High, Axis;
        }

        private readonly int width, height, depth;
        private readonly double spacing;
        private readonly int[] lower, upper;
        private readonly double[] upperWeight;
        private readonly bool[] wet;
        private readonly List<Face> faces = new List<Face>();
        private int nodes;

        public int PressureSamples => nodes;
        public int FineWetCells { get; }

        // heights[x + width*z] gives the count of wet cells from the floor up.
        // Velocity is sampled at the centre of each interior MAC face.
        public TallCellPressureReference(int width, int height, int depth, double spacing,
            int[] heights, bool adaptive, Func<int, double, double, double, double> velocity)
        {
            if (width <= 0 || height <= 0 || depth <= 0 || !(spacing > 0) ||
                heights == null || heights.Length != width * depth || velocity == null)
                throw new ArgumentException("Invalid pressure-grid inputs.");
            this.width = width;
            this.height = height;
            this.depth = depth;
            this.spacing = spacing;
            lower = new int[width * height * depth];
            upper = new int[lower.Length];
            upperWeight = new double[lower.Length];
            wet = new bool[lower.Length];
            int cellCount = 0;
            for (int z = 0; z < depth; z++)
            for (int x = 0; x < width; x++)
            {
                int count = heights[x + width * z];
                if (count < 0 || count >= height)
                    throw new ArgumentException("Each column needs at least one air cell.");
                for (int y = 0; y < count; y++) wet[Index(x, y, z)] = true;
                cellCount += count;
            }
            FineWetCells = cellCount;

            // Subdivide power-of-two tiles at wet/dry transitions and where the
            // surface height differs. Keep the top two layers at fine resolution.
            int root = 1;
            while (root < Math.Max(width, depth)) root *= 2;
            BuildTile(0, 0, root, heights, adaptive);

            for (int z = 0; z < depth; z++)
            for (int y = 0; y < height; y++)
            for (int x = 0; x < width; x++)
            {
                int cell = Index(x, y, z);
                if (x + 1 < width) AddFace(cell, Index(x + 1, y, z), 0, x + 1, y + .5, z + .5, velocity);
                if (y + 1 < height) AddFace(cell, Index(x, y + 1, z), 1, x + .5, y + 1, z + .5, velocity);
                if (z + 1 < depth) AddFace(cell, Index(x, y, z + 1), 2, x + .5, y + .5, z + 1, velocity);
            }
        }

        private int Index(int x, int y, int z) => x + width * (y + height * z);

        private void BuildTile(int x, int z, int size, int[] heights, bool adaptive)
        {
            if (x >= width || z >= depth) return;
            int layers = heights[x + width * z];
            bool merge = adaptive && x + size <= width && z + size <= depth;
            for (int dz = 0; merge && dz < size; dz++)
            for (int dx = 0; merge && dx < size; dx++)
                merge = heights[x + dx + width * (z + dz)] == layers;
            if (size > 1 && (!merge || (layers > 0 && layers <= 4)))
            {
                int half = size / 2;
                BuildTile(x, z, half, heights, adaptive);
                BuildTile(x + half, z, half, heights, adaptive);
                BuildTile(x, z + half, half, heights, adaptive);
                BuildTile(x + half, z + half, half, heights, adaptive);
                return;
            }
            int tallLayers = adaptive && layers > 4 ? layers - 2 : 0;
            if (tallLayers > 0)
            {
                int bottom = nodes++, top = nodes++;
                for (int dz = 0; dz < size; dz++)
                for (int dx = 0; dx < size; dx++)
                for (int y = 0; y < tallLayers; y++)
                {
                    int cell = Index(x + dx, y, z + dz);
                    lower[cell] = bottom;
                    upper[cell] = top;
                    upperWeight[cell] = (y + .5) / tallLayers;
                }
            }
            for (int dz = 0; dz < size; dz++)
            for (int dx = 0; dx < size; dx++)
            for (int y = tallLayers; y < layers; y++)
                lower[Index(x + dx, y, z + dz)] = nodes++;
        }

        private void AddFace(int a, int b, int axis, double x, double y, double z,
            Func<int, double, double, double, double> velocity)
        {
            if (!wet[a] && !wet[b]) return;
            var weights = new Dictionary<int, double>();
            // Air pressure is zero. Use a half-cell distance at the free surface.
            double inverseDistance = (wet[a] && wet[b] ? 1 : 2) / spacing;
            AddCell(a, -inverseDistance);
            AddCell(b, inverseDistance);
            var face = new Face
            {
                Low = a, High = b, Axis = axis,
                Nodes = new int[weights.Count],
                Gradient = new double[weights.Count],
                Velocity = velocity(axis, x * spacing, y * spacing, z * spacing)
            };
            int i = 0;
            foreach (var coefficient in weights)
            {
                face.Nodes[i] = coefficient.Key;
                face.Gradient[i++] = coefficient.Value;
            }
            faces.Add(face);

            void AddCell(int cell, double scale)
            {
                if (!wet[cell]) return;
                Add(lower[cell], scale * (1 - upperWeight[cell]));
                if (upperWeight[cell] > 0) Add(upper[cell], scale * upperWeight[cell]);
            }

            void Add(int node, double value)
            {
                weights.TryGetValue(node, out double old);
                double sum = old + value;
                if (sum == 0) weights.Remove(node);
                else weights[node] = sum;
            }
        }

        // Solve G^T G q = G^T u and return u - Gq. All face dual volumes
        // are uniform in this reference model, so their common weight cancels.
        public (double[] Velocity, double RelativeResidual, int Iterations) Project()
        {
            var rhs = new double[nodes];
            var diagonal = new double[nodes];
            foreach (Face face in faces)
            for (int j = 0; j < face.Nodes.Length; j++)
            {
                int n = face.Nodes[j];
                double g = face.Gradient[j];
                rhs[n] += g * face.Velocity;
                diagonal[n] += g * g;
            }

            var pressure = new double[nodes];
            var residual = (double[])rhs.Clone();
            var direction = new double[nodes];
            var product = new double[nodes];
            double norm = Math.Sqrt(Dot(rhs, rhs));
            double previous = 0;
            int iteration = 0;
            for (; iteration < 400 && Math.Sqrt(Dot(residual, residual)) > norm * 1e-8; iteration++)
            {
                double weightedResidual = 0;
                for (int n = 0; n < nodes; n++)
                    weightedResidual += residual[n] * residual[n] / diagonal[n];
                double beta = iteration == 0 ? 0 : weightedResidual / previous;
                for (int n = 0; n < nodes; n++)
                    direction[n] = residual[n] / diagonal[n] + beta * direction[n];
                Array.Clear(product, 0, product.Length);
                foreach (Face face in faces) Scatter(face, Gradient(face, direction), product);
                double denominator = Dot(direction, product);
                if (!(denominator > 0)) throw new InvalidOperationException("Pressure operator lost positive definiteness.");
                double alpha = weightedResidual / denominator;
                for (int n = 0; n < nodes; n++)
                {
                    pressure[n] += alpha * direction[n];
                    residual[n] -= alpha * product[n];
                }
                previous = weightedResidual;
            }

            var velocity = new double[faces.Count];
            Array.Clear(residual, 0, residual.Length);
            for (int i = 0; i < faces.Count; i++)
            {
                Face face = faces[i];
                velocity[i] = face.Velocity - Gradient(face, pressure);
                Scatter(face, velocity[i], residual);
            }
            return (velocity, norm > 0 ? Math.Sqrt(Dot(residual, residual)) / norm : 0, iteration);
        }

        // Average the projected face-velocity change onto the adjacent liquid cells.
        public double[] CellVelocityCorrections(double[] projected)
        {
            if (projected == null || projected.Length != faces.Count)
                throw new ArgumentException("Expected one velocity for each pressure face.", nameof(projected));
            var change = new double[wet.Length * 3];
            var count = new int[change.Length];
            for (int i = 0; i < faces.Count; i++)
            {
                Face face = faces[i];
                double correction = projected[i] - face.Velocity;
                Accumulate(face.Low);
                Accumulate(face.High);

                void Accumulate(int cell)
                {
                    if (!wet[cell]) return;
                    int slot = 3 * cell + face.Axis;
                    change[slot] += correction;
                    count[slot]++;
                }
            }

            for (int i = 0; i < change.Length; i++)
                if (count[i] > 0) change[i] /= count[i];
            return change;
        }

        private static double Gradient(Face face, double[] values)
        {
            double gradient = 0;
            for (int j = 0; j < face.Nodes.Length; j++)
                gradient += face.Gradient[j] * values[face.Nodes[j]];
            return gradient;
        }

        private static void Scatter(Face face, double value, double[] result)
        {
            for (int j = 0; j < face.Nodes.Length; j++)
                result[face.Nodes[j]] += face.Gradient[j] * value;
        }

        private static double Dot(double[] a, double[] b)
        {
            double sum = 0;
            for (int i = 0; i < a.Length; i++) sum += a[i] * b[i];
            return sum;
        }
    }
}
