using System.Collections.Generic;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Rendering
{
    internal sealed class SphDamBreak2D
    {
        internal const float Width = 600f;
        internal const float Height = 360f;
        internal const float FineCellWidth = 15f;
        internal const float CoarseCellWidth = 30f;
        internal const float StepSeconds = .0005f;

        private const float FineSpacing = 5f;
        private const float CoarseSpacing = 10f;
        private const float FineSupport = 2 * FineSpacing;
        private const float CoarseSupport = 2 * CoarseSpacing;
        private const float InitialWidth = 240f;
        private const float CoarseDepth = 180f;
        private const float InitialDepth = 240f;
        private const float Gravity = 9810f;
        private const float SoundSpeed = 2500f;
        private const float ViscosityRate = 3f;

        private static readonly float ReferenceDensity = LatticeDensity();
        private readonly Vector2[] positions;
        private readonly Vector2[] velocities;
        private readonly Vector2[] accelerations;
        private readonly float[] densities;
        private readonly float[] pressures;
        private readonly List<int>[] neighbours;
        private readonly ParticleGrid fineGrid;
        private readonly ParticleGrid coarseGrid;

        internal int FineCount { get; }
        internal int Count => positions.Length;
        internal int StepCount { get; private set; }
        internal Vector2 Position(int index) => positions[index];
        internal IReadOnlyList<int> FineOccupiedCells => fineGrid.OccupiedCells;
        internal IReadOnlyList<int> CoarseOccupiedCells => coarseGrid.OccupiedCells;
        internal int FineCellsAcross => fineGrid.Columns;
        internal int CoarseCellsAcross => coarseGrid.Columns;

        internal SphDamBreak2D()
        {
            FineCount = (int)(InitialWidth / FineSpacing) * (int)((InitialDepth - CoarseDepth) / FineSpacing);
            int coarseCount = (int)(InitialWidth / CoarseSpacing) * (int)(CoarseDepth / CoarseSpacing);
            positions = new Vector2[FineCount + coarseCount];
            velocities = new Vector2[Count];
            accelerations = new Vector2[Count];
            densities = new float[Count];
            pressures = new float[Count];
            neighbours = new List<int>[Count];
            for (int i = 0; i < Count; i++) neighbours[i] = new List<int>(24);

            fineGrid = new ParticleGrid(FineCellWidth, Count);
            coarseGrid = new ParticleGrid(CoarseCellWidth, Count);
            Reset();
        }

        internal void Reset()
        {
            int index = 0;
            FillLayer(ref index, CoarseDepth, InitialDepth, FineSpacing);
            FillLayer(ref index, 0, CoarseDepth, CoarseSpacing);
            System.Array.Clear(velocities, 0, velocities.Length);
            StepCount = 0;
            RebuildGrids();
        }

        private void FillLayer(ref int index, float bottom, float top, float spacing)
        {
            for (float y = bottom + spacing * .5f; y < top; y += spacing)
            for (float x = spacing * .5f; x < InitialWidth; x += spacing)
                positions[index++] = new Vector2(x, y);
        }

        private void RebuildGrids()
        {
            fineGrid.Clear();
            coarseGrid.Clear();
            for (int i = 0; i < FineCount; i++) fineGrid.Insert(i, positions[i]);
            for (int i = FineCount; i < Count; i++) coarseGrid.Insert(i, positions[i]);
        }

        internal void Advance(int steps)
        {
            for (int step = 0; step < steps; step++)
            {
                FindNeighboursAndDensities();
                CalculateForces();
                Integrate();
                RebuildGrids();
                StepCount++;
            }
        }

        private void FindNeighboursAndDensities()
        {
            for (int i = 0; i < Count; i++)
            {
                bool fine = i < FineCount;
                float support = fine ? FineSupport : CoarseSupport;
                float mass = fine ? FineSpacing * FineSpacing : CoarseSpacing * CoarseSpacing;
                Vector2 position = positions[i];
                List<int> list = neighbours[i];
                list.Clear();
                fineGrid.Gather(i, position, support, positions, list);
                coarseGrid.Gather(i, position, CoarseSupport, positions, list);

                float density = mass * DensityKernel(0, support);
                foreach (int j in list)
                {
                    float otherSupport = j < FineCount ? FineSupport : CoarseSupport;
                    float otherMass = j < FineCount ? FineSpacing * FineSpacing : CoarseSpacing * CoarseSpacing;
                    float distanceSquared = (positions[j] - position).sqrMagnitude;
                    float kernel = DensityKernel(distanceSquared, support);
                    if (support != otherSupport)
                        kernel = .5f * (kernel + DensityKernel(distanceSquared, otherSupport));
                    density += otherMass * kernel;
                }
                densities[i] = density;
                pressures[i] = SoundSpeed * SoundSpeed * Mathf.Max(0, density - ReferenceDensity);
            }
        }

        private void CalculateForces()
        {
            for (int i = 0; i < Count; i++)
            {
                float support = i < FineCount ? FineSupport : CoarseSupport;
                float density = densities[i];
                float ownPressureTerm = pressures[i] / (density * density);
                Vector2 acceleration = new Vector2(0, -Gravity);
                foreach (int j in neighbours[i])
                {
                    Vector2 separation = positions[j] - positions[i];
                    float distanceSquared = separation.sqrMagnitude;
                    float distance = Mathf.Sqrt(distanceSquared);
                    float otherSupport = j < FineCount ? FineSupport : CoarseSupport;
                    float otherMass = j < FineCount ? FineSpacing * FineSpacing : CoarseSpacing * CoarseSpacing;
                    float otherDensity = densities[j];
                    float derivative = PressureDerivative(distance, support);
                    float kernel = DensityKernel(distanceSquared, support);
                    if (support != otherSupport)
                    {
                        derivative = .5f * (derivative + PressureDerivative(distance, otherSupport));
                        kernel = .5f * (kernel + DensityKernel(distanceSquared, otherSupport));
                    }

                    if (distance > 1e-6f)
                        acceleration += otherMass * (ownPressureTerm + pressures[j] / (otherDensity * otherDensity)) *
                            derivative * (separation / distance);
                    acceleration += otherMass * (ViscosityRate * kernel / (.5f * (density + otherDensity))) *
                        (velocities[j] - velocities[i]);
                }
                accelerations[i] = acceleration;
            }
        }

        private void Integrate()
        {
            for (int i = 0; i < Count; i++)
            {
                Vector2 velocity = velocities[i] + accelerations[i] * StepSeconds;
                Vector2 position = positions[i] + velocity * StepSeconds;
                float margin = (i < FineCount ? FineSpacing : CoarseSpacing) * .5f;
                Constrain(ref position.x, ref velocity.x, margin, Width - margin);
                Constrain(ref position.y, ref velocity.y, margin, Height - margin);
                velocities[i] = velocity;
                positions[i] = position;
            }
        }

        private static void Constrain(ref float coordinate, ref float velocity, float low, float high)
        {
            if (coordinate < low)
            {
                coordinate = low;
                velocity = Mathf.Max(velocity, -velocity * .1f);
            }
            else if (coordinate > high)
            {
                coordinate = high;
                velocity = Mathf.Min(velocity, -velocity * .1f);
            }
        }

        private static float DensityKernel(float squaredDistance, float support)
        {
            float h2 = support * support;
            float q = Mathf.Max(0, 1 - squaredDistance / h2);
            return 4f * q * q * q / (Mathf.PI * h2);
        }

        private static float PressureDerivative(float distance, float support)
        {
            float q = Mathf.Max(0, 1 - distance / support);
            return -30f * q * q / (Mathf.PI * support * support * support);
        }

        private static float LatticeDensity()
        {
            float sum = 0;
            for (int y = -2; y <= 2; y++)
            for (int x = -2; x <= 2; x++)
                sum += FineSpacing * FineSpacing * DensityKernel((x * x + y * y) * FineSpacing * FineSpacing, FineSupport);
            return sum;
        }

        private sealed class ParticleGrid
        {
            private readonly float cellWidth;
            private readonly int rows;
            private readonly int[] first;
            private readonly int[] next;
            private readonly List<int> occupiedCells = new List<int>();

            internal int Columns { get; }
            internal IReadOnlyList<int> OccupiedCells => occupiedCells;

            internal ParticleGrid(float width, int particles)
            {
                cellWidth = width;
                Columns = Mathf.CeilToInt(Width / width);
                rows = Mathf.CeilToInt(Height / width);
                first = new int[Columns * rows];
                next = new int[particles];
                for (int i = 0; i < first.Length; i++) first[i] = -1;
            }

            internal void Clear()
            {
                foreach (int cell in occupiedCells) first[cell] = -1;
                occupiedCells.Clear();
            }

            internal void Insert(int particle, Vector2 position)
            {
                int cell = ToColumn(position.x) + ToRow(position.y) * Columns;
                if (first[cell] == -1) occupiedCells.Add(cell);
                next[particle] = first[cell];
                first[cell] = particle;
            }

            internal void Gather(int particle, Vector2 position, float support, Vector2[] positions, List<int> hits)
            {
                int right = ToColumn(position.x + support);
                int top = ToRow(position.y + support);
                int left = ToColumn(position.x - support);
                float reachSquared = support * support;
                for (int y = ToRow(position.y - support); y <= top; y++)
                for (int x = left; x <= right; x++)
                for (int j = first[x + y * Columns]; j >= 0; j = next[j])
                {
                    if (j != particle && (positions[j] - position).sqrMagnitude <= reachSquared)
                        hits.Add(j);
                }
            }

            private int ToColumn(float x) => Mathf.Clamp(Mathf.FloorToInt(x / cellWidth), 0, Columns - 1);
            private int ToRow(float y) => Mathf.Clamp(Mathf.FloorToInt(y / cellWidth), 0, rows - 1);
        }
    }
}
