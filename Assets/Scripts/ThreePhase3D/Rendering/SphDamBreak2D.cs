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
        private const float InitialDepth = 240f;
        private const int ChildrenPerCoarseParticle = 4;
        private const int RefinementIntervalSteps = 80;
        private const float SplitSpeed = 400f;
        private const float MergeSpeed = 650f;
        private const float MergeRadius = 7.5f;
        private const float SurfaceProbeDistance = 1.5f * CoarseSpacing;
        private const float SurfaceProbeRadius = .8f * CoarseSpacing;
        private const float Diagonal = .70710678f;
        private static readonly Vector2[] SurfaceDirections =
        {
            new Vector2(1, 0), new Vector2(Diagonal, Diagonal), new Vector2(0, 1),
            new Vector2(-Diagonal, Diagonal), new Vector2(-1, 0), new Vector2(-Diagonal, -Diagonal),
            new Vector2(0, -1), new Vector2(Diagonal, -Diagonal)
        };
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
        private readonly bool[] refined;
        private readonly bool[] surfaceFamilies;
        private readonly List<int> activeParticles;

        internal int FineCount { get; private set; }
        internal int Count => activeParticles.Count;
        internal IReadOnlyList<int> ActiveParticles => activeParticles;
        internal bool IsFine(int index) => refined[index / ChildrenPerCoarseParticle];
        internal int StepCount { get; private set; }
        internal Vector2 Position(int index) => positions[index];
        internal IReadOnlyList<int> FineOccupiedCells => fineGrid.OccupiedCells;
        internal IReadOnlyList<int> CoarseOccupiedCells => coarseGrid.OccupiedCells;
        internal int FineCellsAcross => fineGrid.Columns;
        internal int CoarseCellsAcross => coarseGrid.Columns;

        internal SphDamBreak2D()
        {
            int coarseCount = (int)(InitialWidth / CoarseSpacing) * (int)(InitialDepth / CoarseSpacing);
            int capacity = coarseCount * ChildrenPerCoarseParticle;
            refined = new bool[coarseCount];
            surfaceFamilies = new bool[coarseCount];
            activeParticles = new List<int>(capacity);
            positions = new Vector2[capacity];
            velocities = new Vector2[capacity];
            accelerations = new Vector2[capacity];
            densities = new float[capacity];
            pressures = new float[capacity];
            neighbours = new List<int>[capacity];
            for (int i = 0; i < capacity; i++) neighbours[i] = new List<int>(24);

            fineGrid = new ParticleGrid(FineCellWidth, capacity);
            coarseGrid = new ParticleGrid(CoarseCellWidth, capacity);
            Reset();
        }

        internal void Reset()
        {
            System.Array.Clear(refined, 0, refined.Length);
            System.Array.Clear(velocities, 0, velocities.Length);
            int index = 0;
            for (float y = CoarseSpacing * .5f; y < InitialDepth; y += CoarseSpacing)
            for (float x = CoarseSpacing * .5f; x < InitialWidth; x += CoarseSpacing)
            {
                positions[index] = new Vector2(x, y);
                index += ChildrenPerCoarseParticle;
            }

            StepCount = 0;
            RebuildActiveParticles();
            RebuildGrids();
        }

        private void RebuildActiveParticles()
        {
            activeParticles.Clear();
            FineCount = 0;
            for (int family = 0; family < refined.Length; family++)
            {
                int first = family * ChildrenPerCoarseParticle;
                int count = refined[family] ? ChildrenPerCoarseParticle : 1;
                for (int i = 0; i < count; i++) activeParticles.Add(first + i);
                if (refined[family]) FineCount += count;
            }
        }

        private void RebuildGrids()
        {
            fineGrid.Clear();
            coarseGrid.Clear();
            foreach (int index in activeParticles)
                (IsFine(index) ? fineGrid : coarseGrid).Insert(index, positions[index]);
        }

        private bool AdaptParticleSizes()
        {
            // Classify against one complete snapshot before splitting or merging changes the grid.
            for (int family = 0; family < refined.Length; family++)
            {
                int first = family * ChildrenPerCoarseParticle;
                surfaceFamilies[family] = false;
                int count = refined[family] ? ChildrenPerCoarseParticle : 1;
                for (int i = 0; i < count; i++)
                {
                    if (!IsExposed(positions[first + i])) continue;
                    surfaceFamilies[family] = true;
                    break;
                }
            }

            bool changed = false;
            for (int family = 0; family < refined.Length; family++)
            {
                int first = family * ChildrenPerCoarseParticle;
                if (refined[family])
                {
                    if (!TryMerge(first, surfaceFamilies[family])) continue;
                    refined[family] = false;
                    changed = true;
                }
                else if (surfaceFamilies[family] && velocities[first].sqrMagnitude < SplitSpeed * SplitSpeed)
                {
                    Split(first);
                    refined[family] = true;
                    changed = true;
                }
            }
            if (changed) RebuildActiveParticles();
            return changed;
        }

        private bool IsExposed(Vector2 position)
        {
            bool firstOpen = false, previousOpen = false;
            for (int i = 0; i < SurfaceDirections.Length; i++)
            {
                Vector2 probe = position + SurfaceDirections[i] * SurfaceProbeDistance;
                // A probe outside the tank hits a solid boundary.
                bool open = probe.x > 0 && probe.x < Width && probe.y > 0 && probe.y < Height &&
                    !fineGrid.HasParticleNear(probe, SurfaceProbeRadius, positions) &&
                    !coarseGrid.HasParticleNear(probe, SurfaceProbeRadius, positions);
                if (open && previousOpen) return true;
                if (i == 0) firstOpen = open;
                previousOpen = open;
            }
            return firstOpen && previousOpen;
        }

        private void Split(int first)
        {
            Vector2 centre = positions[first];
            Vector2 velocity = velocities[first];
            for (int i = 0; i < ChildrenPerCoarseParticle; i++)
            {
                float x = (i % 2 == 0 ? -.5f : .5f) * FineSpacing;
                float y = (i / 2 == 0 ? -.5f : .5f) * FineSpacing;
                positions[first + i] = centre + new Vector2(x, y);
                velocities[first + i] = velocity;
            }
        }

        private bool TryMerge(int first, bool requireFast)
        {
            Vector2 centre = Vector2.zero;
            Vector2 velocity = Vector2.zero;
            for (int i = first; i < first + ChildrenPerCoarseParticle; i++)
            {
                if (requireFast && velocities[i].sqrMagnitude < MergeSpeed * MergeSpeed) return false;
                centre += positions[i];
                velocity += velocities[i];
            }
            centre /= ChildrenPerCoarseParticle;
            velocity /= ChildrenPerCoarseParticle;
            if (requireFast && velocity.sqrMagnitude < MergeSpeed * MergeSpeed) return false;

            float margin = CoarseSpacing * .5f;
            if (centre.x < margin || centre.x > Width - margin ||
                centre.y < margin || centre.y > Height - margin) return false;

            for (int i = first; i < first + ChildrenPerCoarseParticle; i++)
                if ((positions[i] - centre).sqrMagnitude > MergeRadius * MergeRadius) return false;

            positions[first] = centre;
            velocities[first] = velocity;
            return true;
        }

        internal void Advance(int steps)
        {
            for (int step = 0; step < steps; step++)
            {
                FindNeighboursAndDensities();
                CalculateForces();
                Integrate();
                StepCount++;
                RebuildGrids();
                if (StepCount % RefinementIntervalSteps == 0 && AdaptParticleSizes())
                    RebuildGrids();
            }
        }

        private void FindNeighboursAndDensities()
        {
            foreach (int i in activeParticles)
            {
                bool fine = IsFine(i);
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
                    float otherSupport = IsFine(j) ? FineSupport : CoarseSupport;
                    float otherMass = IsFine(j) ? FineSpacing * FineSpacing : CoarseSpacing * CoarseSpacing;
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
            foreach (int i in activeParticles)
            {
                float support = IsFine(i) ? FineSupport : CoarseSupport;
                float density = densities[i];
                float ownPressureTerm = pressures[i] / (density * density);
                Vector2 acceleration = new Vector2(0, -Gravity);
                foreach (int j in neighbours[i])
                {
                    Vector2 separation = positions[j] - positions[i];
                    float distanceSquared = separation.sqrMagnitude;
                    float distance = Mathf.Sqrt(distanceSquared);
                    float otherSupport = IsFine(j) ? FineSupport : CoarseSupport;
                    float otherMass = IsFine(j) ? FineSpacing * FineSpacing : CoarseSpacing * CoarseSpacing;
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
            foreach (int i in activeParticles)
            {
                Vector2 velocity = velocities[i] + accelerations[i] * StepSeconds;
                Vector2 position = positions[i] + velocity * StepSeconds;
                float margin = (IsFine(i) ? FineSpacing : CoarseSpacing) * .5f;
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

            internal bool HasParticleNear(Vector2 position, float radius, Vector2[] positions)
            {
                int left = ToColumn(position.x - radius);
                int right = ToColumn(position.x + radius);
                int top = ToRow(position.y + radius);
                float squaredRadius = radius * radius;
                for (int y = ToRow(position.y - radius); y <= top; y++)
                for (int x = left; x <= right; x++)
                for (int index = first[x + y * Columns]; index >= 0; index = next[index])
                    if ((positions[index] - position).sqrMagnitude <= squaredRadius) return true;
                return false;
            }

            private int ToColumn(float x) => Mathf.Clamp(Mathf.FloorToInt(x / cellWidth), 0, Columns - 1);
            private int ToRow(float y) => Mathf.Clamp(Mathf.FloorToInt(y / cellWidth), 0, rows - 1);
        }
    }
}
