using System.Collections.Generic;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Rendering
{
    internal sealed class SphDamBreak2D
    {
        internal const float Width = 600f;
        internal const float Height = 360f;
        private static readonly float[] CellWidths = { 40f, 30f, 20f, 10f };
        private const int ChildrenPerSplit = 4;
        private const int NodesPerRoot = 85; // 1 + 4 + 16 + 64
        private const int RefinementTicks = 320;
        private const float TickSeconds = .000125f;
        private static readonly float[] ParticleSpacings = { 10f, 5f, 2.5f, 1.25f };
        private static readonly float[] SplitSpeeds = { 650f, 400f, 200f };
        private static readonly float[] MergeSpeeds = { 800f, 550f, 300f };

        private const float CoarseSpacing = 10f;
        private const float InitialWidth = 240f;
        private const float InitialDepth = 240f;
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
        private readonly ParticleGrid[] grids;
        private readonly int[] levels;
        private readonly int[] particlesPerLevel;
        private readonly bool[] split;
        private readonly bool[] exposed;
        private int simulatedTicks;
        private int finestLevel;
        private readonly List<int> activeParticles;

        internal int Count => activeParticles.Count;
        internal IReadOnlyList<int> ActiveParticles => activeParticles;
        internal float SimulatedSeconds => simulatedTicks * TickSeconds;
        private int TicksPerStep => 4 >> Mathf.Min(finestLevel, 2);
        internal float StepSeconds => TicksPerStep * TickSeconds;
        internal float ParticleSpacing(int index) => ParticleSpacings[levels[index]];
        internal int StepCount { get; private set; }
        internal Vector2 Position(int index) => positions[index];
        internal int GridLevelCount => grids.Length;
        internal int GridLevel(int index) => levels[index];
        internal int ParticlesAtLevel(int level) => particlesPerLevel[level];
        internal float CellWidthAtLevel(int level) => CellWidths[level];
        internal IReadOnlyList<int> OccupiedCellsAtLevel(int level) => grids[level].OccupiedCells;
        internal int CellsAcrossAtLevel(int level) => grids[level].Columns;

        internal SphDamBreak2D()
        {
            int coarseCount = (int)(InitialWidth / CoarseSpacing) * (int)(InitialDepth / CoarseSpacing);
            int capacity = coarseCount * NodesPerRoot;
            split = new bool[capacity];
            exposed = new bool[capacity];
            activeParticles = new List<int>(capacity);
            positions = new Vector2[capacity];
            velocities = new Vector2[capacity];
            accelerations = new Vector2[capacity];
            densities = new float[capacity];
            pressures = new float[capacity];
            neighbours = new List<int>[capacity];

            levels = new int[capacity];
            particlesPerLevel = new int[CellWidths.Length];
            grids = new ParticleGrid[CellWidths.Length];
            for (int level = 0; level < grids.Length; level++)
                grids[level] = new ParticleGrid(CellWidths[level], capacity);
            Reset();
        }

        internal void Reset()
        {
            System.Array.Clear(split, 0, split.Length);
            System.Array.Clear(velocities, 0, velocities.Length);
            int index = 0;
            for (float y = CoarseSpacing * .5f; y < InitialDepth; y += CoarseSpacing)
            for (float x = CoarseSpacing * .5f; x < InitialWidth; x += CoarseSpacing)
            {
                positions[index] = new Vector2(x, y);
                index += NodesPerRoot;
            }

            StepCount = simulatedTicks = 0;
            RebuildActiveParticles();
            RebuildGrids();
        }

        private void RebuildActiveParticles()
        {
            activeParticles.Clear();
            finestLevel = 0;
            for (int root = 0; root < positions.Length; root += NodesPerRoot)
                AddLeaves(root, 0);
        }

        private void AddLeaves(int index, int level)
        {
            if (!split[index])
            {
                activeParticles.Add(index);
                finestLevel = Mathf.Max(finestLevel, level);
                return;
            }

            int first = FirstChild(index);
            for (int i = 0; i < ChildrenPerSplit; i++)
                AddLeaves(first + i, level + 1);
        }

        private static int FirstChild(int index)
        {
            int local = index % NodesPerRoot;
            return index - local + ChildrenPerSplit * local + 1;
        }

        private void RebuildGrids()
        {
            foreach (ParticleGrid grid in grids) grid.Clear();
            System.Array.Clear(particlesPerLevel, 0, particlesPerLevel.Length);
            foreach (int index in activeParticles)
            {
                int level = levels[index];
                particlesPerLevel[level]++;
                grids[level].Insert(index, positions[index]);
            }
        }

        private bool AdaptParticleSizes()
        {
            foreach (int index in activeParticles) exposed[index] = IsExposed(index);

            bool changed = false;
            foreach (int index in activeParticles)
            {
                int level = levels[index];
                if (level > 0)
                {
                    int local = index % NodesPerRoot;
                    int parent = index - local + (local - 1) / ChildrenPerSplit;
                    if (!split[parent]) continue;
                    int first = FirstChild(parent);
                    if (index == first && TryMerge(parent, first, level - 1))
                    {
                        split[parent] = false;
                        changed = true;
                        continue;
                    }
                }

                if (level < SplitSpeeds.Length && exposed[index] &&
                    velocities[index].sqrMagnitude < SplitSpeeds[level] * SplitSpeeds[level])
                {
                    Split(index, level + 1);
                    changed = true;
                }
            }
            if (changed) RebuildActiveParticles();
            return changed;
        }

        private bool IsExposed(int index)
        {
            Vector2 position = positions[index];
            float spacing = ParticleSpacing(index);
            bool firstOpen = false, previousOpen = false;
            for (int i = 0; i < SurfaceDirections.Length; i++)
            {
                Vector2 probe = position + SurfaceDirections[i] * (1.5f * spacing);
                // Probes outside the tank touch solid, and occupied probes represent water.
                bool open = probe.x > 0 && probe.x < Width && probe.y > 0 && probe.y < Height &&
                    !HasParticleNear(probe, spacing);
                if (open && previousOpen) return true;
                if (i == 0) firstOpen = open;
                previousOpen = open;
            }
            return firstOpen && previousOpen;
        }

        private bool HasParticleNear(Vector2 position, float spacing)
        {
            for (int level = 0; level < grids.Length; level++)
            {
                float radius = .8f * Mathf.Max(spacing, ParticleSpacings[level]);
                if (grids[level].HasParticleNear(position, radius, positions)) return true;
            }
            return false;
        }

        private void Split(int index, int childLevel)
        {
            int first = FirstChild(index);
            float offset = ParticleSpacings[childLevel] * .5f;
            for (int i = 0; i < ChildrenPerSplit; i++)
            {
                float x = i % 2 == 0 ? -offset : offset;
                float y = i / 2 == 0 ? -offset : offset;
                positions[first + i] = positions[index] + new Vector2(x, y);
                velocities[first + i] = velocities[index];
                levels[first + i] = childLevel;
            }
            split[index] = true;
        }

        private bool TryMerge(int parent, int first, int parentLevel)
        {
            Vector2 centre = Vector2.zero;
            Vector2 velocity = Vector2.zero;
            bool anyExposed = false, allFast = true;
            float speedSquared = MergeSpeeds[parentLevel] * MergeSpeeds[parentLevel];
            for (int i = first; i < first + ChildrenPerSplit; i++)
            {
                if (split[i]) return false;
                anyExposed |= exposed[i];
                allFast &= velocities[i].sqrMagnitude >= speedSquared;
                centre += positions[i];
                velocity += velocities[i];
            }
            centre /= ChildrenPerSplit;
            velocity /= ChildrenPerSplit;
            if (anyExposed && (!allFast || velocity.sqrMagnitude < speedSquared)) return false;

            float spacing = ParticleSpacings[parentLevel];
            float margin = spacing * .5f;
            if (centre.x < margin || centre.x > Width - margin ||
                centre.y < margin || centre.y > Height - margin) return false;

            float mergeRadius = .75f * spacing;
            for (int i = first; i < first + ChildrenPerSplit; i++)
                if ((positions[i] - centre).sqrMagnitude > mergeRadius * mergeRadius) return false;

            positions[parent] = centre;
            velocities[parent] = velocity;
            return true;
        }

        internal void Advance()
        {
            int ticks = TicksPerStep;
            FindNeighboursAndDensities();
            CalculateForces();
            Integrate(ticks * TickSeconds);
            StepCount++;
            simulatedTicks += ticks;
            RebuildGrids();
            if (simulatedTicks % RefinementTicks == 0 && AdaptParticleSizes()) RebuildGrids();
        }

        private void FindNeighboursAndDensities()
        {
            foreach (int i in activeParticles)
            {
                float spacing = ParticleSpacing(i);
                float support = 2 * spacing;
                float mass = spacing * spacing;
                Vector2 position = positions[i];
                List<int> list = neighbours[i] ??= new List<int>(24);
                list.Clear();
                for (int level = 0; level < grids.Length; level++)
                    grids[level].Gather(i, position, Mathf.Max(support, 2 * ParticleSpacings[level]), positions, list);

                float density = mass * DensityKernel(0, support);
                foreach (int j in list)
                {
                    float otherSpacing = ParticleSpacing(j);
                    float otherSupport = 2 * otherSpacing;
                    float otherMass = otherSpacing * otherSpacing;
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
                float support = 2 * ParticleSpacing(i);
                float density = densities[i];
                float ownPressureTerm = pressures[i] / (density * density);
                Vector2 acceleration = new Vector2(0, -Gravity);
                foreach (int j in neighbours[i])
                {
                    Vector2 separation = positions[j] - positions[i];
                    float distanceSquared = separation.sqrMagnitude;
                    float distance = Mathf.Sqrt(distanceSquared);
                    float otherSpacing = ParticleSpacing(j);
                    float otherSupport = 2 * otherSpacing;
                    float otherMass = otherSpacing * otherSpacing;
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

        private void Integrate(float stepSeconds)
        {
            foreach (int i in activeParticles)
            {
                Vector2 velocity = velocities[i] + accelerations[i] * stepSeconds;
                Vector2 position = positions[i] + velocity * stepSeconds;
                float margin = ParticleSpacing(i) * .5f;
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
                sum += CoarseSpacing * CoarseSpacing * DensityKernel((x * x + y * y) * CoarseSpacing * CoarseSpacing, 2 * CoarseSpacing);
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
