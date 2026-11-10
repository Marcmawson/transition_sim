using System;

namespace PhaseTransition.Physics
{
    /// <summary>
    /// Educational 2-D lattice gas: binary site occupancy, attractive nearest-neighbour pairs,
    /// and Metropolis sampling. This is not a quantitative molecular model of water.
    /// </summary>
    public sealed class LatticeGas2D
    {
        private readonly bool[,] occupied;
        private readonly Random random;

        public int Width { get; }
        public int Height { get; }
        public int ParticleCount { get; private set; }
        public double PairAttractionJ { get; }
        public double TemperatureK { get; set; }

        /// <summary>
        /// Pair energy that puts the square-lattice lattice gas' critical point at the
        /// requested temperature. This matches one critical-point coordinate only; it
        /// does not fit water's coexistence curve or molecular structure.
        /// </summary>
        public static double PairEnergyForSquareLatticeCriticalTemperature(double criticalTemperatureK)
        {
            if (criticalTemperatureK <= 0.0) throw new ArgumentOutOfRangeException(nameof(criticalTemperatureK));
            const double squareLatticeGasCriticalRatio = 0.5672963285532555; // k_B*T_c / epsilon
            return 1.380649e-23 * criticalTemperatureK / squareLatticeGasCriticalRatio;
        }

        /// <param name="pairAttractionJ">Attractive energy per adjacent occupied pair, in joules.</param>
        /// <param name="seed">Reproducibility seed.</param>
        public LatticeGas2D(int width, int height, int particleCount, double temperatureK,
            double pairAttractionJ, int seed = 1)
        {
            if (width < 2 || height < 2) throw new ArgumentOutOfRangeException(nameof(width), "Lattice must be at least 2 by 2.");
            if (particleCount < 0 || particleCount > width * height) throw new ArgumentOutOfRangeException(nameof(particleCount));
            if (temperatureK <= 0.0) throw new ArgumentOutOfRangeException(nameof(temperatureK));
            if (pairAttractionJ <= 0.0) throw new ArgumentOutOfRangeException(nameof(pairAttractionJ));

            Width = width;
            Height = height;
            ParticleCount = particleCount;
            TemperatureK = temperatureK;
            PairAttractionJ = pairAttractionJ;
            occupied = new bool[width, height];
            random = new Random(seed);

            int[] sites = new int[width * height];
            for (int i = 0; i < sites.Length; i++) sites[i] = i;
            for (int i = sites.Length - 1; i > 0; i--)
            {
                int j = random.Next(i + 1);
                int tmp = sites[i]; sites[i] = sites[j]; sites[j] = tmp;
            }
            for (int i = 0; i < particleCount; i++)
                occupied[sites[i] % width, sites[i] / width] = true;
        }

        public bool IsOccupied(int x, int y) => occupied[Wrap(x, Width), Wrap(y, Height)];

        /// <summary>
        /// Attempts one particle-vacancy exchange. Call repeatedly; every attempt conserves N.
        /// A lattice sweep is Width*Height attempts. Periodic boundaries are used.
        /// </summary>
        public bool AttemptKawasakiMove()
        {
            int x1 = random.Next(Width), y1 = random.Next(Height);
            int x2 = random.Next(Width), y2 = random.Next(Height);
            if (occupied[x1, y1] == occupied[x2, y2]) return false;

            int pairsBefore = LocalPairCount(x1, y1) + LocalPairCount(x2, y2);
            occupied[x1, y1] = !occupied[x1, y1];
            occupied[x2, y2] = !occupied[x2, y2];
            int pairsAfter = LocalPairCount(x1, y1) + LocalPairCount(x2, y2);
            double deltaEnergy = -PairAttractionJ * (pairsAfter - pairsBefore);

            bool accept = deltaEnergy <= 0.0 ||
                random.NextDouble() < Math.Exp(-deltaEnergy / (1.380649e-23 * TemperatureK));
            if (!accept)
            {
                occupied[x1, y1] = !occupied[x1, y1];
                occupied[x2, y2] = !occupied[x2, y2];
            }
            return accept;
        }

        public void Sweep(int attemptMultiplier = 1)
        {
            int attempts = checked(Width * Height * Math.Max(1, attemptMultiplier));
            for (int i = 0; i < attempts; i++) AttemptKawasakiMove();
        }

        public int CountOccupiedNeighbourPairs()
        {
            int pairs = 0;
            for (int y = 0; y < Height; y++)
                for (int x = 0; x < Width; x++)
                {
                    if (!occupied[x, y]) continue;
                    if (occupied[Wrap(x + 1, Width), y]) pairs++;
                    if (occupied[x, Wrap(y + 1, Height)]) pairs++;
                }
            return pairs;
        }

        public double ConfigurationalEnergyJ => -PairAttractionJ * CountOccupiedNeighbourPairs();

        private int NeighbourCount(int x, int y)
        {
            int count = 0;
            if (occupied[Wrap(x - 1, Width), y]) count++;
            if (occupied[Wrap(x + 1, Width), y]) count++;
            if (occupied[x, Wrap(y - 1, Height)]) count++;
            if (occupied[x, Wrap(y + 1, Height)]) count++;
            return count;
        }

        // Counts bonds incident on an occupied site. A proposed exchange always
        // selects one occupied and one empty site, so no selected-selected bond exists.
        private int LocalPairCount(int x, int y) => occupied[x, y] ? NeighbourCount(x, y) : 0;

        private static int Wrap(int value, int size) => (value % size + size) % size;
    }
}
