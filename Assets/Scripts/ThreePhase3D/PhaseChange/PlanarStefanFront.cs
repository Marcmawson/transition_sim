using System;
using System.Collections.Generic;
using PhysicsSim.ThreePhase3D.Core;

namespace PhysicsSim.ThreePhase3D.PhaseChange
{
    // One-phase planar Stefan growth with constant, equal phase densities.
    // The moving cells span the growing phase. The untransformed phase stays at Tm.
    public sealed class PlanarStefanFront
    {
        private readonly double density, heatCapacity, conductivity, latentHeat, wallOffset;
        private readonly int heatDirection;
        private readonly double[] temperature, next, flux;
        public IReadOnlyList<double> TemperatureOffsetK { get; }
        public double FrontM { get; private set; }
        public double HeatInputJPerM2 { get; private set; }

        public PlanarStefanFront(double densityKgM3, double heatCapacityJkgK, double conductivityWmK,
            double latentHeatJkg, double wallOffsetK, double seedFrontM, double[] seedOffsetK)
        {
            density = Positive(densityKgM3);
            heatCapacity = Positive(heatCapacityJkgK);
            conductivity = Positive(conductivityWmK);
            latentHeat = Positive(latentHeatJkg);
            if (!SiUnits.IsFinite(wallOffsetK) || wallOffsetK == 0)
                throw new ArgumentOutOfRangeException(nameof(wallOffsetK));
            wallOffset = wallOffsetK;
            heatDirection = Math.Sign(wallOffset);
            FrontM = Positive(seedFrontM);
            if (seedOffsetK == null || seedOffsetK.Length < 3)
                throw new ArgumentException("At least three seeded cells are required.", nameof(seedOffsetK));
            temperature = (double[])seedOffsetK.Clone();
            foreach (double value in temperature)
                if (!SiUnits.IsFinite(value) || heatDirection * value < 0 || heatDirection * value > Math.Abs(wallOffset))
                    throw new ArgumentOutOfRangeException(nameof(seedOffsetK));
            next = new double[temperature.Length];
            flux = new double[temperature.Length + 1];
            TemperatureOffsetK = Array.AsReadOnly(temperature);
        }

        public double EnergyJPerM2
        {
            get
            {
                double sum = 0;
                foreach (double value in temperature) sum += value;
                return density * FrontM * (heatDirection * latentHeat + heatCapacity * sum / temperature.Length);
            }
        }

        public double Advance(double requestedSeconds)
        {
            Positive(requestedSeconds);
            int n = temperature.Length;
            double width = FrontM / n;
            // Boundary derivatives reconstructed from the prescribed face
            // temperature and its two nearest cell averages.
            double wallFlux = conductivity * (6 * wallOffset - 7 * temperature[0] + temperature[1]) / (2 * width);
            double frontFlux = conductivity * (7 * temperature[n - 1] - temperature[n - 2]) / (2 * width);
            if (heatDirection * frontFlux < 0) throw new InvalidOperationException("Seed profile drives the front in the wrong direction.");
            double speed = heatDirection * frontFlux / (density * latentHeat);
            double diffusionRate = conductivity / (density * heatCapacity * width * width);
            double dt = Math.Min(requestedSeconds, 0.1 / (diffusionRate + speed / width));
            double nextFront = FrontM + dt * speed;
            double nextWidth = nextFront / n;
            flux[0] = wallFlux;
            flux[n] = frontFlux;
            for (int face = 1; face < n; face++)
            {
                double meshSpeed = speed * face / n;
                flux[face] = conductivity * (temperature[face - 1] - temperature[face]) / width
                    - density * heatCapacity * meshSpeed * temperature[face];
            }

            for (int cell = 0; cell < n; cell++)
            {
                double sensible = density * heatCapacity * width * temperature[cell]
                    + dt * (flux[cell] - flux[cell + 1]);
                next[cell] = sensible / (density * heatCapacity * nextWidth);
                if (!SiUnits.IsFinite(next[cell]) || heatDirection * next[cell] < -1e-10 || heatDirection * next[cell] > Math.Abs(wallOffset) + 1e-10)
                    throw new InvalidOperationException("Planar phase change produced an invalid cell temperature.");
            }

            Array.Copy(next, temperature, n);
            FrontM = nextFront;
            HeatInputJPerM2 += wallFlux * dt;
            return dt;
        }

        private static double Positive(double value)
        {
            if (!SiUnits.IsFinite(value) || value <= 0) throw new ArgumentOutOfRangeException(nameof(value));
            return value;
        }
    }
}
