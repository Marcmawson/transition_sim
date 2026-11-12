using System;
using System.Collections.Generic;
using PhysicsSim.ThreePhase3D.Core;

namespace PhysicsSim.ThreePhase3D.PhaseChange
{
    // One-phase planar Stefan melting with constant, equal phase densities.
    // The moving cells span only the conducting liquid; the ice stays at Tm.
    public sealed class PlanarStefanMelting
    {
        private readonly double density, heatCapacity, conductivity, latentHeat, wallExcess;
        private readonly double[] temperature, next, flux;
        public IReadOnlyList<double> LiquidExcessK { get; }
        public double FrontM { get; private set; }
        public double HeatInputJPerM2 { get; private set; }

        public PlanarStefanMelting(double densityKgM3, double heatCapacityJkgK, double conductivityWmK,
            double latentHeatJkg, double wallExcessK, double seedFrontM, double[] seedExcessK)
        {
            density = Positive(densityKgM3);
            heatCapacity = Positive(heatCapacityJkgK);
            conductivity = Positive(conductivityWmK);
            latentHeat = Positive(latentHeatJkg);
            wallExcess = Positive(wallExcessK);
            FrontM = Positive(seedFrontM);
            if (seedExcessK == null || seedExcessK.Length < 3)
                throw new ArgumentException("At least three seeded liquid cells are required.", nameof(seedExcessK));
            temperature = (double[])seedExcessK.Clone();
            foreach (double value in temperature)
                if (!SiUnits.IsFinite(value) || value < 0 || value > wallExcess)
                    throw new ArgumentOutOfRangeException(nameof(seedExcessK));
            next = new double[temperature.Length];
            flux = new double[temperature.Length + 1];
            LiquidExcessK = Array.AsReadOnly(temperature);
        }

        public double EnergyJPerM2
        {
            get
            {
                double sum = 0;
                foreach (double value in temperature) sum += value;
                return density * FrontM * (latentHeat + heatCapacity * sum / temperature.Length);
            }
        }

        public double Advance(double requestedSeconds)
        {
            Positive(requestedSeconds);
            int n = temperature.Length;
            double width = FrontM / n;
            // Boundary derivatives reconstructed from the prescribed face
            // temperature and its two nearest cell averages.
            double wallFlux = conductivity * (6 * wallExcess - 7 * temperature[0] + temperature[1]) / (2 * width);
            double frontFlux = conductivity * (7 * temperature[n - 1] - temperature[n - 2]) / (2 * width);
            if (frontFlux < 0) throw new InvalidOperationException("Seed profile drives the front in the wrong direction.");
            double speed = frontFlux / (density * latentHeat);
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
                if (!SiUnits.IsFinite(next[cell]) || next[cell] < -1e-10 || next[cell] > wallExcess + 1e-10)
                    throw new InvalidOperationException("Planar melting produced an invalid cell temperature.");
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
