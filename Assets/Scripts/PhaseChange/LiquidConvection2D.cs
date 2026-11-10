using System;

namespace PhaseTransition.Physics
{
    /// <summary>
    /// First liquid-flow stage for the beaker: 2-D incompressible Navier-Stokes with
    /// temperature-dependent water density (Boussinesq buoyancy), viscosity, advection,
    /// and a pressure projection. Solid cells are treated as stationary obstacles.
    /// Vapour/compressible gas flow and solid-body motion are not handled here.
    /// </summary>
    public sealed class LiquidConvection2D
    {
        private readonly int width;
        private readonly int height;
        private readonly double[,] u;
        private readonly double[,] v;
        private readonly double[,] uNext;
        private readonly double[,] vNext;
        private readonly double[,] pressure;
        private readonly double[,] pressureNext;
        private readonly double[,] density;

        private const double GravityMPerS2 = 9.80665;
        private const double ReferenceDensityKgPerM3 = 999.972; // liquid water near 4 C, 1 atm
        private const double MinimumFlowFraction = 0.02;
        private const int PressureIterations = 120;

        public LiquidConvection2D(int width, int height)
        {
            this.width = width;
            this.height = height;
            u = new double[width, height];
            v = new double[width, height];
            uNext = new double[width, height];
            vNext = new double[width, height];
            pressure = new double[width, height];
            pressureNext = new double[width, height];
            density = new double[width, height];
        }

        public double GetVelocityX(int x, int y) => u[x, y];
        public double GetVelocityY(int x, int y) => v[x, y];

        public void Reset()
        {
            Array.Clear(u, 0, u.Length);
            Array.Clear(v, 0, v.Length);
            Array.Clear(uNext, 0, uNext.Length);
            Array.Clear(vNext, 0, vNext.Length);
            Array.Clear(pressure, 0, pressure.Length);
            Array.Clear(pressureNext, 0, pressureNext.Length);
            for (int y = 0; y < height; y++)
                for (int x = 0; x < width; x++)
                    density[x, y] = ReferenceDensityKgPerM3;
        }

        public void Advance(double[,] liquidFraction, double[,] temperatureC,
            double dxM, double dyM, double dtS)
        {
            AdvectVelocity(liquidFraction, dxM, dyM, dtS);
            ApplyBuoyancyAndViscosity(liquidFraction, temperatureC, dxM, dyM, dtS);
            Project(liquidFraction, dxM, dyM, dtS);
            EnforceSolidBoundaries(liquidFraction);
        }

        private void AdvectVelocity(double[,] liquidFraction, double dx, double dy, double dt)
        {
            for (int y = 0; y < height; y++)
                for (int x = 0; x < width; x++)
                {
                    if (!IsFluid(liquidFraction, x, y))
                    {
                        uNext[x, y] = 0.0;
                        vNext[x, y] = 0.0;
                        continue;
                    }

                    double backX = x - u[x, y] * dt / dx;
                    double backY = y - v[x, y] * dt / dy;
                    uNext[x, y] = Sample(u, liquidFraction, backX, backY, x, y);
                    vNext[x, y] = Sample(v, liquidFraction, backX, backY, x, y);
                }
            Copy(uNext, u);
            Copy(vNext, v);
        }

        private void ApplyBuoyancyAndViscosity(double[,] liquidFraction,
            double[,] temperatureC, double dx, double dy, double dt)
        {
            for (int y = 0; y < height; y++)
                for (int x = 0; x < width; x++)
                {
                    if (!IsFluid(liquidFraction, x, y))
                    {
                        density[x, y] = ReferenceDensityKgPerM3;
                        continue;
                    }

                    double t = Clamp(temperatureC[x, y], 0.0, 100.0);
                    density[x, y] = LiquidWaterDensityKgPerM3(t);
                    double buoyancy = GravityMPerS2 *
                        (ReferenceDensityKgPerM3 - density[x, y]) / ReferenceDensityKgPerM3;
                    v[x, y] += buoyancy * dt * liquidFraction[x, y];
                }

            // Viscous momentum diffusion: nu = dynamic viscosity / density.
            // The Andrade/Vogel fit follows water's strong viscosity decrease with temperature.
            for (int y = 0; y < height; y++)
                for (int x = 0; x < width; x++)
                {
                    if (!IsFluid(liquidFraction, x, y))
                    {
                        uNext[x, y] = 0.0;
                        vNext[x, y] = 0.0;
                        continue;
                    }

                    double t = Clamp(temperatureC[x, y], 0.0, 100.0) + 273.15;
                    double dynamicViscosity = 2.414e-5 * Math.Pow(10.0, 247.8 / (t - 140.0));
                    double nu = dynamicViscosity / density[x, y];
                    double lapU = Laplacian(u, liquidFraction, x, y, dx, dy);
                    double lapV = Laplacian(v, liquidFraction, x, y, dx, dy);
                    uNext[x, y] = u[x, y] + nu * dt * lapU;
                    vNext[x, y] = v[x, y] + nu * dt * lapV;
                }
            Copy(uNext, u);
            Copy(vNext, v);
        }

        private void Project(double[,] liquidFraction, double dx, double dy, double dt)
        {
            double invDx2 = 1.0 / (dx * dx);
            double invDy2 = 1.0 / (dy * dy);
            for (int y = 0; y < height; y++)
                for (int x = 0; x < width; x++)
                    if (IsFluid(liquidFraction, x, y))
                        pressure[x, y] = 0.0;

            for (int iteration = 0; iteration < PressureIterations; iteration++)
            {
                for (int y = 0; y < height; y++)
                    for (int x = 0; x < width; x++)
                    {
                        if (!IsFluid(liquidFraction, x, y))
                        {
                            pressureNext[x, y] = 0.0;
                            continue;
                        }

                        double pL = PressureAt(liquidFraction, x - 1, y, x, y);
                        double pR = PressureAt(liquidFraction, x + 1, y, x, y);
                        double pD = PressureAt(liquidFraction, x, y - 1, x, y);
                        double pU = PressureAt(liquidFraction, x, y + 1, x, y);
                        double divergence = (VelocityUAt(liquidFraction, x + 1, y) -
                                             VelocityUAt(liquidFraction, x - 1, y)) / (2.0 * dx) +
                                            (VelocityVAt(liquidFraction, x, y + 1) -
                                             VelocityVAt(liquidFraction, x, y - 1)) / (2.0 * dy);
                        pressureNext[x, y] = ((pL + pR) * invDx2 + (pD + pU) * invDy2 -
                            ReferenceDensityKgPerM3 * divergence / dt) /
                            (2.0 * invDx2 + 2.0 * invDy2);
                    }
                Copy(pressureNext, pressure);
            }

            for (int y = 0; y < height; y++)
                for (int x = 0; x < width; x++)
                {
                    if (!IsFluid(liquidFraction, x, y))
                    {
                        u[x, y] = 0.0;
                        v[x, y] = 0.0;
                        continue;
                    }
                    u[x, y] -= dt / ReferenceDensityKgPerM3 *
                        (PressureAt(liquidFraction, x + 1, y, x, y) - PressureAt(liquidFraction, x - 1, y, x, y)) / (2.0 * dx);
                    v[x, y] -= dt / ReferenceDensityKgPerM3 *
                        (PressureAt(liquidFraction, x, y + 1, x, y) - PressureAt(liquidFraction, x, y - 1, x, y)) / (2.0 * dy);
                }
        }

        private void EnforceSolidBoundaries(double[,] liquidFraction)
        {
            for (int y = 0; y < height; y++)
                for (int x = 0; x < width; x++)
                {
                    if (!IsFluid(liquidFraction, x, y))
                    {
                        u[x, y] = 0.0;
                        v[x, y] = 0.0;
                        continue;
                    }

                    if (x == 0 || x == width - 1 || !IsFluid(liquidFraction, x - 1, y) || !IsFluid(liquidFraction, x + 1, y))
                        u[x, y] = 0.0;
                    if (y == 0 || y == height - 1 || !IsFluid(liquidFraction, x, y - 1) || !IsFluid(liquidFraction, x, y + 1))
                        v[x, y] = 0.0;
                }
        }

        private double Laplacian(double[,] field, double[,] liquidFraction,
            int x, int y, double dx, double dy)
        {
            double centre = field[x, y];
            double left = IsFluid(liquidFraction, x - 1, y) ? field[x - 1, y] : centre;
            double right = IsFluid(liquidFraction, x + 1, y) ? field[x + 1, y] : centre;
            double down = IsFluid(liquidFraction, x, y - 1) ? field[x, y - 1] : centre;
            double up = IsFluid(liquidFraction, x, y + 1) ? field[x, y + 1] : centre;
            return (left - 2.0 * centre + right) / (dx * dx) +
                   (down - 2.0 * centre + up) / (dy * dy);
        }

        private double Sample(double[,] field, double[,] liquidFraction,
            double x, double y, int fallbackX, int fallbackY)
        {
            x = Clamp(x, 0.0, width - 1.0);
            y = Clamp(y, 0.0, height - 1.0);
            int x0 = (int)x, y0 = (int)y;
            int x1 = Math.Min(width - 1, x0 + 1), y1 = Math.Min(height - 1, y0 + 1);
            double tx = x - x0, ty = y - y0;
            double a = ValidSample(field, liquidFraction, x0, y0, fallbackX, fallbackY);
            double b = ValidSample(field, liquidFraction, x1, y0, fallbackX, fallbackY);
            double c = ValidSample(field, liquidFraction, x0, y1, fallbackX, fallbackY);
            double d = ValidSample(field, liquidFraction, x1, y1, fallbackX, fallbackY);
            return (a * (1.0 - tx) + b * tx) * (1.0 - ty) +
                   (c * (1.0 - tx) + d * tx) * ty;
        }

        private double ValidSample(double[,] field, double[,] liquidFraction,
            int x, int y, int fallbackX, int fallbackY) =>
            IsFluid(liquidFraction, x, y) ? field[x, y] : field[fallbackX, fallbackY];

        private double PressureAt(double[,] liquidFraction, int x, int y, int fallbackX, int fallbackY) =>
            IsFluid(liquidFraction, x, y) ? pressure[x, y] : pressure[fallbackX, fallbackY];

        private double VelocityUAt(double[,] liquidFraction, int x, int y) =>
            IsFluid(liquidFraction, x, y) ? u[x, y] : 0.0;

        private double VelocityVAt(double[,] liquidFraction, int x, int y) =>
            IsFluid(liquidFraction, x, y) ? v[x, y] : 0.0;

        private bool IsFluid(double[,] liquidFraction, int x, int y) =>
            x >= 0 && y >= 0 && x < width && y < height && liquidFraction[x, y] >= MinimumFlowFraction;

        private static double LiquidWaterDensityKgPerM3(double temperatureC)
        {
            // Kell correlation for atmospheric-pressure liquid water, 0 to 100 C.
            double delta = temperatureC - 3.9863;
            return 999.972 * (1.0 - ((temperatureC + 288.9414) /
                (508929.2 * (temperatureC + 68.12963))) * delta * delta);
        }

        private static double Clamp(double value, double min, double max) =>
            value < min ? min : value > max ? max : value;

        private static void Copy(double[,] source, double[,] destination)
        {
            for (int y = 0; y < source.GetLength(1); y++)
                for (int x = 0; x < source.GetLength(0); x++)
                    destination[x, y] = source[x, y];
        }
    }
}
