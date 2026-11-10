using System;

namespace PhaseTransition.Physics
{
    /// <summary>
    /// Low-resolution 2-D incompressible liquid solver on a staggered MAC grid.
    /// Pressure is cell-centred; horizontal and vertical velocity components are stored
    /// on their respective cell faces. Empty cells are atmospheric free surface and ice
    /// cells are impermeable obstacles. This remains an educational, coarse-grid model.
    /// </summary>
    public sealed class LiquidConvection2D
    {
        private readonly int width;
        private readonly int height;
        // MAC layout: u is (width + 1) x height, v is width x (height + 1).
        private readonly double[,] u;
        private readonly double[,] v;
        private readonly double[,] uNext;
        private readonly double[,] vNext;
        private readonly double[,] pressure;
        private readonly double[,] density;
        private readonly bool[,] fluid;
        private readonly bool[,] solid;
        private readonly double[,] liquidFractionField;
        private readonly double[,] interfaceNormalX;
        private readonly double[,] interfaceNormalY;
        private readonly double[,] interfaceCurvature;

        private const double GravityMPerS2 = 9.80665;
        private const double ReferenceDensityKgPerM3 = 999.972;
        private const double MinimumFlowFraction = 0.02;
        private const double PressureRelaxation = 1.7;
        private const int PressureIterations = 180;
        private const double InterfaceGradientEpsilon = 1e-8;

        public double MaximumSpeedMS { get; private set; }
        public double MaximumDivergencePerS { get; private set; }

        public LiquidConvection2D(int width, int height)
        {
            this.width = width;
            this.height = height;
            u = new double[width + 1, height];
            v = new double[width, height + 1];
            uNext = new double[width + 1, height];
            vNext = new double[width, height + 1];
            pressure = new double[width, height];
            density = new double[width, height];
            fluid = new bool[width, height];
            solid = new bool[width, height];
            liquidFractionField = new double[width, height];
            interfaceNormalX = new double[width, height];
            interfaceNormalY = new double[width, height];
            interfaceCurvature = new double[width, height];
            Reset();
        }

        public double GetFaceVelocityX(int faceX, int y) => u[faceX, y];
        public double GetFaceVelocityY(int x, int faceY) => v[x, faceY];
        public double GetVelocityX(int x, int y) => 0.5 * (u[x, y] + u[x + 1, y]);
        public double GetVelocityY(int x, int y) => 0.5 * (v[x, y] + v[x, y + 1]);

        public void Reset()
        {
            Array.Clear(u, 0, u.Length);
            Array.Clear(v, 0, v.Length);
            Array.Clear(uNext, 0, uNext.Length);
            Array.Clear(vNext, 0, vNext.Length);
            Array.Clear(pressure, 0, pressure.Length);
            Array.Clear(fluid, 0, fluid.Length);
            Array.Clear(solid, 0, solid.Length);
            for (int y = 0; y < height; y++)
                for (int x = 0; x < width; x++)
                    density[x, y] = ReferenceDensityKgPerM3;
            MaximumSpeedMS = 0.0;
            MaximumDivergencePerS = 0.0;
        }

        public void Advance(double[,] liquidFraction, double[,] iceFraction,
            double[,] temperatureC, double dxM, double dyM, double dtS,
            double surfaceTensionNPerM, double glassContactAngleDegrees)
        {
            BuildMasks(liquidFraction, iceFraction);
            BuildInterfaceGeometry(liquidFraction, dxM, dyM, glassContactAngleDegrees);
            AdvectVelocity(dxM, dyM, dtS);
            ApplyForcesAndViscosity(temperatureC, dxM, dyM, dtS, surfaceTensionNPerM);
            EnforceFaceBoundaries();
            Project(dxM, dyM, dtS);
            EnforceFaceBoundaries();
            MeasureMaximumSpeed(dxM, dyM);
        }

        private void BuildInterfaceGeometry(double[,] liquidFraction, double dx, double dy,
            double contactAngleDegrees)
        {
            // A diffuse interface is represented by the transported liquid volume fraction.
            // Its normalized gradient points from gas toward liquid.
            for (int y = 0; y < height; y++)
                for (int x = 0; x < width; x++)
                {
                    liquidFractionField[x, y] = Clamp(liquidFraction[x, y], 0.0, 1.0);
                    double gx = (FractionAt(liquidFraction, x + 1, y) -
                                 FractionAt(liquidFraction, x - 1, y)) / (2.0 * dx);
                    double gy = (FractionAt(liquidFraction, x, y + 1) -
                                 FractionAt(liquidFraction, x, y - 1)) / (2.0 * dy);
                    double magnitude = System.Math.Sqrt(gx * gx + gy * gy);
                    if (magnitude > InterfaceGradientEpsilon)
                    {
                        interfaceNormalX[x, y] = gx / magnitude;
                        interfaceNormalY[x, y] = gy / magnitude;
                    }
                    else
                    {
                        interfaceNormalX[x, y] = 0.0;
                        interfaceNormalY[x, y] = 0.0;
                    }
                }

            // Impose the specified static contact angle at vertical glass walls. The
            // original vertical normal selects whether the interface rises or falls there.
            double theta = Clamp(contactAngleDegrees, 0.0, 180.0) * System.Math.PI / 180.0;
            double cosTheta = System.Math.Cos(theta);
            double sinTheta = System.Math.Sin(theta);
            for (int y = 0; y < height; y++)
            {
                ApplyWallContactAngle(0, y, 1.0, cosTheta, sinTheta);
                ApplyWallContactAngle(width - 1, y, -1.0, cosTheta, sinTheta);
            }

            // Curvature is minus the divergence of the gas-to-liquid unit normal.
            // Clamp edge samples by replication; wall normals have already been adjusted.
            for (int y = 0; y < height; y++)
                for (int x = 0; x < width; x++)
                {
                    double dNxDx = (NormalXAt(x + 1, y) - NormalXAt(x - 1, y)) / (2.0 * dx);
                    double dNyDy = (NormalYAt(x, y + 1) - NormalYAt(x, y - 1)) / (2.0 * dy);
                    interfaceCurvature[x, y] = -(dNxDx + dNyDy);
                }
        }

        private void ApplyWallContactAngle(int x, int y, double inwardNormalX,
            double cosTheta, double sinTheta)
        {
            double oldY = interfaceNormalY[x, y];
            if (System.Math.Abs(oldY) < 0.1) return;
            interfaceNormalX[x, y] = inwardNormalX * cosTheta;
            interfaceNormalY[x, y] = System.Math.Sign(oldY) * sinTheta;
        }

        private double FractionAt(double[,] fraction, int x, int y)
        {
            if (y >= height) return 0.0; // open top is adjacent to gas
            if (y < 0) y = 0;             // bottom is a solid wall
            x = System.Math.Max(0, System.Math.Min(width - 1, x));
            return Clamp(fraction[x, y], 0.0, 1.0);
        }

        private double NormalXAt(int x, int y)
        {
            x = System.Math.Max(0, System.Math.Min(width - 1, x));
            y = System.Math.Max(0, System.Math.Min(height - 1, y));
            return interfaceNormalX[x, y];
        }

        private double NormalYAt(int x, int y)
        {
            x = System.Math.Max(0, System.Math.Min(width - 1, x));
            y = System.Math.Max(0, System.Math.Min(height - 1, y));
            return interfaceNormalY[x, y];
        }

        private void BuildMasks(double[,] liquidFraction, double[,] iceFraction)
        {
            for (int y = 0; y < height; y++)
                for (int x = 0; x < width; x++)
                {
                    fluid[x, y] = liquidFraction[x, y] >= MinimumFlowFraction;
                    solid[x, y] = !fluid[x, y] && iceFraction[x, y] >= 0.5;
                }
        }

        private void AdvectVelocity(double dx, double dy, double dt)
        {
            Array.Clear(uNext, 0, uNext.Length);
            Array.Clear(vNext, 0, vNext.Length);
            for (int y = 0; y < height; y++)
                for (int x = 1; x < width; x++)
                {
                    if (!IsUFaceActive(x, y)) continue;
                    double px = x * dx;
                    double py = (y + 0.5) * dy;
                    double vx = u[x, y];
                    double vy = SampleV(px, py, dx, dy);
                    uNext[x, y] = SampleU(px - vx * dt, py - vy * dt, dx, dy);
                }

            for (int y = 1; y <= height; y++)
                for (int x = 0; x < width; x++)
                {
                    if (!IsVFaceActive(x, y)) continue;
                    double px = (x + 0.5) * dx;
                    double py = y * dy;
                    double vx = SampleU(px, py, dx, dy);
                    double vy = v[x, y];
                    vNext[x, y] = SampleV(px - vx * dt, py - vy * dt, dx, dy);
                }
            Copy(uNext, u);
            Copy(vNext, v);
        }

        private void ApplyForcesAndViscosity(double[,] temperatureC, double dx, double dy, double dt,
            double surfaceTensionNPerM)
        {
            for (int y = 0; y < height; y++)
                for (int x = 0; x < width; x++)
                {
                    if (fluid[x, y])
                    {
                        double t = Clamp(temperatureC[x, y], 0.0, 100.0);
                        density[x, y] = WaterThermodynamicsSI.LiquidDensityKgPerM3(t);
                    }
                    else density[x, y] = ReferenceDensityKgPerM3;
                }

            // Gravity and Boussinesq density force act on vertical face velocities.
            for (int y = 1; y <= height; y++)
                for (int x = 0; x < width; x++)
                {
                    if (!IsVFaceActive(x, y)) continue;
                    bool belowFluid = y > 0 && fluid[x, y - 1];
                    bool aboveFluid = y < height && fluid[x, y];
                    double rho = belowFluid && aboveFluid
                        ? 0.5 * (density[x, y - 1] + density[x, y])
                        : belowFluid ? density[x, y - 1] : density[x, y];
                    double buoyancy = GravityMPerS2 *
                        (ReferenceDensityKgPerM3 - rho) / ReferenceDensityKgPerM3;
                    v[x, y] += (-GravityMPerS2 + buoyancy) * dt;
                    if (surfaceTensionNPerM > 0.0)
                    {
                        double capillaryAcceleration = surfaceTensionNPerM / rho *
                            AverageCurvatureY(x, y) * (FractionAtCurrent(x, y) - FractionAtCurrent(x, y - 1)) / dy;
                        v[x, y] += capillaryAcceleration * dt;
                    }
                }

            for (int y = 0; y < height; y++)
                for (int x = 1; x < width; x++)
                {
                    if (!IsUFaceActive(x, y) || surfaceTensionNPerM <= 0.0) continue;
                    double rho = AverageFaceDensityX(x, y);
                    double gradAlpha = (FractionAtCurrent(x, y) - FractionAtCurrent(x - 1, y)) / dx;
                    double acceleration = surfaceTensionNPerM / rho * AverageCurvatureX(x, y) * gradAlpha;
                    u[x, y] += acceleration * dt;
                }

            // Explicit viscous diffusion; the caller limits dt against the cell-scale CFL.
            for (int y = 0; y < height; y++)
                for (int x = 1; x < width; x++)
                {
                    if (!IsUFaceActive(x, y)) { uNext[x, y] = 0.0; continue; }
                    double rho = AverageFaceDensityX(x, y);
                    double nu = DynamicViscosity(temperatureC, x, Math.Min(y, height - 1)) / rho;
                    double lap = (UAt(x - 1, y, x, y) - 2.0 * u[x, y] + UAt(x + 1, y, x, y)) / (dx * dx) +
                                 (UAt(x, y - 1, x, y) - 2.0 * u[x, y] + UAt(x, y + 1, x, y)) / (dy * dy);
                    uNext[x, y] = u[x, y] + nu * dt * lap;
                }
            for (int y = 1; y <= height; y++)
                for (int x = 0; x < width; x++)
                {
                    if (!IsVFaceActive(x, y)) { vNext[x, y] = 0.0; continue; }
                    double rho = AverageFaceDensityY(x, y);
                    double nu = DynamicViscosity(temperatureC, x, Math.Min(y, height - 1)) / rho;
                    double lap = (VAt(x - 1, y, x, y) - 2.0 * v[x, y] + VAt(x + 1, y, x, y)) / (dx * dx) +
                                 (VAt(x, y - 1, x, y) - 2.0 * v[x, y] + VAt(x, y + 1, x, y)) / (dy * dy);
                    vNext[x, y] = v[x, y] + nu * dt * lap;
                }
            Copy(uNext, u);
            Copy(vNext, v);
        }

        private double FractionAtCurrent(int x, int y)
        {
            if (x < 0 || x >= width || y < 0 || y >= height) return 0.0;
            return liquidFractionField[x, y];
        }

        private double AverageCurvatureX(int faceX, int y) =>
            0.5 * (interfaceCurvature[faceX - 1, y] + interfaceCurvature[faceX, y]);

        private double AverageCurvatureY(int x, int faceY)
        {
            if (faceY <= 0 || faceY >= height) return 0.0;
            return 0.5 * (interfaceCurvature[x, faceY - 1] + interfaceCurvature[x, faceY]);
        }

        private void Project(double dx, double dy, double dt)
        {
            Array.Clear(pressure, 0, pressure.Length);
            double invDx2 = 1.0 / (dx * dx);
            double invDy2 = 1.0 / (dy * dy);

            // Red-black SOR solves the same finite-volume Laplacian used by the face
            // divergence and face pressure-gradient correction below.
            for (int iteration = 0; iteration < PressureIterations; iteration++)
                for (int colour = 0; colour < 2; colour++)
                    for (int y = 0; y < height; y++)
                        for (int x = 0; x < width; x++)
                        {
                            if (!fluid[x, y] || ((x + y) & 1) != colour) continue;
                            double sum = 0.0, diagonal = 0.0;
                            AddPressureNeighbour(x - 1, y, invDx2, ref sum, ref diagonal);
                            AddPressureNeighbour(x + 1, y, invDx2, ref sum, ref diagonal);
                            AddPressureNeighbour(x, y - 1, invDy2, ref sum, ref diagonal);
                            AddPressureNeighbour(x, y + 1, invDy2, ref sum, ref diagonal);
                            if (diagonal <= 0.0) continue;

                            double divergence = (u[x + 1, y] - u[x, y]) / dx +
                                                (v[x, y + 1] - v[x, y]) / dy;
                            double rhs = ReferenceDensityKgPerM3 * divergence / dt;
                            double solved = (sum - rhs) / diagonal;
                            pressure[x, y] += PressureRelaxation * (solved - pressure[x, y]);
                        }

            for (int y = 0; y < height; y++)
                for (int x = 1; x < width; x++)
                {
                    if (!IsUFaceActive(x, y)) { u[x, y] = 0.0; continue; }
                    double pLeft = fluid[x - 1, y] ? pressure[x - 1, y] : 0.0;
                    double pRight = fluid[x, y] ? pressure[x, y] : 0.0;
                    u[x, y] -= dt / ReferenceDensityKgPerM3 * (pRight - pLeft) / dx;
                }
            for (int y = 1; y < height; y++)
                for (int x = 0; x < width; x++)
                {
                    if (!IsVFaceActive(x, y)) { v[x, y] = 0.0; continue; }
                    double pBelow = fluid[x, y - 1] ? pressure[x, y - 1] : 0.0;
                    double pAbove = fluid[x, y] ? pressure[x, y] : 0.0;
                    v[x, y] -= dt / ReferenceDensityKgPerM3 * (pAbove - pBelow) / dy;
                }

            // The top boundary is open to the atmosphere.
            for (int x = 0; x < width; x++)
                if (fluid[x, height - 1] && !solid[x, height - 1])
                    v[x, height] -= dt / ReferenceDensityKgPerM3 *
                        (0.0 - pressure[x, height - 1]) / dy;
        }

        private void AddPressureNeighbour(int x, int y, double inverseSpacingSquared,
            ref double sum, ref double diagonal)
        {
            if (x < 0 || x >= width || y < 0) return; // solid vessel wall: Neumann pressure
            if (y >= height)
            {
                diagonal += inverseSpacingSquared; // open top: atmospheric pressure, zero gauge
                return;
            }
            if (solid[x, y]) return;
            diagonal += inverseSpacingSquared;
            if (fluid[x, y]) sum += pressure[x, y]; // air neighbour contributes p = 0
        }

        private void EnforceFaceBoundaries()
        {
            for (int y = 0; y < height; y++)
            {
                u[0, y] = 0.0;
                u[width, y] = 0.0;
                for (int x = 1; x < width; x++)
                    if (!IsUFaceActive(x, y)) u[x, y] = 0.0;
            }
            for (int x = 0; x < width; x++)
            {
                v[x, 0] = 0.0;
                for (int y = 1; y < height; y++)
                    if (!IsVFaceActive(x, y)) v[x, y] = 0.0;
                if (!fluid[x, height - 1]) v[x, height] = 0.0;
            }

        }

        private bool IsUFaceActive(int x, int y)
        {
            if (x <= 0 || x >= width || y < 0 || y >= height) return false;
            if (solid[x - 1, y] || solid[x, y]) return false;
            return fluid[x - 1, y] || fluid[x, y];
        }

        private bool IsVFaceActive(int x, int y)
        {
            if (x < 0 || x >= width || y <= 0 || y > height) return false;
            if (y == height) return fluid[x, height - 1] && !solid[x, height - 1];
            if (solid[x, y - 1] || solid[x, y]) return false;
            return fluid[x, y - 1] || fluid[x, y];
        }

        private double SampleU(double px, double py, double dx, double dy)
        {
            double gx = Clamp(px / dx, 0.0, width);
            double gy = Clamp(py / dy - 0.5, 0.0, height - 1.0);
            return Bilinear(u, gx, gy);
        }

        private double SampleV(double px, double py, double dx, double dy)
        {
            double gx = Clamp(px / dx - 0.5, 0.0, width - 1.0);
            double gy = Clamp(py / dy, 0.0, height);
            return Bilinear(v, gx, gy);
        }

        private static double Bilinear(double[,] field, double x, double y)
        {
            int x0 = (int)x, y0 = (int)y;
            int x1 = Math.Min(field.GetLength(0) - 1, x0 + 1);
            int y1 = Math.Min(field.GetLength(1) - 1, y0 + 1);
            double tx = x - x0, ty = y - y0;
            double a = field[x0, y0] * (1.0 - tx) + field[x1, y0] * tx;
            double b = field[x0, y1] * (1.0 - tx) + field[x1, y1] * tx;
            return a * (1.0 - ty) + b * ty;
        }

        private double UAt(int x, int y, int fallbackX, int fallbackY)
        {
            // Tangential velocity is half a cell above the bottom. Its reflected ghost
            // gives zero velocity at the wall without zeroing an interior face after projection.
            if (y < 0) return -u[fallbackX, fallbackY];
            if (x == 0 || x == width) return 0.0;
            return x > 0 && x < width && y < height && IsUFaceActive(x, y)
                ? u[x, y] : u[fallbackX, fallbackY];
        }

        private double VAt(int x, int y, int fallbackX, int fallbackY)
        {
            // The same reflection enforces no-slip tangential velocity at the side walls.
            if (x < 0 || x >= width) return -v[fallbackX, fallbackY];
            if (y == 0) return 0.0;
            return y > 0 && y <= height && IsVFaceActive(x, y)
                ? v[x, y] : v[fallbackX, fallbackY];
        }

        private double AverageFaceDensityX(int faceX, int y)
        {
            bool left = fluid[faceX - 1, y], right = fluid[faceX, y];
            if (left && right) return 0.5 * (density[faceX - 1, y] + density[faceX, y]);
            return left ? density[faceX - 1, y] : density[faceX, y];
        }

        private double AverageFaceDensityY(int x, int faceY)
        {
            bool below = fluid[x, faceY - 1], above = fluid[x, faceY];
            if (below && above) return 0.5 * (density[x, faceY - 1] + density[x, faceY]);
            return below ? density[x, faceY - 1] : density[x, faceY];
        }

        private static double DynamicViscosity(double[,] temperatureC, int x, int y)
        {
            double t = Clamp(temperatureC[x, y], 0.0, 100.0) + 273.15;
            return 2.414e-5 * Math.Pow(10.0, 247.8 / (t - 140.0));
        }

        private void MeasureMaximumSpeed(double dx, double dy)
        {
            MaximumSpeedMS = 0.0;
            MaximumDivergencePerS = 0.0;
            for (int y = 0; y < height; y++)
                for (int x = 0; x < width; x++)
                {
                    if (!fluid[x, y]) continue;
                    double vx = GetVelocityX(x, y);
                    double vy = GetVelocityY(x, y);
                    double speed = Math.Sqrt(vx * vx + vy * vy);
                    if (speed > MaximumSpeedMS) MaximumSpeedMS = speed;
                    double divergence = (u[x + 1, y] - u[x, y]) / dx +
                                        (v[x, y + 1] - v[x, y]) / dy;
                    double absoluteDivergence = Math.Abs(divergence);
                    if (absoluteDivergence > MaximumDivergencePerS)
                        MaximumDivergencePerS = absoluteDivergence;
                }
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
