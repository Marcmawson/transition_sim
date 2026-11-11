using System;
using System.Linq;
using PhysicsSim.ThreePhase3D.Verification;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;
using PhysicsSim.ThreePhase3D.Geometry;
using PhysicsSim.ThreePhase3D.PhaseChange;
using PhysicsSim.ThreePhase3D.Materials;

internal static class SurfaceGeometryMain
{
    private static void Main()
    {
        var grid = new Grid3D(15, 15, 15, new D3(.012, .008, .01));
        var transport = new GeometricVof3D(grid, 997, 1.2);
        var alpha = new double[transport.CellCount];
        int centre = transport.CellIndex(7, 7, 7);
        const double a = 3, b = 5;
        for (int curved = 0; curved <= 1; curved++)
        {
            for (int k = 0; k < 15; k++)
            for (int j = 0; j < 15; j++)
            for (int i = 0; i < 15; i++)
            {
                double x = (i - 7) * grid.Spacing.X, y = (j - 7) * grid.Spacing.Y;
                // Cell-area average of a quadratic height, exact across the
                // central 3x3 columns where the graph stays within one z cell.
                double height = .25 * grid.Spacing.Z - curved *
                    (a * (x * x + grid.Spacing.X * grid.Spacing.X / 12) +
                     b * (y * y + grid.Spacing.Y * grid.Spacing.Y / 12));
                alpha[transport.CellIndex(i, j, k)] =
                    Math.Max(0, Math.Min(1, height / grid.Spacing.Z - (k - 7) + .5));
            }
            Check(SurfaceGeometry.Evaluate(transport, alpha)[centre], 2 * curved * (a + b));
            for (int cell = 0; cell < alpha.Length; cell++) alpha[cell] = 1 - alpha[cell];
            Check(SurfaceGeometry.Evaluate(transport, alpha)[centre], -2 * curved * (a + b));
        }

        Array.Clear(alpha, 0, alpha.Length);
        alpha[centre] = .5;
        var unresolved = SurfaceGeometry.Evaluate(transport, alpha);
        if (unresolved[centre].Support != CurvatureSupport.Unresolved ||
            !double.IsNaN(unresolved[centre].InverseMetres) ||
            unresolved[0].Support != CurvatureSupport.PurePhase)
            throw new InvalidOperationException("Incorrect support diagnostic for an isolated mixed cell.");
        Console.WriteLine("Isolated mixed cell: unresolved; pure cell: pure phase.");
        CheckAlternateAxis();
        CheckSphericalDrop();
        CheckThermocapillarySlab();
        CheckWetting();
    }

    private static void CheckSphericalDrop()
    {
        const int n = 24, samples = 12;
        const double h = .001, radius = .007, sigma = .072, dt = .0001;
        var grid = new Grid3D(n, n, n, new D3(h, h, h));
        var transport = new GeometricVof3D(grid, 997, 1.2);
        var alpha = new double[transport.CellCount];
        for (int k = 0; k < n; k++)
        for (int j = 0; j < n; j++)
        for (int i = 0; i < n; i++)
        {
            int inside = 0;
            for (int z = 0; z < samples; z++)
            for (int y = 0; y < samples; y++)
            for (int x = 0; x < samples; x++)
            {
                double dx = (i + (x + .5) / samples - n / 2.0) * h;
                double dy = (j + (y + .5) / samples - n / 2.0) * h;
                double dz = (k + (z + .5) / samples - n / 2.0) * h;
                if (dx * dx + dy * dy + dz * dz < radius * radius) inside++;
            }
            alpha[transport.CellIndex(i, j, k)] = inside / (double)(samples * samples * samples);
        }

        var flow = new PeriodicVofProjection(grid, 997, 1.2);
        var state = VofChecks.Fields(grid, alpha, VofChecks.Uniform(alpha.Length, 0, 0, 0));
        var baseline = flow.Advance(state, dt);
        var disabled = flow.AdvanceCapillary(state, dt, 0);
        if (!baseline.CellPressurePa.SequenceEqual(disabled.CellPressurePa) ||
            !baseline.Fields.FaceVelocity.SelectMany(axis => axis)
                .SequenceEqual(disabled.Fields.FaceVelocity.SelectMany(axis => axis)))
            throw new InvalidOperationException("Zero surface tension changed the S05 projection.");
        Console.WriteLine("Zero surface tension: identical to the S05 projection.");

        double expected = 2 * sigma / radius;
        double initialVolume = alpha.Sum() * grid.CellVolume;
        for (int iteration = 1; iteration <= 3; iteration++)
        {
            var step = flow.AdvanceCapillary(state, dt, sigma, 16384);
            double jump = step.CellPressurePa[transport.CellIndex(n / 2, n / 2, n / 2)] - step.CellPressurePa[0];
            double speed = step.Fields.FaceVelocity.Max(axis => axis.Max(value => Math.Abs(value)));
            double drift = step.Fields.LiquidFraction.Sum() * grid.CellVolume - initialVolume;
            Console.WriteLine($"Drop step {iteration}: jump={jump:G8} Pa (expected {expected:G8}), max face speed={speed:G8} m/s, volume drift={drift:G8} m^3, extrapolated faces={step.ExtrapolatedCurvatureFaces}");
            if (!step.Pressure.Converged || !(Math.Abs(jump / expected - 1) <= .05) ||
                !(speed <= .0002) || !(Math.Abs(drift) <= initialVolume * 1e-10))
                throw new InvalidOperationException("Stationary sphere capillary check failed.");
            state = step.Fields;
        }
    }

    private static void CheckThermocapillarySlab()
    {
        const int n = 20;
        const double h = .001, dt = .0001, meanTemperature = 310, amplitude = 10;
        var grid = new Grid3D(n, n, 3, new D3(h, h, h));
        var transport = new GeometricVof3D(grid, 997, 1.2);
        var alpha = new double[transport.CellCount];
        var temperature = new double[alpha.Length];
        var sigma = new double[alpha.Length];
        var normalSigma = new double[alpha.Length];
        for (int k = 0; k < 3; k++)
        for (int j = 0; j < n; j++)
        for (int i = 0; i < n; i++)
        {
            int cell = transport.CellIndex(i, j, k);
            alpha[cell] = j == 5 || j == 14 ? .5 : j > 5 && j < 14 ? 1 : 0;
            temperature[cell] = meanTemperature + amplitude * Math.Sin(2 * Math.PI * i / n);
            sigma[cell] = WaterTransport.SurfaceTension(temperature[cell]);
            normalSigma[cell] = WaterTransport.SurfaceTension(meanTemperature + .5 * j);
        }

        var force = MarangoniStress.ForceDensity(transport, alpha, sigma);
        var normalForce = MarangoniStress.ForceDensity(transport, alpha, normalSigma);
        if (normalForce.Any(f => f.Length > 1e-10) || force.Any(f => f.Y != 0 || f.Z != 0))
            throw new InvalidOperationException("Marangoni force must be tangential.");
        double stress = 0;
        for (int j = 0; j < n / 2; j++) stress += h * force[transport.CellIndex(0, j, 0)].X;
        double expected = WaterTransport.SurfaceTensionDerivative(meanTemperature) * amplitude * 2 * Math.PI / (n * h);
        double netForce = force.Sum(f => f.X) * grid.CellVolume;

        var state = VofChecks.Fields(grid, alpha, VofChecks.Uniform(alpha.Length, 0, 0, 0));
        var solver = new PeriodicVofProjection(grid, 997, 1.2);
        var step = solver.AdvanceThermocapillary(state, dt, temperature);
        double coldwardSpeed = -step.Fields.FaceVelocity[0][transport.CellIndex(0, 4, 0)];
        var uniform = solver.AdvanceThermocapillary(state, dt,
            Enumerable.Repeat(meanTemperature, alpha.Length).ToArray());
        var isothermal = solver.AdvanceCapillary(state, dt, WaterTransport.SurfaceTension(meanTemperature));
        if (!uniform.CellPressurePa.SequenceEqual(isothermal.CellPressurePa) ||
            !uniform.Fields.FaceVelocity.SelectMany(axis => axis)
                .SequenceEqual(isothermal.Fields.FaceVelocity.SelectMany(axis => axis)))
            throw new InvalidOperationException("Uniform temperature changed the isothermal result.");

        Console.WriteLine($"Thermocapillary slab: stress={stress:G8} Pa (expected {expected:G8}), net force={netForce:G8} N, coldward speed={coldwardSpeed:G8} m/s");
        if (!(Math.Abs(stress / expected - 1) < .03) || !(Math.Abs(netForce) < 1e-16) ||
            !(coldwardSpeed > 0) || !step.Pressure.Converged)
            throw new InvalidOperationException("Thermocapillary slab check failed.");
    }

    private static void CheckWetting()
    {
        var profile = new WettingProfile("Synthetic flat wall", "Analytical test fixture", 65, .001, .01,
            new[] { new WettingMeasurement(0, 80), new WettingMeasurement(.02, 100) },
            new[] { new WettingMeasurement(0, 50), new WettingMeasurement(.02, 30) });
        if (profile.AngleDegrees(0, 70) != 70 || profile.AngleDegrees(0, 90) != 80 ||
            profile.AngleDegrees(.01, 65) != 90 || profile.AngleDegrees(-.01, 65) != 40)
            throw new InvalidOperationException("Wetting branches or hysteresis limits failed.");
        try
        {
            profile.AngleDegrees(.03, 65);
            throw new InvalidOperationException("An uncalibrated contact-line speed was accepted.");
        }
        catch (ArgumentOutOfRangeException) { }

        var grid = new Grid3D(4, 1, 1, new D3(1, 1, 1));
        var slab = SurfacePrimitives.Box(new D3(-1, -1, -1), new D3(5, .4, 2));
        var geometry = CutCellGeometry.Build(grid, slab);
        int cell = grid.Cells.Index(1, 0, 0);
        var patch = geometry.Patches.First(p => p.Cell == cell && p.FluidExposedAreaM2 > 0);
        var contact = new WettingContact(profile, patch, -.01);
        var normal = contact.InterfaceNormal(new D3(-1, 0, 0));
        double angle = Math.Acos(D3.Dot(normal, patch.NormalLocal)) * 180 / Math.PI;
        var traction = profile.WallTractionPa(new D3(2, 7, 0), patch.NormalLocal, .004, .01);
        if (Math.Abs(angle - 40) > 1e-12 || (traction - new D3(-4, 0, 0)).Length > 1e-12)
            throw new InvalidOperationException("Contact-angle orientation or Navier traction failed.");

        var open = new FlowBoundary(FlowBoundaryKind.OpenPressure);
        var wall = new FlowBoundary(FlowBoundaryKind.NoSlip);
        var plan = new VariableCoefficientProjection(geometry,
            new BoundaryConditions3D(open, open, wall, wall, wall, wall), _ => 997, _ => 0);
        var liquid = plan.Volumes.Select((v, i) => v * (.2 + .15 * i)).ToArray();
        var gas = plan.Volumes.Zip(liquid, (volume, water) => volume - water).ToArray();
        var state = new CutCellInventories(liquid, gas, new double[gas.Length],
            new double[gas.Length], new double[gas.Length]);
        var velocity = new double[plan.FaceCount];
        for (int i = 0; i <= 4; i++)
        {
            int f = plan.FaceIndex(0, i, 0, 0);
            velocity[f] = .06 / plan.Faces[f].Area;
        }
        var transport = new CutCellVofTransport(plan, donor => donor == cell ? contact : null);
        var result = transport.Advance(state, velocity, 1, _ => new CutCellInflow(0, 0, 0, 0));
        var clipped = new CutCellVofGeometry(geometry);
        int group = plan.GroupByCell[cell];
        double target = liquid[group] / plan.Volumes[group] * geometry.FluidVolumeM3[cell];
        var plane = clipped.Reconstruct(cell, normal, target);
        double swept = clipped.Sweep(cell, 0, true, .06, plane);
        double ledger = result.Fields.LiquidM3.Sum() - liquid.Sum() - result.BoundaryInflow[0];
        if (Math.Abs(clipped.LiquidVolume(cell, plane) - target) > 1e-10 ||
            Math.Abs(result.LiquidFaceM3[plan.FaceIndex(0, 2, 0, 0)] - swept) > 1e-10 ||
            Math.Abs(ledger) > 1e-10)
            throw new InvalidOperationException("Wetting reconstruction or transport ledger failed.");
        var tracked = CutCellVofTransport.WithTrackedWetting(plan, profile);
        var slow = velocity.Select(value => value / 10).ToArray();
        var detected = tracked.TrackContact(cell, group, liquid[group], new D3(-1, 0, 0), slow);
        var separated = tracked.TrackContact(cell, group, liquid[group], new D3(0, -1, 0), slow);
        if (detected == null || Math.Abs(detected.SignedSpeedMPerS + .01) > 1e-8 || separated != null)
            throw new InvalidOperationException("Contact tracking must distinguish an intersecting plane from a separated interface.");
        var autoResult = tracked.Advance(state, slow, 10, _ => new CutCellInflow(0, 0, 0, 0));
        int outlet = plan.FaceIndex(0, 2, 0, 0);
        if (Math.Abs(autoResult.LiquidFaceM3[outlet] - result.LiquidFaceM3[outlet]) > 1e-8)
            throw new InvalidOperationException("Tracked wetting did not reproduce the prescribed contact flux.");
        Console.WriteLine($"Tracked wetting: receding speed={detected.SignedSpeedMPerS:G8} m/s; separated interface excluded.");
        Console.WriteLine($"Synthetic wetting: receding angle={angle:G8}°, traction={traction.X:G8} Pa, liquid ledger={ledger:G8} m^3");
    }

    private static void CheckAlternateAxis()
    {
        // The z axis has the strongest fraction gradient but has only five
        // cells. The same inclined plane is a valid height function along x.
        var grid = new Grid3D(11, 5, 5, new D3(.01, .01, .01));
        var transport = new GeometricVof3D(grid, 997, 1.2);
        var alpha = new double[transport.CellCount];
        for (int k = 0; k < 5; k++)
        for (int j = 0; j < 5; j++)
        for (int i = 0; i < 11; i++)
        {
            double offset = 1.5 * (k - 2) - (i - 5) + .5;
            alpha[transport.CellIndex(i, j, k)] =
                (ClampedIntegral(offset + .75) - ClampedIntegral(offset - .75)) / 1.5;
        }
        var sample = SurfaceGeometry.Evaluate(transport, alpha)[transport.CellIndex(5, 2, 2)];
        if (sample.Support != CurvatureSupport.AlternateAxis || !(Math.Abs(sample.InverseMetres) <= 1e-10))
            throw new InvalidOperationException("The inclined plane should use the alternate axis and have zero curvature.");
        Console.WriteLine("Inclined plane: alternate axis; zero curvature.");
    }

    private static double ClampedIntegral(double x)
    {
        if (x <= 0) return 0;
        if (x >= 1) return x - .5;
        return .5 * x * x;
    }

    private static void Check(SurfaceCurvature actual, double expected)
    {
        if (actual.Support != CurvatureSupport.HeightFunction ||
            !(Math.Abs(actual.InverseMetres - expected) <= 1e-10))
            throw new InvalidOperationException($"Expected curvature {expected:R}, got {actual.InverseMetres:R} ({actual.Support}).");
        Console.WriteLine($"Curvature {actual.InverseMetres:G8} 1/m (expected {expected:G8})");
    }
}
