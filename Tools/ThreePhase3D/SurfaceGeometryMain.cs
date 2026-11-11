using System;
using System.Linq;
using PhysicsSim.ThreePhase3D.Verification;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;
using PhysicsSim.ThreePhase3D.PhaseChange;

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
