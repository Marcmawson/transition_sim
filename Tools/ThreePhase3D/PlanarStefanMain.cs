using System;
using PhysicsSim.ThreePhase3D.PhaseChange;

internal static class PlanarStefanMain
{
    private static void Main()
    {
        const double density = 1000, cp = 4200, conductivity = 0.6, latent = 333427, excess = 10;
        const double seedTime = 10, endTime = 100;
        double alpha = conductivity / (density * cp);
        double stefan = cp * excess / latent;
        double lower = 0, upper = 1;
        for (int i = 0; i < 55; i++)
        {
            double mid = (lower + upper) / 2;
            if (Math.Sqrt(Math.PI) * mid * Math.Exp(mid * mid) * Erf(mid) < stefan) lower = mid;
            else upper = mid;
        }
        double lambda = (lower + upper) / 2;
        double Front(double t) => 2 * lambda * Math.Sqrt(alpha * t);
        double Profile(double x, double t) => excess * (1 - Erf(x / (2 * Math.Sqrt(alpha * t))) / Erf(lambda));
        double Average(double left, double width, double time) =>
            (Profile(left, time) + 4 * Profile(left + width / 2, time) + Profile(left + width, time)) / 6;

        double previousFront = double.PositiveInfinity, previousProfile = double.PositiveInfinity;
        foreach (int n in new[] { 16, 32, 64 })
        {
            var seed = new double[n];
            double seedWidth = Front(seedTime) / n;
            for (int i = 0; i < n; i++) seed[i] = Average(i * seedWidth, seedWidth, seedTime);
            var simulation = new PlanarStefanMelting(density, cp, conductivity, latent, excess, Front(seedTime), seed);
            double initial = simulation.EnergyJPerM2;
            for (double t = seedTime; t < endTime - 1e-10;)
                t += simulation.Advance(endTime - t);

            double expectedFront = Front(endTime);
            double frontError = Math.Abs(simulation.FrontM - expectedFront);
            double width = simulation.FrontM / n;
            double profileError = 0;
            for (int i = 0; i < n; i++)
                profileError = Math.Max(profileError, Math.Abs(simulation.LiquidExcessK[i] - Average(i * width, width, endTime)));
            double residual = simulation.EnergyJPerM2 - initial - simulation.HeatInputJPerM2;
            Console.WriteLine(FormattableString.Invariant(
                $"V21 {n} cells: front {simulation.FrontM:G9} m, reference {expectedFront:G9} m, absolute error {frontError:G5} m, max temperature error {profileError:G5} K, energy residual {residual:G5} J/m2."));
            if (frontError > 0.02 * expectedFront || frontError >= previousFront ||
                profileError > 0.2 || profileError >= previousProfile || Math.Abs(residual) > 1e-5)
                throw new InvalidOperationException("V21 planar melting missed the accuracy or conservation limits.");
            previousFront = frontError;
            previousProfile = profileError;
        }
    }

    // Series for the small arguments used by this fixture.
    private static double Erf(double x)
    {
        double term = x, sum = term;
        for (int n = 1; n < 20; n++)
        {
            term *= -x * x / n;
            sum += term / (2 * n + 1);
        }
        return 2 * sum / Math.Sqrt(Math.PI);
    }
}
