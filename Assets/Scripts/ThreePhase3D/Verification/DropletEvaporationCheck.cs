using System;
using PhysicsSim.ThreePhase3D.Flow;

namespace PhysicsSim.ThreePhase3D.Verification
{
    internal static class DropletEvaporationCheck
    {
        internal static void Run(Action<string> log)
        {
            const double liquidDensity = 998, gasDensity = 1.2, diffusivity = 2.5e-5;
            const double surface = 0.02, liquidEnthalpy = 112000, vapourEnthalpy = 2550000;
            foreach (double initial in new[] { 5e-6, 10e-6, 15e-6 })
            foreach (double ambient in new[] { 0.005, surface })
            {
                double initialMass = Mass(initial);
                double slope = 8 * gasDensity * diffusivity / liquidDensity * Math.Log((1 - ambient) / (1 - surface));
                foreach (double duration in new[] { 0.01, 0.5 })
                foreach (int steps in new[] { 1, 10, 100 })
                {
                    double radius = initial, vapour = 0, gasEnergy = 0, heat = 0;
                    for (int step = 0; step < steps; step++)
                    {
                        var result = DropletEvaporation.Advance(radius, liquidDensity, gasDensity,
                            diffusivity, surface, ambient, liquidEnthalpy, vapourEnthalpy, duration / steps);
                        radius = result.RadiusM;
                        vapour += result.VapourKg;
                        gasEnergy += result.VapourEnthalpyJ;
                        heat += result.HeatRequiredJ;
                        double residual = Mass(radius) * liquidEnthalpy + gasEnergy - initialMass * liquidEnthalpy - heat;
                        if (Math.Abs(Mass(radius) + vapour - initialMass) > initialMass * 1e-12 ||
                            Math.Abs(residual) > initialMass * vapourEnthalpy * 1e-12)
                            throw new InvalidOperationException("S14 evaporation failed mass or enthalpy closure.");
                    }
                    double expectedDiameterSquared = Math.Max(0, 4 * initial * initial - slope * duration);
                    if (Math.Abs(4 * radius * radius - expectedDiameterSquared) > 4 * initial * initial * 1e-12 ||
                        (ambient == surface && vapour != 0))
                        throw new InvalidOperationException("S14 evaporation differed from the analytical diameter-squared law.");
                }
                log(FormattableString.Invariant(
                    $"S14 V37 evaporation passed: radius {initial * 1e6:G3} um, ambient vapour mass fraction {ambient:G3}, diameter-squared slope {-slope:G8} m2/s."));
            }

            double Mass(double radius) => (4 * Math.PI / 3) * liquidDensity * radius * radius * radius;
        }
    }
}
