using System;
using PhysicsSim.ThreePhase3D.Flow;

namespace PhysicsSim.ThreePhase3D.Verification
{
    internal static class DropletSensibleHeatCheck
    {
        internal static void Run(Action<string> log)
        {
            const double conductivity = 0.026, density = 998, specificHeat = 4182, duration = 0.005;
            foreach (double radius in new[] { 5e-6, 10e-6, 15e-6 })
            foreach (double initialGas in new[] { 285.0, 300.0, 315.0 })
            {
                const double initialLiquid = 300;
                double liquidCapacity = 4 * Math.PI * radius * radius * radius * density * specificHeat / 3;
                double gasCapacity = 10 * liquidCapacity;
                double equilibrium = (initialLiquid + 10 * initialGas) / 11;
                double relaxation = density * specificHeat * radius * radius / (3 * conductivity * 1.1);
                double expected = equilibrium + (initialLiquid - equilibrium) * Math.Exp(-duration / relaxation);
                double liquid = initialLiquid;
                double gas = initialGas;
                foreach (int steps in new[] { 1, 10, 100 })
                {
                    liquid = initialLiquid;
                    gas = initialGas;
                    for (int step = 0; step < steps; step++)
                    {
                        double energy = DropletSensibleHeat.EnergyIntoLiquidJ(radius, conductivity,
                            liquidCapacity, gasCapacity, liquid, gas, duration / steps);
                        liquid += energy / liquidCapacity;
                        gas -= energy / gasCapacity;
                        double residualK = liquid - initialLiquid + 10 * (gas - initialGas);
                        if (Math.Abs(residualK) > 1e-10 || (gas - liquid) * (initialGas - initialLiquid) < 0)
                            throw new InvalidOperationException("S14 droplet heat exchange failed energy closure or overshot equilibrium.");
                    }
                    if (Math.Abs(liquid - expected) > 1e-10)
                        throw new InvalidOperationException("S14 droplet temperature differs from the analytical solution.");
                }
                log(FormattableString.Invariant(
                    $"S14 V37 sensible heat passed: radius {radius * 1e6:G3} um, gas initially {initialGas:G3} K, droplet {liquid:F6} K, gas {gas:F6} K."));
            }
        }
    }
}
