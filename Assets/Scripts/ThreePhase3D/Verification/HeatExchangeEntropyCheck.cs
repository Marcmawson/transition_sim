using System;
using PhysicsSim.ThreePhase3D.Flow;

namespace PhysicsSim.ThreePhase3D.Verification
{
    internal static class HeatExchangeEntropyCheck
    {
        internal static void Run(Action<string> log)
        {
            const double coldCapacity = 2, hotCapacity = 3;
            const double initialCold = 280, initialHot = 340;
            const double radius = 0.01, conductivity = 0.1, duration = 3000;
            double equilibrium = (coldCapacity * initialCold + hotCapacity * initialHot) / (coldCapacity + hotCapacity);
            double initialEnergy = coldCapacity * initialCold + hotCapacity * initialHot;
            double expectedEntropy = coldCapacity * Math.Log(equilibrium / initialCold) +
                hotCapacity * Math.Log(equilibrium / initialHot);
            foreach (int steps in new[] { 16, 64, 256 })
            {
                double cold = initialCold, hot = initialHot, entropy = 0;
                for (int step = 0; step < steps; step++)
                {
                    double energy = DropletSensibleHeat.EnergyIntoLiquidJ(radius, conductivity,
                        coldCapacity, hotCapacity, cold, hot, duration / steps);
                    cold += energy / coldCapacity;
                    hot -= energy / hotCapacity;
                    double nextEntropy = coldCapacity * Math.Log(cold / initialCold) +
                        hotCapacity * Math.Log(hot / initialHot);
                    if (nextEntropy < entropy - 1e-13 || cold > hot + 1e-12 ||
                        Math.Abs(coldCapacity * cold + hotCapacity * hot - initialEnergy) > 1e-10)
                        throw new InvalidOperationException("V42 heat exchange failed energy conservation or entropy monotonicity.");
                    entropy = nextEntropy;
                }
                if (Math.Abs(cold - equilibrium) > 1e-10 || Math.Abs(hot - equilibrium) > 1e-10 ||
                    Math.Abs(entropy - expectedEntropy) > 1e-12 || !(entropy > 0))
                    throw new InvalidOperationException("V42 heat exchange failed the analytical equilibrium check.");
                log(FormattableString.Invariant(
                    $"S15 V42 isolated heat entropy passed: {steps} steps, equilibrium {equilibrium:F3} K, entropy produced {entropy:G9} J/K."));
            }
        }
    }
}
