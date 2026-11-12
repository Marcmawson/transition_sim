using System;
using PhysicsSim.ThreePhase3D.Core;

namespace PhysicsSim.ThreePhase3D.Flow
{
    // Quasi-steady spherical conduction with uniform temperatures and fixed properties.
    internal static class DropletSensibleHeat
    {
        internal static double EnergyIntoLiquidJ(double radiusM, double gasConductivityWmK,
            double liquidHeatCapacityJK, double gasHeatCapacityJK, double liquidK, double gasK, double dt)
        {
            if (!Positive(radiusM) || !Positive(gasConductivityWmK) || !Positive(liquidHeatCapacityJK) ||
                !Positive(gasHeatCapacityJK) || !Positive(liquidK) || !Positive(gasK) || !Positive(dt))
                throw new ArgumentException("Droplet heat exchange requires positive finite geometry, properties, temperatures and timestep.");

            double conductance = 4 * Math.PI * gasConductivityWmK * radiusM;
            double reducedCapacity = Math.Min(liquidHeatCapacityJK, gasHeatCapacityJK) /
                (1 + Math.Min(liquidHeatCapacityJK, gasHeatCapacityJK) / Math.Max(liquidHeatCapacityJK, gasHeatCapacityJK));
            double ratio = conductance * dt / reducedCapacity;
            double response = ratio < 1e-5 ? ratio * (1 - ratio * (0.5 - ratio / 6)) : 1 - Math.Exp(-ratio);
            return reducedCapacity * (gasK - liquidK) * response;
        }

        private static bool Positive(double value) => SiUnits.IsFinite(value) && value > 0;
    }
}
