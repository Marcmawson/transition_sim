using System;

namespace PhysicsSim.ThreePhase3D.Flow
{
    // Each parcel represents identical seeded droplets sharing one gas inventory.
    internal static class SeededFogCell
    {
        internal static ((double RadiusM, double TemperatureK, int Multiplicity)[] Parcels,
            double VapourKg, double GasEnthalpyJ) Advance(
            (double RadiusM, double TemperatureK, int Multiplicity)[] parcels,
            double dryAirKg, double vapourKg, double gasEnthalpyJ, double pressurePa,
            double conductivity, double diffusivity, double dt)
        {
            long seedCount = 0;
            foreach (var parcel in parcels)
            {
                if (parcel.Multiplicity <= 0 || !(parcel.RadiusM > 0))
                    throw new ArgumentException("Fog parcels require a positive seed radius and multiplicity.", nameof(parcels));
                seedCount += parcel.Multiplicity;
            }
            if (seedCount == 0) return (Array.Empty<(double, double, int)>(), vapourKg, gasEnthalpyJ);

            // Each seed sees the same starting gas state. The per-seed share
            // depends on physical count, so splitting a parcel does not change the update.
            double seedAir = dryAirKg / seedCount, seedVapour = vapourKg / seedCount;
            double seedEnthalpy = gasEnthalpyJ / seedCount;
            var updated = new (double RadiusM, double TemperatureK, int Multiplicity)[parcels.Length];
            double vapourChange = 0, enthalpyChange = 0;
            for (int index = 0; index < parcels.Length; index++)
            {
                var parcel = parcels[index];
                var exchange = DropletGasCellExchange.Advance(parcel.RadiusM, parcel.TemperatureK,
                    seedAir, seedVapour, seedEnthalpy, pressurePa, conductivity, diffusivity, dt, allowCondensation: true);
                updated[index] = (exchange.RadiusM, exchange.LiquidK, parcel.Multiplicity);
                vapourChange += (exchange.VapourKg - seedVapour) * parcel.Multiplicity;
                enthalpyChange += (exchange.GasEnthalpyJ - seedEnthalpy) * parcel.Multiplicity;
            }
            double updatedVapour = vapourKg + vapourChange;
            double updatedEnthalpy = gasEnthalpyJ + enthalpyChange;
            if (updatedVapour < 0)
                throw new InvalidOperationException("Fog condensation would overdraw the cell vapour inventory.");
            DropletGasCellExchange.GasVolumeM3(dryAirKg, updatedVapour, updatedEnthalpy, pressurePa);
            return (updated, updatedVapour, updatedEnthalpy);
        }
    }
}
