using System;
using PhysicsSim.ThreePhase3D.Flow;

namespace PhysicsSim.ThreePhase3D.Verification
{
    internal static class SeededFogCheck
    {
        internal static void Run(Action<string> log)
        {
            const double temperature = 290, duration = 0.001;
            const double referenceK = DropletGasCellExchange.ReferenceK;
            var seeds = new[] { (8e-6, temperature, 12), (10e-6, temperature, 20), (12e-6, temperature, 8) };
            var expanded = new (double, double, int)[40];
            int cursor = 0;
            foreach (var seed in seeds)
                for (int count = 0; count < seed.Item3; count++)
                    expanded[cursor++] = (seed.Item1, temperature, 1);
            double initialLiquid = Water(seeds);
            double dryAir = 10 * initialLiquid;
            foreach (double humidity in new[] { 0.005, 0.018 })
            {
                double initialVapour = dryAir * humidity;
                double initialGasEnergy = (dryAir * DropletGasCellExchange.AirCp +
                    initialVapour * DropletGasCellExchange.VapourCp) * (temperature - referenceK) +
                    initialVapour * DropletGasCellExchange.VapourReferenceEnthalpy;
                var grouped = Simulate(seeds, 32);
                var reference = Simulate(expanded, 32);
                var coarse = Simulate(seeds, 16);
                var fine = Simulate(seeds, 64);
                if (Math.Abs(grouped.VapourKg - reference.VapourKg) > 1e-12 * initialLiquid ||
                    Math.Abs(grouped.GasEnthalpyJ - reference.GasEnthalpyJ) > 1e-12 * initialGasEnergy ||
                    Math.Abs(grouped.VapourKg - fine.VapourKg) >= Math.Abs(coarse.VapourKg - fine.VapourKg))
                    throw new InvalidOperationException("S14 fog inventory did not agree across parcel counts or refine in time.");
                cursor = 0;
                for (int bin = 0; bin < seeds.Length; bin++)
                {
                    var parcel = grouped.Parcels[bin];
                    if (Math.Sign(parcel.RadiusM - seeds[bin].Item1) != Math.Sign(humidity - 0.01))
                        throw new InvalidOperationException("S14 fog seed size changed in the wrong direction.");
                    for (int count = 0; count < parcel.Multiplicity; count++)
                    {
                        var single = reference.Parcels[cursor++];
                        if (Math.Abs(single.RadiusM - parcel.RadiusM) > 1e-16 ||
                            Math.Abs(single.TemperatureK - parcel.TemperatureK) > 1e-9)
                            throw new InvalidOperationException("S14 fog radius or temperature changed with parcel regrouping.");
                    }
                }
                log(FormattableString.Invariant(
                    $"S14 V38 seeded fog passed: initial vapour/dry-air ratio {humidity:G3}, liquid change {initialVapour - fine.VapourKg:G8} kg, radii {fine.Parcels[0].RadiusM * 1e6:F6}/{fine.Parcels[1].RadiusM * 1e6:F6}/{fine.Parcels[2].RadiusM * 1e6:F6} um."));

                ((double RadiusM, double TemperatureK, int Multiplicity)[] Parcels, double VapourKg, double GasEnthalpyJ)
                    Simulate((double, double, int)[] parcels, int steps)
                {
                    var state = (Parcels: parcels, VapourKg: initialVapour, GasEnthalpyJ: initialGasEnergy);
                    double totalEnergy = initialGasEnergy + Heat(parcels);
                    for (int step = 0; step < steps; step++)
                    {
                        state = SeededFogCell.Advance(state.Parcels, dryAir, state.VapourKg, state.GasEnthalpyJ,
                            101325, 0.026, 2.5e-5, duration / steps);
                        if (Math.Abs(Water(state.Parcels) + state.VapourKg - initialLiquid - initialVapour) > 1e-12 * initialLiquid ||
                            Math.Abs(Heat(state.Parcels) + state.GasEnthalpyJ - totalEnergy) > 1e-12 * totalEnergy)
                            throw new InvalidOperationException("S14 fog failed water or enthalpy conservation.");
                    }
                    return state;
                }
            }

            double Water((double Radius, double Temperature, int Count)[] parcels)
            {
                double total = 0;
                foreach (var parcel in parcels)
                    total += DropletGasCellExchange.Mass(parcel.Radius) * parcel.Count;
                return total;
            }

            double Heat((double Radius, double Temperature, int Count)[] parcels)
            {
                double total = 0;
                foreach (var parcel in parcels)
                    total += DropletGasCellExchange.Mass(parcel.Radius) * parcel.Count *
                        DropletGasCellExchange.LiquidCp * (parcel.Temperature - referenceK);
                return total;
            }
        }
    }
}
