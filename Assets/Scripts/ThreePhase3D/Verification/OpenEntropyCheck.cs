using System;
using PhysicsSim.ThreePhase3D.Materials;

namespace PhysicsSim.ThreePhase3D.Verification
{
    internal static class OpenEntropyCheck
    {
        internal static void Run(Action<string> log)
        {
            const double pressure = 101325, inletTemperature = 290, outletTemperature = 310;
            const double heaterTemperature = 350, gasConstant = 8.314462618;
            const double airCapacity = 29.1, vapourCapacity = 33.6;

            foreach (var inlet in new[] { (SecondHumidity: 0.004, Heat: false),
                (SecondHumidity: 0.004, Heat: true), (SecondHumidity: 0.016, Heat: false),
                (SecondHumidity: 0.016, Heat: true) })
            {
                const double firstHumidity = 0.004;
                double firstWater = HumidGas.WaterMoleFraction(firstHumidity);
                double secondWater = HumidGas.WaterMoleFraction(inlet.SecondHumidity);
                double outlet = inlet.Heat ? outletTemperature : inletTemperature;
                const double firstFlow = 0.05, secondFlow = 0.03;
                double totalFlow = firstFlow + secondFlow;
                double waterFlow = firstFlow * firstWater + secondFlow * secondWater;
                double mixedWater = waterFlow / totalFlow;
                double capacityFlow = (totalFlow - waterFlow) * airCapacity + waterFlow * vapourCapacity;
                double heat = capacityFlow * (outlet - inletTemperature);

                // Ideal-gas species entropy at partial pressure. Reference terms cancel
                // because each species has the same total molar inflow and outflow.
                double EntropyFlow(double flow, double water, double temperature)
                {
                    double logTemperature = Math.Log(temperature / inletTemperature);
                    return flow * ((1 - water) * (airCapacity * logTemperature -
                        gasConstant * Math.Log((1 - water) * pressure / 100000)) +
                        water * (vapourCapacity * logTemperature -
                        gasConstant * Math.Log(water * pressure / 100000)));
                }

                double advectedOut = EntropyFlow(totalFlow, mixedWater, outlet) -
                    EntropyFlow(firstFlow, firstWater, inletTemperature) -
                    EntropyFlow(secondFlow, secondWater, inletTemperature);
                // Steady storage is zero. Advected entropy leaves while heat entropy enters.
                double produced = advectedOut - heat / heaterTemperature;

                double Mixing(double water) => water * Math.Log(water / mixedWater) +
                    (1 - water) * Math.Log((1 - water) / (1 - mixedWater));
                double expectedMixing = gasConstant *
                    (firstFlow * Mixing(firstWater) + secondFlow * Mixing(secondWater));
                double expectedHeating = capacityFlow * Math.Log(outlet / inletTemperature) -
                    heat / heaterTemperature;
                if (Math.Abs(totalFlow * mixedWater - waterFlow) > 1e-14 ||
                    Math.Abs(totalFlow * (1 - mixedWater) - (totalFlow - waterFlow)) > 1e-14 ||
                    Math.Abs(produced - expectedMixing - expectedHeating) > 1e-12 ||
                    (inlet.SecondHumidity == firstHumidity && !inlet.Heat
                        ? Math.Abs(produced) > 1e-12 : !(produced > 0)))
                    throw new InvalidOperationException("V42 open entropy balance failed.");

                log(FormattableString.Invariant(
                    $"S15 V42 open entropy passed: heat {heat:G8} W, mixing {expectedMixing:G8} W/K, production {produced:G8} W/K."));
            }
        }
    }
}
