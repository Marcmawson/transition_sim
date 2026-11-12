using System;
using PhysicsSim.ThreePhase3D.Materials;
using PhysicsSim.ThreePhase3D.PhaseChange;

internal static class SolidLiquidMain
{
    private static void Main()
    {
        const double mass = 0.25;
        var ice = new IceIh();
        var water = new Water95();
        double previousMeltingTemperature = double.PositiveInfinity;
        foreach (double pressure in new[] { 2000, 101325, 125000 })
        {
            var model = new SolidLiquidInterface(pressure, 250, 290);
            double tm = model.MeltingTemperatureK;
            if (tm >= previousMeltingTemperature || model.LatentHeatJkg < 330000 || model.LatentHeatJkg > 336000)
                throw new InvalidOperationException("Unexpected melting curve or latent heat.");
            previousMeltingTemperature = tm;

            var cold = model.Recover(mass, mass * ice.Evaluate(260, pressure).EnthalpyJkg);
            Near(cold.TemperatureK, 260, 1e-8);
            Near(cold.IceMassKg, mass, 0);
            var warm = model.Recover(mass, mass * water.Evaluate(280, pressure, WaterBranch.Liquid).EnthalpyJkg);
            Near(warm.TemperatureK, 280, 1e-8);
            Near(warm.LiquidMassKg, mass, 0);

            var frozen = model.Recover(mass, mass * model.IceMeltingEnthalpyJkg);
            var melted = model.Recover(mass, mass * model.LiquidMeltingEnthalpyJkg);
            foreach (double fraction in new[] { 0, 0.25, 0.5, 0.75, 1 })
            {
                double energy = mass * (model.IceMeltingEnthalpyJkg + fraction * model.LatentHeatJkg);
                var mixed = model.Recover(mass, energy);
                Near(mixed.TemperatureK, tm, 1e-8);
                Near(mixed.LiquidMassKg, mass * fraction, 1e-12);
                Near(mixed.IceMassKg + mixed.LiquidMassKg, mass, 1e-12);
                Near(mixed.IceMassKg * model.IceMeltingEnthalpyJkg +
                    mixed.LiquidMassKg * model.LiquidMeltingEnthalpyJkg, energy, 1e-8);
                Near(mixed.IceVolumeM3 + mixed.LiquidVolumeM3,
                    (1 - fraction) * frozen.IceVolumeM3 + fraction * melted.LiquidVolumeM3, 1e-12);
            }

            double expansion = frozen.IceVolumeM3 - melted.LiquidVolumeM3;
            if (expansion <= 0) throw new InvalidOperationException("Freezing must increase volume in this interval.");
            Console.WriteLine(FormattableString.Invariant(
                $"S10 equilibrium passed at {pressure:G} Pa: melt {tm:F6} K, latent {model.LatentHeatJkg / 1000:F3} kJ/kg, expansion {expansion * 1e6:F3} mL for 250 g."));
        }
        CheckHeadspace(mass);
    }

    private static void CheckHeadspace(double mass)
    {
        const double initialPressure = 101325, initialGasVolume = 0.001;
        double gasInventory = initialPressure * initialGasVolume; // nRT for the isothermal ideal gas.
        var initialModel = new SolidLiquidInterface(initialPressure, 250, 290);
        var liquid = Recover(initialModel, 0);
        double initialWaterVolume = liquid.LiquidVolumeM3;
        double vesselVolume = initialWaterVolume + initialGasVolume;
        var ice = new IceIh();
        var water = new Water95();
        double iceDensity = ice.Evaluate(initialModel.MeltingTemperatureK, initialPressure).DensityKgM3;
        // The public stable-liquid provider starts at 273.16 K, within 0.008 K
        // of this reference melting point. Allow for that temperature difference.
        double liquidDensity = water.Evaluate(273.16, initialPressure, WaterBranch.Liquid).DensityKgM3;
        double previousPressure = initialPressure;
        double previousGasVolume = initialGasVolume;
        foreach (double iceFraction in new[] { 0.0, 0.25, 0.5, 0.75, 1.0 })
        {
            double referenceDisplacement = mass * iceFraction * (1 / iceDensity - 1 / liquidDensity);
            var vented = Recover(initialModel, iceFraction);
            double ventedDisplacement = vented.IceVolumeM3 + vented.LiquidVolumeM3 - initialWaterVolume;
            Near(ventedDisplacement, referenceDisplacement, 2e-10);

            double Residual(double pressure)
            {
                var model = new SolidLiquidInterface(pressure, 250, 290);
                var state = Recover(model, iceFraction);
                double gasVolume = vesselVolume - state.IceVolumeM3 - state.LiquidVolumeM3;
                return pressure * gasVolume - gasInventory;
            }

            double lower = initialPressure, upper = 125000;
            if (Residual(lower) > 1e-10 || Residual(upper) <= 0)
                throw new InvalidOperationException("V23 headspace pressure is outside the fixture bracket.");
            for (int i = 0; i < 45; i++)
            {
                double mid = (lower + upper) / 2;
                if (Residual(mid) < 0) lower = mid;
                else upper = mid;
            }

            double pressurePa = (lower + upper) / 2;
            var sealedState = Recover(new SolidLiquidInterface(pressurePa, 250, 290), iceFraction);
            double gas = vesselVolume - sealedState.IceVolumeM3 - sealedState.LiquidVolumeM3;
            double referencePressure = gasInventory / (initialGasVolume - referenceDisplacement);
            Near(pressurePa, referencePressure, 0.1);
            Near(pressurePa * gas, gasInventory, 1e-9);
            Near(sealedState.IceMassKg + sealedState.LiquidMassKg, mass, 1e-12);
            Near(sealedState.IceMassKg, mass * iceFraction, 1e-12);
            if (gas <= 0 || (iceFraction > 0 && (pressurePa <= previousPressure || gas >= previousGasVolume)))
                throw new InvalidOperationException("V23 freezing must compress the headspace.");
            previousPressure = pressurePa;
            previousGasVolume = gas;
            Console.WriteLine(FormattableString.Invariant(
                $"V23 {iceFraction:P0} ice: vented displacement {ventedDisplacement * 1e6:F4} mL, sealed displacement {(initialGasVolume - gas) * 1e6:F4} mL, sealed pressure {pressurePa:F4} Pa."));
        }

        SolidLiquidInterface.State Recover(SolidLiquidInterface model, double iceFraction) =>
            model.Recover(mass, mass * (model.IceMeltingEnthalpyJkg + (1 - iceFraction) * model.LatentHeatJkg));
    }

    private static void Near(double actual, double expected, double tolerance)
    {
        if (double.IsNaN(actual) || Math.Abs(actual - expected) > tolerance)
            throw new InvalidOperationException($"Expected {expected:G17}, got {actual:G17}.");
    }
}
