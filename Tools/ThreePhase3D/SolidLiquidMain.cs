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
    }

    private static void Near(double actual, double expected, double tolerance)
    {
        if (double.IsNaN(actual) || Math.Abs(actual - expected) > tolerance)
            throw new InvalidOperationException($"Expected {expected:G17}, got {actual:G17}.");
    }
}
