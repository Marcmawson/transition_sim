using System;
using PhysicsSim.ThreePhase3D.Flow;
using PhysicsSim.ThreePhase3D.Materials;

namespace PhysicsSim.ThreePhase3D.Verification
{
    internal static class IceVapourFilmCheck
    {
        internal static void Run(Action<string> log)
        {
            const double pressure = 101325, diffusivity = 2e-5, film = 0.001, area = 1e-4, dt = 0.01;
            var humid = new HumidGas();
            foreach (double temperature in new[] { 253.15, 263.15 })
            {
                var surface = humid.EquilibriumWithIce(temperature, pressure);
                double saturated = surface.WaterMassFraction;
                // Prescribed dilute-vapour caloric model, shared reference with the S14 fixture.
                double vapourH = DropletGasCellExchange.VapourReferenceEnthalpy +
                    DropletGasCellExchange.VapourCp * (temperature - DropletGasCellExchange.ReferenceK);
                foreach (double ratio in new[] { 0.5, 1.0, 1.5 })
                {
                    double ambient = saturated * ratio;
                    const double dryAir = 1e-5, ice = 1e-4;
                    double vapour = dryAir * ambient / (1 - ambient);
                    var result = IceVapourFilm.Exchange(surface, ambient, diffusivity, film, area, dt, ice, vapour, vapourH);
                    double flux = result.VapourKg / (area * dt);
                    // Integrate the constant-flux species ODE across the film and
                    // compare the recovered ambient composition to its boundary value.
                    double recovered = 1 - (1 - saturated) *
                        Math.Exp(flux * film / (surface.Gas.DensityKgM3 * diffusivity));
                    double updatedVapour = vapour + result.VapourKg;
                    double updatedGasMass = dryAir + vapour + result.VapourKg;
                    if (Math.Sign(flux) != Math.Sign(1 - ratio) || Math.Abs(recovered - ambient) > 1e-15 ||
                        Math.Abs((ice - result.VapourKg) + updatedVapour - ice - vapour) > 1e-18 ||
                        Math.Abs(updatedGasMass - updatedVapour - dryAir) > 1e-18)
                        throw new InvalidOperationException("V41 species flux or water balance failed.");
                    if (Math.Sign(result.SurfaceHeatJ) != Math.Sign(flux) ||
                        Math.Abs(result.IceEnthalpyJ + result.GasEnthalpyJ - result.SurfaceHeatJ) > 1e-15)
                        throw new InvalidOperationException("V41 surface heat ledger failed.");
                    log(FormattableString.Invariant(
                        $"S15 V41 ice-vapour film passed: {temperature:F2} K, Y/Yice {ratio:F1}, flux {flux:G8} kg/m2/s, surface heat {result.SurfaceHeatJ:G8} J."));
                }
            }
            const double initialK = 263.15, initialIce = 1e-7, dryAirKg = 1e-5;
            var start = humid.EquilibriumWithIce(initialK, pressure);
            var iceModel = new IceIh();
            foreach (double humidity in new[] { 0.5, 1.5 })
            {
                var coarse = RunTransient(8);
                var fine = RunTransient(16);
                var reference = RunTransient(32);
                if (Math.Sign(reference.Temperature - initialK) != Math.Sign(humidity - 1) ||
                    Math.Sign(reference.Vapour) != Math.Sign(1 - humidity) ||
                    Math.Abs(fine.Temperature - reference.Temperature) >= Math.Abs(coarse.Temperature - reference.Temperature))
                    throw new InvalidOperationException("V41 transient ice exchange did not warm, cool or refine as expected.");
                log(FormattableString.Invariant(
                    $"S15 V41 transient ice passed: Y/Yice {humidity:F1}, final {reference.Temperature:F6} K, vapour change {reference.Vapour:G8} kg."));

                (double Temperature, double Vapour) RunTransient(int steps)
                {
                    double initialFraction = humidity * start.WaterMassFraction;
                    double initialVapour = dryAirKg * initialFraction / (1 - initialFraction);
                    double vapour = initialVapour, ice = initialIce, kelvin = initialK, gasEnergy = 0, heat = 0;
                    double initialEnergy = ice * start.Ice.EnthalpyJkg;
                    for (int step = 0; step < steps; step++)
                    {
                        var surface = humid.EquilibriumWithIce(kelvin, pressure);
                        double vapourH = DropletGasCellExchange.VapourReferenceEnthalpy +
                            DropletGasCellExchange.VapourCp * (kelvin - DropletGasCellExchange.ReferenceK);
                        var next = IceVapourFilm.AdvanceLumped(surface, vapour / (dryAirKg + vapour),
                            diffusivity, film, area, 0.02 / steps, ice, vapour, vapourH, initialK, 25);
                        ice = next.IceKg;
                        vapour += next.VapourKg;
                        gasEnergy += next.GasEnthalpyJ;
                        heat += next.HeatIntoIceJ;
                        double target = next.IceEnthalpyJ / ice;
                        double low = 250, high = 270;
                        if (!(target >= iceModel.Evaluate(low, pressure).EnthalpyJkg &&
                            target <= iceModel.Evaluate(high, pressure).EnthalpyJkg))
                            throw new InvalidOperationException("V41 ice enthalpy left the reference bracket.");
                        for (int iteration = 0; iteration < 45; iteration++)
                        {
                            double middle = (low + high) * 0.5;
                            if (iceModel.Evaluate(middle, pressure).EnthalpyJkg < target) low = middle;
                            else high = middle;
                        }
                        kelvin = (low + high) * 0.5;
                        if (Math.Abs(ice + vapour - initialIce - initialVapour) > 1e-19 ||
                            Math.Abs(ice * iceModel.Evaluate(kelvin, pressure).EnthalpyJkg + gasEnergy - initialEnergy - heat) > 1e-13)
                            throw new InvalidOperationException("V41 transient water or enthalpy ledger failed.");
                    }
                    return (kelvin, vapour - initialVapour);
                }
            }
        }
    }
}
