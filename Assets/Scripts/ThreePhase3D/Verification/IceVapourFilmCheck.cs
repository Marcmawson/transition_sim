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
        }
    }
}
