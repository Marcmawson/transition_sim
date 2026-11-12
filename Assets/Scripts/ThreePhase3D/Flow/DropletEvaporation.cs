using System;
using PhysicsSim.ThreePhase3D.Core;

namespace PhysicsSim.ThreePhase3D.Flow
{
    // Isothermal, stagnant-gas Spalding limit with frozen film properties.
    internal static class DropletEvaporation
    {
        internal static (double RadiusM, double VapourKg, double VapourEnthalpyJ, double HeatRequiredJ) Advance(
            double radiusM, double liquidDensity, double gasDensity, double diffusivity,
            double surfaceVapourFraction, double ambientVapourFraction,
            double liquidEnthalpyJkg, double vapourEnthalpyJkg, double dt, bool allowCondensation = false)
        {
            if (!SiUnits.IsFinite(radiusM) || radiusM < 0 || !Positive(liquidDensity) ||
                !Positive(gasDensity) || !Positive(diffusivity) || !Positive(dt) ||
                !(ambientVapourFraction >= 0 && ambientVapourFraction < 1 &&
                    surfaceVapourFraction >= 0 && surfaceVapourFraction < 1 &&
                    (allowCondensation || ambientVapourFraction <= surfaceVapourFraction)) ||
                !SiUnits.IsFinite(liquidEnthalpyJkg) || !Positive(vapourEnthalpyJkg - liquidEnthalpyJkg))
                throw new ArgumentException("Invalid isolated evaporation properties or supersaturated ambient gas.");

            double spalding = (surfaceVapourFraction - ambientVapourFraction) / (1 - surfaceVapourFraction);
            double driving = Math.Abs(spalding) < 1e-5 ? spalding * (1 - spalding * (0.5 - spalding / 3)) :
                Math.Log((1 - ambientVapourFraction) / (1 - surfaceVapourFraction));
            if (radiusM == 0) return default;
            double radiusSquared = radiusM * radiusM;
            double decrement = Math.Min(radiusSquared, 2 * gasDensity * diffusivity / liquidDensity * driving * dt);
            double remaining = Math.Sqrt(radiusSquared - decrement);
            double radiusLoss = decrement / (radiusM + remaining);
            double vapour = (4 * Math.PI / 3) * liquidDensity * radiusLoss *
                (radiusSquared + radiusM * remaining + remaining * remaining);
            return (remaining, vapour, vapour * vapourEnthalpyJkg, vapour * (vapourEnthalpyJkg - liquidEnthalpyJkg));
        }

        private static bool Positive(double value) => SiUnits.IsFinite(value) && value > 0;
    }
}
