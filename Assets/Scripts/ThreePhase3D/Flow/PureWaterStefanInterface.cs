using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Materials;

namespace PhysicsSim.ThreePhase3D.Flow
{
    // Equilibrium planar liquid–vapour interface. Heat fluxes point into the interface.
    public sealed class PureWaterStefanInterface
    {
        public SaturationState Saturation { get; }
        public double MassFluxKgM2PerS { get; }
        public double LiquidHeatWm2 { get; }
        public double VapourHeatWm2 { get; }
        public double InterfaceSpeedRelativeToLiquidMPerS => -MassFluxKgM2PerS / Saturation.Liquid.DensityKgM3;
        public double VapourMinusLiquidVelocityMPerS => MassFluxKgM2PerS *
            (1 / Saturation.Vapour.DensityKgM3 - 1 / Saturation.Liquid.DensityKgM3);

        public PureWaterStefanInterface(double pressurePa, double lowerK, double upperK,
            double liquidHeatWm2, double vapourHeatWm2)
        {
            if (!SiUnits.IsFinite(pressurePa) || pressurePa <= 0 ||
                !SiUnits.IsFinite(lowerK) || !SiUnits.IsFinite(upperK) || lowerK >= upperK ||
                !SiUnits.IsFinite(liquidHeatWm2) || !SiUnits.IsFinite(vapourHeatWm2))
                throw new ArgumentException("A planar Stefan interface requires finite heat fluxes and a pressure bracket.");
            var water = new Water95();
            if (water.Saturation(lowerK).PressurePa > pressurePa || water.Saturation(upperK).PressurePa < pressurePa)
                throw new ArgumentException("Saturation pressure is outside the temperature bracket.");
            for (int iteration = 0; iteration < 48; iteration++)
            {
                double middle = (lowerK + upperK) * 0.5;
                var candidate = water.Saturation(middle);
                double residual = candidate.PressurePa - pressurePa;
                if (Math.Abs(residual) <= Math.Max(1e-5, pressurePa * 1e-10))
                {
                    Saturation = candidate;
                    LiquidHeatWm2 = liquidHeatWm2;
                    VapourHeatWm2 = vapourHeatWm2;
                    MassFluxKgM2PerS = (liquidHeatWm2 + vapourHeatWm2) / candidate.LatentEnthalpyJkg;
                    if (!SiUnits.IsFinite(MassFluxKgM2PerS))
                        throw new ArgumentException("Heat fluxes must give a finite mass transfer.");
                    return;
                }
                if (residual < 0) lowerK = middle;
                else upperK = middle;
            }
            throw new InvalidOperationException("Planar interface saturation temperature did not converge.");
        }

        public (double WaterKg, double LiquidM3, double VapourM3, double LiquidEnthalpyJ, double VapourEnthalpyJ)
            Transfer(double areaM2, double dt, double liquidKg, double vapourKg)
        {
            if (!SiUnits.IsFinite(areaM2) || areaM2 < 0 || !SiUnits.IsFinite(dt) || dt <= 0 ||
                !SiUnits.IsFinite(liquidKg) || liquidKg < 0 || !SiUnits.IsFinite(vapourKg) || vapourKg < 0)
                throw new ArgumentException("Phase transfer requires a finite area, timestep and donor inventories.");
            double exposure = areaM2 * dt;
            double water = exposure * MassFluxKgM2PerS;
            if (!SiUnits.IsFinite(water) || water > liquidKg || -water > vapourKg)
                throw new InvalidOperationException("Stefan transfer would overdraw the donor. Reduce the timestep.");
            var liquid = Saturation.Liquid;
            var vapour = Saturation.Vapour;
            return (water, -water / liquid.DensityKgM3, water / vapour.DensityKgM3,
                -water * liquid.EnthalpyJkg - LiquidHeatWm2 * exposure,
                water * vapour.EnthalpyJkg - VapourHeatWm2 * exposure);
        }
    }
}
