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
            : this(FindSaturation(pressurePa, lowerK, upperK), liquidHeatWm2, vapourHeatWm2)
        {
        }

        private PureWaterStefanInterface(SaturationState saturation, double liquidHeatWm2, double vapourHeatWm2)
        {
            if (!SiUnits.IsFinite(liquidHeatWm2) || !SiUnits.IsFinite(vapourHeatWm2))
                throw new ArgumentException("Stefan heat fluxes must be finite.");
            Saturation = saturation;
            LiquidHeatWm2 = liquidHeatWm2;
            VapourHeatWm2 = vapourHeatWm2;
            MassFluxKgM2PerS = (liquidHeatWm2 + vapourHeatWm2) / saturation.LatentEnthalpyJkg;
            if (!SiUnits.IsFinite(MassFluxKgM2PerS))
                throw new ArgumentException("Heat fluxes must give a finite mass transfer.");
        }

        public static PureWaterStefanInterface SphericalConduction(double radiusM, double ambientPressurePa,
            double surfaceTensionNm, double bulkLiquidK, double liquidConductivityWmK, double lowerK, double upperK)
        {
            if (!SiUnits.IsFinite(radiusM) || radiusM <= 0 ||
                !SiUnits.IsFinite(ambientPressurePa) || ambientPressurePa <= 0 ||
                !SiUnits.IsFinite(surfaceTensionNm) || surfaceTensionNm < 0 ||
                !SiUnits.IsFinite(bulkLiquidK) || bulkLiquidK <= 0 ||
                !SiUnits.IsFinite(liquidConductivityWmK) || liquidConductivityWmK <= 0)
                throw new ArgumentException("Spherical conduction requires finite positive geometry and material properties.");
            var saturation = FindSaturation(ambientPressurePa + 2 * surfaceTensionNm / radiusM, lowerK, upperK);
            double heat = liquidConductivityWmK * (bulkLiquidK - saturation.Liquid.TemperatureK) / radiusM;
            return new PureWaterStefanInterface(saturation, heat, 0);
        }

        private static SaturationState FindSaturation(double pressurePa, double lowerK, double upperK)
        {
            if (!SiUnits.IsFinite(pressurePa) || pressurePa <= 0 ||
                !SiUnits.IsFinite(lowerK) || !SiUnits.IsFinite(upperK) || lowerK >= upperK)
                throw new ArgumentException("A Stefan interface requires a pressure bracket.");
            var water = new Water95();
            if (water.Saturation(lowerK).PressurePa > pressurePa || water.Saturation(upperK).PressurePa < pressurePa)
                throw new ArgumentException("Saturation pressure is outside the temperature bracket.");
            for (int iteration = 0; iteration < 48; iteration++)
            {
                double middle = (lowerK + upperK) * 0.5;
                var candidate = water.Saturation(middle);
                double residual = candidate.PressurePa - pressurePa;
                if (Math.Abs(residual) <= Math.Max(1e-5, pressurePa * 1e-10))
                    return candidate;
                if (residual < 0) lowerK = middle;
                else upperK = middle;
            }
            throw new InvalidOperationException("Stefan interface saturation temperature did not converge.");
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
