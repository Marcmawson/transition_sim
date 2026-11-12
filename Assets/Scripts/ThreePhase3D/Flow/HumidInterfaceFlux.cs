using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Materials;

namespace PhysicsSim.ThreePhase3D.Flow
{
    // Isothermal gas film with no net dry-air flux. Positive flux evaporates water.
    public static class HumidInterfaceFlux
    {
        // Integrated inventory increments before geometric remap and pressure projection.
        public static (double WaterKg, double LiquidVolumeM3, double GasVolumeM3,
            double LiquidEnthalpyJ, double GasEnthalpyJ) InventoryTransfer(
            (GasLiquidEquilibrium Surface, double MassFluxKgM2PerS, double LiquidHeatWm2,
                double GasHeatWm2, double LatentHeatJkg) solved, double areaM2, double seconds,
            double availableLiquidKg, double availableVapourKg)
        {
            if (solved.Surface == null || !SiUnits.IsFinite(areaM2) || areaM2 < 0 ||
                !SiUnits.IsFinite(seconds) || seconds <= 0 ||
                !SiUnits.IsFinite(availableLiquidKg) || availableLiquidKg < 0 ||
                !SiUnits.IsFinite(availableVapourKg) || availableVapourKg < 0)
                throw new ArgumentException("Phase transfer requires a solved interface and finite donor inventories.");
            double balance = solved.LiquidHeatWm2 + solved.GasHeatWm2 -
                solved.MassFluxKgM2PerS * solved.LatentHeatJkg;
            if (!(Math.Abs(balance) <= 1e-7))
                throw new ArgumentException("The interface heat and latent fluxes must balance.", nameof(solved));
            double areaTime = areaM2 * seconds;
            double mass = solved.MassFluxKgM2PerS * areaTime;
            if (!SiUnits.IsFinite(mass) || mass > availableLiquidKg || -mass > availableVapourKg)
                throw new InvalidOperationException("Phase transfer would overdraw the donor. Reduce the timestep.");
            double liquidH = solved.Surface.Liquid.EnthalpyJkg;
            return (mass, -mass / solved.Surface.Liquid.DensityKgM3, mass / solved.Surface.Gas.DensityKgM3,
                -mass * liquidH - solved.LiquidHeatWm2 * areaTime,
                mass * (liquidH + solved.LatentHeatJkg) - solved.GasHeatWm2 * areaTime);
        }

        // Balance heat from the two reservoirs against the phase-change enthalpy.
        // Use pure saturated vapour enthalpy for the transferred vapour in dilute air.
        public static (GasLiquidEquilibrium Surface, double MassFluxKgM2PerS, double LiquidHeatWm2,
            double GasHeatWm2, double LatentHeatJkg) SolveTemperature(
            double pressurePa, double bulkWaterMassFraction, double filmThicknessM,
            double liquidTemperatureK, double gasTemperatureK, double liquidConductanceWm2K,
            double gasConductanceWm2K, double lowerK, double upperK)
        {
            if (!Positive(liquidTemperatureK) || !Positive(gasTemperatureK) ||
                !Positive(liquidConductanceWm2K) || !Positive(gasConductanceWm2K) ||
                !Positive(lowerK) || !Positive(upperK) || lowerK >= upperK)
                throw new ArgumentException("The interface requires positive reservoir properties and an ordered temperature bracket.");
            var humid = new HumidGas();
            var water = new Water95();
            if (Residual(Evaluate(lowerK)) < 0 || Residual(Evaluate(upperK)) > 0)
                throw new InvalidOperationException("Interface energy balance is outside the supplied temperature bracket.");
            for (int iteration = 0; iteration < 48; iteration++)
            {
                double middle = 0.5 * (lowerK + upperK);
                var trial = Evaluate(middle);
                double residual = Residual(trial);
                if (Math.Abs(residual) < 1e-7) return trial;
                if (residual > 0) lowerK = middle;
                else upperK = middle;
            }
            throw new InvalidOperationException("The humid interface energy balance did not converge.");

            (GasLiquidEquilibrium Surface, double MassFluxKgM2PerS, double LiquidHeatWm2,
                double GasHeatWm2, double LatentHeatJkg) Evaluate(double temperature)
            {
                var surface = humid.EquilibriumWithLiquid(temperature, pressurePa);
                double diffusivity = HumidGas.ReferenceDiffusivity(temperature, pressurePa);
                double massFlux = WaterFluxKgM2PerS(surface, bulkWaterMassFraction, filmThicknessM, diffusivity);
                double latent = water.Saturation(temperature).Vapour.EnthalpyJkg - surface.Liquid.EnthalpyJkg;
                return (surface, massFlux, liquidConductanceWm2K * (liquidTemperatureK - temperature),
                    gasConductanceWm2K * (gasTemperatureK - temperature), latent);
            }

            double Residual((GasLiquidEquilibrium Surface, double MassFluxKgM2PerS, double LiquidHeatWm2,
                double GasHeatWm2, double LatentHeatJkg) trial) =>
                trial.LiquidHeatWm2 + trial.GasHeatWm2 - trial.MassFluxKgM2PerS * trial.LatentHeatJkg;
            bool Positive(double value) => SiUnits.IsFinite(value) && value > 0;
        }

        public static double WaterFluxKgM2PerS(GasLiquidEquilibrium surface, double bulkWaterMassFraction,
            double filmThicknessM, double diffusivityM2PerS)
        {
            if (surface == null) throw new ArgumentNullException(nameof(surface));
            if (!SiUnits.IsFinite(bulkWaterMassFraction) || bulkWaterMassFraction < 0 || bulkWaterMassFraction >= 1 ||
                !SiUnits.IsFinite(filmThicknessM) || filmThicknessM <= 0 ||
                !SiUnits.IsFinite(diffusivityM2PerS) || diffusivityM2PerS <= 0)
                throw new ArgumentException("Invalid humid gas film properties.");
            return surface.Gas.DensityKgM3 * diffusivityM2PerS / filmThicknessM *
                Math.Log((1 - bulkWaterMassFraction) / (1 - surface.WaterMassFraction));
        }

        public static double NormalVelocityJumpMPerS(GasLiquidEquilibrium surface, double massFluxKgM2PerS)
        {
            if (surface == null) throw new ArgumentNullException(nameof(surface));
            if (!SiUnits.IsFinite(massFluxKgM2PerS)) throw new ArgumentOutOfRangeException(nameof(massFluxKgM2PerS));
            return massFluxKgM2PerS * (1 / surface.Gas.DensityKgM3 - 1 / surface.Liquid.DensityKgM3);
        }
    }
}
