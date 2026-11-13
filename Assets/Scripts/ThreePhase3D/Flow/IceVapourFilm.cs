using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Materials;

namespace PhysicsSim.ThreePhase3D.Flow
{
    // Isothermal planar stagnant gas film. Positive water transfer is sublimation.
    internal static class IceVapourFilm
    {
        internal static (double IceKg, double IceEnthalpyJ, double VapourKg, double GasEnthalpyJ, double HeatIntoIceJ)
            AdvanceLumped(GasIceEquilibrium surface, double ambientFraction, double diffusivity, double thickness,
                double area, double dt, double iceKg, double vapourKg, double vapourEnthalpyJkg,
                double gasTemperatureK, double conductanceWm2K)
        {
            if (!Positive(gasTemperatureK) || !Positive(conductanceWm2K))
                throw new ArgumentException("Ice thermal coupling requires positive gas temperature and conductance.");
            var exchange = Exchange(surface, ambientFraction, diffusivity, thickness, area, dt, iceKg, vapourKg,
                vapourEnthalpyJkg);
            double remaining = iceKg - exchange.VapourKg;
            if (!(remaining > 0)) throw new InvalidOperationException("The lumped ice step requires remaining ice.");
            double heat = conductanceWm2K * area * dt * (gasTemperatureK - surface.TemperatureK);
            double enthalpy = iceKg * surface.Ice.EnthalpyJkg + heat - exchange.GasEnthalpyJ;
            return (remaining, enthalpy, exchange.VapourKg, exchange.GasEnthalpyJ, heat);
        }

        internal static (double VapourKg, double IceEnthalpyJ, double GasEnthalpyJ, double SurfaceHeatJ,
            double PhaseFrontShiftM) Exchange(
            GasIceEquilibrium surface, double ambientFraction, double diffusivity, double thickness,
            double area, double dt, double iceKg, double vapourKg, double vapourEnthalpyJkg)
        {
            if (surface == null) throw new ArgumentNullException(nameof(surface));
            double iceH = surface.Ice.EnthalpyJkg;
            if (!(ambientFraction >= 0 && ambientFraction < 1) ||
                !Positive(diffusivity) || !Positive(thickness) || !Positive(area) || !Positive(dt) ||
                !SiUnits.IsFinite(iceKg) || iceKg < 0 || !SiUnits.IsFinite(vapourKg) || vapourKg < 0 ||
                !Positive(vapourEnthalpyJkg - iceH))
                throw new ArgumentException("Invalid ice-vapour film properties or inventories.");

            double mass = surface.Gas.DensityKgM3 * diffusivity / thickness *
                Math.Log((1 - ambientFraction) / (1 - surface.WaterMassFraction)) * area * dt;
            if (!SiUnits.IsFinite(mass) || mass > iceKg || -mass > vapourKg)
                throw new InvalidOperationException("Ice-vapour exchange would overdraw a donor. Reduce the timestep.");

            // The prescribed-temperature surface receives heat from a reservoir.
            // SurfaceHeatJ is positive when that reservoir supplies heat.
            return (mass, -mass * iceH, mass * vapourEnthalpyJkg, mass * (vapourEnthalpyJkg - iceH),
                -mass / (surface.Ice.DensityKgM3 * area));
        }

        private static bool Positive(double value) => SiUnits.IsFinite(value) && value > 0;
    }
}
