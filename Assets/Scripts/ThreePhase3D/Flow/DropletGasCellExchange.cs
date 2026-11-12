using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Materials;

namespace PhysicsSim.ThreePhase3D.Flow
{
    // One well-mixed gas cell at prescribed pressure, with a constant-cp enthalpy model.
    internal static class DropletGasCellExchange
    {
        internal const double ReferenceK = 273.16, LiquidCp = 4182, AirCp = 1006, VapourCp = 1860;
        internal const double VapourReferenceEnthalpy = 2500900, LiquidDensity = 998;

        internal static (double RadiusM, double LiquidK, double VapourKg, double GasEnthalpyJ) Advance(
            double radiusM, double liquidK, double dryAirKg, double vapourKg, double gasEnthalpyJ,
            double pressurePa, double conductivity, double diffusivity, double dt)
        {
            if (!SiUnits.IsFinite(radiusM) || radiusM < 0 || !SiUnits.IsFinite(dryAirKg) || dryAirKg <= 0 ||
                !SiUnits.IsFinite(vapourKg) || vapourKg < 0 || !SiUnits.IsFinite(pressurePa) ||
                pressurePa < 80000 || pressurePa > 101325 || !SiUnits.IsFinite(dt) || dt <= 0)
                throw new ArgumentException("Invalid dilute droplet-cell inventory, pressure or timestep.");

            double gasCapacity = dryAirKg * AirCp + vapourKg * VapourCp;
            double gasK = ReferenceK + (gasEnthalpyJ - vapourKg * VapourReferenceEnthalpy) / gasCapacity;
            CheckTemperature(gasK);
            if (radiusM == 0) return (0, liquidK, vapourKg, gasEnthalpyJ);
            CheckTemperature(liquidK);

            double liquidMass = Mass(radiusM);
            double liquidCapacity = liquidMass * LiquidCp;
            double sensible = DropletSensibleHeat.EnergyIntoLiquidJ(radiusM, conductivity,
                liquidCapacity, gasCapacity, liquidK, gasK, dt);
            double heatedK = liquidK + sensible / liquidCapacity;
            double ambient = vapourKg / (dryAirKg + vapourKg);
            double surface = HumidGas.WaterMassFraction(SaturationPressurePa(heatedK) / pressurePa);
            double gasConstant = 8.31451 * ((1 - ambient) / HumidGas.Ma + ambient / HumidGas.Mw);
            double gasDensity = pressurePa / (gasConstant * (gasK - sensible / gasCapacity));
            double liquidH = LiquidCp * (heatedK - ReferenceK);
            double vapourH = VapourReferenceEnthalpy + VapourCp * (heatedK - ReferenceK);
            var transfer = DropletEvaporation.Advance(radiusM, LiquidDensity, gasDensity, diffusivity,
                Math.Max(surface, ambient), ambient, liquidH, vapourH, dt);
            double remainingMass = Mass(transfer.RadiusM);
            if (remainingMass <= 0)
                throw new InvalidOperationException("The coupled evaporation step crossed extinction. Reduce the timestep.");
            double cooledK = heatedK - transfer.HeatRequiredJ / (remainingMass * LiquidCp);
            CheckTemperature(cooledK);
            double updatedVapour = vapourKg + transfer.VapourKg;
            double updatedEnthalpy = gasEnthalpyJ - sensible + transfer.VapourEnthalpyJ;
            CheckTemperature(ReferenceK + (updatedEnthalpy - updatedVapour * VapourReferenceEnthalpy) /
                (dryAirKg * AirCp + updatedVapour * VapourCp));
            return (transfer.RadiusM, cooledK, updatedVapour, updatedEnthalpy);
        }

        internal static double Mass(double radius) => (4 * Math.PI / 3) * LiquidDensity * radius * radius * radius;

        // IAPWS SR1-86 (1992), equation 1.
        private static double SaturationPressurePa(double kelvin)
        {
            double theta = 1 - kelvin / 647.096;
            double exponent = -7.85951783 * theta + 1.84408259 * Math.Pow(theta, 1.5) -
                11.7866497 * Math.Pow(theta, 3) + 22.6807411 * Math.Pow(theta, 3.5) -
                15.9618719 * Math.Pow(theta, 4) + 1.80122502 * Math.Pow(theta, 7.5);
            return 22.064e6 * Math.Exp(647.096 / kelvin * exponent);
        }

        private static void CheckTemperature(double kelvin)
        {
            if (!(kelvin >= ReferenceK && kelvin <= 320))
                throw new InvalidOperationException("The droplet-cell temperature left the 273.16–320 K model range. Check inputs or reduce the timestep.");
        }
    }
}
