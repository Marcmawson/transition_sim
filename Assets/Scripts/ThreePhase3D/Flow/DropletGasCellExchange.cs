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
            double gasK = GasTemperature(dryAirKg, vapourKg, gasEnthalpyJ);
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
            CheckTemperature(GasTemperature(dryAirKg, updatedVapour, updatedEnthalpy));
            return (transfer.RadiusM, cooledK, updatedVapour, updatedEnthalpy);
        }

        internal static double Mass(double radius) => (4 * Math.PI / 3) * LiquidDensity * radius * radius * radius;

        // Stationary, well-mixed wall liquid with a prescribed planar gas film.
        internal static (double LiquidKg, double LiquidEnthalpyJ, double VapourKg, double GasEnthalpyJ) AdvanceWallFilm(
            double liquidKg, double liquidEnthalpyJ, double dryAirKg, double vapourKg, double gasEnthalpyJ,
            double pressurePa, double areaM2, double thicknessM, double conductivity, double diffusivity, double dt)
        {
            if (!Positive(liquidKg) || !Positive(dryAirKg) || !SiUnits.IsFinite(vapourKg) || vapourKg < 0 ||
                !(pressurePa >= 80000 && pressurePa <= 101325) || !Positive(areaM2) ||
                !Positive(thicknessM) || !Positive(conductivity) || !Positive(diffusivity) || !Positive(dt))
                throw new ArgumentException("Invalid wall-film inventory or transport properties.");

            double liquidK = ReferenceK + liquidEnthalpyJ / (liquidKg * LiquidCp);
            double gasK = GasTemperature(dryAirKg, vapourKg, gasEnthalpyJ);
            CheckTemperature(liquidK);
            CheckTemperature(gasK);
            double surface = HumidGas.WaterMassFraction(SaturationPressurePa(liquidK) / pressurePa);
            double ambient = vapourKg / (dryAirKg + vapourKg);
            double density = (dryAirKg + vapourKg) / GasVolumeM3(dryAirKg, vapourKg, gasEnthalpyJ, pressurePa);
            double areaTimePerThickness = areaM2 * dt / thicknessM;
            double evaporated = density * diffusivity * areaTimePerThickness *
                Math.Max(0, Math.Log((1 - ambient) / (1 - surface)));
            double sensible = conductivity * areaTimePerThickness * (gasK - liquidK);
            double transfer = evaporated * (VapourReferenceEnthalpy + VapourCp * (liquidK - ReferenceK)) - sensible;
            double remaining = liquidKg - evaporated;
            if (!(remaining > 0))
                throw new InvalidOperationException("The wall film would run dry in one step. Reduce the timestep.");
            double remainingEnthalpy = liquidEnthalpyJ - transfer;
            double updatedVapour = vapourKg + evaporated, updatedGasEnthalpy = gasEnthalpyJ + transfer;
            CheckTemperature(ReferenceK + remainingEnthalpy / (remaining * LiquidCp));
            CheckTemperature(GasTemperature(dryAirKg, updatedVapour, updatedGasEnthalpy));
            return (remaining, remainingEnthalpy, updatedVapour, updatedGasEnthalpy);
        }

        private static bool Positive(double value) => SiUnits.IsFinite(value) && value > 0;

        internal static int PressureGroupAt(VariableCoefficientProjection plan, D3 worldPosition)
        {
            var grid = plan.Grid;
            D3 local = grid.Frame.ToLocal(worldPosition);
            double x = local.X / grid.Spacing.X, y = local.Y / grid.Spacing.Y, z = local.Z / grid.Spacing.Z;
            if (!(x >= 0 && x < grid.Cells.X && y >= 0 && y < grid.Cells.Y && z >= 0 && z < grid.Cells.Z))
                throw new InvalidOperationException("Droplet centre is outside the gas grid.");
            int group = plan.PressureGroup((int)x, (int)y, (int)z);
            if (group < 0)
                throw new InvalidOperationException("Droplet centre has no fluid pressure group.");
            return group;
        }

        internal static bool ExitsOpenBoundary(VariableCoefficientProjection plan, D3 start, D3 end)
        {
            var grid = plan.Grid;
            D3 localStart = grid.Frame.ToLocal(start), localEnd = grid.Frame.ToLocal(end);
            double first = double.PositiveInfinity;
            bool open = false;
            for (int axis = 0; axis < 3; axis++)
            {
                int count = axis == 0 ? grid.Cells.X : axis == 1 ? grid.Cells.Y : grid.Cells.Z;
                double limit = grid.Spacing[axis] * count;
                double target = localEnd[axis];
                if (target >= 0 && target < limit) continue;
                bool positive = target >= limit;
                double fraction = ((positive ? limit : 0) - localStart[axis]) / (target - localStart[axis]);
                bool sideOpen = plan.Boundaries.Side(axis, positive).Kind == FlowBoundaryKind.OpenPressure;
                if (fraction < first)
                {
                    first = fraction;
                    open = sideOpen;
                }
                else if (fraction == first) open &= sideOpen;
            }
            return open;
        }

        internal static bool ContactsBoxWall(VariableCoefficientProjection plan, int axis, bool positive,
            D3 start, D3 end, double radiusM)
        {
            if (plan.HasEmbeddedSolid || plan.IsPeriodic ||
                plan.Boundaries.Side(axis, positive).Kind == FlowBoundaryKind.OpenPressure)
                throw new ArgumentException("Droplet contact requires a solid exterior face of a Cartesian box.");
            var grid = plan.Grid;
            int count = axis == 0 ? grid.Cells.X : axis == 1 ? grid.Cells.Y : grid.Cells.Z;
            double wall = positive ? grid.Spacing[axis] * count : 0;
            double direction = positive ? 1 : -1;
            double before = direction * (wall - grid.Frame.ToLocal(start)[axis]);
            double after = direction * (wall - grid.Frame.ToLocal(end)[axis]);
            return after < before && after <= radiusM;
        }

        internal static double GasVolumeM3(double dryAirKg, double vapourKg, double gasEnthalpyJ, double pressurePa)
        {
            double temperature = GasTemperature(dryAirKg, vapourKg, gasEnthalpyJ);
            CheckTemperature(temperature);
            return 8.31451 * temperature / pressurePa * (dryAirKg / HumidGas.Ma + vapourKg / HumidGas.Mw);
        }

        internal static double ProjectionSourceM3PerS(double dryAirKg, double beforeVapourKg, double beforeEnthalpyJ,
            double afterVapourKg, double afterEnthalpyJ, double pressurePa, double dt)
        {
            if (!SiUnits.IsFinite(dt) || dt <= 0)
                throw new ArgumentOutOfRangeException(nameof(dt));
            // Projection enforces outward flux + source = 0.
            return (GasVolumeM3(dryAirKg, beforeVapourKg, beforeEnthalpyJ, pressurePa) -
                GasVolumeM3(dryAirKg, afterVapourKg, afterEnthalpyJ, pressurePa)) / dt;
        }

        private static double GasTemperature(double dryAirKg, double vapourKg, double gasEnthalpyJ) =>
            ReferenceK + (gasEnthalpyJ - vapourKg * VapourReferenceEnthalpy) / (dryAirKg * AirCp + vapourKg * VapourCp);

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
