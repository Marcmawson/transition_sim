using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Materials;

namespace PhysicsSim.ThreePhase3D.PhaseChange
{
    // Equilibrium recovery for a pure-water inventory at prescribed pressure.
    // V21-V23 also require conduction and an evolving interface.
    public sealed class SolidLiquidInterface
    {
        private const double TripleTemperatureK = 273.16;
        private readonly IceIh ice = new IceIh();
        private readonly Water95 water = new Water95();
        private readonly double pressurePa, minimumK, maximumK;
        private readonly ThermodynamicState meltingIce, meltingLiquid;
        private readonly double minimumEnthalpy, maximumEnthalpy;

        public double MeltingTemperatureK { get; }
        public double IceMeltingEnthalpyJkg => meltingIce.EnthalpyJkg;
        public double IceMeltingEntropyJkgK => meltingIce.EntropyJkgK;
        public double LiquidMeltingEnthalpyJkg => meltingLiquid.EnthalpyJkg;
        public double LatentHeatJkg => LiquidMeltingEnthalpyJkg - IceMeltingEnthalpyJkg;
        public double LiquidMeltingDensityKgM3 => meltingLiquid.DensityKgM3;

        public SolidLiquidInterface(double pressurePa, double minimumK, double maximumK)
        {
            if (!SiUnits.IsFinite(pressurePa) || pressurePa < 611.66 || pressurePa > 5e6)
                throw new ArgumentOutOfRangeException(nameof(pressurePa));
            MeltingTemperatureK = PhaseEquilibrium.MeltingTemperature(pressurePa);
            if (MeltingTemperatureK < 273.15)
                throw new ArgumentOutOfRangeException(nameof(pressurePa), "Melting point is below the liquid EOS temperature range.");
            if (!SiUnits.IsFinite(minimumK) || minimumK < 200 || minimumK >= MeltingTemperatureK)
                throw new ArgumentOutOfRangeException(nameof(minimumK));
            if (!SiUnits.IsFinite(maximumK) || maximumK < TripleTemperatureK || maximumK > 300)
                throw new ArgumentOutOfRangeException(nameof(maximumK));
            this.pressurePa = pressurePa;
            this.minimumK = minimumK;
            this.maximumK = maximumK;
            meltingIce = ice.Evaluate(MeltingTemperatureK, pressurePa);
            meltingLiquid = LiquidAt(MeltingTemperatureK);
            minimumEnthalpy = ice.Evaluate(minimumK, pressurePa).EnthalpyJkg;
            maximumEnthalpy = water.Evaluate(maximumK, pressurePa, WaterBranch.Liquid).EnthalpyJkg;
            if (!(LatentHeatJkg > 0))
                throw new InvalidOperationException("Liquid enthalpy must exceed ice enthalpy at equilibrium.");
        }

        public State Recover(double massKg, double enthalpyJ)
        {
            if (!SiUnits.IsFinite(massKg) || massKg <= 0)
                throw new ArgumentOutOfRangeException(nameof(massKg));
            if (!SiUnits.IsFinite(enthalpyJ))
                throw new ArgumentOutOfRangeException(nameof(enthalpyJ));

            double h = enthalpyJ / massKg;
            double tolerance = 8e-15 * Math.Max(Math.Abs(minimumEnthalpy), Math.Abs(maximumEnthalpy));
            if (h < minimumEnthalpy - tolerance || h > maximumEnthalpy + tolerance)
                throw new ArgumentOutOfRangeException(nameof(enthalpyJ), "Inventory is outside the configured ice/liquid interval.");

            if (Math.Abs(h - IceMeltingEnthalpyJkg) <= tolerance) h = IceMeltingEnthalpyJkg;
            if (Math.Abs(h - LiquidMeltingEnthalpyJkg) <= tolerance) h = LiquidMeltingEnthalpyJkg;
            if (h < IceMeltingEnthalpyJkg)
            {
                var solid = Invert(h, minimumK, MeltingTemperatureK, t => ice.Evaluate(t, pressurePa));
                return new State(solid.TemperatureK, massKg, 0, massKg / solid.DensityKgM3, 0,
                    massKg * solid.EntropyJkgK);
            }
            if (h > LiquidMeltingEnthalpyJkg)
            {
                var liquid = Invert(h, MeltingTemperatureK, maximumK, LiquidAt);
                return new State(liquid.TemperatureK, 0, massKg, 0, massKg / liquid.DensityKgM3,
                    massKg * liquid.EntropyJkgK);
            }

            double liquidMass = massKg * ((h - IceMeltingEnthalpyJkg) / LatentHeatJkg);
            double iceMass = massKg - liquidMass;
            return new State(MeltingTemperatureK, iceMass, liquidMass,
                iceMass / meltingIce.DensityKgM3, liquidMass / meltingLiquid.DensityKgM3,
                iceMass * meltingIce.EntropyJkgK + liquidMass * meltingLiquid.EntropyJkgK);
        }

        private ThermodynamicState LiquidAt(double temperatureK)
        {
            // The upper endpoint was checked for liquid stability. In this
            // narrow interval, follow the dense EOS root directly.
            double density = 1000;
            ThermodynamicState best = default;
            double bestError = double.PositiveInfinity;
            for (int i = 0; i < 12; i++)
            {
                var state = water.ReferenceRhoT(temperatureK, density);
                double error = state.PressurePa - pressurePa;
                if (Math.Abs(error) < Math.Max(3e-5, pressurePa * 1e-11)) return state;
                if (Math.Abs(error) < bestError)
                {
                    best = state;
                    bestError = Math.Abs(error);
                }
                density -= error * state.DensityPressureDerivative;
                if (!SiUnits.IsFinite(density) || density < 990 || density > 1010) break;
            }
            // The dense root can oscillate at the numerical pressure noise floor.
            if (bestError < 1e-3) return best;
            throw new InvalidOperationException("The dense liquid density inversion did not converge.");
        }

        private static ThermodynamicState Invert(double target, double lowerK, double upperK,
            Func<double, ThermodynamicState> evaluate)
        {
            for (int i = 0; i < 42; i++)
            {
                double middleK = (lowerK + upperK) * 0.5;
                if (evaluate(middleK).EnthalpyJkg < target) lowerK = middleK;
                else upperK = middleK;
            }
            return evaluate((lowerK + upperK) * 0.5);
        }

        public readonly struct State
        {
            public readonly double TemperatureK, IceMassKg, LiquidMassKg, IceVolumeM3, LiquidVolumeM3;
            public readonly double EntropyJPerK;

            internal State(double temperatureK, double iceMassKg, double liquidMassKg,
                double iceVolumeM3, double liquidVolumeM3, double entropyJPerK)
            {
                TemperatureK = temperatureK;
                IceMassKg = iceMassKg;
                LiquidMassKg = liquidMassKg;
                IceVolumeM3 = iceVolumeM3;
                LiquidVolumeM3 = liquidVolumeM3;
                EntropyJPerK = entropyJPerK;
            }
        }
    }
}
