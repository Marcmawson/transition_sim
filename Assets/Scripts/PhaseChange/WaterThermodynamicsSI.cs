using System;

namespace PhaseTransition.Physics
{
    /// <summary>
    /// Near-atmospheric, SI-unit water/ice equilibrium helpers for the first beaker model.
    /// This is an enthalpy model, not a molecular or Navier-Stokes solver.
    /// </summary>
    public static class WaterThermodynamicsSI
    {
        public const double StandardGravity = 9.80665; // m s^-2
        public const double StandardAtmosphere = 101325.0; // Pa
        public const double WaterMolarMass = 0.01801528; // kg mol^-1
        public const double IceFusionEnthalpy = 333550.0; // J kg^-1, near 0 C and 1 atm
        public const double LiquidHeatCapacity = 4181.3; // J kg^-1 K^-1, near room temperature
        public const double IceCpA = 2.1128; // J mol^-1 K^-1; correlation intercept
        public const double IceCpB = 0.130484; // J mol^-1 K^-2; Cp = A + B*T, 170-270 K

        // IAPWS-IF97 saturation enthalpy data, kJ/kg, at 0,20,...,100 C.
        private static readonly double[] SatTemperatureC = { 0, 20, 40, 60, 80, 100 };
        private static readonly double[] SatVaporizationJPerKg =
            { 2500900, 2453500, 2406000, 2357700, 2308100, 2256400 };

        public enum Phase
        {
            SolidIce,
            MeltingMixture,
            Liquid,
            BoilingMixture,
            Vapour
        }

        public struct EquilibriumState
        {
            public Phase phase;
            public double temperatureC;
            public double iceMassFraction;
            public double liquidMassFraction;
            public double vapourMassFraction;
            public double saturationPressurePa;
        }

        /// <summary>Standard-atmosphere pressure for altitude in the troposphere (-500 to 11000 m).</summary>
        public static double AmbientPressureAtAltitudePa(double altitudeM)
        {
            const double seaLevelTemperatureK = 288.15;
            const double lapseRateKPerM = 0.0065;
            const double dryAirGasConstant = 287.05287; // J kg^-1 K^-1
            double h = Math.Max(-500.0, Math.Min(11000.0, altitudeM));
            double baseTerm = 1.0 - lapseRateKPerM * h / seaLevelTemperatureK;
            return StandardAtmosphere * Math.Pow(baseTerm,
                StandardGravity / (dryAirGasConstant * lapseRateKPerM));
        }

        /// <summary>Absolute pressure increase beneath a water surface, p = p_surface + rho*g*h.</summary>
        public static double HydrostaticPressurePa(double surfacePressurePa, double depthM,
            double densityKgPerM3 = 1000.0)
        {
            return surfacePressurePa + densityKgPerM3 * StandardGravity * Math.Max(0.0, depthM);
        }

        /// <summary>IAPWS-IF97 Region 4 saturation pressure. Input T is kelvin; valid 273.15..647.096 K.</summary>
        public static double SaturationPressurePa(double temperatureK)
        {
            if (temperatureK < 273.15 || temperatureK > 647.096)
                throw new ArgumentOutOfRangeException(nameof(temperatureK), "IF97 saturation relation is valid from 273.15 K to the critical point.");

            double n1 = 0.11670521452767e4;
            double n2 = -0.72421316703206e6;
            double n3 = -0.17073846940092e2;
            double n4 = 0.12020824702470e5;
            double n5 = -0.32325550322333e7;
            double n6 = 0.14915108613530e2;
            double n7 = -0.48232657361591e4;
            double n8 = 0.40511340542057e6;
            double n9 = -0.23855557567849;
            double n10 = 0.65017534844798e3;
            double theta = temperatureK + n9 / (temperatureK - n10);
            double a = theta * theta + n1 * theta + n2;
            double b = n3 * theta * theta + n4 * theta + n5;
            double c = n6 * theta * theta + n7 * theta + n8;
            double pMpa = Math.Pow(2.0 * c / (-b + Math.Sqrt(b * b - 4.0 * a * c)), 4.0);
            return pMpa * 1_000_000.0;
        }

        /// <summary>Invert IF97 saturation pressure by bisection; valid between triple and critical pressures.</summary>
        public static double SaturationTemperatureK(double pressurePa)
        {
            const double triplePointPressurePa = 611.657;
            const double criticalPointPressurePa = 22.064e6;
            if (pressurePa < triplePointPressurePa || pressurePa > criticalPointPressurePa)
                throw new ArgumentOutOfRangeException(nameof(pressurePa), "Pressure is outside the IF97 liquid-vapour coexistence curve.");
            double low = 273.15;
            double high = 647.096;
            for (int i = 0; i < 80; i++)
            {
                double mid = 0.5 * (low + high);
                if (SaturationPressurePa(mid) < pressurePa) low = mid;
                else high = mid;
            }
            return 0.5 * (low + high);
        }

        /// <summary>
        /// Ice enthalpy relative to liquid water at 0 C. Uses the published ice heat-capacity
        /// linear correlation in its 170-270 K range, integrated analytically.
        /// </summary>
        public static double IceSpecificEnthalpyJPerKg(double temperatureK)
        {
            if (temperatureK < 170.0 || temperatureK > 273.15)
                throw new ArgumentOutOfRangeException(nameof(temperatureK), "Ice Cp correlation is only used here from 170 K to the melting point.");
            double t0 = 273.15;
            double molarDelta = IceCpA * (temperatureK - t0) +
                                0.5 * IceCpB * (temperatureK * temperatureK - t0 * t0);
            return molarDelta / WaterMolarMass - IceFusionEnthalpy;
        }

        /// <summary>
        /// Maps specific enthalpy to equilibrium phase fractions at a supplied absolute pressure.
        /// Enthalpy reference is liquid water at 0 C. Vapour/liquid coexistence is represented
        /// only over the IF97 saturation table range (0-100 C).
        /// </summary>
        public static EquilibriumState StateFromSpecificEnthalpy(double enthalpyJPerKg, double pressurePa)
        {
            double tsatK = SaturationTemperatureK(pressurePa);
            double tsatC = tsatK - 273.15;
            if (tsatC < 0.0 || tsatC > 100.0)
                throw new ArgumentOutOfRangeException(nameof(pressurePa), "The initial beaker approximation supports boiling points from 0 to 100 C.");

            double meltEnthalpy = 0.0;
            double boilStartEnthalpy = LiquidHeatCapacity * tsatC;
            double latentVaporization = Interpolate(SatTemperatureC, SatVaporizationJPerKg, tsatC);
            double boilEndEnthalpy = boilStartEnthalpy + latentVaporization;
            EquilibriumState state = new EquilibriumState { saturationPressurePa = pressurePa };

            if (enthalpyJPerKg < -IceFusionEnthalpy)
            {
                // Invert the monotonic ice enthalpy correlation with bisection.
                double target = enthalpyJPerKg;
                double lo = 170.0, hi = 273.15;
                for (int i = 0; i < 64; i++)
                {
                    double mid = 0.5 * (lo + hi);
                    if (IceSpecificEnthalpyJPerKg(mid) < target) lo = mid;
                    else hi = mid;
                }
                state.phase = Phase.SolidIce;
                state.temperatureC = 0.5 * (lo + hi) - 273.15;
                state.iceMassFraction = 1.0;
            }
            else if (enthalpyJPerKg < meltEnthalpy)
            {
                // The interval [-L_f,0] is the ice-water coexistence plateau at 0 C.
                state.phase = Phase.MeltingMixture;
                state.temperatureC = 0.0;
                state.iceMassFraction = -enthalpyJPerKg / IceFusionEnthalpy;
                state.liquidMassFraction = 1.0 - state.iceMassFraction;
            }
            else if (enthalpyJPerKg < boilStartEnthalpy)
            {
                state.phase = Phase.Liquid;
                state.temperatureC = enthalpyJPerKg / LiquidHeatCapacity;
                state.liquidMassFraction = 1.0;
            }
            else if (enthalpyJPerKg < boilEndEnthalpy)
            {
                state.phase = Phase.BoilingMixture;
                state.temperatureC = tsatC;
                state.liquidMassFraction = 1.0 - (enthalpyJPerKg - boilStartEnthalpy) / latentVaporization;
                state.vapourMassFraction = 1.0 - state.liquidMassFraction;
            }
            else
            {
                state.phase = Phase.Vapour;
                state.temperatureC = tsatC + (enthalpyJPerKg - boilEndEnthalpy) / 2000.0;
                state.vapourMassFraction = 1.0;
            }
            return state;
        }

        private static double Interpolate(double[] x, double[] y, double value)
        {
            if (value <= x[0]) return y[0];
            for (int i = 1; i < x.Length; i++)
            {
                if (value <= x[i])
                {
                    double f = (value - x[i - 1]) / (x[i] - x[i - 1]);
                    return y[i - 1] + f * (y[i] - y[i - 1]);
                }
            }
            return y[y.Length - 1];
        }
    }
}
