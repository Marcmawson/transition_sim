using System;

namespace PhysicsSim.ThreePhase3D.Core
{
    public static class SiUnits
    {
        public const double CelsiusOffsetK = 273.15;

        // Physics-ID: SI-CONVERSIONS
        // Implements: R004 R043
        // Model: Specification.md section 5; T_K = T_C + 273.15.
        // Sources: D00, BIPM SI Brochure 9th ed. v4.01 (2026).
        // Source-Locators: sections 2.3.1 (kelvin), 2.3.4 (derived units), 3 (prefixes).
        // Inputs-SI: Celsius [deg C]; lengths [m]; pressure [Pa]; density [kg/m3].
        // Outputs-SI: temperature [K]; area [m2]; volume [m3]; mass [kg].
        // State-Ownership: none; pure conversions.
        // Conservation: no inventory mutation; pressure gauge reference is explicit.
        // Discretisation: double-precision scalar arithmetic, no physical discretisation.
        // Validity: finite values; positive density and absolute temperature/pressure.
        // Validation: V01
        // Evidence: PENDING
        // Status: IMPLEMENTED_UNVERIFIED
        // Limitations: these conversions do not supply an equation of state.
        public static double CelsiusToKelvin(double celsius)
        {
            double result = celsius + CelsiusOffsetK;
            RequirePositive(result, nameof(celsius));
            return result;
        }

        public static double KelvinToCelsius(double kelvin)
        {
            RequirePositive(kelvin, nameof(kelvin));
            return kelvin - CelsiusOffsetK;
        }

        public static double AbsolutePressure(double gaugePa, double referencePa)
        {
            RequireFinite(gaugePa, nameof(gaugePa));
            RequirePositive(referencePa, nameof(referencePa));
            double absolute = gaugePa + referencePa;
            RequirePositive(absolute, nameof(absolute));
            return absolute;
        }

        public static double GaugePressure(double absolutePa, double referencePa)
        {
            RequirePositive(absolutePa, nameof(absolutePa));
            RequirePositive(referencePa, nameof(referencePa));
            return absolutePa - referencePa;
        }

        public static double MassFromVolume(double volumeM3, double densityKgPerM3)
        {
            RequireNonnegative(volumeM3, nameof(volumeM3));
            RequirePositive(densityKgPerM3, nameof(densityKgPerM3));
            return volumeM3 * densityKgPerM3;
        }

        public static double VolumeFromMass(double massKg, double densityKgPerM3)
        {
            RequireNonnegative(massKg, nameof(massKg));
            RequirePositive(densityKgPerM3, nameof(densityKgPerM3));
            return massKg / densityKgPerM3;
        }

        public static double CylinderVolume(double radiusM, double heightM)
        {
            RequirePositive(radiusM, nameof(radiusM));
            RequirePositive(heightM, nameof(heightM));
            return Math.PI * radiusM * radiusM * heightM;
        }

        public static bool IsFinite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
        public static void RequireFinite(double value, string name)
        {
            if (!IsFinite(value)) throw new ArgumentOutOfRangeException(name, "Must be finite.");
        }
        public static void RequirePositive(double value, string name)
        {
            if (!IsFinite(value) || value <= 0) throw new ArgumentOutOfRangeException(name, "Must be finite and positive.");
        }
        public static void RequireNonnegative(double value, string name)
        {
            if (!IsFinite(value) || value < 0) throw new ArgumentOutOfRangeException(name, "Must be finite and nonnegative.");
        }
    }
}
