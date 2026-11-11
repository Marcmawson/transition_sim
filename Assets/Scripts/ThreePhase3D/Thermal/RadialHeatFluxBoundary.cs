using System;
using PhysicsSim.ThreePhase3D.Core;

namespace PhysicsSim.ThreePhase3D.Thermal
{
    // S08/V18 reference for circular, axisymmetric external-base faces.
    public static class RadialHeatFluxBoundary
    {
        // Each returned value is the integral of the normalized Gaussian over one
        // annular face, in W. The last edge is the heated disk's outer radius.
        public static double[] GaussianFacePowerW(double absorbedPowerW, double widthM, double[] outerEdgesM)
        {
            if (!SiUnits.IsFinite(absorbedPowerW) || absorbedPowerW < 0)
                throw new ArgumentOutOfRangeException(nameof(absorbedPowerW));
            if (!SiUnits.IsFinite(widthM) || widthM <= 0)
                throw new ArgumentOutOfRangeException(nameof(widthM));
            if (outerEdgesM == null || outerEdgesM.Length == 0)
                throw new ArgumentException("At least one annular face is required.", nameof(outerEdgesM));

            double previous = 0;
            foreach (double edge in outerEdgesM)
            {
                if (!SiUnits.IsFinite(edge) || edge <= previous)
                    throw new ArgumentException("Outer edges must be finite and strictly increasing.", nameof(outerEdgesM));
                previous = edge;
            }

            // Scale the integral by pi R² so a broad profile has a finite
            // uniform-flux limit even when (R / width)² rounds to zero.
            double radius = previous;
            double scaledRadius = radius / widthM;
            double exponent = .5 * scaledRadius * scaledRadius;
            var facePower = new double[outerEdgesM.Length];
            if (double.IsPositiveInfinity(exponent))
            {
                double innerTail = 1;
                for (int i = 0; i < facePower.Length; i++)
                {
                    double edge = outerEdgesM[i] / widthM;
                    double outerTail = Math.Exp(-.5 * edge * edge);
                    facePower[i] = absorbedPowerW * (innerTail - outerTail);
                    innerTail = outerTail;
                }
                return facePower;
            }

            double normalization = IntegralFactor(exponent);
            double inner = 0;
            for (int i = 0; i < facePower.Length; i++)
            {
                double outer = outerEdgesM[i] / radius;
                double areaFraction = (outer - inner) * (outer + inner);
                double weight = areaFraction * Math.Exp(-exponent * inner * inner) *
                    IntegralFactor(exponent * areaFraction) / normalization;
                facePower[i] = absorbedPowerW * weight;
                inner = outer;
            }
            return facePower;
        }

        private static double IntegralFactor(double x)
        {
            // (1 - exp(-x)) / x, evaluated without cancellation near zero.
            return x < 1e-4 ? 1 - x * (.5 - x / 6) : (1 - Math.Exp(-x)) / x;
        }
    }
}
