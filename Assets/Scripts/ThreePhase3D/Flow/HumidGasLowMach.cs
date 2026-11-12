using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Materials;

namespace PhysicsSim.ThreePhase3D.Flow
{
    public static class HumidGasLowMach
    {
        // Pressure projection imposes outward flux + source = 0.
        // The caller handles boundary compatibility or the closed-domain thermodynamic pressure.
        public static double ProjectionSourceM3PerS(ThermodynamicState before, ThermodynamicState after,
            double gasMassKg, double seconds)
        {
            if (!SiUnits.IsFinite(gasMassKg) || gasMassKg < 0 ||
                !SiUnits.IsFinite(seconds) || seconds <= 0)
                throw new ArgumentException("Gas mass and timestep must be finite and physically admissible.");
            return gasMassKg * (1 / Density(before) - 1 / Density(after)) / seconds;
        }

        // Acceleration after subtracting the hydrostatic gradient of the ambient gas.
        public static D3 BuoyancyMPerS2(ThermodynamicState local, ThermodynamicState ambient, D3 gravityMPerS2)
        {
            double localDensity = Density(local);
            return gravityMPerS2 * ((localDensity - Density(ambient)) / localDensity);
        }

        private static double Density(ThermodynamicState state)
        {
            if (!SiUnits.IsFinite(state.DensityKgM3) || state.DensityKgM3 <= 0)
                throw new ArgumentException("A positive finite EOS density is required.");
            return state.DensityKgM3;
        }
    }
}
