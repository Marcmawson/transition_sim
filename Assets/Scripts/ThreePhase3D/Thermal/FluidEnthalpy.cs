using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Materials;

namespace PhysicsSim.ThreePhase3D.Thermal
{
    // S09 isobaric reference. The prescribed transport density is the same
    // density used to convert the accepted VOF liquid volume into mass.
    public sealed class FluidEnthalpy
    {
        private readonly WaterPropertyTable table;
        private readonly double transportDensity;
        private readonly float[] values;

        public FluidEnthalpy(WaterPropertyTable table, double transportDensityKgM3)
        {
            this.table = table ?? throw new ArgumentNullException(nameof(table));
            if (table.Branch != WaterBranch.Liquid)
                throw new ArgumentException("A liquid water table is required.", nameof(table));
            if (!SiUnits.IsFinite(transportDensityKgM3) || transportDensityKgM3 <= 0)
                throw new ArgumentOutOfRangeException(nameof(transportDensityKgM3));

            transportDensity = transportDensityKgM3;
            values = table.CopyValues();
            for (int i = 1; i < table.Count; i++)
                if (SpecificEnthalpy(i) <= SpecificEnthalpy(i - 1))
                    throw new ArgumentException("Liquid enthalpy must increase with temperature.", nameof(table));
        }

        public double EnergyJ(double liquidVolumeM3, double temperatureK)
        {
            if (!SiUnits.IsFinite(liquidVolumeM3) || liquidVolumeM3 < 0)
                throw new ArgumentOutOfRangeException(nameof(liquidVolumeM3));
            if (!SiUnits.IsFinite(temperatureK) || temperatureK < table.MinimumK || temperatureK > table.MaximumK)
                throw new ArgumentOutOfRangeException(nameof(temperatureK));

            double position = (temperatureK - table.MinimumK) / table.StepK;
            int lower = Math.Min(table.Count - 2, (int)position);
            double h = SpecificEnthalpy(lower) + (position - lower) *
                (SpecificEnthalpy(lower + 1) - SpecificEnthalpy(lower));
            return liquidVolumeM3 * transportDensity * h;
        }

        // A dry cell has no liquid temperature. Reject inconsistent inventory
        // and out-of-range enthalpy instead of clamping or creating heat.
        public double? TemperatureK(double liquidVolumeM3, double energyJ)
        {
            if (!SiUnits.IsFinite(liquidVolumeM3) || liquidVolumeM3 < 0)
                throw new ArgumentOutOfRangeException(nameof(liquidVolumeM3));
            if (!SiUnits.IsFinite(energyJ))
                throw new ArgumentOutOfRangeException(nameof(energyJ));
            if (liquidVolumeM3 == 0)
            {
                if (energyJ != 0) throw new ArgumentException("A dry cell cannot contain liquid enthalpy.", nameof(energyJ));
                return null;
            }

            double h = energyJ / (liquidVolumeM3 * transportDensity);
            int lo = 0, hi = table.Count - 1;
            // Allow only rounding at the two endpoints.
            double minimum = SpecificEnthalpy(lo), maximum = SpecificEnthalpy(hi);
            double tolerance = 8e-15 * Math.Max(Math.Abs(minimum), Math.Abs(maximum));
            if (h < minimum - tolerance || h > maximum + tolerance)
                throw new ArgumentOutOfRangeException(nameof(energyJ), "Liquid enthalpy lies outside the isobaric table.");
            if (h <= minimum) return table.MinimumK;
            if (h >= maximum) return table.MaximumK;
            while (hi - lo > 1)
            {
                int mid = (lo + hi) / 2;
                if (SpecificEnthalpy(mid) <= h) lo = mid;
                else hi = mid;
            }

            double fraction = (h - SpecificEnthalpy(lo)) / (SpecificEnthalpy(hi) - SpecificEnthalpy(lo));
            return table.MinimumK + (lo + fraction) * table.StepK;
        }

        private double SpecificEnthalpy(int row) => values[row * WaterPropertyTable.Width + 1];
    }
}
