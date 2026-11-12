using System;
using PhysicsSim.ThreePhase3D.Core;

namespace PhysicsSim.ThreePhase3D.Flow
{
    // Binary diffusion in a frozen gas volume. The caller supplies the
    // gas-open area and the current thermodynamic properties.
    public static class HumidGasDiffusion
    {
        public static CutCellInventories Advance(VariableCoefficientProjection plan, CutCellInventories fields,
            double seconds, double[] gasMassKg, double[] vapourEnthalpyJkg, double[] airEnthalpyJkg,
            Func<int, double> gasAreaM2, Func<int, double> diffusivityM2PerS)
        {
            if (plan == null || fields == null || gasMassKg == null || vapourEnthalpyJkg == null ||
                airEnthalpyJkg == null || gasAreaM2 == null || diffusivityM2PerS == null)
                throw new ArgumentNullException();
            int count = plan.GroupCount;
            if (!SiUnits.IsFinite(seconds) || seconds <= 0 || gasMassKg.Length != count ||
                vapourEnthalpyJkg.Length != count || airEnthalpyJkg.Length != count ||
                fields.GasM3.Length != count || fields.VapourKg.Length != count || fields.GasEnergyJ.Length != count)
                throw new ArgumentException("Gas state must match the pressure groups and have a positive timestep.");

            var fraction = new double[count];
            var density = new double[count];
            for (int i = 0; i < count; i++)
            {
                double mass = gasMassKg[i], volume = fields.GasM3[i], vapour = fields.VapourKg[i];
                if (!SiUnits.IsFinite(mass) || !SiUnits.IsFinite(volume) || !SiUnits.IsFinite(vapour) ||
                    !SiUnits.IsFinite(vapourEnthalpyJkg[i]) || !SiUnits.IsFinite(airEnthalpyJkg[i]) ||
                    !SiUnits.IsFinite(fields.GasEnergyJ[i]) ||
                    volume < 0 || mass < 0 || (volume == 0) != (mass == 0) || vapour < 0 || vapour > mass)
                    throw new ArgumentException("Invalid gas inventory or species enthalpy.");
                if (mass == 0) continue;
                fraction[i] = vapour / mass;
                density[i] = mass / volume;
            }

            var weights = new double[count];
            var nextVapour = (double[])fields.VapourKg.Clone();
            var nextEnergy = (double[])fields.GasEnergyJ.Clone();
            for (int face = 0; face < plan.FaceCount; face++)
            {
                var metric = plan.Faces[face];
                if (metric.Active == 0 || metric.Low == metric.High) continue;
                double area = gasAreaM2(face);
                if (!SiUnits.IsFinite(area) || area < 0 || area > metric.Area * (1 + 1e-7))
                    throw new ArgumentException("Gas area must fit inside the open face.");
                if (area == 0) continue;
                int low = metric.Low, high = metric.High;
                if (low < 0 || high < 0)
                    throw new NotSupportedException("An open gas diffusion face requires a reservoir model.");
                if (!(density[low] > 0 && density[high] > 0))
                    throw new ArgumentException("An open gas face requires gas on both sides.");
                double diffusivity = diffusivityM2PerS(face);
                if (!SiUnits.IsFinite(diffusivity) || diffusivity <= 0)
                    throw new ArgumentException("Binary diffusivity must be positive and finite.");
                double meanDensity = 2 / (1 / density[low] + 1 / density[high]);
                double weight = seconds * meanDensity * diffusivity * area / plan.Grid.Spacing[metric.Axis];
                weights[low] += weight;
                weights[high] += weight;
                double transfer = weight * (fraction[low] - fraction[high]);
                double enthalpy = transfer * (transfer >= 0
                    ? vapourEnthalpyJkg[low] - airEnthalpyJkg[high]
                    : vapourEnthalpyJkg[high] - airEnthalpyJkg[low]);
                nextVapour[low] -= transfer;
                nextVapour[high] += transfer;
                nextEnergy[low] -= enthalpy;
                nextEnergy[high] += enthalpy;
            }

            for (int group = 0; group < count; group++)
                if (!SiUnits.IsFinite(weights[group]) || weights[group] > gasMassKg[group] ||
                    !(nextVapour[group] >= 0 && nextVapour[group] <= gasMassKg[group]) ||
                    !SiUnits.IsFinite(nextEnergy[group]))
                    throw new InvalidOperationException("Gas diffusion exceeds the explicit timestep or inventory bound.");
            return new CutCellInventories(fields.LiquidM3, fields.GasM3, fields.LiquidEnergyJ, nextEnergy, nextVapour);
        }
    }
}
