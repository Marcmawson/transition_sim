using System;
using PhysicsSim.ThreePhase3D.Core;

namespace PhysicsSim.ThreePhase3D.Flow
{
    // Binary diffusion in a frozen gas volume. The caller supplies the
    // gas-open area and the current thermodynamic properties.
    public static class HumidGasDiffusion
    {
        public readonly struct Reservoir
        {
            public readonly double DensityKgM3, WaterMassFraction, VapourEnthalpyJkg, AirEnthalpyJkg;

            public Reservoir(double densityKgM3, double waterMassFraction, double vapourEnthalpyJkg, double airEnthalpyJkg)
            {
                if (!SiUnits.IsFinite(densityKgM3) || densityKgM3 <= 0 ||
                    !SiUnits.IsFinite(waterMassFraction) || waterMassFraction < 0 || waterMassFraction > 1 ||
                    !SiUnits.IsFinite(vapourEnthalpyJkg) || !SiUnits.IsFinite(airEnthalpyJkg))
                    throw new ArgumentException("Invalid gas diffusion reservoir.");
                DensityKgM3 = densityKgM3;
                WaterMassFraction = waterMassFraction;
                VapourEnthalpyJkg = vapourEnthalpyJkg;
                AirEnthalpyJkg = airEnthalpyJkg;
            }
        }

        public static CutCellInventories AdvanceReconstructed(VariableCoefficientProjection plan, CutCellInventories fields,
            double seconds, double[] gasMassKg, double[] vapourEnthalpyJkg, double[] airEnthalpyJkg,
            Func<int, double> diffusivityM2PerS)
        {
            var areas = new CutCellVofTransport(plan).ReconstructGasFaceAreas(fields);
            return Advance(plan, fields, seconds, gasMassKg, vapourEnthalpyJkg, airEnthalpyJkg,
                face => areas[face], diffusivityM2PerS);
        }

        public static CutCellInventories Advance(VariableCoefficientProjection plan, CutCellInventories fields,
            double seconds, double[] gasMassKg, double[] vapourEnthalpyJkg, double[] airEnthalpyJkg,
            Func<int, double> gasAreaM2, Func<int, double> diffusivityM2PerS) =>
            AdvanceCore(plan, fields, seconds, gasMassKg, vapourEnthalpyJkg, airEnthalpyJkg,
                gasAreaM2, diffusivityM2PerS, null).Fields;

        public static (CutCellInventories Fields, double VapourInKg, double EnthalpyInJ) AdvanceOpen(
            VariableCoefficientProjection plan, CutCellInventories fields, double seconds,
            double[] gasMassKg, double[] vapourEnthalpyJkg, double[] airEnthalpyJkg,
            Func<int, double> diffusivityM2PerS, Func<int, Reservoir> reservoir)
        {
            if (reservoir == null) throw new ArgumentNullException(nameof(reservoir));
            var areas = new CutCellVofTransport(plan).ReconstructGasFaceAreas(fields);
            return AdvanceCore(plan, fields, seconds, gasMassKg, vapourEnthalpyJkg, airEnthalpyJkg,
                face => areas[face], diffusivityM2PerS, reservoir);
        }

        private static (CutCellInventories Fields, double VapourInKg, double EnthalpyInJ) AdvanceCore(
            VariableCoefficientProjection plan, CutCellInventories fields,
            double seconds, double[] gasMassKg, double[] vapourEnthalpyJkg, double[] airEnthalpyJkg,
            Func<int, double> gasAreaM2, Func<int, double> diffusivityM2PerS, Func<int, Reservoir> reservoir)
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
            double vapourIn = 0, energyIn = 0;
            for (int face = 0; face < plan.FaceCount; face++)
            {
                var metric = plan.Faces[face];
                if (metric.Active == 0 || metric.Low == metric.High) continue;
                double area = gasAreaM2(face);
                if (!SiUnits.IsFinite(area) || area < 0 || area > metric.Area * (1 + 1e-7))
                    throw new ArgumentException("Gas area must fit inside the open face.");
                if (area == 0) continue;
                int low = metric.Low, high = metric.High;
                bool boundary = low < 0 || high < 0;
                if (boundary && reservoir == null)
                    throw new NotSupportedException("An open gas diffusion face requires a reservoir model.");
                var exterior = boundary ? reservoir(face) : default;
                double rhoLow = low < 0 ? exterior.DensityKgM3 : density[low];
                double rhoHigh = high < 0 ? exterior.DensityKgM3 : density[high];
                if (!(rhoLow > 0 && rhoHigh > 0))
                    throw new ArgumentException("An open gas face requires gas on both sides.");
                double diffusivity = diffusivityM2PerS(face);
                if (!SiUnits.IsFinite(diffusivity) || diffusivity <= 0)
                    throw new ArgumentException("Binary diffusivity must be positive and finite.");
                double meanDensity = 2 / (1 / rhoLow + 1 / rhoHigh);
                double distance = plan.Grid.Spacing[metric.Axis] * (boundary ? 0.5 : 1);
                double weight = seconds * meanDensity * diffusivity * area / distance;
                double yLow = low < 0 ? exterior.WaterMassFraction : fraction[low];
                double yHigh = high < 0 ? exterior.WaterMassFraction : fraction[high];
                double transfer = weight * (yLow - yHigh);
                double enthalpy = transfer * (transfer >= 0
                    ? (low < 0 ? exterior.VapourEnthalpyJkg : vapourEnthalpyJkg[low]) -
                        (high < 0 ? exterior.AirEnthalpyJkg : airEnthalpyJkg[high])
                    : (high < 0 ? exterior.VapourEnthalpyJkg : vapourEnthalpyJkg[high]) -
                        (low < 0 ? exterior.AirEnthalpyJkg : airEnthalpyJkg[low]));
                Add(low, -transfer, -enthalpy);
                Add(high, transfer, enthalpy);

                void Add(int group, double massChange, double energyChange)
                {
                    if (group < 0)
                    {
                        vapourIn -= massChange;
                        energyIn -= energyChange;
                        return;
                    }
                    weights[group] += weight;
                    nextVapour[group] += massChange;
                    nextEnergy[group] += energyChange;
                }
            }

            for (int group = 0; group < count; group++)
                if (!SiUnits.IsFinite(weights[group]) || weights[group] > gasMassKg[group] ||
                    !(nextVapour[group] >= 0 && nextVapour[group] <= gasMassKg[group]) ||
                    !SiUnits.IsFinite(nextEnergy[group]))
                    throw new InvalidOperationException("Gas diffusion exceeds the explicit timestep or inventory bound.");
            return (new CutCellInventories(fields.LiquidM3, fields.GasM3, fields.LiquidEnergyJ, nextEnergy, nextVapour),
                vapourIn, energyIn);
        }
    }
}
