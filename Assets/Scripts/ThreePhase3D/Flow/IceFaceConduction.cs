using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Geometry;
using PhysicsSim.ThreePhase3D.PhaseChange;
using PhysicsSim.ThreePhase3D.Solids;

namespace PhysicsSim.ThreePhase3D.Flow
{
    public static class IceFaceConduction
    {
        // One-sided liquid conduction to an isothermal ice face at melting equilibrium.
        // The caller supplies the wetted area reconstructed on each interface patch.
        public static (CutCellInventories Fields, double HeatJ) Transfer(
            VariableCoefficientProjection plan, CutCellInventories fields, IceBody body,
            SolidLiquidInterface equilibrium, D3 outwardNormal, double seconds, double liquidDensityKgM3,
            double conductivityWmK, double[] liquidTemperatureK, Func<InterfacePatch, double> wettedAreaM2)
        {
            if (plan == null || fields == null || body == null || equilibrium == null ||
                liquidTemperatureK == null || wettedAreaM2 == null)
                throw new ArgumentNullException();
            if (!SiUnits.IsFinite(seconds) || seconds <= 0 ||
                !SiUnits.IsFinite(liquidDensityKgM3) || liquidDensityKgM3 <= 0 ||
                !SiUnits.IsFinite(conductivityWmK) || conductivityWmK <= 0)
                throw new ArgumentException("Conduction requires positive finite time and material properties.");
            if (liquidTemperatureK.Length != plan.GroupCount || fields.LiquidM3.Length != plan.GroupCount ||
                fields.LiquidEnergyJ.Length != plan.GroupCount)
                throw new ArgumentException("Liquid state must match the pressure groups.");

            var normal = outwardNormal.Unit();
            var gridFrame = plan.Grid.Frame;
            var localNormal = new D3(D3.Dot(normal, gridFrame.U), D3.Dot(normal, gridFrame.V),
                D3.Dot(normal, gridFrame.W));
            var extraction = new double[plan.GroupCount];
            foreach (var patch in plan.Geometry.Patches)
            {
                if (patch.Role != SolidRole.Ice || patch.FluidExposedAreaM2 <= 0 ||
                    D3.Dot(localNormal, patch.NormalLocal) < 1 - 1e-10) continue;
                double wet = wettedAreaM2(patch);
                if (!SiUnits.IsFinite(wet) || wet < 0 || wet > patch.FluidExposedAreaM2)
                    throw new ArgumentException("Wetted area must lie within the exposed patch.", nameof(wettedAreaM2));
                if (wet == 0) continue;
                int group = plan.GroupByCell[patch.Cell];
                if (group < 0 || !(fields.LiquidM3[group] > 0))
                    throw new InvalidOperationException("A wetted ice patch needs a liquid donor.");
                double excessK = liquidTemperatureK[group] - equilibrium.MeltingTemperatureK;
                if (!SiUnits.IsFinite(excessK))
                    throw new ArgumentException("Liquid temperature must be finite.", nameof(liquidTemperatureK));
                if (excessK <= 0) continue;
                if (!(patch.CellCentreNormalDistanceM > 0))
                    throw new InvalidOperationException("Conduction requires a positive normal sample distance.");
                extraction[group] += seconds * conductivityWmK * wet * excessK / patch.CellCentreNormalDistanceM;
            }

            double heat = 0;
            var energy = (double[])fields.LiquidEnergyJ.Clone();
            for (int group = 0; group < extraction.Length; group++)
            {
                if (extraction[group] == 0) continue;
                double minimumEnergy = fields.LiquidM3[group] * liquidDensityKgM3 * equilibrium.LiquidMeltingEnthalpyJkg;
                if (!SiUnits.IsFinite(minimumEnergy) || !SiUnits.IsFinite(energy[group]))
                    throw new ArgumentException("Liquid inventories must be finite.", nameof(fields));
                extraction[group] = Math.Min(extraction[group], Math.Max(0, energy[group] - minimumEnergy));
                heat += extraction[group];
            }

            double latentCapacity = body.MassKg * equilibrium.LatentHeatJkg;
            double scale = heat > latentCapacity ? latentCapacity / heat : 1;
            double transferred = 0;
            for (int group = 0; group < energy.Length; group++)
            {
                double amount = extraction[group] * scale;
                energy[group] -= amount;
                transferred += amount;
            }
            return (new CutCellInventories(fields.LiquidM3, fields.GasM3, energy,
                fields.GasEnergyJ, fields.VapourKg), transferred);
        }
    }
}
