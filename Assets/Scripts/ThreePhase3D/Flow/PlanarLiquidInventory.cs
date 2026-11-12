using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Geometry;

namespace PhysicsSim.ThreePhase3D.Flow
{
    public static class PlanarLiquidInventory
    {
        // Seed the liquid half-space using exact cut-cell volumes, then
        // aggregate into the pressure groups used by the transport solver.
        public static CutCellInventories Create(VariableCoefficientProjection plan, D3 surfacePointM,
            D3 upwardNormal, double liquidEnergyJPerM3, double gasEnergyJPerM3, double vapourKgPerM3)
        {
            if (plan == null) throw new ArgumentNullException(nameof(plan));
            if (!SiUnits.IsFinite(liquidEnergyJPerM3) || !SiUnits.IsFinite(gasEnergyJPerM3) ||
                !SiUnits.IsFinite(vapourKgPerM3) || vapourKgPerM3 < 0)
                throw new ArgumentException("Invalid phase energy or vapour density.");
            var grid = plan.Grid;
            var frame = grid.Frame;
            var plane = new ClipPlane(new D3(D3.Dot(upwardNormal, frame.U),
                D3.Dot(upwardNormal, frame.V), D3.Dot(upwardNormal, frame.W)), frame.ToLocal(surfacePointM));
            var clipping = new CutCellVofGeometry(plan.Geometry);
            var liquid = new double[plan.GroupCount];
            var gas = new double[plan.GroupCount];
            var liquidEnergy = new double[plan.GroupCount];
            var gasEnergy = new double[plan.GroupCount];
            var vapour = new double[plan.GroupCount];
            for (int cell = 0; cell < plan.GroupByCell.Length; cell++)
            {
                int group = plan.GroupByCell[cell];
                if (group < 0) continue;
                double volume = plan.Geometry.FluidVolumeM3[cell];
                double wet = clipping.LiquidVolume(cell, plane);
                double tolerance = 1e-10 * grid.CellVolume;
                if (!SiUnits.IsFinite(wet) || wet < -tolerance || wet > volume + tolerance)
                    throw new InvalidOperationException("Clipped liquid volume is outside the accessible cell.");
                liquid[group] += Math.Max(0, Math.Min(volume, wet));
            }

            for (int group = 0; group < plan.GroupCount; group++)
            {
                liquid[group] = Math.Min(plan.Volumes[group], liquid[group]);
                gas[group] = plan.Volumes[group] - liquid[group];
                liquidEnergy[group] = liquid[group] * liquidEnergyJPerM3;
                gasEnergy[group] = gas[group] * gasEnergyJPerM3;
                vapour[group] = gas[group] * vapourKgPerM3;
            }
            return new CutCellInventories(liquid, gas, liquidEnergy, gasEnergy, vapour);
        }
    }
}
