using System;
using System.Linq;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;
using PhysicsSim.ThreePhase3D.Geometry;
using PhysicsSim.ThreePhase3D.Materials;
namespace PhysicsSim.ThreePhase3D.Verification
{
    public static class HumidPhaseExchangeCheck
    {
        public static void Run(Func<VariableCoefficientProjection, float[], double, double[], FlowStep> project,
            Action<string> log)
        {
            if (project == null || log == null) throw new ArgumentNullException();
            const double pressure = 101325, film = 0.01;
            var model = new HumidGas();
            var warm = model.EquilibriumWithLiquid(293.15, pressure);
            var cool = model.EquilibriumWithLiquid(283.15, pressure);
            double bulk = (warm.WaterMassFraction + cool.WaterMassFraction) * 0.5;
            double warmD = HumidGas.ReferenceDiffusivity(warm.TemperatureK, pressure);
            double coolD = HumidGas.ReferenceDiffusivity(cool.TemperatureK, pressure);
            double evaporation = HumidInterfaceFlux.WaterFluxKgM2PerS(warm, bulk, film, warmD);
            double condensation = HumidInterfaceFlux.WaterFluxKgM2PerS(cool, bulk, film, coolD);
            double equilibrium = HumidInterfaceFlux.WaterFluxKgM2PerS(warm, warm.WaterMassFraction, film, warmD);
            double velocityJump = HumidInterfaceFlux.NormalVelocityJumpMPerS(warm, evaporation);
            if (!(evaporation > 0 && condensation < 0 && velocityJump > 0) || equilibrium != 0 ||
                Math.Abs(2 * HumidInterfaceFlux.WaterFluxKgM2PerS(warm, bulk, 2 * film, warmD) - evaporation) > 1e-15)
                throw new InvalidOperationException("Humid interface failed equilibrium, flux direction or film thickness scaling.");
            var cooling = HumidInterfaceFlux.SolveTemperature(pressure, bulk, film,
                warm.TemperatureK, warm.TemperatureK, 100, 10, 283.15, 298.15);
            var heating = HumidInterfaceFlux.SolveTemperature(pressure, bulk, film,
                cool.TemperatureK, cool.TemperatureK, 100, 10, 278.15, 293.15);
            if (!(cooling.Surface.TemperatureK < warm.TemperatureK && cooling.MassFluxKgM2PerS > 0 &&
                heating.Surface.TemperatureK > cool.TemperatureK && heating.MassFluxKgM2PerS < 0) ||
                Math.Abs(cooling.LiquidHeatWm2 + cooling.GasHeatWm2 - cooling.MassFluxKgM2PerS * cooling.LatentHeatJkg) > 1e-7 ||
                Math.Abs(heating.LiquidHeatWm2 + heating.GasHeatWm2 - heating.MassFluxKgM2PerS * heating.LatentHeatJkg) > 1e-7)
                throw new InvalidOperationException("Latent heat failed to cool evaporation or warm condensation.");
            foreach (var solved in new[] { cooling, heating })
            {
                const double area = 0.001, dt = 0.1;
                var transfer = HumidInterfaceFlux.InventoryTransfer(solved, area, dt, 0.001, 0.001);
                if (Math.Abs(transfer.LiquidEnthalpyJ + transfer.GasEnthalpyJ) > 1e-11 ||
                    Math.Abs(transfer.LiquidVolumeM3 * solved.Surface.Liquid.DensityKgM3 + transfer.WaterKg) > 1e-18 ||
                    Math.Abs(transfer.GasVolumeM3 + transfer.LiquidVolumeM3 -
                        HumidInterfaceFlux.NormalVelocityJumpMPerS(solved.Surface, solved.MassFluxKgM2PerS) * area * dt) > 1e-18)
                    throw new InvalidOperationException("Humid interface inventory transfer did not close.");
                double bulkK = solved.MassFluxKgM2PerS > 0 ? warm.TemperatureK : cool.TemperatureK;
                CheckProjectedPhaseExchange(solved, dt, bulkK, bulk, film, project, log, false);
                CheckProjectedPhaseExchange(solved, dt, bulkK, bulk, film, project, log, true);
            }
            log(FormattableString.Invariant(
                $"S12 humid interface passed: evaporation {evaporation:G8}, condensation {condensation:G8} kg/m2/s, Stefan jump {velocityJump:G8} m/s."));
            log(FormattableString.Invariant(
                $"S12 humid heat balance passed: evaporating interface {cooling.Surface.TemperatureK:G9} K, condensing interface {heating.Surface.TemperatureK:G9} K."));
        }

        private static void CheckProjectedPhaseExchange(
            (GasLiquidEquilibrium Surface, double MassFluxKgM2PerS, double LiquidHeatWm2,
                double GasHeatWm2, double LatentHeatJkg) solved, double dt, double bulkK,
            double bulkFraction, double film,
            Func<VariableCoefficientProjection, float[], double, double[], FlowStep> project, Action<string> log,
            bool aligned)
        {
            var grid = new Grid3D(1, aligned ? 2 : 1, 1, new D3(0.01, 0.01, 0.01), RigidFrame.Identity);
            var sides = new FlowBoundary[6];
            sides[3] = new FlowBoundary(FlowBoundaryKind.OpenPressure);
            var plan = new VariableCoefficientProjection(CutCellGeometry.Build(grid),
                new BoundaryConditions3D(sides), _ => 1.2, _ => 0);
            double pressure = solved.Surface.PressurePa;
            double density = solved.Surface.Liquid.DensityKgM3;
            var liquidState = new Water95().Evaluate(bulkK, pressure, WaterBranch.Liquid);
            var gasState = new HumidGas().ReferenceTp(bulkK, pressure, bulkFraction);
            var state = PlanarLiquidInventory.Create(plan, new D3(0, aligned ? 0.01 : 0.005, 0),
                new D3(0, 1, 0), liquidState.EnthalpyJkg * density,
                gasState.EnthalpyJkg * gasState.DensityKgM3, bulkFraction * gasState.DensityKgM3);
            int liquid = plan.GroupByCell[grid.Cells.Index(0, 0, 0)];
            int gas = plan.GroupByCell[grid.Cells.Index(0, aligned ? 1 : 0, 0)];
            double lower = solved.MassFluxKgM2PerS > 0 ? bulkK - 10 : bulkK - 5;
            double upper = solved.MassFluxKgM2PerS > 0 ? bulkK + 5 : bulkK + 10;
            var gasMass = Array.ConvertAll(state.GasM3, volume => volume * gasState.DensityKgM3);
            var recovered = HumidInterfaceFlux.SolveFromInventories(state, liquid, gas,
                density, gasMass[gas], pressure, film, 100, 10, lower, upper);
            if (Math.Abs(recovered.Surface.TemperatureK - solved.Surface.TemperatureK) > 1e-8 ||
                Math.Abs(recovered.MassFluxKgM2PerS - solved.MassFluxKgM2PerS) > 1e-12)
                throw new InvalidOperationException("Recovered thermal state changed the interface solution.");
            var transfer = HumidInterfaceFlux.InventoryTransfer(recovered, grid.FaceArea(1), dt,
                state.LiquidM3[liquid] * density, state.VapourKg[gas]);
            var result = new CutCellVofTransport(plan).AdvanceInventoryPhaseExchange(state, dt, density,
                gasMass, pressure, film, 100, 10, lower, upper,
                source => project(plan, new float[plan.FaceCount], dt, source),
                _ => new CutCellInflow(0, 0, 3e5, 0.01));
            var advected = result.Transport;
            var increments = new[] { transfer.LiquidVolumeM3, transfer.GasVolumeM3,
                transfer.LiquidEnthalpyJ, transfer.GasEnthalpyJ, transfer.WaterKg };
            for (int field = 0; field < state.Planes.Length; field++)
                if (Math.Abs(advected.Fields.Planes[field].Sum() - state.Planes[field].Sum() -
                    increments[field] - advected.BoundaryInflow[field]) > 1e-13)
                    throw new InvalidOperationException("Projected phase exchange did not close its inventory ledger.");
            double expansion = transfer.GasVolumeM3 + transfer.LiquidVolumeM3;
            if (Math.Abs(advected.BoundaryInflow[0] + advected.BoundaryInflow[1] + expansion) > 1e-17)
                throw new InvalidOperationException("Projected phase exchange did not close its volume ledger.");
            for (int group = 0; group < plan.GroupCount; group++)
            {
                double expected = (group == liquid ? transfer.LiquidVolumeM3 : 0) +
                    (group == gas ? transfer.GasVolumeM3 : 0);
                double projected = plan.OutwardFlux(group, result.Pressure.VelocityMPerS) * dt;
                if (Math.Abs(projected - expected) > Math.Abs(expansion) * 1e-5 ||
                    Math.Abs(advected.Fields.LiquidM3[group] + advected.Fields.GasM3[group] - grid.CellVolume) > 1e-17)
                    throw new InvalidOperationException("Projected phase exchange did not close the local volume balance.");
            }
            log(FormattableString.Invariant(
                $"S12 {(aligned ? "face-aligned" : "mixed-cell")} phase exchange passed: net volume {expansion:G8} m3, boundary compensation {-expansion:G8} m3."));
        }
    }
}
