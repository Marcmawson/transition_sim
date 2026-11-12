using System;
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
                CheckProjectedPhaseExchange(solved, dt, project, log);
            }
            log(FormattableString.Invariant(
                $"S12 humid interface passed: evaporation {evaporation:G8}, condensation {condensation:G8} kg/m2/s, Stefan jump {velocityJump:G8} m/s."));
            log(FormattableString.Invariant(
                $"S12 humid heat balance passed: evaporating interface {cooling.Surface.TemperatureK:G9} K, condensing interface {heating.Surface.TemperatureK:G9} K."));
        }

        private static void CheckProjectedPhaseExchange(
            (GasLiquidEquilibrium Surface, double MassFluxKgM2PerS, double LiquidHeatWm2,
                double GasHeatWm2, double LatentHeatJkg) solved, double dt,
            Func<VariableCoefficientProjection, float[], double, double[], FlowStep> project, Action<string> log)
        {
            var grid = new Grid3D(1, 1, 1, new D3(0.01, 0.01, 0.01), RigidFrame.Identity);
            var sides = new FlowBoundary[6];
            sides[3] = new FlowBoundary(FlowBoundaryKind.OpenPressure);
            var plan = new VariableCoefficientProjection(CutCellGeometry.Build(grid),
                new BoundaryConditions3D(sides), _ => 1.2, _ => 0);
            var state = new CutCellInventories(new[] { grid.CellVolume * 0.5 }, new[] { grid.CellVolume * 0.5 },
                new[] { 0.1 }, new[] { 0.2 }, new[] { 5e-9 });
            double density = solved.Surface.Liquid.DensityKgM3;
            var transfer = HumidInterfaceFlux.InventoryTransfer(solved, grid.FaceArea(1), dt,
                state.LiquidM3[0] * density, state.VapourKg[0]);
            var increments = new CutCellInventories(new[] { transfer.LiquidVolumeM3 }, new[] { transfer.GasVolumeM3 },
                new[] { transfer.LiquidEnthalpyJ }, new[] { transfer.GasEnthalpyJ }, new[] { transfer.WaterKg });
            var result = new CutCellVofTransport(plan).AdvanceReconstructedPhaseExchange(state, dt, density,
                _ => solved,
                source => project(plan, new float[plan.FaceCount], dt, source),
                _ => new CutCellInflow(0, 0, 3e5, 0.01));
            var advected = result.Transport;
            for (int field = 0; field < state.Planes.Length; field++)
                if (Math.Abs(advected.Fields.Planes[field][0] - state.Planes[field][0] -
                    increments.Planes[field][0] - advected.BoundaryInflow[field]) > 1e-13)
                    throw new InvalidOperationException("Projected phase exchange did not close its inventory ledger.");
            double expansion = transfer.GasVolumeM3 + transfer.LiquidVolumeM3;
            double pressureVolume = plan.OutwardFlux(0, result.Pressure.VelocityMPerS) * dt;
            if (Math.Abs(pressureVolume - expansion) > Math.Abs(expansion) * 1e-5 ||
                Math.Abs(advected.BoundaryInflow[0] + advected.BoundaryInflow[1] + expansion) > 1e-17 ||
                Math.Abs(advected.Fields.LiquidM3[0] + advected.Fields.GasM3[0] - grid.CellVolume) > 1e-17)
                throw new InvalidOperationException("Projected phase exchange did not close its volume ledger.");
            log(FormattableString.Invariant(
                $"S12 projected phase exchange passed: net volume {expansion:G8} m3, boundary compensation {-expansion:G8} m3."));
        }
    }
}
