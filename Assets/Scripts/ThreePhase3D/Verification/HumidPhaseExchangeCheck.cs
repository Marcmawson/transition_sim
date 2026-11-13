using System;
using PhysicsSim.ThreePhase3D.Diagnostics;
using System.Linq;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;
using PhysicsSim.ThreePhase3D.Geometry;
using PhysicsSim.ThreePhase3D.Materials;
namespace PhysicsSim.ThreePhase3D.Verification
{
    public static class HumidPhaseExchangeCheck
    {
        public static void RunTransientCondensation(Action<string> log)
        {
            if (log == null) throw new ArgumentNullException(nameof(log));
            const double pressure = 101325, surfaceK = 283.15, bulkK = 293.15;
            const double dryAir = 1e-5, area = 1e-4, film = 0.001;
            var humid = new HumidGas();
            var surface = humid.EquilibriumWithLiquid(surfaceK, pressure);
            double initialFraction = 0.85 * humid.EquilibriumWithLiquid(bulkK, pressure).WaterMassFraction;
            double initialVapour = dryAir * initialFraction / (1 - initialFraction);
            double equilibriumVapour = dryAir * surface.WaterMassFraction / (1 - surface.WaterMassFraction);
            double target = 0.5 * (initialVapour + equilibriumVapour);
            double diffusivity = HumidGas.ReferenceDiffusivity(surfaceK, pressure);
            double latent = new Water95().Saturation(surfaceK).Vapour.EnthalpyJkg - surface.Liquid.EnthalpyJkg;
            if (!(initialVapour > target && target > equilibriumVapour && latent > 0))
                throw new InvalidOperationException("The cooler-surface condensation benchmark is not bracketed.");

            // Integrate the separable sharp-interface film equation in vapour mass
            // to find the exact benchmark duration for half of the excess to condense.
            const int intervals = 1024;
            double width = (initialVapour - target) / intervals;
            double elapsed = 0;
            for (int n = 0; n <= intervals; n++)
            {
                double vapour = target + n * width;
                double removal = area * surface.Gas.DensityKgM3 * diffusivity / film *
                    Math.Log((dryAir + vapour) / (dryAir + equilibriumVapour));
                elapsed += (n == 0 || n == intervals ? 1 : n % 2 == 0 ? 2 : 4) / removal;
            }
            elapsed *= width / 3;

            double expected = initialVapour - target;
            var coarse = Advance(16);
            var fine = Advance(64);
            // The prescribed bulk and wall temperatures imply a thermostat.
            // Recover its heat removal from the fixed-pressure enthalpy balance.
            var initialGas = humid.ReferenceTp(bulkK, pressure, initialFraction);
            double finalVapour = initialVapour - fine.CondensedKg;
            double finalMass = dryAir + finalVapour;
            var finalGas = humid.ReferenceTp(bulkK, pressure, finalVapour / finalMass);
            double heatToWall = (dryAir + initialVapour) * initialGas.EnthalpyJkg -
                finalMass * finalGas.EnthalpyJkg - fine.CondensedKg * surface.Liquid.EnthalpyJkg;
            var initialStorage = new EntropyInventory();
            initialStorage.AddMaterial(dryAir + initialVapour, initialGas);
            var finalStorage = new EntropyInventory();
            finalStorage.AddMaterial(finalMass, finalGas);
            finalStorage.AddMaterial(fine.CondensedKg, surface.Liquid);
            var ledger = new ConservationLedger(new InventoryState(), new ConservationTolerance());
            ledger.RecordExternalEntropy(ExternalChannel.OpenBoundary, -heatToWall / surfaceK, 0);
            double production = ledger.InferEntropyProductionJPerK(initialStorage.TotalJPerK, finalStorage.TotalJPerK);
            if (!(heatToWall > 0 && production > 0))
                throw new InvalidOperationException("The thermostatted condensation entropy audit failed.");
            log(FormattableString.Invariant(
                $"S12 V28 condensation entropy passed: thermostat heat {heatToWall:G8} J, production {production:G8} J/K."));
            double coarseError = Math.Abs(coarse.CondensedKg / expected - 1);
            double fineError = Math.Abs(fine.CondensedKg / expected - 1);
            if (!(fineError < coarseError / 3.5 && fineError < 0.005))
                throw new InvalidOperationException("Transient condensation did not refine towards the sharp-interface reference.");
            Near(Math.Abs(fine.LatentJ / (expected * latent) - 1), 0.005);
            log(FormattableString.Invariant(
                $"S12 V28 transient condensation passed: {elapsed:G8} s, condensate {fine.CondensedKg:G8} kg, latent heat {fine.LatentJ:G8} J, 64-step reference error {fineError:P4}."));

            (double CondensedKg, double LatentJ) Advance(int steps)
            {
                double vapour = initialVapour, gasMass = dryAir + initialVapour, condensate = 0, heat = 0;
                double dt = elapsed / steps;
                for (int step = 0; step < steps; step++)
                {
                    double flux = HumidInterfaceFlux.WaterFluxKgM2PerS(surface, vapour / gasMass, film, diffusivity);
                    if (!(flux < 0)) throw new InvalidOperationException("The cooler interface stopped condensing prematurely.");
                    double liquidHeat = flux * latent;
                    var transfer = HumidInterfaceFlux.InventoryTransfer((surface, flux, liquidHeat, 0, latent),
                        area, dt, 0, vapour);
                    vapour += transfer.WaterKg;
                    gasMass += transfer.WaterKg;
                    condensate -= transfer.WaterKg;
                    heat -= liquidHeat * area * dt;
                    Near(Math.Abs(gasMass - vapour - dryAir), 1e-18);
                    Near(Math.Abs(vapour + condensate - initialVapour), 1e-18);
                    Near(Math.Abs(transfer.GasEnthalpyJ + transfer.LiquidEnthalpyJ), 1e-15);
                }
                Near(Math.Abs(heat - condensate * latent), 1e-14);
                return (condensate, heat);
            }
        }

        public static void RunSphericalFilm(Action<string> log)
        {
            if (log == null) throw new ArgumentNullException(nameof(log));
            const double radius = 0.001, outer = 0.005, temperature = 293.15;
            double surfaceArea = 4 * Math.PI * radius * radius;
            double equivalentFilm = radius * (1 - radius / outer);
            var humid = new HumidGas();
            foreach (double pressure in new[] { 80000.0, 101325.0 })
            {
                var surface = humid.EquilibriumWithLiquid(temperature, pressure);
                double diffusivity = HumidGas.ReferenceDiffusivity(temperature, pressure);
                double rho = surface.Gas.DensityKgM3;
                var cooled = HumidInterfaceFlux.SolveTemperature(pressure, 0.25 * surface.WaterMassFraction,
                    equivalentFilm, temperature, temperature, 1000, 25, 283.15, 298.15);
                if (!(cooled.Surface.TemperatureK < temperature && cooled.MassFluxKgM2PerS > 0))
                    throw new InvalidOperationException("The spherical film did not cool during evaporation.");
                Near(Math.Abs(cooled.LiquidHeatWm2 + cooled.GasHeatWm2 -
                    cooled.MassFluxKgM2PerS * cooled.LatentHeatJkg), 1e-7);
                foreach (double saturationRatio in new[] { 0.25, 0.75, 1.0 })
                {
                    double bulk = saturationRatio * surface.WaterMassFraction;
                    double surfacePotential = -Math.Log(1 - surface.WaterMassFraction);
                    double bulkPotential = -Math.Log(1 - bulk);
                    double analyticRate = 4 * Math.PI * rho * diffusivity *
                        (surfacePotential - bulkPotential) / (1 / radius - 1 / outer);
                    double modelRate = surfaceArea *
                        HumidInterfaceFlux.WaterFluxKgM2PerS(surface, bulk, equivalentFilm, diffusivity);
                    Near(Math.Abs(modelRate - analyticRate), 1e-19);
                    double coarseError = CheckShells(16);
                    double fineError = CheckShells(64);
                    if (analyticRate != 0 && !(fineError < coarseError / 12 && fineError < 0.001))
                        throw new InvalidOperationException("Spherical film rate did not converge on radial refinement.");
                    log(FormattableString.Invariant(
                        $"S12 V27 spherical film passed: {pressure:G} Pa, Y/Ysat {saturationRatio:G}, evaporation {modelRate:G8} kg/s, 64-shell relative error {fineError:P4}."));

                    double CheckShells(int count)
                    {
                        double dr = (outer - radius) / count;
                        var resistance = new double[count];
                        double total = 0;
                        for (int shell = 0; shell < count; shell++)
                        {
                            double midpoint = radius + (shell + 0.5) * dr;
                            resistance[shell] = dr / (4 * Math.PI * rho * diffusivity * midpoint * midpoint);
                            total += resistance[shell];
                        }
                        double rate = (surfacePotential - bulkPotential) / total;
                        double potential = surfacePotential;
                        for (int shell = 0; shell < count; shell++)
                        {
                            double midpoint = radius + (shell + 0.5) * dr;
                            double next = potential - rate * resistance[shell];
                            double air = Math.Exp(-0.5 * (potential + next));
                            double area = 4 * Math.PI * midpoint * midpoint;
                            double velocity = rate / (rho * area);
                            double speciesGradient = air * (next - potential) / dr;
                            // Dry-air advection and diffusion cancel throughout the shell.
                            Near(Math.Abs(area * rho * (velocity * air + diffusivity * speciesGradient)), 1e-19);
                            double node = radius + (shell + 1) * dr;
                            double exactPotential = surfacePotential - (surfacePotential - bulkPotential) *
                                (1 / radius - 1 / node) / (1 / radius - 1 / outer);
                            if (count == 64) Near(Math.Abs(Math.Exp(-next) - Math.Exp(-exactPotential)), 1e-5);
                            potential = next;
                        }
                        Near(Math.Abs(potential - bulkPotential), 1e-14);
                        return analyticRate == 0 ? Math.Abs(rate) : Math.Abs(rate / analyticRate - 1);
                    }
                }
            }
        }

        private static void Near(double error, double tolerance)
        {
            if (!(error <= tolerance))
                throw new InvalidOperationException($"Humid phase exchange error {error:G8} exceeds {tolerance:G8}.");
        }

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
