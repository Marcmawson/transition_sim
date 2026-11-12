using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;
using PhysicsSim.ThreePhase3D.Geometry;

namespace PhysicsSim.ThreePhase3D.Verification
{
    internal static class SeededFogCheck
    {
        internal static void Run(Action<string> log)
        {
            const double temperature = 290, duration = 0.001;
            const double referenceK = DropletGasCellExchange.ReferenceK;
            var seeds = new[] { (8e-6, temperature, 12), (10e-6, temperature, 20), (12e-6, temperature, 8) };
            var expanded = new (double, double, int)[40];
            int cursor = 0;
            foreach (var seed in seeds)
                for (int count = 0; count < seed.Item3; count++)
                    expanded[cursor++] = (seed.Item1, temperature, 1);
            double initialLiquid = Water(seeds);
            double dryAir = 10 * initialLiquid;
            foreach (double humidity in new[] { 0.005, 0.018 })
            {
                double initialVapour = dryAir * humidity;
                double initialGasEnergy = (dryAir * DropletGasCellExchange.AirCp +
                    initialVapour * DropletGasCellExchange.VapourCp) * (temperature - referenceK) +
                    initialVapour * DropletGasCellExchange.VapourReferenceEnthalpy;
                var grouped = Simulate(seeds, 32, dryAir, initialVapour, initialGasEnergy);
                var reference = Simulate(expanded, 32, dryAir, initialVapour, initialGasEnergy);
                var coarse = Simulate(seeds, 16, dryAir, initialVapour, initialGasEnergy);
                var fine = Simulate(seeds, 64, dryAir, initialVapour, initialGasEnergy);
                if (Math.Abs(grouped.VapourKg - reference.VapourKg) > 1e-12 * initialLiquid ||
                    Math.Abs(grouped.GasEnthalpyJ - reference.GasEnthalpyJ) > 1e-12 * initialGasEnergy ||
                    Math.Abs(grouped.VapourKg - fine.VapourKg) >= Math.Abs(coarse.VapourKg - fine.VapourKg))
                    throw new InvalidOperationException("S14 fog inventory did not agree across parcel counts or refine in time.");
                cursor = 0;
                for (int bin = 0; bin < seeds.Length; bin++)
                {
                    var parcel = grouped.Parcels[bin];
                    if (Math.Sign(parcel.RadiusM - seeds[bin].Item1) != Math.Sign(humidity - 0.01))
                        throw new InvalidOperationException("S14 fog seed size changed in the wrong direction.");
                    for (int count = 0; count < parcel.Multiplicity; count++)
                    {
                        var single = reference.Parcels[cursor++];
                        if (Math.Abs(single.RadiusM - parcel.RadiusM) > 1e-16 ||
                            Math.Abs(single.TemperatureK - parcel.TemperatureK) > 1e-9)
                            throw new InvalidOperationException("S14 fog radius or temperature changed with parcel regrouping.");
                    }
                }
                log(FormattableString.Invariant(
                    $"S14 V38 seeded fog passed: initial vapour/dry-air ratio {humidity:G3}, liquid change {initialVapour - fine.VapourKg:G8} kg, radii {fine.Parcels[0].RadiusM * 1e6:F6}/{fine.Parcels[1].RadiusM * 1e6:F6}/{fine.Parcels[2].RadiusM * 1e6:F6} um."));

            }

            CheckSampling();
            CheckSpatialTransport();
            CheckConversion();
            CheckDetachedLiquid();

            void CheckDetachedLiquid()
            {
                const double dx = 1e-4;
                var grid = new Grid3D(6, 1, 1, new D3(dx, dx, dx), RigidFrame.Identity);
                var plan = new VariableCoefficientProjection(CutCellGeometry.Build(grid),
                    BoundaryConditions3D.Closed, _ => 1.2, _ => 0);
                var liquid = new double[plan.GroupCount];
                var gas = new double[liquid.Length];
                var energy = new double[liquid.Length];
                var gasEnergy = new double[liquid.Length];
                var vapour = new double[liquid.Length];
                var anchors = new bool[liquid.Length];
                int bulk = plan.PressureGroup(0, 0, 0), neck = plan.PressureGroup(1, 0, 0);
                int detached = plan.PressureGroup(4, 0, 0);
                anchors[bulk] = true;
                liquid[bulk] = plan.Volumes[bulk];
                liquid[neck] = plan.Volumes[neck] * 0.25;
                liquid[detached] = plan.Volumes[detached] * 0.1;
                for (int group = 0; group < liquid.Length; group++)
                {
                    gas[group] = plan.Volumes[group] - liquid[group];
                    energy[group] = liquid[group] * DropletGasCellExchange.LiquidDensity *
                        DropletGasCellExchange.LiquidCp * (temperature - referenceK);
                    gasEnergy[group] = gas[group] * 1e5;
                    vapour[group] = gas[group] * 0.01;
                }
                var fields = new CutCellInventories(liquid, gas, energy, gasEnergy, vapour);
                var eligible = SeededFogCell.DetachedLiquidGroups(plan, fields, anchors, 0.3 * grid.CellVolume);
                if (eligible.Count != 1 || eligible[0].Length != 1 || eligible[0][0] != detached ||
                    SeededFogCell.DetachedLiquidGroups(plan, fields, anchors, 0.05 * grid.CellVolume).Count != 0)
                    throw new InvalidOperationException("S14 detached VOF classification failed bulk connectivity or volume threshold.");
                var momentum = new D3(1e-13, 0, 0);
                var converted = SeededFogCell.DebitResolvedLiquid(fields, eligible[0], momentum);
                var parcel = converted.Parcel;
                var restored = SeededFogCell.ToResolvedLiquid(parcel.RadiusM, parcel.TemperatureK, parcel.Multiplicity, parcel.Velocity);
                for (int group = 0; group < liquid.Length; group++)
                {
                    double filled = converted.Fields.LiquidM3[group] + converted.Fields.GasM3[group] +
                        converted.ParcelDisplacementM3[group];
                    if (Math.Abs(filled - plan.Volumes[group]) > 1e-14 * grid.CellVolume ||
                        converted.Fields.GasM3[group] != gas[group] || converted.Fields.GasEnergyJ[group] != gasEnergy[group] ||
                        converted.Fields.VapourKg[group] != vapour[group])
                        throw new InvalidOperationException("S14 conversion altered gas inventories or lost displaced volume.");
                }
                if (converted.Fields.LiquidM3[detached] != 0 || converted.Fields.LiquidEnergyJ[detached] != 0 ||
                    Math.Abs(restored.LiquidM3 / liquid[detached] - 1) > 1e-13 ||
                    Math.Abs(restored.EnthalpyJ / energy[detached] - 1) > 1e-13 ||
                    (restored.Momentum - momentum).Length > 1e-26)
                    throw new InvalidOperationException("S14 detached component conversion lost liquid inventories.");
                log(FormattableString.Invariant(
                    $"S14 detached VOF conversion passed: anchored neck retained, parcel displacement {restored.LiquidM3:G8} m3."));
                const double dt = 0.0004;
                int destination = plan.PressureGroup(5, 0, 0);
                var displaced = new double[plan.GroupCount];
                displaced[destination] = liquid[detached];
                var moved = new CutCellVofTransport(plan).AdvanceParcelDisplacement(converted.Fields,
                    converted.ParcelDisplacementM3, displaced, dt,
                    source => FlowCpuReference.Project(plan, new float[plan.FaceCount], dt, 4096, null, source));
                int face = plan.FaceIndex(0, 5, 0, 0);
                if (Math.Abs(moved.GasFaceM3[face] + displaced[destination]) > 1e-10 * grid.CellVolume)
                    throw new InvalidOperationException("S14 gas flux did not compensate for moving parcel displacement.");
                for (int group = 0; group < liquid.Length; group++)
                {
                    double expectedGas = plan.Volumes[group] - converted.Fields.LiquidM3[group] - displaced[group];
                    if (Math.Abs(moved.Fields.GasM3[group] - expectedGas) > 1e-10 * grid.CellVolume ||
                        Math.Abs(moved.Fields.GasEnergyJ[group] - expectedGas * 1e5) > 1e-10 * grid.CellVolume * 1e5 ||
                        Math.Abs(moved.Fields.VapourKg[group] - expectedGas * 0.01) > 1e-10 * grid.CellVolume * 0.01 ||
                        Math.Abs(moved.Fields.LiquidM3[group] - converted.Fields.LiquidM3[group]) > 1e-10 * grid.CellVolume)
                        throw new InvalidOperationException("S14 displacement transport did not close the phase inventories.");
                }
                log(FormattableString.Invariant(
                    $"S14 parcel displacement transport passed: swept gas {moved.GasFaceM3[face]:G8} m3."));
            }

            void CheckConversion()
            {
                double maxResidual = 0;
                foreach (double fraction in new[] { 0.001, 0.01, 0.1 })
                foreach (int multiplicity in new[] { 1, 8, 32 })
                {
                    double volume = 1e-12 * fraction;
                    double mass = volume * DropletGasCellExchange.LiquidDensity;
                    double enthalpy = mass * DropletGasCellExchange.LiquidCp * (temperature - referenceK);
                    var momentum = new D3(0.01, -0.02, 0.03) * mass;
                    var parcel = SeededFogCell.FromResolvedLiquid(volume, enthalpy, momentum, multiplicity);
                    var restored = SeededFogCell.ToResolvedLiquid(parcel.RadiusM, parcel.TemperatureK,
                        parcel.Multiplicity, parcel.Velocity);
                    double residual = Math.Max(Math.Abs(restored.LiquidM3 / volume - 1),
                        Math.Max(Math.Abs(restored.EnthalpyJ / enthalpy - 1),
                            (restored.Momentum - momentum).Length / momentum.Length));
                    maxResidual = Math.Max(maxResidual, residual);
                    if (residual > 1e-13 || parcel.Multiplicity != multiplicity)
                        throw new InvalidOperationException("S14 resolved-liquid conversion did not preserve its inventories.");
                }
                log(FormattableString.Invariant(
                    $"S14 resolved/parcel inventory conversion passed: 9 round trips, maximum relative residual {maxResidual:G5}."));
            }

            void CheckSpatialTransport()
            {
                const double dx = 1e-4, transverse = 1e-3;
                var grid = new Grid3D(3, 1, 1, new D3(dx, transverse, transverse), RigidFrame.Identity);
                var plan = new VariableCoefficientProjection(CutCellGeometry.Build(grid),
                    BoundaryConditions3D.Closed, _ => 1.2, _ => 0);
                int upstream = plan.PressureGroup(0, 0, 0), downstream = plan.PressureGroup(1, 0, 0);
                int untouched = plan.PressureGroup(2, 0, 0);
                var carrier = new D3(0.02, 0, 0);
                var initialParcels = new[]
                {
                    (new D3(dx - 7.5e-6, transverse * 0.5, transverse * 0.5), 8e-6, temperature, 12),
                    (new D3(dx - 13.5e-6, transverse * 0.5, transverse * 0.5), 10e-6, temperature, 20)
                };
                double cellAir = 1.2 * grid.CellVolume;
                var initialGas = new (double DryAirKg, double VapourKg, double EnthalpyJ)[plan.GroupCount];
                for (int group = 0; group < initialGas.Length; group++)
                {
                    double vapour = cellAir * (group == upstream ? 0.005 : 0.018);
                    double energy = (cellAir * DropletGasCellExchange.AirCp + vapour * DropletGasCellExchange.VapourCp) *
                        (temperature - referenceK) + vapour * DropletGasCellExchange.VapourReferenceEnthalpy;
                    initialGas[group] = (cellAir, vapour, energy);
                }
                var initial = Totals(initialParcels, initialGas);
                foreach (int steps in new[] { 10, 20 })
                {
                    var state = (Parcels: initialParcels, Gas: initialGas);
                    for (int step = 0; step < steps; step++)
                    {
                        state = SeededFogCell.AdvanceGrid(plan, state.Parcels, state.Gas, carrier,
                            101325, 0.026, 2.5e-5, duration / steps);
                        var total = Totals(state.Parcels, state.Gas);
                        if (Math.Abs(total.Water - initial.Water) > 1e-12 * initial.Water ||
                            Math.Abs(total.Energy - initial.Energy) > 1e-12 * initial.Energy)
                            throw new InvalidOperationException("V38 spatial fog transport failed water or enthalpy conservation.");
                    }
                    double evaporated = state.Gas[upstream].VapourKg - initialGas[upstream].VapourKg;
                    double condensed = initialGas[downstream].VapourKg - state.Gas[downstream].VapourKg;
                    if (!(evaporated > 0 && condensed > 0) || state.Gas[untouched] != initialGas[untouched])
                        throw new InvalidOperationException("V38 spatial fog did not exchange with the expected cells.");
                    for (int index = 0; index < state.Parcels.Length; index++)
                    {
                        var parcel = state.Parcels[index];
                        if (DropletGasCellExchange.PressureGroupAt(plan, parcel.Item1) != downstream ||
                            (parcel.Item1 - initialParcels[index].Item1 - carrier * duration).Length > 1e-16 ||
                            parcel.Item4 != initialParcels[index].Item4)
                            throw new InvalidOperationException("V38 spatial fog position or multiplicity did not match the reference.");
                    }
                    log(FormattableString.Invariant(
                        $"S14 V38 spatial fog passed: {steps} steps, upstream evaporation {evaporated:G8} kg, downstream condensation {condensed:G8} kg."));
                }
                CheckGasFeedback(10);
                CheckGasFeedback(20);
                CheckGasFeedback(10, moving: true);
                CheckGasFeedback(20, moving: true);
                CheckGasFeedback(10, moving: true, exitParcels: true);
                CheckGasFeedback(20, moving: true, exitParcels: true);
                CheckGasFeedback(10, moving: true, deposit: true);
                CheckGasFeedback(20, moving: true, deposit: true);

                void CheckGasFeedback(int steps, bool moving = false, bool exitParcels = false, bool deposit = false)
                {
                    const double pressure = 101325;
                    double dt = duration / steps;
                    var sides = new FlowBoundary[6];
                    sides[1] = new FlowBoundary(FlowBoundaryKind.OpenPressure);
                    var boundaries = new BoundaryConditions3D(sides);
                    var transportPlan = new VariableCoefficientProjection(plan.Geometry, boundaries, _ => 1.2, _ => 0);
                    var gas = ((double DryAirKg, double VapourKg, double EnthalpyJ)[])initialGas.Clone();
                    double startingAir = 0;
                    for (int group = 0; group < gas.Length; group++)
                    {
                        var cell = gas[group];
                        double scale = grid.CellVolume /
                            DropletGasCellExchange.GasVolumeM3(cell.DryAirKg, cell.VapourKg, cell.EnthalpyJ, pressure);
                        gas[group] = (cell.DryAirKg * scale, cell.VapourKg * scale, cell.EnthalpyJ * scale);
                        startingAir += gas[group].DryAirKg;
                    }
                    var reservoir = gas[untouched];
                    var initialTotals = Totals(initialParcels, gas);
                    var parcels = initialParcels;
                    if (exitParcels || deposit)
                    {
                        parcels = ((D3, double, double, int)[])initialParcels.Clone();
                        for (int index = 0; index < parcels.Length; index++)
                            parcels[index].Item1 = new D3(deposit ? parcels[index].Item2 + (index + 1) * 2e-6 :
                                3 * dx - (index + 1) * 2e-6,
                                transverse * 0.5, transverse * 0.5);
                    }
                    var momentum = new D3[gas.Length];
                    var parcelVelocity = new D3[parcels.Length];
                    D3 startingMomentum = default;
                    if (moving)
                    {
                        for (int index = 0; index < parcels.Length; index++)
                        {
                            parcelVelocity[index] = deposit ? -carrier : carrier;
                            startingMomentum += parcelVelocity[index] * (DropletGasCellExchange.Mass(parcels[index].Item2) * parcels[index].Item4);
                        }
                    }
                    D3 reaction = default, momentumOut = default;
                    double airOut = 0, vapourOut = 0, energyOut = 0;
                    long dropletsOut = 0;
                    double liquidOut = 0, liquidEnthalpyOut = 0;
                    D3 liquidMomentumOut = default;
                    (long Count, double Liquid, double Enthalpy, D3 Impulse) wall = default;
                    double filmLiquid = 0, filmEnthalpy = 0, filmVapour = 0;
                    double upstreamExchange = 0, downstreamExchange = 0, boundaryThroughput = 0;
                    for (int step = 0; step < steps; step++)
                    {
                        if (filmLiquid > 0)
                        {
                            var adjacent = gas[upstream];
                            var film = DropletGasCellExchange.AdvanceWallFilm(filmLiquid, filmEnthalpy,
                                adjacent.DryAirKg, adjacent.VapourKg, adjacent.EnthalpyJ,
                                pressure, 1e-10, 2e-5, 0.026, 2.5e-5, dt);
                            filmLiquid = film.LiquidKg;
                            filmEnthalpy = film.LiquidEnthalpyJ;
                            filmVapour += film.VapourKg - adjacent.VapourKg;
                            gas[upstream] = (adjacent.DryAirKg, film.VapourKg, film.GasEnthalpyJ);
                        }
                        double upstreamBefore = gas[upstream].VapourKg, downstreamBefore = gas[downstream].VapourKg;
                        if (moving)
                        {
                            var exchange = SeededFogCell.AdvanceMovingGrid(transportPlan,
                                parcels, gas, parcelVelocity, momentum, pressure, 0.026, 2.5e-5, 1.81e-5, dt,
                                stickingWall: deposit ? (0, false) : ((int, bool)?)null);
                            (parcels, gas, parcelVelocity, momentum) =
                                (exchange.Parcels, exchange.Gas, exchange.ParcelVelocity, exchange.GasMomentum);
                            dropletsOut += exchange.Outlet.Droplets;
                            liquidOut += exchange.Outlet.LiquidKg;
                            liquidEnthalpyOut += exchange.Outlet.EnthalpyJ;
                            liquidMomentumOut += exchange.Outlet.Momentum;
                            wall.Count += exchange.Wall.Droplets;
                            wall.Liquid += exchange.Wall.LiquidKg;
                            wall.Enthalpy += exchange.Wall.EnthalpyJ;
                            wall.Impulse += exchange.Wall.Momentum;
                            filmLiquid += exchange.Wall.LiquidKg;
                            filmEnthalpy += exchange.Wall.EnthalpyJ;
                        }
                        else
                            (parcels, gas) = SeededFogCell.AdvanceGrid(plan, parcels, gas, carrier,
                                pressure, 0.026, 2.5e-5, dt);
                        upstreamExchange += gas[upstream].VapourKg - upstreamBefore;
                        downstreamExchange += gas[downstream].VapourKg - downstreamBefore;
                        var volumes = new double[gas.Length];
                        var source = new double[gas.Length];
                        var inventory = new (double DryAirKg, double VapourKg, double EnthalpyJ, D3 MomentumKgMPerS)[gas.Length];
                        for (int group = 0; group < gas.Length; group++)
                        {
                            var cell = gas[group];
                            volumes[group] = DropletGasCellExchange.GasVolumeM3(cell.DryAirKg, cell.VapourKg, cell.EnthalpyJ, pressure);
                            source[group] = (plan.Volumes[group] - volumes[group]) / dt;
                            inventory[group] = (cell.DryAirKg, cell.VapourKg, cell.EnthalpyJ, momentum[group]);
                        }
                        var pressurePlan = new VariableCoefficientProjection(plan.Geometry, boundaries, cell =>
                        {
                            int group = plan.GroupByCell[cell];
                            return (gas[group].DryAirKg + gas[group].VapourKg) / volumes[group];
                        }, _ => 0);
                        var flow = FlowCpuReference.Project(pressurePlan,
                            HumidGasAdvection.PredictFaces(pressurePlan, inventory), dt, 4096, null, source);
                        var pressureReaction = HumidGasAdvection.ApplyPressure(pressurePlan, flow, inventory);
                        reaction += pressureReaction.WallReaction + pressureReaction.OpenBoundaryReaction;
                        var moved = HumidGasAdvection.Advance(pressurePlan, flow, volumes, inventory,
                            (reservoir.DryAirKg / grid.CellVolume, reservoir.VapourKg / grid.CellVolume,
                                reservoir.EnthalpyJ / grid.CellVolume, default));
                        airOut += moved.Outflow.DryAirKg;
                        vapourOut += moved.Outflow.VapourKg;
                        energyOut += moved.Outflow.EnthalpyJ;
                        momentumOut += moved.Outflow.MomentumKgMPerS;
                        boundaryThroughput += Math.Abs(moved.Outflow.DryAirKg);
                        double air = airOut;
                        D3 totalMomentum = momentumOut + reaction + liquidMomentumOut + wall.Impulse;
                        for (int group = 0; group < gas.Length; group++)
                        {
                            var cell = moved.Gas[group];
                            gas[group] = (cell.DryAirKg, cell.VapourKg, cell.EnthalpyJ);
                            momentum[group] = cell.MomentumKgMPerS;
                            air += cell.DryAirKg;
                            totalMomentum += momentum[group];
                        }
                        var total = Totals(parcels, gas);
                        long dropletCount = dropletsOut + wall.Count;
                        for (int index = 0; index < parcels.Length; index++)
                        {
                            dropletCount += parcels[index].Item4;
                            totalMomentum += parcelVelocity[index] *
                                (DropletGasCellExchange.Mass(parcels[index].Item2) * parcels[index].Item4);
                        }
                        if (dropletCount != 32 || Math.Abs(air - startingAir) > 1e-12 * startingAir ||
                            Math.Abs(total.Water + vapourOut + liquidOut + filmLiquid - initialTotals.Water) > 1e-12 * initialTotals.Water ||
                            Math.Abs(total.Energy + energyOut + liquidEnthalpyOut + filmEnthalpy - initialTotals.Energy) > 1e-12 * initialTotals.Energy ||
                            (totalMomentum - startingMomentum).Length > 1e-24)
                            throw new InvalidOperationException("V38 fog gas feedback failed conservation.");
                    }
                    if (deposit)
                    {
                        if (!(wall.Count == 32 && dropletsOut == 0 && parcels.Length == 0 &&
                            wall.Liquid > 0 && wall.Enthalpy > 0 && wall.Impulse.X < 0 && upstreamExchange > 0))
                            throw new InvalidOperationException("V38 sticking wall did not receive the parcel inventories.");
                        log(FormattableString.Invariant(
                            $"S14 V38 fog deposition passed: {steps} steps, droplets {wall.Count}, liquid {wall.Liquid:G8} kg, enthalpy {wall.Enthalpy:G8} J, wall impulse X {wall.Impulse.X:G8} N s."));
                        if (!(filmVapour > 0 && filmLiquid > 0 && filmEnthalpy < wall.Enthalpy) ||
                            Math.Abs(filmLiquid + filmVapour - wall.Liquid) > 1e-12 * wall.Liquid)
                            throw new InvalidOperationException("V38 wall film did not return deposited water to the gas.");
                        log(FormattableString.Invariant(
                            $"S14 V38 fog film re-evaporation passed: {steps} steps, vapour returned {filmVapour:G8} kg, net film enthalpy loss {wall.Enthalpy - filmEnthalpy:G8} J."));
                        return;
                    }
                    if (exitParcels)
                    {
                        if (!(dropletsOut == 32 && parcels.Length == 0 && parcelVelocity.Length == 0 &&
                            liquidOut > 0 && liquidEnthalpyOut > 0 && liquidMomentumOut.X > 0 && boundaryThroughput > 0))
                            throw new InvalidOperationException("V38 open boundary did not export the complete parcel ensemble.");
                        log(FormattableString.Invariant(
                            $"S14 V38 fog exit passed: {steps} steps, droplets {dropletsOut}, liquid {liquidOut:G8} kg, enthalpy {liquidEnthalpyOut:G8} J, momentum X {liquidMomentumOut.X:G8} N s."));
                        return;
                    }
                    if (dropletsOut != 0 || !(upstreamExchange > 0 && downstreamExchange < 0 && boundaryThroughput > 0) ||
                        gas[untouched] == reservoir)
                        throw new InvalidOperationException("V38 fog exchange did not drive transport into the unseeded cell and boundary.");
                    if (moving)
                    {
                        if (!(parcelVelocity[0].X > 0 && parcelVelocity[0].X < carrier.X) ||
                            DropletGasCellExchange.PressureGroupAt(plan, parcels[0].Item1) != downstream)
                            throw new InvalidOperationException("V38 moving parcel did not slow under drag or cross the face.");
                        log(FormattableString.Invariant(
                            $"S14 V38 moving fog passed: {steps} steps, first parcel speed {parcelVelocity[0].X:G8} m/s, downstream condensation {-downstreamExchange:G8} kg."));
                    }
                    else log(FormattableString.Invariant(
                        $"S14 V38 fog gas feedback passed: {steps} steps, downstream condensation {-downstreamExchange:G8} kg, signed boundary vapour {vapourOut:G8} kg."));
                }
            }

            (double Water, double Energy) Totals((D3, double, double, int)[] parcels,
                (double DryAirKg, double VapourKg, double EnthalpyJ)[] gas)
            {
                double water = 0, energy = 0;
                foreach (var parcel in parcels)
                {
                    double mass = DropletGasCellExchange.Mass(parcel.Item2) * parcel.Item4;
                    water += mass;
                    energy += mass * DropletGasCellExchange.LiquidCp * (parcel.Item3 - referenceK);
                }
                foreach (var cell in gas)
                {
                    water += cell.VapourKg;
                    energy += cell.EnthalpyJ;
                }
                return (water, energy);
            }

            void CheckSampling()
            {
                const int bins = 9, trials = 24, physicalCount = 2304;
                var exact = new (double, double, int)[bins];
                for (int bin = 0; bin < bins; bin++)
                    exact[bin] = ((6 + bin) * 1e-6, temperature, physicalCount / bins);
                double air = 10 * Water(exact), vapour = air * 0.018;
                double energy = (air * DropletGasCellExchange.AirCp + vapour * DropletGasCellExchange.VapourCp) *
                    (temperature - referenceK) + vapour * DropletGasCellExchange.VapourReferenceEnthalpy;
                var expected = Simulate(exact, 32, air, vapour, energy);
                double condensate = vapour - expected.VapourKg;
                if (!(condensate > 0)) throw new InvalidOperationException("The V38 sampling reference did not condense.");
                double[] referenceCdf = CumulativeSize(expected.Parcels);
                double previousSizeRms = double.PositiveInfinity, previousCondensateRms = double.PositiveInfinity;
                foreach (int count in new[] { 16, 64, 256 })
                {
                    double sizeError = 0, condensateError = 0;
                    for (int seed = 0; seed < trials; seed++)
                    {
                        var random = new Random(seed);
                        var sample = new (double, double, int)[count];
                        for (int index = 0; index < sample.Length; index++)
                            sample[index] = ((6 + random.Next(bins)) * 1e-6, temperature, physicalCount / count);
                        var actual = Simulate(sample, 32, air, vapour, energy);
                        double relative = (vapour - actual.VapourKg - condensate) / condensate;
                        condensateError += relative * relative;
                        var actualCdf = CumulativeSize(actual.Parcels);
                        for (int bin = 0; bin < actualCdf.Length; bin++)
                        {
                            double difference = actualCdf[bin] - referenceCdf[bin];
                            sizeError += difference * difference;
                        }
                    }
                    double sizeRms = Math.Sqrt(sizeError / (trials * referenceCdf.Length));
                    double condensateRms = Math.Sqrt(condensateError / trials);
                    if (!(sizeRms < previousSizeRms && condensateRms < previousCondensateRms))
                        throw new InvalidOperationException("V38 sampling errors did not decrease with parcel count.");
                    previousSizeRms = sizeRms;
                    previousCondensateRms = condensateRms;
                    log(FormattableString.Invariant(
                        $"S14 V38 fog sampling passed: {count} parcels, {trials} seeds, size CDF RMS {sizeRms:P3}, condensate RMS {condensateRms:P3}."));
                }
                if (previousSizeRms > 0.04 || previousCondensateRms > 0.04)
                    throw new InvalidOperationException("V38 fine fog sample exceeded its 4% RMS tolerances.");

                double[] CumulativeSize((double RadiusM, double TemperatureK, int Multiplicity)[] parcels)
                {
                    var fractions = new double[bins - 1];
                    for (int bin = 0; bin < fractions.Length; bin++)
                    {
                        double threshold = (6.5 + bin) * 1e-6;
                        int below = 0;
                        foreach (var parcel in parcels)
                            if (parcel.RadiusM < threshold) below += parcel.Multiplicity;
                        fractions[bin] = (double)below / physicalCount;
                    }
                    return fractions;
                }
            }

            ((double RadiusM, double TemperatureK, int Multiplicity)[] Parcels, double VapourKg, double GasEnthalpyJ)
                Simulate((double, double, int)[] parcels, int steps, double air, double vapour, double gasEnergy)
            {
                var state = (Parcels: parcels, VapourKg: vapour, GasEnthalpyJ: gasEnergy);
                double liquid = Water(parcels), totalEnergy = gasEnergy + Heat(parcels);
                for (int step = 0; step < steps; step++)
                {
                    state = SeededFogCell.Advance(state.Parcels, air, state.VapourKg, state.GasEnthalpyJ,
                        101325, 0.026, 2.5e-5, duration / steps);
                    if (Math.Abs(Water(state.Parcels) + state.VapourKg - liquid - vapour) > 1e-12 * liquid ||
                        Math.Abs(Heat(state.Parcels) + state.GasEnthalpyJ - totalEnergy) > 1e-12 * totalEnergy)
                        throw new InvalidOperationException("S14 fog failed water or enthalpy conservation.");
                }
                return state;
            }

            double Water((double Radius, double Temperature, int Count)[] parcels)
            {
                double total = 0;
                foreach (var parcel in parcels)
                    total += DropletGasCellExchange.Mass(parcel.Radius) * parcel.Count;
                return total;
            }

            double Heat((double Radius, double Temperature, int Count)[] parcels)
            {
                double total = 0;
                foreach (var parcel in parcels)
                    total += DropletGasCellExchange.Mass(parcel.Radius) * parcel.Count *
                        DropletGasCellExchange.LiquidCp * (parcel.Temperature - referenceK);
                return total;
            }
        }
    }
}
