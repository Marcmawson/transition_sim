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

                void CheckGasFeedback(int steps)
                {
                    const double pressure = 101325;
                    double dt = duration / steps;
                    var sides = new FlowBoundary[6];
                    sides[1] = new FlowBoundary(FlowBoundaryKind.OpenPressure);
                    var boundaries = new BoundaryConditions3D(sides);
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
                    var momentum = new D3[gas.Length];
                    D3 reaction = default, momentumOut = default;
                    double airOut = 0, vapourOut = 0, energyOut = 0;
                    double upstreamExchange = 0, downstreamExchange = 0, boundaryThroughput = 0;
                    for (int step = 0; step < steps; step++)
                    {
                        var exchange = SeededFogCell.AdvanceGrid(plan, parcels, gas, carrier,
                            pressure, 0.026, 2.5e-5, dt);
                        parcels = exchange.Parcels;
                        upstreamExchange += exchange.Gas[upstream].VapourKg - gas[upstream].VapourKg;
                        downstreamExchange += exchange.Gas[downstream].VapourKg - gas[downstream].VapourKg;
                        gas = exchange.Gas;
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
                        D3 totalMomentum = momentumOut + reaction;
                        for (int group = 0; group < gas.Length; group++)
                        {
                            var cell = moved.Gas[group];
                            gas[group] = (cell.DryAirKg, cell.VapourKg, cell.EnthalpyJ);
                            momentum[group] = cell.MomentumKgMPerS;
                            air += cell.DryAirKg;
                            totalMomentum += momentum[group];
                        }
                        var total = Totals(parcels, gas);
                        if (Math.Abs(air - startingAir) > 1e-12 * startingAir ||
                            Math.Abs(total.Water + vapourOut - initialTotals.Water) > 1e-12 * initialTotals.Water ||
                            Math.Abs(total.Energy + energyOut - initialTotals.Energy) > 1e-12 * initialTotals.Energy ||
                            totalMomentum.Length > 1e-24)
                            throw new InvalidOperationException("V38 fog gas feedback failed conservation.");
                    }
                    if (!(upstreamExchange > 0 && downstreamExchange < 0 && boundaryThroughput > 0) ||
                        gas[untouched] == reservoir)
                        throw new InvalidOperationException("V38 fog exchange did not drive transport into the unseeded cell and boundary.");
                    log(FormattableString.Invariant(
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
