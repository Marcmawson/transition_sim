using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;
using PhysicsSim.ThreePhase3D.Geometry;

namespace PhysicsSim.ThreePhase3D.Verification
{
    internal static class DropletGasCellCheck
    {
        internal static void Run(Action<string> log)
        {
            const double initialRadius = 10e-6, initialK = 300, duration = 0.004;
            double initialMass = DropletGasCellExchange.Mass(initialRadius);
            double dryAir = initialMass * 10, initialVapour = dryAir * 0.005;
            double initialGasEnergy = GasEnthalpy(310, initialVapour);
            double initialTotalEnergy = initialGasEnergy + LiquidEnthalpy(initialMass, initialK);
            var coarse = Simulate(16);
            var medium = Simulate(32);
            var fine = Simulate(64);
            var reference = Simulate(128);
            if (!(fine.LiquidK < initialK && fine.VapourKg > initialVapour && fine.RadiusM < initialRadius) ||
                Math.Abs(fine.LiquidK - reference.LiquidK) >= Math.Abs(medium.LiquidK - reference.LiquidK) ||
                Math.Abs(medium.LiquidK - reference.LiquidK) >= Math.Abs(coarse.LiquidK - reference.LiquidK) ||
                Math.Abs(fine.VapourKg - reference.VapourKg) >= Math.Abs(medium.VapourKg - reference.VapourKg))
                throw new InvalidOperationException("S14 coupled evaporation failed cooling, vapour gain or timestep refinement.");
            log(FormattableString.Invariant(
                $"S14 V37 gas-cell exchange passed: droplet {reference.LiquidK:F6} K, radius {reference.RadiusM * 1e6:F6} um, vapour gain {reference.VapourKg - initialVapour:G8} kg."));
            CheckProjection();
            var coarseGrid = SimulateGrid(10);
            var fineGrid = SimulateGrid(20);
            var referenceGrid = SimulateGrid(40);
            if (!(referenceGrid.RadiusM < initialRadius && referenceGrid.OutletVapourKg != 0) ||
                Math.Abs(fineGrid.RadiusM - referenceGrid.RadiusM) >= Math.Abs(coarseGrid.RadiusM - referenceGrid.RadiusM) ||
                Math.Abs(fineGrid.OutletVapourKg - referenceGrid.OutletVapourKg) >=
                    Math.Abs(coarseGrid.OutletVapourKg - referenceGrid.OutletVapourKg))
                throw new InvalidOperationException("S14 coupled gas-grid results did not refine.");
            log(FormattableString.Invariant(
                $"S14 V37 multi-step gas grid passed: 40 steps, radius {referenceGrid.RadiusM * 1e6:F6} um, outlet vapour {referenceGrid.OutletVapourKg:G8} kg, projected feedback {referenceGrid.FeedbackMps:G8} m/s, wall reaction X {referenceGrid.WallReactionX:G8} N s."));
            SimulateGrid(20, crossCell: true);
            SimulateGrid(40, crossCell: true);
            SimulateGrid(20, exitDomain: true);
            SimulateGrid(40, exitDomain: true);

            (double RadiusM, double OutletVapourKg, double FeedbackMps, double WallReactionX) SimulateGrid(
                int steps, bool crossCell = false, bool exitDomain = false)
            {
                const double pressure = 101325;
                double dt = duration / steps;
                double cellVolume = DropletGasCellExchange.GasVolumeM3(dryAir, initialVapour, initialGasEnergy, pressure);
                double dx = Math.Pow(cellVolume, 1.0 / 3);
                var grid = new Grid3D(2, 2, 1, new D3(dx, dx, dx), RigidFrame.Identity);
                var geometry = CutCellGeometry.Build(grid);
                var sides = new FlowBoundary[6];
                const double outletGaugePa = 0.002;
                sides[1] = new FlowBoundary(FlowBoundaryKind.OpenPressure, outletGaugePa);
                var boundaries = new BoundaryConditions3D(sides);
                var seed = new VariableCoefficientProjection(geometry, boundaries,
                    _ => (dryAir + initialVapour) / cellVolume, _ => 0);
                int firstGroup = seed.PressureGroup(0, 0, 0), secondGroup = seed.PressureGroup(1, 0, 0);
                var gas = new (double DryAirKg, double VapourKg, double EnthalpyJ, D3 MomentumKgMPerS)[seed.GroupCount];
                for (int group = 0; group < gas.Length; group++) gas[group] = (dryAir, initialVapour, initialGasEnergy, default);
                double radius = initialRadius, liquidK = initialK, airOut = 0, vapourOut = 0, energyOut = 0;
                var initialVelocity = new D3(0.005, 0, 0);
                D3 velocity = initialVelocity, position = grid.Centre(0, 0, 0), momentumOut = default;
                if (crossCell) position += new D3(dx * 0.5 - 2e-6, 0, 0);
                if (exitDomain) position = grid.Centre(1, 0, 0) + new D3(dx * 0.5 - 2e-6, 0, 0);
                D3 wallReaction = default, openReaction = default;
                double feedback = 0, firstVapour = 0, secondVapour = 0;
                double liquidOut = 0, liquidEnthalpyOut = 0;
                D3 liquidMomentumOut = default;
                int crossings = 0, exitStep = 0;
                int previousGroup = exitDomain ? secondGroup : firstGroup;
                for (int step = 0; step < steps; step++)
                {
                    int dropletGroup = -1;
                    D3 gasMomentumGain = default;
                    if (radius > 0)
                    {
                        dropletGroup = DropletGasCellExchange.PressureGroupAt(seed, position);
                        if (dropletGroup != previousGroup) crossings++;
                        previousGroup = dropletGroup;
                        var local = gas[dropletGroup];
                        var exchanged = DropletGasCellExchange.Advance(radius, liquidK, local.DryAirKg, local.VapourKg,
                            local.EnthalpyJ, pressure, 0.026, 2.5e-5, dt);
                        double localMass = local.DryAirKg + local.VapourKg;
                        double localVolume = DropletGasCellExchange.GasVolumeM3(local.DryAirKg, local.VapourKg, local.EnthalpyJ, pressure);
                        var motion = StokesDroplet.Advance(position, velocity, local.MomentumKgMPerS / localMass, default,
                            radius, DropletGasCellExchange.LiquidDensity, localMass / localVolume, 1.81e-5, dt,
                            exchanged.VapourKg - local.VapourKg);
                        gasMomentumGain = motion.GasMomentumGain;
                        bool exited = DropletGasCellExchange.ExitsOpenBoundary(seed, position, motion.Position);
                        velocity = motion.Velocity;
                        position = motion.Position;
                        if (!exited) DropletGasCellExchange.PressureGroupAt(seed, position);
                        radius = exchanged.RadiusM;
                        liquidK = exchanged.LiquidK;
                        gas[dropletGroup] = (local.DryAirKg, exchanged.VapourKg, exchanged.GasEnthalpyJ,
                            local.MomentumKgMPerS + motion.GasMomentumGain);
                        double depositedVapour = exchanged.VapourKg - local.VapourKg;
                        if (dropletGroup == firstGroup) firstVapour += depositedVapour;
                        else if (dropletGroup == secondGroup)
                        {
                            secondVapour += depositedVapour;
                            if (!(motion.GasMomentumGain.X > 0 && exchanged.GasEnthalpyJ != local.EnthalpyJ))
                                throw new InvalidOperationException("S14 crossing did not exchange energy and momentum in the receiving cell.");
                        }
                        else throw new InvalidOperationException("S14 droplet entered an unexpected cell.");
                        if (exited)
                        {
                            exitStep = step + 1;
                            liquidOut = DropletGasCellExchange.Mass(radius);
                            liquidEnthalpyOut = LiquidEnthalpy(liquidOut, liquidK);
                            liquidMomentumOut = velocity * liquidOut;
                            radius = 0;
                        }
                    }
                    var volumes = new double[gas.Length];
                    var source = new double[gas.Length];
                    for (int group = 0; group < gas.Length; group++)
                    {
                        var state = gas[group];
                        volumes[group] = DropletGasCellExchange.GasVolumeM3(state.DryAirKg, state.VapourKg, state.EnthalpyJ, pressure);
                        source[group] = (seed.Volumes[group] - volumes[group]) / dt;
                    }
                    var plan = new VariableCoefficientProjection(geometry, boundaries, cell =>
                    {
                        int group = seed.GroupByCell[cell];
                        return (gas[group].DryAirKg + gas[group].VapourKg) / volumes[group];
                    }, _ => 0);
                    var predicted = HumidGasAdvection.PredictFaces(plan, gas);
                    var flow = FlowCpuReference.Project(plan, predicted, dt, 4096, null, source);
                    if (step == 1)
                    {
                        var controlGas = ((double DryAirKg, double VapourKg, double EnthalpyJ, D3 MomentumKgMPerS)[])gas.Clone();
                        for (int group = 0; group < controlGas.Length; group++)
                            controlGas[group].MomentumKgMPerS = group == dropletGroup ? gasMomentumGain : default;
                        var control = FlowCpuReference.Project(plan, HumidGasAdvection.PredictFaces(plan, controlGas),
                            dt, 4096, null, source);
                        for (int face = 0; face < plan.FaceCount; face++)
                            feedback = Math.Max(feedback, Math.Abs(flow.VelocityMPerS[face] - control.VelocityMPerS[face]));
                    }
                    var reactions = HumidGasAdvection.ApplyPressure(plan, flow, gas);
                    wallReaction += reactions.WallReaction;
                    openReaction += reactions.OpenBoundaryReaction;
                    var moved = HumidGasAdvection.Advance(plan, flow, volumes, gas,
                        (dryAir / cellVolume, initialVapour / cellVolume, initialGasEnergy / cellVolume, default));
                    gas = moved.Gas;
                    airOut += moved.Outflow.DryAirKg;
                    vapourOut += moved.Outflow.VapourKg;
                    energyOut += moved.Outflow.EnthalpyJ;
                    momentumOut += moved.Outflow.MomentumKgMPerS;
                    double remaining = DropletGasCellExchange.Mass(radius);
                    double air = airOut, water = vapourOut + liquidOut + remaining;
                    double energy = energyOut + liquidEnthalpyOut + LiquidEnthalpy(remaining, liquidK);
                    D3 momentum = momentumOut + liquidMomentumOut + wallReaction + openReaction + velocity * remaining;
                    foreach (var state in gas)
                    {
                        air += state.DryAirKg;
                        water += state.VapourKg;
                        energy += state.EnthalpyJ;
                        momentum += state.MomentumKgMPerS;
                    }
                    if (Math.Abs(air - gas.Length * dryAir) > 1e-12 * dryAir ||
                        Math.Abs(water - initialMass - gas.Length * initialVapour) > 1e-12 * initialMass ||
                        Math.Abs(energy - initialTotalEnergy - (gas.Length - 1) * initialGasEnergy) > 1e-12 * initialTotalEnergy ||
                        (momentum - initialVelocity * initialMass).Length > 1e-24)
                        throw new InvalidOperationException($"S14 gas-grid inventory mismatch at step {step + 1}.");
                }
                if (!(feedback > 1e-7))
                    throw new InvalidOperationException("Carried gas momentum did not change the projected face velocities.");
                double expectedOpen = outletGaugePa * 2 * grid.FaceArea(0) * duration;
                if ((openReaction - new D3(expectedOpen, 0, 0)).Length > expectedOpen * 1e-7 ||
                    !(wallReaction.Length > 0))
                    throw new InvalidOperationException("S14 wall or prescribed outlet pressure reaction did not balance.");
                if (crossCell)
                {
                    if (!(crossings == 1 && previousGroup == secondGroup && firstVapour > 0 && secondVapour > 0))
                        throw new InvalidOperationException("S14 droplet did not deposit vapour on both sides of the crossed face.");
                    log(FormattableString.Invariant(
                        $"S14 V37 droplet cell crossing passed: {steps} steps, first cell {firstVapour:G8} kg vapour, second cell {secondVapour:G8} kg vapour."));
                }
                else if (crossings != 0)
                    throw new InvalidOperationException("S14 in-cell control unexpectedly crossed a face.");
                if (exitDomain)
                {
                    if (!(exitStep > 0 && exitStep < steps && radius == 0 && liquidOut > 0 && liquidOut < initialMass &&
                        liquidEnthalpyOut > 0 && liquidMomentumOut.X > 0 && secondVapour > 0))
                        throw new InvalidOperationException("S14 open boundary did not export the droplet and its remaining inventories.");
                    log(FormattableString.Invariant(
                        $"S14 V37 droplet exit passed: {steps} steps, exit step {exitStep}, liquid {liquidOut:G8} kg, enthalpy {liquidEnthalpyOut:G8} J, momentum X {liquidMomentumOut.X:G8} N s."));
                }
                else if (exitStep != 0)
                    throw new InvalidOperationException("S14 control droplet unexpectedly exited the domain.");
                return (radius, vapourOut, feedback, wallReaction.X);
            }

            void CheckProjection()
            {
                const double pressure = 101325, dt = 0.0004;
                double volume = DropletGasCellExchange.GasVolumeM3(dryAir, initialVapour, initialGasEnergy, pressure);
                double dx = Math.Pow(volume, 1.0 / 3);
                var grid = new Grid3D(2, 1, 1, new D3(dx, dx, dx), RigidFrame.Identity);
                var sides = new FlowBoundary[6];
                sides[1] = new FlowBoundary(FlowBoundaryKind.OpenPressure);
                var plan = new VariableCoefficientProjection(CutCellGeometry.Build(grid),
                    new BoundaryConditions3D(sides), _ => (dryAir + initialVapour) / volume, _ => 0);
                int donor = plan.GroupByCell[grid.Cells.Index(0, 0, 0)];
                int downstream = plan.GroupByCell[grid.Cells.Index(1, 0, 0)];
                var trial = DropletGasCellExchange.Advance(initialRadius, initialK, dryAir, initialVapour,
                    initialGasEnergy, pressure, 0.026, 2.5e-5, dt);
                var source = new double[plan.GroupCount];
                source[donor] = DropletGasCellExchange.ProjectionSourceM3PerS(dryAir, initialVapour,
                    initialGasEnergy, trial.VapourKg, trial.GasEnthalpyJ, pressure, dt);
                var projected = FlowCpuReference.Project(plan, new float[plan.FaceCount], dt, 4096, null, source);
                double internalFlux = plan.OutwardFlux(donor, projected.VelocityMPerS);
                double netFlux = internalFlux + plan.OutwardFlux(downstream, projected.VelocityMPerS);
                double tolerance = Math.Abs(source[donor]) * 1e-5;
                if (!(trial.VapourKg > initialVapour && source[donor] < 0) ||
                    Math.Abs(internalFlux + source[donor]) > tolerance ||
                    Math.Abs(netFlux + source[donor]) > tolerance)
                    throw new InvalidOperationException($"S14 droplet-cell pressure mismatch: source {source[donor]:G8}, internal flux {internalFlux:G8}, outlet flux {netFlux:G8} m3/s.");
                log(FormattableString.Invariant(
                    $"S14 V37 gas-cell pressure projection passed: source {source[donor]:G8} m3/s, outlet flux {netFlux:G8} m3/s."));

                var gas = new (double DryAirKg, double VapourKg, double EnthalpyJ, D3 MomentumKgMPerS)[plan.GroupCount];
                var gasVolumes = new double[plan.GroupCount];
                var dropletVelocity = new D3(0, -0.01, 0);
                var gravity = new D3(0, -9.81, 0);
                double gasDensity = (dryAir + initialVapour) / volume;
                var motion = StokesDroplet.Advance(default, dropletVelocity, default, gravity,
                    initialRadius, DropletGasCellExchange.LiquidDensity, gasDensity, 1.81e-5, dt,
                    trial.VapourKg - initialVapour);
                var downstreamMomentum = new D3(0, 0.001, 0) * (dryAir + initialVapour);
                gas[donor] = (dryAir, trial.VapourKg, trial.GasEnthalpyJ, motion.GasMomentumGain);
                gas[downstream] = (dryAir, initialVapour, initialGasEnergy, downstreamMomentum);
                gasVolumes[donor] = DropletGasCellExchange.GasVolumeM3(dryAir, trial.VapourKg, trial.GasEnthalpyJ, pressure);
                gasVolumes[downstream] = volume;
                var moved = HumidGasAdvection.Advance(plan, projected, gasVolumes, gas);
                var first = moved.Gas[donor];
                var second = moved.Gas[downstream];
                var outlet = moved.Outflow;
                double initialWater = initialMass + 2 * initialVapour;
                double water = DropletGasCellExchange.Mass(trial.RadiusM) + first.VapourKg + second.VapourKg + outlet.VapourKg;
                double enthalpy = LiquidEnthalpy(DropletGasCellExchange.Mass(trial.RadiusM), trial.LiquidK) +
                    first.EnthalpyJ + second.EnthalpyJ + outlet.EnthalpyJ;
                double outletFraction = netFlux * dt / volume;
                if (!(second.VapourKg > initialVapour && outlet.VapourKg > 0) ||
                    Math.Abs(outlet.VapourKg / initialVapour - outletFraction) > 1e-5 * outletFraction ||
                    Math.Abs(first.DryAirKg + second.DryAirKg + outlet.DryAirKg - 2 * dryAir) > 1e-12 * dryAir ||
                    Math.Abs(water - initialWater) > 1e-12 * initialWater ||
                    Math.Abs(enthalpy - initialTotalEnergy - initialGasEnergy) > 1e-12 * initialTotalEnergy)
                    throw new InvalidOperationException("S14 projected gas advection failed transport or conservation.");
                log(FormattableString.Invariant(
                    $"S14 V37 gas transport passed: downstream vapour gain {second.VapourKg - initialVapour:G8} kg, outlet vapour {outlet.VapourKg:G8} kg, outlet enthalpy {outlet.EnthalpyJ:G8} J."));
                double remainingMass = DropletGasCellExchange.Mass(trial.RadiusM);
                D3 external = gravity * (1 - gasDensity / DropletGasCellExchange.LiquidDensity) * (initialMass * dt);
                D3 residual = first.MomentumKgMPerS + second.MomentumKgMPerS + outlet.MomentumKgMPerS +
                    motion.Velocity * remainingMass - dropletVelocity * initialMass - downstreamMomentum - external;
                D3 internalTransfer = motion.GasMomentumGain * (internalFlux * dt / gasVolumes[donor]);
                if (!(motion.GasMomentumGain.Y < 0 && internalTransfer.Y < 0 && outlet.MomentumKgMPerS.Y > 0) ||
                    residual.Length > 1e-24 ||
                    (second.MomentumKgMPerS - downstreamMomentum + outlet.MomentumKgMPerS - internalTransfer).Length > 1e-24)
                    throw new InvalidOperationException("S14 droplet drag and vapour momentum did not balance across the grid and outlet.");
                log(FormattableString.Invariant(
                    $"S14 V37 gas momentum passed: droplet-to-gas impulse Y {motion.GasMomentumGain.Y:G8} N s, outlet Y {outlet.MomentumKgMPerS.Y:G8} N s, residual {residual.Length:G5} N s."));
            }

            (double RadiusM, double LiquidK, double VapourKg, double GasEnthalpyJ) Simulate(int steps)
            {
                var state = (RadiusM: initialRadius, LiquidK: initialK, VapourKg: initialVapour, GasEnthalpyJ: initialGasEnergy);
                for (int step = 0; step < steps; step++)
                {
                    state = DropletGasCellExchange.Advance(state.RadiusM, state.LiquidK, dryAir,
                        state.VapourKg, state.GasEnthalpyJ, 101325, 0.026, 2.5e-5, duration / steps);
                    double mass = DropletGasCellExchange.Mass(state.RadiusM);
                    if (Math.Abs(mass + state.VapourKg - initialMass - initialVapour) > 1e-12 * initialMass ||
                        Math.Abs(LiquidEnthalpy(mass, state.LiquidK) + state.GasEnthalpyJ - initialTotalEnergy) >
                            1e-12 * initialTotalEnergy)
                        throw new InvalidOperationException("S14 coupled evaporation failed its water or enthalpy ledger.");
                }
                return state;
            }

            double GasEnthalpy(double kelvin, double vapour) =>
                (dryAir * DropletGasCellExchange.AirCp + vapour * DropletGasCellExchange.VapourCp) *
                (kelvin - DropletGasCellExchange.ReferenceK) + vapour * DropletGasCellExchange.VapourReferenceEnthalpy;
            double LiquidEnthalpy(double mass, double kelvin) =>
                mass * DropletGasCellExchange.LiquidCp * (kelvin - DropletGasCellExchange.ReferenceK);
        }
    }
}
