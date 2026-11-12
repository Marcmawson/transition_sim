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
                $"S14 V37 multi-step gas grid passed: 40 steps, radius {referenceGrid.RadiusM * 1e6:F6} um, outlet vapour {referenceGrid.OutletVapourKg:G8} kg."));

            (double RadiusM, double OutletVapourKg) SimulateGrid(int steps)
            {
                const double pressure = 101325;
                double dt = duration / steps;
                double cellVolume = DropletGasCellExchange.GasVolumeM3(dryAir, initialVapour, initialGasEnergy, pressure);
                double dx = Math.Pow(cellVolume, 1.0 / 3);
                var grid = new Grid3D(2, 1, 1, new D3(dx, dx, dx), RigidFrame.Identity);
                var geometry = CutCellGeometry.Build(grid);
                var sides = new FlowBoundary[6];
                sides[1] = new FlowBoundary(FlowBoundaryKind.OpenPressure);
                var boundaries = new BoundaryConditions3D(sides);
                var seed = new VariableCoefficientProjection(geometry, boundaries,
                    _ => (dryAir + initialVapour) / cellVolume, _ => 0);
                int dropletGroup = seed.GroupByCell[grid.Cells.Index(0, 0, 0)];
                var gas = new (double DryAirKg, double VapourKg, double EnthalpyJ, D3 MomentumKgMPerS)[seed.GroupCount];
                for (int group = 0; group < gas.Length; group++) gas[group] = (dryAir, initialVapour, initialGasEnergy, default);
                double radius = initialRadius, liquidK = initialK, airOut = 0, vapourOut = 0, energyOut = 0;
                for (int step = 0; step < steps; step++)
                {
                    var local = gas[dropletGroup];
                    var exchanged = DropletGasCellExchange.Advance(radius, liquidK, local.DryAirKg, local.VapourKg,
                        local.EnthalpyJ, pressure, 0.026, 2.5e-5, dt);
                    radius = exchanged.RadiusM;
                    liquidK = exchanged.LiquidK;
                    gas[dropletGroup] = (local.DryAirKg, exchanged.VapourKg, exchanged.GasEnthalpyJ, local.MomentumKgMPerS);
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
                    var flow = FlowCpuReference.Project(plan, new float[plan.FaceCount], dt, 4096, null, source);
                    var moved = HumidGasAdvection.Advance(plan, flow, volumes, gas,
                        (dryAir / cellVolume, initialVapour / cellVolume, initialGasEnergy / cellVolume, default));
                    gas = moved.Gas;
                    airOut += moved.Outflow.DryAirKg;
                    vapourOut += moved.Outflow.VapourKg;
                    energyOut += moved.Outflow.EnthalpyJ;
                    double air = airOut, water = vapourOut + DropletGasCellExchange.Mass(radius);
                    double energy = energyOut + LiquidEnthalpy(DropletGasCellExchange.Mass(radius), liquidK);
                    foreach (var state in gas)
                    {
                        air += state.DryAirKg;
                        water += state.VapourKg;
                        energy += state.EnthalpyJ;
                    }
                    if (Math.Abs(air - 2 * dryAir) > 1e-12 * dryAir ||
                        Math.Abs(water - initialMass - 2 * initialVapour) > 1e-12 * initialMass ||
                        Math.Abs(energy - initialTotalEnergy - initialGasEnergy) > 1e-12 * initialTotalEnergy)
                        throw new InvalidOperationException($"S14 gas-grid inventory mismatch at step {step + 1}.");
                }
                return (radius, vapourOut);
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
