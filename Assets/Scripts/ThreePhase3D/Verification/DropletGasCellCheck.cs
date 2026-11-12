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
