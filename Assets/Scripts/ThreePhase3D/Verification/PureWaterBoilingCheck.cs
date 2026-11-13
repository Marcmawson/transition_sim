using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;
using PhysicsSim.ThreePhase3D.Geometry;

namespace PhysicsSim.ThreePhase3D.Verification
{
    public static class PureWaterBoilingCheck
    {
        public enum BubbleCase { Buoyancy, Evaporation, Condensation }

        public static void RunBubbleRise(Func<VariableCoefficientProjection, float[], double, int, double[], FlowStep> project, Action<string> log,
            BubbleCase scenario = BubbleCase.Buoyancy,
            Action<int, Grid3D, double[], double, double> capture = null)
        {
            const double dx = 0.001, dt = 0.0004;
            double heatFluxWm2 = scenario switch
            {
                BubbleCase.Buoyancy => 0,
                BubbleCase.Evaporation => 10000,
                BubbleCase.Condensation => -10000,
                _ => throw new ArgumentOutOfRangeException(nameof(scenario))
            };
            var grid = new Grid3D(8, 8, 8, new D3(dx, dx, dx), RigidFrame.Identity);
            var centre = new D3(0.004, 0.003, 0.004);
            var sphere = SurfacePrimitives.Transform(SurfacePrimitives.Sphere(0.0016, 12, 24),
                RigidFrame.Euler(centre, 0, 0, 0));
            var seed = CutCellGeometry.Build(grid, iceSurface: sphere);
            var domain = CutCellGeometry.Build(grid);
            var surface = new PureWaterStefanInterface(101325, 370, 377, heatFluxWm2, 0);
            var saturation = surface.Saturation;
            double rhoL = saturation.Liquid.DensityKgM3, rhoG = saturation.Vapour.DensityKgM3;
            var sides = new FlowBoundary[6];
            if (heatFluxWm2 != 0) sides[3] = new FlowBoundary(FlowBoundaryKind.OpenPressure);
            var boundaries = new BoundaryConditions3D(sides);
            var plan = new VariableCoefficientProjection(domain, boundaries,
                cell => rhoL + (rhoG - rhoL) * seed.IceVolumeM3[cell] / grid.CellVolume, _ => 0);
            var gas = new double[plan.GroupCount];
            var centres = new D3[plan.GroupCount];
            for (int cell = 0; cell < plan.GroupByCell.Length; cell++)
            {
                int group = plan.GroupByCell[cell];
                if (group < 0) continue;
                grid.Cells.Coordinates(cell, out int x, out int y, out int z);
                gas[group] = seed.IceVolumeM3[cell];
                centres[group] = grid.Centre(x, y, z);
            }
            var liquid = new double[gas.Length];
            for (int group = 0; group < gas.Length; group++) liquid[group] = plan.Volumes[group] - gas[group];
            var state = new CutCellInventories(liquid, gas,
                Array.ConvertAll(liquid, v => v * rhoL * saturation.Liquid.EnthalpyJkg),
                Array.ConvertAll(gas, v => v * rhoG * saturation.Vapour.EnthalpyJkg),
                Array.ConvertAll(gas, v => v * rhoG));
            var transport = new CutCellVofTransport(plan);
            double volume = Sum(gas);
            D3 initial = Centre(gas);
            if (heatFluxWm2 != 0)
            {
                var noGravity = RunWithPhaseExchange(0);
                var withGravity = RunWithPhaseExchange(-9.81, capture);
                var rise = Centre(withGravity.GasM3) - initial;
                double control = (Centre(noGravity.GasM3) - initial).Y;
                double gained = Sum(withGravity.GasM3) - volume;
                if (!(rise.Y - control > 1e-8 && Math.Sign(gained) == Math.Sign(heatFluxWm2)))
                    throw new InvalidOperationException("The phase-changing vapour seed did not exchange vapour in the expected direction and rise.");
                var reference = heatFluxWm2 > 0
                    ? (Rise: 4.708612e-5, Control: 1.0661593e-5, VolumeChange: 5.8324244e-10)
                    : (Rise: 2.7708753e-5, Control: -1.047588e-5, VolumeChange: -5.6192099e-10);
                if (Math.Abs(rise.Y - reference.Rise) > 1e-6 || Math.Abs(control - reference.Control) > 1e-6 ||
                    Math.Abs(gained / reference.VolumeChange - 1) > 0.01)
                    throw new InvalidOperationException($"The phase-changing bubble differs from the CPU baseline. Rise {rise.Y:G8} m, control {control:G8} m, vapour volume change {gained:G8} m3.");
                log(FormattableString.Invariant(
                    $"S13 {(heatFluxWm2 > 0 ? "growing" : "condensing")} bubble passed: 6 steps, rise {rise.Y:G8} m, no-gravity rise {control:G8} m, vapour volume change {gained:G8} m3."));
                return;
            }
            var still = Run(default);
            var rising = Run(new D3(0, -9.81, 0));
            var displacement = Centre(rising.GasM3) - initial;
            if ((Centre(still.GasM3) - initial).Length > 1e-12 || displacement.Y <= 1e-8 ||
                Math.Sqrt(displacement.X * displacement.X + displacement.Z * displacement.Z) > 0.02 * displacement.Y)
                throw new InvalidOperationException($"The resolved vapour seed did not rise symmetrically: {displacement.Y:G8} m.");
            log(FormattableString.Invariant(
                $"S13 resolved bubble rise passed: 6 steps, vertical centre displacement {displacement.Y:G8} m, vapour volume {volume:G8} m3."));

            CutCellInventories Run(D3 gravity)
            {
                var fields = state;
                var pressure = FlowCpuReference.Project(plan, new float[plan.FaceCount], dt, 32768);
                for (int step = 0; step < 6; step++)
                {
                    var result = transport.AdvanceCoupledWithAcceleration(fields, pressure, rhoL, rhoG,
                        gravity, maxIterations: 32768);
                    fields = result.Transport.Fields;
                    pressure = result.Pressure;
                    for (int field = 0; field < state.Planes.Length; field++)
                    {
                        double before = Sum(state.Planes[field]);
                        if (Math.Abs(Sum(fields.Planes[field]) - before) > Math.Abs(before) * 1e-9)
                            throw new InvalidOperationException("The closed bubble fixture failed an inventory balance.");
                    }
                }
                return fields;
            }

            CutCellInventories RunWithPhaseExchange(double gravityY,
                Action<int, Grid3D, double[], double, double> sample = null)
            {
                var fields = state;
                var velocity = new float[plan.FaceCount];
                double water = Sum(state.LiquidM3) * rhoL + Sum(state.VapourKg);
                double energy = Sum(state.LiquidEnergyJ) + Sum(state.GasEnergyJ);
                double waterIn = 0, energyIn = 0, vapourProduced = 0, gasIn = 0;
                Capture(0);
                for (int step = 0; step < 6; step++)
                {
                    var current = new VariableCoefficientProjection(domain, boundaries, cell =>
                    {
                        int group = plan.GroupByCell[cell];
                        return (rhoL * fields.LiquidM3[group] + rhoG * fields.GasM3[group]) / plan.Volumes[group];
                    }, _ => 0);
                    var tentative = (float[])velocity.Clone();
                    for (int face = 0; face < current.FaceCount; face++)
                        if (current.Faces[face].Active != 0 && current.Faces[face].Axis == 1)
                            tentative[face] = (float)(tentative[face] + dt * gravityY);
                    var result = new CutCellVofTransport(current).AdvancePureWaterStefanWithMomentum(fields, dt, surface, source =>
                    {
                        vapourProduced += -Sum(source) * dt / (1 / rhoG - 1 / rhoL) / rhoG;
                        return project(current, tentative, dt, 32768, source);
                    }, _ => new CutCellInflow(1, rhoL * saturation.Liquid.EnthalpyJkg, 0, 0));
                    var incomingMomentum = new CutCellMomentum(current).GatherGroupMomentum(fields,
                        result.Pressure.VelocityMPerS, rhoL, rhoG);
                    var outgoingMomentum = new double[3];
                    for (int face = 0; face < current.FaceCount; face++)
                        outgoingMomentum[current.Faces[face].Axis] += result.Momentum.MomentumKgMPerS[face];
                    for (int axis = 0; axis < 3; axis++)
                        if (Math.Abs(outgoingMomentum[axis] - Sum(incomingMomentum[axis]) -
                            result.Momentum.BoundaryMomentumKgMPerS[axis] - result.Momentum.WallImpulseKgMPerS[axis]) > 1e-15)
                            throw new InvalidOperationException("The phase-changing bubble failed its momentum balance.");
                    fields = result.Transport.Fields;
                    velocity = Array.ConvertAll(result.Momentum.VelocityMPerS, value => (float)value);
                    var boundary = result.Transport.BoundaryInflow;
                    waterIn += boundary[0] * rhoL + boundary[4];
                    energyIn += boundary[2] + boundary[3];
                    gasIn += boundary[1];
                    if (Math.Abs(Sum(fields.GasM3) - volume - vapourProduced - gasIn) > 1e-16 ||
                        Math.Abs(Sum(fields.LiquidM3) * rhoL + Sum(fields.VapourKg) - water - waterIn) > 1e-13 ||
                        Math.Abs(Sum(fields.LiquidEnergyJ) + Sum(fields.GasEnergyJ) - energy - energyIn) > 1e-8)
                        throw new InvalidOperationException($"The phase-changing bubble failed its phase or outlet balance at step {step + 1}.");
                    Capture(step + 1);
                }
                return fields;

                void Capture(int step)
                {
                    if (sample == null) return;
                    var fractions = new double[grid.Cells.Count];
                    for (int cell = 0; cell < fractions.Length; cell++)
                    {
                        int group = plan.GroupByCell[cell];
                        if (group >= 0) fractions[cell] = fields.GasM3[group] / plan.Volumes[group];
                    }
                    sample(step, grid, fractions, Sum(fields.GasM3), Centre(fields.GasM3).Y);
                }
            }

            D3 Centre(double[] volumes)
            {
                D3 firstMoment = default;
                for (int group = 0; group < volumes.Length; group++) firstMoment += centres[group] * volumes[group];
                return firstMoment / Sum(volumes);
            }
            double Sum(double[] values) => System.Linq.Enumerable.Sum(values);
        }

        public static void RunPlanarStefan(Func<VariableCoefficientProjection, float[], double, int, double[], FlowStep> project, Action<string> log)
        {
            const double pressure = 101325, dx = 0.01, dt = 0.002;
            var grid = new Grid3D(1, 3, 1, new D3(dx, dx, dx), RigidFrame.Identity);
            double area = grid.FaceArea(1);
            var sides = new FlowBoundary[6];
            sides[3] = new FlowBoundary(FlowBoundaryKind.OpenPressure);
            foreach (double liquidHeat in new[] { 10000.0, -10000.0 })
            foreach (double initialHeight in new[] { 0.01, 0.015 })
            {
                const int steps = 4;
                var surface = new PureWaterStefanInterface(pressure, 370, 377, liquidHeat, 0);
                var liquid = surface.Saturation.Liquid;
                var vapour = surface.Saturation.Vapour;
                Near(Math.Abs(liquid.TemperatureK - 373.1243), 5e-5);
                Near(Math.Abs(liquid.TemperatureK - vapour.TemperatureK), 0);
                if (!(liquid.DensityKgM3 / vapour.DensityKgM3 > 1500))
                    throw new InvalidOperationException("The Stefan fixture requires the physical liquid/vapour density contrast.");
                var plan = new VariableCoefficientProjection(CutCellGeometry.Build(grid),
                    new BoundaryConditions3D(sides), _ => vapour.DensityKgM3, _ => 0);
                var transport = new CutCellVofTransport(plan);
                var fields = PlanarLiquidInventory.Create(plan, new D3(0, initialHeight, 0), new D3(0, 1, 0),
                    liquid.DensityKgM3 * liquid.EnthalpyJkg, vapour.DensityKgM3 * vapour.EnthalpyJkg, vapour.DensityKgM3);
                double Sum(double[] values) => System.Linq.Enumerable.Sum(values);
                double initialLiquid = Sum(fields.LiquidM3);
                double initialWater = initialLiquid * liquid.DensityKgM3 + Sum(fields.VapourKg);
                double initialHeat = Sum(fields.LiquidEnergyJ) + Sum(fields.GasEnergyJ);
                double boundaryWater = 0, boundaryHeat = 0;
                for (int step = 0; step < steps; step++)
                {
                    var result = transport.AdvancePureWaterStefanWithMomentum(fields, dt, surface,
                        source => project(plan, new float[plan.FaceCount], dt, 4096, source),
                        _ => new CutCellInflow(0, 0, vapour.DensityKgM3 * vapour.EnthalpyJkg, vapour.DensityKgM3));
                    fields = result.Transport.Fields;
                    if (initialHeight == dx && liquidHeat > 0)
                        Near(fields.LiquidM3[plan.GroupByCell[grid.Cells.Index(0, 1, 0)]], 1e-18);
                    var boundary = result.Transport.BoundaryInflow;
                    Near(Math.Abs(boundary[0]), 1e-18);
                    boundaryWater += boundary[4];
                    boundaryHeat += boundary[2] + boundary[3];
                    double outlet = result.Pressure.VelocityMPerS[plan.FaceIndex(1, 0, 3, 0)];
                    Near(Math.Abs(outlet - surface.VapourMinusLiquidVelocityMPerS), 1e-7);
                }
                double frontShift = (Sum(fields.LiquidM3) - initialLiquid) / area;
                double analyticShift = -liquidHeat * dt * steps /
                    (surface.Saturation.LatentEnthalpyJkg * liquid.DensityKgM3);
                Near(Math.Abs(frontShift - analyticShift), 1e-11);
                Near(Math.Abs(Sum(fields.LiquidM3) * liquid.DensityKgM3 + Sum(fields.VapourKg) -
                    initialWater - boundaryWater), 1e-14);
                Near(Math.Abs(Sum(fields.LiquidEnergyJ) + Sum(fields.GasEnergyJ) - initialHeat - boundaryHeat), 1e-9);
                log(FormattableString.Invariant(
                    $"S13 V29 planar Stefan passed ({(initialHeight == dx ? "initially aligned" : "initially mixed")}): flux {surface.MassFluxKgM2PerS:G8} kg/m2/s, front shift {frontShift:G8} m, velocity jump {surface.VapourMinusLiquidVelocityMPerS:G8} m/s."));
            }

            void Near(double error, double tolerance)
            {
                if (!(error <= tolerance))
                    throw new InvalidOperationException($"Planar Stefan error {error:G8} exceeds {tolerance:G8}.");
            }
        }
    }
}
