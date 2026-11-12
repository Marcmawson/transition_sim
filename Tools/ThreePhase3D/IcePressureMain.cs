using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;
using PhysicsSim.ThreePhase3D.Geometry;
using PhysicsSim.ThreePhase3D.Solids;

internal static class IcePressureMain
{
    private static void Main()
    {
        CheckRemap();
        CheckWaterline();
        CheckTwoPhaseMotion();
        const double g = 9.81, rho = 1000;
        var size = new D3(0.04, 0.03, 0.02);
        var centre = new D3(0.055, 0.054, 0.053);
        var box = SurfacePrimitives.Box(size * -0.5, size * 0.5);
        double volume = size.X * size.Y * size.Z;
        foreach (double offset in new[] { 0.0, 0.003 })
        {
            var body = IceBody.Box(917, size, RigidFrame.Euler(centre, 0, 0, 0.3));
            var gridFrame = RigidFrame.Euler(new D3(offset, 0, 0), 0.15, 0, 0);
            var grid = new Grid3D(12, 12, 12, new D3(0.01, 0.01, 0.01), gridFrame);
            var geometry = CutCellGeometry.Build(grid, iceSurface: SurfacePrimitives.Transform(box, body.Frame));
            var plan = new VariableCoefficientProjection(geometry, BoundaryConditions3D.Closed, _ => rho, _ => 0);
            var gauge = new double[plan.GroupCount];
            for (int i = 0; i < gauge.Length; i++) gauge[i] = 80;
            var load = IcePressureLoad.FromProjection(body, plan, gauge, point => rho * g * (0.10 - point.Y));
            Near((load.ForceN - new D3(0, rho * g * volume, 0)).Length, 1e-9);
            Near(load.TorqueNm.Length, 1e-10);
            const double dt = 0.01;
            var moved = load.Advance(body, dt, new D3(0, -g, 0));
            Near(Math.Abs(moved.VelocityMPerS.Y - dt * g * (rho / 917 - 1)), 1e-10);

            // A prescribed affine pressure on the top face generates a
            // known moment in body coordinates, independent of the cut grid.
            const double gradient = 1200;
            var topLoad = IcePressureLoad.Integrate(body, geometry, (patch, point) =>
            {
                var n = patch.NormalLocal;
                D3 worldNormal = gridFrame.U * n.X + gridFrame.V * n.Y + gridFrame.W * n.Z;
                return D3.Dot(worldNormal, body.Frame.V) > 0.99
                    ? 100 + gradient * body.Frame.ToLocal(point).X : 0;
            });
            D3 expectedTorque = body.Frame.W * (-gradient * size.Z * Math.Pow(size.X, 3) / 12);
            Near((topLoad.ForceN - body.Frame.V * (-100 * size.X * size.Z)).Length, 1e-9);
            Near((topLoad.TorqueNm - expectedTorque).Length, 1e-10);
            var trial = new IceBody(body.MassKg, body.PrincipalInertiaKgM2, body.Frame,
                new D3(0, body.MassKg * 0.01, 0), new D3(0, 0, body.PrincipalInertiaKgM2.Z * 0.2));
            var flux = IcePressureLoad.OutwardFluidFlux(trial, plan);
            var projection = FlowCpuReference.Project(plan, new float[plan.FaceCount], dt, 16384, null, flux);
            var reaction = IcePressureLoad.FromProjection(trial, plan, projection.GroupPressurePa);
            double workRate = D3.Dot(reaction.ForceN, trial.VelocityMPerS) +
                D3.Dot(reaction.TorqueNm, trial.AngularVelocityRadPerS);
            double pressurePower = 0;
            for (int group = 0; group < plan.GroupCount; group++)
                pressurePower += projection.GroupPressurePa[group] * flux[group];
            Near(Math.Abs(workRate - pressurePower), 1e-10);
            if (workRate >= 0 || !projection.Diagnostics.Converged)
                throw new InvalidOperationException("Moving ice did not produce an opposing pressure reaction.");
            Console.WriteLine(FormattableString.Invariant(
                $"S11 moving boundary passed at offset {offset:G} m: pressure power {workRate:G6} W."));
            var still = new float[plan.FaceCount];
            var coupled = IcePressureCoupling.Advance(body, plan, dt, new D3(0, -g, 0),
                boundary => FlowCpuReference.Project(plan, still, dt, 16384, null, boundary),
                point => rho * g * (0.10 - point.Y));
            if (!(coupled.Body.VelocityMPerS.Y > 0 && coupled.Body.VelocityMPerS.Y < moved.VelocityMPerS.Y) ||
                coupled.Iterations < 2 || coupled.BoundaryVelocityMismatchMPerS > 1e-8)
                throw new InvalidOperationException("The implicit fluid reaction did not reduce the ice rise speed.");
            var nextGeometry = CutCellGeometry.Build(grid,
                iceSurface: SurfacePrimitives.Transform(box, coupled.Body.Frame));
            var nextPlan = new VariableCoefficientProjection(nextGeometry, BoundaryConditions3D.Closed, _ => rho, _ => 0);
            var volumes = (double[])plan.Volumes.Clone();
            var zero = new double[plan.GroupCount];
            var thermal = new double[plan.GroupCount];
            for (int group = 0; group < plan.GroupCount; group++) thermal[group] = 42 * volumes[group];
            var reconciled = MovingCutCellRemap.Reconcile(plan, nextPlan, coupled.Fluid, 0.005 * grid.CellVolume);
            var oldInventory = new CutCellInventories(volumes, zero, thermal, zero, zero);
            var remap = MovingCutCellRemap.ApplyWithMomentum(plan, nextPlan, oldInventory,
                reconciled, coupled.Fluid.VelocityMPerS, rho, 1);
            var remapped = remap.Fields;
            var oldMomentum = new CutCellMomentum(plan).GatherGroupMomentum(oldInventory,
                coupled.Fluid.VelocityMPerS, rho, 1);
            var newMomentum = new double[3];
            for (int face = 0; face < nextPlan.FaceCount; face++)
            {
                var metric = nextPlan.Faces[face];
                if (metric.Active == 0) Near(Math.Abs(remap.Momentum.VelocityMPerS[face]), 0);
                else newMomentum[metric.Axis] += remap.Momentum.MomentumKgMPerS[face];
            }
            for (int axis = 0; axis < 3; axis++)
                Near(Math.Abs(newMomentum[axis] - System.Linq.Enumerable.Sum(oldMomentum[axis])), 1e-12);
            for (int group = 0; group < nextPlan.GroupCount; group++)
            {
                Near(Math.Abs(remapped.LiquidM3[group] - nextPlan.Volumes[group]), 1e-15);
                Near(Math.Abs(remapped.LiquidEnergyJ[group] - 42 * nextPlan.Volumes[group]), 1e-13);
            }
            Near(Math.Abs(System.Linq.Enumerable.Sum(remapped.LiquidM3) - System.Linq.Enumerable.Sum(volumes)), 1e-15);
            Console.WriteLine($"S11 coupled volume and momentum remap passed at offset {offset:G} m.");
            var previousBody = coupled.Body;
            var currentPlan = nextPlan;
            var currentFields = remapped;
            var tentative = Array.ConvertAll(remap.Momentum.VelocityMPerS, value => (float)value);
            double initialWater = System.Linq.Enumerable.Sum(volumes);
            double initialHeat = System.Linq.Enumerable.Sum(thermal);
            for (int step = 2; step <= 3; step++)
            {
                var continuation = IcePressureCoupling.Advance(previousBody, currentPlan, dt, new D3(0, -g, 0),
                    boundary => FlowCpuReference.Project(currentPlan, tentative, dt, 16384, null, boundary),
                    point => rho * g * (0.10 - point.Y));
                var updatedGeometry = CutCellGeometry.Build(grid,
                    iceSurface: SurfacePrimitives.Transform(box, continuation.Body.Frame));
                var updatedPlan = new VariableCoefficientProjection(updatedGeometry, BoundaryConditions3D.Closed, _ => rho, _ => 0);
                var sweeps = MovingCutCellRemap.Reconcile(currentPlan, updatedPlan, continuation.Fluid, 0.005 * grid.CellVolume);
                var updated = MovingCutCellRemap.ApplyWithMomentum(currentPlan, updatedPlan, currentFields,
                    sweeps, continuation.Fluid.VelocityMPerS, rho, 1);
                Near(Math.Abs(System.Linq.Enumerable.Sum(updated.Fields.LiquidM3) - initialWater), 1e-15);
                Near(Math.Abs(System.Linq.Enumerable.Sum(updated.Fields.LiquidEnergyJ) - initialHeat), 1e-13);
                if (continuation.Body.Frame.Origin.Y <= previousBody.Frame.Origin.Y)
                    throw new InvalidOperationException("The submerged ice stopped rising during continuation.");
                previousBody = continuation.Body;
                currentPlan = updatedPlan;
                currentFields = updated.Fields;
                tentative = Array.ConvertAll(updated.Momentum.VelocityMPerS, value => (float)value);
            }
            Console.WriteLine(FormattableString.Invariant(
                $"S11 three-step continuation passed at offset {offset:G} m: rise {previousBody.Frame.Origin.Y - body.Frame.Origin.Y:G6} m."));
            var acceptedLoad = IcePressureLoad.FromProjection(body, plan, coupled.Fluid.GroupPressurePa,
                point => rho * g * (0.10 - point.Y));
            Near((coupled.Body.MomentumKgMPerS - (acceptedLoad.ForceN + new D3(0, -g * body.MassKg, 0)) * dt).Length, 1e-12);
            Near((coupled.Body.AngularMomentumKgM2PerS - acceptedLoad.TorqueNm * dt).Length, 1e-12);
            Console.WriteLine(FormattableString.Invariant(
                $"S11 coupled step at offset {offset:G} m: {coupled.Iterations} iterations, rise {coupled.Body.VelocityMPerS.Y:G6} m/s, mismatch {coupled.BoundaryVelocityMismatchMPerS:G6} m/s."));
            Console.WriteLine(FormattableString.Invariant(
                $"S11 ice pressure passed at grid offset {offset:G} m: lift {load.ForceN.Y:G8} N, applied torque {topLoad.TorqueNm.Z:G8} N m."));
        }
    }

    private static void CheckTwoPhaseMotion()
    {
        const double rhoWater = 1000, rhoAir = 1.2, g = 9.81, dt = 0.002;
        var grid = new Grid3D(8, 8, 8, new D3(0.01, 0.01, 0.01));
        var centre = new D3(0.038, 0.041, 0.039);
        var waterline = new D3(0, centre.Y, 0);
        var size = new D3(0.018, 0.017, 0.016);
        var box = SurfacePrimitives.Box(size * -0.5, size * 0.5);
        var body = IceBody.Box(917, size, RigidFrame.Euler(centre, 0.14, 0, 0));
        CutCellGeometry Geometry(IceBody pose) => CutCellGeometry.Build(grid,
            iceSurface: SurfacePrimitives.Transform(box, pose.Frame));
        var geometry = Geometry(body);
        var boundaries = BoundaryConditions3D.Closed;
        var topology = new VariableCoefficientProjection(geometry, boundaries, _ => rhoWater, _ => 0);
        var inventory = PlanarLiquidInventory.Create(topology, waterline, new D3(0, 1, 0), 42, 12, 0.01);
        double water = System.Linq.Enumerable.Sum(inventory.LiquidM3);
        Near(Math.Abs(water - (0.08 * 0.08 * centre.Y - 0.5 * size.X * size.Y * size.Z)), 1e-15);
        VariableCoefficientProjection WithPhaseDensity(VariableCoefficientProjection groups, CutCellInventories fields) =>
            new VariableCoefficientProjection(groups.Geometry, boundaries, cell =>
        {
            int group = groups.GroupByCell[cell];
            return group < 0 ? rhoAir :
                (rhoWater * fields.LiquidM3[group] + rhoAir * fields.GasM3[group]) / groups.Volumes[group];
        }, _ => 0);
        var plan = WithPhaseDensity(topology, inventory);
        var gravity = new D3(0, -g, 0);
        var initialColumn = new ColumnHydrostaticPressure(plan, inventory, gravity, rhoWater, rhoAir);
        int sampleCell = grid.Cells.Index(0, 1, 0);
        var sample = grid.Centre(0, 1, 0);
        double expectedPressure = g * (rhoWater * (waterline.Y - sample.Y) + rhoAir * (0.08 - waterline.Y));
        Near(Math.Abs(initialColumn.At(sampleCell, sample) - expectedPressure), 1e-8);
        var still = new float[plan.FaceCount];
        var moved = IcePressureCoupling.Advance(body, plan, dt, new D3(0, -g, 0),
            flux => FlowCpuReference.Project(plan, still, dt, 32768, null, flux),
            point => rhoWater * g * Math.Max(0, waterline.Y - point.Y));
        var updatedGeometry = Geometry(moved.Body);
        var updatedPlan = new VariableCoefficientProjection(updatedGeometry, boundaries, _ => rhoWater, _ => 0);
        var swept = MovingCutCellRemap.Reconcile(plan, updatedPlan, moved.Fluid, 0.005 * grid.CellVolume);
        var updated = MovingCutCellRemap.ApplyWithMomentum(plan, updatedPlan, inventory, swept,
            moved.Fluid.VelocityMPerS, rhoWater, rhoAir);
        for (int field = 0; field < inventory.Planes.Length; field++)
            Near(Math.Abs(System.Linq.Enumerable.Sum(updated.Fields.Planes[field]) -
                System.Linq.Enumerable.Sum(inventory.Planes[field])), 1e-13);
        var staticSurface = PlanarLiquidInventory.Create(updatedPlan, waterline, new D3(0, 1, 0), 42, 12, 0.01);
        double redistribution = 0;
        for (int group = 0; group < updatedPlan.GroupCount; group++)
            redistribution += Math.Abs(updated.Fields.LiquidM3[group] - staticSurface.LiquidM3[group]);
        if (moved.Body.VelocityMPerS.Y >= 0 || redistribution <= 1e-12)
            throw new InvalidOperationException("Two-phase ice step failed to sink or redistribute the water inventory.");
        var secondPlan = WithPhaseDensity(updatedPlan, updated.Fields);
        var column = new ColumnHydrostaticPressure(secondPlan, updated.Fields, gravity, rhoWater, rhoAir);
        double pressureChange = 0;
        for (int z = 0; z < grid.Cells.Z; z++)
        for (int x = 0; x < grid.Cells.X; x++)
        {
            int cell = grid.Cells.Index(x, 0, z);
            var point = grid.Centre(x, 0, z);
            pressureChange = Math.Max(pressureChange, Math.Abs(column.At(cell, point) - initialColumn.At(cell, point)));
        }
        var tentative = Array.ConvertAll(updated.Momentum.VelocityMPerS, value => (float)value);
        var second = IcePressureCoupling.Advance(moved.Body, secondPlan, dt, gravity,
            flux => FlowCpuReference.Project(secondPlan, tentative, dt, 32768, null, flux),
            columnPressure: column);
        if (pressureChange <= 1e-5 || second.Body.Frame.Origin.Y >= moved.Body.Frame.Origin.Y)
            throw new InvalidOperationException("Transported liquid did not update hydrostatic loading and ice motion.");
        var finalTopology = new VariableCoefficientProjection(Geometry(second.Body), boundaries, _ => rhoWater, _ => 0);
        var secondSweep = MovingCutCellRemap.Reconcile(secondPlan, finalTopology, second.Fluid, 0.005 * grid.CellVolume);
        var final = MovingCutCellRemap.ApplyWithMomentum(secondPlan, finalTopology, updated.Fields,
            secondSweep, second.Fluid.VelocityMPerS, rhoWater, rhoAir);
        for (int field = 0; field < inventory.Planes.Length; field++)
            Near(Math.Abs(System.Linq.Enumerable.Sum(final.Fields.Planes[field]) -
                System.Linq.Enumerable.Sum(inventory.Planes[field])), 1e-13);
        Console.WriteLine($"S11 updated water-column pressure passed: maximum change {pressureChange:G6} Pa, second-step speed {second.Body.VelocityMPerS.Y:G6} m/s.");
        Console.WriteLine(FormattableString.Invariant(
            $"S11 two-phase ice step passed: vertical speed {moved.Body.VelocityMPerS.Y:G6} m/s, water redistribution {redistribution:G6} m3."));
    }

    private static void CheckWaterline()
    {
        const double rho = 1000, g = 9.81, iceDensity = 917;
        var size = new D3(0.04, 0.03, 0.02);
        var centre = new D3(0.055, 0.054, 0.053);
        var body = IceBody.Box(iceDensity, size, RigidFrame.Euler(centre, 0.31, 0, 0));
        var surface = SurfacePrimitives.Transform(SurfacePrimitives.Box(size * -0.5, size * 0.5), body.Frame);
        var gravity = new D3(0, -g, 0);
        foreach (double offset in new[] { 0.0, 0.003 })
        {
            var grid = new Grid3D(12, 12, 12, new D3(0.01, 0.01, 0.01),
                RigidFrame.Euler(new D3(offset, 0.001 + offset, 0), 0.15, 0.1, 0));
            var geometry = CutCellGeometry.Build(grid, iceSurface: surface);
            foreach (double fraction in new[] { 0.0, 0.5, 0.917, 1.0 })
            {
                double waterline = centre.Y + (fraction - 0.5) * size.Y;
                var load = IcePressureLoad.Hydrostatic(body, geometry, new D3(0, waterline, 0), gravity, rho);
                double displaced = size.X * size.Y * size.Z * fraction;
                Near((load.ForceN - new D3(0, rho * g * displaced, 0)).Length, 1e-10);
                Near(load.TorqueNm.Length, 1e-11);
                if (fraction == 0.917)
                    Near((load.ForceN + gravity * body.MassKg).Length, 1e-10);
            }
            Console.WriteLine($"S11 V24 waterline traction passed at offset {offset:G} m.");
        }
    }

    private static void CheckRemap()
    {
        var grid = new Grid3D(6, 6, 6, new D3(0.01, 0.01, 0.01));
        var size = new D3(0.007, 0.006, 0.006);
        var surface = SurfacePrimitives.Box(size * -0.5, size * 0.5);
        VariableCoefficientProjection Pose(double x) => new VariableCoefficientProjection(
            CutCellGeometry.Build(grid, iceSurface: SurfacePrimitives.Transform(surface, RigidFrame.Euler(new D3(x, 0.024, 0.024), 0, 0, 0))),
            BoundaryConditions3D.Closed, _ => 1000, _ => 0);
        var before = Pose(0.024);
        var after = Pose(0.028);
        var fields = new double[5][];
        for (int field = 0; field < fields.Length; field++) fields[field] = new double[before.GroupCount];
        for (int group = 0; group < before.GroupCount; group++)
        {
            double volume = before.Volumes[group];
            double fraction = group % 2 == 0 ? 0.25 : 0.75;
            fields[0][group] = fraction * volume;
            fields[1][group] = (1 - fraction) * volume;
            fields[2][group] = 40 * fields[0][group];
            fields[3][group] = -12 * fields[1][group];
            fields[4][group] = 0.01 * fields[1][group];
        }
        var swept = new[] { new double[grid.XFaces.Count], new double[grid.YFaces.Count], new double[grid.ZFaces.Count] };
        // This fixture crosses a single face. Its exact fluid displacement is
        // the loss of fluid capacity in the cell on the high side.
        int high = grid.Cells.Index(3, 2, 2);
        int crossed = grid.XFaces.Index(3, 2, 2);
        swept[0][crossed] = after.Geometry.FluidVolumeM3[high] - before.Geometry.FluidVolumeM3[high];
        var initial = new CutCellInventories(fields);
        var moved = MovingCutCellRemap.Apply(before, after, initial, swept);
        for (int field = 0; field < fields.Length; field++)
            Near(Math.Abs(System.Linq.Enumerable.Sum(fields[field]) -
                System.Linq.Enumerable.Sum(moved.Planes[field])), 1e-12);
        for (int group = 0; group < after.GroupCount; group++)
            Near(Math.Abs(moved.LiquidM3[group] + moved.GasM3[group] - after.Volumes[group]), 1e-15);
        int donor = before.PressureGroup(3, 2, 2);
        int receiver = after.PressureGroup(2, 2, 2);
        int oldReceiver = before.PressureGroup(2, 2, 2);
        double expected = fields[0][oldReceiver] - swept[0][crossed] * fields[0][donor] / before.Volumes[donor];
        Near(Math.Abs(moved.LiquidM3[receiver] - expected), 1e-15);
        Console.WriteLine(FormattableString.Invariant(
            $"S11 moving-cell inventory passed: {Math.Abs(swept[0][crossed]) * 1e6:G6} mL transferred; phase volume, energy and vapour conserved."));
    }

    private static void Near(double error, double tolerance)
    {
        if (!SiUnits.IsFinite(error) || error > tolerance)
            throw new InvalidOperationException($"Pressure load error {error:G8} exceeded {tolerance:G8}.");
    }
}
