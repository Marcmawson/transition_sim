using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;
using PhysicsSim.ThreePhase3D.Geometry;
using PhysicsSim.ThreePhase3D.Solids;
using PhysicsSim.ThreePhase3D.Verification;

internal static class IcePressureMain
{
    private static void CheckCoupledSupport()
    {
        const double dt = 0.001, pressurePa = 25;
        var size = new D3(0.018, 0.017, 0.016);
        var floor = new D3(0, 0.02, 0);
        var up = new D3(0, 1, 0);
        var gravity = up * -9.81;
        var centre = new D3(0.038, floor.Y + size.Y / 2, 0.039);
        var body = IceBody.Box(917, size, RigidFrame.Euler(centre, 0, 0, 0));
        var box = SurfacePrimitives.Box(size * -0.5, size * 0.5);
        var slab = SurfacePrimitives.Box(default, new D3(0.08, floor.Y, 0.08));
        var grid = new Grid3D(8, 8, 8, new D3(0.01, 0.01, 0.01), RigidFrame.Euler(default, 0, 0, 0));
        var geometry = CutCellGeometry.Build(grid, slab, SurfacePrimitives.Transform(box, body.Frame));
        var plan = new VariableCoefficientProjection(geometry, BoundaryConditions3D.Closed, _ => 1000, _ => 0);
        var still = new float[plan.FaceCount];
        var hull = new D3[box.positionsM.Length / 3];
        for (int i = 0; i < hull.Length; i++)
            hull[i] = new D3(box.positionsM[3 * i], box.positionsM[3 * i + 1], box.positionsM[3 * i + 2]);
        var accepted = IcePressureCoupling.AdvanceSupported(body, plan, dt, gravity, flux =>
        {
            foreach (double groupFlux in flux) Near(Math.Abs(groupFlux), 1e-12);
            return FlowCpuReference.Project(plan, still, dt, 16384, null, flux);
        }, hull, floor, up, _ => pressurePa);

        double expectedImpulse = (body.MassKg * -gravity.Y + pressurePa * size.X * size.Z) * dt;
        Near((accepted.WallReactionNs + up * expectedImpulse).Length, 1e-10);
        Near((accepted.Body.Frame.Origin - centre).Length, 1e-10);
        Near(accepted.Body.VelocityMPerS.Length, 1e-8);
        Near(accepted.Body.AngularVelocityRadPerS.Length, 1e-6);
        foreach (float velocity in accepted.Fluid.VelocityMPerS) Near(Math.Abs(velocity), 1e-8);
        Console.WriteLine(FormattableString.Invariant(
            $"S11 coupled resting ice passed: wall impulse {accepted.WallReactionNs.Y:G8} N s, mismatch {accepted.BoundaryVelocityMismatchMPerS:G6} m/s."));
    }

    private static void Main()
    {
        CheckCoupledSupport();
        CheckRemap();
        CheckWaterline();
        CheckFlotationTorque();
        CheckColumnPrediction();
        foreach (double offset in new[] { 0.0, 0.003 })
        {
            var twoPhase = IceTwoPhaseCheck.Run((plan, tentative, dt, flux) =>
                FlowCpuReference.Project(plan, tentative, dt, 32768, null, flux), steps: 512, gridOffsetM: offset);
            Console.WriteLine(FormattableString.Invariant(
                $"S11 CPU two-phase ice passed at offset {offset:G} m: final immersion {twoPhase.SubmergedFraction:P4}, final vertical speed {twoPhase.FinalSpeed:G6} m/s."));
            Console.WriteLine(FormattableString.Invariant(
                $"Last 32 steps: immersion {twoPhase.TailMinimumImmersion:P4} to {twoPhase.TailMaximumImmersion:P4}, peak vertical speed {twoPhase.TailPeakVerticalSpeed:G6} m/s, peak angular speed {twoPhase.TailPeakAngularSpeed:G6} rad/s."));
        }
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

    private static void CheckColumnPrediction()
    {
        const double rhoWater = 1000, rhoAir = 1.2, g = 9.81, dt = 0.002;
        var grid = new Grid3D(4, 4, 4, new D3(0.01, 0.01, 0.01));
        var plan = new VariableCoefficientProjection(CutCellGeometry.Build(grid), BoundaryConditions3D.Closed, _ => rhoWater, _ => 0);
        var surface = new D3(0, 0.023, 0);
        var gravity = new D3(0, -g, 0);
        var still = new float[plan.FaceCount];
        var flat = PlanarLiquidInventory.Create(plan, surface, new D3(0, 1, 0), 0, 0, 0);
        var flatPressure = new ColumnHydrostaticPressure(plan, flat, gravity, rhoWater, rhoAir);
        foreach (float value in flatPressure.Predict(still, dt)) Near(Math.Abs(value), 1e-10);

        var sloped = PlanarLiquidInventory.Create(plan, surface, new D3(-0.1, 1, 0), 0, 0, 0);
        var slopedPressure = new ColumnHydrostaticPressure(plan, sloped, gravity, rhoWater, rhoAir);
        int face = plan.FaceIndex(0, 2, 0, 1);
        double expected = -dt * g * (rhoWater - rhoAir) * 0.001 * plan.Faces[face].Mobility;
        Near(Math.Abs(slopedPressure.Predict(still, dt)[face] - expected), 1e-9);
        Console.WriteLine($"S11 column prediction passed: flat surface at rest, downslope face speed {expected:G6} m/s.");
    }

    private static void CheckFlotationTorque()
    {
        const double rho = 1000, g = 9.81, tilt = 0.01, fraction = 0.917;
        var size = new D3(0.018, 0.017, 0.016);
        var centre = new D3(0.038, 0.041, 0.039);
        var waterline = new D3(0, centre.Y + (fraction - 0.5) * size.Y, 0);
        var gravity = new D3(0, -g, 0);
        var surface = SurfacePrimitives.Box(size * -0.5, size * 0.5);
        double draft = fraction * size.Y;
        double metacentricHeight = size.X * size.X / (12 * draft) - (size.Y - draft) / 2;
        double restoringSlope = -rho * g * size.X * size.Z * draft * metacentricHeight;
        foreach (double offset in new[] { 0.0, 0.003 })
        foreach (double angle in new[] { -tilt, 0, tilt })
        {
            var grid = new Grid3D(8, 8, 8, new D3(0.01, 0.01, 0.01),
                RigidFrame.Euler(new D3(offset, 0, 0), 0, 0, 0));
            var body = IceBody.Box(917, size, RigidFrame.Euler(centre, 0, 0, angle));
            var geometry = CutCellGeometry.Build(grid, iceSurface: SurfacePrimitives.Transform(surface, body.Frame));
            var plan = new VariableCoefficientProjection(geometry, BoundaryConditions3D.Closed, _ => rho, _ => 0);
            var fields = PlanarLiquidInventory.Create(plan, waterline, new D3(0, 1, 0), 0, 0, 0);
            const double rhoAir = 1.2;
            var column = new ColumnHydrostaticPressure(plan, fields, gravity, rho, rhoAir);
            var reference = IcePressureLoad.Hydrostatic(body, geometry, waterline, gravity, rho - rhoAir);
            var reconstructed = IcePressureLoad.FromProjection(body, plan, new double[plan.GroupCount],
                columnPressure: column);
            double expected = restoringSlope * angle * (rho - rhoAir) / rho;
            Near(Math.Abs(reference.TorqueNm.Z - expected), 0.001 * Math.Abs(restoringSlope * tilt));
            double error = (reconstructed.TorqueNm - reference.TorqueNm).Length;
            Console.WriteLine(FormattableString.Invariant(
                $"V24 torque at offset {offset:G} m and roll {angle:G} rad: reference {reference.TorqueNm.Z:G6} N m, reconstructed {reconstructed.TorqueNm.Z:G6} N m, vector error {error:G6} N m."));
            Near(error, 1e-10);
        }
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
