using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;
using PhysicsSim.ThreePhase3D.Geometry;
using PhysicsSim.ThreePhase3D.PhaseChange;
using PhysicsSim.ThreePhase3D.Solids;
using PhysicsSim.ThreePhase3D.Verification;

internal static class IcePressureMain
{
    private static void CheckMovingMelt()
    {
        var coarse = RunMovingMelt(3, 0.001);
        var refined = RunMovingMelt(6, 0.0005);
        Near(Math.Abs(coarse.HeatJ / refined.HeatJ - 1), 0.05);
        Near((coarse.Body.Frame.Origin - refined.Body.Frame.Origin).Length, 2e-6);
    }

    private static (IceBody Body, double HeatJ) RunMovingMelt(int steps, double substep, int cellsPerAxis = 8)
    {
        var result = IceMovingMeltCheck.Run(steps, substep, (plan, velocity, dt, flux) =>
            FlowCpuReference.Project(plan, velocity, dt, 32768, null, flux), cellsPerAxis);
        Console.WriteLine(FormattableString.Invariant(
            $"S11 V26 moving melt passed: {cellsPerAxis}³ cells, {steps} steps, heat {result.HeatJ:G8} J, ice loss {result.MassLossKg:G8} kg, centre displacement {result.DisplacementM:G8} m."));
        return (result.Body, result.HeatJ);
    }

    private static void CheckIceFaceConduction()
    {
        var equilibrium = new SolidLiquidInterface(101325, 250, 290);
        double rho = equilibrium.LiquidMeltingDensityKgM3;
        var size = new D3(0.022, 0.02, 0.02);
        var body = IceBody.Box(917, size, RigidFrame.Euler(new D3(0.035, 0.04, 0.04), 0, 0, 0));
        var box = SurfacePrimitives.Box(-size * 0.5, size * 0.5);
        var grid = new Grid3D(8, 8, 8, new D3(0.01, 0.01, 0.01), RigidFrame.Identity);
        var geometry = CutCellGeometry.Build(grid, iceSurface: SurfacePrimitives.Transform(box, body.Frame));
        var plan = new VariableCoefficientProjection(geometry, BoundaryConditions3D.Closed, _ => rho, _ => 0);
        var zero = new double[plan.GroupCount];
        var volume = (double[])plan.Volumes.Clone();
        var water = new PhysicsSim.ThreePhase3D.Materials.Water95();
        double warmK = equilibrium.MeltingTemperatureK + 10;
        double warmH = water.Evaluate(warmK, 101325, PhysicsSim.ThreePhase3D.Materials.WaterBranch.Liquid).EnthalpyJkg;
        var energy = Array.ConvertAll(volume, value => value * rho * warmH);
        var fields = new CutCellInventories(volume, zero, energy, zero, zero);
        var warm = Array.ConvertAll(volume, _ => warmK);
        const double conductivity = 0.6, dt = 0.01;
        var normal = new D3(1, 0, 0);
        var transfer = IceFaceConduction.Transfer(plan, fields, body, equilibrium, normal, dt,
            rho, conductivity, warm, patch => patch.FluidExposedAreaM2);
        double expected = conductivity * size.Y * size.Z * 10 * dt / 0.002;
        Near(Math.Abs(transfer.HeatJ - expected), 1e-12);
        double removed = 0;
        for (int group = 0; group < plan.GroupCount; group++)
            removed += energy[group] - transfer.Fields.LiquidEnergyJ[group];
        Near(Math.Abs(removed - transfer.HeatJ), 1e-12);
        var melted = IceBoxMelting.Apply(body, size, 0, 1, transfer.HeatJ, equilibrium);
        Near(Math.Abs(melted.Released.MassKg * equilibrium.LatentHeatJkg - expected), 1e-12);
        var dry = IceFaceConduction.Transfer(plan, fields, body, equilibrium, normal, dt,
            rho, conductivity, warm, _ => 0);
        Near(Math.Abs(dry.HeatJ), 0);
        var limited = IceFaceConduction.Transfer(plan, fields, body, equilibrium, normal, 1e6,
            rho, conductivity, warm, patch => patch.FluidExposedAreaM2);
        for (int group = 0; group < plan.GroupCount; group++)
            if (limited.Fields.LiquidEnergyJ[group] < volume[group] * rho * equilibrium.LiquidMeltingEnthalpyJkg - 1e-12)
                throw new InvalidOperationException("Conduction cooled a donor below melting equilibrium.");
        var liquid = new double[plan.GroupCount];
        var gas = (double[])volume.Clone();
        foreach (var patch in geometry.Patches)
        {
            if (patch.Role != SolidRole.Ice || patch.NormalLocal.X < 0.99 || patch.FluidExposedAreaM2 <= 0) continue;
            int group = plan.GroupByCell[patch.Cell];
            liquid[group] = volume[group];
            gas[group] = 0;
        }
        var mixedEnergy = Array.ConvertAll(liquid, value => value * rho * warmH);
        var mixed = new CutCellInventories(liquid, gas, mixedEnergy, zero, zero);
        var flow = FlowCpuReference.Project(plan, new float[plan.FaceCount], dt, 16384);
        var coupled = IceFaceMeltingStep.Advance(plan, mixed, flow, body, size, 0, 1, equilibrium,
            1.2, conductivity, warm, patch => patch.FluidExposedAreaM2,
            next => new VariableCoefficientProjection(next, BoundaryConditions3D.Closed, _ => rho, _ => 0),
            grid.CellVolume);
        double Sum(double[] values) => System.Linq.Enumerable.Sum(values);
        Near(Math.Abs(coupled.HeatJ - expected), 1e-12);
        Near(Math.Abs(coupled.Body.MassKg + (Sum(coupled.Fields.LiquidM3) - Sum(liquid)) * rho - body.MassKg), 1e-14);
        Near(Math.Abs(Sum(coupled.Fields.LiquidEnergyJ) - Sum(mixedEnergy) +
            (coupled.Body.MassKg - body.MassKg) * equilibrium.IceMeltingEnthalpyJkg), 1e-11);
        Near(Math.Abs(Sum(coupled.Fields.GasM3) * coupled.GasDensityKgM3 - Sum(gas) * 1.2), 1e-15);
        for (int group = 0; group < coupled.Topology.GroupCount; group++)
            Near(Math.Abs(coupled.Fields.LiquidM3[group] + coupled.Fields.GasM3[group] - coupled.Topology.Volumes[group]), 1e-16);
        Console.WriteLine(FormattableString.Invariant(
            $"S11 V26 coupled face melting passed: {coupled.HeatJ:G8} J transferred, {body.MassKg - coupled.Body.MassKg:G8} kg melted and remapped."));
    }

    private static void CheckMeltwaterSource()
    {
        var equilibrium = new SolidLiquidInterface(101325, 250, 290);
        var grid = new Grid3D(10, 10, 10, new D3(0.01, 0.01, 0.01),
            RigidFrame.Euler(new D3(0.005, 0.004, -0.003), 0.13, 0, 0));
        var size = new D3(0.04, 0.03, 0.02);
        var frame = RigidFrame.Euler(grid.Centre(4, 4, 4), 0.2, 0.3, 0.4);
        var box = IceBody.Box(917, size, frame);
        var ice = new IceBody(box.MassKg, box.PrincipalInertiaKgM2, frame,
            new D3(0.002, -0.003, 0.001), new D3(0.00001, 0.00002, -0.00003));
        var melt = IceBoxMelting.Apply(ice, size, 0, 1, ice.MassKg * equilibrium.LatentHeatJkg / 4, equilibrium);
        CutCellGeometry Geometry(D3 dimensions, RigidFrame pose) => CutCellGeometry.Build(grid,
            iceSurface: SurfacePrimitives.Transform(SurfacePrimitives.Box(dimensions * -0.5, dimensions * 0.5), pose));
        var before = Geometry(size, frame);
        var after = Geometry(melt.DimensionsM, melt.Remaining.Frame);
        double releasedVolume = melt.Released.MassKg / 917;
        var source = MeltwaterCutCellSource.Build(before, after, melt.Released,
            releasedVolume, melt.LiquidVolumeM3, melt.LiquidEnthalpyJ);
        double water = 0, heat = 0, contraction = 0;
        D3 momentum = default;
        int occupied = 0;
        for (int cell = 0; cell < source.Length; cell++)
        {
            var entry = source[cell];
            Near(Math.Abs(entry.LiquidM3 + entry.ContractionM3 -
                (after.FluidVolumeM3[cell] - before.FluidVolumeM3[cell])), 1e-17);
            if (entry.LiquidM3 > 0) occupied++;
            water += entry.LiquidM3;
            heat += entry.EnthalpyJ;
            contraction += entry.ContractionM3;
            momentum += entry.MomentumKgMPerS;
        }
        if (occupied < 2 || contraction <= 0) throw new InvalidOperationException("Expected multi-cell meltwater and contraction.");
        Near(Math.Abs(water - melt.LiquidVolumeM3), 1e-17);
        Near(Math.Abs(heat - melt.LiquidEnthalpyJ), 1e-10);
        Near(Math.Abs(contraction - (releasedVolume - water)), 1e-17);
        Near((momentum - melt.Released.MomentumKgMPerS).Length, 1e-14);
        double rhoLiquid = equilibrium.LiquidMeltingDensityKgM3;
        const double rhoGas = 1.2;
        var oldPlan = new VariableCoefficientProjection(before, BoundaryConditions3D.Closed, _ => rhoGas, _ => 0);
        var newPlan = new VariableCoefficientProjection(after, BoundaryConditions3D.Closed, _ => rhoGas, _ => 0);
        var zero = new double[oldPlan.GroupCount];
        var oldGas = (double[])oldPlan.Volumes.Clone();
        var oldHeat = Array.ConvertAll(oldGas, volume => volume * 12);
        var vapour = Array.ConvertAll(oldGas, volume => volume * 0.01);
        var fields = new CutCellInventories(zero, oldGas, zero, oldHeat, vapour);
        var flow = FlowCpuReference.Project(oldPlan, new float[oldPlan.FaceCount], 0.001, 16384);
        var mapped = MovingCutCellRemap.ApplyMeltingWithMomentum(oldPlan, newPlan, fields, flow, source,
            grid.CellVolume, rhoLiquid, rhoGas);
        double Sum(double[] values) => System.Linq.Enumerable.Sum(values);
        Near(Math.Abs(Sum(mapped.Fields.LiquidM3) * rhoLiquid - melt.Released.MassKg), 1e-14);
        Near(Math.Abs(Sum(mapped.Fields.LiquidEnergyJ) - heat), 1e-10);
        Near(Math.Abs(Sum(mapped.Fields.GasM3) * mapped.GasDensityKgM3 - Sum(oldGas) * rhoGas), 1e-15);
        Near(Math.Abs(Sum(mapped.Fields.GasEnergyJ) - Sum(oldHeat)), 1e-14);
        Near(Math.Abs(Sum(mapped.Fields.VapourKg) - Sum(vapour)), 1e-15);
        for (int group = 0; group < newPlan.GroupCount; group++)
            Near(Math.Abs(mapped.Fields.LiquidM3[group] + mapped.Fields.GasM3[group] - newPlan.Volumes[group]), 1e-16);
        var packed = new double[3];
        for (int face = 0; face < newPlan.FaceCount; face++)
            packed[newPlan.Faces[face].Axis] += mapped.Momentum.MomentumKgMPerS[face];
        D3 mappedMomentum = grid.Frame.U * packed[0] + grid.Frame.V * packed[1] + grid.Frame.W * packed[2];
        Near((mappedMomentum - momentum).Length, 1e-14);
        Console.WriteLine(FormattableString.Invariant(
            $"S11 V26 meltwater remap passed: {water * 1e6:F6} mL liquid, gas density {mapped.GasDensityKgM3:F8} kg/m3."));
        Console.WriteLine(FormattableString.Invariant(
            $"S11 V26 cut-cell meltwater source passed: {occupied} cells, {water * 1e6:F6} mL liquid, {contraction * 1e6:F6} mL contraction."));
    }

    private static void CheckMeltingTransfer()
    {
        var equilibrium = new SolidLiquidInterface(101325, 250, 290);
        var dimensions = new D3(0.04, 0.03, 0.02);
        var frame = RigidFrame.Euler(new D3(0.06, 0.08, 0.04), 0.2, 0.3, 0.4);
        var box = IceBody.Box(917, dimensions, frame);
        var body = new IceBody(box.MassKg, box.PrincipalInertiaKgM2, frame,
            new D3(0.002, -0.003, 0.001), new D3(0.00001, 0.00002, -0.00003));
        double totalHeat = body.MassKg * equilibrium.LatentHeatJkg;
        double heat = totalHeat / 4;
        for (int axis = 0; axis < 3; axis++)
        foreach (int side in new[] { -1, 1 })
        {
            var melt = IceBoxMelting.Apply(body, dimensions, axis, side, heat, equilibrium);
            var ice = melt.Remaining;
            var slab = melt.Released;
            Near(Math.Abs(ice.MassKg + slab.MassKg - body.MassKg), 1e-15);
            Near(Math.Abs(melt.DimensionsM[axis] - dimensions[axis] * 0.75), 1e-15);
            Near(Math.Abs(frame.ToLocal(ice.Frame.Origin)[axis] + side * dimensions[axis] / 8), 1e-15);
            Near((ice.MomentumKgMPerS + slab.MomentumKgMPerS - body.MomentumKgMPerS).Length, 1e-15);
            D3 AboutOriginalCentre(IceBody part) => part.AngularMomentumKgM2PerS +
                D3.Cross(part.Frame.Origin - frame.Origin, part.MomentumKgMPerS);
            Near((AboutOriginalCentre(ice) + AboutOriginalCentre(slab) - body.AngularMomentumKgM2PerS).Length, 1e-15);
            Near(Math.Abs(ice.KineticEnergyJ + slab.KineticEnergyJ - body.KineticEnergyJ), 1e-15);
            Near(Math.Abs(melt.LiquidVolumeM3 * equilibrium.LiquidMeltingDensityKgM3 - slab.MassKg), 1e-15);
            Near(Math.Abs(melt.LiquidEnthalpyJ - slab.MassKg * equilibrium.IceMeltingEnthalpyJkg - heat), 1e-10);
        }
        var dry = IceBoxMelting.Apply(body, dimensions, 0, 1, 0, equilibrium);
        var complete = IceBoxMelting.Apply(body, dimensions, 0, 1, totalHeat * 2, equilibrium);
        if (dry.Remaining != body || dry.Released != null || complete.Remaining != null || complete.Released != body)
            throw new InvalidOperationException("Melting transfer failed at an endpoint.");
        Near(Math.Abs(complete.UnusedHeatJ - totalHeat), 0);
        Console.WriteLine("S11 V26 face melting passed: all six faces conserved mass, enthalpy, momentum and kinetic energy.");
    }

    private static void CheckCoupledImpact()
    {
        var hit = IceImpactCheck.Run((plan, still, seconds, flux) =>
            FlowCpuReference.Project(plan, still, seconds, 16384, null, flux));
        Console.WriteLine(FormattableString.Invariant(
            $"S11 coupled beaker impact passed: {hit.ElapsedSeconds * 1000:F6} ms, wall impulse {hit.WallImpulseNs:G8} N s."));
    }

    private static void CheckPartialWallPressure()
    {
        const double edge = 0.003, pressure = 25, gradient = 500, speed = 0.02, spin = 2;
        var half = new D3(0.009, 0.0085, 0.008);
        var centre = new D3(0.038, 0.02 + half.Y, 0.039);
        var frame = RigidFrame.Euler(centre, 0, 0, 0);
        var box = SurfacePrimitives.Box(-half, half);
        var slab = SurfacePrimitives.Box(default, new D3(centre.X + edge, 0.02, 0.08));
        var grid = new Grid3D(8, 8, 8, new D3(0.01, 0.01, 0.01), RigidFrame.Euler(default, 0, 0, 0));
        var geometry = CutCellGeometry.Build(grid, slab, SurfacePrimitives.Transform(box, frame));
        if (!System.Linq.Enumerable.Any(geometry.Patches,
            patch => patch.Role == SolidRole.Ice && patch.ContactAreaM2 > 0 && patch.FluidExposedAreaM2 > 0))
            throw new InvalidOperationException("The slab must partially cover an ice patch.");

        var body = IceBody.Box(917, half * 2, frame);
        var load = IcePressureLoad.Integrate(body, geometry, (patch, point) =>
            patch.NormalLocal.Y < -0.99 ? pressure + gradient * (point.X - centre.X) : 0);
        double width = 2 * half.Z;
        double exposedArea = width * (half.X - edge);
        double exposedMoment = width * (half.X * half.X - edge * edge) / 2;
        double exposedSecondMoment = width * (Math.Pow(half.X, 3) - Math.Pow(edge, 3)) / 3;
        Near((load.ForceN - new D3(0, pressure * exposedArea + gradient * exposedMoment, 0)).Length, 1e-10);
        Near((load.TorqueNm - new D3(0, 0, pressure * exposedMoment + gradient * exposedSecondMoment)).Length, 1e-11);

        var moving = new IceBody(body.MassKg, body.PrincipalInertiaKgM2, frame,
            new D3(0, body.MassKg * speed, 0), new D3(0, 0, body.PrincipalInertiaKgM2.Z * spin));
        var plan = new VariableCoefficientProjection(geometry, BoundaryConditions3D.Closed, _ => 1000, _ => 0);
        var flux = IcePressureLoad.OutwardFluidFlux(moving, plan);
        double totalFlux = System.Linq.Enumerable.Sum(flux);
        double expectedFlux = -width * (speed * (edge + half.X) + spin * (edge * edge - half.X * half.X) / 2);
        Near(Math.Abs(totalFlux - expectedFlux), 1e-12);
        Console.WriteLine(FormattableString.Invariant(
            $"S11 partial wall pressure passed: force {load.ForceN.Y:G8} N, torque {load.TorqueNm.Z:G8} N m, flux {totalFlux:G8} m3/s."));
    }

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

    private static void Main(string[] args)
    {
        if (args.Length == 1 && args[0] == "--melt-grid")
        {
            foreach (int cells in new[] { 8, 10, 12 }) RunMovingMelt(3, 0.001, cells);
            return;
        }
        CheckMovingMelt();
        CheckIceFaceConduction();
        CheckMeltwaterSource();
        CheckMeltingTransfer();
        CheckCoupledSupport();
        CheckCoupledImpact();
        CheckPartialWallPressure();
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
