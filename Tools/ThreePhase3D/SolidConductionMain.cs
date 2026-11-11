using System;
using System.Collections.Generic;
using System.Linq;
using PhysicsSim.ThreePhase3D.Thermal;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Materials;

internal static class SolidConductionMain
{
    private static void Main()
    {
        double coarse = SlabError(16);
        double fine = SlabError(32);
        if (fine >= coarse / 3.5 || fine > 0.01)
            throw new Exception($"Unexpected slab errors: {coarse:G8}, {fine:G8} K.");
        Console.WriteLine($"PASS insulated slab: maximum errors {coarse:G6} and {fine:G6} K");
        CompositeContact();
        HeatedBase(.036, 9);
        HeatedBase(.063, 250);
        HeatedApparatus(.033, .06);
        HeatedApparatus(.06, .16);
        HeatedWater(.033, .06, 9);
        HeatedWater(.06, .16, 250);
        Cooldown(.033, .06, 9);
        Cooldown(.06, .16, 250);
    }

    private static double SlabError(int count)
    {
        // Insulated slab with T(x,t) = 300 + 20 cos(pi*x/L) exp(-alpha*pi^2*t/L^2).
        const double length = .01, conductivity = 1.2, volumetricCp = 1.8e6, duration = 20;
        double dx = length / count;
        double[] capacity = Enumerable.Repeat(volumetricCp * dx, count).ToArray();
        double[] initial = Enumerable.Range(0, count)
            .Select(i => 300 + 20 * Math.Cos(Math.PI * (i + .5) / count)).ToArray();
        var faces = Enumerable.Range(0, count - 1)
            .Select(i => new SolidConduction.Face(i, i + 1, conductivity / dx)).ToArray();
        var solver = new SolidConduction(capacity, initial, faces);
        double[] power = new double[count];
        int steps = (int)Math.Ceiling(duration / (.35 * solver.MaximumStepSeconds));
        for (int i = 0; i < steps; i++) solver.Step(duration / steps, power);
        double decay = Math.Exp(-conductivity / volumetricCp * Math.PI * Math.PI * duration / (length * length));
        Near(StoredJ(solver, capacity, 300), 0, 1e-8);
        return Enumerable.Range(0, count)
            .Max(i => Math.Abs(solver.TemperatureK[i] - (300 + (initial[i] - 300) * decay)));
    }

    private static void CompositeContact()
    {
        const double area = .01, hot = 330, cold = 290;
        const double left = .002 / 1.2, contact = .001, right = .003 / 16;
        double middleG = CoupledHeatFaces.HeatRateW(area, 2, 1, .002, 1.2, contact, .003, 16);
        var solver = new SolidConduction(new[] { 4.0, 7.0 }, new[] { 300.0, 300.0 },
            new[] { new SolidConduction.Face(0, 1, middleG) });
        double leftG = area / left, rightG = area / right;
        double expected = area * (hot - cold) / (2 * left + contact + 2 * right);
        double[] power = new double[2];
        for (int i = 0; i < 4000; i++)
        {
            power[0] = leftG * (hot - solver.TemperatureK[0]);
            power[1] = rightG * (cold - solver.TemperatureK[1]);
            solver.Step(.01, power);
        }
        Near(power[0], expected, 1e-9);
        Near(power[1], -expected, 1e-9);
        Console.WriteLine($"PASS composite contact: steady heat rate {expected:F6} W");
    }

    private static void HeatedBase(double radius, int waterMl)
    {
        const int radial = 16, axial = 6;
        const double thickness = .003, conductivity = 1.2, volumetricCp = 1.8e6;
        const double initial = 293.15, watts = 10, dt = .01;
        int support = radial * axial;
        var capacity = new double[support + 1];
        var faces = new List<SolidConduction.Face>();
        var power = new double[capacity.Length];
        double dr = radius / radial, dz = thickness / axial;
        var edges = Enumerable.Range(1, radial).Select(i => i * dr).ToArray();
        var burner = RadialHeatFluxBoundary.GaussianFacePowerW(watts, .015, edges);
        for (int r = 0; r < radial; r++)
        {
            double area = Math.PI * (2 * r + 1) * dr * dr;
            int bottom = r * axial;
            power[bottom] = burner[r];
            for (int z = 0; z < axial; z++)
            {
                int cell = bottom + z;
                capacity[cell] = volumetricCp * area * dz;
                if (z + 1 < axial) faces.Add(new SolidConduction.Face(cell, cell + 1, conductivity * area / dz));
                if (r + 1 < radial) faces.Add(new SolidConduction.Face(cell, cell + axial,
                    conductivity * 2 * Math.PI * edges[r] * dz / dr));
            }
        }
        // Synthetic support heat capacity and contact patch for this fixture.
        capacity[support] = 10;
        double supportG = CoupledHeatFaces.HeatRateW(1e-4, 2, 1, dz / 2, conductivity, .001, .003, 16);
        faces.Add(new SolidConduction.Face((radial - 1) * axial, support, supportG));
        var solver = new SolidConduction(capacity, Enumerable.Repeat(initial, capacity.Length).ToArray(), faces.ToArray());
        for (int i = 0; i < 300; i++) solver.Step(dt, power);
        Near(StoredJ(solver, capacity, initial), 30, 1e-8);
        if (!(solver.TemperatureK[0] > solver.TemperatureK[axial - 1] &&
            solver.TemperatureK[0] > solver.TemperatureK[(radial - 1) * axial] &&
            solver.TemperatureK[support] > initial))
            throw new Exception("Expected through-thickness and radial gradients and support warming.");
        double hotBase = solver.TemperatureK[0];
        Array.Clear(power, 0, power.Length);
        for (int i = 0; i < 300; i++) solver.Step(dt, power);
        Near(StoredJ(solver, capacity, initial), 30, 1e-8);
        if (solver.TemperatureK[0] >= hotBase) throw new Exception("Hot spot did not cool after switch-off.");
        Console.WriteLine($"PASS {waterMl} mL fixture base geometry: 30 J stored, radial/axial gradients, support warming");
    }

    private static void HeatedApparatus(double radius, double height)
    {
        const double initial = 293.15, watts = 10, seconds = 600;
        var geometry = new GeometryInputs { innerRadiusM = radius, cavityHeightM = height };
        geometry.tripodRingRadiusM = geometry.OuterRadiusM * .75;
        var mesh = new BeakerSolidMesh(geometry);
        Near(mesh.BurnerWeights.Sum(), 1, 1e-12);
        var solver = new SolidConduction(mesh.CapacityJPerK,
            Enumerable.Repeat(initial, mesh.CapacityJPerK.Length).ToArray(), mesh.Faces);
        var power = mesh.BurnerWeights.Select(value => value * watts).ToArray();
        int steps = (int)Math.Ceiling(seconds / Math.Min(.01, solver.MaximumStepSeconds * .5));
        for (int i = 0; i < steps; i++) solver.Step(seconds / steps, power);
        double stored = StoredJ(solver, mesh.CapacityJPerK, initial);
        Near(stored, watts * seconds, 1e-7);
        var t = solver.TemperatureK;
        if (!(t[mesh.LowerWall] > t[mesh.Rim] && t[mesh.Rim] > initial &&
            t[mesh.Ring] > t[mesh.LegTop] && t[mesh.LegTop] > t[mesh.Foot] && t[mesh.Foot] > initial))
            throw new Exception("Expected wall and tripod gradients.");
        int legCells = mesh.Foot - mesh.LegTop + 1;
        for (int leg = 1; leg < 3; leg++)
            Near(t[mesh.Foot], t[mesh.Foot + leg * legCells], 1e-10);
        Console.WriteLine($"PASS {radius * 2000:F0} mm apparatus: {stored:F6} J, wall and all three legs conduct");
    }

    private static void HeatedWater(double radius, double height, int waterMl)
    {
        const double initial = 293.15, duration = 30, watts = 10;
        var geometry = new GeometryInputs { innerRadiusM = radius, cavityHeightM = height };
        geometry.tripodRingRadiusM = geometry.OuterRadiusM * .75;
        double volume = waterMl * 1e-6;
        var mesh = new BeakerSolidMesh(geometry, volume);
        Near(mesh.WaterDepthM * Math.PI * radius * radius, volume, 1e-14);
        double waterCapacity = mesh.CapacityJPerK.Skip(mesh.WaterStart).Sum();
        var water = new Water95().Evaluate(initial, 101325, WaterBranch.Liquid);
        Near(waterCapacity, volume * water.DensityKgM3 * water.CpJkgK, 1e-9);
        Near(mesh.BurnerWeights.Skip(mesh.WaterStart).Sum(), 0, 1e-14);
        var solver = new SolidConduction(mesh.CapacityJPerK,
            Enumerable.Repeat(initial, mesh.CapacityJPerK.Length).ToArray(), mesh.Faces);
        var power = mesh.BurnerWeights.Select(value => value * watts).ToArray();
        int steps = (int)Math.Ceiling(duration / Math.Min(.01, solver.MaximumStepSeconds * .5));
        double dt = duration / steps;
        solver.Step(dt, power);
        Near(solver.TemperatureK[mesh.WaterStart], initial, 1e-12);
        for (int i = 1; i < steps; i++) solver.Step(dt, power);
        Near(StoredJ(solver, mesh.CapacityJPerK, initial), watts * duration, 1e-7);
        double waterJ = mesh.CapacityJPerK.Select((capacity, i) =>
            i >= mesh.WaterStart ? capacity * (solver.TemperatureK[i] - initial) : 0).Sum();
        if (!(waterJ > 0 && waterJ < watts * duration)) throw new Exception("Expected heat transfer into water.");
        Console.WriteLine($"PASS {waterMl} mL water, {mesh.WaterDepthM * 1000:F3} mm deep: {waterJ:F3} J in water, {waterJ / waterCapacity:F4} K mean rise");
    }

    private static void Cooldown(double radius, double height, int waterMl)
    {
        const double room = 293.15, film = 8, watts = 10, dt = .005;
        var geometry = new GeometryInputs { innerRadiusM = radius, cavityHeightM = height };
        geometry.tripodRingRadiusM = geometry.OuterRadiusM * .75;
        var mesh = new BeakerSolidMesh(geometry, waterMl * 1e-6);
        if (mesh.ExposedAreaM2.Any(area => area < 0))
            throw new Exception("Negative exposed area.");
        var solver = new SolidConduction(mesh.CapacityJPerK,
            Enumerable.Repeat(room, mesh.CapacityJPerK.Length).ToArray(), mesh.Faces);
        double lost = 0, before = 0;
        var power = new double[mesh.CapacityJPerK.Length];
        const int heatingSteps = 6000;
        for (int step = 0; step < 2 * heatingSteps; step++)
        {
            if (step == heatingSteps) before = StoredJ(solver, mesh.CapacityJPerK, room);
            for (int i = 0; i < power.Length; i++)
            {
                double roomPower = film * mesh.ExposedAreaM2[i] * (solver.TemperatureK[i] - room);
                lost += dt * roomPower;
                power[i] = (step < heatingSteps ? watts * mesh.BurnerWeights[i] : 0) - roomPower;
            }
            solver.Step(dt, power);
        }
        double after = StoredJ(solver, mesh.CapacityJPerK, room);
        if (!(after > 0 && after < before && lost > 0))
            throw new Exception("Expected the stored energy to fall after switch-off.");
        Near(after + lost, heatingSteps * dt * watts, 1e-7);
        Console.WriteLine($"PASS {waterMl} mL cooldown: stored {before:F3} to {after:F3} J, room gained {lost:F3} J");
    }

    private static double StoredJ(SolidConduction solver, double[] capacity, double referenceK)
    {
        return capacity.Select((value, i) => value * (solver.TemperatureK[i] - referenceK)).Sum();
    }

    private static void Near(double actual, double expected, double tolerance)
    {
        if (double.IsNaN(actual) || Math.Abs(actual - expected) > tolerance)
            throw new Exception($"Expected {expected:G17}, got {actual:G17}.");
    }
}
