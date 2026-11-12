using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;
using PhysicsSim.ThreePhase3D.Geometry;
using PhysicsSim.ThreePhase3D.Materials;
using PhysicsSim.ThreePhase3D.Verification;

internal static class FlowMain
{
    public static int Main(string[] args)
    {
        if (args.Length == 1 && args[0] == "--humid-diffusion")
        {
            CheckHumidDiffusion();
            CheckReconstructedGasDiffusion();
            CheckOpenGasDiffusion();
            CheckHumidLowMach();
            HumidPhaseExchangeCheck.Run((plan, tentative, dt, source) =>
                FlowCpuReference.Project(plan, tentative, dt, 4096, null, source), Console.WriteLine);
            HumidPhaseExchangeCheck.RunSphericalFilm(Console.WriteLine);
            HumidPhaseExchangeCheck.RunTransientCondensation(Console.WriteLine);
            return 0;
        }
        var suite = new FlowChecks(FlowCpuReference.Advance);
        try { suite.Run(); }
        finally
        {
            int passed = 0;
            foreach (var row in suite.Rows)
            {
                Console.WriteLine(FormattableString.Invariant(
                    $"{(row.Passed ? "PASS" : "FAIL")} {row.Case} | {row.Quantity} | {row.Actual:G10} | {row.Expected:G10} ± {row.Tolerance:G10}"));
                if (row.Passed) passed++;
            }
            Console.WriteLine($"{passed}/{suite.Rows.Count} passed");
        }
        return suite.Passed ? 0 : 1;
    }

    private static void CheckHumidLowMach()
    {
        const double pressure = 101325, gasMass = 0.001, dt = 0.01;
        var model = new HumidGas();
        var ambient = model.Evaluate(293.15, pressure, 0.005);
        var warmer = model.Evaluate(298.15, pressure, 0.005);
        var wetter = model.Evaluate(293.15, pressure, 0.01);
        var gravity = new D3(0, -9.81, 0);
        double thermal = HumidGasLowMach.BuoyancyMPerS2(warmer, ambient, gravity).Y;
        double species = HumidGasLowMach.BuoyancyMPerS2(wetter, ambient, gravity).Y;
        double expansion = HumidGasLowMach.ProjectionSourceM3PerS(ambient, warmer, gasMass, dt);
        double recoveredVolume = gasMass / ambient.DensityKgM3 - dt * expansion;
        if (!(thermal > 0 && species > 0 && expansion < 0) ||
            Math.Abs(recoveredVolume - gasMass / warmer.DensityKgM3) > 1e-15 ||
            HumidGasLowMach.BuoyancyMPerS2(ambient, ambient, gravity).Length != 0 ||
            HumidGasLowMach.ProjectionSourceM3PerS(ambient, ambient, gasMass, dt) != 0)
            throw new InvalidOperationException("Humid low Mach source or buoyancy failed its limiting checks.");
        Console.WriteLine(FormattableString.Invariant(
            $"S12 humid low Mach passed: thermal buoyancy {thermal:G8} m/s2, species buoyancy {species:G8} m/s2, projection source {expansion:G8} m3/s."));
    }

    private static void CheckReconstructedGasDiffusion()
    {
        CheckReconstructedGasDiffusion(null, 0.01);
        CheckReconstructedGasDiffusion(SurfacePrimitives.Box(new D3(0, 0, 0.008),
            new D3(0.02, 0.02, 0.01)), 0.008);
    }

    private static void CheckReconstructedGasDiffusion(TriangleSurface solid, double openWidth)
    {
        const double dx = 0.01, rho = 1.2, dt = 0.1, diffusivity = 2.5e-5;
        var grid = new Grid3D(2, 2, 1, new D3(dx, dx, dx), RigidFrame.Identity);
        var plan = new VariableCoefficientProjection(CutCellGeometry.Build(grid, solid), BoundaryConditions3D.Closed,
            _ => rho, _ => 0);
        var fields = PlanarLiquidInventory.Create(plan, new D3(0, 0.015, 0), new D3(0, 1, 0), 0, 10, 0);
        var gasMass = Array.ConvertAll(fields.GasM3, volume => volume * rho);
        int left = plan.GroupByCell[grid.Cells.Index(0, 1, 0)];
        int right = plan.GroupByCell[grid.Cells.Index(1, 1, 0)];
        fields.VapourKg[left] = gasMass[left] * 0.02;
        fields.VapourKg[right] = gasMass[right] * 0.01;
        var areas = new CutCellVofTransport(plan).ReconstructGasFaceAreas(fields);
        var interfaceAreas = new CutCellVofTransport(plan).ReconstructLiquidGasInterfaceAreas(fields);
        if (Math.Abs(interfaceAreas[left] + interfaceAreas[right] - 2 * dx * openWidth) > 1e-12)
            throw new InvalidOperationException("The reconstructed interface area does not match the planar section.");
        double area = areas[plan.FaceIndex(0, 1, 1, 0)];
        if (Math.Abs(area - 0.5 * dx * openWidth) > 1e-12 || areas[plan.FaceIndex(0, 1, 0, 0)] != 0)
            throw new InvalidOperationException("The reconstructed aperture does not match the half-wetted face.");
        var vapourH = Array.ConvertAll(gasMass, _ => 2.5e6);
        var airH = new double[gasMass.Length];
        var result = HumidGasDiffusion.AdvanceReconstructed(plan, fields, dt, gasMass, vapourH, airH,
            _ => diffusivity);
        double expected = dt * rho * diffusivity * area / dx * 0.01;
        if (Math.Abs(fields.VapourKg[left] - result.VapourKg[left] - expected) > 1e-16 ||
            Math.Abs(result.VapourKg[right] - fields.VapourKg[right] - expected) > 1e-16)
            throw new InvalidOperationException("Vapour transfer did not use the reconstructed gas aperture.");
        Console.WriteLine(FormattableString.Invariant(
            $"S12 reconstructed gas diffusion passed: aperture {area:G8} m2, vapour transferred {expected:G8} kg."));
    }

    private static void CheckOpenGasDiffusion()
    {
        const double dx = 0.01, rho = 1.2, dt = 0.1, diffusivity = 2.5e-5;
        var grid = new Grid3D(1, 1, 1, new D3(dx, dx, dx), RigidFrame.Identity);
        double gasMass = rho * grid.CellVolume;
        foreach (bool positive in new[] { false, true })
        {
            var sides = new FlowBoundary[6];
            sides[positive ? 1 : 0] = new FlowBoundary(FlowBoundaryKind.OpenPressure);
            var plan = new VariableCoefficientProjection(CutCellGeometry.Build(grid), new BoundaryConditions3D(sides),
                _ => rho, _ => 0);
            var zero = new double[1];
            var fields = new CutCellInventories(zero, (double[])plan.Volumes.Clone(), zero,
                new[] { 1.0 }, new[] { 0.01 * gasMass });
            double boundaryFraction = positive ? 0.005 : 0.02;
            var update = HumidGasDiffusion.AdvanceOpen(plan, fields, dt, new[] { gasMass },
                new[] { 2.5e6 }, new[] { 20000.0 }, _ => diffusivity,
                _ => new HumidGasDiffusion.Reservoir(rho, boundaryFraction, 2.6e6, 30000));
            double expected = dt * rho * diffusivity * dx * 2 * (boundaryFraction - 0.01);
            double expectedHeat = expected * (expected > 0 ? 2.6e6 - 20000 : 2.5e6 - 30000);
            if (Math.Abs(update.VapourInKg - expected) > 1e-16 ||
                Math.Abs(update.EnthalpyInJ - expectedHeat) > 1e-9 ||
                Math.Abs(update.Fields.VapourKg[0] - fields.VapourKg[0] - update.VapourInKg) > 1e-16 ||
                Math.Abs(update.Fields.GasEnergyJ[0] - fields.GasEnergyJ[0] - update.EnthalpyInJ) > 1e-14)
                throw new InvalidOperationException("Open gas diffusion disagrees with the reservoir balance.");
            Console.WriteLine(FormattableString.Invariant(
                $"S12 open gas diffusion passed: vapour in {update.VapourInKg:G8} kg, enthalpy in {update.EnthalpyInJ:G8} J."));
        }
    }


    private static void CheckHumidDiffusion()
    {
        const double spacing = 0.01, rho = 1.2, dt = 0.2;
        var grid = new Grid3D(2, 1, 1, new D3(spacing, spacing, spacing), RigidFrame.Identity);
        var plan = new VariableCoefficientProjection(CutCellGeometry.Build(grid), BoundaryConditions3D.Closed, _ => rho, _ => 0);
        double mass = rho * grid.CellVolume;
        var zero = new double[2];
        var vapour = new[] { 0.02 * mass, 0.005 * mass };
        var energy = new[] { 2.0, 3.0 };
        var state = new CutCellInventories(zero, (double[])plan.Volumes.Clone(), zero, energy, vapour);
        var model = new HumidGas();
        double diffusivity = model.BinaryDiffusivityM2s(293.15, 101325, 0.0125);
        var vapourH = new[] { 2.5e6, 2.6e6 };
        var airH = new[] { 20000.0, 30000.0 };
        double Area(int index)
        {
            var face = plan.Faces[index];
            return face.Low >= 0 && face.High >= 0 ? face.Area : 0;
        }
        var next = HumidGasDiffusion.Advance(plan, state, dt, new[] { mass, mass }, vapourH, airH, Area, _ => diffusivity);
        double expected = dt * rho * diffusivity * spacing * (0.02 - 0.005);
        Near(next.VapourKg[0], vapour[0] - expected, 1e-15);
        Near(next.VapourKg[1], vapour[1] + expected, 1e-15);
        Near(next.GasEnergyJ[0], energy[0] - expected * (vapourH[0] - airH[1]), 1e-8);
        Near(next.GasEnergyJ[0] + next.GasEnergyJ[1], 5, 1e-14);
        try
        {
            HumidGasDiffusion.Advance(plan, state, 1000, new[] { mass, mass }, vapourH, airH, Area, _ => diffusivity);
        }
        catch (InvalidOperationException)
        {
            Console.WriteLine(FormattableString.Invariant(
                $"S12 humid diffusion passed: {vapour[0] - next.VapourKg[0]:G8} kg vapour transferred; equal dry-air counterflux, enthalpy conserved."));
            return;
        }
        throw new InvalidOperationException("The gas diffusion timestep guard did not reject an unstable step.");

        void Near(double value, double reference, double tolerance)
        {
            if (!(Math.Abs(value - reference) <= tolerance))
                throw new InvalidOperationException($"Humid diffusion expected {reference:G8}, got {value:G8}.");
        }
    }
}
