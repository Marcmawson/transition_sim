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
            CheckHumidLowMach();
            CheckHumidInterface();
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

    private static void CheckHumidInterface()
    {
        const double pressure = 101325, film = 0.01;
        var model = new HumidGas();
        var warm = model.EquilibriumWithLiquid(293.15, pressure);
        var cool = model.EquilibriumWithLiquid(283.15, pressure);
        double bulk = (warm.WaterMassFraction + cool.WaterMassFraction) * 0.5;
        double warmD = HumidGas.ReferenceDiffusivity(warm.TemperatureK, pressure);
        double coolD = HumidGas.ReferenceDiffusivity(cool.TemperatureK, pressure);
        double evaporation = HumidInterfaceFlux.WaterFluxKgM2PerS(warm, bulk, film, warmD);
        double condensation = HumidInterfaceFlux.WaterFluxKgM2PerS(cool, bulk, film, coolD);
        double equilibrium = HumidInterfaceFlux.WaterFluxKgM2PerS(warm, warm.WaterMassFraction, film, warmD);
        double velocityJump = HumidInterfaceFlux.NormalVelocityJumpMPerS(warm, evaporation);
        if (!(evaporation > 0 && condensation < 0 && velocityJump > 0) || equilibrium != 0 ||
            Math.Abs(2 * HumidInterfaceFlux.WaterFluxKgM2PerS(warm, bulk, 2 * film, warmD) - evaporation) > 1e-15)
            throw new InvalidOperationException("Humid interface failed equilibrium, flux direction or film thickness scaling.");
        var cooling = HumidInterfaceFlux.SolveTemperature(pressure, bulk, film,
            warm.TemperatureK, warm.TemperatureK, 100, 10, 283.15, 298.15);
        var heating = HumidInterfaceFlux.SolveTemperature(pressure, bulk, film,
            cool.TemperatureK, cool.TemperatureK, 100, 10, 278.15, 293.15);
        if (!(cooling.Surface.TemperatureK < warm.TemperatureK && cooling.MassFluxKgM2PerS > 0 &&
            heating.Surface.TemperatureK > cool.TemperatureK && heating.MassFluxKgM2PerS < 0) ||
            Math.Abs(cooling.LiquidHeatWm2 + cooling.GasHeatWm2 - cooling.MassFluxKgM2PerS * cooling.LatentHeatJkg) > 1e-7 ||
            Math.Abs(heating.LiquidHeatWm2 + heating.GasHeatWm2 - heating.MassFluxKgM2PerS * heating.LatentHeatJkg) > 1e-7)
            throw new InvalidOperationException("Latent heat failed to cool evaporation or warm condensation.");
        foreach (var solved in new[] { cooling, heating })
        {
            const double area = 0.001, dt = 0.1;
            var transfer = HumidInterfaceFlux.InventoryTransfer(solved, area, dt, 0.001, 0.001);
            if (Math.Abs(transfer.LiquidEnthalpyJ + transfer.GasEnthalpyJ) > 1e-11 ||
                Math.Abs(transfer.LiquidVolumeM3 * solved.Surface.Liquid.DensityKgM3 + transfer.WaterKg) > 1e-18 ||
                Math.Abs(transfer.GasVolumeM3 + transfer.LiquidVolumeM3 -
                    HumidInterfaceFlux.NormalVelocityJumpMPerS(solved.Surface, solved.MassFluxKgM2PerS) * area * dt) > 1e-18)
                throw new InvalidOperationException("Humid interface inventory transfer did not close.");
        }
        Console.WriteLine(FormattableString.Invariant(
            $"S12 humid interface passed: evaporation {evaporation:G8}, condensation {condensation:G8} kg/m2/s, Stefan jump {velocityJump:G8} m/s."));
        Console.WriteLine(FormattableString.Invariant(
            $"S12 humid heat balance passed: evaporating interface {cooling.Surface.TemperatureK:G9} K, condensing interface {heating.Surface.TemperatureK:G9} K."));
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
