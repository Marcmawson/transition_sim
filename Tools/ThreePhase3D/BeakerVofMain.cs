using System;
using System.Linq;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;

internal static class BeakerVofMain
{
    public static int Main(string[] args)
    {
        if (args.Contains("--floor-heat")) return CheckFloorHeat();
        var beaker = new BeakerVofTimeIntegration();
        bool wetting = args.Contains("--wetting");
        var transport = wetting
            ? CutCellVofTransport.WithTrackedWetting(beaker.Plan, BeakerVofTimeIntegration.SyntheticWettingProfile())
            : new CutCellVofTransport(beaker.Plan);
        int totalPatches = 0;
        double totalImpulse = 0;
        var initial = (double[])beaker.State.LiquidM3.Clone();
        double moved = 0;
        for (int i = 0; i < 8; i++)
        {
            beaker.Step((state, flow) =>
            {
                var trial = wetting
                    ? transport.AdvanceCoupledWithWetting(state, flow,
                        BeakerVofTimeIntegration.LiquidDensity, BeakerVofTimeIntegration.GasDensity,
                        BeakerVofTimeIntegration.LiquidViscosityPaS, BeakerVofTimeIntegration.GasViscosityPaS, maxIterations: BeakerVofTimeIntegration.IterationLimit)
                    : transport.AdvanceCoupled(state, flow,
                        BeakerVofTimeIntegration.LiquidDensity, BeakerVofTimeIntegration.GasDensity,
                        maxIterations: BeakerVofTimeIntegration.IterationLimit);
                totalPatches += trial.Momentum.ContactTractionPatches;
                totalImpulse += trial.Momentum.WallImpulseKgMPerS.Sum(value => Math.Abs(value));
                return trial;
            });
            moved = initial.Zip(beaker.State.LiquidM3,(a,b) => Math.Abs(a-b)).Sum();
            Console.WriteLine(FormattableString.Invariant(
                $"step={beaker.Steps} time={beaker.TimeS:F3} iterations={beaker.Pressure.Diagnostics.Iterations} drift={beaker.LiquidDriftM3:E4} dualMassError={beaker.MaxDualMassErrorKg:E4} momentumError={beaker.MaxMomentumLedgerErrorKgMPerS:E4} redistribution={moved:E4}"));
        }
        if (wetting)
            Console.WriteLine(FormattableString.Invariant(
                $"contact patches={totalPatches} accumulated absolute wall impulse={totalImpulse:E4} kg m/s"));
        return beaker.Steps == 8 && moved > 1e-9 && (!wetting || (totalPatches > 0 && totalImpulse > 0)) ? 0 : 1;
    }

    private static int CheckFloorHeat()
    {
        const double liquidK = 293.15;
        double hot = Run(liquidK + 20);
        double cold = Run(liquidK - 20);
        double equal = Run(liquidK);
        return hot > 1e-5 && Math.Abs(hot + cold) < 1e-9 && Math.Abs(equal) < 1e-9 ? 0 : 1;

        double Run(double glassK)
        {
            // Seeded circulation starts with water in the first resolved cavity layer.
            var beaker = new BeakerVofTimeIntegration();
            beaker.EnableFloorHeating(liquidK, glassK);
            var transport = new CutCellVofTransport(beaker.Plan);
            beaker.Step((state, flow) => transport.AdvanceCoupled(state, flow,
                beaker.LiquidDensityKgM3, BeakerVofTimeIntegration.GasDensity,
                maxIterations: BeakerVofTimeIntegration.IterationLimit));
            Console.WriteLine(FormattableString.Invariant(
                $"Floor heat: glass {glassK:F2} K, liquid {liquidK:F2} K, heat to liquid {beaker.LiquidHeatJ:E8} J, residual {beaker.ThermalResidualJ:E8} J."));
            if (Math.Abs(beaker.ThermalResidualJ) > 1e-9)
                throw new InvalidOperationException("Glass and liquid heat balance did not close.");
            return beaker.LiquidHeatJ;
        }
    }

    public static int CheckGravity()
    {
        var beaker = new BeakerVofTimeIntegration();
        var plan = beaker.Plan;
        const double dt = .001;
        var rest = FlowCpuReference.Project(plan, new float[plan.FaceCount], dt, BeakerVofTimeIntegration.IterationLimit);
        var transport = new CutCellVofTransport(plan);
        var control = Advance(beaker.State, rest, default);
        var gravity = new D3(0, -9.81, 0);
        var first = Advance(beaker.State, rest, gravity);
        var second = Advance(first.Transport.Fields, first.Pressure, gravity);
        double peak = first.Pressure.VelocityMPerS.Max(value => Math.Abs(value));
        double redistribution = second.Transport.Fields.LiquidM3.Zip(beaker.State.LiquidM3, (a,b) => Math.Abs(a-b)).Sum();
        double drift = Math.Abs(second.Transport.Fields.LiquidM3.Sum() - beaker.InitialLiquidVolumeM3);
        Console.WriteLine(FormattableString.Invariant(
            $"Beaker gravity: peak={peak:E5} m/s redistribution={redistribution:E5} m^3 drift={drift:E5} m^3"));
        return control.Pressure.Diagnostics.Converged && first.Pressure.Diagnostics.Converged &&
            second.Pressure.Diagnostics.Converged && control.Pressure.VelocityMPerS.All(value => value == 0) &&
            peak > 1e-5 && redistribution > 1e-11 && drift <= 1e-12 ? 0 : 1;

        CutCellCoupledStep Advance(CutCellInventories state, FlowStep flow, D3 acceleration) =>
            transport.AdvanceCoupledWithAcceleration(state, flow, BeakerVofTimeIntegration.LiquidDensity,
                BeakerVofTimeIntegration.GasDensity, acceleration, maxIterations: BeakerVofTimeIntegration.IterationLimit);
    }

    public static int CheckRelease()
    {
        const double volume = 9e-6, temperature = 293.15, pressure = 101325;
        var release = BeakerVofTimeIntegration.CreateRelease(volume, false, temperature, pressure);
        var byMass = BeakerVofTimeIntegration.CreateRelease(release.InitialLiquidMassKg, true, temperature, pressure);
        var jittered = BeakerVofTimeIntegration.CreateRelease(volume, false, temperature, pressure, surfaceJitterM: .003);
        bool jitterPassed = Math.Abs(jittered.InitialLiquidVolumeM3 - volume) <= 1e-15 &&
            jittered.State.LiquidM3.Zip(release.State.LiquidM3, (a,b) => Math.Abs(a-b)).Sum() > 1e-9;
        var grid = release.Geometry.Grid;
        int headspace = release.Plan.GroupByCell[grid.Cells.Index(8, 10, 8)];
        int exterior = release.Plan.GroupByCell[grid.Cells.Index(0, 6, 8)];
        var initial = release.State.LiquidM3;
        bool initialized = Math.Abs(initial.Sum() - volume) <= 1e-15 &&
            initial.Zip(byMass.State.LiquidM3, (a,b) => Math.Abs(a-b)).Sum() <= 1e-15 &&
            initial[headspace] == 0 && release.State.GasM3[headspace] > 0 &&
            initial[exterior] == 0 && release.State.GasM3[exterior] > 0 &&
            release.Pressure.Diagnostics.Converged && release.Pressure.VelocityMPerS.All(value => value == 0);

        var transport = new CutCellVofTransport(release.Plan);
        for (int step = 0; step < 4; step++)
            release.Step((state, flow) => transport.AdvanceCoupledWithAcceleration(state, flow,
                release.LiquidDensityKgM3, BeakerVofTimeIntegration.GasDensity, new D3(0, -9.81, 0),
                maxIterations: BeakerVofTimeIntegration.IterationLimit));
        double moved = initial.Zip(release.State.LiquidM3, (a,b) => Math.Abs(a-b)).Sum();
        Console.WriteLine(FormattableString.Invariant(
            $"Beaker release: initialized={initialized} jitterPassed={jitterPassed} initial={release.InitialLiquidVolumeM3:E5} m^3 mass={release.InitialLiquidMassKg:E5} kg time={release.TimeS:F3} s redistribution={moved:E5} m^3 drift={release.LiquidDriftM3:E5} m^3"));
        return initialized && jitterPassed && moved > 1e-11 && Math.Abs(release.LiquidDriftM3) <= 1e-12 ? 0 : 1;
    }
}
