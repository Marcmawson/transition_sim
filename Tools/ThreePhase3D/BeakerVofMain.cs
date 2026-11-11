using System;
using System.Linq;
using PhysicsSim.ThreePhase3D.Flow;

internal static class BeakerVofMain
{
    public static int Main(string[] args)
    {
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
}
