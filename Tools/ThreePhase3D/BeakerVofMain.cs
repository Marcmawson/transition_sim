using System;
using System.Linq;
using PhysicsSim.ThreePhase3D.Flow;

internal static class BeakerVofMain
{
    public static int Main()
    {
        var beaker = new BeakerVofTimeIntegration();
        var transport = new CutCellVofTransport(beaker.Plan);
        var initial = (double[])beaker.State.LiquidM3.Clone();
        double moved = 0;
        for (int i = 0; i < 8; i++)
        {
            beaker.Step((state, flow) => transport.AdvanceCoupled(state, flow,
                BeakerVofTimeIntegration.LiquidDensity, BeakerVofTimeIntegration.GasDensity,
                maxIterations: BeakerVofTimeIntegration.IterationLimit));
            moved = initial.Zip(beaker.State.LiquidM3,(a,b) => Math.Abs(a-b)).Sum();
            Console.WriteLine(FormattableString.Invariant(
                $"step={beaker.Steps} time={beaker.TimeS:F3} iterations={beaker.Pressure.Diagnostics.Iterations} drift={beaker.LiquidDriftM3:E4} dualMassError={beaker.MaxDualMassErrorKg:E4} momentumError={beaker.MaxMomentumLedgerErrorKgMPerS:E4} redistribution={moved:E4}"));
        }
        return beaker.Steps == 8 && moved > 1e-9 ? 0 : 1;
    }
}
