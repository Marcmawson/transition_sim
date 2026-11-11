using System;
using PhysicsSim.ThreePhase3D.Flow;
using PhysicsSim.ThreePhase3D.Verification;

internal static class FlowMain
{
    public static int Main()
    {
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
}
