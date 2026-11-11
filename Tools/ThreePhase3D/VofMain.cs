using System;
using System.Linq;
using PhysicsSim.ThreePhase3D.Verification;

internal static class VofMain
{
    public static int Main()
    {
        var suite = new VofChecks();
        try { suite.Run(); }
        finally
        {
            foreach (var row in suite.Rows)
                Console.WriteLine(FormattableString.Invariant(
                    $"{(row.Passed ? "PASS" : "FAIL")} {row.Case} | {row.Quantity} | {row.Actual:G10} | {row.Expected:G10} ± {row.Tolerance:G10}"));
            Console.WriteLine($"{suite.Rows.Count(row => row.Passed)}/{suite.Rows.Count} passed");
        }
        return suite.Passed ? 0 : 1;
    }
}
