using System;
using System.IO;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Verification;

public static class CheckMain
{
    public static int Main(string[] args)
    {
        var results = FoundationChecks.RunS00();
        foreach (var check in results) Console.WriteLine((check.passed ? "PASS " : "FAIL ") + check.validationId + " " + check.name + (check.passed ? "" : "\n" + check.detail));
        if (args.Length > 0) File.WriteAllBytes(args[0], RecordJson.Encode(results));
        return results.Exists(check => !check.passed) ? 1 : 0;
    }
}
