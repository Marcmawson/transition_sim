using System;
using System.IO;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Diagnostics;
using PhysicsSim.ThreePhase3D.Verification;

public static class CheckMain
{
    public static int Main(string[] args)
    {
        try
        {
            if (args.Length >= 3 && args[0] == "--verify")
            { EvidenceValidator.VerifyBundle(args[1], args[2]); Console.WriteLine("PASS bundle integrity and dependency freshness."); return 0; }
            if (args.Length < 2) throw new ArgumentException("Expected project root and bundle output directory.");
            var run = FoundationRun.Execute(args[0], args[1], typeof(CheckMain).Assembly.Location,
                "Standalone managed foundation runner; not a Unity player build", new RunHardware {
                    os = Environment.OSVersion.ToString(), gpu = "NOT_QUERIED_STANDALONE", graphics_api = "NOT_EXECUTED",
                    driver = "NOT_QUERIED_STANDALONE", gpu_executed = false }, "2022.3.46f1");
            foreach (var check in run.checks) Console.WriteLine((check.passed ? "PASS " : "FAIL ") + check.validationId + " " + check.name + (check.passed ? "" : "\n" + check.detail));
            Console.WriteLine("Run bundle: " + run.directory);
            Console.WriteLine("Source: " + run.gitCommit + "; dirty=" + run.dirty);
            if (args.Length > 2 && args[2].Length > 0) File.WriteAllBytes(args[2], RecordJson.Encode(run.checks));
            return run.Passed ? 0 : 1;
        }
        catch (Exception ex) { Console.Error.WriteLine(ex); return 1; }
    }
}
