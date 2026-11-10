using System;
using PhysicsSim.ThreePhase3D.Verification;
public static class MaterialMain
{
    public static int Main(string[] args)
    {try {MaterialChecks.Execute(args[0]);return 0;} catch(Exception e){Console.Error.WriteLine(e);return 1;}}
}
