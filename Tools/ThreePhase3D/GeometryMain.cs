using System;
using PhysicsSim.ThreePhase3D.Verification;
public static class GeometryMain
{
    public static int Main(string[] args){try{GeometryChecks.Execute(args[0]);return 0;}catch(Exception e){Console.Error.WriteLine(e);return 1;}}
}
