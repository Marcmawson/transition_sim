using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Solids;

internal static class IceBodyMain
{
    private static void Main()
    {
        var body = IceBody.Box(917, new D3(0.04, 0.03, 0.02), RigidFrame.Identity);
        Near(body.MassKg, 917 * 0.04 * 0.03 * 0.02, 1e-15);
        double inertia = body.MassKg * (0.04 * 0.04 + 0.03 * 0.03) / 12;
        Near(body.PrincipalInertiaKgM2.Z, inertia, 1e-15);

        var forced = body.Advance(0.1, new D3(0, -9.81 * body.MassKg, 0), new D3(0, 0, inertia * 4));
        Near(forced.Frame.Origin.Y, -0.04905, 1e-12);
        Near(forced.VelocityMPerS.Y, -0.981, 1e-12);
        Near(forced.AngularVelocityRadPerS.Z, 0.4, 1e-12);
        Near(forced.Frame.U.Y, Math.Sin(0.02), 1e-12);
        Near(body.Frame.Origin.Length, 0, 0);

        var kicked = body.ApplyImpulse(new D3(0, 0.001, 0), new D3(0.01, 0, 0));
        Near(kicked.MomentumKgMPerS.Y, 0.001, 1e-15);
        Near(kicked.AngularMomentumKgM2PerS.Z, 1e-5, 1e-15);
        Near(kicked.VelocityAt(new D3(0.01, 0, 0)).Y,
            kicked.VelocityMPerS.Y + 0.01 * kicked.AngularVelocityRadPerS.Z, 1e-12);
        var drift = kicked;
        for (int i = 0; i < 1000; i++) drift = drift.Advance(0.001, default, default);
        Near((drift.MomentumKgMPerS - kicked.MomentumKgMPerS).Length, 0, 0);
        Near((drift.AngularMomentumKgM2PerS - kicked.AngularMomentumKgM2PerS).Length, 0, 0);
        Near(drift.KineticEnergyJ, kicked.KineticEnergyJ, 1e-12);
        Near(drift.Frame.U.Y, Math.Sin(kicked.AngularVelocityRadPerS.Z), 1e-12);
        var rotated = new IceBody(body.MassKg, body.PrincipalInertiaKgM2,
            RigidFrame.Euler(default, 0, 0, Math.PI / 2), default, new D3(0, 1e-5, 0));
        Near(rotated.AngularVelocityRadPerS.Y, 1e-5 / body.PrincipalInertiaKgM2.X, 1e-12);
        Console.WriteLine("S11 rigid ice body passed: box inertia, force, torque, off-centre impulse, point velocity and free motion.");
    }

    private static void Near(double actual, double expected, double tolerance)
    {
        if (!SiUnits.IsFinite(actual) || Math.Abs(actual - expected) > tolerance)
            throw new InvalidOperationException($"Expected {expected:G17}, got {actual:G17}.");
    }
}
