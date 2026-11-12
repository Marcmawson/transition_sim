using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;
using PhysicsSim.ThreePhase3D.Geometry;
using PhysicsSim.ThreePhase3D.Solids;

internal static class IcePressureMain
{
    private static void Main()
    {
        const double g = 9.81, rho = 1000;
        var size = new D3(0.04, 0.03, 0.02);
        var centre = new D3(0.055, 0.054, 0.053);
        var box = SurfacePrimitives.Box(size * -0.5, size * 0.5);
        double volume = size.X * size.Y * size.Z;
        foreach (double offset in new[] { 0.0, 0.003 })
        {
            var body = IceBody.Box(917, size, RigidFrame.Euler(centre, 0, 0, 0.3));
            var gridFrame = RigidFrame.Euler(new D3(offset, 0, 0), 0.15, 0, 0);
            var grid = new Grid3D(12, 12, 12, new D3(0.01, 0.01, 0.01), gridFrame);
            var geometry = CutCellGeometry.Build(grid, iceSurface: SurfacePrimitives.Transform(box, body.Frame));
            var plan = new VariableCoefficientProjection(geometry, BoundaryConditions3D.Closed, _ => rho, _ => 0);
            var gauge = new double[plan.GroupCount];
            for (int i = 0; i < gauge.Length; i++) gauge[i] = 80;
            var load = IcePressureLoad.FromProjection(body, plan, gauge, point => rho * g * (0.10 - point.Y));
            Near((load.ForceN - new D3(0, rho * g * volume, 0)).Length, 1e-9);
            Near(load.TorqueNm.Length, 1e-10);
            const double dt = 0.01;
            var moved = load.Advance(body, dt, new D3(0, -g, 0));
            Near(Math.Abs(moved.VelocityMPerS.Y - dt * g * (rho / 917 - 1)), 1e-10);

            // A prescribed affine pressure on the top face generates a
            // known moment in body coordinates, independent of the cut grid.
            const double gradient = 1200;
            var topLoad = IcePressureLoad.Integrate(body, geometry, (patch, point) =>
            {
                var n = patch.NormalLocal;
                D3 worldNormal = gridFrame.U * n.X + gridFrame.V * n.Y + gridFrame.W * n.Z;
                return D3.Dot(worldNormal, body.Frame.V) > 0.99
                    ? 100 + gradient * body.Frame.ToLocal(point).X : 0;
            });
            D3 expectedTorque = body.Frame.W * (-gradient * size.Z * Math.Pow(size.X, 3) / 12);
            Near((topLoad.ForceN - body.Frame.V * (-100 * size.X * size.Z)).Length, 1e-9);
            Near((topLoad.TorqueNm - expectedTorque).Length, 1e-10);
            Console.WriteLine(FormattableString.Invariant(
                $"S11 ice pressure passed at grid offset {offset:G} m: lift {load.ForceN.Y:G8} N, applied torque {topLoad.TorqueNm.Z:G8} N m."));
        }
    }

    private static void Near(double error, double tolerance)
    {
        if (!SiUnits.IsFinite(error) || error > tolerance)
            throw new InvalidOperationException($"Pressure load error {error:G8} exceeded {tolerance:G8}.");
    }
}
