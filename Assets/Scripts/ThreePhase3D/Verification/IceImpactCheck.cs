using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;
using PhysicsSim.ThreePhase3D.Geometry;
using PhysicsSim.ThreePhase3D.Solids;
namespace PhysicsSim.ThreePhase3D.Verification
{
    public static class IceImpactCheck
    {
        public static (double ElapsedSeconds, double WallImpulseNs) Run(
            Func<VariableCoefficientProjection, float[], double, double[], FlowStep> project)
        {
            const double dt = 0.01, restitution = 0.5;
            var glass = new GeometryInputs { externalBaseHeightM = 0, baseThicknessM = 0.02 };
            var size = new D3(0.018, 0.017, 0.016);
            var centre = new D3(0.038, 0.02 + size.Y / 2 + 0.0002, 0.039);
            var box = SurfacePrimitives.Box(size * -0.5, size * 0.5);
            var initial = IceBody.Box(917, size, RigidFrame.Euler(centre, 0, 0, 0));
            var body = new IceBody(initial.MassKg, initial.PrincipalInertiaKgM2, initial.Frame,
                new D3(0, -0.04 * initial.MassKg, 0), default);
            var slab = SurfacePrimitives.Box(default, new D3(0.08, 0.02, 0.08));
            var grid = new Grid3D(8, 8, 8, new D3(0.01, 0.01, 0.01), RigidFrame.Identity);
            var geometry = CutCellGeometry.Build(grid, slab, SurfacePrimitives.Transform(box, body.Frame));
            var plan = new VariableCoefficientProjection(geometry, BoundaryConditions3D.Closed, _ => 1.2, _ => 0);
            var still = new float[plan.FaceCount];
            var hull = new D3[box.positionsM.Length / 3];
            for (int i = 0; i < hull.Length; i++)
                hull[i] = new D3(box.positionsM[3 * i], box.positionsM[3 * i + 1], box.positionsM[3 * i + 2]);
            var gravity = new D3(0, -9.81, 0);
            var beakerFrame = RigidFrame.Euler(new D3(0.04, 0, 0.04), 0, 0, 0);
            FlowStep Project(double seconds, double[] flux) =>
                project(plan, still, seconds, flux);
            var hit = IcePressureCoupling.AdvanceToBeakerImpact(body, plan, dt, gravity,
                Project, hull, glass, 24, beakerFrame, restitution);
            if (!hit.Hit || hit.ElapsedSeconds <= 0 || hit.ElapsedSeconds >= dt ||
                hit.Step.WallReactionNs.Y >= 0)
                throw new InvalidOperationException("The pressure-coupled ice did not rebound within the step.");
            Near(Math.Abs(hit.Step.Fluid.Diagnostics.Dt - hit.ElapsedSeconds), 1e-10);
            var load = IcePressureLoad.FromProjection(body, plan, hit.Step.Fluid.GroupPressurePa);
            if (load.ForceN.Length <= 1e-9) throw new InvalidOperationException("The coupled impact fixture needs a pressure load.");
            D3 incident = body.MomentumKgMPerS + (load.ForceN + gravity * body.MassKg) * hit.Step.Fluid.Diagnostics.Dt;
            Near((hit.Step.Body.MomentumKgMPerS + hit.Step.WallReactionNs - incident).Length, 1e-12);
            foreach (D3 vertex in hull)
            {
                D3 point = hit.Step.Body.Frame.ToWorld(vertex);
                if (point.Y < 0.02 - 1e-10)
                    throw new InvalidOperationException("The coupled impact penetrated the floor.");
                if (point.Y <= 0.02 + 1e-10 && hit.Step.Body.VelocityAt(point).Y <= 0)
                    throw new InvalidOperationException("The touching ice point did not rebound.");
            }
            var clear = IcePressureCoupling.AdvanceToBeakerImpact(body, plan, 0.001, gravity,
                Project, hull, glass, 24, beakerFrame, restitution);
            if (clear.Hit || clear.Step.WallReactionNs.Length != 0)
                throw new InvalidOperationException("The short coupled step should stay clear.");
            Near(Math.Abs(clear.ElapsedSeconds - 0.001), 0);
            return (hit.ElapsedSeconds, hit.Step.WallReactionNs.Y);
        }

        private static void Near(double error, double tolerance)
        {
            if (!(error <= tolerance))
                throw new InvalidOperationException($"Ice impact error {error:G6} exceeds {tolerance:G6}.");
        }
    }
}
