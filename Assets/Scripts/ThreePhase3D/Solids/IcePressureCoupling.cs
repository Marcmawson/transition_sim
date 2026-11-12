using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;

namespace PhysicsSim.ThreePhase3D.Solids
{
    // Fixed-geometry pressure/body coupling. The supplied projection must
    // include the trial boundary flux in its RHS and convergence diagnostics.
    public static class IcePressureCoupling
    {
        public static Result Advance(IceBody body, VariableCoefficientProjection plan, double dt,
            D3 gravityMPerS2, Func<double[], FlowStep> project, Func<D3, double> backgroundPressurePa = null,
            ColumnHydrostaticPressure columnPressure = null)
        {
            if (body == null) throw new ArgumentNullException(nameof(body));
            if (plan == null) throw new ArgumentNullException(nameof(plan));
            if (project == null) throw new ArgumentNullException(nameof(project));
            if (!SiUnits.IsFinite(dt) || dt <= 0) throw new ArgumentOutOfRangeException(nameof(dt));

            const int iterationLimit = 80;
            const double relaxation = 0.35, velocityTolerance = 1e-8;
            double radius = Math.Sqrt(body.PrincipalInertiaKgM2.Length / body.MassKg);
            var trial = body;
            double mismatch = double.PositiveInfinity;
            for (int iteration = 1; iteration <= iterationLimit; iteration++)
            {
                var fluid = project(IcePressureLoad.OutwardFluidFlux(trial, plan));
                if (fluid == null || !fluid.Diagnostics.Converged || fluid.Diagnostics.Dt != dt ||
                    fluid.VelocityMPerS.Length != plan.FaceCount || fluid.GroupPressurePa.Length != plan.GroupCount)
                    throw new InvalidOperationException("Ice coupling requires a converged pressure step on the same grid and timestep.");

                var load = IcePressureLoad.FromProjection(body, plan, fluid.GroupPressurePa, backgroundPressurePa, columnPressure);
                D3 momentum = body.MomentumKgMPerS + (load.ForceN + gravityMPerS2 * body.MassKg) * dt;
                D3 angular = body.AngularMomentumKgM2PerS + load.TorqueNm * dt;
                var corrected = new IceBody(body.MassKg, body.PrincipalInertiaKgM2, body.Frame, momentum, angular);
                mismatch = Math.Max((corrected.VelocityMPerS - trial.VelocityMPerS).Length,
                    radius * (corrected.AngularVelocityRadPerS - trial.AngularVelocityRadPerS).Length);
                if (mismatch <= velocityTolerance)
                    return new Result(load.Advance(body, dt, gravityMPerS2), fluid, iteration, mismatch);

                trial = new IceBody(body.MassKg, body.PrincipalInertiaKgM2, body.Frame,
                    trial.MomentumKgMPerS + (momentum - trial.MomentumKgMPerS) * relaxation,
                    trial.AngularMomentumKgM2PerS + (angular - trial.AngularMomentumKgM2PerS) * relaxation);
            }
            throw new InvalidOperationException($"Ice pressure coupling failed after {iterationLimit} iterations. Boundary velocity mismatch {mismatch:G6} m/s.");
        }

        public readonly struct Result
        {
            public readonly IceBody Body;
            public readonly FlowStep Fluid;
            public readonly int Iterations;
            public readonly double BoundaryVelocityMismatchMPerS;

            internal Result(IceBody body, FlowStep fluid, int iterations, double mismatch)
            {
                Body = body;
                Fluid = fluid;
                Iterations = iterations;
                BoundaryVelocityMismatchMPerS = mismatch;
            }
        }
    }
}
