using System;
using System.Collections.Generic;
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
            => AdvanceCore(body, plan, dt, gravityMPerS2, project, backgroundPressurePa, columnPressure, null);

        public static Result AdvanceSupported(IceBody body, VariableCoefficientProjection plan, double dt,
            D3 gravityMPerS2, Func<double[], FlowStep> project, IReadOnlyList<D3> verticesLocalM,
            D3 planePointM, D3 planeNormal, Func<D3, double> backgroundPressurePa = null,
            ColumnHydrostaticPressure columnPressure = null)
            => AdvanceCore(body, plan, dt, gravityMPerS2, project, backgroundPressurePa, columnPressure,
                (D3 force, D3 torque, out D3 reaction) => IceWallContact.AdvanceSupported(
                    body, verticesLocalM, planePointM, planeNormal, dt, force, torque, out reaction));

        private delegate IceBody SupportedStep(D3 force, D3 torque, out D3 wallReactionNs);

        // Kick with the converged pressure load, then sweep with the incident momenta.
        // Each retry projects from the same fluid state for the revised duration.
        public static (Result Step, double ElapsedSeconds, bool Hit) AdvanceToBeakerImpact(
            IceBody body, VariableCoefficientProjection plan, double dt, D3 gravityMPerS2,
            Func<double, double[], FlowStep> project, IReadOnlyList<D3> verticesLocalM,
            GeometryInputs glass, int segments, RigidFrame beakerFrame, double restitution,
            Func<D3, double> backgroundPressurePa = null, ColumnHydrostaticPressure columnPressure = null)
        {
            if (project == null) throw new ArgumentNullException(nameof(project));
            double duration = dt;
            const int retryLimit = 32;
            for (int retry = 0; retry < retryLimit; retry++)
            {
                var step = AdvanceCore(body, plan, duration, gravityMPerS2, flux => project(duration, flux),
                    backgroundPressurePa, columnPressure, null, drift: false);
                var sweep = IceWallContact.AdvanceAndResolveBeakerImpact(step.Body, verticesLocalM,
                    glass, segments, beakerFrame, dt, restitution, out D3 reaction);
                if (sweep.Hit && sweep.ElapsedSeconds == 0)
                    throw new InvalidOperationException("Resolve existing beaker contact before advancing the fluid.");
                if (sweep.Hit && Math.Abs(duration - sweep.ElapsedSeconds) <= 1e-10)
                    return (new Result(sweep.Body, step.Fluid, step.Iterations,
                        step.BoundaryVelocityMismatchMPerS, reaction), sweep.ElapsedSeconds, true);

                if (!sweep.Hit && duration == dt)
                    return (new Result(sweep.Body, step.Fluid, step.Iterations,
                        step.BoundaryVelocityMismatchMPerS, default), dt, false);
                if (!sweep.Hit)
                {
                    // A shorter pressure kick can leave the hull clear. Accept only the projected interval.
                    var clear = IceWallContact.AdvanceAndResolveBeakerImpact(step.Body, verticesLocalM,
                        glass, segments, beakerFrame, duration, restitution, out _);
                    if (clear.Hit)
                        throw new InvalidOperationException("The shortened beaker sweep disagrees with the full sweep.");
                    return (new Result(clear.Body, step.Fluid, step.Iterations,
                        step.BoundaryVelocityMismatchMPerS, default), duration, false);
                }
                duration = sweep.ElapsedSeconds;
            }
            throw new InvalidOperationException("The pressure projection and beaker impact time did not converge.");
        }

        private static Result AdvanceCore(IceBody body, VariableCoefficientProjection plan, double dt,
            D3 gravityMPerS2, Func<double[], FlowStep> project, Func<D3, double> backgroundPressurePa,
            ColumnHydrostaticPressure columnPressure, SupportedStep support, bool drift = true)
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
                D3 force = load.ForceN + gravityMPerS2 * body.MassKg;
                D3 momentum = body.MomentumKgMPerS + force * dt;
                D3 angular = body.AngularMomentumKgM2PerS + load.TorqueNm * dt;
                D3 wallReaction = default;
                var supported = support?.Invoke(force, load.TorqueNm, out wallReaction);
                if (supported != null)
                {
                    momentum = supported.MomentumKgMPerS;
                    angular = supported.AngularMomentumKgM2PerS;
                }
                // Compare velocities in the frame used to build the cut cells.
                var corrected = new IceBody(body.MassKg, body.PrincipalInertiaKgM2, body.Frame, momentum, angular);
                mismatch = Math.Max((corrected.VelocityMPerS - trial.VelocityMPerS).Length,
                    radius * (corrected.AngularVelocityRadPerS - trial.AngularVelocityRadPerS).Length);
                if (mismatch <= velocityTolerance)
                    return new Result(supported ?? (drift ? load.Advance(body, dt, gravityMPerS2) : corrected),
                        fluid, iteration, mismatch, wallReaction);

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
            public readonly D3 WallReactionNs;

            internal Result(IceBody body, FlowStep fluid, int iterations, double mismatch, D3 wallReactionNs)
            {
                Body = body;
                Fluid = fluid;
                Iterations = iterations;
                BoundaryVelocityMismatchMPerS = mismatch;
                WallReactionNs = wallReactionNs;
            }
        }
    }
}
