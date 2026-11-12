using System;
using PhysicsSim.ThreePhase3D.Core;

namespace PhysicsSim.ThreePhase3D.Flow
{
    public sealed class FlowStep
    {
        public readonly float[] VelocityMPerS;
        public readonly double[] GroupPressurePa;
        public readonly FlowDiagnostics Diagnostics;

        internal FlowStep(float[] velocity, double[] pressure, FlowDiagnostics diagnostics)
        {
            VelocityMPerS = velocity;
            GroupPressurePa = pressure;
            Diagnostics = diagnostics;
        }
    }

    public sealed class FlowConvergenceException : InvalidOperationException
    {
        public readonly FlowDiagnostics Diagnostics;
        internal FlowConvergenceException(FlowDiagnostics diagnostics)
            : base($"Pressure projection failed after {diagnostics.Iterations} iterations at dt={diagnostics.Dt:G6} s. " +
                $"Residual={diagnostics.ResidualL2:E3}, RHS={diagnostics.RhsL2:E3} m³/s², " +
                $"dt × max divergence={diagnostics.Dt * diagnostics.MaxDivergencePerS:E3}.")
        {
            Diagnostics = diagnostics;
        }
    }

    // Double-precision pressure iteration for small verification cases. Both
    // backends return a new field only after acceptance, so a failed trial can
    // be retried with the original velocity and a different timestep.
    public static class FlowCpuReference
    {
        public static FlowStep Advance(MomentumPredictor3D predictor, float[] velocity,
            D3 acceleration, double requestedSeconds, int maxIterations = 4096)
        {
            double dt = predictor.SelectTimestep(velocity, acceleration, requestedSeconds);
            return Project(predictor.Plan, predictor.Predict(velocity, acceleration, dt), dt, maxIterations);
        }

        internal static FlowStep Project(VariableCoefficientProjection plan, float[] tentative, double dt, int maxIterations) =>
            Project(plan, tentative, dt, maxIterations, null);

        internal static FlowStep Project(VariableCoefficientProjection plan, float[] tentative, double dt, int maxIterations,
            Func<float[], bool> acceptVelocity, double[] boundaryFlux = null)
        {
            plan.ValidateBoundaryFlux(boundaryFlux);
            if (maxIterations <= 0) throw new ArgumentOutOfRangeException(nameof(maxIterations));
            var corrected = new float[plan.FaceCount];
            var pressure = new double[plan.GroupCount];
            var next = new double[pressure.Length];
            FlowDiagnostics diagnostics = default;
            for (int iteration = 0; ; iteration++)
            {
                if (iteration % 16 == 0 || iteration == maxIterations)
                {
                    for (int face = 0; face < corrected.Length; face++)
                    {
                        var metric = plan.Faces[face];
                        corrected[face] = metric.Active == 0 ? 0 : (float)(tentative[face] - dt * plan.Gradient(metric, pressure));
                    }
                    diagnostics = plan.Measure(tentative, corrected, pressure, dt, iteration, boundaryFlux);
                    if (diagnostics.Converged && Array.TrueForAll(corrected, value => SiUnits.IsFinite(value)) &&
                        (acceptVelocity == null || acceptVelocity(corrected)))
                    {
                        plan.SetClosedGauges(pressure);
                        return new FlowStep(corrected, pressure, diagnostics);
                    }
                    if (iteration == maxIterations) throw new FlowConvergenceException(diagnostics);
                }
                for (int group = 0; group < pressure.Length; group++)
                {
                    double residual = -(boundaryFlux?[group] ?? 0) / dt;
                    for (int entry = plan.Rows[group]; entry < plan.Rows[group + 1]; entry++)
                    {
                        int signed = plan.Incidence[entry], face = Math.Abs(signed) - 1;
                        var metric = plan.Faces[face];
                        residual -= Math.Sign(signed) * (double)metric.Area *
                            ((double)tentative[face] / dt - plan.Gradient(metric, pressure));
                    }
                    next[group] = plan.Diagonal[group] > 0 ? pressure[group] + (2.0 / 3) * residual / plan.Diagonal[group] : 0;
                }
                var swap = pressure;
                pressure = next;
                next = swap;
            }
        }
    }
}
