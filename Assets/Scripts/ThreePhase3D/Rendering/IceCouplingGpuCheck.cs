using System;
using System.Linq;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;
using PhysicsSim.ThreePhase3D.Geometry;
using PhysicsSim.ThreePhase3D.Solids;
using PhysicsSim.ThreePhase3D.Verification;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Rendering
{
    public static class IceCouplingGpuCheck
    {
        public static void Run(Func<string, float, bool> cancel)
        {
            var shader = Resources.Load<ComputeShader>("ThreePhase3D/SinglePhaseFlow");
            if (!SystemInfo.supportsComputeShaders || shader == null)
                throw new NotSupportedException("The S11 ice check requires the GPU pressure solver.");
            Check(shader, 0, 0.000257786, cancel);
            Check(shader, 0.003, 0.000260333, cancel);
        }

        public static void RunTwoPhase(Func<string, float, bool> cancel)
        {
            var shader = Resources.Load<ComputeShader>("ThreePhase3D/SinglePhaseFlow");
            if (!SystemInfo.supportsComputeShaders || shader == null)
                throw new NotSupportedException("The S11 two-phase check requires the GPU pressure solver.");

            FlowGpuSolver solver = null;
            VariableCoefficientProjection activePlan = null;
            double[] pressureSeed = null;
            int step = 0, trial = 0;
            try
            {
                var result = IceTwoPhaseCheck.Run((plan, tentative, dt, flux) =>
                {
                    if (activePlan != plan)
                    {
                        solver?.Dispose();
                        solver = new FlowGpuSolver(plan, null, shader);
                        activePlan = plan;
                        pressureSeed = null;
                        step++;
                        trial = 0;
                    }
                    if (cancel?.Invoke($"Two-phase step {step}/2, projection {++trial}", (step - 1) / 2f) == true)
                        throw new OperationCanceledException();
                    var projected = solver.Project(tentative, dt, 32768,
                        initialPressure: pressureSeed, boundaryFlux: flux);
                    pressureSeed = projected.GroupPressurePa;
                    return projected;
                });
                if (Math.Abs(result.FirstSpeed - (-0.00747123)) > 2e-6 ||
                    Math.Abs(result.SecondSpeed - (-0.0143223)) > 2e-6)
                    throw new InvalidOperationException($"S11 GPU two-phase speeds differ from the CPU reference: {result.FirstSpeed:G6}, {result.SecondSpeed:G6} m/s.");
                Debug.Log($"S11 GPU two-phase ice passed: first speed {result.FirstSpeed:G6} m/s, second speed {result.SecondSpeed:G6} m/s, pressure change {result.PressureChange:G6} Pa.");
            }
            finally
            {
                solver?.Dispose();
            }
        }

        public static void RunImpact(Func<string, float, bool> cancel)
        {
            var shader = Resources.Load<ComputeShader>("ThreePhase3D/SinglePhaseFlow");
            if (!SystemInfo.supportsComputeShaders || shader == null)
                throw new NotSupportedException("The S11 impact check requires the GPU pressure solver.");

            FlowGpuSolver solver = null;
            double[] pressureSeed = null;
            int trial = 0;
            try
            {
                var hit = IceImpactCheck.Run((plan, tentative, dt, flux) =>
                {
                    if (cancel?.Invoke($"Impact projection {++trial}, timestep {dt * 1000:F3} ms", 0) == true)
                        throw new OperationCanceledException();
                    solver ??= new FlowGpuSolver(plan, null, shader);
                    var projected = solver.Project(tentative, dt, 16384,
                        initialPressure: pressureSeed, boundaryFlux: flux);
                    pressureSeed = projected.GroupPressurePa;
                    return projected;
                });
                if (Math.Abs(hit.ElapsedSeconds - 0.002916205) > 1e-6 ||
                    Math.Abs(hit.WallImpulseNs - (-0.0001156161)) > 1e-7)
                    throw new InvalidOperationException($"S11 GPU impact differs from the CPU reference: {hit.ElapsedSeconds:G8} s, {hit.WallImpulseNs:G8} N s.");
                Debug.Log($"S11 GPU beaker impact passed: {hit.ElapsedSeconds * 1000:F6} ms, wall impulse {hit.WallImpulseNs:G8} N s.");
            }
            finally
            {
                solver?.Dispose();
            }
        }

        public static void RunMovingMelt(Func<string, float, bool> cancel)
        {
            var shader = Resources.Load<ComputeShader>("ThreePhase3D/SinglePhaseFlow");
            if (!SystemInfo.supportsComputeShaders || shader == null)
                throw new NotSupportedException("The S11 moving melt check requires the GPU pressure solver.");

            const int steps = 3;
            FlowGpuSolver solver = null;
            VariableCoefficientProjection activePlan = null;
            double[] pressureSeed = null;
            int stage = 0, trial = 0;
            try
            {
                var result = IceMovingMeltCheck.Run(steps, 0.001, (plan, velocity, dt, flux) =>
                {
                    if (activePlan != plan)
                    {
                        solver?.Dispose();
                        solver = new FlowGpuSolver(plan, null, shader);
                        activePlan = plan;
                        pressureSeed = null;
                        stage++;
                        trial = 0;
                    }
                    if (cancel?.Invoke($"Moving melt stage {stage}/{2 * steps}, projection {++trial}",
                        (stage - 1f) / (2 * steps)) == true)
                        throw new OperationCanceledException();
                    var flow = solver.Project(velocity, dt, 32768, initialPressure: pressureSeed, boundaryFlux: flux);
                    pressureSeed = flow.GroupPressurePa;
                    return flow;
                });
                if (Math.Abs(result.HeatJ / 0.0041447627 - 1) > 0.01 ||
                    Math.Abs(result.MassLossKg / 1.2430813e-8 - 1) > 0.01 ||
                    Math.Abs(result.DisplacementM - 2.6424854e-6) > 1e-6)
                    throw new InvalidOperationException($"S11 GPU moving melt differs from the CPU baseline: heat {result.HeatJ:G8} J, ice loss {result.MassLossKg:G8} kg, centre displacement {result.DisplacementM:G8} m.");
                Debug.Log($"S11 GPU moving melt passed: heat {result.HeatJ:G8} J, ice loss {result.MassLossKg:G8} kg, centre displacement {result.DisplacementM:G8} m.");
            }
            finally
            {
                solver?.Dispose();
            }
        }

        private static void Check(ComputeShader shader, double offset, double referenceRise,
            Func<string, float, bool> cancel)
        {
            const double density = 1000, gravity = 9.81, dt = 0.01;
            const int steps = 3;
            var grid = new Grid3D(12, 12, 12, new D3(0.01, 0.01, 0.01),
                RigidFrame.Euler(new D3(offset, 0, 0), 0.15, 0, 0));
            var size = new D3(0.04, 0.03, 0.02);
            var box = SurfacePrimitives.Box(size * -0.5, size * 0.5);
            var centre = new D3(0.055, 0.054, 0.053);
            var body = IceBody.Box(917, size, RigidFrame.Euler(centre, 0, 0, 0.3));
            var plan = BuildPlan(body);
            var zero = new double[plan.GroupCount];
            var fields = new CutCellInventories((double[])plan.Volumes.Clone(), zero,
                plan.Volumes.Select(volume => 42 * volume).ToArray(), zero, zero);
            var tentative = new float[plan.FaceCount];
            double initialWater = fields.LiquidM3.Sum(), initialHeat = fields.LiquidEnergyJ.Sum();
            double maxMismatch = 0;
            for (int step = 0; step < steps; step++)
            {
                using var solver = new FlowGpuSolver(plan, null, shader);
                double[] pressureSeed = null;
                int projection = 0;
                var accepted = IcePressureCoupling.Advance(body, plan, dt, new D3(0, -gravity, 0), flux =>
                {
                    if (cancel?.Invoke($"Offset {offset:G} m, step {step + 1}/{steps}, projection {++projection}",
                        (float)step / steps) == true)
                        throw new OperationCanceledException();
                    var flow = solver.Project(tentative, dt, 16384, initialPressure: pressureSeed, boundaryFlux: flux);
                    pressureSeed = flow.GroupPressurePa;
                    return flow;
                }, point => density * gravity * (0.10 - point.Y));

                var updated = BuildPlan(accepted.Body);
                var swept = MovingCutCellRemap.Reconcile(plan, updated, accepted.Fluid, 0.005 * grid.CellVolume);
                var remap = MovingCutCellRemap.ApplyWithMomentum(plan, updated, fields,
                    swept, accepted.Fluid.VelocityMPerS, density, 1);
                if (accepted.Body.Frame.Origin.Y <= body.Frame.Origin.Y ||
                    Math.Abs(remap.Fields.LiquidM3.Sum() - initialWater) > 1e-15 ||
                    Math.Abs(remap.Fields.LiquidEnergyJ.Sum() - initialHeat) > 1e-13)
                    throw new InvalidOperationException("The S11 GPU ice continuation failed its rise or conservation check.");

                body = accepted.Body;
                plan = updated;
                fields = remap.Fields;
                tentative = Array.ConvertAll(remap.Momentum.VelocityMPerS, value => (float)value);
                maxMismatch = Math.Max(maxMismatch, accepted.BoundaryVelocityMismatchMPerS);
            }

            double rise = body.Frame.Origin.Y - centre.Y;
            if (Math.Abs(rise - referenceRise) > 1e-6)
                throw new InvalidOperationException($"S11 GPU rise {rise:G6} m differs from CPU reference {referenceRise:G6} m.");
            Debug.Log($"S11 GPU submerged ice passed: offset {offset:G} m, {steps} steps, rise {rise * 1000:F6} mm, boundary mismatch {maxMismatch:G6} m/s.");

            VariableCoefficientProjection BuildPlan(IceBody pose) => new VariableCoefficientProjection(
                CutCellGeometry.Build(grid, iceSurface: SurfacePrimitives.Transform(box, pose.Frame)),
                BoundaryConditions3D.Closed, _ => density, _ => 0);
        }
    }
}
